"""
MicroPython DWA (Dynamic Window Approach) 局部路径规划器
========================================================

算法概述：
---------
DWA（动态窗口法）是一种实时避障导航算法，核心思想是：
  1. 根据当前速度和加/减速限制，确定下一个控制周期可达的"速度窗口"
  2. 在窗口内采样多组候选速度 (v, ω)
  3. 模拟每条速度对应的未来轨迹
  4. 用三个指标评分：朝向终点角度、与障碍物距离、速度大小
  5. 选评分最高的 (v, ω) 发给执行器

算法每 ~100ms 运行一次，边走边看，不需要提前知道所有障碍物位置。

通信协议（下位机 → 上位机 / 状态帧）：
    @O#x#y#v#w#theta!   推荐格式
    @S#x#y#v#w#theta!   兼容格式
    @P#x#y#v#w#theta!   兼容格式

各字段单位需保持一致：
    x, y, 障碍物坐标, 目标点: 统一用 cm 或 m
    v (线速度): 距离单位 / 秒
    w, theta (角速度, 朝向): 弧度/秒 和 弧度
    theta: 逆时针为正; 默认 0 指向世界坐标系 Y+ 方向

输出指令帧（上位机 → 下位机）：
    @VW#v#w!           用于差速/全向底盘的线速度+角速度控制
    @VW#vx#vy#w!       用于需要分量形式的底盘（VW3 模式）
    @V#vx#vy!          仅线速度分量（VXY 兼容模式，无角速度）
"""

import math
import time

from machine import UART  


# ---- 默认串口配置 ----
UART_PORT = 2      # 使用的 UART 端口号
UART_BAUD = 9600    # 波特率


# ----------------------------- Config ---------------------------------

class DWAConfig:
    """
    DWA 算法全部可调参数
    ---------------------
    所有速度和距离单位需与下位机保持一致（cm 或 m 二选一）。
    """
    def __init__(self):
        # ---- 速度边界（需与下位机控制器的单位匹配）----
        self.min_v = 0.0       # 最小线速度（≥0，一般不允许倒退）
        self.max_v = 45.0      # 最大线速度
        self.min_w = -2.2      # 最小角速度（负 = 顺时针）
        self.max_w = 2.2       # 最大角速度（正 = 逆时针）

        # ---- 加速度限制（决定了动态窗口的大小）----
        self.max_acc_v = 80.0  # 最大线加速度
        self.max_acc_w = 5.0   # 最大角加速度

        # ---- 采样与预测参数 ----
        self.dt = 0.1                  # 轨迹模拟的时间步长 (秒)
        self.predict_time = 2,0        # 预测总时长 (秒)，越长越"远视"
        self.v_resolution = 2.8       # 线速度采样步长，越小越密
        self.w_resolution = 0.18       # 角速度采样步长，越小越密

        # ---- 碰撞模型 ----
        self.robot_radius = 12.0             # 机器人半径（视为圆形）
        self.safe_margin = 5.0               # 额外安全边距
        self.max_obstacle_score_dist = 80.0  # 障碍物距离评分上限（超过此距离视为"无限远"）

        # ---- 评分权重（三者之和一般为 1.0）----
        self.heading_weight = 0.55    # 朝向权重：偏向终点方向
        self.dist_weight = 0.30       # 距离权重：远离障碍物
        self.velocity_weight = 0.15   # 速度权重：尽快到达

        # ---- 终点到达判断 ----
        self.goal_tolerance = 4.0     # 到达阈值（距离 ≤ 此值则认为到达）
        self.stop_v = 0.0             # 到达后输出的线速度
        self.stop_w = 0.0             # 到达后输出的角速度

        # ---- 坐标系转换 ----
        # 项目约定：theta=0 指向 Y+，逆时针为正
        # 数学标准： theta=0 指向 X+，逆时针为正
        # 因此在 DWA 轨迹预测前需要 +π/2 偏移
        self.theta_to_math_offset = math.pi * 0.5


class RobotState:
    """
    机器人即时状态
    ---------------
    x, y   : 世界坐标系下的位置
    v      : 当前线速度
    w      : 当前角速度
    theta  : 当前朝向角 (弧度，逆时针为正，0=Y+)
    """
    __slots__ = ("x", "y", "v", "w", "theta")  # MicroPython 内存优化

    def __init__(self, x=0.0, y=0.0, v=0.0, w=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.v = v
        self.w = w
        self.theta = theta


class DWATrajectory:
    """
    一条候选轨迹及其评分
    --------------------
    v, w        : 候选速度对 (线速度, 角速度)
    end_x, end_y : 轨迹末端在世界坐标系下的位置
    end_theta    : 轨迹末端的朝向角 (数学坐标系)
    heading      : 朝向得分 = π - |终点方向 − 末端朝向|，越高越好
    dist         : 轨迹上离障碍物的最近距离，越高越好
    velocity     : 线速度绝对值，越高越好
    score        : 综合评分（heading/dist/velocity 加权求和后归一化）
    collision    : 该轨迹是否发生碰撞
    """
    __slots__ = ("v", "w", "end_x", "end_y", "end_theta", "heading",
                 "dist", "velocity", "score", "collision")

    def __init__(self, v, w):
        self.v = v
        self.w = w
        self.end_x = 0.0
        self.end_y = 0.0
        self.end_theta = 0.0
        self.heading = 0.0       # heading 得分 (π − Δθ)
        self.dist = 0.0          # 最近障碍物距离
        self.velocity = 0.0      # 线速度（用于速度评分项）
        self.score = -1.0        # 综合评分，初始 -1
        self.collision = False   # 碰撞标志


# ----------------------------- Helpers --------------------------------

def init_uart(port=UART_PORT, baud=UART_BAUD):
    """初始化 MicroPython UART，配置为 8N1 无校验 20ms 超时。
    非 MicroPython 环境会抛出 RuntimeError。"""
    if UART is None:
        raise RuntimeError("machine.UART is only available on MicroPython")
    uart = UART(port, baud)
    uart.init(baud, bits=8, parity=None, stop=1, timeout=20)
    return uart


def ticks_ms():
    """获取当前时间戳，毫秒精度。MicroPython 用 time.ticks_ms()，PC 用 time.time() 转换。"""
    if hasattr(time, "ticks_ms"):
        return time.ticks_ms()
    return int(time.time() * 1000)


def ticks_add(ticks, delta):
    """时间戳 + 偏移量 (ms)。处理 MicroPython 的 ticks 回绕。"""
    if hasattr(time, "ticks_add"):
        return time.ticks_add(ticks, delta)
    return ticks + delta


def ticks_diff(a, b):
    """两时间戳差值 = a − b。处理 MicroPython 的 ticks 回绕。"""
    if hasattr(time, "ticks_diff"):
        return time.ticks_diff(a, b)
    return a - b


def sleep_ms(ms):
    """毫秒级延时。MicroPython 用 time.sleep_ms()，PC 用 time.sleep() 转换。"""
    if hasattr(time, "sleep_ms"):
        time.sleep_ms(ms)
    else:
        time.sleep(ms / 1000.0)


def clamp(value, low, high):
    """将 value 钳制在 [low, high] 区间内。"""
    if value < low:
        return low
    if value > high:
        return high
    return value


def normalize_angle(angle):
    """将任意弧度值归一化到 (−π, π] 区间。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def deg_to_rad_if_needed(theta):
    """智能角度转换：如果 theta 绝对值 > 2π，认为是度数，自动转为弧度。
    下位机常以度数上报朝向，此函数做兼容处理。"""
    # 2π ≈ 6.2831853；超过此范围的视为度数
    if theta > 6.2831853 or theta < -6.2831853:
        return theta * math.pi / 180.0
    return theta


def dist2(x1, y1, x2, y2):
    """两点间欧几里得距离的平方。用平方避免开根号，提高性能。"""
    dx = x1 - x2
    dy = y1 - y2
    return dx * dx + dy * dy


# ----------------------------- UART State ------------------------------

class StateFrameReader:
    """
    UART 帧缓冲解析器
    ------------------
    从原始字节流中提取以 '@' 开头 '!' 结尾的状态帧，
    解析为 RobotState 对象。支持帧粘连、半帧到达等情况。
    """
    def __init__(self, max_buf=256):
        self.buf = b""            # 未处理完的字节缓冲区
        self.max_buf = max_buf    # 缓冲区最大长度（防止异常数据堆积）
        self.last_state = None    # 最近一次成功解析的状态

    def feed(self, data):
        """喂入新到达的字节，返回解析出的状态（如有）。无新帧则返回 None。"""
        if not data:
            return None
        self.buf += data
        # 缓冲区溢出时丢弃旧数据
        if len(self.buf) > self.max_buf:
            self.buf = self.buf[-self.max_buf:]
        return self._parse_frames()

    def _parse_frames(self):
        """循环解析缓冲区中所有完整帧（@...!格式）。
        返回最后一条成功解析的状态；若无完整帧则返回 None。"""
        parsed = None
        while True:
            start = self.buf.find(b"@")
            if start < 0:
                # 无帧头：清空缓冲区
                self.buf = b""
                return parsed
            if start > 0:
                # 丢掉帧头前的垃圾数据
                self.buf = self.buf[start:]

            end = self.buf.find(b"!")
            if end < 0:
                # 有帧头但无帧尾：等待更多数据
                return parsed

            # 取出帧体（@ 和 ! 之间的内容）
            frame = self.buf[1:end]
            self.buf = self.buf[end + 1:]
            state = parse_state_frame(frame)
            if state is not None:
                self.last_state = state
                parsed = state


def parse_state_frame(frame):
    """
    解析单帧数据（不含 @ 和 ! 符号）。
    帧格式: TYPE#x#y#v#w#theta
    TYPE: O/S/P/STATE 均为状态帧
    返回 RobotState 或 None（解析失败）。
    """
    if isinstance(frame, bytes):
        try:
            frame = frame.decode("ascii")
        except Exception:
            return None

    parts = frame.split("#")
    if len(parts) < 6:
        return None

    frame_type = parts[0]
    if frame_type not in ("O", "S", "P", "STATE"):
        return None

    try:
        x = float(parts[1])
        y = float(parts[2])
        v = float(parts[3])
        w = float(parts[4])
        # 自动兼容度数/弧度
        theta = deg_to_rad_if_needed(float(parts[5]))
    except Exception:
        return None

    return RobotState(x, y, v, w, theta)


def read_robot_state(uart, reader, timeout_ms=50):
    """
    从 UART 读取一帧状态数据。
    在 timeout_ms 内轮询串口；有新帧则立即返回，超时则返回上次状态。
    """
    deadline = ticks_add(ticks_ms(), timeout_ms)
    while ticks_diff(deadline, ticks_ms()) > 0:
        if uart.any():
            state = reader.feed(uart.read(uart.any()))
            if state is not None:
                return state
        sleep_ms(2)
    return reader.last_state


def send_velocity(uart, v, w, mode="VW", state=None, config=None):
    """
    将 DWA 计算出的速度指令发送给下位机。

    参数:
        mode: 输出格式
            "VW"  — @VW#v#w!          (线速度 + 角速度，推荐)
            "VW3" — @VW#vx#vy#w!      (前向分量 + 侧向分量 + 角速度)
            "VXY" — @V#vx#vy!         (仅线速度分量，无角速度，兼容模式)
    """
    if mode == "VW":
        uart.write("@VW#%.2f#%.2f!\n" % (v, w))
    elif mode == "VW3":
        # 某些下位机使用 @VW#vx#vy#w! 格式，vx 为前向速度
        uart.write("@VW#%.2f#%.2f#%.2f!\n" % (v, 0.0, w))
    elif mode == "VXY":
        # 兼容仅支持 @V#vx#vy! 的下位机
        # 角速度 w 在此模式下无法表达，需用朝向分解线速度
        if state is None:
            uart.write("@V#%.2f#%.2f!\n" % (v, 0.0))
        else:
            cfg = config if config is not None else DWAConfig()
            theta = normalize_angle(state.theta + cfg.theta_to_math_offset)
            uart.write("@V#%.2f#%.2f!\n" %
                       (v * math.cos(theta), v * math.sin(theta)))
    else:
        raise ValueError("unknown velocity command mode")


# ----------------------------- DWA Core --------------------------------

class DWAPlanner:
    """
    DWA 核心规划器
    --------------
    每个控制周期调用 plan(state)：
      1. 计算动态窗口 (v, ω 可达范围)
      2. 在窗口内均匀采样候选速度对
      3. 模拟每条轨迹 → 评分 → 剔除碰撞轨迹
      4. 归一化三项评分并加权求和
      5. 返回最优 (v, ω)

    如果所有候选轨迹都碰撞（死胡同），进入逃逸模式：原地转向远离最近障碍物。
    """

    def __init__(self, config=None):
        self.cfg = config if config is not None else DWAConfig()
        self.obstacles = []       # 障碍物列表 [(x, y), ...] 或 [(x, y, r), ...]
        self.goal = (0.0, 0.0)    # 目标点 (x, y)

    def set_goal(self, x, y):
        """设置导航终点坐标。"""
        self.goal = (float(x), float(y))

    def set_obstacles(self, obstacles):
        """
        设置障碍物列表。
        obstacles: [(x, y), ...] 或 [(x, y, radius), ...]
        通常由上层感知模块（激光/深度相机）每帧更新。
        """
        self.obstacles = obstacles if obstacles is not None else []

    def reached_goal(self, state):
        """判断是否已到达终点（距离 ≤ goal_tolerance）。"""
        gx, gy = self.goal
        return math.sqrt(dist2(state.x, state.y, gx, gy)) <= self.cfg.goal_tolerance

    def plan(self, state):
        """
        DWA 主循环：输入当前状态，输出最优 (v, ω)。
        返回 (v, w, trajectory) 三元组；trajectory 为 DWATrajectory 或 None。
        """
        # 已到达终点 → 输出停止指令
        if self.reached_goal(state):
            return self.cfg.stop_v, self.cfg.stop_w, None

        # 第一步：计算动态窗口
        v_low, v_high, w_low, w_high = self._dynamic_window(state)

        # 第二步：在窗口内采样并模拟轨迹
        candidates = []
        v = v_low
        while v <= v_high + 0.00001:              # 浮点容差
            w = w_low
            while w <= w_high + 0.00001:
                traj = self._predict_trajectory(state, v, w)
                self._score_trajectory(traj)
                if not traj.collision:
                    candidates.append(traj)
                w += self.cfg.w_resolution
            v += self.cfg.v_resolution

        # 所有轨迹都碰撞：逃逸模式（原地转向）
        if not candidates:
            return self.cfg.stop_v, self._escape_turn(state), None

        # 第三步：归一化评分并选最优
        self._normalize_and_score(candidates)
        best = candidates[0]
        for traj in candidates[1:]:
            if traj.score > best.score:
                best = traj
        return best.v, best.w, best

    def _dynamic_window(self, state):
        """
        计算动态窗口 Vr = Vs ∩ Vd。

        Vs（运动学边界）: [min_v, max_v] × [min_w, max_w]
        Vd（加速度边界）: 当前速度 ± 最大加速度 × dt

        Vr 取两者交集。若交集为空（不应发生），退化为当前速度。
        """
        cfg = self.cfg
        dt = cfg.dt

        # 线速度窗口 = [运动学下限, 运动学上限] ∩ [当前−加速度边界, 当前+加速度边界]
        v_low = max(cfg.min_v, state.v - cfg.max_acc_v * dt)
        v_high = min(cfg.max_v, state.v + cfg.max_acc_v * dt)

        # 角速度窗口同理
        w_low = max(cfg.min_w, state.w - cfg.max_acc_w * dt)
        w_high = min(cfg.max_w, state.w + cfg.max_acc_w * dt)

        # 安全检查：若窗口为空，锁定在当前速度
        if v_low > v_high:
            v_low = clamp(state.v, cfg.min_v, cfg.max_v)
            v_high = v_low
        if w_low > w_high:
            w_low = clamp(state.w, cfg.min_w, cfg.max_w)
            w_high = w_low
        return v_low, v_high, w_low, w_high

    def _predict_trajectory(self, state, v, w):
        """
        模拟一条轨迹：从当前状态出发，以速度 (v, w) 匀速运动 predict_time 秒。

        运动模型（质点 + 朝向）：
            x_{k+1} = x_k + v · cos(θ_k) · dt
            y_{k+1} = y_k + v · sin(θ_k) · dt
            θ_{k+1} = θ_k + w · dt

        沿途检测碰撞：一旦 min_dist ≤ 0 → 标记 collision = True 并提前终止。
        返回携带末端状态和最近距离的 DWATrajectory。
        """
        cfg = self.cfg
        traj = DWATrajectory(v, w)
        x = state.x
        y = state.y
        # 坐标系转换：项目约定 → 数学标准（θ=0 指向 X+）
        theta = normalize_angle(state.theta + cfg.theta_to_math_offset)
        t = 0.0
        min_dist = cfg.max_obstacle_score_dist  # 初始化为"无限远"

        while t < cfg.predict_time:
            # 一步前向欧拉积分
            x += v * math.cos(theta) * cfg.dt
            y += v * math.sin(theta) * cfg.dt
            theta = normalize_angle(theta + w * cfg.dt)

            # 检测该步与障碍物的最近距离
            d = self._nearest_obstacle_distance_at(x, y)
            if d < min_dist:
                min_dist = d
            if min_dist <= 0.0:          # 碰撞！
                traj.collision = True
                break
            t += cfg.dt

        traj.end_x = x
        traj.end_y = y
        traj.end_theta = theta
        traj.dist = min_dist
        return traj

    def _score_trajectory(self, traj):
        """
        对一条轨迹打分（单项，不归一化）。

        朝向得分 heading：
            Δθ = |atan2(终点−轨迹末端) − 轨迹末端朝向|
            heading = π − Δθ    （Δθ = 0 → π 分；Δθ = π → 0 分）

        速度得分 velocity：
            直接用线速度绝对值，鼓励高效移动

        碰撞二次判别：
            若刹车距离 > 最近障碍物距离 → 撞，标记 collision = True
            刹车距离 = v² / (2 · max_acc_v)（匀减速模型）
        """
        # 计算轨迹末端朝向与终点方向的夹角
        goal_angle = math.atan2(self.goal[1] - traj.end_y,
                                self.goal[0] - traj.end_x)
        angle_error = abs(normalize_angle(goal_angle - traj.end_theta))
        traj.heading = math.pi - angle_error   # Δθ=0 → π分; Δθ=π → 0分
        traj.velocity = abs(traj.v)

        # 刹车距离检查：即使轨迹模拟未撞，速度也可能快到刹不住
        stop_dist = (traj.v * traj.v) / (2.0 * self.cfg.max_acc_v)
        if traj.dist <= 0.0 or stop_dist > traj.dist:
            traj.collision = True

    def _nearest_obstacle_distance_at(self, px, py):
        """
        计算点 (px, py) 到最近障碍物的有符号距离。

        有符号距离 = 欧几里得距离 − robot_radius − safe_margin − 障碍物自身半径
        若 ≤ 0，表示机器人外廓已侵入障碍物安全区。
        若无障碍物，返回 max_obstacle_score_dist（视为"无限远"）。
        """
        cfg = self.cfg
        if not self.obstacles:
            return cfg.max_obstacle_score_dist

        min_d = cfg.max_obstacle_score_dist
        for obs in self.obstacles:
            ox = obs[0]
            oy = obs[1]
            radius = obs[2] if len(obs) >= 3 else 0.0   # 障碍物自身半径，默认 0
            # 扣除机器人半径 + 安全边距 + 障碍物半径
            d = math.sqrt(dist2(px, py, ox, oy))
            d -= cfg.robot_radius + cfg.safe_margin + radius
            if d < min_d:
                min_d = d
            if min_d <= 0.0:
                return min_d
        return min_d

    def _normalize_and_score(self, candidates):
        """
        对候选轨迹的三项得分做"和归一化"并加权求和。

        归一化方式：每项得分 / 该项所有轨迹得分之和。
        这样三项得分被压到同一量级，权重 (α, β, γ) 才能真正控制相对重要性。

        最终评分：
            score = α · heading_norm + β · dist_norm + γ · velocity_norm

        其中 α + β + γ 通常为 1.0，各项被归一化到 [0, 1] 的相对尺度。
        """
        heading_sum = 0.0
        dist_sum = 0.0
        velocity_sum = 0.0

        for traj in candidates:
            if traj.heading > 0.0:
                heading_sum += traj.heading
            if traj.dist > 0.0:
                # 截断到 max_obstacle_score_dist，超过的不加分
                dist_sum += min(traj.dist, self.cfg.max_obstacle_score_dist)
            if traj.velocity > 0.0:
                velocity_sum += traj.velocity

        # 防止除以零
        if heading_sum <= 0.0:
            heading_sum = 1.0
        if dist_sum <= 0.0:
            dist_sum = 1.0
        if velocity_sum <= 0.0:
            velocity_sum = 1.0

        for traj in candidates:
            heading_norm = max(traj.heading, 0.0) / heading_sum
            dist_norm = min(max(traj.dist, 0.0),
                            self.cfg.max_obstacle_score_dist) / dist_sum
            velocity_norm = traj.velocity / velocity_sum
            traj.score = (self.cfg.heading_weight * heading_norm +
                          self.cfg.dist_weight * dist_norm +
                          self.cfg.velocity_weight * velocity_norm)

    def _escape_turn(self, state):
        """
        逃逸模式：所有候选轨迹都碰撞时调用。

        策略：找到最近的障碍物，如果它在左侧则往右转（反之亦然），
        以最大角速度的 60% 原地旋转，直到有安全轨迹出现。
        若无障碍物则默认逆时针转。
        """
        if not self.obstacles:
            # 没有障碍物却全碰撞：反常，默认正转
            return clamp(self.cfg.max_w * 0.5, self.cfg.min_w, self.cfg.max_w)

        # 找最近障碍物
        nearest = self.obstacles[0]
        nearest_d2 = dist2(state.x, state.y, nearest[0], nearest[1])
        for obs in self.obstacles[1:]:
            d2 = dist2(state.x, state.y, obs[0], obs[1])
            if d2 < nearest_d2:
                nearest = obs
                nearest_d2 = d2

        # 判断障碍物在左侧还是右侧
        theta = normalize_angle(state.theta + self.cfg.theta_to_math_offset)
        obs_angle = math.atan2(nearest[1] - state.y, nearest[0] - state.x)
        err = normalize_angle(obs_angle - theta)

        # 障碍物在正前方偏左 → 右转（角速度为负）
        # 障碍物在正前方偏右 → 左转（角速度为正）
        if err >= 0.0:
            return clamp(-self.cfg.max_w * 0.6, self.cfg.min_w, self.cfg.max_w)
        return clamp(self.cfg.max_w * 0.6, self.cfg.min_w, self.cfg.max_w)


# ----------------------------- Runner ----------------------------------

def run_once(uart, planner, reader, command_mode="VW"):
    """
    执行一次 DWA 迭代：读取状态 → 规划 → 发送指令。
    返回 (v, w, trajectory) 三元组；无状态时返回 None。
    适合在外部控制循环中逐帧调用。
    """
    state = read_robot_state(uart, reader)
    if state is None:
        return None
    v, w, traj = planner.plan(state)
    send_velocity(uart, v, w, command_mode, state, planner.cfg)
    return v, w, traj


def run_loop(uart, goal, obstacles=None, config=None,
             command_mode="VW", period_ms=100):
    """
    DWA 主循环：阻塞运行，每 period_ms 毫秒迭代一次。

    参数:
        uart:         已初始化的 UART 对象
        goal:         终点坐标 (x, y)
        obstacles:    初始障碍物列表（通常为 None，由外部每帧更新）
        config:       DWAConfig 实例
        command_mode: 速度指令格式 ("VW" / "VW3" / "VXY")
        period_ms:    控制周期 (ms)，建议 50~200
    """
    planner = DWAPlanner(config)
    planner.set_goal(goal[0], goal[1])
    planner.set_obstacles(obstacles)
    reader = StateFrameReader()

    while True:
        result = run_once(uart, planner, reader, command_mode)
        if result is not None:
            v, w, _ = result
            print("[DWA] v=%.2f w=%.2f" % (v, w))
        sleep_ms(period_ms)


if __name__ == "__main__":
    # ---- 示例：PC 环境下无法真正运行（需要 MicroPython UART）----
    # 实际部署时，goal 和 obstacles 应由上层任务/感知模块下发
    uart = init_uart()
    cfg = DWAConfig()
    demo_goal = (200.0, 120.0)
    demo_obstacles = [(120.0, 80.0, 8.0), (150.0, 115.0, 8.0)]
    run_loop(uart, demo_goal, demo_obstacles, cfg)
