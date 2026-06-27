"""
四步状态机：网球 → 沙包 → 小熊(暂缺) → 网球

旋转链: +90° → -180° → +90° (净 360°, 回到 0°)
朝向序列: 0°(Y↑) → 90°(X←) → -90°(X→) → 0°(Y↑)
坐标系: 逆时针=正, 顺时针=负, 绝对位姿归一化至 (-180, 180]
扫描方式: 连续移动至扫描终点, 物体进入画面中心时急停, 查位置后偏移10cm推送

工作流程:
  1. 通过 UART 向下位机发送移动/旋转指令
  2. 发扫描终点, 连续移动+检测, 物体居中时发 @V#0#0! 急停
  3. 查询下位机实际位置 → 偏移 10cm → 推至对面边缘
  4. 每次进行完一个循环同步检测信标 (AprilTag) 进行 PnP 重定位

下位机协议: ASCII 帧 @...#...!  (见 uart_comm.py)
"""

import sensor
import image
import time
import math
import gc

from uart_comm import (init_uart, send_rotate, send_move_to,
                       send_move_delta, send_set_position,
                       send_speed, query_position,
                       UART_PORT, UART_BAUD)

BEAR_DEBUG_FRAMES = 10         # 小熊调试只跑少量帧, 避免日志过长
BEAR_SCAN_FRAMES = 300         # 小熊正式扫描帧数

# ── 导入识别模块 ──────────────────────────────────
from tennis_recognition import find_tennis
from earthbags_recognition import find_earthbag
from beacon_recognition import find_beacon
from bear_recognition import find_bear, init_bear

# ── 场地参数 (单位: cm) ────────
FIELD_WIDTH = 320    # 场地宽度 (横向)
FIELD_HEIGHT = 240   # 场地长度 (纵向)

# ── 信标世界坐标 ──
# 格式: AprilTag ID → (world_x_cm, world_y_cm)

BEACON_WORLD_POSITIONS = {
    0: (110, 240),   # 信标 0 位于左上方，x轴坐标与物品放置区start_x相同
    1: (210, 240),    # 信标 1 位于右上方，x轴坐标与物品放置区end_x相同
}

# ── 信标 AprilTag 物理尺寸 (米) ──
BEACON_TAG_SIZE_M = 0.12  # 12cm 标签

# ── 相机内参 (QQVGA 160x120, 轻量化) ──
# 基于 OpenMV 镜头: 焦距 2.8mm, 感光元件 3.984mm x 2.952mm
CAM_FX = (2.8 / 3.984) * 160   # ≈ 112 px
CAM_FY = (2.8 / 2.952) * 120   # ≈ 114 px
CAM_CX = 320 * 0.5             # 160 px
CAM_CY = 240 * 0.5             # 120 px
FOCAL_LENGTH_PX = 112           # 距离估算用近似焦距

# ── 物体真实尺寸 (直径/宽度, cm) ──
OBJECT_SIZES = {
    'tennis':   6.6,    # 网球直径
    'earthbag': 7.0,   # 沙包宽度
    'bear':     15.0,   # 小熊宽度 (待标定)
}

# ── 抵近参数 ──
CENTER_TOLERANCE_PX = 30      # 横向居中容差 (像素), 放宽适配移动中检测
OFFSET_CM = 10                # 偏移量 (cm)
SCAN_TIMEOUT_FRAMES = 300     # 扫描超时帧数 (~10秒 @30fps)

# ── 物体个数配置 ──
TENNIS_TOTAL = 0       # 网球总个数
EARTHBAG_TOTAL = 1     # 沙包总个数
BEAR_TOTAL = 1         # 小熊总个数


# =============================================================
#  小车状态管理
# =============================================================

class RobotState:
    """跟踪小车朝向 (坐标由下位机维护)"""

    def __init__(self, start_x=40, start_y=-12):
        self.x = start_x
        self.y = start_y
        self.heading = 0          # 朝向角度: 0=Y+, 90=X-, -90=X+, 180=Y- (逆时针为正)

    def set_position(self, x, y):
        self.x = x
        self.y = y

    def rotate(self, angle):
        """归一化至 (-180, 180]"""
        self.heading = ((self.heading + angle + 180) % 360) - 180


# =============================================================
#  检测函数包装器 (统一返回格式)
# =============================================================

def _detect_tennis(img):
    r = find_tennis(img)
    if r is not None:
        cx, cy, _, w, h = r
        return (cx, cy, max(w, h))
    return None


def _detect_earthbag(img):
    r = find_earthbag(img)
    if r is not None:
        _, cx, cy, w, h = r
        return (cx, cy, max(w, h))
    return None


def _detect_bear(img):
    r = find_bear(img)
    if r is not None:
        cx, cy, w, h = r
        return (cx, cy, max(w, h))
    return None


# =============================================================
#  信标 PnP 重定位
# =============================================================

def localize_from_beacon(img, state, uart):
    """PnP 重定位, 更新 state 并同步下位机"""
    tags = img.find_apriltags(fx=CAM_FX, fy=CAM_FY, cx=CAM_CX, cy=CAM_CY,
                              tagsize=BEACON_TAG_SIZE_M)
    for tag in tags:
        tag_id = tag.id()
        if tag_id not in BEACON_WORLD_POSITIONS:
            continue

        beacon_wx, beacon_wy = BEACON_WORLD_POSITIONS[tag_id]

        tx_cm = tag.x_translation() * 100
        tz_cm = tag.z_translation() * 100
        heading = state.heading

        if heading == 0:
            wdx, wdy = tx_cm, tz_cm
        elif heading == -90:
            wdx, wdy = tz_cm, -tx_cm
        elif heading == 180:
            wdx, wdy = -tx_cm, -tz_cm
        elif heading == 90:
            wdx, wdy = -tz_cm, tx_cm
        else:
            continue

        rx = int(beacon_wx - wdx)
        ry = int(beacon_wy - wdy)
        rx = max(0, min(FIELD_WIDTH, rx))
        ry = max(0, min(FIELD_HEIGHT, ry))

        state.set_position(rx, ry)
        send_set_position(uart, rx, ry)
        print("[信标] PnP 重定位! Tag#%d → (%d, %d)" % (tag_id, rx, ry))
        return True
    return False


# =============================================================
#  辅助函数
# =============================================================

def _wait_frames(ms):
    time.sleep_ms(ms)


def is_centered(cx, img_w):
    """物体是否横向居中 (纵向与距离有关, 不检查)"""
    return abs(cx - img_w // 2) < CENTER_TOLERANCE_PX


def rotate_to_heading(uart, state, target_heading):
    """旋转到指定绝对朝向。"""
    if state.heading == target_heading:
        return
    delta = (target_heading - state.heading) % 360
    angle = delta if delta <= 180 else delta - 360
    print("[指令] 旋转 %d° → 朝向 %d°" % (angle, target_heading))
    send_rotate(uart, angle)
    _wait_frames(100)
    state.rotate(angle)


# =============================================================
#  连续扫描 (移动中检测, 中心急停)
# =============================================================

def scan_until_centered(uart, target_x, target_y, detect_fn,
                        timeout=SCAN_TIMEOUT_FRAMES):
    """
    发送机器人到 (target_x, target_y), 移动中持续检测。
    物体进入画面中心时: 发 @V#0#0! 急停 → 查位置 → 返回。
    超时未找到: 急停, 返回 None。

    detect_fn: fn(img) → (cx, cy, pixel_size) 或 None
    返回: (cx, cy, px, actual_x, actual_y, img)  或 (None,)*6
    """
    print("[扫描] 连续移动至 (%d, %d), 超时%d帧" % (target_x, target_y, timeout))
    send_move_to(uart, target_x, target_y)

    for frm in range(timeout):
        img = sensor.snapshot()
        r = detect_fn(img)

        # 首帧标记: 若卡死在推理上, 这条不会打印 → 可区分"卡死"与"只是慢"
        if frm == 0:
            print("[扫描] 首帧检测完成, 进入扫描循环")
        # 每30帧打印进度 (熊检测~6fps, 30帧≈5秒)
        if frm > 0 and frm % 30 == 0:
            print("[扫描] ...%d/%d帧" % (frm, timeout))

        if r is not None:
            cx, cy, px = r
            if is_centered(cx, img.width()):
                print("[扫描] ✓ 物体横向居中! 帧%d cx=%d/%d → 急停" %
                      (frm, cx, img.width()))
                send_speed(uart, 0, 0)
                _wait_frames(15)   # 等 ~500ms 让机器人停稳

                # 查询下位机实际位置
                pos = query_position(uart)
                if pos is not None:
                    ax, ay = pos
                    print("[扫描] 下位机位置: (%.1f, %.1f)" % (ax, ay))
                    return cx, cy, px, ax, ay, img
                else:
                    # 查位置失败, 用目标坐标兜底
                    print("[扫描] ⚠ 查位置失败, 用终点兜底")
                    return cx, cy, px, target_x, target_y, img

    # 超时
    print("[扫描] ✗ 超时, 未找到")
    send_speed(uart, 0, 0)
    return None, None, None, None, None, None


def scan_bear_debug(uart, target_x, target_y, timeout=SCAN_TIMEOUT_FRAMES):
    """
    小熊专用调试扫描: 拆开 snapshot / find_bear / tf.detect 的卡点。
    返回格式与 scan_until_centered 相同。
    """
    print("[小熊调试] 扫描至 (%d, %d), 超时%d帧" %
          (target_x, target_y, timeout))
    send_move_to(uart, target_x, target_y)

    for frm in range(timeout):
        print("[小熊调试] 帧%d snapshot前" % frm)
        img = sensor.snapshot()
        print("[小熊调试] 帧%d snapshot后 img=%dx%d" %
              (frm, img.width(), img.height()))

        print("[小熊调试] 帧%d find_bear前" % frm)
        r0 = find_bear(img)
        print("[小熊调试] 帧%d find_bear后" % frm)

        if frm == 0:
            print("[小熊调试] 首帧完成")
        if frm > 0 and frm % 30 == 0:
            print("[小熊调试] ...%d/%d帧" % (frm, timeout))

        if r0 is not None:
            cx, cy, w, h = r0
            px = max(w, h)
            if is_centered(cx, img.width()):
                print("[小熊调试] ✓ 小熊横向居中 帧%d cx=%d/%d" %
                      (frm, cx, img.width()))
                send_speed(uart, 0, 0)
                return cx, cy, px, target_x, target_y, img

    print("[小熊调试] ✗ 超时, 未找到")
    send_speed(uart, 0, 0)
    return None, None, None, None, None, None


def scan_bear_direct(uart, target_x, target_y, timeout=SCAN_TIMEOUT_FRAMES):
    """
    小熊正式扫描路径。
    重要: 直接调用 find_bear(img), 不通过 detect_fn 回调和 _detect_bear 包装。
    OpenART mini 上回调包装路径会让 tf.detect 卡死。
    """
    print("[小熊扫描] 连续移动至 (%d, %d), 超时%d帧" %
          (target_x, target_y, timeout))
    send_move_to(uart, target_x, target_y)

    for frm in range(timeout):
        img = sensor.snapshot()
        r = find_bear(img)

        if frm == 0:
            print("[小熊扫描] 首帧检测完成, 进入扫描循环")
        if frm > 0 and frm % 30 == 0:
            print("[小熊扫描] ...%d/%d帧" % (frm, timeout))

        if r is not None:
            cx, cy, w, h = r
            px = max(w, h)
            if is_centered(cx, img.width()):
                print("[小熊扫描] ✓ 小熊横向居中! 帧%d cx=%d/%d → 急停" %
                      (frm, cx, img.width()))
                send_speed(uart, 0, 0)
                _wait_frames(15)   # 等 ~500ms 让机器人停稳

                # 和普通扫描一样, 居中急停后查询下位机实际位置
                pos = query_position(uart)
                if pos is not None:
                    ax, ay = pos
                    print("[小熊扫描] 下位机位置: (%.1f, %.1f)" % (ax, ay))
                    return cx, cy, px, ax, ay, img
                else:
                    print("[小熊扫描] ⚠ 查位置失败, 用终点兜底")
                    return cx, cy, px, target_x, target_y, img

    print("[小熊扫描] ✗ 超时, 未找到")
    send_speed(uart, 0, 0)
    return None, None, None, None, None, None


# =============================================================
#  偏移 + 推送
# =============================================================

def _push_to_edge(uart, state, cur_x, cur_y,
                  offset_wx=0, offset_wy=0):
    """
    从 (cur_x, cur_y) 偏移 OFFSET_CM cm 后, 沿朝向直线推至对面边缘。
    返回: (final_x, final_y)
    """
    h = state.heading

    # ── 步骤1: 横向偏移 ──
    if offset_wx != 0 or offset_wy != 0:
        print("[偏移] 横向偏移 (%+d, %+d) cm" % (offset_wx, offset_wy))
        send_move_delta(uart, offset_wx, offset_wy, 40)
        _wait_frames(100)
        cur_x += offset_wx
        cur_y += offset_wy

    # ── 步骤2: 推至边缘 ──
    if h == 0:
        ex, ey = cur_x, FIELD_HEIGHT
    elif h == -90:
        ex, ey = FIELD_WIDTH, cur_y
    elif h == 90:
        ex, ey = 0, cur_y
    else:
        ex, ey = cur_x, cur_y

    print("[推送] 沿 %d° 直线推至边缘 (%d, %d)" % (h, ex, ey))
    send_move_to(uart, ex, ey)
    _wait_frames(300)
    return ex, ey


def _push_bear_to_left_edge(uart, cur_x, cur_y, offset_y=OFFSET_CM):
    """
    小熊专用推送: cur_x/cur_y 来自下位机 query_position()。
    先沿 Y 方向偏移, 偏移后再次 query_position() 获取真实位置,
    再推到左边界 x=0。
    不按 state.heading 推, 因为小熊目标要求固定推向 X-。
    """
    print("[小熊推送] 当前下位机位置 (%.1f, %.1f)" % (cur_x, cur_y))
    if offset_y != 0:
        print("[小熊偏移] Y方向偏移 %+d cm" % offset_y)
        send_move_delta(uart, 0, offset_y, 40)
        _wait_frames(100)
        cur_y += offset_y

        pos = query_position(uart)
        if pos is not None:
            cur_x, cur_y = pos
            print("[小熊偏移] 偏移后下位机位置 (%.1f, %.1f)" %
                  (cur_x, cur_y))
        else:
            print("[小熊偏移] ⚠ 偏移后查位置失败, 使用估算位置 (%.1f, %.1f)" %
                  (cur_x, cur_y))

    ex, ey = 0, cur_y
    print("[小熊推送] 推至左边界 X=0 → (%d, %.1f)" % (ex, ey))
    send_move_to(uart, ex, ey)
    _wait_frames(300)
    return ex, ey


# =============================================================
#  决策主循环
# =============================================================

def main():
    print("===== 决策系统初始化 =====")

    # ── 0. 预加载熊模型 (最先加载, 此时内存最干净, 确保tf.load成功) ──
    bear_available = init_bear()

    # ── 1. 初始化传感器 ──
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=2000)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)

    # ── 1.5 熊模型预热推理 (关键!) ──
    # OpenMV 的 tf arena 往往在「第一次 tf.detect」才真正分配, 而非 tf.load。
    # 若拖到第3步才首次推理, 那时堆已被前两步搞满/碎片化 → arena 分配硬卡死。
    # 这里趁开机堆干净, 先空跑一次 detect 把 arena 占好, 之后复用不再分配。
    if bear_available:
        print("[小熊] 预热推理 (在干净堆上分配 tensor arena)...")
        gc.collect()
        print("[小熊] 预热前 mem_free=%d" % gc.mem_free())
        find_bear(sensor.snapshot())
        gc.collect()
        print("[小熊] 预热完成 mem_free=%d" % gc.mem_free())

    # ── 2. 初始化串口 ──
    uart = init_uart()
    print("UART 初始化完成 (端口%d, %d bps)" % (UART_PORT, UART_BAUD))

    # ── 3. 初始化小车状态 ──
    state = RobotState(start_x=40, start_y=-12)
    _wait_frames(30)
    send_set_position(uart, 40, -12)

    # ── 4. 四步主循环 ──
    step = 1
    cmd_x, cmd_y = 40, -12   # 当前命令坐标

    # ── 物体剩余个数 ──
    tennis_count = TENNIS_TOTAL
    earthbag_count = EARTHBAG_TOTAL
    bear_count = BEAR_TOTAL

    print("===== 开始决策循环 =====")
    print("[计数] 初始: 网球=%d 沙包=%d 小熊=%d" %
          (tennis_count, earthbag_count, bear_count))

    while True:
        print("\n========== 第 %d 步 ==========" % step)

        # ── 步骤 1: 0°(Y↑) → (110,70) → 扫描网球 X:110→210 ──
        if step == 1:
            if tennis_count == 0:
                print("[计数] 网球已清空, 跳过步骤1")
                step = 2
                continue

            # 确保朝向 0°
            if state.heading != 0:
                delta = (0 - state.heading) % 360
                angle = delta if delta <= 180 else delta - 360
                print("[指令] 旋转 %d° → 朝向 0°" % angle)
                send_rotate(uart, angle)
                _wait_frames(30)
                state.rotate(angle)

            # 前往扫描起点
            print("[指令] 前往扫描起点 (110, 70)")
            send_move_to(uart, 110, 70)
            _wait_frames(150)
            cmd_x, cmd_y = 110, 70

            # 连续扫描: 从当前点到 (210, 70)
            cx, cy, px, ax, ay, img = scan_until_centered(
                uart, 210, 70, _detect_tennis)

            if cx is not None:
                print("[检测] 网球! 图像(%d,%d) 像素:%d  位置(%.1f,%.1f)" %
                      (cx, cy, px, ax, ay))
                img.draw_cross(cx, cy, color=(0, 255, 0))
                cmd_x, cmd_y = _push_to_edge(
                    uart, state, ax, ay, offset_wx=OFFSET_CM, offset_wy=0)
                tennis_count -= 1
                print("[计数] 网球剩余: %d" % tennis_count)
            else:
                print("[检测] 未找到网球, 跳过")
                cmd_x = 210

            step = 2

        # ── 步骤 2: 90°(X←) → Y=170 → 扫描沙包 Y:170→70 ──
        elif step == 2:
            if earthbag_count == 0:
                print("[计数] 沙包已清空, 跳过步骤2")
                step = 3
                continue

            send_rotate(uart, -90)
            _wait_frames(100)
            state.rotate(-90)
            print("[状态] 朝向: %d°" % state.heading)

            print("[指令] 平移至 Y=170 (保持 X=%d)" % cmd_x)
            send_move_to(uart, cmd_x, 170)
            _wait_frames(100)
            cmd_x, cmd_y = cmd_x, 170

            # 切换灰度模式
            sensor.reset()
            sensor.set_pixformat(sensor.GRAYSCALE)
            sensor.set_framesize(sensor.QQVGA)
            sensor.set_auto_gain(False)
            sensor.set_auto_whitebal(False)
            _wait_frames(30)

            cx, cy, px, ax, ay, img = scan_until_centered(
                uart, cmd_x, 70, _detect_earthbag)

            if cx is not None:
                print("[检测] 沙包! 图像(%d,%d) 像素:%d  位置(%.1f,%.1f)" %
                      (cx, cy, px, ax, ay))
                cmd_x, cmd_y = _push_to_edge(
                    uart, state, ax, ay, offset_wx=-OFFSET_CM, offset_wy=0)
                # 后退10cm
                send_move_delta(uart, -OFFSET_CM, 0, 40)
                _wait_frames(450)
                cmd_x = FIELD_WIDTH - OFFSET_CM
                earthbag_count -= 1
                print("[计数] 沙包剩余: %d" % earthbag_count)
            else:
                print("[检测] 沿Y未找到沙包, 回到X扫描起点 (110,70) 继续沿X找沙包")

                send_move_to(uart, 110, 70)
                _wait_frames(1000)
                cmd_x, cmd_y = 110, 70

                rotate_to_heading(uart, state, 0)
                print("[状态] 朝向: %d°" % state.heading)

                cx2, cy2, px2, ax2, ay2, img2 = scan_until_centered(
                    uart, 210, 70, _detect_earthbag)

                if cx2 is not None:
                    print("[检测] X向补扫沙包! 图像(%d,%d) 像素:%d  位置(%.1f,%.1f)" %
                          (cx2, cy2, px2, ax2, ay2))

                    print("[沙包补扫] X方向偏移 %+d cm" % OFFSET_CM)
                    send_move_delta(uart, OFFSET_CM, 0, 40)
                    _wait_frames(450)

                    pos = query_position(uart)
                    if pos is not None:
                        cur_x, cur_y = pos
                        print("[沙包补扫] 偏移后下位机位置 (%.1f, %.1f)" %
                              (cur_x, cur_y))
                    else:
                        cur_x, cur_y = ax2 + OFFSET_CM, ay2
                        print("[沙包补扫] ⚠ 偏移后查位失败, 使用估算位置 (%.1f, %.1f)" %
                              (cur_x, cur_y))

                    print("[沙包补扫] 平移至 Y=170 (保持 X=%.1f)" % cur_x)
                    send_move_to(uart, cur_x, 170)
                    _wait_frames(600)

                    pos = query_position(uart)
                    if pos is not None:
                        cur_x, cur_y = pos
                        print("[沙包补扫] 到Y=170后下位机位置 (%.1f, %.1f)" %
                              (cur_x, cur_y))
                    else:
                        cur_y = 170
                        print("[沙包补扫] ⚠ 到Y=170后查位失败, 使用估算位置 (%.1f, %.1f)" %
                              (cur_x, cur_y))

                    rotate_to_heading(uart, state, -90)
                    print("[状态] 朝向: %d°" % state.heading)

                    print("[沙包补扫] 推至右边界 X=320")
                    send_move_to(uart, FIELD_WIDTH, cur_y)
                    _wait_frames(300)
                    # 后退10cm
                    send_move_delta(uart, -OFFSET_CM, 0, 40)
                    _wait_frames(450)
                    cmd_x, cmd_y = FIELD_WIDTH - OFFSET_CM, cur_y
                    earthbag_count -= 1
                    print("[计数] 沙包剩余: %d" % earthbag_count)
                else:
                    print("[检测] X向补扫也未找到沙包, 跳过")
                    cmd_x, cmd_y = 210, 70

            # 切回彩色
            sensor.reset()
            sensor.set_pixformat(sensor.RGB565)
            sensor.set_framesize(sensor.QQVGA)
            sensor.set_auto_gain(False)
            sensor.set_auto_whitebal(False)
            _wait_frames(30)

            step = 3

        # ── 步骤 3: 0°(Y↑) → (210,70) → 扫描小熊 Y:70→170 ──
        elif step == 3:
            do_scan = True
            if bear_count == 0:
                print("[计数] 小熊已清空, 跳过步骤3")
                do_scan = False
            elif not bear_available:
                print("[小熊] 模型不可用, 跳过步骤3")
                do_scan = False

            if do_scan:
                rotate_to_heading(uart, state, 0)
                print("[状态] 朝向: %d°" % state.heading)

                # 前往扫描起点 (210, 70)
                print("[指令] 前往扫描起点 (210, 70)")
                send_move_to(uart, 210, 70)
                _wait_frames(300)
                cmd_x, cmd_y = 210, 70

                # 熊模型用 QVGA。必须 sensor.reset(): 第2步用过 GRAYSCALE/QQVGA,
                # 只靠 set_framesize 切回 QVGA 帧缓冲重建不干净, tf.detect 会死在
                # 逐像素读取上(已用 mem_free 排除内存因素)。reset 让其与开机预热同路径。
                sensor.reset()
                sensor.set_pixformat(sensor.RGB565)
                sensor.set_framesize(sensor.QVGA)
                sensor.set_auto_gain(False)
                sensor.set_auto_whitebal(False)
                sensor.skip_frames(time=300)
                _wait_frames(30)

                gc.collect()
                print("[小熊] 推理前 mem_free=%d" % gc.mem_free())

                cx, cy, px, ax, ay, img = scan_bear_direct(
                    uart, 210, 170,
                    timeout=BEAR_SCAN_FRAMES)

                if cx is not None:
                    print("[检测] 小熊! 图像(%d,%d) 像素:%d  位置(%.1f,%.1f)" %
                          (cx, cy, px, ax, ay))
                    cmd_x, cmd_y = _push_bear_to_left_edge(
                        uart, ax, ay, offset_y=OFFSET_CM - 5)
                    bear_count -= 1
                    print("[计数] 小熊剩余: %d" % bear_count)
                else:
                    print("[检测] 未找到小熊, 跳过")
                    cmd_y = 170

            # ── 核心扫描循环结束, 检查计数 ──
            if tennis_count == 0 and earthbag_count == 0 and bear_count == 0:
                print("[计数] 所有物体已清空 → 返回起点")
                send_move_to(uart, 110, 100)
                _wait_frames(500)
                print("[循环] 返回起点 (25, -25)")
                send_move_to(uart, 25, -25)
                _wait_frames(500)
                cmd_x, cmd_y = 25, -25

                print("[信标] 开始本轮重定位...")
                img = sensor.snapshot()
                if localize_from_beacon(img, state, uart):
                    cmd_x, cmd_y = state.x, state.y
                    print("[信标] 重定位完成 → (%.1f, %.1f)" %
                          (cmd_x, cmd_y))
                else:
                    print("[信标] 未检测到信标, 使用命令坐标继续")

                tennis_count = TENNIS_TOTAL
                earthbag_count = EARTHBAG_TOTAL
                bear_count = BEAR_TOTAL
                print("[计数] 计数已重置: 网球=%d 沙包=%d 小熊=%d" %
                      (tennis_count, earthbag_count, bear_count))
            else:
                print("[计数] 剩余: 网球=%d 沙包=%d 小熊=%d → 下一轮核心扫描" %
                      (tennis_count, earthbag_count, bear_count))

            step = 1

        # ── 步骤 4: 0°(Y↑) → X=110 → 扫描网球 X:110→210 ──
        elif step == 4:
            if tennis_count == 0:
                print("[计数] 网球已清空, 跳过步骤4")
                if tennis_count == 0 and earthbag_count == 0 and bear_count == 0:
                    print("[计数] 所有物体已清空 → 返回起点")
                    send_move_to(uart, 110, 100)
                    _wait_frames(500)
                    print("[循环] 返回起点 (25, -25)")
                    send_move_to(uart, 25, -25)
                    _wait_frames(500)
                    cmd_x, cmd_y = 25, -25
                    print("[信标] 开始本轮重定位...")
                    img = sensor.snapshot()
                    if localize_from_beacon(img, state, uart):
                        cmd_x, cmd_y = state.x, state.y
                        print("[信标] 重定位完成 → (%.1f, %.1f)" %
                              (cmd_x, cmd_y))
                    else:
                        print("[信标] 未检测到信标, 使用命令坐标继续")
                    tennis_count = TENNIS_TOTAL
                    earthbag_count = EARTHBAG_TOTAL
                    bear_count = BEAR_TOTAL
                    print("[计数] 计数已重置: 网球=%d 沙包=%d 小熊=%d" %
                          (tennis_count, earthbag_count, bear_count))
                else:
                    print("[计数] 剩余: 网球=%d 沙包=%d 小熊=%d → 继续下一轮" %
                          (tennis_count, earthbag_count, bear_count))
                step = 1
                continue

            rotate_to_heading(uart, state, 0)
            print("[状态] 朝向: %d°" % state.heading)

            print("[指令] 平移至 X=110 (保持 Y=%d)" % cmd_y)
            send_move_to(uart, 110, cmd_y)
            _wait_frames(150)
            cmd_x, cmd_y = 110, cmd_y

            cx, cy, px, ax, ay, img = scan_until_centered(
                uart, 210, cmd_y, _detect_tennis)

            if cx is not None:
                print("[检测] 网球! 图像(%d,%d) 像素:%d  位置(%.1f,%.1f)" %
                      (cx, cy, px, ax, ay))
                img.draw_cross(cx, cy, color=(0, 255, 0))
                cmd_x, cmd_y = _push_to_edge(
                    uart, state, ax, ay, offset_wx=OFFSET_CM, offset_wy=0)
                tennis_count -= 1
                print("[计数] 网球剩余: %d" % tennis_count)
            else:
                print("[检测] 未找到网球, 跳过")
                cmd_x = 210

            # ── 检查是否全部清空 ──
            if tennis_count == 0 and earthbag_count == 0 and bear_count == 0:
                send_move_to(uart, 110, 100)
                _wait_frames(500)

                print("[计数] 所有物体已清空 → 返回起点")
                print("[循环] 返回起点 (25, -25)")
                send_move_to(uart, 25, -25)
                _wait_frames(500)
                cmd_x, cmd_y = 25, -25

                # ── 信标重定位 ──
                print("[信标] 开始本轮重定位...")
                img = sensor.snapshot()
                if localize_from_beacon(img, state, uart):
                    cmd_x, cmd_y = state.x, state.y
                    print("[信标] 重定位完成 → (%.1f, %.1f)" %
                          (cmd_x, cmd_y))
                else:
                    print("[信标] 未检测到信标, 使用命令坐标继续")

                # ── 重置计数 ──
                tennis_count = TENNIS_TOTAL
                earthbag_count = EARTHBAG_TOTAL
                bear_count = BEAR_TOTAL
                print("[计数] 计数已重置: 网球=%d 沙包=%d 小熊=%d" %
                      (tennis_count, earthbag_count, bear_count))
            else:
                print("[计数] 剩余: 网球=%d 沙包=%d 小熊=%d → 继续下一轮" %
                      (tennis_count, earthbag_count, bear_count))

            step = 1

        _wait_frames(50)


# =============================================================
if __name__ == "__main__":
    main()
