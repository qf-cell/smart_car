from hardware_init import *

# 串口接收缓冲区
_uart6_buf = bytearray()
MAX_BUF_SIZE = 256  # 缓冲区上限

# 跟随车在主车左侧的距离（米），用于环绕补偿
FOLLOWER_OFFSET = 0.2


def _read_available(uart_obj):
    """非阻塞读取串口所有可用字节，返回 bytes 或 None"""
    n = uart_obj.any()
    if n > 0:
        return uart_obj.read(n)
    return None


def _extract_frames(buf, parse_func):
    """
    从缓冲区中提取所有 @...! 完整帧并调用 parse_func 解析，
    解析成功的帧从缓冲区中移除，不完整的帧保留等待后续数据。
    """
    while True:
        at_idx = buf.find(b'@')
        if at_idx == -1:
            # 没有帧头，清空缓冲区
            buf[:] = b''
            break
        # 丢弃 @ 之前的垃圾字节
        if at_idx > 0:
            buf[:] = buf[at_idx:]
        
        excl_idx = buf.find(b'!')
        if excl_idx == -1:
            # 帧尾未到，保留缓冲区等待更多数据
            break
        
        # 提取完整帧
        frame = buf[:excl_idx + 1]
        buf[:] = buf[excl_idx + 1:]
        
        # 解析帧
        try:
            cmd_str = frame.decode('ascii')
            parse_func(cmd_str)
        except Exception:
            # 解析失败则丢弃该帧
            pass


# 对外接口：uart_control() — 从 uart6（摄像头）接收控制指令
def uart_control():
    """
    非阻塞读取 uart6 的所有可用数据，解析完整的控制指令帧。
    支持的帧格式：
        @A#a!   → 设置角度
        @V#x#y! → 设置速度
        @P#x#y! → 设置位置
        @G!     → 获取位置
        @S#x#y! → 设置坐标
        @D#x#y! → 设置走多远
    """
    global _uart6_buf
    data = _read_available(uart6)
    if data:
        _uart6_buf += data
        if len(_uart6_buf) > MAX_BUF_SIZE:
            _uart6_buf[:] = _uart6_buf[-MAX_BUF_SIZE:]
    _extract_frames(_uart6_buf, prase_command)


# 对外接口：uart_send_feedforward() — 向跟随车发送前馈数据
def uart_send_feedforward():
    """
    发送前馈指令给跟随车。
    帧格式：@VW#vx#vy#w!
    
    速度值定义在主车坐标系中，因两车车头同向，跟随车可直接使用。
    vx 包含环绕补偿：跟随车在主车左侧 FOLLOWER_OFFSET 米处，
    主车旋转时跟随车获得切向速度 -w * FOLLOWER_OFFSET。
    """
    vx_ff = car.target_vx
    vy_ff = car.target_vy
    w_ff = car.target_w
    
    # 环绕补偿
    # 跟随车在主车左侧 20cm，主车旋转时跟随车绕主车做圆周运动。
    vy_ff += w_ff * FOLLOWER_OFFSET
    
    # 角速度死区：过滤主控车自身微调角度的行为
    # 微小角度修正由跟随车摄像头反馈闭环处理，前馈仅负责大角度主动转向
    if abs(w_ff) < 0.2:
        w_ff = 0.0
    
    wireless_uart.write(f"@VW#{vx_ff:.2f}#{vy_ff:.2f}#{w_ff:.1f}!\n")


# 指令解析函数
def prase_command(cmd_str):
    """解析上位机控制指令（来自 uart6）"""
    if not (cmd_str.startswith('@') and cmd_str.endswith('!')):
        return
    content = cmd_str[1:-1]
    parts = content.split('#')
    cmd_type = parts[0]
    
    if cmd_type == 'A':
        if len(parts) < 2:
            return
        angle = float(parts[1])
        set_angle(angle)
    elif cmd_type == 'V':
        if len(parts) < 3:
            return
        vx = float(parts[1])
        vy = float(parts[2])
        set_speed(vx, vy)
    elif cmd_type == 'P':
        if len(parts) < 3:
            return
        x = float(parts[1])
        y = float(parts[2])
        set_position(x, y)
    elif cmd_type == 'G':
        px, py = get_position()
        uart6.write(f"@P#{px:.2f}#{py:.2f}!\n")
    elif cmd_type == 'S':
        if len(parts) < 3:
            return
        set_current_position(float(parts[1]), float(parts[2]))
        px, py = get_position()
    elif cmd_type == 'D':
        if len(parts) < 3:
            return
        x = float(parts[1])
        y = float(parts[2])
        px, py = get_position()
        set_position(px + x, py + y)


# 辅助函数
def set_angle(angle):
    car.target_angle = angle


def set_speed(vx, vy):
    car.set_car_speed(vx, vy)
    navigation.enable_navigation = False


def set_position(x, y):
    navigation.go_to_position_cnt = 0
    navigation.enable_navigation = True
    navigation.set_target_position(x, y)


def set_current_position(x, y):
    navigation.px = x
    navigation.py = y


def get_position():
    return navigation.px, navigation.py