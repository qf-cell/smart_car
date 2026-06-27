"""
下位机串口通信模块
协议: ASCII 帧 @...#...!  (对应下位机 uart.py 的 prase_command)

支持的指令:
  @A#angle!      → 旋转 (度)
  @P#x#y!        → 绝对移动 (米)
  @D#dx#dy#v!    → 相对位移 + 合速度 (米, 米/秒)
  @S#x#y!        → 设置当前坐标 (米, 重定位用)
  @G!            → 查询当前位置

内部单位: 厘米 → 发送时自动转换为米
"""

from machine import UART

# ── UART 配置 ──────────────────────────
UART_PORT = 2
UART_BAUD = 9600


def init_uart(port=UART_PORT, baud=UART_BAUD):
    """初始化 UART 串口"""
    uart = UART(port, baud)
    uart.init(baud, bits=8, parity=None, stop=1, timeout=100)
    return uart


def _to_m(cm):
    """厘米 → 米"""
    return cm / 100.0


# ══════════════════════════════════════════
#  发送函数 (输入均为厘米, 内部转米)
# ══════════════════════════════════════════

def send_rotate(uart, angle_deg):
    """
    旋转指令 → @A#angle!
    angle_deg: 负=顺时针, 正=逆时针
    """
    uart.write("@A#%.1f!\n" % angle_deg)


def send_move_to(uart, x_cm, y_cm):
    """
    移动到绝对坐标 → @P#x#y!
    x_cm, y_cm: 目标坐标 (厘米)
    """
    uart.write("@P#%.2f#%.2f!\n" % (_to_m(x_cm), _to_m(y_cm)))


def send_move_delta(uart, dx_cm, dy_cm, speed_cm_s=40):
    """
    相对位移 + 合速度 → @D#dx#dy#v!
    dx_cm, dy_cm: 位移量 (厘米)
    speed_cm_s:   合速度 (厘米/秒), 默认 40 = 0.4 m/s
    """
    uart.write("@D#%.2f#%.2f#%.2f!\n" %
               (_to_m(dx_cm), _to_m(dy_cm), _to_m(speed_cm_s)))


def send_set_position(uart, x_cm, y_cm):
    """
    强制设置下位机当前坐标 → @S#x#y!
    用于信标重定位后同步下位机导航坐标
    """
    uart.write("@S#%.2f#%.2f!\n" % (_to_m(x_cm), _to_m(y_cm)))


def send_speed(uart, vx, vy):
    """
    发送速度指令 → @V#vx#vy!
    vx, vy: 速度 (m/s), vx=vy=0 急停
    """
    uart.write("@V#%.2f#%.2f!\n" % (vx, vy))


def query_position(uart, timeout_ms=800):
    """
    查询下位机当前坐标 → 发送 @G!, 读取响应 @P#x#y!
    返回 (x_cm, y_cm) 或 None (超时/解析失败)
    """
    import time

    # 清空接收缓冲区
    flushed = 0
    while uart.any():
        uart.read(uart.any())
        flushed += 1

    uart.write("@G!\n")

    buf = b''
    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        if uart.any():
            buf += uart.read(uart.any())
            at_idx = buf.find(b'@P#')
            if at_idx >= 0:
                excl_idx = buf.find(b'!', at_idx)
                if excl_idx > at_idx:
                    frame = buf[at_idx + 3:excl_idx]  # skip "@P#"
                    parts = frame.split(b'#')
                    if len(parts) >= 2:
                        try:
                            x_cm = float(parts[0]) * 100
                            y_cm = float(parts[1]) * 100
                            return (x_cm, y_cm)
                        except Exception as e:
                            print("[UART] 解析失败: %s buf=%s" % (e, buf))
                            return None
        time.sleep_ms(10)

    print("[UART] 查询超时 %dms flushed=%d buf=%s" %
          (timeout_ms, flushed, buf[:80] if buf else b'(empty)'))
    return None
