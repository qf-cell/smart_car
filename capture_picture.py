"""
capture_picture.py - 按空格键拍照并保存到 bear/ 文件夹
"""

import sensor
import image
import time
import os
import pyb

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# 保存目录 (SD 卡)
SAVE_DIR = "/sd/bear"

# 检测 SD 卡是否插入 (直接访问 /sd/)
try:
    os.listdir('/sd/')
except OSError as e:
    raise RuntimeError("未检测到 SD 卡！请插入 SD 卡后重试。错误: %s" % e)

# 确保目录存在
try:
    os.mkdir(SAVE_DIR)
except OSError:
    pass

# 续编号
existing = [f for f in os.listdir(SAVE_DIR) if f.endswith('.jpg')]
count = len(existing)

print("===== 拍照程序 =====")
print("按 空格键 拍照，保存到 %s/" % SAVE_DIR)
print("当前已有 %d 张图片" % count)

vcp = pyb.USB_VCP()
clock = time.clock()
last_save = 0
SAVE_INTERVAL_MS = 300

while True:
    clock.tick()
    img = sensor.snapshot()

    # 屏幕提示
    img.draw_string(0, 0, "Press SPACE", color=(0, 255, 0), scale=2)
    img.draw_string(0, 25, "Count: %d" % count, color=(255, 255, 0))
    img.draw_string(0, 50, "FPS: %.1f" % clock.fps(), color=(255, 255, 255))

    # 检测空格键
    if vcp.any():
        char = vcp.read(1)
        now = time.ticks_ms()
        if char == b' ' and time.ticks_diff(now, last_save) > SAVE_INTERVAL_MS:
            count += 1
            filename = "%s/img_%04d.jpg" % (SAVE_DIR, count)
            img.save(filename)
            print("[拍照] 保存: %s" % filename)
            last_save = now
            img.draw_string(0, 75, "SAVED!", color=(0, 255, 0), scale=3)

    time.sleep_ms(50)
