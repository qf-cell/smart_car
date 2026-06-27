import sensor, image, time

# 网球的 LAB 阈值 (需要根据实际环境调整)
# 格式: (L_min, L_max, A_min, A_max, B_min, B_max)
tennis_threshold = (7, 96, -115, -9, 18, 95)

def find_tennis(img):
    """
    在给定图像中检测网球
    参数:
        img: sensor.snapshot() 获取的图像
    返回:
        (cx, cy, roundness, w, h) 如果找到网球
        None 如果未找到
    """
    blobs = img.find_blobs([tennis_threshold], pixels_threshold=200, area_threshold=140, merge=True)

    for b in blobs:
        # 形状过滤：网球是圆的，圆度应该接近 1
        if b.roundness() > 0.5:
            return (b.cx(), b.cy(), b.roundness(), b.w(), b.h())

    return None


# ===== 以下是独立运行模式 =====
if __name__ == "__main__":
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565) # 使用彩色模式
    sensor.set_framesize(sensor.QVGA)    # 320x240 分辨率
    sensor.skip_frames(time = 2000)     # 等待感光元件稳定
    sensor.set_auto_gain(False)          # 颜色识别必须关闭自动增益
    sensor.set_auto_whitebal(False)      # 颜色识别必须关闭自动白平衡
    clock = time.clock()

    while(True):
        clock.tick()
        img = sensor.snapshot()

        # 1. 寻找色块
        # pixels_threshold: 过滤掉太小的杂点
        # area_threshold: 过滤掉面积太小的
        # merge=True: 如果多个色块靠在一起，合并它们
        blobs = img.find_blobs([tennis_threshold], pixels_threshold=200, area_threshold=140, merge=True)

        for b in blobs:
            # 2. 形状过滤：网球是圆的，圆度应该接近 1
            if b.roundness() > 0.5:

                # 在图像上画框和十字
                img.draw_rectangle(b.rect())
                img.draw_cross(b.cx(), b.cy())

                # 打印结果
                print("找到网球! 中心坐标: x=%d, y=%d" % (b.cx(), b.cy()))

        print(clock.fps()) # 显示帧率
