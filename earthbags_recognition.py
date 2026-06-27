import sensor, image, time

# 不区分颜色, 只靠形状识别沙包 (正方形 + 高矩形度)
# 灰度阈值: 排除极暗(地面)和极亮(过曝)后取中间
gray_threshold = (20, 90)   # L 范围, 覆盖红/蓝沙包常见亮度


def find_earthbag(img):
    """
    基于轮廓形状检测沙包, 不区分红/蓝色。
    返回: ("Sandbag", cx, cy, w, h) 或 None
    """
    blobs = img.find_blobs([gray_threshold], pixels_threshold=50,
                           area_threshold=50, merge=True)

    for b in blobs:
        # 0. 忽略地面 (图像下方 40%)
        if b.cy() > img.height() * 1:
            continue

        # 1. 正方形: 长宽比 0.7~1.3
        aspect = b.w() / b.h()
        if aspect < 0.7 or aspect > 1.3:
            continue

        # 2. 高矩形度: 物体填满包围盒
        rectness = b.pixels() / (b.w() * b.h())
        if rectness < 0.6:
            continue

        return ("Sandbag", b.cx(), b.cy(), b.w(), b.h())

    return None


if __name__ == "__main__":
    sensor.reset()
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=2000)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    clock = time.clock()

    while True:
        clock.tick()
        img = sensor.snapshot()

        result = find_earthbag(img)

        if result is not None:
            _, cx, cy, w, h = result
            img.draw_rectangle((cx - w // 2, cy - h // 2, w, h))
            img.draw_cross(cx, cy)
            img.draw_string(cx - 30, cy - 30, "BAG", scale=2)

            print("找到沙包! x:%d y:%d w:%d h:%d" % (cx, cy, w, h))

        print("FPS: %f" % clock.fps())
