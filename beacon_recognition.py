import sensor, image, time, math

# 识别配置
white_threshold = (150, 173)    # 提高阈值，只看最亮的部分
CONFIRM_FRAMES = 5              # 连续确认帧数

# 相机参数（用于 AprilTag 3D 定位）
f_x = (2.8 / 3.984) * 160
f_y = (2.8 / 2.952) * 120
c_x = 160 * 0.5
c_y = 120 * 0.5


def degrees(radians):
    return (180 * radians) / math.pi


def find_beacon(img, target_blob_out=None):
    """
    在给定图像中检测信标（单帧检测，用于决策循环）
    参数:
        img: sensor.snapshot() 获取的图像
        target_blob_out: 可选，用于传回 blob 对象的列表（仅供绘图）
    返回:
        (cx, cy, w, h) 如果找到信标
        None 如果未找到
    """
    blobs = img.find_blobs([white_threshold], pixels_threshold=150, area_threshold=100, merge=True)

    for blob in blobs:
        aspect_ratio = blob.w() / blob.h()
        if 0.65 < aspect_ratio < 1.35:
            if target_blob_out is not None:
                target_blob_out.append(blob)
            return (blob.cx(), blob.cy(), blob.w(), blob.h())

    return None


# ===== 以下是独立运行模式（含多帧确认 + AprilTag 定位）=====
if __name__ == "__main__":
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)  # 160x120 速度快
    sensor.skip_frames(time=2000)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    clock = time.clock()

    recognition_counter = 0

    while True:
        clock.tick()
        img = sensor.snapshot()

        # 寻找符合条件的色块
        blobs = img.find_blobs([white_threshold], pixels_threshold=150, area_threshold=100, merge=True)

        found_valid_blob = False
        target_blob = None

        for blob in blobs:
            aspect_ratio = blob.w() / blob.h()
            if 0.65 < aspect_ratio < 1.35:
                found_valid_blob = True
                target_blob = blob
                break

        # 连续帧确认逻辑
        if found_valid_blob:
            recognition_counter += 1
        else:
            recognition_counter = 0

        # 最终判定
        if recognition_counter >= CONFIRM_FRAMES:
            recognition_counter = CONFIRM_FRAMES
            img.draw_rectangle(target_blob.rect(), color=255, thickness=2)
            img.draw_string(target_blob.x(), target_blob.y() - 15, "BEACON CONFIRMED", color=255)
            print("信标识别锁定中...")
        else:
            if found_valid_blob:
                img.draw_rectangle(target_blob.rect(), color=150)
                img.draw_string(0, 0, "Wait... %d/%d" % (recognition_counter, CONFIRM_FRAMES))

        # AprilTag 3D 定位
        for tag in img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y):
            img.draw_rectangle(tag.rect(), color=(255, 0, 0))
            img.draw_cross(tag.cx(), tag.cy(), color=(0, 255, 0))
            print_args = (tag.x_translation(), tag.y_translation(), tag.z_translation(),
                          degrees(tag.x_rotation()), degrees(tag.y_rotation()), degrees(tag.z_rotation()))
            print("Tx: %f, Ty %f, Tz %f, Rx %f, Ry %f, Rz %f" % print_args)
