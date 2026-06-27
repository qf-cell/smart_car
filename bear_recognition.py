"""
小熊识别模块 — 星瞳科技 AI 识别 (tflite)
逐飞 OpenART mini 版（使用 tf.detect 高级 API，无需手动 resize）

两种模式:
  1. 数据集采集模式 (DATA_COLLECT = True):  按 key 保存截图, 用于训练
  2. AI 推理模式   (DATA_COLLECT = False): 加载 tflite 模型进行实时识别

使用方法:
  1. 采集数据集: DATA_COLLECT = True, 运行此文件, 对准小熊按 key 保存
  2. 训练模型:   将采集的图片上传到星瞳科技训练平台, 导出 bear_model.tflite
                 注意: 必须选择"带后处理"导出, 否则 tf.detect 不可用
  3. 部署推理:   将 bear_model.tflite 放入 OpenMV 闪存或 SD 卡, DATA_COLLECT = False
"""

import sensor
import image
import time
import tf

# ═══════════════════════════════════════════
#  配置
# ═══════════════════════════════════════════

DATA_COLLECT = False
MODEL_PATH = "bear_model.tflite"          # 模型文件路径 (闪存根目录或 /sd/ 下)
SCORE_THRESHOLD = 0.10                     # 置信度阈值

# 数据集采集参数
SAVE_DIR = "bear_dataset"
SAVE_INTERVAL_MS = 300

# ═══════════════════════════════════════════
#  模型加载
# ═══════════════════════════════════════════

_net = None
_loaded_once = False   # True=曾经加载成功过; 若之后 _net 变 None 说明被GC回收


def init_bear():
    """预加载熊模型（初始化阶段调用, 内存干净时一次完成）"""
    net = _load_model()
    if net is not None:
        print("[小熊] 模型预加载成功")
        return True
    else:
        print("[小熊] 模型预加载失败，步骤3将被跳过")
        return False


def _load_model():
    """
    加载 tflite 模型。
    - 首次调用正常加载
    - 加载成功后若 _net 被GC回收(变None), 不再重试, 返回None
    - 加载失败后永不再试
    """
    global _net, _loaded_once
    if _net is not None:
        return _net
    if _net is False:
        return None
    # _net is None
    if _loaded_once:
        # 曾经加载成功过, 现在变 None = 被GC回收了
        # 此时内存已碎片化, 重试大概率OOM硬错误, 放弃
        return None
    # 首次加载
    try:
        _net = tf.load(MODEL_PATH)
        _loaded_once = True
        print("[小熊] 模型加载成功: %s" % MODEL_PATH)
    except Exception as e:
        print("[小熊] 模型加载失败: %s" % e)
        _net = False   # 标记失败, 永不再试
    return _net if _net is not False else None


# ═══════════════════════════════════════════
#  检测入口 — 使用 tf.detect 高级 API
# ═══════════════════════════════════════════

def find_bear(img):
    """
    检测小熊, 返回 (cx, cy, w, h) 或 None。
    模型只在 init_bear() 中加载一次, 此处不会触发重新加载。
    """
    net = _load_model()
    if net is None:
        return None

    try:
        import gc
        print("[小熊] >>> 进入 tf.detect (mem_free=%d, img=%dx%d)" %
              (gc.mem_free(), img.width(), img.height()))
        objects = tf.detect(net, img)
        print("[小熊] <<< tf.detect 返回 %d 个候选" %
              (len(objects) if objects else 0))
    except Exception as e:
        print("[小熊] tf.detect 异常: %s" % e)
        return None
    except:  # 兜底, 有些硬错误 MicroPython 也能拦
        print("[小熊] tf.detect 未知异常")
        return None

    if objects:
        for obj in objects:
            x1, y1, x2, y2, label, score = obj
            if score > SCORE_THRESHOLD:
                # 归一化坐标 → 像素坐标 (参考例程, x1 需 -0.1 偏移修正)
                iw = img.width()
                ih = img.height()
                # 先算归一化宽高 (偏移前), 再各自映射 — 否则 w 会偏大
                w_norm = x2 - x1
                h_norm = y2 - y1
                x1_px = int((x1 - 0.1) * iw)
                y1_px = int(y1 * ih)
                w_px = int(w_norm * iw)
                h_px = int(h_norm * ih)
                # box → 中心坐标 + 宽高
                cx = x1_px + w_px // 2
                cy = y1_px + h_px // 2
                print("[小熊] 检测到! 置信度:%.2f 坐标(%d,%d) 尺寸(%d,%d)" %
                      (score, cx, cy, w_px, h_px))
                return (cx, cy, w_px, h_px)

    return None


# ═══════════════════════════════════════════
#  数据集采集模式
# ═══════════════════════════════════════════

def _data_collection():
    """
    数据集采集主循环。
    对准小熊后, 按板载按钮 (P0) 保存当前帧。
    保存格式: bear_dataset/img_0001.jpg, img_0002.jpg ...
    """
    import os
    import pyb

    # 确保目录存在
    try:
        os.mkdir(SAVE_DIR)
    except OSError:
        pass

    # 续编号
    existing = [f for f in os.listdir(SAVE_DIR) if f.endswith('.jpg')]
    count = len(existing)

    print("===== 小熊数据集采集模式 =====")
    print("对准小熊, 按 P0 按钮保存截图")
    print("当前已有 %d 张图片" % count)

    last_save = 0
    while True:
        img = sensor.snapshot()

        # 提示信息
        img.draw_string(0, 0, "DATA COLLECT", color=(255, 0, 0), scale=2)
        img.draw_string(0, 20, "Count: %d" % count, color=(255, 255, 0))
        img.draw_string(0, 40, "Press P0 to save", color=(0, 255, 0))

        now = time.ticks_ms()
        if pyb.Pin("P0", pyb.Pin.IN, pyb.Pin.PULL_UP).value() == 0:
            if time.ticks_diff(now, last_save) > SAVE_INTERVAL_MS:
                count += 1
                filename = "%s/img_%04d.jpg" % (SAVE_DIR, count)
                img.save(filename)
                print("[采集] 保存: %s" % filename)
                last_save = now
                img.draw_string(0, 60, "SAVED!", color=(0, 255, 0), scale=3)

        time.sleep_ms(50)


# ═══════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════

if __name__ == "__main__":
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=2000)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)

    if DATA_COLLECT:
        _data_collection()
    else:
        print("===== 小熊 AI 识别测试 =====")
        clock = time.clock()
        while True:
            clock.tick()
            img = sensor.snapshot()
            result = find_bear(img)
            if result is not None:
                cx, cy, w, h = result
                img.draw_rectangle((cx - w // 2, cy - h // 2, w, h),
                                   color=(255, 0, 0))
                img.draw_cross(cx, cy, color=(255, 0, 0))
                img.draw_string(cx - 30, cy - 30, "BEAR", color=(255, 0, 0))
            print("FPS: %.1f" % clock.fps())
