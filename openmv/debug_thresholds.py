# 一時的な閾値調整用デバッグスクリプト
#
# 使い方:
#   1. このファイルを OpenMV に main.py として書き込む
#   2. IDE のターミナルに「色名 / 面積 / 矩形 / 中心 / LAB平均」が出る
#   3. 数値を調整して TH に反映し、さらに Threshold Editor と突き合わせる
#   4. 調整が終わったら camera_only_display.py / goal_wall_navigation.py に戻す
#
# 露出・ゲインは固定する。オートだと同じ物体でもLAB値が変わって閾値が壊れる。

import sensor
import image
import time
import pyb

# ---------- カメラ設定（環境に合わせて調整） ----------
EXPOSURE_US = 10000
GAIN_DB = 8.0
FRAMESIZE = sensor.QVGA
PRINT_INTERVAL_MS = 500

# ---------- 現在の閾値（ここを書き換えて調整する） ----------
TH = {
    "BALL":   [(60, 95, 15, 80, 40, 120)],
    "YELLOW": [(64, 91, -6, 14, 26, 62)],
    "BLUE":   [(85, 0, 68, -128, -48, -17)],
    "BLACK":  [(91, 52, 61, 8, 20, -10)],
    "RED":    [(30, 90, 140, 255, 110, 200)],
}

COLORS = {
    "BALL":   (255, 170, 0),
    "YELLOW": (255, 255, 0),
    "BLUE":   (0, 255, 255),
    "BLACK":  (255, 80, 80),
    "RED":    (255, 0, 255),
}


def blob_value(blob, name):
    """OpenMV v5 はプロパティ、旧版はメソッド。両対応。"""
    value = getattr(blob, name)
    return value() if callable(value) else value


def safe_text(img, x, y, text, color):
    for call in (
        lambda: img.draw_string(x, y, text, color, 1),
        lambda: img.draw_string((x, y), text, color, 1),
        lambda: img.draw_string(x, y, text, color),
        lambda: img.draw_string(x, y, text),
    ):
        try:
            call()
            return
        except Exception:
            continue


def lab_mean(img, rect):
    """矩形内のLAB平均を返す（閾値の妥当性確認用）。"""
    if rect is None:
        return None
    try:
        st = img.get_statistics(roi=rect)
        return (int(st.l_mean()), int(st.a_mean()), int(st.b_mean()))
    except Exception:
        return None


def init():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(FRAMESIZE)
    sensor.skip_frames(time=300)
    try:
        sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
    except Exception:
        pass
    try:
        sensor.set_auto_gain(False, gain_db=GAIN_DB)
    except Exception:
        pass
    try:
        sensor.set_auto_whitebal(False)
    except Exception:
        pass
    # 競技ルール: IR/可視LEDはOFF
    try:
        sensor.set_ir_led(False)
    except Exception:
        pass
    try:
        for i in (1, 2, 3):
            pyb.LED(i).off()
    except Exception:
        pass


def main():
    init()
    clock = time.clock()
    last_print = pyb.millis()
    while True:
        clock.tick()
        img = sensor.snapshot()
        lines = []
        for name, thresholds in TH.items():
            count = 0
            for blob in img.find_blobs(thresholds, pixels_threshold=20,
                                       area_threshold=20, merge=True, margin=6):
                count += 1
                px = blob_value(blob, "pixels")
                cx = blob_value(blob, "cx")
                cy = blob_value(blob, "cy")
                rect = blob_value(blob, "rect")
                mean = lab_mean(img, rect)
                lines.append("{} px={} rect={} cxcy=({},{}) lab={}".format(
                    name, px, rect, cx, cy, mean))
                try:
                    img.draw_rectangle(rect, COLORS[name], 1)
                except Exception:
                    pass
                safe_text(img, cx, cy, name, COLORS[name])
            if count == 0:
                lines.append("{}: NONE".format(name))
        now = pyb.millis()
        if (now - last_print) >= PRINT_INTERVAL_MS:
            last_print = now
            print("--- fps {:.1f} ---".format(clock.fps()))
            for line in lines:
                print(line)


if __name__ == "__main__":
    main()
