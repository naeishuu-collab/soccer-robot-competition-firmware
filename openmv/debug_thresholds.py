# 一時的な閾値調整用デバッグスクリプト
#
# 使い方:
#   1. このファイルを OpenMV に main.py として書き込む
#   2. IDE のターミナルに「色名 / 面積 / 矩形 / 中心 / LAB平均」が出る
#   3. 数値を調整して TH に反映し、さらに Threshold Editor と突き合わせる
#   4. 調整が終わったら camera_only_display.py / goal_wall_navigation.py に戻す
#
# 明るさ・解像度の設定は camera_only_display.py と完全に同じにしている。
# （露出/ゲイン=オート、白色バランス=OFF、解像度は B320X320 優先）

import sensor
import image
import time
import pyb

# ---------- カメラ設定（camera_only_display.py と同じ） ----------
FRAMESIZE_ORDER = ("B320X320", "B640X480", "QVGA", "QQVGA")
PRINT_INTERVAL_MS = 500

# ---------- 現在の閾値（ここを書き換えて調整する） ----------
TH = {
    "BALL":   [(53, 78, 11, 45, 40, 82)],
    "YELLOW": [(68, 80, -5, 6, 58, 72)],
    "BLUE":   [(39, 42, 3, 7, -45, -41), (19, 25, 13, 20, -46, -40)],
    "BLACK":  [(5, 30, -12, 0, -2, 10)],
    "RED":    [(30, 90, 140, 255, 110, 200)],
}

COLORS = {
    "BALL":   (255, 170, 0),
    "YELLOW": (255, 255, 0),
    "BLUE":   (0, 255, 255),
    "BLACK":  (255, 80, 80),
    "RED":    (255, 0, 255),
}


def draw_blob_outline(img, blob, color):
    """blob.corners（輪郭由来の4角）を draw_edges で描く。中心点は描かない。
    OpenMV v5 は convex_hull() が無く draw_line も座標タプル必須のため、
    旧版の draw_line と rect フォールバックも用意する。"""
    if blob is None:
        return
    corners = None
    for attr in ("corners", "min_corners"):
        try:
            candidate = blob_value(blob, attr)
            if candidate and len(candidate) >= 4:
                corners = candidate
                break
        except Exception:
            continue
    if corners is None:
        try:
            rect = blob_value(blob, "rect")
            if rect and len(rect) >= 4:
                x, y, w, h = rect
                corners = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
        except Exception:
            return
    if corners is not None:
        # v5: draw_edges(corners, color, size, thickness)
        try:
            img.draw_edges(corners, color, 0, 2)
            return
        except Exception:
            pass
        # 旧版: draw_line(x0, y0, x1, y1, color, thickness)
        try:
            for i in range(len(corners)):
                p1 = corners[i]
                p2 = corners[(i + 1) % len(corners)]
                img.draw_line(p1[0], p1[1], p2[0], p2[1], color, 2)
            return
        except Exception:
            pass
    # 最後の手段: 矩形
    try:
        img.draw_rectangle(blob_value(blob, "rect"), color, 2)
    except Exception:
        pass

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
    # camera_only_display.py と同じ順で解像度を試す
    for size in FRAMESIZE_ORDER:
        try:
            sensor.set_framesize(getattr(sensor, size))
            break
        except Exception:
            continue
    sensor.skip_frames(time=300)
    # camera_only_display.py と同じ明るさ設定
    try:
        sensor.set_auto_exposure(True)
    except Exception:
        pass
    try:
        sensor.set_auto_gain(True)
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
        # 壁は画面下2/3だけを探す（背景の黒い棚などの誤検出を減らす）
        wall_roi = (0, img.height() // 3, img.width(), img.height() * 2 // 3)
        for name, thresholds in TH.items():
            count = 0
            roi = wall_roi if name == "BLACK" else None
            min_px = 120 if name == "BLACK" else 20
            for blob in img.find_blobs(thresholds, roi=roi,
                                       pixels_threshold=min_px,
                                       area_threshold=min_px, merge=True, margin=6):
                rect = blob_value(blob, "rect")
                # 壁は横長の帯だけを表示（PCや影の黒領域を除外）
                if name == "BLACK" and rect is not None and len(rect) >= 4:
                    if rect[2] < 25 or rect[3] < 3 or rect[3] > 150 or rect[2] < 1.3 * rect[3]:
                        continue
                count += 1
                px = blob_value(blob, "pixels")
                cx = blob_value(blob, "cx")
                cy = blob_value(blob, "cy")
                mean = lab_mean(img, rect)
                lines.append("{} px={} rect={} cxcy=({},{}) lab={}".format(
                    name, px, rect, cx, cy, mean))
                draw_blob_outline(img, blob, COLORS[name])
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
