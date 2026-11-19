# main.py -- 黒い壁のみ検出して座標＋サイズを送信する最小スクリプト
#
# 参考: Yellow/Blue goals + single wall (coord + size) からゴール検出を省いた版。
# 送信: 0x14 [0x14, wx, wy, size_hi, size_lo, FF, FF]
#       0x02 [0x02, 255, 255, 255, 255, wx, wy]  （既存パーサ互換用）
# 壁は横長の帯のみを採用し、PCや影などの四角い黒領域を除外する。

import sensor
import image
import time
import math
import pyb
from pyb import UART

# ---------- 設定 ----------
STREAM_INTERVAL_MS = 200
WALL_PIX_TH = 150

HDR_GOALWALL = 0x02
HDR_WALL = 0x14

# 実測 L11/A-7/B5 を中心に、旧 L31-33 もカバー（緑フィールドは L40以上なので除外）
BlackThresholds = [(5, 32, -20, 5, -5, 15)]

uart = UART(3, 9600, timeout_char=1000)
uart.init(9600, bits=8, parity=None, stop=1, timeout_char=1000)


def blob_value(blob, name):
    """OpenMV v5 はプロパティ、旧版はメソッド。両対応。"""
    value = getattr(blob, name)
    return value() if callable(value) else value


def norm_coord(cx, cy, img):
    if cx is None or cy is None:
        return 255, 255
    return int((cx * 255) // img.width()), int((cy * 255) // img.height())


def make_packet(hdr, b1, b2, b3, b4, b5=0xFF, b6=0xFF):
    return bytearray([hdr & 0xFF, b1 & 0xFF, b2 & 0xFF, b3 & 0xFF,
                      b4 & 0xFF, b5 & 0xFF, b6 & 0xFF])


def uart_write(pkt):
    try:
        uart.write(pkt)
    except Exception:
        pass





def _clamp_corners(corners, width, height):
    """角を画像内に収める（画面外の線を消す）。"""
    return [(max(0, min(width - 1, int(x))), max(0, min(height - 1, int(y))))
            for x, y in corners]


def _order_corners(corners):
    """角を重心回りの角度順に並べ、線の交差を防ぐ。"""
    import math
    cx = sum(p[0] for p in corners) / float(len(corners))
    cy = sum(p[1] for p in corners) / float(len(corners))
    return sorted(corners, key=lambda p: math.atan2(p[1] - cy, p[0] - cx))


def draw_wall_outline(img, blob):
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
        corners = _clamp_corners(corners, img.width(), img.height())
        corners = _order_corners(corners)
        # v5: draw_edges(corners, color, size, thickness)
        try:
            img.draw_edges(corners, (0, 0, 255), 0, 2)
            return
        except Exception:
            pass
        # 旧版: draw_line(x0, y0, x1, y1, color, thickness)
        try:
            for i in range(len(corners)):
                p1 = corners[i]
                p2 = corners[(i + 1) % len(corners)]
                img.draw_line(p1[0], p1[1], p2[0], p2[1], (0, 0, 255), 2)
            return
        except Exception:
            pass
    # 最後の手段: 矩形
    try:
        img.draw_rectangle(blob_value(blob, "rect"), (0, 0, 255), 2)
    except Exception:
        pass


# ---------- 初期化 ----------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
try:
    sensor.set_framesize(sensor.B320X320)
except Exception:
    sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=300)
try:
    sensor.set_auto_exposure(True)
    sensor.set_auto_gain(True)
    sensor.set_auto_whitebal(False)
except Exception:
    pass
# 競技ルール: IR発光器と可視LEDはOFF
try:
    sensor.set_ir_led(False)
except Exception:
    pass
try:
    for led_id in (1, 2, 3):
        pyb.LED(led_id).off()
except Exception:
    pass

clock = time.clock()
last_stream_ms = pyb.millis()

while True:
    clock.tick()
    img = sensor.snapshot()

    # 壁は画面下2/3だけを探す（ルール3.1.4: 壁より上の色を無視）
    wall_roi = (0, img.height() // 4, img.width(), img.height() * 3 // 4)
    best_size = 0
    best_cx = None
    best_cy = None
    best_rect = None
    best_blob = None
    wall_blobs = []
    raw_blobs = list(img.find_blobs(BlackThresholds, roi=wall_roi,
                                    pixels_threshold=WALL_PIX_TH,
                                    area_threshold=WALL_PIX_TH,
                                    merge=True, margin=6))
    rejected = []
    for blob in raw_blobs:
        rect = blob_value(blob, "rect")
        if rect is None or len(rect) < 4:
            continue
        w, h = rect[2], rect[3]
        # 壁は横長の帯。四角い黒領域（PCや影）は除外する。
        if w < 30 or h < 4 or h > 150 or w < 1.5 * h:
            continue
        # 床の黒い物体を壁と誤検出しない（壁は床より上）。
        if rect[1] > wall_roi[1] + wall_roi[3] // 2:
            if len(rejected) < 3:
                rejected.append(rect)
            continue
        # 遮蔽で壁が分断されても、すべての断片を描画対象にする。
        wall_blobs.append(blob)
        size = blob_value(blob, "pixels")
        if size > best_size:
            best_size = size
            best_cx = blob_value(blob, "cx")
            best_cy = blob_value(blob, "cy")
            best_rect = rect
            best_blob = blob

    if best_size > 0 and best_cx is not None:
        wx, wy = norm_coord(best_cx, best_cy, img)
        wsize = best_size
        # 壁の形に沿った青い輪郭を描く（中心点は描かない）。
        # 人などに遮られて分断されても、すべての断片を描画する。
        for blob in wall_blobs:
            draw_wall_outline(img, blob)
    else:
        wx = wy = 255
        wsize = 0

    try:
        # raw=色閾値で拾った数 / rej=形状で弾いた rect / Wsize=採用した壁
        print("raw:{} rej:{} Wsize:{} rect:{} wx:{} wy:{}".format(
            len(raw_blobs), rejected, wsize, best_rect, wx, wy))
    except Exception:
        pass

    now = pyb.millis()
    if (now - last_stream_ms) >= STREAM_INTERVAL_MS:
        last_stream_ms = now
        uart_write(make_packet(HDR_GOALWALL, 255, 255, 255, 255, wx, wy))
        uart_write(make_packet(HDR_WALL, wx, wy,
                               (wsize >> 8) & 0xFF, wsize & 0xFF, 0xFF, 0xFF))
