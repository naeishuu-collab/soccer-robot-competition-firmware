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
WALL_PIX_TH = 300

HDR_GOALWALL = 0x02
HDR_WALL = 0x14

# 実機計測 L31-33 / A-6--4 / B-1-2 に少し余裕を持たせた値
BlackThresholds = [(27, 40, -11, 1, -6, 7)]

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
# 競技ルール: IR/可視LEDはOFF
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
    wall_roi = (0, img.height() // 3, img.width(), img.height() * 2 // 3)
    best_size = 0
    best_cx = None
    best_cy = None
    best_rect = None
    for blob in img.find_blobs(BlackThresholds, roi=wall_roi,
                               pixels_threshold=WALL_PIX_TH,
                               area_threshold=WALL_PIX_TH,
                               merge=True, margin=6):
        rect = blob_value(blob, "rect")
        if rect is None or len(rect) < 4:
            continue
        w, h = rect[2], rect[3]
        # 壁は横長の帯。四角い黒領域（PCや影）は除外する。
        if w < 40 or h < 4 or w < 2.0 * h:
            continue
        size = blob_value(blob, "pixels")
        if size > best_size:
            best_size = size
            best_cx = blob_value(blob, "cx")
            best_cy = blob_value(blob, "cy")
            best_rect = rect

    if best_size > 0 and best_cx is not None:
        wx, wy = norm_coord(best_cx, best_cy, img)
        wsize = best_size
        try:
            img.draw_rectangle(best_rect, (255, 255, 255), 1)
        except Exception:
            pass
    else:
        wx = wy = 255
        wsize = 0

    try:
        print("Wsize:{} rect:{} wx:{} wy:{}".format(wsize, best_rect, wx, wy))
    except Exception:
        pass

    now = pyb.millis()
    if (now - last_stream_ms) >= STREAM_INTERVAL_MS:
        last_stream_ms = now
        uart_write(make_packet(HDR_GOALWALL, 255, 255, 255, 255, wx, wy))
        uart_write(make_packet(HDR_WALL, wx, wy,
                               (wsize >> 8) & 0xFF, wsize & 0xFF, 0xFF, 0xFF))
