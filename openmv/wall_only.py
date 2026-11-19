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
WALL_PIX_TH = 120

HDR_GOALWALL = 0x02
HDR_WALL = 0x14

# 最新実測 L53-54/A-5--4/B-1-1（緑のフィールドを拾わない）
BlackThresholds = [(5, 30, -12, 0, -2, 10)]

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
