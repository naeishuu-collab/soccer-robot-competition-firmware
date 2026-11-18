# OpenMV: wall avoidance + goal shooting
# ------------------------------------------------------------
# 目的:
#   - 黒い壁が正面近くにあるときは後退＋旋回で回避する
#   - 狙うゴールが正面にあるときはシュート要求を出す
#
# MAIN との通信は仮定。既存 main.py の 0x02 / 0x14 に加え、
# ナビゲーション指令 0x20 を送る。MAIN 側の受信実装はまだない。
#
#   0x02: [0x02, yx, yy, bx, by, wx, wy]         既存互換
#   0x14: [0x14, wx, wy, size_hi, size_lo, FF, FF] 既存互換
#   0x20: [0x20, vx, vy, turn, kick, mode, xor]
#         vx/vy/turn = -100..100 を 8bit 符号付きで送信
#                      (MAIN の Motion は /100 して -1..1 に変換する想定)
#         kick  = 0/1 (要求の立ち上がりだけを送る)
#         mode  = 1:AVOID 2:SEARCH 3:APPROACH 4:SHOOT
#         xor   = 先頭 6 バイトの排他的論理和
#
# 注意: ボールの有無は判定しない。キックの最終ゲートは
#       MAIN/IR 側 (ballDetected など) に任せる。
# ------------------------------------------------------------

try:
    import sensor
    import image
    import time
    import pyb
    from pyb import UART
    OPENMV = True
except ImportError:
    # ホスト側のロジックテスト用 (OpenMV 固有モジュールは使わない)
    OPENMV = False
    sensor = None
    image = None
    time = None
    pyb = None
    UART = None

import math

# ---------- 設定 ----------
TARGET_GOAL = "YELLOW"          # "YELLOW" / "BLUE"
PIXELS_THRESHOLD = 20
WALL_PIX_TH = 70
STREAM_INTERVAL_MS = 100

WALL_AVOID_ANGLE_DEG = 60.0     # この範囲内の壁を「正面の壁」とみなす
WALL_NEAR_PIX = 250             # これより大きい黒 blob は「近い壁」
GOAL_SHOOT_ANGLE_DEG = 15.0     # この範囲内ならシュート可能
GOAL_NEAR_PIX = 60              # これより大きいゴール blob は「近い」

KICK_COOLDOWN_MS = 1500         # 送信側でも再発射を抑制する

HDR_GOALWALL = 0x02
HDR_WALL = 0x14
HDR_NAV = 0x20

MODE_AVOID = 1
MODE_SEARCH = 2
MODE_APPROACH = 3
MODE_SHOOT = 4

YellowThresholds = [(64, 91, -6, 14, 26, 62)]
BlueThresholds = [(85, 0, 68, -128, -48, -17)]
BlackThresholds = [(91, 52, 61, 8, 20, -10)]

GoalRoi = (30, 30, 260, 260)


# ---------- 純粋ロジック (ホストでもテスト可能) ----------
def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def norm_coord(cx, cy, img):
    if cx is None or cy is None:
        return 255, 255
    nx = int((cx * 255) // img.width())
    ny = int((cy * 255) // img.height())
    return nx, ny


def angle_from_normalized(nx, ny):
    """画面上方向 = 0°、時計回り。未検出は None。"""
    if nx == 255 or ny == 255:
        return None
    dx = float(nx) - 128.0
    dy = float(ny) - 128.0
    rad = math.atan2(dy, dx) + math.pi / 2.0
    deg = rad * 180.0 / math.pi
    return deg % 360.0


def angle_error(target_deg, current_deg):
    """current から target への最短符号付き誤差 (-180, 180]。正は時計回り。"""
    err = (target_deg - current_deg + 540.0) % 360.0 - 180.0
    return err


def decide_navigation(yx, yy, bx, by, wx, wy, wsize, goal_sizes):
    """ detections -> (vx, vy, turn, want_kick, mode_code, mode_name)
        goal_sizes = (yellow_pixels, blue_pixels)
        vx/vy/turn は -100..100、turn は CCW 正 (MAIN の Motion と同じ)。"""
    if TARGET_GOAL == "BLUE":
        gx, gy, gsize = bx, by, goal_sizes[1]
    else:
        gx, gy, gsize = yx, yy, goal_sizes[0]

    # --- 壁回避: 正面近くの壁を最優先 ---
    if wx != 255 and wy != 255:
        wall_angle = angle_from_normalized(wx, wy)
        if wall_angle is not None:
            wall_err = angle_error(wall_angle, 0.0)
            if abs(wall_err) <= WALL_AVOID_ANGLE_DEG and wsize >= WALL_NEAR_PIX:
                # 後退しつつ、壁と逆方向へ旋回 (MAIN の turn は CCW 正)
                turn = int(35.0 * (1.0 if wall_err >= 0.0 else -1.0))
                return 0, -50, turn, 0, MODE_AVOID, "AVOID"

    # --- ゴールが見えない: ゆっくり探索 ---
    if gx == 255 or gy == 255:
        return 0, 10, 20, 0, MODE_SEARCH, "SEARCH"

    # --- ゴールへ接近 / シュート ---
    goal_angle = angle_from_normalized(gx, gy)
    err = angle_error(goal_angle, 0.0)
    # ゴールが右 (err>0) なら時計回り = MAIN では turn 負
    turn = int(clamp(-err * 0.6, -60.0, 60.0))
    vy = 20 if abs(err) > 45.0 else 55
    if abs(err) <= GOAL_SHOOT_ANGLE_DEG and gsize >= GOAL_NEAR_PIX:
        return 0, vy, turn, 1, MODE_SHOOT, "SHOOT"
    return 0, vy, turn, 0, MODE_APPROACH, "APPROACH"


def make_nav_packet(vx, vy, turn, kick, mode):
    body = [
        HDR_NAV,
        int(clamp(vx, -100, 100)) & 0xFF,
        int(clamp(vy, -100, 100)) & 0xFF,
        int(clamp(turn, -100, 100)) & 0xFF,
        kick & 0x01,
        mode & 0xFF,
    ]
    checksum = 0
    for value in body:
        checksum ^= value
    return bytearray(body + [checksum])


def make_legacy_packets(yx, yy, bx, by, wx, wy, wsize):
    goal = bytearray([HDR_GOALWALL, yx, yy, bx, by, wx, wy])
    wall = bytearray([HDR_WALL, wx, wy, (wsize >> 8) & 0xFF, wsize & 0xFF, 0xFF, 0xFF])
    return goal, wall


# ---------- OpenMV 固有 ----------
def init_camera():
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


def biggest_blob(img, thresholds, roi, pixel_threshold):
    best = 0
    cx = None
    cy = None
    for blob in img.find_blobs(thresholds, roi=roi,
                               pixels_threshold=pixel_threshold,
                               area_threshold=pixel_threshold,
                               merge=True, margin=6):
        if blob.pixels() > best:
            best = blob.pixels()
            cx = blob.cx()
            cy = blob.cy()
    return best, cx, cy


def main_loop():
    init_camera()
    uart = UART(3, 9600, timeout_char=1000)
    uart.init(9600, bits=8, parity=None, stop=1, timeout_char=1000)
    clock = time.clock()
    last_stream_ms = pyb.millis()
    last_kick_ms = -KICK_COOLDOWN_MS

    while True:
        clock.tick()
        img = sensor.snapshot()

        ysize, ycx, ycy = biggest_blob(img, YellowThresholds, GoalRoi, PIXELS_THRESHOLD)
        bsize, bcx, bcy = biggest_blob(img, BlueThresholds, GoalRoi, PIXELS_THRESHOLD)
        wsize, wcx, wcy = biggest_blob(img, BlackThresholds, None, WALL_PIX_TH)

        yx, yy = norm_coord(ycx, ycy, img)
        bx, by = norm_coord(bcx, bcy, img)
        wx, wy = norm_coord(wcx, wcy, img)

        now = pyb.millis()
        vx, vy, turn, want_kick, mode, mode_name = decide_navigation(
            yx, yy, bx, by, wx, wy, wsize, (ysize, bsize))

        # 送信側でもキックの立ち上がりとクールダウンを保証する
        kick = 0
        if want_kick and (now - last_kick_ms) >= KICK_COOLDOWN_MS:
            kick = 1
            last_kick_ms = now

        try:
            print("NAV {} vx{} vy{} turn{} kick{} wall{}".format(
                mode_name, vx, vy, turn, kick, wsize))
        except Exception:
            pass

        if (now - last_stream_ms) >= STREAM_INTERVAL_MS:
            last_stream_ms = now
            goal_pkt, wall_pkt = make_legacy_packets(yx, yy, bx, by, wx, wy, wsize)
            uart.write(goal_pkt)
            uart.write(wall_pkt)
            uart.write(make_nav_packet(vx, vy, turn, kick, mode))


if __name__ == "__main__":
    main_loop()
