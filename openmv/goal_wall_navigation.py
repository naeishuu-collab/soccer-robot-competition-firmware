# OpenMV H7: wall distance -> virtual line + goal shooting
# ------------------------------------------------------------
# 大会: RCJ 関東ブロック / サッカーインフラレッド (2026ルール適用)
# 競技台 (Appendix A):
#   プレイングフィールド 158cm x 219cm (白線含む)
#   外エリア 12cm / 全体 182cm x 243cm
#   壁 高さ22cm マットブラック
#   ゴール 幅60cm 高さ10cm 奥行7.4cm (黄/青)
#   ペナルティエリア 25cm(奥行) x 80cm(幅)
#   センターサークル 直径60cm / ボール 42mm IRオレンジ
#
# 重要ルール 6.2.B.1 (赤外線リーグ):
#   ToF / LiDAR / IR距離センサー等 IRを発する距離計は禁止。
#   本スクリプトは OpenMV (可視光) の黒い壁blobサイズから距離を推定するため合法。
#
# MAIN との通信: UART3 (OpenMV) <-> MAIN Serial2 (GP16/17, 9600bps)
#   0x02: [0x02, yx, yy, bx, by, wx, wy]           既存互換
#   0x14: [0x14, wx, wy, size_hi, size_lo, FF, FF] 既存互換
#   0x20: [0x20, vx, vy, turn, kick, mode, flags, xor]
#         vx/vy/turn = -100..100 符号付き (MAIN は /100 して Motion -1..1)
#         kick  = 0/1
#         mode  = 1:AVOID 2:SEARCH 3:APPROACH 4:SHOOT
#         flags = bit0: 仮想ライン(白線)接近, bit1: ペナルティエリア接近
#         xor   = 先頭7バイトの排他的論理和
# ------------------------------------------------------------

try:
    import sensor
    import image
    import time
    import pyb
    from pyb import UART
    OPENMV = True
except ImportError:
    OPENMV = False
    sensor = None
    image = None
    time = None
    pyb = None
    UART = None

import math

# ---------- 競技台定数 (cm) ----------
FIELD_W_CM = 158.0           # 短辺 (白線含む)
FIELD_L_CM = 219.0           # 長辺
OUTER_CM = 12.0              # 白線外のアウトエリア
WALL_H_CM = 22.0
PENALTY_DEPTH_CM = 25.0
PENALTY_WIDTH_CM = 80.0
GOAL_W_CM = 60.0
CENTER_CIRCLE_CM = 60.0
VIRTUAL_LINE_MARGIN_CM = 8.0  # 白線からこの距離以内で「ライン上」とみなす

# ---------- 検出設定 ----------
PIXELS_THRESHOLD = 20
WALL_PIX_TH = 70
STREAM_INTERVAL_MS = 100

WALL_AVOID_ANGLE_DEG = 60.0
WALL_AVOID_DIST_CM = 35.0     # 壁がこの距離より近ければ回避
GOAL_SHOOT_ANGLE_DEG = 15.0
GOAL_NEAR_PIX = 60

# 壁blobサイズ -> 距離の較正 (OpenMV H7 + 標準レンズ用の初期値)
# 実機で 30cm と 60cm の壁を撮って WALL_REF_PIX@30CM を調整すること。
WALL_REF_PIX = 6000
WALL_REF_DIST_CM = 30.0

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


# ---------- 純粋ロジック ----------
def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def norm_coord(cx, cy, img):
    if cx is None or cy is None:
        return 255, 255
    return int((cx * 255) // img.width()), int((cy * 255) // img.height())


def angle_from_normalized(nx, ny):
    if nx == 255 or ny == 255:
        return None
    dx = float(nx) - 128.0
    dy = float(ny) - 128.0
    rad = math.atan2(dy, dx) + math.pi / 2.0
    return (rad * 180.0 / math.pi) % 360.0


def angle_error(target_deg, current_deg):
    return (target_deg - current_deg + 540.0) % 360.0 - 180.0


def wall_distance_cm(wsize):
    """黒い壁のblob.pixels()から距離[cm]を推定する (逆比例モデル)。"""
    if wsize is None or wsize <= 0:
        return None
    return WALL_REF_DIST_CM * float(WALL_REF_PIX) / float(wsize)


def estimate_virtual_line(wx, wy, wsize):
    """壁距離から白線(プレイングフィールド境界)とペナルティエリア線を推定する。
       戻り値: (line_detected, penalty_detected, line_angle_deg, dist_cm)
       line_angle は白線の方向 (中心から見た壁方向)。"""
    if wx == 255 or wy == 255 or wsize <= 0:
        return False, False, None, None
    angle = angle_from_normalized(wx, wy)
    if angle is None:
        return False, False, None, None
    dist_wall = wall_distance_cm(wsize)
    if dist_wall is None:
        return False, False, None, None
    # 白線は壁から OUTER_CM 内側にある
    dist_line = dist_wall - OUTER_CM
    line_detected = dist_line <= VIRTUAL_LINE_MARGIN_CM
    penalty_detected = dist_wall <= (OUTER_CM + PENALTY_DEPTH_CM)
    return line_detected, penalty_detected, angle, dist_line


def decide_navigation(yx, yy, bx, by, wx, wy, wsize, goal_sizes):
    """detections -> (vx, vy, turn, want_kick, mode_code, mode_name, flags)"""
    gx, gy, gsize = (bx, by, goal_sizes[1]) if TARGET_GOAL == "BLUE" else (yx, yy, goal_sizes[0])

    line_hit, penalty_hit, wall_angle, wall_dist = estimate_virtual_line(wx, wy, wsize)
    flags = (1 if line_hit else 0) | (2 if penalty_hit else 0)

    # 1) 壁/白線が近ければ最優先で内側へ回避し、キックはしない
    if wall_angle is not None and wall_dist is not None:
        wall_err = angle_error(wall_angle, 0.0)
        near_wall = abs(wall_err) <= WALL_AVOID_ANGLE_DEG and wall_dist <= WALL_AVOID_DIST_CM
        if near_wall or line_hit:
            turn = int(35.0 * (1.0 if wall_err >= 0.0 else -1.0))
            return 0, -50, turn, 0, MODE_AVOID, "AVOID", flags

    # 2) ゴールが見えない -> 探索
    if gx == 255 or gy == 255:
        return 0, 10, 20, 0, MODE_SEARCH, "SEARCH", flags

    goal_angle = angle_from_normalized(gx, gy)
    err = angle_error(goal_angle, 0.0)
    turn = int(clamp(-err * 0.6, -60.0, 60.0))
    vy = 20 if abs(err) > 45.0 else 55
    if abs(err) <= GOAL_SHOOT_ANGLE_DEG and gsize >= GOAL_NEAR_PIX and not line_hit:
        return 0, vy, turn, 1, MODE_SHOOT, "SHOOT", flags
    return 0, vy, turn, 0, MODE_APPROACH, "APPROACH", flags


TARGET_GOAL = "YELLOW"


def make_nav_packet(vx, vy, turn, kick, mode, flags=0):
    body = [
        HDR_NAV,
        int(clamp(vx, -100, 100)) & 0xFF,
        int(clamp(vy, -100, 100)) & 0xFF,
        int(clamp(turn, -100, 100)) & 0xFF,
        kick & 0x01,
        mode & 0xFF,
        flags & 0xFF,
    ]
    checksum = 0
    for value in body:
        checksum ^= value
    return bytearray(body + [checksum])


def make_legacy_packets(yx, yy, bx, by, wx, wy, wsize):
    return (bytearray([HDR_GOALWALL, yx, yy, bx, by, wx, wy]),
            bytearray([HDR_WALL, wx, wy, (wsize >> 8) & 0xFF, wsize & 0xFF, 0xFF, 0xFF]))


# ---------- OpenMV H7 固有 ----------
def init_camera():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    # H7 は B320X320 非対応の場合があるため順に試す
    for size in ("B320X320", "B640X480", "QVGA", "QQVGA"):
        try:
            sensor.set_framesize(getattr(sensor, size))
            break
        except Exception:
            continue
    sensor.skip_frames(time=300)
    try:
        sensor.set_auto_exposure(True)
        sensor.set_auto_gain(True)
        sensor.set_auto_whitebal(False)
    except Exception:
        pass


def biggest_blob(img, thresholds, roi, pixel_threshold):
    best, cx, cy = 0, None, None
    for blob in img.find_blobs(thresholds, roi=roi,
                               pixels_threshold=pixel_threshold,
                               area_threshold=pixel_threshold,
                               merge=True, margin=6):
        if blob.pixels() > best:
            best, cx, cy = blob.pixels(), blob.cx(), blob.cy()
    return best, cx, cy


def main_loop():
    init_camera()
    uart = UART(3, 9600, timeout_char=1000)
    uart.init(9600, bits=8, parity=None, stop=1, timeout_char=1000)
    clock = time.clock()
    last_stream_ms = pyb.millis()
    last_kick_ms = -1000
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
        vx, vy, turn, want_kick, mode, name, flags = decide_navigation(
            yx, yy, bx, by, wx, wy, wsize, (ysize, bsize))
        # クールダウン削除: 要求が立ち上がるたびにキックを送る
        kick = 1 if want_kick else 0
        try:
            print("NAV {} vx{} vy{} turn{} kick{} wall={}cm flags{}".format(
                name, vx, vy, turn, kick,
                -1 if wsize <= 0 else int(wall_distance_cm(wsize)), flags))
        except Exception:
            pass
        if (now - last_stream_ms) >= STREAM_INTERVAL_MS:
            last_stream_ms = now
            goal, wall = make_legacy_packets(yx, yy, bx, by, wx, wy, wsize)
            uart.write(goal)
            uart.write(wall)
            uart.write(make_nav_packet(vx, vy, turn, kick, mode, flags))


if __name__ == "__main__":
    main_loop()
