# OpenMV H7: 壁距離 → 仮想白線 + ゴールシュート
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
# カメラは絶対位置を持たないため、cm定数は「距離推定の較正と妥当性チェック」に使う。
# WALL_H_CM / PENALTY_WIDTH_CM / CENTER_CIRCLE_CM は資料用の参照値で判定には使わない。
MAX_FIELD_CM = max(FIELD_W_CM, FIELD_L_CM)
GOAL_SHOOT_MAX_DIST_CM = FIELD_L_CM * 0.6
GOAL_REF_PIX = 500.0
GOAL_REF_DIST_CM = 100.0

# ---------- 検出設定 ----------
PIXELS_THRESHOLD = 20
WALL_PIX_TH = 70
STREAM_INTERVAL_MS = 200

WALL_AVOID_ANGLE_DEG = 60.0
WALL_AVOID_DIST_CM = 35.0     # 壁がこの距離より近ければ回避
GOAL_SHOOT_ANGLE_DEG = 15.0
GOAL_NEAR_PIX = 60
# 人物の服などの小さな誤検出をゴールと扱わないための最小面積。
GOAL_MIN_PIX = 200

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
# オレンジのIRボール。黄ゴールと色が近いため、後段で「ボール優先」の重なり判定も行う。
BallThresholds = [(60, 95, 15, 80, 40, 120)]
# 相手ロボットのマーカー（赤系）。実機の色で要調整。
ObstacleThresholds = [(30, 90, 140, 255, 110, 200)]
GoalRoi = (30, 30, 260, 260)

# 攻撃するゴールの色と守るゴールの色（試合のサイドに合わせて入れ替える）
TARGET_GOAL = "YELLOW"
OWN_GOAL = "BLUE"
# 起動後に最初に安定して見えたゴールの色を相手ゴールとしてロックする。
# コイントス／後半のエンド入れ替えに合わせて自動で決まるが、最初に自ゴールを
# 見た場合は誤るため、必要なら AUTO_TARGET_COLOR=False にして定数で固定する。
AUTO_TARGET_COLOR = True
TARGET_LOCK_FRAMES = 5
# セット時の正面(0°)に最も近いゴールを相手ゴールとする。それより後ろのゴールは候補にしない。
TARGET_LOCK_MAX_ANGLE_DEG = 90.0
target_locked = not AUTO_TARGET_COLOR
target_candidate = None
target_candidate_count = 0
BALL_FRONT_ANGLE_DEG = 25.0
# シュートはボールとゴールが同じ方向に並んでいないと外れるため、両者の方位差も見る。
BALL_GOAL_ALIGN_TOL_DEG = 12.0
OBSTACLE_PIX_TH = 60
MODE_DEFEND = 5
FLAG_VIRTUAL_LINE = 0x01
FLAG_PENALTY_AREA = 0x02
FLAG_BALL = 0x04
FLAG_BALL_ALIGNED = 0x08
FLAG_BALL_IN_KICKER = 0x10
# ボールのblob面積→距離（逆比例モデル）。実機のカメラ/レンズで要較正。
BALL_REF_PIX = 4000.0
BALL_REF_DIST_CM = 10.0
# キッカーのポケットに入っているとみなす距離[cm]（ロボット半径約11cm＋捕獲ゾーン3cm）。
BALL_KICK_MIN_CM = 5.0
BALL_KICK_MAX_CM = 16.0


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
    return min(MAX_FIELD_CM, WALL_REF_DIST_CM * float(WALL_REF_PIX) / float(wsize))


def goal_distance_cm(goal_size):
    """ゴールのblob面積から距離[cm]を推定（要較正）。"""
    if goal_size is None or goal_size <= 0:
        return None
    return min(MAX_FIELD_CM, GOAL_REF_DIST_CM * float(GOAL_REF_PIX) / float(goal_size))


def ball_distance_cm(ball_size):
    """ボールのblob面積から距離[cm]を推定（逆比例モデル・要較正）。"""
    if ball_size is None or ball_size <= 0:
        return None
    return BALL_REF_DIST_CM * float(BALL_REF_PIX) / float(ball_size)


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


def lock_target_from_goals(ysize, bsize, yx=255, yy=255, bx=255, by=255):
    """起動時に正面(0°)へ最も近いゴールの色を相手ゴールとしてロックする。

    カメラが広範囲に見えて両方のゴールが同時に映っても、方位误差が小さい
    （＝セッティングで正面に向かっている）ほうを選ぶ。
    """
    global TARGET_GOAL, OWN_GOAL, target_locked, target_candidate, target_candidate_count
    if target_locked or not AUTO_TARGET_COLOR:
        return
    candidates = []
    for size, nx, ny, color in ((ysize, yx, yy, "YELLOW"), (bsize, bx, by, "BLUE")):
        if size < GOAL_MIN_PIX:
            continue
        angle = angle_from_normalized(nx, ny)
        if angle is None:
            continue
        err = abs(angle_error(angle, 0.0))
        if err <= TARGET_LOCK_MAX_ANGLE_DEG:
            candidates.append((err, color))
    if not candidates:
        target_candidate = None
        target_candidate_count = 0
        return
    candidates.sort()
    color = candidates[0][1]
    if color == target_candidate:
        target_candidate_count += 1
    else:
        target_candidate = color
        target_candidate_count = 1
    if target_candidate_count >= TARGET_LOCK_FRAMES:
        TARGET_GOAL = color
        OWN_GOAL = "BLUE" if color == "YELLOW" else "YELLOW"
        target_locked = True
        print("TARGET LOCKED: " + TARGET_GOAL)


def decide_navigation(yx, yy, bx, by, wx, wy, wsize, goal_sizes,
                      ball=None, obstacle=None):
    """detections -> (vx, vy, turn, want_kick, mode_code, mode_name, flags)

    ball: (nx, ny, size) or None  … カメラで見つけたオレンジボール
    obstacle: (nx, ny, size) or None … 相手ロボットのマーカー
    """
    if target_locked:
        gx, gy, gsize = (bx, by, goal_sizes[1]) if TARGET_GOAL == "BLUE" else (yx, yy, goal_sizes[0])
        ox, oy, osize = (yx, yy, goal_sizes[0]) if TARGET_GOAL == "BLUE" else (bx, by, goal_sizes[1])
    else:
        # 攻撃対象が未確定の間は自ゴール判定も行わず、シュートも禁止する。
        gx = gy = ox = oy = 255
        gsize = osize = 0

    line_hit, penalty_hit, wall_angle, wall_dist = estimate_virtual_line(wx, wy, wsize)
    flags = (1 if line_hit else 0) | (2 if penalty_hit else 0)

    ball_seen = ball is not None and ball[0] != 255 and ball[1] != 255 and ball[2] > 0
    ball_angle = angle_from_normalized(ball[0], ball[1]) if ball_seen else None
    ball_aligned = ball_angle is not None and abs(angle_error(ball_angle, 0.0)) <= BALL_FRONT_ANGLE_DEG
    if ball_seen:
        flags |= FLAG_BALL
    if ball_aligned:
        flags |= FLAG_BALL_ALIGNED
    ball_dist = ball_distance_cm(ball[2]) if ball_seen else None
    ball_in_kicker = (ball_aligned and ball_dist is not None and
                      BALL_KICK_MIN_CM <= ball_dist <= BALL_KICK_MAX_CM)
    if ball_in_kicker:
        flags |= FLAG_BALL_IN_KICKER

    # 1) 壁/白線が近ければ最優先で内側へ回避し、キックはしない
    if wall_angle is not None and wall_dist is not None:
        wall_err = angle_error(wall_angle, 0.0)
        near_wall = abs(wall_err) <= WALL_AVOID_ANGLE_DEG and wall_dist <= WALL_AVOID_DIST_CM
        away_turn = int(35.0 * (1.0 if wall_err >= 0.0 else -1.0))
        if near_wall or line_hit:
            return 0, -50, away_turn, 0, MODE_AVOID, "AVOID", flags
        # 2) ペナルティエリア内/境界なら壁から離れて中央へ戻る (rule 2.6.1)
        if penalty_hit:
            return 0, 60, away_turn, 0, MODE_AVOID, "AVOID", flags

    # 3) 相手ロボットが正面にいれば回避し、キックはしない
    if obstacle is not None and obstacle[0] != 255 and obstacle[2] >= OBSTACLE_PIX_TH:
        obs_angle = angle_from_normalized(obstacle[0], obstacle[1])
        if obs_angle is not None:
            obs_err = angle_error(obs_angle, 0.0)
            if abs(obs_err) <= 45.0:
                away_turn = int(-35.0 * (1.0 if obs_err >= 0.0 else -1.0))
                return 0, -30, away_turn, 0, MODE_AVOID, "AVOID", flags

    # 4) 自ゴールだけが見えている場合は退避（自ゴールには蹴らない）
    if (ox != 255 and oy != 255) and (gx == 255 or gy == 255):
        return 0, -20, 25, 0, MODE_DEFEND, "DEFEND", flags

    # 5) 攻撃対象ゴールが見えない -> ボールがあれば回収、なければ探索
    if gx == 255 or gy == 255:
        if ball_seen:
            err = angle_error(ball_angle, 0.0)
            return 0, 40, int(clamp(-err * 0.6, -60.0, 60.0)), 0, MODE_APPROACH, "APPROACH", flags
        return 0, 10, 20, 0, MODE_SEARCH, "SEARCH", flags

    # 6) 攻撃対象ゴールが見えている。シュートは「ボールがキッカーに入っており、
    #    かつボールとゴールが同じ方向に並んでいる」場合だけ。
    goal_angle = angle_from_normalized(gx, gy)
    err = angle_error(goal_angle, 0.0)
    turn = int(clamp(-err * 0.6, -60.0, 60.0))
    vy = 20 if abs(err) > 45.0 else 55
    goal_dist = goal_distance_cm(gsize)
    if goal_dist is not None and goal_dist > GOAL_SHOOT_MAX_DIST_CM:
        return 0, vy, turn, 0, MODE_APPROACH, "APPROACH", flags
    ball_goal_error = abs(angle_error(goal_angle, ball_angle)) if ball_angle is not None else 180.0
    if (target_locked and abs(err) <= GOAL_SHOOT_ANGLE_DEG and gsize >= GOAL_NEAR_PIX and
            not line_hit and ball_in_kicker and ball_goal_error <= BALL_GOAL_ALIGN_TOL_DEG):
        return 0, vy, turn, 1, MODE_SHOOT, "SHOOT", flags
    return 0, vy, turn, 0, MODE_APPROACH, "APPROACH", flags


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
    # RCJ 2026 ルール6.2.B: 赤外線リーグではIR発光器は禁止。
    # OpenMV H7 の補助IR LEDと可視状態LEDをすべて消灯し、
    # 相手機体に干渉しないようにする（ルール3.1.3）。API差に備えガード付き。
    try:
        sensor.set_ir_led(False)
    except Exception:
        pass
    try:
        for led_id in (1, 2, 3):
            pyb.LED(led_id).off()
    except Exception:
        pass


# ---------- 画面表示用（OpenMV v5 互換） ----------
COLOR_GOAL_YELLOW = (255, 255, 0)
COLOR_GOAL_BLUE = (0, 255, 255)
COLOR_WALL = (255, 80, 80)
COLOR_BALL = (255, 170, 0)
COLOR_OBSTACLE = (255, 0, 255)
COLOR_TEXT = (255, 255, 255)
COLOR_INFO = (0, 255, 0)


def blob_value(blob, name):
    """OpenMV v5 は pixels/cx/cy/rect がプロパティ、旧版はメソッド。両対応する。"""
    value = getattr(blob, name)
    return value() if callable(value) else value


def safe_draw_string(img, x, y, text, color):
    x = int(max(0, x))
    y = int(max(0, y))
    text = str(text)
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
    try:
        img.draw_string((x, y), text)
    except Exception:
        pass


def safe_draw_rectangle(img, rect, color):
    if not isinstance(rect, (tuple, list)):
        return
    try:
        img.draw_rectangle(rect, color, 1)
    except Exception:
        try:
            img.draw_rectangle(rect, color)
        except Exception:
            pass


def draw_detections(img, objects, status):
    """objects: (name, cx, cy, color, rect) のリストを画面に重ねる。"""
    for name, cx, cy, color, rect in objects:
        safe_draw_rectangle(img, rect, color)
        safe_draw_string(img, cx - 20, cy - 14, name, color)
    y = 2
    for line in status:
        safe_draw_string(img, 2, y, line, COLOR_INFO if y == 0 else COLOR_TEXT)
        y += 12


def rect_center_inside(rect, cx, cy):
    if rect is None or cx is None or cy is None:
        return False
    x, y, w, h = rect
    return x <= cx <= x + w and y <= cy <= y + h


def biggest_blob(img, thresholds, roi, pixel_threshold):
    best, cx, cy, rect = 0, None, None, None
    for blob in img.find_blobs(thresholds, roi=roi,
                               pixels_threshold=pixel_threshold,
                               area_threshold=pixel_threshold,
                               merge=True, margin=6):
        size = blob_value(blob, "pixels")
        if size > best:
            best = size
            cx = blob_value(blob, "cx")
            cy = blob_value(blob, "cy")
            rect = blob_value(blob, "rect")
    return best, cx, cy, rect


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
        ysize, ycx, ycy, y_rect = biggest_blob(img, YellowThresholds, GoalRoi, GOAL_MIN_PIX)
        bsize, bcx, bcy, b_rect = biggest_blob(img, BlueThresholds, GoalRoi, GOAL_MIN_PIX)
        # ルール3.1.4: 壁の上の色（服など）を無視するため、
        # 壁探索を画像下2/3に制限する。
        wall_roi = (0, img.height() // 3, img.width(), img.height() * 2 // 3)
        wsize, wcx, wcy, w_rect = biggest_blob(img, BlackThresholds, wall_roi, WALL_PIX_TH)
        ball_size, ball_cx, ball_cy, ball_rect = biggest_blob(img, BallThresholds, wall_roi, PIXELS_THRESHOLD)
        obs_size, obs_cx, obs_cy, obs_rect = biggest_blob(img, ObstacleThresholds, wall_roi, PIXELS_THRESHOLD)
        # オレンジボールは黄ゴール閾値にも引っかかるため、ボールと重なったゴール検出は破棄する。
        if ball_size > 0 and ball_cx is not None:
            if rect_center_inside(ball_rect, ycx, ycy):
                ysize, ycx, ycy, y_rect = 0, None, None, None
            if rect_center_inside(ball_rect, bcx, bcy):
                bsize, bcx, bcy, b_rect = 0, None, None, None
        yx, yy = norm_coord(ycx, ycy, img)
        bx, by = norm_coord(bcx, bcy, img)
        wx, wy = norm_coord(wcx, wcy, img)
        ball_x, ball_y = norm_coord(ball_cx, ball_cy, img)
        obs_x, obs_y = norm_coord(obs_cx, obs_cy, img)
        lock_target_from_goals(ysize, bsize, yx, yy, bx, by)
        now = pyb.millis()
        vx, vy, turn, want_kick, mode, name, flags = decide_navigation(
            yx, yy, bx, by, wx, wy, wsize, (ysize, bsize),
            (ball_x, ball_y, ball_size), (obs_x, obs_y, obs_size))

        # 検知した物体の名前と枠をカメラ画面に表示する。
        objects = []
        if ysize >= PIXELS_THRESHOLD and ycx is not None:
            objects.append(("YELLOW GOAL" if TARGET_GOAL == "YELLOW" else "OWN GOAL",
                            ycx, ycy, COLOR_GOAL_YELLOW, y_rect))
        if bsize >= PIXELS_THRESHOLD and bcx is not None:
            objects.append(("BLUE GOAL" if TARGET_GOAL == "BLUE" else "OWN GOAL",
                            bcx, bcy, COLOR_GOAL_BLUE, b_rect))
        if wsize >= WALL_PIX_TH and wcx is not None:
            objects.append(("BLACK WALL", wcx, wcy, COLOR_WALL, w_rect))
        if ball_size >= PIXELS_THRESHOLD and ball_cx is not None:
            objects.append(("BALL", ball_cx, ball_cy, COLOR_BALL, ball_rect))
        if obs_size >= OBSTACLE_PIX_TH and obs_cx is not None:
            objects.append(("OBSTACLE", obs_cx, obs_cy, COLOR_OBSTACLE, obs_rect))
        status = [
            "DETECT: " + (",".join(obj[0] for obj in objects) if objects else "NONE"),
            "TARGET: " + (TARGET_GOAL if target_locked else "LOCKING..."),
            "MODE: " + name,
            "KICK: " + ("YES" if want_kick else "NO"),
        ]
        draw_detections(img, objects, status)

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
