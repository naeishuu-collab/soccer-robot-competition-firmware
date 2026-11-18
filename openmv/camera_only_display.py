# OpenMV H7 単体デモ：検知した物体の名前をカメラ画面に表示する
#
# 特徴:
#   - MAIN / IR / LINE 基板は不要。カメラ1枚で完結する。
#   - 黄色ゴール / 青ゴール / 黒い壁を検出し、名前と枠を画像に重ねて表示。
#   - 仮想白線・ペナルティエリア・ナビモードも画面上部に表示。
#   - 競技ルール対応（補助IR LED/可視LED OFF、壁探索は画像下2/3）は維持。
#   - 既定では 0x20 ナビパケットも UART3 へ送出する（SEND_UART=False で無効化）。
#
# 注: OpenMV の内蔵フォントは ASCII のみのため、画面表示は英語名。
#     日本語名は USB ターミナルへ print する。

import math

# ---------- 検出設定 ----------
PIXELS_THRESHOLD = 20
WALL_PIX_TH = 70
STREAM_INTERVAL_MS = 200

YellowThresholds = [(64, 91, -6, 14, 26, 62)]
BlueThresholds = [(85, 0, 68, -128, -48, -17)]
BlackThresholds = [(91, 52, 61, 8, 20, -10)]
GoalRoi = (30, 30, 260, 260)
# オレンジのIRボール。実機の色で要調整。
BallThresholds = [(60, 95, 40, 95, 60, 130)]
# 相手ロボットのマーカー（赤系）。実機の色で要調整。
ObstacleThresholds = [(30, 90, 140, 255, 110, 200)]

# ---------- 競技台定数 (cm) ----------
OUTER_CM = 12.0
PENALTY_DEPTH_CM = 25.0
VIRTUAL_LINE_MARGIN_CM = 8.0
WALL_AVOID_ANGLE_DEG = 60.0
WALL_AVOID_DIST_CM = 35.0
GOAL_SHOOT_ANGLE_DEG = 15.0
GOAL_NEAR_PIX = 60
WALL_REF_PIX = 6000
WALL_REF_DIST_CM = 30.0

HDR_NAV = 0x20
MODE_AVOID = 1
MODE_SEARCH = 2
MODE_APPROACH = 3
MODE_SHOOT = 4

TARGET_GOAL = "YELLOW"
OWN_GOAL = "BLUE"
# 最初に安定して見えたゴールの色を相手ゴールとして自動ロックする。
AUTO_TARGET_COLOR = True
TARGET_LOCK_FRAMES = 5
target_locked = not AUTO_TARGET_COLOR
target_candidate = None
target_candidate_count = 0
BALL_FRONT_ANGLE_DEG = 25.0
BALL_GOAL_ALIGN_TOL_DEG = 12.0
OBSTACLE_PIX_TH = 60
MODE_DEFEND = 5
FLAG_VIRTUAL_LINE = 0x01
FLAG_PENALTY_AREA = 0x02
FLAG_BALL = 0x04
FLAG_BALL_ALIGNED = 0x08
FLAG_BALL_IN_KICKER = 0x10
BALL_REF_PIX = 4000.0
BALL_REF_DIST_CM = 10.0
BALL_KICK_MIN_CM = 5.0
BALL_KICK_MAX_CM = 16.0
SEND_UART = True

# ---------- 描画色 (RGB565) ----------
COLOR_GOAL_YELLOW = (255, 255, 0)
COLOR_GOAL_BLUE = (0, 255, 255)
COLOR_WALL = (255, 80, 80)
COLOR_BALL = (255, 170, 0)
COLOR_OBSTACLE = (255, 0, 255)
COLOR_TEXT = (255, 255, 255)
COLOR_INFO = (0, 255, 0)


# ---------- 純粋ロジック（ホストでもテスト可能） ----------
def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


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
    """黒い壁のblob.pixels()から距離[cm]を推定する（逆比例モデル）。"""
    if wsize is None or wsize <= 0:
        return None
    return WALL_REF_DIST_CM * float(WALL_REF_PIX) / float(wsize)


def ball_distance_cm(ball_size):
    """ボールのblob面積から距離[cm]を推定（逆比例モデル・要較正）。"""
    if ball_size is None or ball_size <= 0:
        return None
    return BALL_REF_DIST_CM * float(BALL_REF_PIX) / float(ball_size)


def estimate_virtual_line(wx, wy, wsize):
    if wx == 255 or wy == 255 or wsize <= 0:
        return False, False, None, None
    angle = angle_from_normalized(wx, wy)
    dist_wall = wall_distance_cm(wsize)
    if angle is None or dist_wall is None:
        return False, False, None, None
    dist_line = dist_wall - OUTER_CM
    line_hit = dist_line <= VIRTUAL_LINE_MARGIN_CM
    penalty_hit = dist_wall <= (OUTER_CM + PENALTY_DEPTH_CM)
    return line_hit, penalty_hit, angle, dist_line


def lock_target_from_goals(ysize, bsize):
    """最初に安定して見えたゴールの色を攻撃対象としてロックする。"""
    global TARGET_GOAL, OWN_GOAL, target_locked, target_candidate, target_candidate_count
    if target_locked or not AUTO_TARGET_COLOR:
        return
    if ysize >= GOAL_NEAR_PIX and bsize < GOAL_NEAR_PIX:
        color = "YELLOW"
    elif bsize >= GOAL_NEAR_PIX and ysize < GOAL_NEAR_PIX:
        color = "BLUE"
    else:
        target_candidate = None
        target_candidate_count = 0
        return
    if color == target_candidate:
        target_candidate_count += 1
    else:
        target_candidate = color
        target_candidate_count = 1
    if target_candidate_count >= TARGET_LOCK_FRAMES:
        TARGET_GOAL = color
        OWN_GOAL = "BLUE" if color == "YELLOW" else "YELLOW"
        target_locked = True


def decide_navigation(yx, yy, bx, by, wx, wy, wsize, goal_sizes,
                      ball=None, obstacle=None):
    """detections -> (vx, vy, turn, want_kick, mode_code, mode_name, flags)"""
    if target_locked:
        gx, gy, gsize = (bx, by, goal_sizes[1]) if TARGET_GOAL == "BLUE" else (yx, yy, goal_sizes[0])
        ox, oy, osize = (yx, yy, goal_sizes[0]) if TARGET_GOAL == "BLUE" else (bx, by, goal_sizes[1])
    else:
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

    if wall_angle is not None and wall_dist is not None:
        wall_err = angle_error(wall_angle, 0.0)
        near_wall = abs(wall_err) <= WALL_AVOID_ANGLE_DEG and wall_dist <= WALL_AVOID_DIST_CM
        away_turn = int(35.0 * (1.0 if wall_err >= 0.0 else -1.0))
        if near_wall or line_hit:
            return 0, -50, away_turn, 0, MODE_AVOID, "AVOID", flags
        if penalty_hit:
            return 0, 60, away_turn, 0, MODE_AVOID, "AVOID", flags

    if obstacle is not None and obstacle[0] != 255 and obstacle[2] >= OBSTACLE_PIX_TH:
        obs_angle = angle_from_normalized(obstacle[0], obstacle[1])
        if obs_angle is not None:
            obs_err = angle_error(obs_angle, 0.0)
            if abs(obs_err) <= 45.0:
                away_turn = int(-35.0 * (1.0 if obs_err >= 0.0 else -1.0))
                return 0, -30, away_turn, 0, MODE_AVOID, "AVOID", flags

    if (ox != 255 and oy != 255) and (gx == 255 or gy == 255):
        return 0, -20, 25, 0, MODE_DEFEND, "DEFEND", flags

    if gx == 255 or gy == 255:
        if ball_seen:
            err = angle_error(ball_angle, 0.0)
            return 0, 40, int(clamp(-err * 0.6, -60.0, 60.0)), 0, MODE_APPROACH, "APPROACH", flags
        return 0, 10, 20, 0, MODE_SEARCH, "SEARCH", flags

    goal_angle = angle_from_normalized(gx, gy)
    err = angle_error(goal_angle, 0.0)
    turn = int(clamp(-err * 0.6, -60.0, 60.0))
    vy = 20 if abs(err) > 45.0 else 55
    ball_goal_error = abs(angle_error(goal_angle, ball_angle)) if ball_angle is not None else 180.0
    if (target_locked and abs(err) <= GOAL_SHOOT_ANGLE_DEG and gsize >= GOAL_NEAR_PIX and
            not line_hit and ball_in_kicker and ball_goal_error <= BALL_GOAL_ALIGN_TOL_DEG):
        return 0, vy, turn, 1, MODE_SHOOT, "SHOOT", flags
    return 0, vy, turn, 0, MODE_APPROACH, "APPROACH", flags


def detected_objects(y_size, ycx, ycy, b_size, bcx, bcy, w_size, wcx, wcy,
                     ball=None, obstacle=None):
    """検知結果を (英語名, 日本語名, cx, cy, 色, サイズ) のリストにする。"""
    found = []
    if y_size >= PIXELS_THRESHOLD and ycx is not None:
        name = "YELLOW GOAL" if TARGET_GOAL == "YELLOW" else "OWN GOAL"
        found.append((name, "黄色ゴール", ycx, ycy, COLOR_GOAL_YELLOW, y_size))
    if b_size >= PIXELS_THRESHOLD and bcx is not None:
        name = "BLUE GOAL" if TARGET_GOAL == "BLUE" else "OWN GOAL"
        found.append((name, "青ゴール", bcx, bcy, COLOR_GOAL_BLUE, b_size))
    if w_size >= WALL_PIX_TH and wcx is not None:
        found.append(("BLACK WALL", "黒い壁", wcx, wcy, COLOR_WALL, w_size))
    if ball is not None and ball[0] != 255 and ball[2] > 0:
        found.append(("BALL", "ボール", ball[0], ball[1], COLOR_BALL, ball[2]))
    if obstacle is not None and obstacle[0] != 255 and obstacle[2] >= OBSTACLE_PIX_TH:
        found.append(("OBSTACLE", "相手ロボット", obstacle[0], obstacle[1], COLOR_OBSTACLE, obstacle[2]))
    return found


def status_lines(objects, mode_name, want_kick, line_hit, penalty_hit, wall_cm, flags=0):
    """画面上部に表示するステータス行。"""
    names = ",".join(obj[0] for obj in objects) if objects else "NONE"
    target = TARGET_GOAL if target_locked else "LOCKING..."
    lines = ["DETECT: " + names, "TARGET: " + target,
             "MODE: " + mode_name, "KICK: " + ("YES" if want_kick else "NO")]
    if wall_cm is not None:
        lines.append("WALL: %dcm" % int(wall_cm))
    if line_hit:
        lines.append("VIRTUAL LINE: DETECTED")
    if penalty_hit:
        lines.append("PENALTY AREA: IN")
    if flags & FLAG_BALL:
        state = "SEEN"
        if flags & FLAG_BALL_ALIGNED:
            state += " / FRONT"
        if flags & FLAG_BALL_IN_KICKER:
            state += " / IN KICKER"
        lines.append("BALL: " + state)
    elif mode_name in ("SHOOT", "APPROACH"):
        lines.append("BALL: NOT SEEN")
    return lines


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


# ---------- OpenMV H7 固有 ----------
def init_camera():
    import sensor
    import pyb

    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
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
    # RCJ 2026 ルール 6.2.B / 3.1.3: IR発光器と可視LEDは禁止。
    try:
        sensor.set_ir_led(False)
    except Exception:
        pass
    try:
        for led_id in (1, 2, 3):
            pyb.LED(led_id).off()
    except Exception:
        pass


def biggest_blob_info(img, thresholds, roi, pixel_threshold):
    """(size, cx, cy, rect) を返す。未検出なら (0, None, None, None)。"""
    best, cx, cy, rect = 0, None, None, None
    for blob in img.find_blobs(thresholds, roi=roi,
                               pixels_threshold=pixel_threshold,
                               area_threshold=pixel_threshold,
                               merge=True, margin=6):
        if blob.pixels() > best:
            best, cx, cy, rect = blob.pixels(), blob.cx(), blob.cy(), blob.rect()
    return best, cx, cy, rect


def norm_coord(cx, cy, img):
    if cx is None or cy is None:
        return 255, 255
    return int((cx * 255) // img.width()), int((cy * 255) // img.height())


def draw_overlay(img, objects, rects, lines):
    for name, cx, cy, color, _size in objects:
        rect = rects.get(name)
        if rect is not None:
            try:
                img.draw_rectangle(rect, color=color, thickness=1)
            except Exception:
                img.draw_rectangle(rect, color=color)
        img.draw_string(max(0, cx - 20), max(0, cy - 14), name, color=color, scale=1)
    y = 2
    for line in lines:
        img.draw_string(2, y, line, color=COLOR_INFO if y == 0 else COLOR_TEXT, scale=1)
        y += 12


def main_loop():
    import pyb
    import time
    from machine import UART

    init_camera()
    sensor = __import__("sensor")

    uart = None
    if SEND_UART:
        uart = UART(3, 9600, timeout_char=1000)
        uart.init(9600, bits=8, parity=None, stop=1, timeout_char=1000)

    clock = time.clock()
    last_stream_ms = pyb.millis()

    while True:
        clock.tick()
        img = sensor.snapshot()
        width, height = img.width(), img.height()
        goal_roi = GoalRoi
        wall_roi = (0, height // 3, width, height * 2 // 3)

        y_size, ycx, ycy, y_rect = biggest_blob_info(img, YellowThresholds, goal_roi, PIXELS_THRESHOLD)
        b_size, bcx, bcy, b_rect = biggest_blob_info(img, BlueThresholds, goal_roi, PIXELS_THRESHOLD)
        w_size, wcx, wcy, w_rect = biggest_blob_info(img, BlackThresholds, wall_roi, WALL_PIX_TH)
        ball_size, ball_cx, ball_cy, ball_rect = biggest_blob_info(
            img, BallThresholds, wall_roi, PIXELS_THRESHOLD)
        obs_size, obs_cx, obs_cy, obs_rect = biggest_blob_info(
            img, ObstacleThresholds, wall_roi, PIXELS_THRESHOLD)

        yx, yy = norm_coord(ycx, ycy, img)
        bx, by = norm_coord(bcx, bcy, img)
        wx, wy = norm_coord(wcx, wcy, img)
        ball_x, ball_y = norm_coord(ball_cx, ball_cy, img)
        obs_x, obs_y = norm_coord(obs_cx, obs_cy, img)
        ball = (ball_x, ball_y, ball_size)
        obstacle = (obs_x, obs_y, obs_size)

        lock_target_from_goals(y_size, b_size)
        vx, vy, turn, want_kick, mode, mode_name, flags = decide_navigation(
            yx, yy, bx, by, wx, wy, w_size, (y_size, b_size), ball, obstacle)
        line_hit = (flags & 0x01) != 0
        penalty_hit = (flags & 0x02) != 0
        wall_cm = wall_distance_cm(w_size)

        objects = detected_objects(y_size, ycx, ycy, b_size, bcx, bcy, w_size, wcx, wcy, ball, obstacle)
        rects = {}
        if y_rect is not None:
            rects["YELLOW GOAL" if TARGET_GOAL == "YELLOW" else "OWN GOAL"] = y_rect
        if b_rect is not None:
            rects["BLUE GOAL" if TARGET_GOAL == "BLUE" else "OWN GOAL"] = b_rect
        if w_rect is not None:
            rects["BLACK WALL"] = w_rect
        if ball_rect is not None:
            rects["BALL"] = ball_rect
        if obs_rect is not None:
            rects["OBSTACLE"] = obs_rect
        lines = status_lines(objects, mode_name, want_kick, line_hit, penalty_hit, wall_cm, flags)
        draw_overlay(img, objects, rects, lines)

        names_ja = ", ".join(obj[1] for obj in objects) if objects else "なし"
        print("DETECT: " + names_ja + " | MODE: " + mode_name)

        now = pyb.millis()
        if SEND_UART and uart is not None and (now - last_stream_ms) >= STREAM_INTERVAL_MS:
            last_stream_ms = now
            uart.write(make_nav_packet(vx, vy, turn, want_kick, mode, flags))


if __name__ == "__main__":
    main_loop()
