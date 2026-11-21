"""H7 Plus competition camera. Copy camera_calibration.py alongside main.py."""
import math

import camera_calibration as cfg

try:
    import sensor
    import image
    import time
    import pyb
except ImportError:
    sensor = image = time = pyb = None

OUTER_CM = 12
LINE_CLEARANCE_CM = 18
LOOKAHEAD_CM = 50
RAY_COUNT = 24
GOAL_STABLE_FRAMES = 3
GOAL_MAX_JUMP_DEG = 20
GOAL_MAX_AIM_DEG = 7
GOAL_EDGE_MARGIN_DEG = 2
STREAM_INTERVAL_MS = 50
MAX_FRAME_AGE_MS = 100

MODE_WAIT = 0
MODE_AVOID = 1
MODE_TRACK_IR = 2
MODE_ALIGN = 3
MODE_AIMED = 4
FLAG_VIRTUAL_LINE = 0x01
FLAG_OMNI = 0x20  # Distinguishes the calibrated camera protocol from legacy 0x20 senders.

GOAL_COLORS = {"YELLOW": (255, 255, 0), "BLUE": (0, 110, 255)}
MODES = ("WAIT", "AVOID", "TRACK IR", "ALIGN", "AIMED")


def angle_error(angle, reference=0):
    return (angle - reference + 180) % 360 - 180


def image_angle(dx, dy):
    return math.degrees(math.atan2(dx, -dy)) % 360


def blob_value(blob, name):
    value = getattr(blob, name)
    return value() if callable(value) else value


def in_lab(lab, threshold):
    l, a, b = lab
    return (threshold[0] <= l <= threshold[1] and threshold[2] <= a <= threshold[3]
            and threshold[4] <= b <= threshold[5])


def init_camera():
    for led_id in (1, 2, 3, 4):
        pyb.LED(led_id).off()
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(getattr(sensor, cfg.FRAME_SIZE))
    sensor.set_auto_exposure(True)
    sensor.set_auto_gain(True)
    sensor.set_auto_whitebal(True)
    sensor.skip_frames(time=1500)
    rgb_gain = sensor.get_rgb_gain_db() if cfg.RGB_GAIN_DB is None else cfg.RGB_GAIN_DB
    sensor.set_auto_whitebal(False, rgb_gain_db=rgb_gain)
    if cfg.EXPOSURE_US is not None:
        sensor.set_auto_exposure(False, exposure_us=cfg.EXPOSURE_US)
    if cfg.GAIN_DB is not None:
        sensor.set_auto_gain(False, gain_db=cfg.GAIN_DB)
    sensor.skip_frames(time=300)


class Mirror:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.cx, self.cy = cfg.MIRROR_CENTER or ((width - 1) / 2, (height - 1) / 2)
        self.inner, self.outer = cfg.MIRROR_RING or (0, 0)
        self.points = cfg.GROUND_POINTS
        self.ring_ready = (cfg.IMAGE_SIZE == (width, height) and cfg.MIRROR_CENTER is not None
                           and 0 <= self.inner < self.outer
                           <= min(self.cx, self.cy, width - 1 - self.cx, height - 1 - self.cy))
        self.rays = self.ground_rays() if self.ring_ready and self.ground_ready() else ()

    def ground_ready(self):
        p = self.points
        if len(p) < 2 or not (0 <= p[0][1] <= LINE_CLEARANCE_CM and p[-1][1] >= LOOKAHEAD_CM):
            return False
        radial_sign = 1 if p[-1][0] > p[0][0] else -1
        return (all(self.inner <= radius <= self.outer for radius, _ in p)
                and all(radial_sign * (r1 - r0) > 0 and d1 > d0
                        for (r0, d0), (r1, d1) in zip(p, p[1:])))

    def pixels_on_ring(self, x, y):
        squared_radius = (x - self.cx) ** 2 + (y - self.cy) ** 2
        return self.inner ** 2 <= squared_radius <= self.outer ** 2

    def bearing(self, x, y):
        raw = image_angle(x - self.cx, y - self.cy)
        return ((raw - cfg.FRONT_DEG) * (1 if cfg.CLOCKWISE else -1)) % 360

    def ground_rays(self):
        # Precompute pixel coordinates and distances once, through the calibrated range.
        rays = []
        for i in range(RAY_COUNT):
            bearing = 360 * i / RAY_COUNT
            angle = math.radians(cfg.FRONT_DEG + bearing * (1 if cfg.CLOCKWISE else -1))
            dx, dy = math.sin(angle), -math.cos(angle)
            samples = []
            for (r0, d0), (r1, d1) in zip(self.points, self.points[1:]):
                steps = math.ceil(abs(r1 - r0))
                for step in range(steps + 1):
                    fraction = step / steps
                    distance = d0 + (d1 - d0) * fraction
                    if distance > LOOKAHEAD_CM:
                        break
                    radius = r0 + (r1 - r0) * fraction
                    x, y = round(self.cx + radius * dx), round(self.cy + radius * dy)
                    if not samples or (x, y) != samples[-1][:2]:
                        samples.append((x, y, distance))
            rays.append((bearing, samples))
        return rays


def find_color_blob(img, mirror, thresholds, min_pixels):
    counts = [0, 0, 0]  # Color components, too small, rejected by mirror geometry.
    best = None
    best_pixels = 0
    roi = None
    if mirror.ring_ready:
        radius = math.ceil(mirror.outer)
        x, y = round(mirror.cx), round(mirror.cy)
        left, top = max(0, x - radius), max(0, y - radius)
        roi = (left, top, min(mirror.width, x + radius + 1) - left,
               min(mirror.height, y + radius + 1) - top)
    for blob in img.find_blobs(thresholds, roi=roi, pixels_threshold=1, area_threshold=1):
        counts[0] += 1
        pixels = blob_value(blob, "pixels")
        if pixels < min_pixels:
            counts[1] += 1
            continue
        x, y = blob_value(blob, "cx"), blob_value(blob, "cy")
        corners = blob_value(blob, "corners")
        if mirror.ring_ready and (not mirror.pixels_on_ring(x, y)
                                  or not all(mirror.pixels_on_ring(*p) for p in corners)):
            counts[2] += 1
            continue
        if pixels > best_pixels:
            best, best_pixels = blob, pixels
    return best, counts


def goal_geometry(blob, mirror):
    center = mirror.bearing(blob_value(blob, "cx"), blob_value(blob, "cy"))
    offsets = [angle_error(mirror.bearing(x, y), center) for x, y in blob_value(blob, "corners")]
    lower, upper = min(offsets), max(offsets)
    # shortcut: contour corners approximate the opening; replace with tracked posts if needed.
    aim = (abs(angle_error(center)) <= GOAL_MAX_AIM_DEG
           and 0 < upper - lower <= 90
           and lower + GOAL_EDGE_MARGIN_DEG <= angle_error(0, center)
           <= upper - GOAL_EDGE_MARGIN_DEG)
    return center, aim


class GoalTrack:
    def __init__(self):
        self.last = None
        self.count = 0
        self.aim_count = 0

    def update(self, goal):
        if goal is None:
            self.last, self.count, self.aim_count = None, 0, 0
            return None
        angle, aimed = goal
        continuous = self.last is not None and abs(angle_error(angle, self.last)) <= GOAL_MAX_JUMP_DEG
        self.count = self.count + 1 if continuous else 1
        self.aim_count = (self.aim_count + 1 if continuous else 1) if aimed else 0
        self.last = angle
        if self.count >= GOAL_STABLE_FRAMES:
            return angle, self.aim_count >= GOAL_STABLE_FRAMES
        return None


def scan_ray(samples, color_at):
    green_run = 0
    ground_seen = False
    previous = None
    for x, y, distance in samples:
        color = color_at(x, y)
        if previous is not None and previous[0] == color:
            return ground_seen, previous
        previous = (color, distance, x, y) if green_run >= 2 and color in ("WHITE", "BLACK") else None
        green_run = green_run + 1 if color == "GREEN" else 0
        ground_seen = ground_seen or green_run >= 2
    return ground_seen, None


def scan_field(img, mirror):
    def color_at(x, y):
        lab = image.rgb_to_lab(img.get_pixel((x, y), rgbtuple=True))
        if in_lab(lab, cfg.GREEN):
            return "GREEN"
        if in_lab(lab, cfg.WHITE):
            return "WHITE"
        if in_lab(lab, cfg.BLACK):
            return "BLACK"
        return None

    visible = set()
    boundaries = []
    for bearing, samples in mirror.rays:
        ground, edge = scan_ray(samples, color_at)
        if ground:
            visible.add(bearing)
        if edge is not None:
            boundaries.append((bearing,) + edge)
    return visible, boundaries


def decide_navigation(ready, visible, boundaries, goal):
    if not ready:
        return 0, 0, 0, 0, MODE_WAIT, FLAG_OMNI
    away_x = away_y = 0.0
    boundary_near = False
    for bearing, color, distance, _, _ in boundaries:
        clearance = LINE_CLEARANCE_CM + (OUTER_CM if color == "BLACK" else 0)
        if distance <= clearance:
            boundary_near = True
            angle = math.radians(bearing)
            weight = clearance - distance + 1
            away_x -= math.sin(angle) * weight
            away_y -= math.cos(angle) * weight
    if boundary_near:
        length = math.sqrt(away_x ** 2 + away_y ** 2)
        vx, vy = (round(35 * away_x / length), round(35 * away_y / length)) if length >= 1 else (0, 0)
        return vx, vy, 0, 0, MODE_AVOID, FLAG_OMNI | FLAG_VIRTUAL_LINE
    step = 360 / RAY_COUNT
    if len(visible) < RAY_COUNT // 2 or not {360 - step, 0, step}.issubset(visible):
        return 0, 0, 0, 0, MODE_WAIT, FLAG_OMNI
    if goal is None:
        return 0, 0, 0, 0, MODE_TRACK_IR, FLAG_OMNI
    angle, aimed = goal
    turn = round(max(-35, min(35, -0.6 * angle_error(angle))))
    return 0, 0, 0 if aimed else turn, int(aimed), MODE_AIMED if aimed else MODE_ALIGN, FLAG_OMNI


def make_nav_packet(vx, vy, turn, aimed, mode, flags):
    packet = bytearray((0x20, vx & 0xff, vy & 0xff, turn & 0xff, aimed, mode, flags, 0))
    for byte in packet[:7]:
        packet[7] ^= byte
    return packet


def draw_mirror(img, mirror):
    cx, cy = round(mirror.cx), round(mirror.cy)
    img.draw_cross((cx, cy), color=(255, 255, 255))
    if mirror.ring_ready:
        for radius in (mirror.inner, mirror.outer):
            img.draw_circle((cx, cy, round(radius)), color=(255, 255, 255))
        front = math.radians(cfg.FRONT_DEG)
        end_x = round(cx + mirror.outer * math.sin(front))
        end_y = round(cy - mirror.outer * math.cos(front))
        img.draw_line((cx, cy, end_x, end_y), color=(255, 255, 255))


def draw_goal(img, blob, name):
    img.draw_edges(blob_value(blob, "corners"), color=GOAL_COLORS[name], thickness=3)


def main_loop(send_uart=True, display=False):
    init_camera()
    frame = sensor.snapshot()
    mirror = Mirror(frame.width(), frame.height())
    target = cfg.TARGET_GOAL
    ready = bool(mirror.rays and target in GOAL_COLORS)
    uart = pyb.UART(3, 9600) if send_uart else None
    track = GoalTrack()
    last_tx = time.ticks_ms()
    while True:
        started = time.ticks_ms()
        img = sensor.snapshot()
        visible, boundaries = scan_field(img, mirror) if mirror.rays else ((), [])
        goal = None
        detections = []
        for color, thresholds in (("YELLOW", cfg.YELLOW), ("BLUE", cfg.BLUE)):
            if not display and (not ready or color != target):
                continue
            blob, _ = find_color_blob(img, mirror, thresholds, cfg.GOAL_MIN_PIXELS)
            if blob is not None:
                if ready and color == target:
                    goal = goal_geometry(blob, mirror)
                if display:
                    detections.append((blob, color))
        now = time.ticks_ms()
        fresh = time.ticks_diff(now, started) <= MAX_FRAME_AGE_MS
        command = decide_navigation(ready and fresh, visible, boundaries, track.update(goal if fresh else None))
        if uart is not None and time.ticks_diff(now, last_tx) >= STREAM_INTERVAL_MS:
            uart.write(make_nav_packet(*command))
            last_tx = now
        if display:
            for blob, color in detections:
                draw_goal(img, blob, color)
            draw_mirror(img, mirror)
            for _, color, _, x, y in boundaries:
                img.draw_cross((x, y), color=(255, 80, 80) if color == "BLACK" else (255, 255, 255))
            img.draw_string((2, 2), "TARGET: {}  MODE: {}".format(target, MODES[command[4]]),
                            color=(255, 255, 255))


if __name__ == "__main__":
    main_loop(send_uart=True, display=True)
