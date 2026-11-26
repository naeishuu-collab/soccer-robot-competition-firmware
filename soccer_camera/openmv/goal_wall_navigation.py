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

LINE_CLEARANCE_CM = 25  # Provisional, verify stopping distance before floor use.
LOOKAHEAD_CM = 50
RAY_COUNT = 24
MIN_FLOOR_RAYS = RAY_COUNT // 2
RAY_STEP_DEG = 360 / RAY_COUNT
SECTOR_STEP_DEG = 45
SECTOR_RAYS = tuple(frozenset((bearing + offset) % 360 for offset in (-30, -15, 0, 15, 30))
                    for bearing in range(0, 360, SECTOR_STEP_DEG))
SECTOR_DIRECTIONS = tuple((math.sin(math.radians(bearing)), math.cos(math.radians(bearing)))
                          for bearing in range(0, 360, SECTOR_STEP_DEG))
WALL_NEIGHBOR_GAP_CM = 8
STREAM_INTERVAL_MS = 50
MAX_FRAME_AGE_MS = 100

MODE_WAIT = 0
MODE_AVOID = 1
MODE_TRACK_IR = 2
FLAG_VIRTUAL_LINE = 0x01
FLAG_DIRECTIONAL = 0x40  # Requires the matching MAIN firmware.

GOAL_COLORS = {"YELLOW": (255, 255, 0), "BLUE": (0, 110, 255)}
MODES = ("WAIT", "AVOID", "TRACK IR")


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


def find_color_blob(img, mirror, thresholds, min_pixels, accept=None):
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
        if mirror.ring_ready and not mirror.pixels_on_ring(x, y):
            counts[2] += 1
            continue
        if accept is not None and not accept(blob):
            continue
        if pixels > best_pixels:
            best, best_pixels = blob, pixels
    return best, counts


def scan_ray(samples, color_at, include_wall=False):
    green_run = 0
    ground_seen = False
    previous = None
    white_bridge = False
    for index, (x, y, distance) in enumerate(samples):
        color = color_at(x, y)
        if distance <= LINE_CLEARANCE_CM and color != "GREEN":
            ground_seen = False
        if previous is not None and previous[0] == color:
            return ground_seen, previous
        follows_turf = green_run >= 2 or (white_bridge and color == "WHITE")
        is_boundary = color == "WHITE" or (include_wall and color == "BLACK")
        previous = (color, distance, x, y) if follows_turf and is_boundary else None
        # A mixed edge pixel may be neither turf nor white.
        white_bridge = color is None and green_run >= 2
        green_run = green_run + 1 if color == "GREEN" else 0
        if index == 1:
            ground_seen = green_run >= 2
    return ground_seen, None


def scan_field(img, mirror, white_candidates=None, include_wall=False):
    low_saturation = getattr(cfg, "GREEN_LOW_SATURATION", None)

    def color_at(x, y):
        lab = image.rgb_to_lab(img.get_pixel((x, y), rgbtuple=True))
        if in_lab(lab, cfg.GREEN) or (low_saturation and in_lab(lab, low_saturation)):
            return "GREEN"
        if in_lab(lab, cfg.WHITE):
            if white_candidates is not None:
                white_candidates.add(bearing)
            return "WHITE"
        if in_lab(lab, cfg.BLACK):
            return "BLACK"
        return None

    visible = set()
    candidates = {}
    for bearing, samples in mirror.rays:
        ground, edge = scan_ray(samples, color_at, include_wall)
        if ground:
            visible.add(bearing)
        if edge is not None:
            candidates[bearing] = edge
    boundaries = []
    for bearing, edge in candidates.items():
        color, distance, _, _ = edge
        if color == "BLACK":
            # An isolated dark transition can come from a cable or a shadow.
            neighbors = (candidates.get((bearing - RAY_STEP_DEG) % 360),
                         candidates.get((bearing + RAY_STEP_DEG) % 360))
            if not any(other is not None and other[0] == color
                       and abs(distance - other[1]) <= WALL_NEIGHBOR_GAP_CM for other in neighbors):
                continue
        boundaries.append((bearing,) + edge)
    return visible, boundaries


def setup_wait_reason(mirror):
    if cfg.IMAGE_SIZE != (mirror.width, mirror.height):
        return "SET IMAGE_SIZE {}x{}".format(mirror.width, mirror.height)
    if not mirror.ring_ready:
        return "CHECK MIRROR_CENTER/RING"
    # The first possible edge follows two distinct green sample pixels.
    if not mirror.rays or any(len(s) < 4 or s[2][2] > LINE_CLEARANCE_CM for _, s in mirror.rays):
        return "CHECK GROUND_POINTS"
    return None


def wait_detail(setup_reason, frame_ms, visible):
    if setup_reason:
        return setup_reason
    if frame_ms > MAX_FRAME_AGE_MS:
        return "SLOW FRAME {}ms > {}ms".format(frame_ms, MAX_FRAME_AGE_MS)
    return "FLOOR {}/{}  CHECK PATH".format(len(visible), RAY_COUNT)


def decide_navigation(ready, visible, boundaries):
    stop = (0, 0, 0, MODE_WAIT)
    if not ready:
        return stop

    away_x = away_y = 0.0
    blocked = {}
    for bearing, color, distance, _, _ in boundaries:
        if color == "WHITE" and distance <= LINE_CLEARANCE_CM:
            angle = math.radians(bearing)
            toward_x, toward_y = math.sin(angle), math.cos(angle)
            blocked[bearing] = (toward_x, toward_y)
            weight = LINE_CLEARANCE_CM - distance + 1
            away_x -= toward_x * weight
            away_y -= toward_y * weight

    clear = set(visible).difference(blocked)
    mask = 0
    for sector, rays in enumerate(SECTOR_RAYS):
        if rays.issubset(clear):
            mask |= 1 << sector

    if blocked:
        retreat_mask = 0
        best = None
        best_score = 0
        for sector, (dx, dy) in enumerate(SECTOR_DIRECTIONS):
            if not mask & (1 << sector):
                continue
            # The weighted sum ranks choices, but no choice may approach a nearby edge.
            if any(toward_x * dx + toward_y * dy > 1e-6
                   for toward_x, toward_y in blocked.values()):
                continue
            score = away_x * dx + away_y * dy
            if score <= 1e-6:
                continue
            retreat_mask |= 1 << sector
            if score > best_score:
                best = (round(35 * dx), round(35 * dy))
                best_score = score
        return stop if best is None else (best[0], best[1], retreat_mask, MODE_AVOID)

    if len(visible) < MIN_FLOOR_RAYS or not mask:
        return stop
    return 0, 0, mask, MODE_TRACK_IR


def make_nav_packet(vx, vy, sectors, mode, sequence):
    flags = FLAG_DIRECTIONAL | (FLAG_VIRTUAL_LINE if mode == MODE_AVOID else 0)
    packet = bytearray((0x20, vx & 0xff, vy & 0xff, sectors, sequence, mode, flags, 0))
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


def draw_floor_scan(img, mirror):
    # Show actual sampled radii on the four cardinal rays.
    for bearing, samples in mirror.rays:
        if bearing % 90 == 0 and samples:
            x0, y0, _ = samples[0]
            x1, y1, _ = samples[-1]
            img.draw_line((x0, y0, x1, y1), color=(0, 220, 220))


def draw_goal(img, blob, name):
    img.draw_edges(blob_value(blob, "corners"), color=GOAL_COLORS[name], thickness=3)


def main_loop(send_uart=True, display=False):
    init_camera()
    frame = sensor.snapshot()
    mirror = Mirror(frame.width(), frame.height())
    target = cfg.TARGET_GOAL
    setup_reason = setup_wait_reason(mirror)
    goal_thresholds = (("YELLOW", cfg.YELLOW), ("BLUE", cfg.BLUE))
    uart = pyb.UART(3, 9600) if send_uart else None
    last_tx = time.ticks_ms()
    last_command = None
    sequence = 0
    while True:
        started = time.ticks_ms()
        img = sensor.snapshot()
        white_candidates = set() if display else None
        visible, boundaries = scan_field(img, mirror, white_candidates) if mirror.rays else ((), [])
        now = time.ticks_ms()
        frame_ms = time.ticks_diff(now, started)
        command = decide_navigation(setup_reason is None and frame_ms <= MAX_FRAME_AGE_MS,
                                    visible, boundaries)
        mode = command[3]
        if uart is not None and (command != last_command
                                 or time.ticks_diff(now, last_tx) >= STREAM_INTERVAL_MS):
            vx, vy, sectors, _ = command
            uart.write(make_nav_packet(vx, vy, sectors, mode, sequence))
            sequence = (sequence + 1) & 0xff
            last_tx = now
            last_command = command
        if display:
            # Detect both colors before drawing so overlays never enter the other search.
            detections = [(name, find_color_blob(img, mirror, thresholds, cfg.GOAL_MIN_PIXELS)[0])
                          for name, thresholds in goal_thresholds]
            for name, blob in detections:
                if blob is not None:
                    draw_goal(img, blob, name)
            draw_mirror(img, mirror)
            draw_floor_scan(img, mirror)
            white_distances = []
            for _, _, distance, x, y in boundaries:
                white_distances.append(distance)
                img.draw_cross((x, y), color=(255, 255, 255))
            status = "TARGET: {}  MODE: {}".format(target, MODES[mode])
            img.draw_string((3, 3), status, color=(0, 0, 0))
            img.draw_string((2, 2), status, color=(255, 255, 255))
            if mode == MODE_WAIT:
                detail = wait_detail(setup_reason, frame_ms, visible)
                img.draw_string((3, 15), detail, color=(0, 0, 0))
                img.draw_string((2, 14), detail, color=(255, 255, 255))
            if white_distances:
                line_status = "WHITE LINE: DETECTED ({}, {:.0f} cm)".format(
                    len(white_distances), min(white_distances))
            elif white_candidates:
                line_status = "WHITE LINE: UNCONFIRMED ({})".format(len(white_candidates))
            elif mirror.rays:
                line_status = "WHITE LINE: NOT DETECTED"
            else:
                line_status = "WHITE LINE: CHECK CALIBRATION"
            img.draw_string((3, 27), line_status, color=(0, 0, 0))
            img.draw_string((2, 26), line_status, color=(255, 255, 255))


if __name__ == "__main__":
    main_loop(send_uart=True, display=True)
