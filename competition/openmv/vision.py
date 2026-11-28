"""Floor/line detection, goal recognition, and C6 6C UART payload."""
import math
from array import array
import camera_calibration as cfg

try:
    import micropython
except ImportError:
    class micropython:
        viper = staticmethod(lambda function: function)

    def ptr8(buffer):
        return buffer

    def ptr16(buffer):
        return memoryview(buffer).cast("H")

    def ptr32(buffer):
        return buffer


PRODUCTION_API = 2
CLEARANCE_CM = 25
RELEASE_CM = 30
ESCAPE_MEMORY_MS = 200
LINE_NEIGHBOR_CM = 8
BLOCK_CM = 35
LOOKAHEAD_CM = 50
MAX_FRAME_MS = 100
GOAL_TTL_MS = 300
SECTOR_WINDOWS = tuple(tuple(b for b in range(0, 360, 15)
                             if abs((b - i * 22.5 + 180) % 360 - 180) <= 30)
                       for i in range(16))


def difference(a, b):
    return abs((a - b + 180) % 360 - 180)


class Mirror:
    def __init__(self, width, height, window=None):
        self.rays = []
        self.roi = None
        radii = cfg.GROUND_POINTS
        if not cfg.MIRROR_CENTER or not cfg.MIRROR_RING or not radii or cfg.FRONT_DEG is None:
            return
        if window is None:
            ox = oy = 0
            expected_w, expected_h = cfg.IMAGE_SIZE
        else:
            ox, oy, expected_w, expected_h = window
        self.cx, self.cy = cfg.MIRROR_CENTER[0] - ox, cfg.MIRROR_CENTER[1] - oy
        self.inner, self.outer = cfg.MIRROR_RING
        pairs = tuple(zip(radii, radii[1:]))
        if (width, height) != (expected_w, expected_h) or not (0 <= self.inner < self.outer <=
                                                    min(self.cx, self.cy, width - 1 - self.cx, height - 1 - self.cy)):
            return
        radius = math.ceil(self.outer)
        self.roi = (round(self.cx) - radius, round(self.cy) - radius, radius * 2 + 1, radius * 2 + 1)
        self.size = (width, height)
        if len(radii) < 2 or not (0 < radii[0][1] <= CLEARANCE_CM and radii[-1][1] >= LOOKAHEAD_CM):
            return
        sign = 1 if radii[-1][0] > radii[0][0] else -1
        if (not all(self.inner <= r <= self.outer for r, _ in radii) or
                not all((r1 - r0) * sign > 0 and d1 > d0 for (r0, d0), (r1, d1) in pairs)):
            return
        for bearing in range(0, 360, 15):
            angle = math.radians(cfg.FRONT_DEG + bearing * (1 if cfg.CLOCKWISE else -1))
            dx, dy = math.sin(angle), -math.cos(angle)
            samples = []
            for (r0, d0), (r1, d1) in pairs:
                steps = math.ceil(abs(r1 - r0))
                for step in range(steps + 1):
                    f = step / steps
                    cm = d0 + (d1 - d0) * f
                    if cm > LOOKAHEAD_CM:
                        break
                    r = r0 + (r1 - r0) * f
                    x, y = round(self.cx + r * dx), round(self.cy + r * dy)
                    if not samples or (x, y) != samples[-1][:2]:
                        samples.append((x, y, cm))
            if len(samples) < 4 or samples[2][2] > CLEARANCE_CM or samples[-1][2] < 48:
                self.rays = []
                return
            self.rays.append((bearing, samples))

    def bearing(self, x, y):
        raw = math.degrees(math.atan2(x - self.cx, self.cy - y))
        return ((raw - cfg.FRONT_DEG) * (1 if cfg.CLOCKWISE else -1)) % 360


def in_lab(lab, limits):
    l, a, b = lab
    return limits[0] <= l <= limits[1] and limits[2] <= a <= limits[3] and limits[4] <= b <= limits[5]


def classify(lab):
    if in_lab(lab, cfg.GREEN) or in_lab(lab, cfg.GREEN_LOW_SATURATION):
        return 1
    if in_lab(lab, cfg.WHITE):
        return 2
    return 0


@micropython.viper
def fill_row(frame, base: int):
    pixels = ptr16(frame)
    i = 0
    while i < 256:
        pixels[i] = base + i
        i += 1


@micropython.viper
def pixel_code(frame, table, offset: int) -> int:
    return int(ptr8(table)[ptr16(frame)[offset]])


@micropython.viper
def scan_kernel(frame, table, points, span: int) -> int:
    pixels = ptr16(frame)
    lut = ptr8(table)
    packed = ptr32(points)
    i = span & 0xffff
    end = span >> 16
    index = 0
    clear = 1
    white = 0
    while i < end:
        point = int(packed[i])
        color = int(lut[pixels[point >> 1]])
        if point & 1 and color != 1:
            clear = 0
        if color == 2 and white:
            return (index << 1) | clear
        white = int(color == 2)
        i += 1
        index += 1
    return clear


class Scanner:
    def __init__(self, mirror, width, rgb_to_lab):
        self.rays = mirror.rays
        self.width = width
        self.rgb_to_lab = rgb_to_lab
        self.table = None
        self.fast = False
        self.cache = {}
        self.frame_buffer = None
        self.spans = []
        points = []
        for _, samples in self.rays:
            start = len(points)
            for x, y, cm in samples:
                points.append((x + y * width) * 2 + int(cm <= BLOCK_CM))
            self.spans.append(start | (len(points) << 16))
        self.packed = array("i", points)

    def prepare(self, image):
        table = bytearray(65536)
        frame = image.bytearray()
        for base in range(0, 65536, 256):
            fill_row(frame, base)
            for x in range(256):
                table[base + x] = classify(self.rgb_to_lab(image.get_pixel((x % self.width, x // self.width), rgbtuple=True)))
        self.table = table

    def color(self, image, x, y):
        rgb = image.get_pixel((x, y), rgbtuple=True)
        found = self.cache.get(rgb)
        if found is None:
            found = classify(self.rgb_to_lab(rgb))
            if len(self.cache) >= 2048:
                self.cache.clear()
            self.cache[rgb] = found
        return found

    def goal_color(self, image, x, y):
        if self.frame_buffer is not None:
            return pixel_code(self.frame_buffer, self.table, y * self.width + x)
        return self.color(image, x, y)

    def reference(self, image, samples):
        clear, white = 1, False
        for index, (x, y, cm) in enumerate(samples):
            color = self.color(image, x, y)
            if cm <= BLOCK_CM and color != 1:
                clear = 0
            if color == 2 and white:
                return index * 2 + clear
            white = color == 2
        return clear

    def verify(self, image):
        frame = image.bytearray()
        for (_, samples), span in zip(self.rays, self.spans):
            for x, y, _ in samples:
                if pixel_code(frame, self.table, y * self.width + x) != self.color(image, x, y):
                    return False
            if scan_kernel(frame, self.table, self.packed, span) != self.reference(image, samples):
                return False
        return True

    def scan(self, image):
        visible, lines = set(), {}
        frame = self.frame_buffer = image.bytearray() if self.fast else None
        for (bearing, samples), span in zip(self.rays, self.spans):
            result = (scan_kernel(frame, self.table, self.packed, span) if self.fast
                      else self.reference(image, samples))
            if result & 1:
                visible.add(bearing)
            if result >> 1:
                lines[bearing] = samples[(result >> 1) - 1][2]
        return visible, lines


SECTOR_BITS = tuple(sum(1 << (ray // 15) for ray in rays) for rays in SECTOR_WINDOWS)
LINE_BLOCKS = tuple(sum(1 << i for i in range(16) if difference(i * 22.5, ray) < 60)
                    for ray in range(0, 360, 15))
CLOSE_BLOCKS = tuple(sum(1 << i for i in range(16) if difference(i * 22.5, ray) < 105)
                     for ray in range(0, 360, 15))


class LineGuard:
    """Require observed turf in the commanded wedge and keep a brief release hysteresis."""
    def __init__(self, ticks_diff):
        self.ticks_diff = ticks_diff
        self.near = ()
        self.close_ms = 0
        self.escape_fresh = False
        self.clear_frames = 0
        self.confirmed = ()
        self.coverage = self.blocked = self.unknown = self.near_blocked = self.escape_away = 0

    def update(self, visible, lines, now):
        threshold = RELEASE_CM if self.near else CLEARANCE_CM
        # Two white pixels are only a ray candidate. A nearby ray must agree
        # before a new line can trigger an escape; every raw hit still closes approach.
        self.confirmed = tuple(b for b, cm in lines.items() if any(
            abs(lines.get((b + step) % 360, 1000) - cm) <= LINE_NEIGHBOR_CM for step in (-15, 15)))
        raw_close = tuple(b for b, cm in lines.items() if cm <= threshold)
        close = tuple(b for b in raw_close if b in self.confirmed or b in self.near)
        if close:
            # Keep also any earlier rays that are still blind. Freshly visible turf
            # is allowed to drop an older ray once another close ray is measured.
            still_hidden = tuple(b for b in self.near if b not in visible and b not in lines)
            self.near = tuple(set(close + still_hidden))
            self.close_ms, self.clear_frames = now, 0
            self.escape_fresh = True
        elif self.near:
            # Losing sight of the previous line direction is not evidence that it is clear.
            if all(b in visible or (b in lines and lines[b] > RELEASE_CM) for b in self.near):
                self.clear_frames += 1
            else:
                self.clear_frames = 0
            if self.clear_frames >= 2 and self.ticks_diff(now, self.close_ms) >= 100:
                self.near = ()
        blocked = 0
        for bearing, cm in lines.items():
            if cm <= BLOCK_CM:
                blocked |= LINE_BLOCKS[bearing // 15]
        observed = sum(1 << (ray // 15) for ray in visible)
        self.coverage = 0
        for i, window in enumerate(SECTOR_BITS):
            if observed & window == window:
                self.coverage |= 1 << i
        self.near_blocked = 0
        # For an ambiguous close hit only a positively observed, away direction can remain open.
        for b in raw_close:
            if b not in close:
                self.near_blocked |= CLOSE_BLOCKS[b // 15]
        self.blocked = blocked | self.near_blocked
        self.unknown = 0xffff & ~self.coverage
        mask = self.coverage & ~self.blocked
        self.escape_away = 0
        if self.ticks_diff(now, self.close_ms) >= ESCAPE_MEMORY_MS:
            self.escape_fresh = False
        if self.near and self.escape_fresh:
            for bit, bearing in ((1, 0), (256, 180)):
                if all(difference(bearing, b) >= 105 for b in self.near):
                    self.escape_away |= bit
        return mask, self.escape_away & mask, min(lines.items(), key=lambda pair: pair[1]) if lines else None


def find_goal(image, mirror, thresholds, minimum, color_at=None, roi=None, stride=1, report=None, probe=False, blobs=None):
    if mirror.roi is None:
        return None
    best = None
    roi = roi or mirror.roi
    rx, ry, rw, rh = roi
    fx, fy, fw, fh = mirror.roi
    seen = small = clipped = ring = shape = turf = accepted = 0
    winning_box = winning_center = None
    native_min = 1 if probe else minimum
    if blobs is None:
        blobs = image.find_blobs(thresholds, roi=roi, pixels_threshold=native_min,
                                area_threshold=native_min, x_stride=stride, y_stride=1, merge=True, margin=2)
    for blob in blobs:
        seen += 1
        rect = blob.rect
        legacy = callable(rect)
        x, y, w, h = rect() if legacy else rect
        if ((rx > fx and x <= rx) or (ry > fy and y <= ry) or
                (rx + rw < fx + fw and x + w >= rx + rw) or
                (ry + rh < fy + fh and y + h >= ry + rh)):
            clipped += 1
            continue
        if legacy:
            cx, cy, pixels = blob.cx(), blob.cy(), blob.pixels()
        else:
            cx, cy, pixels = blob.cx, blob.cy, blob.pixels
        if pixels < minimum or w * h < minimum:
            small += 1
            continue
        dx, dy = cx - mirror.cx, cy - mirror.cy
        radius = math.sqrt(dx * dx + dy * dy)
        if not mirror.inner <= radius <= mirror.outer or not radius or not w or not h:
            ring += 1
            continue
        density = pixels / (w * h)
        if density < cfg.GOAL_MIN_DENSITY:
            shape += 1
            continue
        bearing = mirror.bearing(cx, cy)
        corners = ((x, y), (x + w, y), (x, y + h), (x + w, y + h))
        offsets = [(mirror.bearing(px, py) - bearing + 180) % 360 - 180 for px, py in corners]
        width = max(offsets) - min(offsets)
        height = max(1, (abs(w * dx) + abs(h * dy)) / radius)
        if not cfg.GOAL_MIN_WIDTH_DEG <= width <= cfg.GOAL_MAX_WIDTH_DEG:
            shape += 1
            continue
        quality = min(1, pixels / (4 * minimum)) * min(1, 0.5 + density)
        if math.radians(width) * radius / height < cfg.GOAL_MIN_ASPECT:
            shape += 1
            quality *= 0.55
        if color_at is not None:
            beyond = radius + height / 2 + 3
            if beyond <= mirror.outer:
                theta = math.atan2(dy, dx)
                angles = (theta - 0.07, theta, theta + 0.07)
                if sum(color_at(image, round(mirror.cx + beyond * math.cos(a)),
                                round(mirror.cy + beyond * math.sin(a))) == 1 for a in angles) >= 2:
                    turf += 1
                    quality *= 0.5
        accepted += 1
        candidate = (round(bearing) % 360, max(1, round(100 * quality)))
        if best is None or candidate[1] > best[1]:
            best = candidate
            winning_box, winning_center = (x, y, w, h), (cx, cy)
    if report is not None:
        report[:] = [seen, small, clipped, ring, shape, turf, accepted, winning_box, winning_center]
    return best


GOAL_THRESHOLDS = cfg.YELLOW + cfg.BLUE
YELLOW_CODE = (1 << len(cfg.YELLOW)) - 1
BLUE_CODE = ((1 << len(GOAL_THRESHOLDS)) - 1) ^ YELLOW_CODE


def goal_color(blob):
    code = blob.code
    code = code() if callable(code) else code
    yellow = bool(code & YELLOW_CODE)
    blue = bool(code & BLUE_CODE)
    return -1 if yellow == blue else (0 if yellow else 1)


def same_goal_color(first, second):
    first_color = goal_color(first)
    return first_color >= 0 and first_color == goal_color(second)


def find_goals(image, mirror, roi, stride, color_at, reports):
    minima = (cfg.YELLOW_MIN_PIXELS, cfg.BLUE_MIN_PIXELS)
    minimum = min(minima)
    grouped = ([], [])
    if mirror.roi is None:
        return None, None
    for blob in image.find_blobs(GOAL_THRESHOLDS, roi=roi, pixels_threshold=minimum, area_threshold=minimum,
                                 x_stride=stride, y_stride=1, merge=True, margin=2, merge_cb=same_goal_color):
        color = goal_color(blob)
        if color >= 0:
            grouped[color].append(blob)
    return (find_goal(image, mirror, None, minima[0], color_at, roi, stride, reports[0], blobs=grouped[0]),
            find_goal(image, mirror, None, minima[1], color_at, roi, stride, reports[1], blobs=grouped[1]))


def goal_fields(observation, now, ticks_diff):
    if observation is None:
        return 65535, 0, 255
    (bearing, quality), seen = observation
    age = ticks_diff(now, seen)
    return (bearing, quality, (age + 9) // 10) if 0 <= age < GOAL_TTL_MS else (65535, 0, 255)


def crc_entry(value):
    for _ in range(8):
        value = ((value << 1) ^ (0x1021 if value & 0x8000 else 0)) & 65535
    return value


CRC_TABLE = array("H", (crc_entry(i << 8) for i in range(256)))


def crc16(data, length=None):
    crc = 65535
    for i in range(len(data) if length is None else length):
        crc = ((crc << 8) & 65535) ^ CRC_TABLE[(crc >> 8) ^ data[i]]
    return crc


def packet(sequence, ready, escaping, mask, escape, nearest, goals, frame_ms, data=None):
    slow = frame_ms > MAX_FRAME_MS
    if slow or not ready:
        mask = escape = 0
    flags = int(ready) | (int(slow) << 1) | (int(escaping) << 2) | (int(cfg.TARGET_GOAL == "BLUE") << 3)
    angle, cm = (65535, 255) if nearest is None else (round(nearest[0]) % 360, round(nearest[1]))
    if data is None:
        data = bytearray(23)
    data[0], data[1], data[2], data[3] = 0xc6, 0x6c, sequence, flags
    data[4], data[5], data[6], data[7] = mask >> 8, mask & 255, escape >> 8, escape & 255
    data[8], data[9], data[10], data[11] = angle >> 8, angle & 255, cm, min(frame_ms, 255)
    for index, (bearing, quality, age) in enumerate(goals):
        pos = 12 + index * 4
        data[pos], data[pos + 1], data[pos + 2], data[pos + 3] = bearing >> 8, bearing & 255, quality, age
    data[20] = 1
    check = crc16(data, 21)
    data[21], data[22] = check >> 8, check & 255
    return data
