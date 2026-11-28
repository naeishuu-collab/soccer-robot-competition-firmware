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


PRODUCTION_API = 6
CLEARANCE_CM = 25
RELEASE_CM = 30
ESCAPE_MEMORY_MS = 200
LINE_NEIGHBOR_CM = 8
STRIPE_MAX_CM = 8
BLOCK_CM = 35
LOOKAHEAD_CM = 50
MAX_FRAME_MS = 100
GOAL_TTL_MS = 300
SECTOR_WINDOWS = tuple(tuple(b for b in range(0, 360, 15)
                             if abs((b - i * 22.5 + 180) % 360 - 180) <= 30)
                       for i in range(16))


def difference(a, b):
    return abs((a - b + 180) % 360 - 180)


def safety_distances(profile):
    if profile is None:
        return CLEARANCE_CM, BLOCK_CM
    body, speed, latency, brake, margin = profile
    values = (body, speed, latency, brake, margin)
    if not all(0 < value < 10000 for value in values):
        raise ValueError("Invalid measured motion profile")
    clearance = max(CLEARANCE_CM, body + speed * latency / 1000 + brake + margin)
    block = max(BLOCK_CM, clearance + RELEASE_CM - CLEARANCE_CM)
    if block > LOOKAHEAD_CM:
        raise ValueError("Measured stopping envelope exceeds camera horizon")
    return clearance, block


def contains_point(x, y, polygon):
    inside = False
    x0, y0 = polygon[-1]
    for x1, y1 in polygon:
        cross = (x - x0) * (y1 - y0) - (y - y0) * (x1 - x0)
        if cross == 0 and min(x0, x1) <= x <= max(x0, x1) and min(y0, y1) <= y <= max(y0, y1):
            return True
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
        x0, y0 = x1, y1
    return inside


class Mirror:
    def __init__(self, width, height, window=None, scale=1):
        self.rays = []
        self.roi = None
        self.blind_pixels = set()
        self.inner_context = []
        self.self_outlines = []
        self.horizon = LOOKAHEAD_CM
        self.distance_segments = ()
        radii = tuple((r * scale, cm) for r, cm in cfg.GROUND_POINTS)
        if not cfg.MIRROR_CENTER or not cfg.MIRROR_RING or not radii or cfg.FRONT_DEG is None:
            return
        if window is None:
            ox = oy = 0
            expected_w, expected_h = (round(cfg.IMAGE_SIZE[0] * scale), round(cfg.IMAGE_SIZE[1] * scale))
        else:
            ox, oy, expected_w, expected_h = window
        self.cx, self.cy = cfg.MIRROR_CENTER[0] * scale - ox, cfg.MIRROR_CENTER[1] * scale - oy
        self.inner, self.outer = (r * scale for r in cfg.MIRROR_RING)
        self.distance_segments = tuple(zip(radii, radii[1:]))
        pairs = self.distance_segments
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
        rectangles = cfg.LINE_SELF_MASKS
        polygons = cfg.LINE_SELF_POLYGONS
        if any(w <= 0 or h <= 0 for _, _, w, h in rectangles) or any(len(p) < 3 for p in polygons):
            raise ValueError("Invalid self-occlusion calibration")
        for x, y, w, h in rectangles:
            polygon = ((x, y), (x+w, y), (x+w, y+h), (x, y+h))
            self.self_outlines.append(tuple((round(px * scale - ox), round(py * scale - oy)) for px, py in polygon))
        for polygon in polygons:
            self.self_outlines.append(tuple((round(px * scale - ox), round(py * scale - oy)) for px, py in polygon))

        def is_blind(x, y):
            qx, qy = (x + ox) / scale, (y + oy) / scale
            return (any(bx <= qx < bx + bw and by <= qy < by + bh for bx, by, bw, bh in rectangles)
                    or any(contains_point(qx, qy, polygon) for polygon in polygons))

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
            if len(samples) < 4 or samples[1][2] > CLEARANCE_CM or samples[-1][2] < 48:
                self.rays = []
                return
            self.rays.append((bearing, samples))
            # This is optical context only. It is outside the distance table.
            r = radii[0][0] - 2 * sign
            x, y = round(self.cx + r * dx), round(self.cy + r * dy)
            self.inner_context.append(x + y * width if self.inner <= r <= self.outer
                                      and not is_blind(x, y) else -1)
            self.horizon = min(self.horizon, samples[-1][2])
            for x, y, _ in samples:
                if is_blind(x, y):
                    self.blind_pixels.add(x + y * width)

    def pixel_radius(self, x, y):
        return math.sqrt((x - self.cx) ** 2 + (y - self.cy) ** 2)

    def distance_at_radius(self, radius):
        for (r0, cm0), (r1, cm1) in self.distance_segments:
            if min(r0, r1) <= radius <= max(r0, r1) and r0 != r1:
                return (cm0 + (radius - r0) * (cm1 - cm0) / (r1 - r0), (r0, cm0, r1, cm1))
        return None, None

    def bearing(self, x, y):
        raw = math.degrees(math.atan2(x - self.cx, self.cy - y))
        return ((raw - cfg.FRONT_DEG) * (1 if cfg.CLOCKWISE else -1)) % 360


def calibration_targets(image, mirror):
    """Bench only. Use one known-distance target at a time, within 30 degrees of the front."""
    if not mirror.roi:
        return ()
    candidates = []
    for blob in image.find_blobs([cfg.WHITE], roi=mirror.roi, pixels_threshold=4, area_threshold=4,
                                 x_stride=1, y_stride=1, merge=False):
        rect = blob.rect
        if callable(rect):
            box, x, y, pixels = rect(), blob.cx(), blob.cy(), blob.pixels()
        else:
            box, x, y, pixels = rect, blob.cx, blob.cy, blob.pixels
        radius = mirror.pixel_radius(x, y)
        if mirror.inner <= radius <= mirror.outer and difference(mirror.bearing(x, y), 0) <= 30:
            distance, segment = mirror.distance_at_radius(radius)
            candidates.append((pixels, box, (x, y), radius, distance, segment))
    candidates.sort(reverse=True)
    return candidates[:3]


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


def line_code(lab, candidate=False):
    strict = in_lab(lab, cfg.WHITE_CANDIDATE)
    white = strict if candidate else in_lab(lab, cfg.WHITE)
    exposure = 1 if lab[0] < cfg.MIN_VISIBLE_L else 2 if lab[0] > cfg.MAX_VISIBLE_L else 0
    return int(white) | (int(strict) << 1) | (exposure << 2)


@micropython.viper
def scan_kernel(frame, table, points, spans, results, count: int):
    pixels = ptr16(frame)
    lut = ptr8(table)
    packed = ptr32(points)
    bounds = ptr32(spans)
    output = ptr32(results)
    ray = 0
    while ray < count:
        span = int(bounds[ray])
        i = span & 65535
        stop = span >> 16
        start = i
        first = 0
        length = 0
        reasons = 0
        strict = 0
        active = 0
        while i < stop:
            point = int(packed[i])
            code = 0
            if not (point & 2):
                code = int(lut[pixels[point >> 2]])
            if point & 1:
                if point & 2:
                    reasons |= 4
                else:
                    reasons |= code >> 2
            if code & 2:
                strict = 1
            if code & 1:
                if first == 0:
                    first = i - start + 1
                    active = 1
                if active:
                    length += 1
            else:
                active = 0
            i += 1
        censored = 0
        if first == 1:
            context = int(packed[start - 1])
            if context & 2 or (int(lut[pixels[context >> 2]]) & 13):
                censored = 1
        if first and first + length - 1 == stop - start:
            censored = 1
        output[ray] = int(reasons == 0) | first << 1 | length << 9 | strict << 17 | reasons << 18 | censored << 21
        ray += 1


class Scanner:
    def __init__(self, mirror, width, rgb_to_lab, block_cm=BLOCK_CM, candidate=False):
        self.rays = mirror.rays
        self.width = width
        self.rgb_to_lab = rgb_to_lab
        self.block_cm = block_cm
        self.candidate = candidate
        self.table = None
        self.fast = False
        self.cache = {}
        points, spans = [], []
        for (_, samples), context in zip(self.rays, mirror.inner_context):
            points.append(2 if context < 0 else context * 4)
            start = len(points)
            for x, y, cm in samples:
                offset = x + y * width
                points.append(offset * 4 | int(cm <= block_cm) | (2 if offset in mirror.blind_pixels else 0))
            spans.append(start | (len(points) << 16))
        self.packed = array("i", points)
        self.spans = array("i", spans)
        self.results = array("i", [0] * len(spans))
        self.visible, self.lines, self.runs = set(), {}, {}
        self.strict_hits = 0
        self.reasons = {}
        self.censored = set()

    def prepare(self, image):
        table = bytearray(65536)
        frame = image.bytearray()
        for base in range(0, 65536, 256):
            fill_row(frame, base)
            for x in range(256):
                lab = self.rgb_to_lab(image.get_pixel((x % self.width, x // self.width), rgbtuple=True))
                table[base + x] = line_code(lab, self.candidate)
        self.table = table

    def reference_code(self, image, offset):
        rgb = image.get_pixel((offset % self.width, offset // self.width), rgbtuple=True)
        found = self.cache.get(rgb)
        if found is None:
            found = line_code(self.rgb_to_lab(rgb), self.candidate)
            if len(self.cache) >= 2048:
                self.cache.clear()
            self.cache[rgb] = found
        return found

    def goal_color(self, image, x, y):
        # Green remains an optional color cue for goal confidence only.
        return classify(self.rgb_to_lab(image.get_pixel((x, y), rgbtuple=True)))

    def reference(self, image, span):
        start, stop = span & 65535, span >> 16
        first = length = strict = 0
        reasons, active = 0, False
        for i in range(start, stop):
            point = self.packed[i]
            code = 0 if point & 2 else self.reference_code(image, point >> 2)
            if point & 1:
                reasons |= 4 if point & 2 else code >> 2
            if code & 2:
                strict = 1
            if code & 1:
                if not first:
                    first, active = i - start + 1, True
                if active:
                    length += 1
            else:
                active = False
        censored = False
        if first == 1:
            context = self.packed[start - 1]
            censored = bool(context & 2 or self.reference_code(image, context >> 2) & 13)
        if first and first + length - 1 == stop - start:
            censored = True
        return int(not reasons) | first << 1 | length << 9 | strict << 17 | reasons << 18 | int(censored) << 21

    def verify(self, image):
        frame = image.bytearray()
        for point in self.packed:
            if point & 2:
                continue
            offset = point >> 2
            if pixel_code(frame, self.table, offset) != self.reference_code(image, offset):
                return False
        scan_kernel(frame, self.table, self.packed, self.spans, self.results, len(self.rays))
        return all(result == self.reference(image, span) for result, span in zip(self.results, self.spans))

    def scan(self, image):
        self.visible.clear()
        self.lines.clear()
        self.runs.clear()
        self.reasons.clear()
        self.censored.clear()
        self.strict_hits = 0
        if self.fast:
            scan_kernel(image.bytearray(), self.table, self.packed, self.spans, self.results, len(self.rays))
        for index, (bearing, samples) in enumerate(self.rays):
            result = self.results[index] if self.fast else self.reference(image, self.spans[index])
            if result & 1:
                self.visible.add(bearing)
            self.strict_hits += (result >> 17) & 1
            self.reasons[bearing] = (result >> 18) & 7
            if result & (1 << 21):
                self.censored.add(bearing)
            first = ((result >> 1) & 255) - 1
            if first >= 0:
                length = (result >> 9) & 255
                self.lines[bearing] = samples[first][2]
                self.runs[bearing] = (first, first + length - 1)
        return self.visible, self.lines


SECTOR_BITS = tuple(sum(1 << (ray // 15) for ray in rays) for rays in SECTOR_WINDOWS)
SUSPECT_BLOCKS = tuple(sum(1 << i for i in range(16) if difference(i * 22.5, ray) < 45)
                       for ray in range(0, 360, 15))
LINE_BLOCKS = tuple(sum(1 << i for i in range(16) if difference(i * 22.5, ray) < 60)
                    for ray in range(0, 360, 15))
HELD_APPROACH = tuple(sum(1 << i for i in range(16) if difference(i * 22.5, ray) < 90)
                      for ray in range(0, 360, 15))
CLOSE_BLOCKS = tuple(sum(1 << i for i in range(16) if difference(i * 22.5, ray) < 105)
                     for ray in range(0, 360, 15))


def connected_rays(lines):
    remaining = set(lines)
    groups = []
    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        group = [seed]
        for ray in group:
            for delta in (-15, 15):
                neighbor = (ray + delta) % 360
                if neighbor in remaining and abs(lines[ray] - lines[neighbor]) <= LINE_NEIGHBOR_CM:
                    remaining.remove(neighbor)
                    group.append(neighbor)
        groups.append(group)
    return groups


def observation_reasons(ray_reasons):
    masks = [0, 0, 0]
    for sector, window in enumerate(SECTOR_WINDOWS):
        reason = 0
        for ray in window:
            reason |= ray_reasons.get(ray, 0)
        for i in range(3):
            if reason & (1 << i):
                masks[i] |= 1 << sector
    return tuple(masks)


class LineGuard:
    """Use camera coverage and line geometry; the field surface color is assumed."""
    def __init__(self, ticks_diff, clearance=CLEARANCE_CM, block=BLOCK_CM, body_cm=0):
        self.ticks_diff = ticks_diff
        self.clearance, self.block, self.body_cm = clearance, block, body_cm
        self.release = clearance + RELEASE_CM - CLEARANCE_CM
        self.near = ()
        self.near_history = {}
        self.confirmed = set()
        self.coverage = self.blocked = self.unknown = self.escape_away = 0
        self.present_blocked = self.confirmed_blocked = self.ambiguous_blocked = self.held_blocked = 0
        self.escape_requested = False
        self.regions = []
        self.tracks = []
        self.line_angle = self.line_distance = self.line_xy = None
        self.closing_cm_s = self.ttc_ms = None

    def update(self, visible, lines, now, runs=None, rays=None, censored=()):
        self.confirmed.clear()
        self.regions.clear()
        tracks = []
        self.closing_cm_s = self.ttc_ms = None
        self.line_angle = self.line_distance = self.line_xy = None
        nearest = min(lines.items(), key=lambda item: item[1]) if lines else None
        if nearest:
            self.line_angle, cm = nearest
            if self.line_angle not in censored:
                self.line_distance = cm
            if runs is not None:
                self.line_xy = rays[self.line_angle // 15][1][runs[self.line_angle][0]][:2]
        eligible = lines
        if runs is not None:
            eligible = {}
            for bearing, cm in lines.items():
                first, last = runs[bearing]
                samples = rays[bearing // 15][1]
                if bearing not in censored and samples[last][2] - samples[first][2] <= STRIPE_MAX_CM:
                    eligible[bearing] = cm
        accepted = []
        for group in connected_rays(eligible):
            plausible = len(group) >= 3 or (len(group) == 2 and (
                runs is None or any(runs[b][1] > runs[b][0] for b in group)))
            if plausible:
                accepted.append(group)
                self.confirmed.update(group)
                bearing = min(group, key=lambda ray: lines[ray])
                cm = lines[bearing]
                tracks.append((bearing, cm, now))
                if nearest and nearest[0] in group:
                    # Match only local, unambiguous observations; never mask a new reading.
                    matches = [(old_cm, dt) for old_bearing, old_cm, then in self.tracks
                               for dt in (self.ticks_diff(now, then),)
                               if difference(bearing, old_bearing) <= 22.5 and abs(old_cm - cm) <= 12
                               and 5 <= dt <= 150]
                    if len(matches) == 1:
                        old_cm, dt = matches[0]
                        self.closing_cm_s = (old_cm - cm) * 1000 / dt
                        if self.closing_cm_s > 0:
                            self.ttc_ms = round(max(0, cm - self.body_cm) * 1000 / self.closing_cm_s)
        self.tracks = tracks

        if runs is not None and lines:
            uncertain = {b: cm for b, cm in lines.items() if b not in self.confirmed}
            for groups, confirmed in ((accepted, True), (connected_rays(uncertain), False)):
                for group in groups:
                    xmin, ymin, xmax, ymax = 100000, 100000, -1, -1
                    for bearing in group:
                        first, last = runs[bearing]
                        samples = rays[bearing // 15][1]
                        x0, y0, _ = samples[first]
                        x1, y1, _ = samples[last]
                        xmin, xmax = min(xmin, x0, x1), max(xmax, x0, x1)
                        ymin, ymax = min(ymin, y0, y1), max(ymax, y0, y1)
                    self.regions.append(((xmin, ymin, xmax - xmin + 1, ymax - ymin + 1), confirmed))

        for b in self.confirmed:
            threshold = self.release if b in self.near_history else self.clearance
            if lines[b] <= threshold:
                self.near_history[b] = (now, 0)
        self.held_blocked = 0
        self.escape_requested = False
        for b, (last_seen, clear_frames) in tuple(self.near_history.items()):
            age = self.ticks_diff(now, last_seen)
            clear = b in visible and (b not in lines or (b in eligible and lines[b] > self.release))
            clear_frames = clear_frames + 1 if clear else 0
            if clear_frames >= 2 and self.ticks_diff(now, last_seen) >= 100:
                del self.near_history[b]
            else:
                self.near_history[b] = (last_seen, clear_frames)
                if age < ESCAPE_MEMORY_MS:
                    self.escape_requested = True
                    self.held_blocked |= CLOSE_BLOCKS[b // 15]
                else:
                    self.held_blocked |= HELD_APPROACH[b // 15]
        self.near = tuple(self.near_history)
        self.confirmed_blocked = self.ambiguous_blocked = 0
        for bearing, cm in lines.items():
            if cm <= self.block:
                index = bearing // 15
                if bearing in self.confirmed:
                    self.confirmed_blocked |= LINE_BLOCKS[index]
                else:
                    self.ambiguous_blocked |= SUSPECT_BLOCKS[index]
        observed = sum(1 << (ray // 15) for ray in visible)
        self.coverage = 0
        for i, window in enumerate(SECTOR_BITS):
            if observed & window == window:
                self.coverage |= 1 << i
        self.present_blocked = self.confirmed_blocked | self.ambiguous_blocked
        self.blocked = self.present_blocked | self.held_blocked
        self.unknown = 0xffff & ~self.coverage
        mask = self.coverage & ~self.blocked
        self.escape_away = ((1 | 256) & ~self.held_blocked) if self.escape_requested else 0
        # A clipped run still blocks its approach cone, but its distance is not known.
        wire_nearest = None if self.line_distance is None else (self.line_angle, self.line_distance)
        return mask, self.escape_away & mask, wire_nearest


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
