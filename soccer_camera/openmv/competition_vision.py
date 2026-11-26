"""Protocol 2 recognition: line guard, goals, ball, vision heading, packet.

Copy next to main.py and set PROTOCOL = 2 in camera_calibration.py (MAIN must be built with ROBOT_UPGRADE).
"""
import math

SECTORS = 16
SECTOR_STEP = 360 / SECTORS
SECTOR_HALF_WIDTH = 30  # Every floor ray this close to a heading must be visible and clear.
APPROACH_LIMIT_DEG = 60  # Never head closer than this to a ray with a nearby line.
ESCAPE_LIMIT_DEG = 75
RELEASE_MARGIN_CM = 5
RELEASE_FRAMES = 2
GOAL_HOLD_MS = 800  # must outlast the slowest goal detection period
HEADING_MAX_AGE = 30  # x10 ms: older goal bearings may predate a rotation
BALL_HOLD_MS = 450
UNKNOWN_ANGLE = 0xFFFF
UNKNOWN_BYTE = 255
PACKET_SIZE = 37

FLAG_READY = 0x01
FLAG_ESCAPE = 0x02
FLAG_TARGET_BLUE = 0x04
FLAG_SLOW = 0x08
FLAG_TARGET_VALID = 0x10


def wrap(angle):
    return (angle + 180) % 360 - 180


def polar(x, y):
    return math.degrees(math.atan2(x, y)) % 360


def value(blob, name):
    attribute = getattr(blob, name)
    return attribute() if callable(attribute) else attribute


class LineGuard:
    """Turns ray distances into a clear-sector mask and a continuous escape heading."""

    def __init__(self, clearance_cm, block_cm):
        self.clearance = clearance_cm
        self.block = max(block_cm, clearance_cm + RELEASE_MARGIN_CM)
        self.escaping = False
        self.escape = None
        self.release = 0

    def update(self, ready, visible, boundaries):
        """Returns (mask, escape_bearing or None, nearest (bearing, cm) or None, line count)."""
        if not ready:
            # Unknown floor is never "all clear". The escape state survives until the floor is seen again.
            self.escape = None
            return 0, None, None, 0
        lines = {}
        for bearing, color, cm, _, _ in boundaries:
            if color == "WHITE":
                lines[bearing] = min(cm, lines.get(bearing, cm))
        limit = self.clearance + (RELEASE_MARGIN_CM if self.escaping else 0)
        close = [b for b, cm in lines.items() if cm <= limit]
        blocked = [b for b, cm in lines.items() if cm <= self.block]
        if close:
            self.escaping, self.release = True, 0
        elif self.escaping:
            self.release += 1
            if self.release >= RELEASE_FRAMES:
                self.escaping, self.escape = False, None

        clear = set(visible).difference(close)
        open_sectors = []
        mask = 0
        for sector in range(SECTORS):
            heading = sector * SECTOR_STEP
            window = [r for r in range(0, 360, 15) if abs(wrap(r - heading)) <= SECTOR_HALF_WIDTH]
            if all(r in clear for r in window):
                open_sectors.append(heading)
                if all(abs(wrap(heading - b)) >= APPROACH_LIMIT_DEG for b in blocked):
                    mask |= 1 << sector

        escape = None
        if close:
            scores = {h: min(abs(wrap(h - b)) for b in close) for h in open_sectors}
            # Only multiples of 45 degrees come out exactly on all four wheels.
            good = {h: s for h, s in scores.items() if s >= ESCAPE_LIMIT_DEG and h % 45 == 0}
            # Prefer ways out that do not head toward a second, slightly farther line.
            good = {h: s for h, s in good.items()
                    if all(abs(wrap(h - b)) >= APPROACH_LIMIT_DEG for b in blocked)} or good
            if good:
                # Farthest from every near line; keep the previous choice unless clearly worse.
                away_x = sum(-math.sin(math.radians(b)) * (self.clearance - lines[b] + 6) for b in close)
                away_y = sum(-math.cos(math.radians(b)) * (self.clearance - lines[b] + 6) for b in close)
                away = polar(away_x, away_y)
                best = max(good, key=lambda h: (good[h], -abs(wrap(h - away))))
                if self.escape in good and good[self.escape] >= good[best] - 15:
                    best = self.escape
                escape = best
                mask = 0
                for h in good:
                    mask |= 1 << int(h / SECTOR_STEP + 0.5)
            else:
                mask = 0  # No verified way out: stop.
        elif self.escaping and self.escape is not None:
            if mask & (1 << int(self.escape / SECTOR_STEP + 0.5)):
                escape = self.escape
        self.escape = escape

        nearest = min(lines.items(), key=lambda item: item[1]) if lines else None
        ordered = sorted(lines)
        groups = sum(1 for i, b in enumerate(ordered) if (b - ordered[i - 1]) % 360 > 30)
        return mask, escape, nearest, max(groups, 1 if ordered else 0)


def observed_rays(visible, boundaries):
    """Rays that told us something: open floor, or a white line (which hides the floor behind it)."""
    return len(set(visible) | {edge[0] for edge in boundaries if edge[1] == "WHITE"})


def table_distance(table, radius):
    """Radius (px) to centimetres. Invalid table or radius outside it: unknown, never extrapolated."""
    if len(table) < 2:
        return None
    pairs = list(zip(table, table[1:]))
    sign = 1 if table[-1][0] > table[0][0] else -1
    if not all(sign * (r1 - r0) > 0 and 0 < d0 < d1 for (r0, d0), (r1, d1) in pairs):
        return None
    for (r0, d0), (r1, d1) in pairs:
        if min(r0, r1) <= radius <= max(r0, r1):
            return d0 + (d1 - d0) * (radius - r0) / (r1 - r0)
    return None


class Track:
    """One tracked object: candidate choice, circular smoothing, confidence and short hold."""

    def __init__(self, table, hold_ms=GOAL_HOLD_MS):
        self.hold_ms = hold_ms
        self.table = table
        self.bearing = None
        self.distance = None
        self.width = 0
        self.confidence = 0
        self.seen_ms = 0
        self.hits = 0

    def update(self, now, candidates):
        """candidates: (bearing, quality 0..1, radius_px, width_deg)."""
        if self.bearing is not None and now - self.seen_ms > self.hold_ms:
            self.bearing, self.hits = None, 0
        if not candidates:
            self.confidence = int(self.confidence * 0.7)
            self.hits = max(0, self.hits - 1)
            return
        previous = self.bearing

        def score(c):
            near = previous is not None and abs(wrap(c[0] - previous)) <= 30
            return c[1] * (1.5 if near else 1.0)

        bearing, quality, radius, width = max(candidates, key=score)
        jump = previous is not None and abs(wrap(bearing - previous)) > 45
        if previous is None or jump:
            self.bearing, self.hits = bearing, 1
        else:
            self.bearing = (previous + 0.6 * wrap(bearing - previous)) % 360
            self.hits = min(self.hits + 1, 3)
        self.confidence = int(100 * quality * self.hits / 3)
        self.distance = table_distance(self.table, radius)
        self.width = width
        self.seen_ms = now

    def fields(self, now):
        if self.bearing is None or now - self.seen_ms > self.hold_ms:
            return None, None, 0, 0, UNKNOWN_BYTE
        age = min(254, int((now - self.seen_ms) / 10))
        return self.bearing, self.distance, self.confidence, self.width, age


def blob_candidate(blob, mirror, min_pixels, ball):
    """Returns (bearing, quality, radius, width_deg) or None."""
    pixels = value(blob, "pixels")
    cx, cy = value(blob, "cx"), value(blob, "cy")
    if pixels < min_pixels or (mirror.ring_ready and not mirror.pixels_on_ring(cx, cy)):
        return None
    w, h = value(blob, "w"), value(blob, "h")
    x, y = value(blob, "x"), value(blob, "y")
    if w <= 0 or h <= 0:
        return None
    bearing = mirror.bearing(cx, cy)
    quality = min(1.0, pixels / (4.0 * min_pixels))
    corners = ((x, y), (x + w, y), (x, y + h), (x + w, y + h))
    offsets = [wrap(mirror.bearing(px, py) - bearing) for px, py in corners]
    width = min(180, int(max(offsets) - min(offsets)))
    if ball:
        aspect = min(w, h) / max(w, h)
        fill = pixels / (w * h)
        if aspect < 0.45 or fill < 0.20:
            return None
        quality *= min(1.0, aspect / 0.8) * min(1.0, fill / 0.6)
        radius = math.sqrt((cx - mirror.cx) ** 2 + (cy - mirror.cy) ** 2)
    else:
        # The goal's floor edge is its part nearest to the mirror axis.
        nearest_x = min(max(mirror.cx, x), x + w)
        nearest_y = min(max(mirror.cy, y), y + h)
        radius = math.sqrt((nearest_x - mirror.cx) ** 2 + (nearest_y - mirror.cy) ** 2)
    return bearing, quality, radius, width


def estimate_heading(own, opponent):
    """Approximate field heading (robot clockwise from the attack axis) from BOTH goals.

    own/opponent: (bearing, confidence). One goal alone gives only a relative bearing, never a heading.
    Exact on the field's two centre axes, approximate elsewhere; confidence drops as the goals
    stop looking opposite each other.
    """
    if own[0] is None or opponent[0] is None or min(own[1], opponent[1]) < 40:
        return None, 0
    separation = abs(wrap(opponent[0] - own[0]))
    if separation < 100:
        return None, 0
    ox, oy = math.sin(math.radians(opponent[0])), math.cos(math.radians(opponent[0]))
    wx, wy = math.sin(math.radians(own[0])), math.cos(math.radians(own[0]))
    axis = polar(ox - wx, oy - wy)
    confidence = int(min(own[1], opponent[1]) * min(1.0, (separation - 90) / 70))
    return (-axis) % 360, confidence


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def make_packet(sequence, flags, mask, escape, nearest, line_count, ball, yellow, blue, heading, frame_ms):
    packet = bytearray((0xC5, 0x5C, 2, PACKET_SIZE, sequence & 0xFF, flags))

    def angle(a):
        a = UNKNOWN_ANGLE if a is None else int(a + 0.5) % 360
        packet.extend((a & 0xFF, a >> 8))

    def byte(v, unknown=UNKNOWN_BYTE):
        packet.append(unknown if v is None else max(0, min(254, int(v + 0.5))))

    packet.extend((mask & 0xFF, mask >> 8))
    angle(escape)
    angle(None if nearest is None else nearest[0])
    byte(None if nearest is None else nearest[1])
    byte(line_count)
    bearing, distance, confidence, _, age = ball
    angle(bearing)
    byte(distance)
    byte(confidence)
    packet.append(age)
    for goal in (yellow, blue):
        bearing, distance, confidence, width, age = goal
        angle(bearing)
        byte(distance)
        byte(confidence)
        byte(width)
        packet.append(age)
    angle(heading[0])
    byte(heading[1])
    byte(frame_ms)
    crc = crc16(packet)
    packet.extend((crc & 0xFF, crc >> 8))
    return packet


class Pipeline:
    """Line detection runs every frame; goals and ball run less often, even less when slow."""

    def __init__(self, cfg, mirror, clearance_cm, max_frame_ms):
        self.cfg = cfg
        self.mirror = mirror
        self.max_frame_ms = max_frame_ms
        self.guard = LineGuard(clearance_cm, getattr(cfg, "LINE_BLOCK_CM", clearance_cm + 10))
        self.ball = Track(getattr(cfg, "BALL_DISTANCE", ()), BALL_HOLD_MS)
        goal_table = getattr(cfg, "GOAL_DISTANCE", ())
        self.goals = {"YELLOW": Track(goal_table), "BLUE": Track(goal_table)}
        self.sequence = 0
        self.frame = 0
        self.timing = {"ball": 0, "goals": 0, "pose": 0}
        self.ran = {"ball": 0, "goals": 0}
        self.heading = (None, 0)

    def candidates(self, img, thresholds, min_pixels, ball):
        mirror = self.mirror
        if not mirror.ring_ready or not thresholds:
            return []
        radius = math.ceil(mirror.outer)
        left, top = max(0, round(mirror.cx) - radius), max(0, round(mirror.cy) - radius)
        roi = (left, top, min(mirror.width - left, 2 * radius + 1), min(mirror.height - top, 2 * radius + 1))
        found = img.find_blobs(thresholds, roi=roi, pixels_threshold=min_pixels, area_threshold=min_pixels,
                               merge=True, margin=2)
        result = [blob_candidate(b, mirror, min_pixels, ball) for b in found]
        return [c for c in result if c is not None]

    def unhide_ball(self, now, visible, boundaries):
        """A ball close to the robot hides its own floor rays; fill that gap if both sides are clear."""
        bearing, _, confidence, width, age = self.ball.fields(now)
        if bearing is None or confidence < 40 or age > 15:
            return visible
        white = {e[0]: e[2] for e in boundaries if e[1] == "WHITE"}
        result = set(visible)
        for ray in range(0, 360, 15):
            if ray in visible or ray in white or abs(wrap(ray - bearing)) > max(15, width / 2):
                continue
            sides = [[(ray + s * d) % 360 for d in (15, 30)] for s in (-1, 1)]
            if all(any(r in visible and white.get(r, 255) > self.guard.block for r in side) for side in sides):
                result.add(ray)
        return result

    def step(self, img, now, ready, frame_ms, visible, boundaries, clock=None):
        cfg = self.cfg
        clock = clock or (lambda: 0)
        began = clock()
        # Detection time delays the packet too, so it counts against the line budget.
        slow = frame_ms + self.timing["ball"] + self.timing["goals"] > self.max_frame_ms * 0.7
        ball_period, goal_period = (4, 6) if slow else (2, 3)
        self.frame += 1
        def affordable(name):
            # Line data comes first: skip a detection that would make this packet stale, but not forever.
            fits = frame_ms + clock() - began + self.timing[name] <= self.max_frame_ms * 0.9
            if fits or self.frame - self.ran[name] >= 20:
                self.ran[name] = self.frame
                return True
            return False

        if self.frame % ball_period == 0 and affordable("ball"):
            start = clock()
            self.ball.update(now, self.candidates(img, getattr(cfg, "BALL", None),
                                                  getattr(cfg, "BALL_MIN_PIXELS", 10), True))
            self.timing["ball"] = clock() - start
        if self.frame % goal_period == 1 and affordable("goals"):
            start = clock()
            for name, track in self.goals.items():
                found = self.candidates(img, getattr(cfg, name), getattr(cfg, name + "_MIN_PIXELS", 12), False)
                if name == "YELLOW" and self.ball.bearing is not None:
                    # The orange ball also passes the yellow thresholds: drop ball-sized blobs at its bearing.
                    found = [c for c in found if not (abs(wrap(c[0] - self.ball.bearing)) <= 10
                                                      and c[3] <= self.ball.width + 5)]
                track.update(now, found)
            self.timing["goals"] = clock() - start
        age_ms = frame_ms + clock() - began
        floor_ok = ready
        ready = floor_ok and age_ms <= self.max_frame_ms
        visible = self.unhide_ball(now, visible, boundaries)
        mask, escape, nearest, count = self.guard.update(ready, visible, boundaries)
        start = clock()
        ball = self.ball.fields(now)
        yellow = list(self.goals["YELLOW"].fields(now))
        blue = list(self.goals["BLUE"].fields(now))
        # Both goals cannot sit in nearly the same direction: one of them is a false detection.
        if yellow[0] is not None and blue[0] is not None and abs(wrap(yellow[0] - blue[0])) < 60:
            yellow[2] //= 2
            blue[2] //= 2
        target = getattr(cfg, "TARGET_GOAL", None)
        flags = FLAG_READY if ready else 0
        if escape is not None:
            flags |= FLAG_ESCAPE
        if floor_ok and not ready:
            flags |= FLAG_SLOW  # late, not blind: MAIN may reuse the previous line data briefly
        self.heading = (None, 0)
        if target in ("YELLOW", "BLUE"):
            flags |= FLAG_TARGET_VALID | (FLAG_TARGET_BLUE if target == "BLUE" else 0)
            opponent, own = (blue, yellow) if target == "BLUE" else (yellow, blue)
            if max(own[4], opponent[4]) <= HEADING_MAX_AGE:
                self.heading = estimate_heading((own[0], own[2]), (opponent[0], opponent[2]))
        self.timing["pose"] = clock() - start
        packet = make_packet(self.sequence, flags, mask, escape, nearest, count, ball, yellow, blue,
                             self.heading, frame_ms)
        self.sequence = (self.sequence + 1) & 0xFF
        self.status = "V2 " + ("OK" if ready else "SLOW" if floor_ok else "NOFLOOR")
        self.status += " mask={:04x} esc={} line={} ball={}/{} Y={}/{} B={}/{} head={}/{}".format(
            mask, escape, nearest, ball[0], ball[2], yellow[0], yellow[2], blue[0], blue[2], *self.heading)
        return packet
