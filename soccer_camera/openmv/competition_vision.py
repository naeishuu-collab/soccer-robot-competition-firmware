"""OpenMV側の拡張ビジョン処理。goal_wall_navigation.pyの校正/画像処理を土台に、
ゴール・ライン・姿勢情報を拡張パケットにまとめる（ボールは扱わない）。MAINの拡張処理
(main/main_logic.h)と対になる実験経路で、現行の標準プログラムとは別に選択する。
この経路を使う場合はPROTOCOL=2にし、MAINもROBOT_UPGRADE=1で同時にビルドする。
"""
import math
from array import array

try:
    import micropython
except ImportError:
    class micropython:  # host: the native decorator does nothing
        native = staticmethod(lambda function: function)

SECTORS = 16
SECTOR_STEP = 360 / SECTORS
SECTOR_HALF_WIDTH = 30  # Every floor ray this close to a heading must be visible and clear.
APPROACH_LIMIT_DEG = 60  # Never head closer than this to a ray with a nearby line.
ESCAPE_LIMIT_DEG = 75
RELEASE_MARGIN_CM = 5
RELEASE_FRAMES = 2
RELEASE_MS = 100  # with a faster camera the release must still take at least this long
GOAL_MIN_INTERVAL_MS = 80  # per colour: a faster line loop does not search the goals more often
GOAL_HOLD_MS = 800  # must outlast the slowest goal detection period
HEADING_MAX_AGE = 30  # x10 ms: older goal bearings may predate a rotation
GOAL_MIN_SEPARATION_DEG = 90
UNKNOWN_ANGLE = 0xFFFF
UNKNOWN_BYTE = 255
PROTOCOL_VERSION = 3
PACKET_SIZE = 32

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


# Floor rays that must all be clear for each of the 16 headings (computed once).
SECTOR_WINDOWS = tuple(tuple(r for r in range(0, 360, 15)
                             if abs((r - s * SECTOR_STEP + 180) % 360 - 180) <= SECTOR_HALF_WIDTH)
                       for s in range(SECTORS))


class LineGuard:
    """白線の観測距離から安全セクターと回避方位を作り、一時的な見失いによる急変を抑える。"""

    def __init__(self, clearance_cm, block_cm):
        self.clearance = clearance_cm
        self.block = max(block_cm, clearance_cm + RELEASE_MARGIN_CM)
        self.escaping = False
        self.escape = None
        self.release = 0
        self.close_ms = None
        self.streak = {}

    def update(self, ready, visible, boundaries, thin=None, now=None):
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
        # Confirmed line: thin white with turf beyond it, or seen on neighbouring bearings in consecutive
        # frames. Anything else white (robot shell, glare) still closes headings, but starts no retreat.
        self.streak = {b: 1 + max(self.streak.get((b + o) % 360, 0) for o in (-15, 0, 15)) for b in lines}
        near = [b for b, cm in lines.items() if cm <= limit]
        close = [b for b in near if thin is None or b in thin or self.streak[b] >= 2]
        blocked = [b for b, cm in lines.items() if cm <= self.block]
        if close:
            self.escaping, self.release = True, 0
            self.close_ms = now
        elif self.escaping:
            self.release += 1
            held = now is None or self.close_ms is None or now - self.close_ms >= RELEASE_MS
            if self.release >= RELEASE_FRAMES and held:
                self.escaping, self.escape = False, None

        clear = set(visible).difference(near)
        open_sectors = []
        mask = 0
        for sector in range(SECTORS):
            heading = sector * SECTOR_STEP
            for r in SECTOR_WINDOWS[sector]:
                if r not in clear:
                    break
            else:
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
                # Front/right/back/left are bench verified: prefer them unless clearly worse than a diagonal.
                best = max(good, key=lambda h: (good[h] + (20 if h % 90 == 0 else 0), -abs(wrap(h - away))))
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
    """対象候補をフレーム間で対応付け、方位の平滑化・信頼度・短時間保持を行う。"""

    def __init__(self, table, hold_ms=GOAL_HOLD_MS):
        self.hold_ms = hold_ms
        self.table = table
        self.bearing = None
        self.distance = None
        self.width = 0
        self.confidence = 0
        self.seen_ms = 0
        self.hits = 0
        self.pending = None

    def update(self, now, candidates):
        """candidates: (bearing, quality 0..1, radius_px, width_deg)."""
        if self.bearing is not None and now - self.seen_ms > self.hold_ms:
            self.bearing, self.hits = None, 0
        previous = self.bearing

        def score(c):
            near = previous is not None and abs(wrap(c[0] - previous)) <= 30
            return c[1] * (1.5 if near else 1.0)

        best = max(candidates, key=score) if candidates else None
        jump = best is not None and previous is not None and abs(wrap(best[0] - previous)) > 45
        confirmed_jump = jump and self.pending is not None and abs(wrap(best[0] - self.pending)) <= 30
        if best is None or (jump and not confirmed_jump):
            # Nothing, or a sudden move seen only once: keep the old bearing and trust it less.
            self.pending = None if best is None else best[0]
            self.confidence = int(self.confidence * 0.7)
            self.hits = max(0, self.hits - 1)
            return
        bearing, quality, radius, width = best
        self.pending = None
        if previous is None:
            self.bearing, self.hits = bearing, 1
        elif confirmed_jump:
            self.bearing, self.hits = bearing, 2
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


def goal_candidate(blob, mirror, min_pixels, cfg, color_of=None, report=None):
    """色領域をゴールとして評価する。不合格はNone、合格は(方位, 品質, 半径, 視角)。

    大きさだけでは選ばない: ミラー上の角度幅、弧と高さの比、密度、外側/内側の床との関係を見る。
    相手機体やボールのように床の上にある物は、中心から見て外側にも緑床が見えるので除外する。
    """
    # report (diagnostics only) receives the measured values and the reason for rejection.
    def reject(reason):
        if report is not None:
            report["reason"] = reason

    pixels = value(blob, "pixels")
    cx, cy = value(blob, "cx"), value(blob, "cy")
    x, y, w, h = (value(blob, key) for key in ("x", "y", "w", "h"))
    radius = math.sqrt((cx - mirror.cx) ** 2 + (cy - mirror.cy) ** 2)
    if report is not None:
        report.update({"pixels": pixels, "cx": cx, "cy": cy, "box": (x, y, w, h), "radius": radius})
    if pixels < min_pixels:
        return reject("SMALL")
    if mirror.ring_ready and not mirror.pixels_on_ring(cx, cy):
        return reject("RING")
    if w <= 0 or h <= 0 or radius < 1:
        return reject("EMPTY")
    bearing = mirror.bearing(cx, cy)
    corners = ((x, y), (x + w, y), (x, y + h), (x + w, y + h))
    offsets = [wrap(mirror.bearing(px, py) - bearing) for px, py in corners]
    width = min(180, int(max(offsets) - min(offsets)))
    ux, uy = (cx - mirror.cx) / radius, (cy - mirror.cy) / radius
    height = max(1.0, abs(w * ux) + abs(h * uy))  # radial extent: the goal's apparent height
    arc = math.radians(width) * radius
    density = pixels / (w * h)
    if report is not None:
        report.update({"bearing": bearing, "width": width, "aspect": arc / height, "density": density})
    if not getattr(cfg, "GOAL_MIN_WIDTH_DEG", 6) <= width <= getattr(cfg, "GOAL_MAX_WIDTH_DEG", 150):
        return reject("WIDTH")
    if arc / height < getattr(cfg, "GOAL_MIN_ASPECT", 0.8):
        return reject("TALL")
    if density < getattr(cfg, "GOAL_MIN_DENSITY", 0.2):
        return reject("SPARSE")
    quality = min(1.0, pixels / (4.0 * min_pixels)) * min(1.0, 0.5 + density)
    if color_of is not None:
        def colors(distance):
            found = []
            for turn in (-4, 0, 4):
                a = math.atan2(uy, ux) + math.radians(turn)
                px = round(mirror.cx + distance * math.cos(a))
                py = round(mirror.cy + distance * math.sin(a))
                if 0 <= px < mirror.width and 0 <= py < mirror.height and (
                        not mirror.ring_ready or mirror.pixels_on_ring(px, py)):
                    found.append(color_of(px, py))
            return found

        beyond = colors(radius + height / 2 + 3)
        if len(beyond) >= 2 and beyond.count("GREEN") * 3 >= len(beyond) * 2:
            return reject("TURF")  # turf continues behind it: an object standing on the field
        before = colors(radius - height / 2 - 3)
        if len(before) >= 2 and "GREEN" not in before and "WHITE" not in before:
            quality *= 0.6  # no floor in front of it: weaker evidence (may be hidden by a robot)
    if report is not None:
        report.update({"reason": "OK", "quality": quality})
    # The goal's floor edge is its part nearest to the mirror axis.
    return bearing, quality, max(0.0, radius - height / 2), width


def separate_goals(yellow, blue):
    """青と黄は同じ向きには見えない。近すぎる場合は弱い方を捨て、強い方も半分の信頼度にする。"""
    if yellow[0] is None or blue[0] is None or abs(wrap(yellow[0] - blue[0])) >= GOAL_MIN_SEPARATION_DEG:
        return
    weak, strong = (yellow, blue) if yellow[2] <= blue[2] else (blue, yellow)
    weak[2] = 0
    strong[2] //= 2


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


def _crc_entry(value):
    crc = value << 8
    for _ in range(8):
        crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


CRC_TABLE = tuple(_crc_entry(i) for i in range(256))


@micropython.native
def crc16(data):
    """CRC16-CCITT (0xFFFF, 0x1021), one table lookup per byte."""
    crc = 0xFFFF
    table = CRC_TABLE
    for byte in data:
        crc = ((crc << 8) & 0xFFFF) ^ table[(crc >> 8) ^ byte]
    return crc


def make_packet(sequence, flags, mask, escape, nearest, line_count, yellow, blue, heading, frame_ms):
    """検出結果をMAIN拡張側が読む固定長パケットへ符号化し、CRCを付ける。ボール情報は送らない。"""
    packet = bytearray((0xC5, 0x5C, PROTOCOL_VERSION, PACKET_SIZE, sequence & 0xFF, flags))

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


class Stats:
    """処理ごとの時間(µs)を固定長の配列に貯め、表示するときだけ平均・最大・P95を計算する。"""

    def __init__(self, names, size=128):
        self.size = size
        self.names = names
        self.values = {name: array("i", [0] * size) for name in names}
        self.count = {name: 0 for name in names}

    def add(self, name, microseconds):
        i = self.count[name]
        self.values[name][i % self.size] = microseconds
        self.count[name] = i + 1

    def summary(self, name):
        n = min(self.count[name], self.size)
        if not n:
            return None
        ordered = sorted(self.values[name][i] for i in range(n))
        return sum(ordered) / n / 1000, ordered[-1] / 1000, ordered[(n * 95 + 99) // 100 - 1] / 1000

    def report(self):
        parts = []
        for name in self.names:
            result = self.summary(name)
            if result is not None:
                parts.append("{}={:.1f}/{:.1f}/{:.1f}".format(name, *result))
            self.count[name] = 0
        return " ".join(parts)


class Pipeline:
    """拡張ビジョンの1フレーム処理をまとめる。ラインは毎フレーム、ゴールは1色ずつ交互に間引いて調べる。"""

    def __init__(self, cfg, mirror, clearance_cm, max_frame_ms):
        self.cfg = cfg
        self.mirror = mirror
        self.max_frame_ms = max_frame_ms
        self.guard = LineGuard(clearance_cm, getattr(cfg, "LINE_BLOCK_CM", clearance_cm + 10))
        goal_table = getattr(cfg, "GOAL_DISTANCE", ())
        self.goals = {"YELLOW": Track(goal_table), "BLUE": Track(goal_table)}
        self.sequence = 0
        self.frame = 0
        self.timing = {"YELLOW": 0, "BLUE": 0, "pose": 0}
        self.ran = {"YELLOW": 0, "BLUE": 0}
        self.ran_ms = {"YELLOW": None, "BLUE": None}
        self.keep_reports = False  # the overlay shows what this search saw instead of searching again
        self.reports = {"YELLOW": [], "BLUE": []}
        self.forced = bool(getattr(cfg, "FORCE_FLOOR_OK", False))  # shown on screen and in the log
        self.heading = (None, 0)
        self.last = None
        # Stage timer: mark(name, since) records the time since `since` and returns now. Set by the camera loop.
        self.mark = lambda name, since=None: 0

    def candidates(self, img, name, color_of=None):
        mirror = self.mirror
        thresholds = getattr(self.cfg, name)
        if not mirror.ring_ready or not thresholds:
            return []
        min_pixels = getattr(self.cfg, name + "_MIN_PIXELS", 12)
        radius = math.ceil(mirror.outer)
        left, top = max(0, round(mirror.cx) - radius), max(0, round(mirror.cy) - radius)
        roi = (left, top, min(mirror.width - left, 2 * radius + 1), min(mirror.height - top, 2 * radius + 1))
        # x_stride 2: a goal is always wider than two pixels.
        found = img.find_blobs(thresholds, roi=roi, x_stride=2, pixels_threshold=min_pixels,
                               area_threshold=min_pixels, merge=True, margin=2)
        result = []
        reports = [] if self.keep_reports else None
        for blob in found:
            report = None if reports is None else {}
            candidate = goal_candidate(blob, mirror, min_pixels, self.cfg, color_of, report)
            if candidate is not None:
                result.append(candidate)
            if reports is not None:
                reports.append(report)
        if reports is not None:
            reports.sort(key=lambda r: (r.get("reason") != "OK", -r["pixels"]))
            self.reports[name] = reports
        return result

    def fill_gap(self, visible, boundaries):
        """幅1本(15度)だけの床の欠けは、両隣2本ずつに床が見え近い線がなければ開く(小さい物体の陰)。

        直線の白線は複数の方向にまたがるので、この条件の欠けには隠れない。実機では未確認の仮定。
        """
        if not getattr(self.cfg, "FILL_SINGLE_GAP", True):
            return visible
        white = {e[0]: e[2] for e in boundaries if e[1] == "WHITE"}
        result = set(visible)
        for ray in range(0, 360, 15):
            if ray not in visible and ray not in white and all(
                    (ray + o) % 360 in visible and white.get((ray + o) % 360, 255) > self.guard.block
                    for o in (-30, -15, 15, 30)):
                result.add(ray)
        return result

    def step(self, img, now, ready, frame_ms, visible, boundaries, clock=None, thin=None, color_of=None):
        # 1) 時間予算内でゴールを1色ずつ追跡 2) ライン安全判定 3) パケット化。
        # 期限超過や不十分な床観測はready=Falseとして安全側に倒す。
        cfg = self.cfg
        clock = clock or (lambda: 0)
        began = clock()
        # Detection time delays the packet too, so it counts against the line budget.
        slow = frame_ms + max(self.timing["YELLOW"], self.timing["BLUE"]) > self.max_frame_ms * 0.7
        period = 6 if slow else 4  # slow: still inside GOAL_HOLD_MS at 10 fps
        self.frame += 1
        stage = self.mark(None)
        for name, phase in (("YELLOW", 1), ("BLUE", 1 + period // 2)):
            if self.frame % period != phase:
                continue
            if self.ran_ms[name] is not None and now - self.ran_ms[name] < GOAL_MIN_INTERVAL_MS:
                continue
            # Line data comes first: skip a detection that would make this packet stale, but not forever.
            fits = frame_ms + clock() - began + self.timing[name] <= self.max_frame_ms * 0.9
            if fits or self.frame - self.ran[name] >= 20:
                self.ran[name] = self.frame
                self.ran_ms[name] = now
                start = clock()
                self.goals[name].update(now, self.candidates(img, name, color_of))
                self.timing[name] = clock() - start
        age_ms = frame_ms + clock() - began
        floor_ok = ready
        ready = floor_ok and age_ms <= self.max_frame_ms
        stage = self.mark("goal", stage)
        visible = self.fill_gap(visible, boundaries)
        stage = self.mark("gap", stage)
        # ライン情報を確定してから、追跡値と姿勢値を同じ連番の送信データにまとめる。
        mask, escape, nearest, count = self.guard.update(ready, visible, boundaries, thin, now)
        self.mark("guard", stage)
        start = clock()
        yellow = list(self.goals["YELLOW"].fields(now))
        blue = list(self.goals["BLUE"].fields(now))
        separate_goals(yellow, blue)
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
        packet = make_packet(self.sequence, flags, mask, escape, nearest, count, yellow, blue,
                             self.heading, frame_ms)
        self.sequence = (self.sequence + 1) & 0xFF
        self.last = (ready, floor_ok, mask, escape, nearest, yellow, blue)
        return packet

    def status(self):
        """診断表示用。毎フレームの文字列生成を避けるため、表示するときだけ作る。"""
        if self.last is None:
            return "V3 no frame yet"
        ready, floor_ok, mask, escape, nearest, yellow, blue = self.last
        return "V3 {} mask={:04x} esc={} line={} Y={}/{} B={}/{} head={}/{}".format(
            ("OK (FORCED)" if self.forced else "OK") if ready else "SLOW" if floor_ok else "NOFLOOR", mask, escape, nearest,
            yellow[0], yellow[2], blue[0], blue[2], *self.heading)
