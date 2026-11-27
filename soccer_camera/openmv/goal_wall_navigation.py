"""現行OpenMVプログラム。ミラー画像から床の安全方向と白線回避方向を求め、MAINへUART送信する。
camera_calibration.pyの校正値を使い、MAIN側main.inoの8バイト受信形式と対になる。
カメラ表示・しきい値診断スクリプトも、このファイルの検出関数と初期化処理を再利用する。
OpenMVにはこのファイルをmain.pyとして置き、camera_calibration.pyを同じ場所に保存する。
"""
import math

import camera_calibration as cfg

try:
    import sensor
    import image
    import time
    import pyb
except ImportError:
    sensor = image = time = pyb = None

try:
    import micropython
except ImportError:
    class micropython:  # host: the native decorator does nothing
        native = staticmethod(lambda function: function)

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
COLOR_CACHE_LIMIT = 2048  # distinct camera colours remembered between frames
STREAM_INTERVAL_MS = 50
MAX_FRAME_AGE_MS = 100
LINE_MAX_WIDTH_CM = 8  # a field line is about 2 cm; wider white is not "thin"

MODE_WAIT = 0
MODE_AVOID = 1
MODE_TRACK_IR = 2
FLAG_VIRTUAL_LINE = 0x01
FLAG_DIRECTIONAL = 0x40  # Requires the matching MAIN firmware.

GOAL_COLORS = {"YELLOW": (255, 255, 0), "BLUE": (0, 110, 255)}
MODES = ("WAIT", "AVOID", "TRACK IR")


def image_angle(dx, dy):
    return math.degrees(math.atan2(dx, -dy)) % 360


def distance(dx, dy):
    return math.sqrt(dx * dx + dy * dy)


def blob_value(blob, name):
    value = getattr(blob, name)
    return value() if callable(value) else value


def in_lab(lab, threshold):
    l, a, b = lab
    return (threshold[0] <= l <= threshold[1] and threshold[2] <= a <= threshold[3]
            and threshold[4] <= b <= threshold[5])


def init_camera():
    # 一度自動調整で照明を安定させてから、校正ファイルの露出/ゲインに固定する。
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
    """全周ミラーの中心/リングを管理し、放射状の床サンプルを画像座標へ対応付ける。"""
    def __init__(self, width, height, frame=None):
        # 校正値を読み、画像サイズとミラー寸法が有効なら走査点を事前計算する。
        self.width = width
        self.height = height
        self.cx, self.cy = cfg.MIRROR_CENTER or ((width - 1) / 2, (height - 1) / 2)
        self.inner, self.outer = cfg.MIRROR_RING or (0, 0)
        self.detected = False
        if (getattr(cfg, "MIRROR_AUTO_DETECT", False) and frame is not None
                and cfg.IMAGE_SIZE == (width, height) and cfg.MIRROR_CENTER and cfg.MIRROR_RING):
            self.detect_ring(frame)
        self.points = cfg.GROUND_POINTS
        self.ring_ready = (cfg.IMAGE_SIZE == (width, height) and cfg.MIRROR_CENTER is not None
                           and 0 <= self.inner < self.outer
                           <= min(self.cx, self.cy, width - 1 - self.cx, height - 1 - self.cy))
        self.rays = self.ground_rays() if self.ring_ready and self.ground_ready() else ()

    def detect_ring(self, frame):
        # 円候補→内外リングの組→輪郭コントラストの順に絞り、中心と半径を補正する。
        if not hasattr(frame, "find_circles") or not hasattr(frame, "get_pixel"):
            return
        circles = frame.find_circles(threshold=1500, x_margin=10, y_margin=10, r_margin=8,
                                     r_min=max(2, int(self.inner * 0.5)),
                                     r_max=int(self.outer * 1.2), r_step=2)
        inner = [c for c in circles if self.inner * 0.5 <= blob_value(c, "r") <= self.inner * 1.4
                 and distance(blob_value(c, "x") - self.cx, blob_value(c, "y") - self.cy) <= 30]
        outer = [c for c in circles if self.outer * 0.55 <= blob_value(c, "r") <= self.outer * 1.2
                 and distance(blob_value(c, "x") - self.cx, blob_value(c, "y") - self.cy) <= 30]
        pairs = [(i, o) for i in inner for o in outer
                 if 0.25 * blob_value(o, "r") <= blob_value(i, "r") <= 0.55 * blob_value(o, "r")
                 and all(blob_value(i, "r") <= radius <= blob_value(o, "r")
                         for radius, _ in cfg.GROUND_POINTS)
                 and distance(blob_value(i, "x") - blob_value(o, "x"),
                                blob_value(i, "y") - blob_value(o, "y")) <= 6
                 and blob_value(o, "r") <= min(blob_value(o, "x"), blob_value(o, "y"),
                                               self.width - 1 - blob_value(o, "x"),
                                               self.height - 1 - blob_value(o, "y"))]
        if not pairs:
            return
        def edge_score(circle, inner_edge):
            x, y, radius = (blob_value(circle, key) for key in ("x", "y", "r"))
            support = 0
            samples = 0
            quadrants = [0, 0, 0, 0]
            for angle in range(0, 360, 15):
                dx, dy = math.cos(math.radians(angle)), math.sin(math.radians(angle))
                a = (round(x + (radius - 3) * dx), round(y + (radius - 3) * dy))
                b = (round(x + (radius + 3) * dx), round(y + (radius + 3) * dy))
                if not (0 <= a[0] < self.width and 0 <= a[1] < self.height
                        and 0 <= b[0] < self.width and 0 <= b[1] < self.height):
                    continue
                inside = frame.get_pixel(a, rgbtuple=True)
                outside = frame.get_pixel(b, rgbtuple=True)
                delta = sum(outside) - sum(inside)
                samples += 1
                if (delta if inner_edge else -delta) > 45:
                    support += 1
                    quadrants[angle // 90] += 1
            return support if samples >= 18 and support >= 12 and min(quadrants) >= 2 else 0

        inner_scores = {id(c): edge_score(c, True) for c in inner}
        outer_scores = {id(c): edge_score(c, False) for c in outer}
        pairs = [(i, o) for i, o in pairs if inner_scores[id(i)] and outer_scores[id(o)]]
        if not pairs:
            return
        i, o = max(pairs, key=lambda pair: inner_scores[id(pair[0])] + outer_scores[id(pair[1])])
        x, y, inner_radius, outer_radius = (blob_value(o, "x"), blob_value(o, "y"),
                                             blob_value(i, "r"), blob_value(o, "r"))
        self.cx, self.cy, self.inner, self.outer = x, y, inner_radius, outer_radius
        self.detected = True

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
        # 各方位で校正距離に対応する画素を前計算し、毎フレームの床チェックを軽くする。
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


def find_color_blob(img, mirror, thresholds, min_pixels, accept=None, dense_search=False):
    """ミラーの有効領域から最大の色領域を選ぶ。色の診断用で、走行判断には使わない。"""
    # 候補検出 → 最小画素数 → ミラー円内 → 呼び出し側の追加条件、を通った最大領域を返す。
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
    options = {"x_stride": 1, "y_stride": 1, "merge": True, "margin": 2} if dense_search else {}
    for blob in img.find_blobs(thresholds, roi=roi, pixels_threshold=1, area_threshold=1, **options):
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


@micropython.native
def scan_ray(samples, color_at, include_wall=False):
    # 近距離から順に色を見て床の確認と、緑床の先にある白線/壁の最初の境界を返す。
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


def line_is_thin(samples, edge, color_at):
    """白の始点から先を調べ、細い白の向こうに緑床が続けばTrue(白線らしい)。判断できなければFalse。"""
    start = next((i for i, s in enumerate(samples) if s[0] == edge[2] and s[1] == edge[3]), None)
    if start is None:
        return False
    for x, y, cm in samples[start + 1:]:
        color = color_at(x, y)
        if color == "GREEN":
            return True
        if cm - edge[1] > LINE_MAX_WIDTH_CM or color == "BLACK":
            return False
    return False


def classify_rgb(rgb):
    lab = image.rgb_to_lab(rgb)
    low_saturation = getattr(cfg, "GREEN_LOW_SATURATION", None)
    if in_lab(lab, cfg.GREEN) or (low_saturation and in_lab(lab, low_saturation)):
        return "GREEN"
    if in_lab(lab, cfg.WHITE):
        return "WHITE"
    if in_lab(lab, cfg.BLACK):
        return "BLACK"
    return None



def classify_hazard(rgb):
    # Hazard and known visibility are separate: unclassified pixels stay unknown.
    lab = image.rgb_to_lab(rgb)
    if in_lab(lab, cfg.WHITE):
        return "WHITE"
    if in_lab(lab, cfg.BLACK):
        return "BLACK"
    low_saturation = getattr(cfg, "GREEN_LOW_SATURATION", None)
    if in_lab(lab, cfg.GREEN) or (low_saturation and in_lab(lab, low_saturation)):
        return "GREEN"
    return None


def scan_hazards(img, mirror, cache):
    # Only calibrated rays are observable. Two adjoining samples confirm a boundary.
    visible, boundaries = set(), []
    for bearing, samples in mirror.rays:
        if len(samples) < 2:
            continue
        previous = None
        observable = True
        for x, y, cm in samples:
            color = cache(img, x, y)
            if color is None:
                observable = False
            if color in ("WHITE", "BLACK") and previous is not None and previous[0] == color:
                boundaries.append((bearing,) + previous)
                break
            previous = (color, cm, x, y) if color in ("WHITE", "BLACK") else None
        if observable:
            visible.add(bearing)
    return visible, boundaries

def ground_color(img, x, y):
    return classify_rgb(img.get_pixel((x, y), rgbtuple=True))


class ColorCache:
    """ground_color()と同じ結果を、見たことのある色は表引きで返す（床の色は繰り返し現れる）。

    しきい値は起動中に変わらない前提。変える場合は作り直す。
    """

    def __init__(self):
        self.known = {}

    def __call__(self, img, x, y):
        rgb = img.get_pixel((x, y), rgbtuple=True)
        color = self.known.get(rgb, False)  # None is a real answer, so False marks "not seen"
        return self.learn(rgb) if color is False else color

    def learn(self, rgb):
        color = classify_rgb(rgb)
        if len(self.known) >= COLOR_CACHE_LIMIT:
            self.known.clear()  # the same dict object stays in use
        self.known[rgb] = color
        return color


class HazardColorCache(ColorCache):
    def learn(self, rgb):
        color = classify_hazard(rgb)
        if len(self.known) >= COLOR_CACHE_LIMIT:
            self.known.clear()
        self.known[rgb] = color
        return color


def scan_field(img, mirror, white_candidates=None, include_wall=False, thin=None, cache=None):
    """各方位の床色を調べ、走行可能セクターと境界候補を作る。"""
    # 各方位のサンプル色を分類し、床が見える方位集合と隣接確認済みの境界一覧に集約する。
    cache = cache or ColorCache()
    # Looked up once: this function runs hundreds of times per frame.
    get_pixel, lookup, learn = img.get_pixel, cache.known.get, cache.learn

    def color_at(x, y):
        rgb = get_pixel((x, y), rgbtuple=True)
        color = lookup(rgb, False)
        if color is False:
            color = learn(rgb)
        if color == "WHITE" and white_candidates is not None:
            white_candidates.add(bearing)
        return color

    visible = set()
    candidates = {}
    for bearing, samples in mirror.rays:
        ground, edge = scan_ray(samples, color_at, include_wall)
        if ground:
            visible.add(bearing)
        if edge is not None:
            candidates[bearing] = edge
            if thin is not None and edge[0] == "WHITE" and line_is_thin(samples, edge, color_at):
                thin.add(bearing)
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


def prepare_fast_scan(mirror, frames=3):
    """高速走査を作り、実フレームで従来版と一致した場合だけ返す。使えなければNone（従来版のまま）。"""
    try:
        import fast_scan
        started = time.ticks_ms()
        img = sensor.snapshot()
        table = fast_scan.build_table(img, classify_rgb)  # overwrites this frame: it is not used again
        scanner = fast_scan.FastScanner(mirror.rays, mirror.width, mirror.height, table, LINE_CLEARANCE_CM,
                                        line_is_thin)
        for _ in range(frames):
            if not scanner.matches(sensor.snapshot(), classify_rgb,
                                   lambda frame, thin: scan_field(frame, mirror, thin=thin)):
                print("fast scan OFF: result differs from the reference scan")
                return None
        print("fast scan ON (prepared in {} ms)".format(time.ticks_diff(time.ticks_ms(), started)))
        return scanner
    except Exception as error:  # any problem keeps the proven path
        print("fast scan OFF: {!r}".format(error))
        return None


def prepare_hazard_scan(mirror, cache):
    try:
        import fast_scan
        frame = sensor.snapshot()
        table = fast_scan.build_table(frame, classify_hazard)
        scanner = fast_scan.HazardScanner(mirror.rays, mirror.width, mirror.height, table,
                                         LINE_CLEARANCE_CM, line_is_thin)
        frame = sensor.snapshot()
        if scanner.scan(frame) != scan_hazards(frame, mirror, cache):
            return None
        return scanner
    except Exception as error:
        print("hazard fast scan OFF: {!r}".format(error))
        return None


def classify_white(rgb):
    return "WHITE" if in_lab(image.rgb_to_lab(rgb), cfg.WHITE) else None


def prepare_competition_scan(mirror):
    try:
        import fast_scan
        frame = sensor.snapshot()
        table = fast_scan.build_table(frame, classify_white)
        return fast_scan.CompetitionScanner(mirror.rays, mirror.width, mirror.height, table,
                                             LINE_CLEARANCE_CM, line_is_thin)
    except Exception as error:
        print("competition scan unavailable: {!r}".format(error))
        return None  # fail closed if the native path cannot start


def experimental_hazard_scan():
    # No colour-only method can establish visibility of arbitrary occluders.
    return bool(getattr(cfg, "EXPERIMENTAL_HAZARD_SCAN", False))


def floor_forced():
    """FORCE_FLOOR_OK: 「緑床か白線が一定数以上見える」という全体条件だけを成立扱いにする。

    方向ごとの確認は変わらず、床が見えない方向は閉じたまま。設定の異常と処理遅延の停止も変わらない。
    """
    return bool(getattr(cfg, "FORCE_FLOOR_OK", False))


def floor_report(img, mirror, classify, visible, boundaries, setup_reason, observed=0, mask=0):
    """NOFLOORの原因調べ: 各方位の最初の2点（床が見えるかを決める点）の色を数える。"""
    counts = {"GREEN": 0, "WHITE": 0, "BLACK": 0, None: 0}
    for _, samples in mirror.rays:
        for x, y, _ in samples[:2]:
            counts[classify(img, x, y)] += 1
    # The real result is logged even when FORCE_FLOOR_OK overrides it.
    return ("floor {} actual={} observed={}/{} forced={} mask={:04x} visible={} lines={} "
            "near-samples green={} white={} black={} other={}").format(
        setup_reason or "setup-ok", "OK" if observed >= MIN_FLOOR_RAYS else "NOFLOOR", observed, RAY_COUNT,
        int(floor_forced()), mask, len(visible), len(boundaries), counts["GREEN"], counts["WHITE"],
        counts["BLACK"], counts[None])


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
    """床観測と白線境界から、MAINへ渡す停止/直進/回避モードを選ぶ。"""
    # 校正/フレームが不十分なら停止。近い白線があれば離れる向き、なければ床が見える向きを許可。
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

    if (len(visible) < MIN_FLOOR_RAYS and not floor_forced()) or not mask:
        return stop
    return 0, 0, mask, MODE_TRACK_IR


def make_nav_packet(vx, vy, sectors, mode, sequence):
    """MAINの受信形式に合わせて、方向・セクターマスク・連番・検査値を8バイト化する。"""
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


def read_telemetry(uart, link, now):
    # The overlay never consumes more than 32 bytes in one camera iteration.
    waiting = uart.any()
    if waiting:
        link.feed(uart.read(min(waiting, 32)) or b"", now)


class LatestPacketTx:
    """Bounded nonblocking writer: one in-flight frame, never a telemetry queue."""
    def __init__(self, uart, ticks_diff, max_age_ms=100):
        self.uart, self.ticks_diff, self.max_age_ms = uart, ticks_diff, max_age_ms
        self.packet, self.offset, self.started = None, 0, 0

    def send(self, packet, now):
        if self.packet is not None and self.ticks_diff(now, self.started) > self.max_age_ms:
            self.packet = None  # receiver must resynchronise after a partial expired frame
        if self.packet is None:
            self.packet, self.offset, self.started = packet, 0, now
        try:
            count = self.uart.write(memoryview(self.packet)[self.offset:])
        except OSError:
            count = None
        if count:
            self.offset += count
            if self.offset >= len(self.packet):
                self.packet = None


def main_loop(send_uart=True, display=False):
    """撮影→検出→安全方向決定→必要ならMAINへ送信、をフレームごとに繰り返す。"""
    # 初期化後、各フレームで「撮影→床/境界走査→ゴール検出→判断→UART/表示」を実行。
    init_camera()
    frame = sensor.snapshot()
    mirror = Mirror(frame.width(), frame.height(), frame)
    print("mirror {} center=({}, {}) ring=({}, {})".format(
        "detected" if mirror.detected else "configured", mirror.cx, mirror.cy, mirror.inner, mirror.outer))
    target = cfg.TARGET_GOAL
    setup_reason = setup_wait_reason(mirror)
    vision = None
    if getattr(cfg, "PROTOCOL", 1) == 2:
        import competition_vision
        vision = competition_vision.Pipeline(cfg, mirror, LINE_CLEARANCE_CM, MAX_FRAME_AGE_MS)
    colors = ColorCache()
    hazard_colors = HazardColorCache()
    scanner = None
    stats = None
    if vision is not None:
        stats = competition_vision.Stats(("capture", "scan", "vision", "goal", "gap", "guard", "uart", "show",
                                          "search", "draw", "total"))

        def mark(name, since=None):
            now_us = time.ticks_us()
            if since is not None:
                stats.add(name, time.ticks_diff(now_us, since))
            return now_us

        vision.mark = mark
        if mirror.rays and getattr(cfg, "FAST_SCAN", True):
            scanner = (prepare_competition_scan(mirror) if getattr(cfg, "COMPETITION_TEST", False)
                       else prepare_hazard_scan(mirror, hazard_colors) if experimental_hazard_scan()
                       else prepare_fast_scan(mirror))
    overlay = None
    debug_goals = getattr(cfg, "GOAL_DEBUG", False)
    if vision is not None and (display or debug_goals):
        import visualize  # loaded only when something is shown, so competition runs pay nothing
        overlay = visualize.Overlay(cfg, mirror, show=display, debug_goals=debug_goals)
        overlay.mark = mark
    nonblocking = bool(getattr(cfg, "EXPERIMENTAL_NONBLOCKING_UART", False))
    uart = (pyb.UART(3, 115200 if vision else 9600, timeout=0, timeout_char=0)
            if send_uart and nonblocking else pyb.UART(3, 115200 if vision else 9600) if send_uart else None)
    writer = LatestPacketTx(uart, time.ticks_diff) if uart is not None and nonblocking else None
    last_tx = time.ticks_ms()
    last_command = None
    last_report = last_tx
    sequence = 0
    frames = shown_frames = 0
    shown = {"BLUE": (False, 0), "YELLOW": (False, 0)}
    loop_us = None
    while True:
        if stats is not None:
            tick = time.ticks_us()
            if loop_us is not None:
                stats.add("total", time.ticks_diff(tick, loop_us))
            loop_us = tick
        started = time.ticks_ms()
        # 先に床/白線を評価して安全指令を作る。ゴール表示など重い処理より走行許可を優先する。
        img = sensor.snapshot()
        captured = time.ticks_ms() if vision else started
        if stats is not None:
            shot = time.ticks_us()
            stats.add("capture", time.ticks_diff(shot, loop_us))
        white_candidates = set() if display and vision is None else None
        thin = set()
        if not mirror.rays:
            visible, boundaries = (), []
        elif vision is not None and getattr(cfg, "COMPETITION_TEST", False):
            visible, boundaries = scanner.scan(img) if scanner is not None else ((), [])
            thin = None
        elif vision is not None and experimental_hazard_scan():
            visible, boundaries = (scanner.scan(img) if scanner is not None
                                   else scan_hazards(img, mirror, hazard_colors))
            thin = None  # confirmation uses adjacent rays and successive frames
        elif scanner is not None:
            visible, boundaries = scanner.scan(img, thin)
        elif vision is not None:
            visible, boundaries = scan_field(img, mirror, white_candidates, thin=thin, cache=colors)
        else:
            visible, boundaries = scan_field(img, mirror, white_candidates)
        now = time.ticks_ms()
        frame_ms = time.ticks_diff(now, started)
        if vision is None:
            command = decide_navigation(setup_reason is None and frame_ms <= MAX_FRAME_AGE_MS,
                                        visible, boundaries)
            mode = command[3]
        if vision is not None:
            scanned = time.ticks_us()
            stats.add("scan", time.ticks_diff(scanned, shot))
            # Frame age is judged inside the pipeline, which also counts the detection time.
            observed = competition_vision.observed_rays(visible, boundaries)
            ready = setup_reason is None and (observed >= MIN_FLOOR_RAYS or floor_forced())
            packet = vision.step(img, now, ready, frame_ms, visible, boundaries, time.ticks_ms, thin,
                                 lambda x, y: colors(img, x, y))
            stepped = time.ticks_us()
            stats.add("vision", time.ticks_diff(stepped, scanned))
            if uart is not None:
                if writer is None:
                    uart.write(packet)
                else:
                    writer.send(packet, now)
                stats.add("uart", time.ticks_diff(time.ticks_us(), stepped))
        elif uart is not None and (command != last_command
                                 or time.ticks_diff(now, last_tx) >= STREAM_INTERVAL_MS):
            # 旧8-byte系は状態変化時または定期送信。新しい拡張系はPipelineが毎フレーム送る。
            vx, vy, sectors, _ = command
            packet = make_nav_packet(vx, vy, sectors, mode, sequence)
            if writer is None:
                uart.write(packet)
            else:
                writer.send(packet, now)
            sequence = (sequence + 1) & 0xff
            last_tx = now
            last_command = command
        frames += 1
        if (display or debug_goals or getattr(cfg, "DEBUG_STATS", False)) and time.ticks_diff(now, last_report) >= 2000:
            # FPS and stage times, to compare versions on the camera itself.
            # capture includes waiting for the exposure (EXPOSURE_US limits the frame rate).
            print("fps={:.1f} capture_ms={} line_ms={}".format(
                (frames - shown_frames) * 1000 / time.ticks_diff(now, last_report),
                time.ticks_diff(captured, started), time.ticks_diff(now, captured)))
            shown_frames, last_report = frames, now
            if vision is not None:
                # Repeated, because the startup lines scroll away: what is actually running on the camera.
                print("cfg scan={} VISUALIZE={} GOAL_DEBUG={} FORCE_FLOOR_OK={}".format(
                    "fast" if scanner is not None else "reference", int(display), int(bool(debug_goals)),
                    int(floor_forced())))
                print("ms avg/max/p95", stats.report())
                print(vision.timing, vision.status())
                if mirror.rays:
                    print(floor_report(img, mirror, colors, visible, boundaries, setup_reason,
                                       observed, vision.last[2]))
        if overlay is not None:
            show_start = time.ticks_us()
            # After the packet has been sent: display work never delays this frame's line data.
            if uart is not None:
                read_telemetry(uart, overlay.link, time.ticks_ms())
            overlay.draw(img, time.ticks_ms(), vision, boundaries, time.ticks_diff,
                         lambda x, y: colors(img, x, y))
            stats.add("show", time.ticks_diff(time.ticks_us(), show_start))
        elif display and vision is None:
            # Preview only (never sent to MAIN): one goal colour per frame, judged like protocol 3 does.
            name = ("BLUE", "YELLOW")[frames % 2]
            try:
                import competition_vision
            except ImportError:
                competition_vision = None
            if competition_vision is not None:
                min_pixels = getattr(cfg, name + "_MIN_PIXELS", 12)
                roi = None
                if mirror.ring_ready:
                    r = math.ceil(mirror.outer)
                    left, top = max(0, round(mirror.cx) - r), max(0, round(mirror.cy) - r)
                    roi = (left, top, min(mirror.width - left, 2 * r + 1), min(mirror.height - top, 2 * r + 1))
                found = img.find_blobs(getattr(cfg, name), roi=roi, pixels_threshold=min_pixels,
                                       area_threshold=min_pixels, x_stride=2, merge=True, margin=2)
                scored = [(competition_vision.goal_candidate(b, mirror, min_pixels, cfg,
                                                             lambda x, y: ground_color(img, x, y)), b)
                          for b in found]
                scored = [item for item in scored if item[0] is not None]
                shown[name] = (bool(scored), len(found))
                if scored:
                    draw_goal(img, max(scored, key=lambda item: item[0][1])[1], name)
            draw_mirror(img, mirror)
            draw_floor_scan(img, mirror)
            white_distances = []
            for _, _, distance, x, y in boundaries:
                white_distances.append(distance)
                img.draw_cross((x, y), color=(255, 255, 255))
            status = "TARGET: {}  MODE: {}".format(target, MODES[mode])
            img.draw_string((3, 3), status, color=(0, 0, 0))
            img.draw_string((2, 2), status, color=(255, 255, 255))
            if mode == MODE_WAIT or setup_reason:
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
            ring_status = "RING: " + ("AUTO" if mirror.detected else "CONFIG")
            img.draw_string((3, 39), ring_status, color=(0, 0, 0))
            img.draw_string((2, 38), ring_status, color=(255, 255, 255))
            for row, name in ((50, "BLUE"), (62, "YELLOW")):
                text = "{}: {} C{}".format(name, "YES" if shown[name][0] else "NO", shown[name][1])
                img.draw_string((3, row + 1), text, color=(0, 0, 0))
                img.draw_string((2, row), text, color=(255, 255, 255))


if __name__ == "__main__":
    # VISUALIZE = False draws nothing (competition). GOAL_DEBUG works with it either way.
    main_loop(send_uart=True, display=getattr(cfg, "VISUALIZE", False))
