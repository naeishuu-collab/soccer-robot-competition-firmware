"""認識結果と移動指令の表示（OpenMV IDE用）と、ゴール認識の診断。走行判断には何も戻さない。

goal_wall_navigation.py(main.py) が PROTOCOL = 2 かつ VISUALIZE か GOAL_DEBUG のときだけ読み込む。
オレンジの矢印はMAINから届いたIRのボール「方向」で、距離ではない。カメラはボールを認識しない。
"""
import math

import competition_vision as vision_module

TELEMETRY_SIZE = 19
STALE_LINK_MS = 400  # MAIN data older than this is not shown
STALE_BALL_MS = 200  # IR detection older than this (at the camera) is not shown
DEBUG_INTERVAL_FRAMES = 10
DEBUG_MAX_LISTED = 5

RED, YELLOW, BLUE = (255, 0, 0), (255, 255, 0), (0, 110, 255)
ORANGE, GREEN, WHITE, CYAN = (255, 128, 0), (0, 255, 0), (255, 255, 255), (0, 220, 220)
COLORS = {"YELLOW": YELLOW, "BLUE": BLUE}
# Order matches brain::Tactic and brain::Mode in main/main_logic.h.
TACTICS = ("SEARCH", "APPROACH", "ORBIT", "ALIGN", "ATTACK", "RECOVER", "AVOID_LINE", "SAFE_STOP")
MODES = ("FUSED", "BNO_ONLY", "VISION_ONLY", "DEGRADED")
BNO_STATES = ("OFF", "NO DATA", "USED")


class MainLink:
    """MAIN -> カメラの表示用データ（19バイト、D7 7D、CRC16-CCITT）を受け取る。"""

    def __init__(self):
        self.buffer = bytearray()
        self.data = None
        self.received_ms = 0
        self.errors = 0

    def feed(self, chunk, now):
        if not chunk:
            return
        self.buffer.extend(chunk)
        while True:
            start = 0
            while start < len(self.buffer) and self.buffer[start] != 0xD7:
                start += 1
            self.buffer = self.buffer[start:]
            if len(self.buffer) < TELEMETRY_SIZE:
                return
            f = self.buffer[:TELEMETRY_SIZE]
            if (f[1] != 0x7D or f[2] != 1 or f[3] != TELEMETRY_SIZE
                    or vision_module.crc16(f[:-2]) != f[-2] | f[-1] << 8):
                self.errors += 1
                self.buffer = self.buffer[1:]
                continue
            self.buffer = self.buffer[TELEMETRY_SIZE:]

            def angle(i):
                a = f[i] | f[i + 1] << 8
                return a if a < 360 else None

            ball = angle(6) if f[5] & 8 and f[9] != 255 else None
            self.data = {
                "sequence": f[4], "upgrade": bool(f[5] & 1), "ir_link": bool(f[5] & 2),
                "camera_link": bool(f[5] & 4), "hold": bool(f[5] & 16), "ball": ball,
                "ball_confidence": None if f[8] == 255 else f[8], "ball_age_ms": f[9] * 10,
                "direction": angle(10), "power": f[12], "turn": f[13] - 128,
                "tactic": TACTICS[f[14]] if f[14] < len(TACTICS) else "LEGACY",
                "mode": MODES[f[15]] if f[15] < len(MODES) else "LEGACY",
                "bno": BNO_STATES[f[16]] if f[16] < len(BNO_STATES) else "?",
            }
            self.received_ms = now

    def current(self, now, ticks_diff):
        """(data or None, ball bearing or None). 古い情報は現在値として返さない。"""
        if self.data is None:
            return None, None
        age = ticks_diff(now, self.received_ms)
        if age > STALE_LINK_MS:
            return None, None
        ball = self.data["ball"]
        if ball is not None and self.data["ball_age_ms"] + age > STALE_BALL_MS:
            ball = None
        return self.data, ball


def image_point(cfg, mirror, bearing, radius):
    angle = math.radians(cfg.FRONT_DEG + bearing * (1 if cfg.CLOCKWISE else -1))
    return round(mirror.cx + radius * math.sin(angle)), round(mirror.cy - radius * math.cos(angle))


def inspect_goals(img, name, cfg, mirror, color_of, stride=2):
    """色の候補をすべて（画面全体、小さいものも）調べ、合格→画素数の順に理由付きで返す。"""
    min_pixels = getattr(cfg, name + "_MIN_PIXELS", 12)
    reports = []
    for blob in img.find_blobs(getattr(cfg, name), pixels_threshold=3, area_threshold=3, x_stride=stride,
                               merge=True, margin=2):
        report = {}
        vision_module.goal_candidate(blob, mirror, min_pixels, cfg, color_of, report)
        reports.append(report)
    # Accepted candidates first, then by size: the first entry is what to show.
    reports.sort(key=lambda r: (r.get("reason") != "OK", -r["pixels"]))
    return reports


def describe(name, report):
    text = "{} {} px={} r={:.0f}".format(name, report.get("reason", "?"), report["pixels"], report["radius"])
    if "width" in report:
        text += " w={}deg asp={:.2f} den={:.2f} at={:.0f}".format(
            report["width"], report["aspect"], report["density"], report["bearing"])
    return text


class Overlay:
    def __init__(self, cfg, mirror, show=True, debug_goals=False):
        self.cfg = cfg
        self.mirror = mirror
        self.show = show
        self.debug_goals = debug_goals
        self.link = MainLink()
        self.frame = 0
        self.reports = {"YELLOW": [], "BLUE": []}
        self.fps = 0.0
        self.fps_ms = None
        self.fps_frames = 0

    def text(self, img, x, y, message, color=WHITE):
        img.draw_string((x + 1, y + 1), message, color=(0, 0, 0))
        img.draw_string((x, y), message, color=color)

    def arrow(self, img, bearing, radius, color, thickness=2):
        cx, cy = round(self.mirror.cx), round(self.mirror.cy)
        tip = image_point(self.cfg, self.mirror, bearing, radius)
        img.draw_line((cx, cy, tip[0], tip[1]), color=color, thickness=thickness)
        img.draw_circle((tip[0], tip[1], 3), color=color, thickness=thickness)

    def goal_status(self, name):
        """未検出の場合の説明。色が無いのか、候補があって除外されたのかを区別する。"""
        reports = self.reports[name]
        if not reports:
            return "NO COLOR"
        if reports[0].get("reason") == "OK":
            return "SEEN"
        return "REJECT {} x{}".format(reports[0].get("reason", "?"), len(reports))

    def draw(self, img, now, vision, boundaries, ticks_diff, color_of):
        self.frame += 1
        self.fps_frames += 1
        if self.fps_ms is None:
            self.fps_ms, self.fps_frames = now, 0
        elif ticks_diff(now, self.fps_ms) >= 1000:
            self.fps = self.fps_frames * 1000 / ticks_diff(now, self.fps_ms)
            self.fps_ms, self.fps_frames = now, 0
        # One colour per frame keeps the extra cost to a single search. Done before any drawing.
        name = ("YELLOW", "BLUE")[self.frame % 2]
        self.reports[name] = inspect_goals(img, name, self.cfg, self.mirror, color_of,
                                           1 if self.debug_goals else 2)
        if self.debug_goals and self.frame % DEBUG_INTERVAL_FRAMES in (0, 1):
            print("ring center=({}, {}) inner={} outer={} front={}".format(
                self.mirror.cx, self.mirror.cy, self.mirror.inner, self.mirror.outer, self.cfg.FRONT_DEG))
            if not self.reports[name]:
                print(name, "no colour candidate: check LAB threshold", getattr(self.cfg, name))
            for report in self.reports[name][:DEBUG_MAX_LISTED]:
                print(describe(name, report))
        if not self.show:
            return
        data, ball = self.link.current(now, ticks_diff)
        ready, floor_ok, mask, escape, _, yellow, blue = vision.last
        mirror = self.mirror
        cx, cy = round(mirror.cx), round(mirror.cy)
        for radius in (mirror.inner, mirror.outer):
            img.draw_circle((cx, cy, round(radius)), color=(128, 128, 128))
        for sector in range(16):  # safe directions: cyan ticks just outside the ring
            if mask & (1 << sector):
                a = image_point(self.cfg, mirror, sector * 22.5, mirror.outer - 6)
                b = image_point(self.cfg, mirror, sector * 22.5, mirror.outer + 6)
                img.draw_line((a[0], a[1], b[0], b[1]), color=CYAN, thickness=3)
        for edge in boundaries:
            img.draw_cross((edge[3], edge[4]), color=WHITE)
        self.arrow(img, 0, mirror.outer, RED, 1)
        goals = {"YELLOW": yellow, "BLUE": blue}
        for goal_name, reports in self.reports.items():
            for report in reports[:DEBUG_MAX_LISTED if self.debug_goals else 1]:
                accepted = report.get("reason") == "OK"
                if not accepted and not self.debug_goals:
                    continue
                img.draw_rectangle(report["box"], color=COLORS[goal_name], thickness=2 if accepted else 1)
                img.draw_cross((report["cx"], report["cy"]), color=COLORS[goal_name])
                label = "{:.0f}".format(report["bearing"]) if accepted else report.get("reason", "?")
                self.text(img, report["box"][0], max(0, report["box"][1] - 10), label, COLORS[goal_name])
        if ball is not None:
            self.arrow(img, ball, mirror.outer * 0.75, ORANGE)
        if data and data["direction"] is not None and data["power"]:
            self.arrow(img, data["direction"], mirror.outer * 0.55, GREEN, 3)

        row = 0
        for goal_name in ("YELLOW", "BLUE"):
            bearing, _, confidence = goals[goal_name][0], goals[goal_name][1], goals[goal_name][2]
            line = "{} {}".format(goal_name[0], "---" if bearing is None else "{:.0f} c{}".format(bearing, confidence))
            self.text(img, 2, row, line + " " + self.goal_status(goal_name), COLORS[goal_name])
            row += 10
        self.text(img, 2, row, "FPS {:.1f} {}".format(self.fps, "OK" if ready else "SLOW" if floor_ok else "NOFLOOR"))
        row = 190
        if data is None:
            self.text(img, 2, row, "MAIN: NO DATA (check J6 pin1 -> P5)", ORANGE)
            return
        self.text(img, 2, row, "IR ball {} conf {}".format(
            "---" if ball is None else ball,
            "-" if ball is None or data["ball_confidence"] is None else data["ball_confidence"]), ORANGE)
        self.text(img, 2, row + 10, "CMD {} pwr {} turn {}{}".format(
            "---" if data["direction"] is None else data["direction"], data["power"], data["turn"],
            " HOLD" if data["hold"] else ""), GREEN)
        self.text(img, 2, row + 20, "{} {}".format(data["tactic"], data["mode"]))
        self.text(img, 2, row + 30, "link IR {} CAM {} BNO {}".format(
            "OK" if data["ir_link"] else "NG", "OK" if data["camera_link"] else "NG", data["bno"]))
