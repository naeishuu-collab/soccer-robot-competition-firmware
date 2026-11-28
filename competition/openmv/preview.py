"""Annotation and bounded timing statistics for the OpenMV IDE."""
import math
from array import array
import camera_calibration as cfg
from vision import BLOCK_CM

PRODUCTION_API = 2
WHITE, RED, GREEN = (255, 255, 255), (255, 48, 48), (0, 255, 96)
GRAY, CYAN = (144, 144, 144), (0, 220, 255)
NAMES = ("YELLOW", "BLUE")
COLORS = ((255, 220, 0), (60, 150, 255))
METRICS = ("CAPTURE", "LINE_FLOOR", "MASK", "TX", "YELLOW", "BLUE", "DRAW",
           "FRAME", "LINE_INTERVAL", "YELLOW_INTERVAL", "BLUE_INTERVAL", "GOALS_BOTH")


class Timings:
    def __init__(self):
        self.count = [0] * len(METRICS)
        self.total = [0] * len(METRICS)
        self.maximum = [0] * len(METRICS)
        self.bins = array("H", [0] * (32 * len(METRICS)))

    def reset(self):
        for i in range(len(METRICS)):
            self.count[i] = self.total[i] = self.maximum[i] = 0
        for i in range(len(self.bins)):
            self.bins[i] = 0

    def add(self, index, us):
        self.count[index] += 1
        self.total[index] += us
        if us > self.maximum[index]:
            self.maximum[index] = us
        self.bins[index * 32 + min(31, us // 4000)] += 1

    def report(self):
        values = []
        for i, name in enumerate(METRICS):
            count = self.count[i]
            if not count:
                continue
            need = (95 * count + 99) // 100
            for bucket in range(32):
                need -= self.bins[i * 32 + bucket]
                if need <= 0:
                    break
            p95 = ">124000" if bucket == 31 else str((bucket + 1) * 4000)
            values.append("%s=%d/%s/%d" % (name, self.total[i] // count, p95, self.maximum[i]))
        return "AVG/P95_MAX_BIN/MAX_US " + " ".join(values)


class Overlay:
    def __init__(self, mirror):
        self.mirror = mirror
        self.last_ms = None
        self.errors = 0
        self.last_error = "none"
        self.disabled = set()
        self.points = []
        self.ground_end = [max(i for i, sample in enumerate(ray) if sample[2] <= BLOCK_CM)
                           for _, ray in mirror.rays]
        if mirror.rays:
            for heading in range(16):
                angle = math.radians(cfg.FRONT_DEG + heading * 22.5 * (1 if cfg.CLOCKWISE else -1))
                self.points.append((round(mirror.cx + math.sin(angle) * mirror.outer),
                                    round(mirror.cy - math.cos(angle) * mirror.outer)))

    def primitive(self, image, name, coords, color, text=None, **style):
        if name in self.disabled:
            return
        try:
            draw = getattr(image, name)
            if text is None:
                draw(coords, color=color, **style)
            else:
                draw(coords, text, color=color, **style)
        except (AttributeError, TypeError, ValueError, MemoryError, OSError, RuntimeError) as exc:
            self.errors += 1
            self.last_error = "%s/%s:%s" % (name, type(exc).__name__, str(exc)[:80])
            self.disabled.add(name)

    def draw(self, image, mode, now, ticks_diff, fps, frame_ms, guard, lines, mask, escape,
             nearest, current, goal_fields, visible=()):
        if mode == "OFF":
            return
        full = mode == "FULL"
        due = self.last_ms is None or ticks_diff(now, self.last_ms) >= (100 if full else 20)
        draw = self.primitive
        # Always ink fresh detections, even between FULL overlay refreshes.
        for color in range(2):
            result = current[color]
            if result is not None:
                (angle, confidence), box, center = result
                draw(image, "draw_rectangle", box, COLORS[color])
                draw(image, "draw_cross", center, COLORS[color])
                draw(image, "draw_string", (box[0], max(12, box[1] - 10)), COLORS[color],
                     "%s %d %d%%" % (NAMES[color], angle, confidence))
            elif due and full and goal_fields[color][0] != 65535:
                draw(image, "draw_string", (2, 22 + color * 10), GRAY,
                     "%s cached %d" % (NAMES[color], goal_fields[color][0]))
        if not due:
            return
        self.last_ms = now
        draw(image, "draw_string", (2, 2), WHITE, "FPS %.1f  FRAME %dms" % (fps, frame_ms))
        if not self.points:
            return
        mirror = self.mirror
        cx, cy = round(mirror.cx), round(mirror.cy)
        if nearest:
            for bearing, cm in lines.items() if full else (nearest,):
                samples = mirror.rays[bearing // 15][1]
                x, y, _ = min(samples, key=lambda sample: abs(sample[2] - cm))
                color = RED if bearing in guard.confirmed else CYAN
                draw(image, "draw_cross", (x, y), color, size=6 if (bearing, cm) == nearest else 3)
            draw(image, "draw_string", (2, 12), RED, "LINE %ddeg %dcm" % (nearest[0], round(nearest[1])))
        if escape:
            x, y = self.points[8 if escape & 256 else 0]
            draw(image, "draw_arrow", (cx, cy, x, y), CYAN, thickness=2)
        if full:
            draw(image, "draw_circle", (cx, cy, round(mirror.inner)), GRAY)
            draw(image, "draw_circle", (cx, cy, round(mirror.outer)), GRAY)
            draw(image, "draw_cross", (cx, cy), WHITE)
            x, y = self.points[0]
            draw(image, "draw_line", (cx, cy, x, y), WHITE)
            for ray in visible:
                samples = mirror.rays[ray // 15][1]
                x0, y0, _ = samples[0]
                x1, y1, _ = samples[self.ground_end[ray // 15]]
                draw(image, "draw_line", (x0, y0, x1, y1), GREEN)
            for i, (x, y) in enumerate(self.points):
                bit = 1 << i
                color = RED if guard.blocked & bit else GREEN if mask & bit else GRAY
                draw(image, "draw_circle", (x, y, 3), color, fill=True)
            draw(image, "draw_string", (2, 43), WHITE, "SAFE %04X ESC %04X" % (mask, escape))
