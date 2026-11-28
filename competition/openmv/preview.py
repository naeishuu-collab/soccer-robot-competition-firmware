"""Annotation and bounded timing statistics for the OpenMV IDE."""
import math
from array import array
import camera_calibration as cfg

WHITE, RED, GREEN = (255, 255, 255), (255, 48, 48), (0, 255, 96)
GRAY, CYAN = (144, 144, 144), (0, 220, 255)
NAMES = ("YELLOW", "BLUE")
COLORS = ((255, 220, 0), (60, 150, 255))
METRICS = ("CAPTURE", "LINE_FLOOR", "MASK", "TX", "YELLOW", "BLUE", "DRAW",
           "FRAME", "LINE_INTERVAL", "YELLOW_INTERVAL", "BLUE_INTERVAL")


class Timings:
    def __init__(self):
        self.count = [0] * len(METRICS)
        self.total = [0] * len(METRICS)
        self.maximum = [0] * len(METRICS)
        self.bins = array("H", [0] * (32 * len(METRICS)))

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
        self.points = []
        if mirror.rays:
            for heading in range(16):
                angle = math.radians(cfg.FRONT_DEG + heading * 22.5 * (1 if cfg.CLOCKWISE else -1))
                self.points.append((round(mirror.cx + math.sin(angle) * mirror.outer),
                                    round(mirror.cy - math.cos(angle) * mirror.outer)))

    def draw(self, image, mode, now, ticks_diff, fps, frame_ms, guard, lines, mask, escape,
             nearest, current, goal_fields):
        if mode == "OFF":
            return
        full = mode == "FULL"
        if self.last_ms is not None and ticks_diff(now, self.last_ms) < (100 if full else 40):
            return
        self.last_ms = now
        image.draw_string(2, 2, "FPS %.1f  FRAME %dms" % (fps, frame_ms), color=WHITE)
        if full:
            image.draw_string(2, 52, "Proc %dms" % frame_ms, color=WHITE)
        for color in range(2):
            result = current[color]
            if result is not None:
                (angle, confidence), box, center = result
                image.draw_rectangle(box, color=COLORS[color])
                image.draw_cross(center[0], center[1], color=COLORS[color])
                image.draw_string(box[0], max(12, box[1] - 10),
                                  "%s %d %d%%" % (NAMES[color], angle, confidence), color=COLORS[color])
            elif goal_fields[color][0] != 65535 and full:
                image.draw_string(2, 22 + color * 10, "%s cached %d" % (NAMES[color], goal_fields[color][0]),
                                  color=GRAY)
        if not self.points:
            return
        mirror = self.mirror
        cx, cy = round(mirror.cx), round(mirror.cy)
        if nearest:
            for bearing, cm in lines.items() if full else (nearest,):
                samples = mirror.rays[bearing // 15][1]
                x, y, _ = min(samples, key=lambda sample: abs(sample[2] - cm))
                image.draw_cross(x, y, color=RED, size=6 if (bearing, cm) == nearest else 3)
            image.draw_string(2, 12, "LINE %ddeg %dcm" % nearest, color=RED)
        if escape:
            x, y = self.points[8 if escape & 256 else 0]
            image.draw_arrow(cx, cy, x, y, color=CYAN, thickness=2)
        if full:
            image.draw_circle(cx, cy, round(mirror.inner), color=GRAY)
            image.draw_circle(cx, cy, round(mirror.outer), color=GRAY)
            image.draw_cross(cx, cy, color=WHITE)
            x, y = self.points[0]
            image.draw_line(cx, cy, x, y, color=WHITE)
            for i, (x, y) in enumerate(self.points):
                bit = 1 << i
                color = RED if guard.blocked & bit else GREEN if mask & bit else GRAY
                image.draw_circle(x, y, 3, color=color, fill=True)
            image.draw_string(2, 43, "SAFE %04X ESC %04X" % (mask, escape), color=WHITE)
