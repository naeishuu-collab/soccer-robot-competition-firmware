#!/usr/bin/env python3
"""The fast floor scan must give exactly the reference result. (viper itself is only exercised on the camera.)"""
import random
import sys
import unittest
from array import array
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "openmv"))
import competition_vision as v
import fast_scan
import goal_wall_navigation as nav


def to_rgb(raw):
    return ((raw >> 11) << 3, ((raw >> 5) & 63) << 2, (raw & 31) << 3)


def classify(rgb):
    # Any deterministic split works here: the point is that both paths agree.
    return ("GREEN", "WHITE", "BLACK", None)[((rgb[0] >> 3) + (rgb[1] >> 2) * 3 + (rgb[2] >> 3) * 7) % 4]


SHADE = {}
for value in range(4000):
    SHADE.setdefault(classify(to_rgb(value)), []).append(value)


class Frame:
    def __init__(self, width=320, height=240):
        self.w, self.h = width, height
        self.buffer = bytearray(width * height * 2)
        self.pixels = memoryview(self.buffer).cast("H")

    def width(self):
        return self.w

    def bytearray(self):
        return self.buffer

    def get_pixel(self, position, *, rgbtuple):
        return to_rgb(self.pixels[position[1] * self.w + position[0]])


class Lab:
    rgb_to_lab = staticmethod(lambda rgb: rgb)


def paint(frame, rays, rng):
    for _, samples in rays:
        i = 0
        while i < len(samples):
            color = rng.choice(("GREEN", "GREEN", "GREEN", "WHITE", "WHITE", "BLACK", None))
            run = rng.choice((1, 1, 2, 3, 5, 12))
            for x, y, _ in samples[i:i + run]:
                frame.pixels[y * frame.w + x] = rng.choice(SHADE[color])
            i += run


class FastScanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mirror = nav.Mirror(320, 240)
        cls.table = fast_scan.build_table(Frame(), classify)

    def scanner(self, clearance=None):
        return fast_scan.FastScanner(self.mirror.rays, 320, 240, self.table,
                                     nav.LINE_CLEARANCE_CM if clearance is None else clearance, nav.line_is_thin)

    def reference(self, frame, thin):
        with patch.object(nav, "classify_rgb", classify):
            return nav.scan_field(frame, self.mirror, thin=thin)

    def test_table_covers_every_colour(self):
        for raw in range(0, 65536, 97):
            self.assertEqual(fast_scan.NAMES[self.table[raw]], classify(to_rgb(raw)))
        self.assertNotIn(0, self.table)

    def test_identical_to_the_reference_on_random_floors(self):
        rng = random.Random(3)
        scanner, frame = self.scanner(), Frame()
        found = thin_total = hidden = 0
        for i in range(3000):
            paint(frame, self.mirror.rays, rng)
            fast_thin, slow_thin = set(), set()
            fast = scanner.scan(frame, fast_thin)
            slow = self.reference(frame, slow_thin)
            self.assertEqual(fast, slow, i)
            self.assertEqual(fast_thin, slow_thin, i)
            found += len(fast[1])
            thin_total += len(fast_thin)
            hidden += 24 - len(fast[0])
        # The comparison exercised lines, thin lines and unseen floor.
        self.assertGreater(min(found, thin_total, hidden), 1000)

    def test_identical_at_other_clearances(self):
        rng = random.Random(4)
        frame = Frame()
        for clearance in (18, 30, 50):
            scanner = self.scanner(clearance)
            with patch.object(nav, "LINE_CLEARANCE_CM", clearance):
                for _ in range(200):
                    paint(frame, self.mirror.rays, rng)
                    self.assertEqual(scanner.scan(frame), self.reference(frame, None) if False else
                                     self.reference(frame, set()))

    def test_unrecognised_floor_stays_unrecognised(self):
        frame = Frame()
        for i in range(320 * 240):
            frame.pixels[i] = SHADE[None][0]
        visible, boundaries = self.scanner().scan(frame)
        self.assertEqual((visible, boundaries), (set(), []))
        self.assertEqual(v.LineGuard(25, 35).update(False, visible, boundaries)[0], 0)

    def test_self_check_accepts_a_match_and_rejects_any_difference(self):
        frame = Frame()
        paint(frame, self.mirror.rays, random.Random(5))
        self.assertTrue(self.scanner().matches(frame, classify, self.reference))
        broken = bytearray(self.table)
        x, y, _ = self.mirror.rays[0][1][0]
        raw = frame.pixels[y * 320 + x]
        broken[raw] = broken[raw] % 4 + 1
        bad = fast_scan.FastScanner(self.mirror.rays, 320, 240, broken, 25, nav.line_is_thin)
        self.assertFalse(bad.matches(frame, classify, self.reference))
        wrong = fast_scan.FastScanner(self.mirror.rays, 320, 240, self.table, 18, nav.line_is_thin)
        self.assertFalse(wrong.matches(frame, classify,
                                       lambda f, thin: (set(range(0, 360, 15)), [])))

    def test_camera_falls_back_when_anything_is_wrong(self):
        class Sensor:
            @staticmethod
            def snapshot():
                return object()  # no buffer access at all

        class Time:
            ticks_ms = staticmethod(lambda: 0)
            ticks_diff = staticmethod(lambda a, b: a - b)

        with patch.multiple(nav, sensor=Sensor, time=Time), patch("builtins.print") as log:
            self.assertIsNone(nav.prepare_fast_scan(self.mirror))
        self.assertIn("fast scan OFF", log.call_args[0][0])

    def test_camera_uses_it_only_after_matching_frames(self):
        frame = Frame()
        paint(frame, self.mirror.rays, random.Random(6))

        class Sensor:
            shots = 0

            @staticmethod
            def snapshot():
                Sensor.shots += 1
                if Sensor.shots == 1:
                    return Frame()  # the table is built over this one
                return frame

        class Time:
            ticks_ms = staticmethod(lambda: 0)
            ticks_diff = staticmethod(lambda a, b: a - b)

        with patch.multiple(nav, sensor=Sensor, time=Time, classify_rgb=classify), patch("builtins.print"):
            self.assertIsNotNone(nav.prepare_fast_scan(self.mirror))
        self.assertEqual(Sensor.shots, 4)

    def test_sample_outside_the_image_is_refused(self):
        with self.assertRaises(ValueError):
            fast_scan.FastScanner([(0, [(320, 0, 20)])], 320, 240, self.table, 25, nav.line_is_thin)


class MainLoopTests(unittest.TestCase):
    """The protocol 2 camera loop end to end, with and without the fast scan and the overlay."""

    def run_loop(self, display, fast, debug=False, frames=60):
        mirror = nav.Mirror(320, 240)
        painted = Frame()
        paint(painted, mirror.rays, random.Random(7))
        original = bytes(painted.buffer)

        class Image(Frame):
            def __init__(self):
                super().__init__()
                self.buffer[:] = original

            def height(self):
                return self.h

            def find_blobs(self, thresholds, **kwargs):
                return []

            def __getattr__(self, name):
                if not name.startswith("draw_"):
                    raise AttributeError(name)
                return lambda *args, **kwargs: None

        class Sensor:
            shots = 0

            @staticmethod
            def snapshot():
                Sensor.shots += 1
                if Sensor.shots > frames:
                    raise StopIteration
                return Image()

        class Time:
            now = 0

            @staticmethod
            def ticks_ms():
                Time.now += 12
                return Time.now

            ticks_us = staticmethod(lambda: Time.now * 1000)
            ticks_diff = staticmethod(lambda a, b: a - b)

        sent = []

        class Board:
            @staticmethod
            def UART(port, baud):
                return Board()

            def write(self, packet):
                sent.append(bytes(packet))

            def any(self):
                return 0

        with patch.multiple(nav, sensor=Sensor, time=Time, pyb=Board, image=Lab, classify_rgb=classify,
                            init_camera=lambda: None), \
                patch.multiple(nav.cfg, PROTOCOL=2, GOAL_DEBUG=debug, FAST_SCAN=fast, create=True), \
                patch("builtins.print") as log:
            with self.assertRaises(StopIteration):
                nav.main_loop(display=display)
        return sent, " ".join(str(a) for call in log.call_args_list for a in call[0])

    def test_every_configuration_runs_and_reports_timing(self):
        for display, debug in ((False, False), (True, False), (True, True), (False, True)):
            sent, output = self.run_loop(display, True, debug)
            self.assertIn("fast scan ON", output)
            self.assertIn("ms avg/max/p95", output)
            for stage in ("capture=", "scan=", "vision=", "uart=", "total="):
                self.assertIn(stage, output)
            self.assertEqual("show=" in output, display or debug)
            self.assertIn("near-samples green=", output)
            self.assertIn("actual=", output)
            self.assertIn("forced=", output)
            self.assertTrue(sent and all(len(p) == v.PACKET_SIZE for p in sent))

    def test_report_names_the_running_configuration_and_stages(self):
        for fast, scan in ((True, "fast"), (False, "reference")):
            output = self.run_loop(True, fast, True, frames=200)[1]
            self.assertIn("cfg scan={} VISUALIZE=1 GOAL_DEBUG=1".format(scan), output)
            for stage in ("goal=", "gap=", "guard=", "search=", "draw="):
                self.assertIn(stage, output)

    def test_fast_and_reference_paths_send_identical_packets(self):
        fast, _ = self.run_loop(False, True)
        slow, output = self.run_loop(False, False)
        self.assertNotIn("fast scan ON", output)
        # The fast path uses three extra frames for its self-check; the packets themselves match.
        self.assertEqual(fast, slow[:len(fast)])


class ForcedFloorTests(unittest.TestCase):
    """FORCE_FLOOR_OK lifts only the overall floor count; unknown directions stay closed."""

    EVERY = set(range(0, 360, 15))
    FEW = {150, 165, 180, 195, 210}  # five rays: far below the traditional minimum of 12

    def packet(self, forced, visible, boundaries=(), setup=None, frame_ms=30):
        class Empty:
            def find_blobs(self, *args, **kwargs):
                return []

        with patch.multiple(nav.cfg, IMAGE_SIZE=(320, 240), MIRROR_CENTER=(160, 120), MIRROR_RING=(20, 100),
                            FORCE_FLOOR_OK=forced, create=True):
            pipe = v.Pipeline(nav.cfg, nav.Mirror(320, 240), 25, 100)
            observed = v.observed_rays(visible, list(boundaries))
            ready = setup is None and (observed >= nav.MIN_FLOOR_RAYS or nav.floor_forced())
            packet = pipe.step(Empty(), 0, ready, frame_ms, visible, list(boundaries))
            return packet, packet[6] | packet[7] << 8, pipe

    def test_false_keeps_the_traditional_stop(self):
        packet, mask, pipe = self.packet(False, self.FEW)
        self.assertFalse(packet[5] & v.FLAG_READY)
        self.assertEqual(mask, 0)
        self.assertIn("NOFLOOR", pipe.status())

    def test_forced_opens_only_the_directions_with_seen_floor(self):
        packet, mask, pipe = self.packet(True, self.FEW)
        self.assertTrue(packet[5] & v.FLAG_READY)
        self.assertEqual(mask, 1 << 8)  # only 180 degrees has its whole +-30 window seen
        self.assertIn("OK (FORCED)", pipe.status())

    def test_forced_with_no_seen_floor_still_allows_nothing(self):
        packet, mask, _ = self.packet(True, set())
        self.assertTrue(packet[5] & v.FLAG_READY)
        self.assertEqual(mask, 0)  # MAIN stops with "no safe direction"

    def test_forced_keeps_setup_and_slow_stops(self):
        packet, mask, _ = self.packet(True, self.EVERY, setup="CHECK GROUND_POINTS")
        self.assertFalse(packet[5] & v.FLAG_READY)
        self.assertEqual(mask, 0)
        packet, mask, _ = self.packet(True, self.EVERY, frame_ms=101)
        self.assertFalse(packet[5] & v.FLAG_READY)
        self.assertTrue(packet[5] & v.FLAG_SLOW)
        self.assertEqual(mask, 0)

    def test_forced_keeps_white_line_escape_and_closed_headings(self):
        wall = [(b, "WHITE", 15, 0, 0) for b in (345, 0, 15)]
        packet, mask, _ = self.packet(True, self.EVERY, wall)
        self.assertTrue(packet[5] & v.FLAG_ESCAPE)
        self.assertEqual(packet[8] | packet[9] << 8, 180)
        self.assertFalse(mask & 1)

    def test_traditional_protocol_follows_the_same_switch(self):
        with patch.object(nav.cfg, "FORCE_FLOOR_OK", True, create=True):
            self.assertEqual(nav.decide_navigation(True, self.FEW, [])[3], nav.MODE_TRACK_IR)
            self.assertEqual(nav.decide_navigation(True, set(), []), (0, 0, 0, nav.MODE_WAIT))
            self.assertEqual(nav.decide_navigation(False, self.EVERY, []), (0, 0, 0, nav.MODE_WAIT))
        with patch.object(nav.cfg, "FORCE_FLOOR_OK", False, create=True):
            self.assertEqual(nav.decide_navigation(True, self.FEW, []), (0, 0, 0, nav.MODE_WAIT))

    def test_missing_setting_means_traditional(self):
        saved = nav.cfg.__dict__.pop("FORCE_FLOOR_OK", None)
        try:
            self.assertFalse(nav.floor_forced())
        finally:
            if saved is not None:
                nav.cfg.FORCE_FLOOR_OK = saved

    def test_log_keeps_the_real_result(self):
        with patch.object(nav.cfg, "FORCE_FLOOR_OK", True, create=True):
            line = nav.floor_report(None, nav.Mirror(320, 240), lambda *a: None, self.FEW, [], None, 5, 0x100)
        self.assertIn("actual=NOFLOOR observed=5/24 forced=1 mask=0100", line)


class ForcedFloorCostTests(unittest.TestCase):
    """Performance guard: forcing the floor condition must not add image searches or other image work."""

    def work(self, forced):
        asked = []

        class Image:
            def find_blobs(self, thresholds, **kwargs):
                asked.append((str(thresholds), kwargs["roi"], kwargs["x_stride"]))
                return []

            def __getattr__(self, name):
                raise AssertionError("unexpected image access: " + name)

        few = set(range(0, 90, 15))
        with patch.multiple(nav.cfg, IMAGE_SIZE=(320, 240), MIRROR_CENTER=(160, 120), MIRROR_RING=(20, 100),
                            FORCE_FLOOR_OK=forced, create=True):
            pipe = v.Pipeline(nav.cfg, nav.Mirror(320, 240), 25, 100)
            for i in range(120):
                ready = v.observed_rays(few, []) >= nav.MIN_FLOOR_RAYS or nav.floor_forced()
                pipe.step(Image(), i * 40, ready, 20, few, [])
        return asked

    def test_same_image_work_with_and_without_the_override(self):
        forced = self.work(True)
        self.assertEqual(forced, self.work(False))
        self.assertGreater(len(forced), 0)


class TimingAndReleaseTests(unittest.TestCase):
    def test_statistics(self):
        stats = v.Stats(("scan",), size=100)
        for value in range(1, 251):
            stats.add("scan", value * 1000)
        average, worst, p95 = stats.summary("scan")  # the last 100 values: 151..250
        self.assertEqual((average, worst, p95), (200.5, 250.0, 245.0))
        self.assertIn("scan=200.5/250.0/245.0", stats.report())
        self.assertEqual(stats.report(), "")

    def test_line_release_takes_time_as_well_as_frames(self):
        every = set(range(0, 360, 15))
        wall = [(b, "WHITE", 15, 0, 0) for b in (345, 0, 15)]
        guard = v.LineGuard(25, 35)
        self.assertEqual(guard.update(True, every, wall, None, 1000)[1], 180)
        for now in (1020, 1040, 1060, 1080):  # a 50 fps camera
            self.assertEqual(guard.update(True, every, [], None, now)[1], 180, now)
        self.assertIsNone(guard.update(True, every, [], None, 1100)[1])

    def test_goal_search_rate_does_not_rise_with_frame_rate(self):
        asked = []

        class Frame_:
            def find_blobs(self, thresholds, **kwargs):
                asked.append(thresholds)
                return []

        with patch.multiple(nav.cfg, IMAGE_SIZE=(320, 240), MIRROR_CENTER=(160, 120), MIRROR_RING=(20, 100)):
            pipe = v.Pipeline(nav.cfg, nav.Mirror(320, 240), 25, 100)
            for i in range(200):  # 100 fps for two seconds
                pipe.step(Frame_(), i * 10, True, 5, set(range(0, 360, 15)), [])
        self.assertLessEqual(len(asked), 2 * 2000 // v.GOAL_MIN_INTERVAL_MS)
        self.assertGreater(len(asked), 20)


if __name__ == "__main__":
    unittest.main()
