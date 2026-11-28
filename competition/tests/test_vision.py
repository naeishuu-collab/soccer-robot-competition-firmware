from pathlib import Path
import random
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "openmv"))
import vision
import preview


class Raster:
    def __init__(self, colors):
        self.colors = colors
        self.frame = bytearray(320 * 240 * 2)
        self.pixels = memoryview(self.frame).cast("H")

    def get_pixel(self, point, rgbtuple):
        x, y = point
        return self.colors[self.pixels[y * 320 + x] % len(self.colors)]

    def bytearray(self):
        return self.frame


class VisionTests(unittest.TestCase):
    def setUp(self):
        self.mirror = vision.Mirror(320, 240)
        self.rays = set(range(0, 360, 15))
        self.assertEqual(len(self.mirror.rays), 24)
        self.guard = vision.LineGuard(lambda a, b: a - b)

    def test_calibration_and_colors(self):
        self.assertEqual(vision.Mirror(160, 120).rays, [])
        self.assertAlmostEqual(self.mirror.bearing(221, 129), 356)
        self.assertEqual(len(vision.cfg.YELLOW), 5)
        self.assertEqual(len(vision.cfg.BLUE), 2)
        self.assertEqual(vision.classify((37, -9, 5)), 1)
        self.assertEqual(vision.classify((80, 0, 0)), 2)
        self.assertEqual(vision.classify((10, 0, 0)), 0)

    def test_scan_kernel_against_reference(self):
        colors = [(37, -9, 5), (80, 0, 0), (10, 0, 0)]
        raster = Raster(colors)
        scanner = vision.Scanner(self.mirror, 320, lambda color: color)
        scanner.prepare(raster)
        rng = random.Random(420)
        for sample in range(200):
            for _, ray in self.mirror.rays:
                for x, y, _ in ray:
                    raster.pixels[y * 320 + x] = rng.randrange(3)
            self.assertTrue(scanner.verify(raster))
            scanner.fast = False
            expected = scanner.scan(raster)
            scanner.fast = True
            self.assertEqual(scanner.scan(raster), expected)

    def test_unknown_never_clears_and_far_line_is_irrelevant(self):
        self.assertEqual(self.guard.update(set(), {}, 0), (0, 0, None))
        mask, escape, nearest = self.guard.update(self.rays, {180: 45}, 1)
        self.assertEqual((mask, escape, nearest), (65535, 0, (180, 45)))
        mask, _, _ = self.guard.update(self.rays - {180}, {}, 2)
        self.assertTrue(mask & 1)
        self.assertFalse(mask & 256)

    def test_escape_and_hysteresis(self):
        _, escape, _ = self.guard.update(self.rays - {0, 15}, {0: 20, 15: 21}, 1)
        self.assertEqual(escape, 256)
        self.assertEqual(self.guard.update(self.rays - {0, 15}, {}, 200)[1], 256)
        self.assertTrue(self.guard.near)
        self.assertEqual(self.guard.update(self.rays - {0, 15}, {}, 201)[1], 0)
        self.guard.update(self.rays, {}, 220)
        self.guard.update(self.rays, {}, 260)
        self.assertFalse(self.guard.near)
        _, escape, _ = self.guard.update(self.rays - {0, 15, 180, 195}, {0: 20, 15: 21, 180: 21, 195: 22}, 300)
        self.assertEqual(escape, 0)
        self.assertTrue(self.guard.near)

    def test_goal_shape_and_ttl(self):
        class Blob:
            def rect(self): return 148, 68, 46, 8
            def cx(self): return 171
            def cy(self): return 72
            def pixels(self): return 300

        class Frame:
            def find_blobs(self, thresholds, **options):
                return [Blob()]

        goal = vision.find_goal(Frame(), self.mirror, vision.cfg.BLUE, 8)
        self.assertEqual(goal, (86, 100))

        class BlobV5:
            rect = (148, 68, 46, 8)
            cx = 171
            cy = 72
            pixels = 300

        class FrameV5:
            def find_blobs(self, thresholds, **options):
                return [BlobV5()]

        self.assertEqual(vision.find_goal(FrameV5(), self.mirror, vision.cfg.BLUE, 8), goal)
        self.assertEqual(vision.find_goal(Frame(), self.mirror, vision.cfg.BLUE, 8,
                                          lambda _img, _x, _y: 1), (86, 50))
        self.assertEqual(vision.goal_fields((goal, 500), 799, lambda a, b: a-b), (86, 100, 30))
        self.assertEqual(vision.goal_fields((goal, 500), 800, lambda a, b: a-b), (65535, 0, 255))

    def test_mask_table_matches_reference(self):
        rng = random.Random(821)
        for sample in range(500):
            visible = {bearing for bearing in self.rays if rng.randrange(4)}
            lines = {bearing: rng.randrange(31, 51) for bearing in self.rays if rng.randrange(8) == 0}
            expected = sum(1 << i for i, rays in enumerate(vision.SECTOR_WINDOWS)
                           if all(ray in visible for ray in rays) and all(
                               vision.difference(i * 22.5, b) >= 60 for b, cm in lines.items() if cm <= 35))
            self.assertEqual(self.guard.update(visible, lines, sample)[0], expected)

    def test_crop_keeps_calibrated_ray_coordinates(self):
        crop = (80, 48, 176, 168)
        translated = vision.Mirror(176, 168, crop)
        self.assertEqual(len(translated.rays), 24)
        for (old_heading, old), (new_heading, new) in zip(self.mirror.rays, translated.rays):
            self.assertEqual(old_heading, new_heading)
            self.assertEqual(new, [(x-80, y-48, cm) for x, y, cm in old])
        self.assertEqual(vision.Mirror(120, 120, (80,48,120,120)).rays, [])

    def test_overlay_modes_and_stale_goals(self):
        calls = []

        class Surface:
            def __getattr__(self, name):
                return lambda *args, **kwargs: calls.append((name, args, kwargs))

        overlay = preview.Overlay(self.mirror)
        field = Surface()
        mask, escape, nearest = self.guard.update(self.rays - {0, 15}, {0: 20, 15: 21}, 1)
        goals = ((55, 80, 0), (122, 60, 5))
        current = (((55, 80), (10, 25, 20, 10), (20, 30)), None)

        def draw(mode, now):
            overlay.draw(field, mode, now, lambda a,b: a-b, 25, 40, self.guard,
                         {0: 20, 15: 21}, mask, escape, nearest, current, goals)

        draw("OFF", 0)
        self.assertEqual(calls, [])
        draw("LIGHT", 50)
        self.assertEqual(sum(name == "draw_rectangle" for name, _, _ in calls), 1)
        self.assertFalse(any(name == "draw_circle" for name, _, _ in calls))
        self.assertTrue(any(name == "draw_arrow" for name, _, _ in calls))
        calls.clear()
        draw("FULL", 200)
        self.assertEqual(sum(name == "draw_circle" for name, _, _ in calls), 18)
        self.assertEqual(sum(name == "draw_rectangle" for name, _, _ in calls), 1)
        self.assertTrue(any("BLUE cached" in args[-1] for name, args, _ in calls if name == "draw_string"))
        calls.clear()
        draw("FULL", 220)
        self.assertEqual([name for name, _, _ in calls], ["draw_rectangle", "draw_cross", "draw_string"])

    def test_goal_threshold_probe_separates_small_regions(self):
        class Small:
            def rect(self): return 148, 68, 3, 2
            def cx(self): return 149
            def cy(self): return 69
            def pixels(self): return 5

        class Frame:
            def find_blobs(self, thresholds, **kwargs):
                assert kwargs["pixels_threshold"] == 1
                return [Small()]

        counts = []
        self.assertIsNone(vision.find_goal(Frame(), self.mirror, vision.cfg.YELLOW, 12, report=counts, probe=True))
        self.assertEqual(counts[:2], [1, 1])

    def test_v5_coordinates_and_failed_primitive_isolation(self):
        calls = []

        class StrictV5:
            sizes = {"draw_string": 2, "draw_cross": 2, "draw_circle": 3,
                     "draw_rectangle": 4, "draw_line": 4, "draw_arrow": 4}

            def __getattr__(self, name):
                def draw(coords, *text, **style):
                    if not isinstance(coords, tuple):
                        raise TypeError("object isn't a tuple or list")
                    assert len(coords) == self.sizes[name] and len(style["color"]) == 3
                    assert len(text) == (1 if name == "draw_string" else 0)
                    calls.append(name)
                    if name == "draw_string":
                        raise TypeError("simulated text formatter failure")
                return draw

        overlay = preview.Overlay(self.mirror)
        lines = {180: 20, 195: 22}
        visible = self.rays - set(lines)
        mask, escape, nearest = self.guard.update(visible, lines, 0)
        current = (((0, 80), (10, 30, 20, 10), (20, 35)), ((90, 70), (60, 40, 20, 10), (70, 45)))
        overlay.draw(StrictV5(), "FULL", 0, lambda a,b: a-b, 25, 40, self.guard, lines, mask, escape,
                     nearest, current, ((0,80,0),(90,70,0)), visible)
        self.assertEqual(overlay.errors, 1)
        self.assertIn("draw_string/TypeError", overlay.last_error)
        self.assertEqual(calls.count("draw_rectangle"), 2)
        self.assertIn("draw_arrow", calls)
        self.assertIn("draw_line", calls)
        self.assertIn("draw_circle", calls)

    def test_paired_native_blobs_keep_colors_separate(self):
        class Blob:
            rect, cx, cy, pixels = (148, 68, 46, 8), 171, 72, 300

            def __init__(self, code):
                self.code = code

        yellow = Blob(2)
        blue = Blob(1 << len(vision.cfg.YELLOW))
        mixed = Blob(yellow.code | blue.code)

        class Frame:
            calls = 0

            def find_blobs(self, thresholds, **kwargs):
                self.calls += 1
                assert thresholds == vision.cfg.YELLOW + vision.cfg.BLUE
                assert kwargs["merge_cb"](yellow, Blob(4))
                assert not kwargs["merge_cb"](yellow, blue)
                return [yellow, blue, mixed]

        frame = Frame()
        reports = [[], []]
        self.assertEqual(vision.find_goals(frame, self.mirror, self.mirror.roi, 1, None, reports),
                         ((86, 100), (86, 100)))
        self.assertEqual(frame.calls, 1)
        self.assertEqual([report[0] for report in reports], [1, 1])

    def test_white_fixture_and_unknown_floor_keep_safe_masks_conservative(self):
        raster = Raster([(37, -9, 5), (80, 0, 0), (10, 0, 0)])
        scanner = vision.Scanner(self.mirror, 320, lambda color: color)
        rear = self.mirror.rays[12][1]
        neighbor = self.mirror.rays[13][1]

        def mark(ray, color):
            for x, y, _ in ray[:2]:
                raster.pixels[y * 320 + x] = color

        mark(rear, 1)  # An isolated image feature, such as a white wire.
        visible, lines = scanner.scan(raster)
        mask, escape, nearest = self.guard.update(visible, lines, 0)
        self.assertEqual(set(lines), {180})
        self.assertEqual(escape, 0)
        self.assertFalse(self.guard.near)
        self.assertTrue(mask & 1)
        self.assertFalse(mask & 256)
        self.assertEqual(nearest, (180, 18))

        mark(neighbor, 1)
        visible, lines = scanner.scan(raster)
        mask, escape, _ = self.guard.update(visible, lines, 1)
        self.assertEqual(escape, 1)
        self.assertTrue(mask & 1)
        self.assertFalse(mask & 256)

        for bearing, ray in self.mirror.rays:
            if bearing not in lines:
                mark(ray, 2)
        visible, lines = scanner.scan(raster)
        mask, escape, _ = self.guard.update(visible, lines, 2)
        self.assertEqual((mask, escape), (0, 0))
        self.assertEqual(self.guard.unknown, 0xffff)
        self.assertEqual(self.guard.escape_away, 1)  # Direction is away, but the floor is unknown.

    def test_packet_crc_and_slow_invalidation(self):
        miss = ((65535, 0, 255),) * 2
        pkt = vision.packet(9, True, True, 256, 256, (0, 21), miss, 101)
        self.assertEqual(len(pkt), 23)
        self.assertEqual(pkt[:12].hex(), "c66c09070000000000001565")
        self.assertEqual(int.from_bytes(pkt[-2:], "big"), vision.crc16(pkt[:-2]))
        self.assertEqual(vision.crc16(b"123456789"), 0x29b1)
