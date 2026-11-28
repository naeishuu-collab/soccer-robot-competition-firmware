from pathlib import Path
import random
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "openmv"))
import vision


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
        _, escape, _ = self.guard.update(self.rays - {0}, {0: 20}, 1)
        self.assertEqual(escape, 256)
        self.assertEqual(self.guard.update(self.rays - {0}, {}, 200)[1], 256)
        self.assertTrue(self.guard.near)
        self.assertEqual(self.guard.update(self.rays - {0}, {}, 201)[1], 0)
        self.guard.update(self.rays, {}, 220)
        self.guard.update(self.rays, {}, 260)
        self.assertFalse(self.guard.near)
        _, escape, _ = self.guard.update(self.rays - {0, 180}, {0: 20, 180: 21}, 300)
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
        self.assertIsNone(vision.find_goal(Frame(), self.mirror, vision.cfg.BLUE, 8,
                                          lambda _img, _x, _y: 1))
        self.assertEqual(vision.goal_fields((goal, 500), 799, lambda a, b: a-b), (86, 100, 30))
        self.assertEqual(vision.goal_fields((goal, 500), 800, lambda a, b: a-b), (65535, 0, 255))

    def test_packet_crc_and_slow_invalidation(self):
        miss = ((65535, 0, 255),) * 2
        pkt = vision.packet(9, True, True, 256, 256, (0, 21), miss, 101)
        self.assertEqual(len(pkt), 23)
        self.assertEqual(pkt[:12].hex(), "c66c09070000000000001565")
        self.assertEqual(int.from_bytes(pkt[-2:], "big"), vision.crc16(pkt[:-2]))
        self.assertEqual(vision.crc16(b"123456789"), 0x29b1)
