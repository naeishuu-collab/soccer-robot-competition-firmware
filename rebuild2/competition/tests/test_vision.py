from pathlib import Path
import random
import subprocess
import time
import sys
import unittest
from unittest.mock import patch

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
            observed, white = scanner.scan(raster)
            expected = observed.copy(), white.copy(), scanner.runs.copy()
            scanner.fast = True
            observed, white = scanner.scan(raster)
            self.assertEqual((observed, white, scanner.runs), expected)

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
            confirmed = {b for b, cm in lines.items() if any(
                abs(lines.get((b + shift) % 360, -100) - cm) <= 8 for shift in (-15, 15))}
            expected = sum(1 << i for i, rays in enumerate(vision.SECTOR_WINDOWS)
                           if all(ray in visible for ray in rays) and all(
                               vision.difference(i * 22.5, b) >= (60 if b in confirmed else 45)
                               for b, cm in lines.items() if cm <= 35))
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
        self.assertTrue(any("UNKNOWN LINE" in args[-1] for name, args, _ in calls if name == "draw_string"))
        self.assertFalse(any(name == "draw_circle" for name, _, _ in calls))
        self.assertTrue(any(name == "draw_arrow" for name, _, _ in calls))
        calls.clear()
        draw("FULL", 200)
        self.assertEqual(sum(name == "draw_circle" for name, _, _ in calls), 18)
        self.assertEqual(sum(name == "draw_rectangle" for name, _, _ in calls), 1)
        self.assertTrue(any("BLUE cached" in args[-1] for name, args, _ in calls if name == "draw_string"))
        calls.clear()
        draw("FULL", 220)
        self.assertEqual([name for name, _, _ in calls], ["draw_rectangle", "draw_cross", "draw_string", "draw_arrow"])

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
        mask, escape, nearest = self.guard.update(visible, lines, 0, scanner.runs, self.mirror.rays)
        self.assertEqual(set(lines), {180})
        self.assertEqual(escape, 0)
        self.assertFalse(self.guard.near)
        self.assertTrue(mask & 1)
        self.assertFalse(mask & 256)
        self.assertEqual(nearest, (180, 18))

        mark(neighbor, 1)
        visible, lines = scanner.scan(raster)
        mask, escape, _ = self.guard.update(visible, lines, 1, scanner.runs, self.mirror.rays)
        self.assertEqual(escape, 1)
        self.assertTrue(mask & 1)
        self.assertFalse(mask & 256)

        for bearing, ray in self.mirror.rays:
            if bearing not in lines:
                mark(ray, 2)
        visible, lines = scanner.scan(raster)
        mask, escape, _ = self.guard.update(visible, lines, 2, scanner.runs, self.mirror.rays)
        self.assertEqual((mask, escape), (0, 0))
        self.assertEqual(self.guard.unknown, 0xffff)
        self.assertEqual(self.guard.escape_away, 1)  # Direction is away, but the floor is unknown.

    def test_assumed_floor_reproduces_and_unblocks_recorded_stop(self):
        # A valid, non-green field image with the reported 165 degree candidate.
        neutral, white, dark = (55, 0, 0), (93, -1, 1), (8, 0, 0)
        raster = Raster([neutral, white, dark])
        scanner = vision.Scanner(self.mirror, 320, lambda lab: lab)
        for bearing in (150, 165):
            ray = self.mirror.rays[bearing // 15][1]
            for x, y, _ in ray[:2]:
                raster.pixels[y * 320 + x] = 1
        visible, lines = scanner.scan(raster)
        self.assertEqual(lines[165], 18)
        self.assertFalse(vision.classify(neutral))  # Previously rejected as non-green on all rays.
        old = vision.LineGuard(lambda a, b: a-b)
        self.assertEqual(old.update(set(), lines, 0)[:2], (0, 0))
        mask, escape, _ = self.guard.update(visible, lines, 0, scanner.runs, self.mirror.rays)
        self.assertEqual(escape, 1)
        self.assertTrue(mask & 1)
        self.assertFalse(mask & 256)
        white_pixels = [p for bearing in (150, 165) for p in self.mirror.rays[bearing // 15][1][:2]]
        xs, ys = zip(*[(x, y) for x, y, _ in white_pixels])
        self.assertEqual(self.guard.regions, [((min(xs), min(ys), max(xs)-min(xs)+1, max(ys)-min(ys)+1), True)])

    def test_white_coverage_wrap_multiple_lines_and_glare(self):
        samples = self.mirror.rays
        lines = {345: 20, 0: 21, 165: 21, 180: 20}
        mask, escape, _ = self.guard.update(self.rays, lines, 10, {b:(1,3) for b in lines}, samples)
        self.assertEqual(escape, 0)
        self.assertFalse(mask & (1 | 256))
        self.assertEqual(len(self.guard.regions), 2)

        guard = vision.LineGuard(lambda a,b: a-b)
        line = {75: 18, 90: 18}
        mask, escape, _ = guard.update(self.rays, line, 20, {b:(0,10) for b in line}, samples)
        self.assertEqual(escape, 0)  # A broad reflection is reserved as ambiguous.
        self.assertFalse(guard.near)
        self.assertFalse(mask & (1 << 4))
        self.assertEqual(len(guard.regions), 1)
        self.assertFalse(guard.regions[0][1])

    def test_front_line_and_side_reflections_leave_observed_reverse(self):
        # The confirmed front line closes the front. Two short side reflections
        # previously added 105-degree cones and closed the remaining headings.
        lines = {0: 18, 15: 19, 105: 18, 255: 18}
        wide = {0: (0, 1), 15: (0, 1), 105: (0, 0), 255: (0, 0)}
        safe, escape, _ = self.guard.update(self.rays, lines, 0, wide, self.mirror.rays)
        old_extra = vision.CLOSE_BLOCKS[105 // 15] | vision.CLOSE_BLOCKS[255 // 15]
        self.assertEqual(safe & ~old_extra, 0)  # Replay the prior ambiguous-candidate veto.
        self.assertFalse(safe & (1 | (1 << 5) | (1 << 11)))
        self.assertEqual(escape, 256)
        self.assertTrue(safe & 256)
        safe, escape, _ = self.guard.update(self.rays - {180}, lines, 1, wide, self.mirror.rays)
        self.assertFalse(safe & 256)
        self.assertEqual(escape, 0)

    def test_logged_rear_line_with_two_offset_suspects(self):
        # Production's previous 60-degree rule closed front for either off-lane candidate.
        lines = {180: 19, 195: 20, 45: 18, 315: 18}
        runs = {180: (1, 3), 195: (1, 3), 45: (0, 0), 315: (0, 0)}
        visible = {330, 345, 0, 15, 30}
        safe, escape, _ = self.guard.update(visible, lines, 0, runs, self.mirror.rays)
        self.assertEqual((safe, escape), (1, 1))
        self.assertEqual(self.guard.confirmed, {180, 195})
        self.assertTrue(vision.LINE_BLOCKS[45 // 15] & 1)
        self.assertFalse(self.guard.ambiguous_blocked & 1)
        self.assertTrue(self.guard.unknown & 256)
        self.assertEqual(self.guard.update(visible - {0}, lines, 30, runs, self.mirror.rays)[:2], (0, 0))
        self.assertEqual(self.guard.update(visible, {}, 70)[:2], (1, 1))
        # Two clear observations at the hazards allow chase again.
        self.guard.update(self.rays, {}, 130)
        self.guard.update(self.rays, {}, 165)
        self.assertFalse(self.guard.escape_requested)
        self.assertFalse(self.guard.held_blocked)

    def test_direction_policy_comparison(self):
        previous = subprocess.check_output(
            ["git", "show", "3636020:competition/openmv/vision.py"],
            cwd=Path(__file__).resolve().parents[2])
        prior = {}
        exec(compile(previous, "previous-vision.py", "exec"), prior)
        front = {330, 345, 0, 15, 30}
        rear = {150, 165, 180, 195, 210}
        rear_line = {180: 19, 195: 20}
        front_line = {0: 19, 15: 20}
        # Scenes describe available rays, confirmed geometry, and isolated suspect pixels.
        scenes = (
            (self.rays, {}, {}, 1, 1),
            (front, {}, {45: 19}, 1, 1),
            (front, rear_line, {45: 18, 315: 18}, 1, 1),
            (rear, front_line, {135: 18, 225: 18}, 256, 256),
            (front - {0}, rear_line, {}, 1, 0),
            (self.rays, front_line | rear_line, {}, 1, 0),
            (front, front_line, {}, 1, 0),
            (front, {}, {0: 18}, 1, 0),
        )
        rows = [(visible, known | suspect, {b: (1, 3) for b in known} | {b: (0, 0) for b in suspect},
                 request, allowed, bool(known))
                for visible, known, suspect, request, allowed in scenes]
        results = []
        for constructor in (prior["LineGuard"], vision.LineGuard):
            stops = moves = hazardous = escapes = resumes = 0
            elapsed = 0
            for iteration in range(200):
                for visible, lines, runs, request, allowed, near in rows:
                    guard = constructor(lambda a, b: a-b)
                    started = time.perf_counter_ns()
                    safe, retreat, _ = guard.update(visible, lines, 0, runs, self.mirror.rays)
                    elapsed += time.perf_counter_ns() - started
                    if iteration == 0:
                        moved = retreat if guard.escape_requested else safe & request
                        stops += bool(allowed and not moved)
                        moves += bool(moved)
                        hazardous += bool(moved & ~allowed)
                        escapes += bool(near and allowed and retreat == allowed)
                        if near and allowed:
                            guard.update(self.rays, {}, 60)
                            safe, _, _nearest = guard.update(self.rays, {}, 110)
                            resumes += bool(safe & request and not guard.escape_requested)
            results.append((stops, moves, hazardous, escapes, resumes, round(elapsed / (200 * len(rows)) / 1000, 1)))
        self.assertEqual([row[:5] for row in results], [(3, 1, 0, 0, 2), (0, 4, 0, 2, 2)])
        print("Camera policy before/after (needless stops, moves, unsafe, escapes, resumes, host us):", results)

    def test_old_side_hold_does_not_close_tangent_straight_lane(self):
        self.guard.update(self.rays, {90: 19, 105: 20}, 0)
        safe, _, _ = self.guard.update(self.rays - {90, 105}, {}, 201)
        self.assertFalse(self.guard.escape_requested)
        self.assertTrue(safe & 1)  # Neutral to old 90, moving away from old 105.
        self.assertFalse(safe & 256)  # Forward projection onto old 105 bearing.

    def test_censored_distance_uses_wire_unknown_without_clearing_heading(self):
        lines = {0: 18}
        safe, escape, nearest = self.guard.update(self.rays, lines, 0, {0: (0, 2)}, self.mirror.rays, {0})
        self.assertFalse(safe & 1)
        self.assertEqual(escape, 0)
        self.assertEqual(self.guard.line_angle, 0)
        self.assertIsNotNone(self.guard.line_xy)
        self.assertEqual(nearest, (0, None))
        miss = ((65535, 0, 255),) * 2
        packet = vision.packet(0, True, False, safe, escape, nearest, miss, 41)
        self.assertEqual(bytes(packet[8:11]), b"\xff\xff\xff")
        packet = vision.packet(0, True, False, safe, escape, nearest, miss, 41, blocked=self.guard.blocked)
        self.assertEqual(bytes(packet[8:11]), b"\x00\x00\xff")
        self.assertEqual(packet[20], 3)

    def test_distance_calibration_reports_target_and_interval_without_clamping(self):
        self.assertEqual(self.mirror.distance_at_radius(50), (18, (50, 18, 74, 50)))
        self.assertEqual(self.mirror.distance_at_radius(49.9), (None, None))
        self.assertEqual(self.mirror.distance_at_radius(74.1), (None, None))
        self.assertEqual(self.mirror.distance_at_radius(70.25)[0], 45)
        x, y = round(self.mirror.cx) + 49, round(self.mirror.cy)
        class Target:
            rect, cx, cy, pixels = (x-2, y-1, 4, 3), x, y, 8
        class Frame:
            def find_blobs(self, limits, **kwargs):
                self.limits = limits
                return [Target()]
        frame = Frame()
        self.assertEqual(vision.calibration_targets(frame, self.mirror),
                         [(8, Target.rect, (x, y), 49, None, None)])
        self.assertEqual(frame.limits, [vision.cfg.WHITE])

    def test_dynamic_line_limits_immediately_and_holds_through_blur(self):
        guard = vision.LineGuard(lambda a,b: a-b)
        for now, distance in ((0, 38), (32, 33), (64, 28), (96, 23)):
            mask, escape, _ = guard.update(self.rays, {345:distance+1, 0:distance}, now)
            if distance <= vision.BLOCK_CM:
                self.assertFalse(mask & 1)
        self.assertEqual(escape, 256)
        self.assertGreater(guard.closing_cm_s, 100)
        self.assertIsNotNone(guard.ttc_ms)
        mask, escape, _ = guard.update(self.rays, {}, 112)
        self.assertFalse(mask & 1)
        self.assertEqual(escape, 256)  # One missing / blurred frame cannot clear the last line.
        guard.update(self.rays, {90: 19, 105: 20}, 144)
        self.assertFalse(guard.coverage & ~guard.blocked & (1 << 4))
        self.assertIsNone(guard.closing_cm_s)  # A different region cannot supply speed.

    def test_new_line_does_not_clear_another_recent_hazard(self):
        guard = vision.LineGuard(lambda a,b: a-b)
        _, escape, _ = guard.update(self.rays, {345:20, 0:20}, 0)
        self.assertEqual(escape, 256)
        mask, escape, _ = guard.update(self.rays, {165:21, 180:20}, 30)
        self.assertFalse(mask & (1 | 256))
        self.assertEqual(escape, 0)
        _, escape, _ = guard.update(self.rays, {165:21, 180:20}, 110)
        self.assertEqual(escape, 1)

    def test_blind_hazard_holds_until_sustained_observed_clearance(self):
        guard = vision.LineGuard(lambda a, b: a-b)
        front = {345: 20, 0: 20}
        self.assertEqual(guard.update(self.rays, front, 0)[1], 256)
        hidden = self.rays - {0, 345}
        for now in (30, 80, 150, 210, 500, 1000):
            safe, _, _ = guard.update(hidden, {}, now)
            self.assertFalse(safe & 1)
            self.assertTrue(guard.near_history)
            pkt = vision.packet(1, True, guard.escape_requested, safe, 0, None,
                                ((65535, 0, 255), (65535, 0, 255)), 40, blocked=guard.blocked)
            self.assertEqual(pkt[20], 3)
            self.assertTrue(pkt[3] & 16)
        guard.update(self.rays, {}, 1010)
        safe, escape, _ = guard.update(self.rays, {}, 1040)
        self.assertFalse(guard.near_history)
        self.assertEqual(guard.blocked, 0)
        self.assertEqual(escape, 0)
        self.assertTrue(safe & 1)

    def test_reappearing_line_restarts_clearance_delay(self):
        guard = vision.LineGuard(lambda a, b: a-b)
        line = {345: 20, 0: 20}
        guard.update(self.rays, line, 0)
        guard.update(self.rays, {}, 30)
        guard.update(self.rays, line, 60)
        for now in (90, 120, 150):
            safe, escape, _ = guard.update(self.rays, {}, now)
            self.assertEqual(escape, 256)
            self.assertFalse(safe & 1)
        safe, escape, _ = guard.update(self.rays, {}, 160)
        self.assertEqual(escape, 0)
        self.assertFalse(guard.near_history)
        self.assertTrue(safe & 1)

    def test_side_line_requires_observed_direction_away_from_every_ray(self):
        # A 105-degree nearest line does not establish the rest of its extent.
        # Both synthetic scenes have the same nearest ray but different safe exits.
        forward = vision.LineGuard(lambda a, b: a-b)
        safe, escape, nearest = forward.update(self.rays, {105: 20, 120: 21}, 0)
        self.assertEqual(nearest, (105, 20))
        self.assertEqual(escape, 1)
        self.assertTrue(safe & 1)
        blocked = vision.LineGuard(lambda a, b: a-b)
        safe, escape, nearest = blocked.update(self.rays, {90: 21, 105: 20}, 0)
        self.assertEqual(nearest, (105, 20))
        self.assertEqual(escape, 0)
        self.assertFalse(safe & (1 | 256))
        unobserved = vision.LineGuard(lambda a, b: a-b)
        safe, escape, _ = unobserved.update(self.rays - {0}, {105: 20, 120: 21}, 0)
        self.assertFalse(safe & 1)
        self.assertEqual(escape, 0)

    def test_white_candidate_visibility_and_measured_clearance(self):
        self.assertTrue(vision.line_code((85, -3, 3), True) & 1)
        self.assertTrue(vision.line_code((80, 0, 0)) & 1)
        self.assertFalse(vision.line_code((80, 0, 0), True) & 1)
        for lab in ((99, -3, 7), (100, -2, 8), (89, -5, 5), (98, 0, 4), (97, 0, 9)):
            self.assertTrue(vision.line_code(lab) & 1)
            self.assertTrue(vision.line_code(lab, True) & 1)
        self.assertFalse(vision.line_code((100, 30, 0)) & 4)
        self.assertEqual(vision.safety_distances(None), (25, 35))
        self.assertEqual(vision.safety_distances((9, 100, 100, 10, 3)), (32,35+2))
        with self.assertRaises(ValueError):
            vision.safety_distances((9,100,100,40,3))
        self.assertEqual(len(vision.Mirror(160, 120, scale=0.5).rays), 24)

    def test_synthetic_motion_permits_observed_routes_only(self):
        # The old classifier returned no green coverage on these non-green frames.
        scenes = (
            ({}, True, 1, 0, True),
            ({150:18,165:18}, True, 1, 256, True),
            ({345:20,0:20}, True, 256, 1, True),
            ({90:45}, True, 1, 0, True),
            ({165:18}, True, 1, 256, True),
            ({90:19,105:20}, True, 1, 1 << 4, False),
            ({0:20,15:21,165:20,180:21}, True, 1, 1 | 256, False),
            ({}, False, 1, 0xffff, False),
        )
        stops = [0, 0]
        dangerous = [0, 0]
        for lines, observed, requested, hazard, expected in scenes:
            for i, visible in enumerate((set(), self.rays if observed else set())):
                guard = vision.LineGuard(lambda a,b: a-b)
                safe, escape, _ = guard.update(visible, lines, 0)
                active = escape if guard.near else safe & requested
                stops[i] += int(expected and not active)
                dangerous[i] += int(bool((safe | escape) & hazard))
                if i == 1:
                    self.assertEqual(bool(active), expected)
        self.assertEqual(stops, [5, 0])
        self.assertEqual(dangerous, [0, 0])
        print("Synthetic field scenes: unnecessary stops 5/5 -> 0/5; hazardous permissions 0/8 -> 0/8")

    def test_frame_edge_reflection_never_promotes_adjacent_white(self):
        raster = Raster([(55, 0, 0), (93, -1, 1)])
        for bearing in (0, 15, 165, 180):
            for x, y, _ in self.mirror.rays[bearing // 15][1][:2]:
                raster.pixels[y * 320 + x] = 1
        for bearing in (165, 180):
            raster.pixels[self.mirror.inner_context[bearing // 15]] = 1

        scanner = vision.Scanner(self.mirror, 320, lambda sample: sample)
        visible, lines = scanner.scan(raster)
        self.assertEqual(scanner.censored, {165, 180})
        old_mask, old_escape, _ = self.guard.update(visible, lines, 0, scanner.runs, self.mirror.rays)
        self.assertEqual(self.guard.confirmed, {0, 15, 165, 180})
        revised = vision.LineGuard(lambda a, b: a-b)
        mask, escape, _ = revised.update(visible, lines, 0, scanner.runs, self.mirror.rays, scanner.censored)
        self.assertEqual(revised.confirmed, {0, 15})
        self.assertEqual([confirmed for _, confirmed in revised.regions], [True, False])
        self.assertEqual((old_mask, old_escape, escape), (0, 0, 0))
        self.assertFalse(mask & (1 | 256))
        self.assertTrue(revised.ambiguous_blocked & 256)

    def test_polygon_mask_and_exposure_reasons(self):
        x0, y0, _ = self.mirror.rays[0][1][0]
        polygon = ((x0-1,y0-1), (x0+1,y0-1), (x0+1,y0+1), (x0-1,y0+1))
        with patch.object(vision.cfg, "LINE_SELF_POLYGONS", (polygon,)):
            mirror = vision.Mirror(320, 240)
        self.assertEqual(mirror.self_outlines, [polygon])
        calls = []
        class TestSurface:
            def __getattr__(self, name):
                return lambda coords, *args, **style: calls.append((name, coords, style["color"]))
        miss = ((65535, 0, 255), (65535, 0, 255))
        preview.Overlay(mirror).draw(TestSurface(), "FULL", 0, lambda a,b: a-b, 25, 40, self.guard,
                                     {}, 0, 0, None, (None, None), miss)
        self.assertEqual(sum(name == "draw_line" and color == preview.PURPLE for name, _, color in calls), 4)
        raster = Raster([(55, 0, 0), (6, 0, 0), (100, 0, 0)])
        for bearing, color in ((0, 2), (90, 1), (180, 2)):
            x, y, _ = mirror.rays[bearing//15][1][0]
            raster.pixels[y*320+x] = color
        scanner = vision.Scanner(mirror, 320, lambda color: color)
        visible, lines = scanner.scan(raster)
        self.assertEqual([scanner.reasons[b] for b in (0,90,180)], [4,1,2])
        self.assertNotIn(0, lines)
        self.assertNotIn(0, visible)
        dark, clipped, masked = vision.observation_reasons(scanner.reasons)
        self.assertTrue(dark & (1 << 4))
        self.assertTrue(clipped & 256)
        self.assertTrue(masked & 1)
        mask, _, _ = self.guard.update(visible, lines, 1, scanner.runs, mirror.rays, scanner.censored)
        self.assertFalse(mask & (1 | 256))

    def test_stale_escape_request_expires_with_held_sector_still_closed(self):
        _, escape, _ = self.guard.update(self.rays, {165:18,180:19}, 0)
        self.assertEqual(escape, 1)
        held = self.rays - {165,180}
        safe, escape, _ = self.guard.update(held, {}, 205)
        self.assertFalse(self.guard.escape_requested)
        self.assertEqual(escape, 0)
        self.assertTrue(safe & 1)
        self.assertFalse(safe & 256)

    def test_packet_crc_and_slow_invalidation(self):
        miss = ((65535, 0, 255),) * 2
        pkt = vision.packet(9, True, True, 256, 256, (0, 21), miss, 101)
        self.assertEqual(len(pkt), 23)
        self.assertEqual(pkt[:12].hex(), "c66c090f0000000000001565")
        self.assertEqual(int.from_bytes(pkt[-2:], "big"), vision.crc16(pkt[:-2]))
        self.assertEqual(vision.crc16(b"123456789"), 0x29b1)
