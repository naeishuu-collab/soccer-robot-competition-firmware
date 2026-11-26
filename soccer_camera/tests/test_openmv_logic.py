#!/usr/bin/env python3
"""Host tests for the competition camera logic."""
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "openmv"))
import goal_wall_navigation as nav
import wall_only


def edge(angle, color="WHITE", cm=15):
    return angle, color, cm, 0, 0


VISIBLE = set(range(0, 360, 15))


class OpenMvNavigationTests(unittest.TestCase):
    def test_defaults_wait_for_measurements(self):
        self.assertFalse(nav.Mirror(320, 320).rays)
        self.assertEqual(nav.decide_navigation(False, VISIBLE, [])[3], nav.MODE_WAIT)

    def test_wait_display_identifies_setup_freshness_and_floor_failures(self):
        with patch.multiple(nav.cfg, IMAGE_SIZE=None, MIRROR_CENTER=None, MIRROR_RING=None,
                            GROUND_POINTS=()):
            self.assertEqual(nav.setup_wait_reason(nav.Mirror(320, 240)),
                             "SET IMAGE_SIZE 320x240")
            nav.cfg.IMAGE_SIZE = (320, 240)
            self.assertEqual(nav.setup_wait_reason(nav.Mirror(320, 240)),
                             "CHECK MIRROR_CENTER/RING")
            nav.cfg.MIRROR_CENTER = (160, 120)
            nav.cfg.MIRROR_RING = (20, 100)
            self.assertEqual(nav.setup_wait_reason(nav.Mirror(320, 240)),
                             "CHECK GROUND_POINTS")
            nav.cfg.GROUND_POINTS = ((40, 10), (80, 55))
            mirror = nav.Mirror(320, 240)
            self.assertIsNone(nav.setup_wait_reason(mirror))
        self.assertEqual(nav.wait_detail(None, 120, VISIBLE), "SLOW FRAME 120ms > 100ms")
        self.assertEqual(nav.wait_detail(None, 100, VISIBLE - {0}), "FLOOR 23/24  CHECK PATH")

    def test_calibration_uses_actual_size_center_rotation_and_mirroring(self):
        with patch.multiple(nav.cfg, IMAGE_SIZE=(320, 240), MIRROR_CENTER=(149, 110),
                            MIRROR_RING=(20, 100), FRONT_DEG=90, CLOCKWISE=False,
                            GROUND_POINTS=((70, 10), (50, 30), (32, 52))):
            mirror = nav.Mirror(320, 240)
            self.assertTrue(mirror.ring_ready)
            self.assertEqual(len(mirror.rays), 24)
            self.assertAlmostEqual(mirror.bearing(219, 110), 0)
            self.assertAlmostEqual(mirror.bearing(149, 40), 90)
            self.assertAlmostEqual(mirror.bearing(79, 110), 180)
            self.assertAlmostEqual(mirror.bearing(149, 180), 270)
            self.assertFalse(mirror.pixels_on_ring(149, 110))
            self.assertTrue(mirror.pixels_on_ring(219, 110))
            self.assertFalse(nav.Mirror(320, 320).rays)
            self.assertEqual(mirror.rays[0][1][0], (219, 110, 10))
            self.assertLessEqual(mirror.rays[0][1][-1][2], nav.LOOKAHEAD_CM)

    def test_ground_calibration_rejects_nonmonotonic_points_and_incomplete_range(self):
        with patch.multiple(nav.cfg, IMAGE_SIZE=(320, 320), MIRROR_CENTER=(160, 160),
                            MIRROR_RING=(20, 120), GROUND_POINTS=((70, 10), (80, 30), (32, 52))):
            self.assertFalse(nav.Mirror(320, 320).rays)
            with patch.object(nav.cfg, "GROUND_POINTS", ((70, 10), (50, 30))):
                self.assertFalse(nav.Mirror(320, 320).rays)

    def test_openmv_v5_pixel_and_drawing_coordinates(self):
        class Frame:
            def __init__(self):
                self.calls = []
            def get_pixel(self, position, *, rgbtuple):
                self.calls.append(("pixel", position))
                return (1, 2, 3)
            def draw_cross(self, position, *, color):
                self.calls.append(("cross", position))
            def draw_circle(self, circle, *, color):
                self.calls.append(("circle", circle))
            def draw_line(self, line, *, color):
                self.calls.append(("line", line))

        frame = Frame()
        class Mirror:
            cx, cy, inner, outer, ring_ready = 10, 11, 2, 5, True
            rays = [(0, [(1, 2, 10), (2, 2, 11), (3, 2, 12), (4, 2, 13)])]
        nav.draw_mirror(frame, Mirror())
        self.assertIn(("cross", (10, 11)), frame.calls)
        self.assertIn(("circle", (10, 11, 2)), frame.calls)
        class ImageModule:
            @staticmethod
            def rgb_to_lab(rgb):
                return (30, -20, 0)
        with patch.object(nav, "image", ImageModule):
            visible, _ = nav.scan_field(frame, Mirror())
        self.assertEqual(visible, {0})
        self.assertIn(("pixel", (1, 2)), frame.calls)

    def test_wall_tracking_hysteresis_and_no_stale_detection(self):
        class Blob:
            def __init__(self, x, y, pixels):
                self.cx, self.cy, self.pixels = x, y, pixels
        track = wall_only.WallTrack()
        self.assertIsNone(track.update(Blob(10, 10, 149), 100, 100))
        self.assertEqual(track.update(Blob(20, 20, 150), 100, 100), (51, 51, 150))
        self.assertEqual(track.update(Blob(30, 20, 110), 100, 100), (61, 51, 110))
        self.assertIsNone(track.update(Blob(80, 80, 110), 100, 100))
        self.assertEqual(track.update(Blob(80, 80, 200), 100, 100), (204, 204, 200))
        self.assertIsNone(track.update(None, 100, 100))
        self.assertIsNone(track.update(Blob(80, 80, 110), 100, 100))

    def test_rectangular_wall_selection_and_cropped_candidate(self):
        class Blob:
            def __init__(self, corners, pixels):
                self.corners, self.pixels = corners, pixels
        class Frame:
            def __init__(self, blobs): self.blobs = blobs
            def find_blobs(self, *args, **kwargs): return self.blobs
        rectangular = Blob([(10, 10), (30, 10), (30, 30), (10, 30)], 300)
        cropped = Blob([(0, 10), (20, 10), (20, 30), (0, 30)], 250)
        sparse = Blob([(10, 10), (30, 10), (30, 30), (10, 30)], 150)
        degenerate = Blob([(0, 0)] * 4, 200)
        self.assertIs(wall_only.find_rectangular_wall(Frame([sparse, rectangular]))[0], rectangular)
        self.assertIs(wall_only.find_rectangular_wall(Frame([cropped]))[0], cropped)
        self.assertIsNone(wall_only.find_rectangular_wall(Frame([sparse, degenerate]))[0])

    def test_green_to_white_or_black_boundary_and_noise(self):
        samples = [(i, 0, i + 10) for i in range(5)]
        for color in ("WHITE", "BLACK"):
            colors = ["GREEN", "GREEN", color, color, None]
            self.assertEqual(nav.scan_ray(samples, lambda x, _: colors[x]), (False, (color, 12, 2, 0)))
        colors = ["GREEN", "GREEN", "BLACK", None, "BLACK"]
        self.assertEqual(nav.scan_ray(samples, lambda x, _: colors[x]), (False, None))
        colors = ["GREEN", "GREEN", "BLACK", "WHITE", "BLACK"]
        self.assertEqual(nav.scan_ray(samples, lambda x, _: colors[x]), (False, None))
        self.assertEqual(nav.scan_ray(samples, lambda x, _: "BLACK"), (False, None))

    def test_wall_scan_requires_neighbor_at_similar_distance(self):
        class Mirror:
            rays = [
                (bearing, [(x, row, base + x) for x in range(4)])
                for row, (bearing, base) in enumerate(
                    ((0, 20), (15, 32), (90, 20), (105, 20), (180, 15), (345, 21)))
            ]

        class Frame:
            def get_pixel(self, position, *, rgbtuple):
                x, row = position
                if x < 2:
                    return (18, -15, 6)
                return (90, 0, 0) if row == 4 else (8, 0, 0)

        class ImageModule:
            rgb_to_lab = staticmethod(lambda values: values)

        with patch.object(nav, "image", ImageModule):
            visible, boundaries = nav.scan_field(Frame(), Mirror())
        self.assertEqual(visible, {15})
        self.assertEqual([(bearing, color) for bearing, color, *_ in boundaries],
                         [(0, "BLACK"), (90, "BLACK"), (105, "BLACK"),
                          (180, "WHITE"), (345, "BLACK")])
        self.assertEqual(nav.decide_navigation(True, visible, boundaries)[3], nav.MODE_WAIT)

    def test_avoid_moves_away_only_into_visible_floor(self):
        for bearing, motion in ((0, (0, -35)), (90, (-35, 0)), (180, (0, 35)), (270, (35, 0))):
            result = nav.decide_navigation(True, VISIBLE, [edge(bearing)])
            self.assertEqual(result[:2], motion)
            self.assertEqual(result[3], nav.MODE_AVOID)
        alternative = nav.decide_navigation(True, VISIBLE - {180}, [edge(0)])
        self.assertEqual(alternative[3], nav.MODE_AVOID)
        self.assertEqual(abs(alternative[0]), 25)
        self.assertEqual(alternative[1], -25)
        blocked_retreat = nav.decide_navigation(True, VISIBLE - {135, 180, 225}, [edge(0)])
        self.assertEqual(blocked_retreat, (0, 0, 0, nav.MODE_WAIT))

    def test_corner_avoidance_and_conflicting_boundaries(self):
        corner = nav.decide_navigation(True, VISIBLE, [edge(0), edge(90)])
        self.assertEqual(corner[:2], (-25, -25))
        opposite = nav.decide_navigation(True, VISIBLE, [edge(0), edge(180)])
        self.assertEqual(opposite, (0, 0, 0, nav.MODE_WAIT))

    def test_retreat_mask_excludes_headings_toward_any_close_edge(self):
        boundaries = [edge(0, cm=18), edge(135, cm=25)]
        vx, vy, mask, mode = nav.decide_navigation(True, VISIBLE - {0, 135}, boundaries)
        self.assertEqual((vx, vy, mode), (-25, -25, nav.MODE_AVOID))
        self.assertFalse(mask & (1 << 4))  # 180 degrees approaches the 135-degree edge.
        self.assertTrue(mask & (1 << 5))

    def test_clearance_for_wall_and_white_line(self):
        for boundary in (edge(90, "BLACK", 29), edge(90, "WHITE", 25)):
            self.assertEqual(nav.decide_navigation(True, VISIBLE, [boundary])[3], nav.MODE_AVOID)
        for boundary in (edge(90, "BLACK", 31), edge(90, "WHITE", 26)):
            self.assertEqual(nav.decide_navigation(True, VISIBLE, [boundary])[3], nav.MODE_TRACK_IR)

    def test_directional_floor_mask(self):
        self.assertEqual(nav.decide_navigation(True, set(range(0, 165, 15)), [])[3], nav.MODE_WAIT)
        command = nav.decide_navigation(True, VISIBLE - {75, 90, 105}, [])
        self.assertEqual(command[3], nav.MODE_TRACK_IR)
        self.assertTrue(command[2] & 1)
        self.assertFalse(command[2] & (1 << 2))

    def test_repository_calibration_can_detect_and_avoid_a_near_white_line(self):
        mirror = nav.Mirror(320, 240)
        self.assertIsNone(nav.setup_wait_reason(mirror))
        for bearing, samples in mirror.rays:
            colors = {(x, y): "GREEN" if i < 2 else "WHITE"
                      for i, (x, y, _) in enumerate(samples)}
            _, boundary = nav.scan_ray(samples, lambda x, y: colors[x, y])
            self.assertEqual(nav.decide_navigation(True, VISIBLE, [(bearing,) + boundary])[3],
                             nav.MODE_AVOID)
        with patch.object(nav, "LINE_CLEARANCE_CM", 18):
            self.assertEqual(nav.setup_wait_reason(mirror), "CHECK GROUND_POINTS")

    def test_camera_reports_white_line_sends_changed_heading_and_stops_on_stale_frame(self):
        class Frame:
            labels = []

            def __init__(self, line):
                self.line = line

            def width(self):
                return 320

            def height(self):
                return 240

            def find_blobs(self, thresholds, **kwargs):
                return ()

            def draw_string(self, position, text, *, color):
                if position == (2, 26):
                    self.labels.append(text)

            def draw_cross(self, *args, **kwargs):
                pass

            draw_circle = draw_line = draw_cross

        class Sensor:
            frames = iter(Frame(line) for line in (None, 0, 90, 90, None, 0, 0, 0, 0))

            @staticmethod
            def snapshot():
                return next(Sensor.frames)

        class Time:
            timestamps = iter((0, 0, 50, 55, 70, 100, 140, 150, 200, 200,
                               250, 250, 300, 300, 350, 350, 470))

            @staticmethod
            def ticks_ms():
                return next(Time.timestamps)

            @staticmethod
            def ticks_diff(now, previous):
                return now - previous

        class Board:
            packets = []

            @staticmethod
            def UART(port, baud):
                return Board()

            def write(self, packet):
                Board.packets.append(packet)

        with patch.multiple(nav.cfg, IMAGE_SIZE=(320, 240), MIRROR_CENTER=(160, 120),
                            MIRROR_RING=(20, 100), GROUND_POINTS=((40, 10), (80, 55)),
                            TARGET_GOAL="YELLOW"):
            with patch.multiple(nav, sensor=Sensor, time=Time, pyb=Board,
                                init_camera=lambda: None,
                                scan_field=lambda img, _: (
                                    VISIBLE, [edge(img.line)] if img.line is not None else [edge(0, "BLACK", 35)])):
                with self.assertRaises(StopIteration):
                    nav.main_loop(display=True)
        self.assertEqual([packet[5] for packet in Board.packets],
                         [nav.MODE_AVOID, nav.MODE_AVOID, nav.MODE_AVOID,
                          nav.MODE_TRACK_IR, nav.MODE_AVOID, nav.MODE_AVOID,
                          nav.MODE_AVOID, nav.MODE_WAIT])
        self.assertEqual(Board.packets[0][1:3], bytes((0, 221)))
        self.assertEqual(Board.packets[1][1:3], bytes((221, 0)))
        self.assertTrue(all(packet[1:4] == b"\x00\x00\x00"
                            for packet in Board.packets if packet[5] == nav.MODE_WAIT))
        self.assertEqual(Frame.labels.count("WHITE LINE: DETECTED (1, 15 cm)"), 7)
        self.assertEqual(Frame.labels[3], "WHITE LINE: NOT DETECTED")

    def test_diagnostic_goal_blob_filters_by_mirror_and_size(self):
        class Blob:
            def __init__(self, x, y, pixels, corners):
                self.cx, self.cy, self.pixels, self.corners = x, y, pixels, corners

        true_goal = Blob(160, 100, 80, [(152, 100), (168, 100), (168, 95), (152, 95)])
        outside = Blob(160, 8, 200, [(152, 8), (168, 8), (168, 9), (152, 9)])
        small = Blob(160, 102, 3, [(159, 100), (161, 100), (161, 103), (159, 103)])

        class Frame:
            def find_blobs(self, *args, **kwargs):
                return outside, small, true_goal

        with patch.multiple(nav.cfg, IMAGE_SIZE=(320, 320), MIRROR_CENTER=(160, 160),
                            MIRROR_RING=(20, 100), FRONT_DEG=0, CLOCKWISE=True):
            mirror = nav.Mirror(320, 320)
            self.assertEqual(nav.find_color_blob(Frame(), mirror, nav.cfg.YELLOW, 30),
                             (true_goal, [3, 1, 1]))

    def test_blue_threshold_covers_supplied_lab_sample(self):
        for l in (21, 22):
            for a in (8, 9):
                for b in (-34, -33):
                    self.assertTrue(any(nav.in_lab((l, a, b), threshold)
                                        for threshold in nav.cfg.BLUE))
        self.assertFalse(any(nav.in_lab((21, 8, 0), threshold) for threshold in nav.cfg.BLUE))

    def test_wire_format_signed_motion_checksum_and_protocol_marker(self):
        vx, vy, mask, mode = nav.decide_navigation(True, VISIBLE, [edge(90)])
        packet = nav.make_nav_packet(vx, vy, mask, mode, 42)
        self.assertEqual(list(packet), [0x20, 221, 0, 224, 42, 1, 0x41, 119])


if __name__ == "__main__":
    unittest.main()
