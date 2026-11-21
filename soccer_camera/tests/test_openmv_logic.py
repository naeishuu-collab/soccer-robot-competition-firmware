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
        self.assertEqual(nav.decide_navigation(False, VISIBLE, [], (0, True))[4], nav.MODE_WAIT)

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

    def test_green_to_white_or_black_boundary_and_noise(self):
        samples = [(i, 0, i + 10) for i in range(5)]
        for color in ("WHITE", "BLACK"):
            colors = ["GREEN", "GREEN", color, color, None]
            self.assertEqual(nav.scan_ray(samples, lambda x, _: colors[x]), (True, (color, 12, 2, 0)))
        colors = ["GREEN", "GREEN", "BLACK", None, "BLACK"]
        self.assertEqual(nav.scan_ray(samples, lambda x, _: colors[x]), (True, None))
        colors = ["GREEN", "GREEN", "BLACK", "WHITE", "BLACK"]
        self.assertEqual(nav.scan_ray(samples, lambda x, _: colors[x]), (True, None))
        self.assertEqual(nav.scan_ray(samples, lambda x, _: "BLACK"), (False, None))

    def test_avoid_moves_away_on_all_four_sides_and_inhibits_aim(self):
        for bearing, motion in ((0, (0, -35)), (90, (-35, 0)), (180, (0, 35)), (270, (35, 0))):
            result = nav.decide_navigation(True, VISIBLE, [edge(bearing)], (0, True))
            self.assertEqual(result, (*motion, 0, 0, nav.MODE_AVOID, nav.FLAG_OMNI | nav.FLAG_VIRTUAL_LINE))

    def test_corner_avoidance_and_conflicting_boundaries(self):
        corner = nav.decide_navigation(True, VISIBLE, [edge(0), edge(90)], None)
        self.assertEqual(corner[:2], (-25, -25))
        opposite = nav.decide_navigation(True, VISIBLE, [edge(0), edge(180)], None)
        self.assertEqual(opposite[:3], (0, 0, 0))
        self.assertEqual(opposite[4], nav.MODE_AVOID)

    def test_wall_clearance_includes_outer_area_without_inventing_penalty_flag(self):
        command = nav.decide_navigation(True, VISIBLE, [edge(90, "BLACK", 29)], None)
        self.assertEqual(command[4], nav.MODE_AVOID)
        self.assertFalse(command[5] & 0x02)
        for boundary in (edge(90, "BLACK", 31), edge(90, "WHITE", 19)):
            self.assertEqual(nav.decide_navigation(True, VISIBLE, [boundary], None)[4], nav.MODE_TRACK_IR)

    def test_poor_floor_visibility_waits(self):
        self.assertEqual(nav.decide_navigation(True, set(range(0, 165, 15)), [], (0, True))[4], nav.MODE_WAIT)
        self.assertEqual(nav.decide_navigation(True, VISIBLE - {0}, [], (0, True))[4], nav.MODE_WAIT)

    def test_goal_tracking_wraps_and_drops_aim_immediately(self):
        track = nav.GoalTrack()
        self.assertIsNone(track.update((358, True)))
        self.assertIsNone(track.update((1, True)))
        self.assertEqual(track.update((0, True)), (0, True))
        self.assertEqual(track.update((0, False)), (0, False))
        self.assertIsNone(track.update((45, True)))
        self.assertIsNone(track.update(None))
        self.assertIsNone(track.update((45, True)))

    def test_goal_commands_are_stationary_or_turn_only(self):
        self.assertEqual(nav.decide_navigation(True, VISIBLE, [], (10, False))[:5], (0, 0, -6, 0, nav.MODE_ALIGN))
        self.assertEqual(nav.decide_navigation(True, VISIBLE, [], (0, True))[:5], (0, 0, 0, 1, nav.MODE_AIMED))
        self.assertEqual(nav.decide_navigation(True, VISIBLE, [], None)[:5], (0, 0, 0, 0, nav.MODE_TRACK_IR))

    def test_goal_mask_and_both_edges_of_aim_window(self):
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
            self.assertEqual(nav.goal_geometry(true_goal, mirror), (0, True))
            narrow = Blob(160, 100, 30, [(159, 100), (161, 100), (161, 95), (159, 95)])
            self.assertFalse(nav.goal_geometry(narrow, mirror)[1])

    def test_wire_format_signed_motion_checksum_and_protocol_marker(self):
        packet = nav.make_nav_packet(*nav.decide_navigation(True, VISIBLE, [edge(90)], None))
        self.assertEqual(list(packet), [0x20, 221, 0, 0, 0, 1, 0x21, 221])


if __name__ == "__main__":
    unittest.main()
