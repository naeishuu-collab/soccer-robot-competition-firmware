#!/usr/bin/env python3
"""Host tests for the OpenMV navigation decision logic (no camera required)."""
import importlib.util
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "goal_wall_navigation", ROOT / "openmv/goal_wall_navigation.py")
nav = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(nav)


class OpenMvNavigationTests(unittest.TestCase):
    def test_angle_conventions(self):
        self.assertAlmostEqual(nav.angle_from_normalized(128, 0), 0.0)
        self.assertAlmostEqual(nav.angle_from_normalized(254, 128), 90.0)
        self.assertAlmostEqual(nav.angle_from_normalized(128, 254), 180.0)
        self.assertAlmostEqual(nav.angle_from_normalized(0, 128), 270.0)
        self.assertIsNone(nav.angle_from_normalized(255, 255))

    def test_shortest_angle_error(self):
        self.assertAlmostEqual(nav.angle_error(359.0, 0.0), -1.0)
        self.assertAlmostEqual(nav.angle_error(1.0, 359.0), 2.0)
        self.assertAlmostEqual(nav.angle_error(90.0, 0.0), 90.0)

    def test_front_wall_takes_priority_and_inhibits_kick(self):
        vx, vy, turn, kick, mode, name, flags = nav.decide_navigation(
            128, 40, 255, 255, 128, 30, 6000, (500, 0))
        self.assertEqual(name, "AVOID")
        self.assertEqual(vx, 0)
        self.assertEqual(vy, -50)
        self.assertGreater(turn, 0)  # 壁と逆 (CCW) へ旋回
        self.assertEqual(kick, 0)

    def test_front_goal_shoots(self):
        result = nav.decide_navigation(128, 30, 255, 255, 255, 255, 0, (500, 0))
        self.assertEqual(result[5], "SHOOT")
        self.assertEqual(result[3], 1)
        self.assertEqual(result[2], 0)

    def test_off_axis_goal_approaches_without_kick(self):
        result = nav.decide_navigation(230, 128, 255, 255, 255, 255, 0, (500, 0))
        self.assertEqual(result[5], "APPROACH")
        self.assertEqual(result[3], 0)
        self.assertLess(result[2], 0)  # 右のゴールへ時計回り

    def test_no_goal_searches(self):
        result = nav.decide_navigation(255, 255, 255, 255, 255, 255, 0, (0, 0))
        self.assertEqual(result[5], "SEARCH")
        self.assertEqual(result[3], 0)

    def test_target_goal_selection(self):
        original = nav.TARGET_GOAL
        try:
            nav.TARGET_GOAL = "BLUE"
            result = nav.decide_navigation(255, 255, 128, 30, 255, 255, 0, (0, 500))
            self.assertEqual(result[5], "SHOOT")
        finally:
            nav.TARGET_GOAL = original

    def test_wall_distance_and_virtual_line(self):
        self.assertIsNone(nav.wall_distance_cm(0))
        self.assertAlmostEqual(nav.wall_distance_cm(6000), 30.0)
        self.assertAlmostEqual(nav.wall_distance_cm(3000), 60.0)
        # 壁が近い (blob大) -> 白線接近 (dist_line = 30-12 = 18cm なので余裕を広げる)
        original_margin = nav.VIRTUAL_LINE_MARGIN_CM
        nav.VIRTUAL_LINE_MARGIN_CM = 20.0
        try:
            line, penalty, angle, dist = nav.estimate_virtual_line(128, 30, 6000)
            self.assertTrue(line)
            self.assertTrue(penalty)
            self.assertAlmostEqual(angle, 0.0, delta=1.0)
        finally:
            nav.VIRTUAL_LINE_MARGIN_CM = original_margin
        # 壁が遠い -> ラインなし
        line, penalty, angle, dist = nav.estimate_virtual_line(128, 30, 300)
        self.assertFalse(line)
        self.assertFalse(penalty)

    def test_nav_packet_checksum_and_signed_values(self):
        pkt = nav.make_nav_packet(-50, 55, -12, 1, nav.MODE_SHOOT, 1)
        self.assertEqual(len(pkt), 8)
        self.assertEqual(pkt[0], 0x20)
        self.assertEqual(pkt[1], 206)  # -50 & 0xFF
        self.assertEqual(pkt[2], 55)
        self.assertEqual(pkt[3], 244)  # -12 & 0xFF
        self.assertEqual(pkt[4], 1)
        self.assertEqual(pkt[5], nav.MODE_SHOOT)
        self.assertEqual(pkt[6], 1)
        checksum = 0
        for value in pkt[:7]:
            checksum ^= value
        self.assertEqual(pkt[7], checksum)

    def test_legacy_packet_layout(self):
        goal, wall = nav.make_legacy_packets(10, 20, 30, 40, 50, 60, 0x0123)
        self.assertEqual(list(goal), [0x02, 10, 20, 30, 40, 50, 60])
        self.assertEqual(list(wall), [0x14, 50, 60, 0x01, 0x23, 0xFF, 0xFF])


if __name__ == "__main__":
    unittest.main()
