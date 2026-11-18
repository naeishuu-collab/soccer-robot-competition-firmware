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

    def test_front_goal_shoots_only_with_ball_in_front(self):
        ball = (128, 30, 100)
        result = nav.decide_navigation(128, 30, 255, 255, 255, 255, 0, (500, 0), ball)
        self.assertEqual(result[5], "SHOOT")
        self.assertEqual(result[3], 1)
        self.assertEqual(result[2], 0)
        self.assertTrue(result[6] & nav.FLAG_BALL)
        self.assertTrue(result[6] & nav.FLAG_BALL_ALIGNED)
        # ボールが見えていなければ、ゴールが正面でも蹴らない
        no_ball = nav.decide_navigation(128, 30, 255, 255, 255, 255, 0, (500, 0))
        self.assertEqual(no_ball[5], "APPROACH")
        self.assertEqual(no_ball[3], 0)

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
            result = nav.decide_navigation(255, 255, 128, 30, 255, 255, 0, (0, 500),
                                           (128, 30, 100))
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

    def test_penalty_area_moves_away_from_wall(self):
        # 壁は中距離 (60cm) だがペナルティエリア判定が立つ場合は前進して壁から離れる
        original = nav.PENALTY_DEPTH_CM
        nav.PENALTY_DEPTH_CM = 100.0
        try:
            result = nav.decide_navigation(128, 30, 255, 255, 128, 30, 3000, (500, 0))
            self.assertEqual(result[5], "AVOID")
            self.assertGreater(result[1], 0)  # 前進して中央へ戻る
            self.assertEqual(result[3], 0)    # ペナルティ内ではキックしない
            self.assertTrue(result[6] & 2)    # flags bit1 = PA
        finally:
            nav.PENALTY_DEPTH_CM = original

    def test_rule_compliance_leds_off(self):
        src = (Path(__file__).resolve().parents[1] / "openmv/goal_wall_navigation.py").read_text(encoding="utf-8")
        self.assertIn("sensor.set_ir_led(False)", src)
        self.assertIn("pyb.LED(led_id).off()", src)
        self.assertIn("wall_roi", src)

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

    def test_own_goal_only_defends_without_kick(self):
        # TARGET_GOAL=YELLOW のとき青ゴールだけ見えても自ゴールへは蹴らない
        result = nav.decide_navigation(255, 255, 128, 30, 255, 255, 0, (0, 500),
                                       (128, 30, 100))
        self.assertEqual(result[5], "DEFEND")
        self.assertEqual(result[3], 0)
        self.assertEqual(result[1], -20)

    def test_obstacle_in_front_avoids_without_kick(self):
        result = nav.decide_navigation(128, 30, 255, 255, 255, 255, 0, (500, 0),
                                       (128, 30, 100), (128, 40, 200))
        self.assertEqual(result[5], "AVOID")
        self.assertEqual(result[3], 0)

    def test_ball_without_goal_approaches_ball(self):
        result = nav.decide_navigation(255, 255, 255, 255, 255, 255, 0, (0, 0),
                                       (180, 128, 100))
        self.assertEqual(result[5], "APPROACH")
        self.assertEqual(result[3], 0)
        self.assertTrue(result[6] & nav.FLAG_BALL)

    def test_rules_detectors_present(self):
        src = (Path(__file__).resolve().parents[1] / "openmv/goal_wall_navigation.py").read_text(encoding="utf-8")
        self.assertIn("BallThresholds", src)
        self.assertIn("ObstacleThresholds", src)
        self.assertIn("OWN_GOAL", src)

    def test_camera_only_display_detects_ball_and_obstacle(self):
        spec = importlib.util.spec_from_file_location(
            "camera_only_display", Path(__file__).resolve().parents[1] / "openmv/camera_only_display.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        objects = module.detected_objects(500, 120, 60, 0, None, None, 0, None, None,
                                          (128, 100, 80), (128, 110, 120))
        names = [obj[0] for obj in objects]
        self.assertIn("BALL", names)
        self.assertIn("OBSTACLE", names)
        lines = module.status_lines(objects, "SHOOT", 1, False, False, 30.0,
                                    module.FLAG_BALL | module.FLAG_BALL_ALIGNED)
        self.assertTrue(any("BALL: SEEN" in line for line in lines))


if __name__ == "__main__":
    unittest.main()
