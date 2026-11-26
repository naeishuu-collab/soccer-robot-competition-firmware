#!/usr/bin/env python3
"""Host tests for the protocol 2 camera pipeline."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "openmv"))
import competition_vision as v
import goal_wall_navigation as nav

ALL = set(range(0, 360, 15))


def lines(*items):
    return [(b, "WHITE", cm, 0, 0) for b, cm in items]


def wall(center, cm=15, spread=45):
    return [((center + o) % 360, cm) for o in range(-spread, spread + 1, 15)]


def allows(mask, heading):
    return bool(mask & (1 << int(heading / 22.5 + 0.5) % 16))


class LineGuardTests(unittest.TestCase):
    def test_unknown_floor_allows_nothing(self):
        self.assertEqual(v.LineGuard(25, 35).update(False, ALL, []), (0, None, None, 0))
        self.assertEqual(v.LineGuard(25, 35).update(True, set(), [])[0], 0)

    def test_open_floor_allows_all_sixteen(self):
        self.assertEqual(v.LineGuard(25, 35).update(True, ALL, [])[0], 0xFFFF)

    def test_front_line_escapes_backwards(self):
        mask, escape, nearest, count = v.LineGuard(25, 35).update(True, ALL, lines(*wall(0)))
        self.assertEqual(escape, 180)
        self.assertEqual(nearest[1], 15)
        self.assertEqual(count, 1)
        self.assertFalse(allows(mask, 0) or allows(mask, 90))
        self.assertTrue(allows(mask, 180))

    def test_slow_zone_closes_approach_before_escape(self):
        mask, escape, _, _ = v.LineGuard(25, 35).update(True, ALL, lines(*wall(0, 30, 15)))
        self.assertIsNone(escape)
        self.assertFalse(allows(mask, 0) or allows(mask, 45))
        self.assertTrue(allows(mask, 90) and allows(mask, 180))

    def test_corner_escapes_diagonally(self):
        near = wall(0) + wall(90)
        _, escape, _, _ = v.LineGuard(25, 35).update(True, ALL, lines(*near))
        self.assertEqual(escape, 225)

    def test_surrounded_stops(self):
        near = [(b, 15) for b in range(0, 360, 15)]
        mask, escape, _, _ = v.LineGuard(25, 35).update(True, ALL, lines(*near))
        self.assertEqual((mask, escape), (0, None))

    def test_separate_lines_are_counted(self):
        count = v.LineGuard(25, 35).update(True, ALL, lines(*(wall(0, 30, 15) + wall(180, 30, 15))))[3]
        self.assertEqual(count, 2)

    def test_release_hysteresis_and_continuity(self):
        guard = v.LineGuard(25, 35)
        self.assertEqual(guard.update(True, ALL, lines(*wall(0)))[1], 180)
        # 28 cm is outside the trigger but inside the release margin.
        self.assertTrue(guard.update(True, ALL, lines(*wall(0, 28)))[1] is not None)
        self.assertEqual(guard.update(True, ALL, [])[1], 180)
        self.assertIsNone(guard.update(True, ALL, [])[1])


class RecognitionTests(unittest.TestCase):
    def test_distance_table_never_extrapolates(self):
        table = ((30, 20), (60, 100))
        self.assertEqual(v.table_distance(table, 45), 60)
        for bad in (10, 80):
            self.assertIsNone(v.table_distance(table, bad))
        self.assertIsNone(v.table_distance((), 45))
        self.assertIsNone(v.table_distance(((30, 100), (60, 20)), 45))

    def test_track_wraps_angles_and_expires(self):
        t = v.Track(())
        for now, bearing in ((0, 355), (50, 5), (100, 5)):
            t.update(now, [(bearing, 1.0, 40, 10)])
        self.assertTrue(t.bearing < 10 or t.bearing > 350)
        self.assertEqual(t.confidence, 100)
        t.update(150, [])
        self.assertLess(t.confidence, 100)
        self.assertIsNone(t.fields(500)[0])

    def test_track_prefers_continuity_and_resets_confidence_on_jump(self):
        t = v.Track(())
        for now in (0, 50, 100):
            t.update(now, [(90, 0.8, 40, 10)])
        t.update(150, [(90, 0.7, 40, 10), (270, 0.9, 40, 10)])
        self.assertLess(abs(v.wrap(t.bearing - 90)), 5)
        t.update(200, [(270, 0.9, 40, 10)])
        self.assertLess(t.confidence, 40)

    def test_heading_needs_both_goals(self):
        self.assertEqual(v.estimate_heading((None, 0), (0, 100)), (None, 0))
        self.assertEqual(v.estimate_heading((180, 100), (0, 100))[0], 0)
        # Robot turned 30 degrees clockwise: the opponent goal appears at 330.
        self.assertAlmostEqual(v.estimate_heading((150, 100), (330, 100))[0], 30, delta=0.01)
        # Sideways translation on the centre line keeps the estimate.
        self.assertAlmostEqual(v.wrap(v.estimate_heading((140, 100), (40, 100))[0]), 0, delta=0.01)
        self.assertEqual(v.estimate_heading((30, 100), (0, 100)), (None, 0))
        self.assertEqual(v.estimate_heading((180, 20), (0, 100)), (None, 0))

    def test_ball_candidate_rejects_elongated_and_small(self):
        class Blob:
            def __init__(self, w, h, pixels):
                self.x, self.y, self.w, self.h, self.pixels = 200, 110, w, h, pixels
                self.cx, self.cy = 200 + w // 2, 110 + h // 2

        mirror = nav.Mirror(320, 240)
        self.assertIsNotNone(v.blob_candidate(Blob(10, 10, 70), mirror, 10, True))
        self.assertIsNone(v.blob_candidate(Blob(40, 8, 200), mirror, 10, True))
        self.assertIsNone(v.blob_candidate(Blob(10, 10, 4), mirror, 10, True))

    def test_pipeline_runs_without_display_and_flags_unready(self):
        class Frame:
            def find_blobs(self, *args, **kwargs):
                return []

        pipe = v.Pipeline(nav.cfg, nav.Mirror(320, 240), 25, 100)
        packet = pipe.step(Frame(), 0, False, 30, set(), [])
        self.assertEqual(len(packet), v.PACKET_SIZE)
        self.assertFalse(packet[5] & v.FLAG_READY)
        self.assertEqual(packet[6] | packet[7], 0)
        self.assertEqual(v.crc16(packet[:-2]), packet[-2] | packet[-1] << 8)

    def test_golden_packet_shared_with_main(self):
        packet = v.make_packet(7, 0x15, 0xFF0F, None, (350, 31), 1, (10, None, 80, 0, 2),
                               [0, 120, 90, 25, 0], [180, None, 60, 20, 3], (355, 55), 33)
        golden = Path(__file__).with_name("camera_v2_golden.txt").read_text().split()
        self.assertEqual([int(x, 16) for x in golden], list(packet))


if __name__ == "__main__":
    unittest.main()
