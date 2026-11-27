#!/usr/bin/env python3
"""Host tests for the protocol 2 camera pipeline."""
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

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

    def test_escape_avoids_a_second_farther_line(self):
        near = lines(*(wall(0, 15, 15) + wall(180, 32, 15)))
        escape = v.LineGuard(25, 35).update(True, ALL, near)[1]
        self.assertIn(escape, (90, 112.5, 247.5, 270))

    def test_unreadable_floor_beside_a_heading_keeps_it_closed(self):
        # The sector window is about the robot's own width: it is never narrowed to avoid a stop.
        narrow = {75, 90, 105, 0, 15, 345}
        self.assertEqual(v.LineGuard(25, 35).update(True, narrow, lines((0, 15)))[:2], (0, None))

    def test_escape_prefers_bench_verified_directions_when_comparable(self):
        escape = v.LineGuard(25, 35).update(True, ALL, lines((0, 15), (30, 15)))[1]
        self.assertEqual(escape % 90, 0)

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


class PacketWriterTests(unittest.TestCase):
    class FakeUart:
        def __init__(self, results):
            self.results = list(results)
            self.writes = []

        def write(self, data):
            self.writes.append(bytes(data))
            result = self.results.pop(0)
            if isinstance(result, Exception):
                raise result
            return result

    def test_partial_write_resumes_same_packet_then_completes(self):
        uart = self.FakeUart((2, 3))
        tx = nav.LatestPacketTx(uart, lambda now, before: now - before)
        self.assertFalse(tx.send(b"abcde", 0))
        self.assertEqual(tx.offset, 2)
        self.assertTrue(tx.send(b"newer", 1))
        self.assertEqual(uart.writes, [b"abcde", b"cde"])
        self.assertEqual((tx.completed, tx.partial, tx.bytes_written), (1, 1, 5))

    def test_expired_partial_packet_is_dropped_before_latest_packet(self):
        uart = self.FakeUart((2, 5))
        tx = nav.LatestPacketTx(uart, lambda now, before: now - before, max_age_ms=10)
        self.assertFalse(tx.send(b"old-frame", 0))
        self.assertTrue(tx.send(b"newer", 11))
        self.assertEqual(uart.writes[-1], b"newer")
        self.assertEqual(tx.expired, 1)
        self.assertEqual(tx.last_error, "NONE")

    def test_uart_errors_and_no_progress_are_counted(self):
        uart = self.FakeUart((OSError("bus"), 0))
        tx = nav.LatestPacketTx(uart, lambda now, before: now - before)
        self.assertFalse(tx.send(b"packet", 0))
        self.assertEqual(tx.last_error, "UART_OSERROR")
        self.assertFalse(tx.send(b"packet", 1))
        self.assertEqual(tx.last_error, "UART_NO_PROGRESS")
        self.assertEqual(tx.failed, 2)


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
        self.assertIsNone(t.fields(1000)[0])

    def test_track_ignores_a_single_jump_and_adopts_a_repeated_one(self):
        t = v.Track(())
        for now in (0, 50, 100):
            t.update(now, [(90, 0.8, 40, 10)])
        t.update(150, [(90, 0.7, 40, 10), (270, 0.9, 40, 10)])
        self.assertLess(abs(v.wrap(t.bearing - 90)), 5)
        t.update(200, [(270, 0.9, 40, 10)])
        self.assertLess(abs(v.wrap(t.bearing - 90)), 5)  # seen once: not adopted
        t.update(250, [(90, 0.7, 40, 10)])
        t.update(300, [(270, 0.9, 40, 10)])
        self.assertLess(abs(v.wrap(t.bearing - 90)), 5)  # not consecutive
        t.update(350, [(272, 0.9, 40, 10)])
        self.assertLess(abs(v.wrap(t.bearing - 272)), 5)
        self.assertLess(t.confidence, 70)

    def test_heading_needs_both_goals(self):
        self.assertEqual(v.estimate_heading((None, 0), (0, 100)), (None, 0))
        self.assertEqual(v.estimate_heading((180, 100), (0, 100))[0], 0)
        # Robot turned 30 degrees clockwise: the opponent goal appears at 330.
        self.assertAlmostEqual(v.estimate_heading((150, 100), (330, 100))[0], 30, delta=0.01)
        # Sideways translation on the centre line keeps the estimate.
        self.assertAlmostEqual(v.wrap(v.estimate_heading((140, 100), (40, 100))[0]), 0, delta=0.01)
        self.assertEqual(v.estimate_heading((30, 100), (0, 100)), (None, 0))
        self.assertEqual(v.estimate_heading((180, 20), (0, 100)), (None, 0))

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

    def blobs(self, yellow=None, blue=None):
        class Blob:
            def __init__(self, x, y, w, h):
                self.x, self.y, self.w, self.h, self.pixels = x, y, w, h, w * h * 3 // 4
                self.cx, self.cy = x + w // 2, y + h // 2

        found = {"YELLOW": yellow, "BLUE": blue}

        class Frame:
            def find_blobs(self, thresholds, **kwargs):
                for name, box in found.items():
                    if thresholds is getattr(nav.cfg, name) and box:
                        return [Blob(*box)]
                return []

        return Frame()

    def run_pipe(self, frame, steps=24, clock=None, frame_ms=30):
        with patch.multiple(nav.cfg, IMAGE_SIZE=(320, 240), MIRROR_CENTER=(160, 120), MIRROR_RING=(20, 100)):
            pipe = v.Pipeline(nav.cfg, nav.Mirror(320, 240), 25, 100)
            for i in range(steps):
                packet = pipe.step(frame, i * 50, True, frame_ms, ALL, [], clock)
        return pipe, packet

    def test_heading_needs_two_recently_seen_goals(self):
        both = self.blobs(yellow=(150, 40, 20, 10), blue=(150, 190, 20, 10))
        pipe, packet = self.run_pipe(both)
        self.assertNotEqual(packet[26] | packet[27] << 8, v.UNKNOWN_ANGLE)
        with patch.multiple(nav.cfg, IMAGE_SIZE=(320, 240), MIRROR_CENTER=(160, 120), MIRROR_RING=(20, 100)):
            later = pipe.step(self.blobs(), 24 * 50 + 500, True, 30, ALL, [])
        self.assertEqual(later[26] | later[27] << 8, v.UNKNOWN_ANGLE)

    def test_detection_time_counts_against_line_freshness(self):
        ticks = iter(range(0, 100000, 90))
        _, packet = self.run_pipe(self.blobs(), steps=2, clock=lambda: next(ticks), frame_ms=60)
        self.assertFalse(packet[5] & v.FLAG_READY)
        self.assertEqual(packet[6] | packet[7], 0)

    def test_escape_is_always_a_direction_the_wheels_can_follow_exactly(self):
        for first in range(0, 360, 15):
            for second in range(0, 360, 45):
                near = lines(*(wall(first, 15, 30) + wall(second, 22, 15)))
                escape = v.LineGuard(25, 35).update(True, ALL, near)[1]
                self.assertTrue(escape is None or escape % 45 == 0, (first, second, escape))

    def test_nearby_lines_count_as_observed_floor(self):
        visible = set(range(90, 210, 15))
        seen = [(b, "WHITE", 15, 0, 0) for b in range(0, 75, 15)]
        self.assertEqual(v.observed_rays(visible, seen), 13)

    def test_slow_detection_is_skipped_to_keep_line_data_fresh(self):
        clock = [0]

        class Slow:
            def find_blobs(self, *args, **kwargs):
                clock[0] += 50
                return []

        with patch.multiple(nav.cfg, IMAGE_SIZE=(320, 240), MIRROR_CENTER=(160, 120), MIRROR_RING=(20, 100)):
            pipe = v.Pipeline(nav.cfg, nav.Mirror(320, 240), 25, 100)
            ready = [bool(pipe.step(Slow(), i * 70, True, 60, ALL, [], lambda: clock[0])[5] & v.FLAG_READY)
                     for i in range(60)]
        self.assertGreaterEqual(sum(ready), 50)

    def pipe(self, frame, color_of=None, steps=24, target="YELLOW"):
        with patch.multiple(nav.cfg, IMAGE_SIZE=(320, 240), MIRROR_CENTER=(160, 120), MIRROR_RING=(20, 100),
                            TARGET_GOAL=target):
            p = v.Pipeline(nav.cfg, nav.Mirror(320, 240), 25, 100)
            for i in range(steps):
                packet = p.step(frame, i * 50, True, 30, ALL, [], color_of=color_of)
        return p, packet

    def yellow_bearing(self, packet):
        return packet[14] | packet[15] << 8

    def test_camera_never_detects_or_reports_a_ball(self):
        for name in ("BALL", "BALL_MIN_PIXELS", "BALL_DISTANCE"):
            self.assertFalse(hasattr(nav.cfg, name))
        for name in ("ball_like", "overlaps_ball"):
            self.assertFalse(hasattr(nav, name))
        asked = []

        class Frame:
            def find_blobs(self, thresholds, **kwargs):
                asked.append(thresholds)
                return []

        pipe, packet = self.pipe(Frame())
        self.assertTrue(asked)
        self.assertTrue(all(t is nav.cfg.YELLOW or t is nav.cfg.BLUE for t in asked))
        self.assertEqual(len(packet), 32)
        self.assertEqual(sorted(pipe.goals), ["BLUE", "YELLOW"])
        self.assertLessEqual(len(asked), 24 // 2 + 1)  # at most one colour search every other frame

    def test_orange_object_on_the_field_is_neither_ball_nor_goal(self):
        # An orange robot passes the yellow thresholds, but turf continues behind it.
        _, packet = self.pipe(self.blobs(yellow=(150, 60, 22, 14)), color_of=lambda x, y: "GREEN")
        self.assertEqual(self.yellow_bearing(packet), v.UNKNOWN_ANGLE)

    def test_goal_against_the_wall_is_accepted(self):
        def floor_inside(x, y):
            return "GREEN" if (x - 160) ** 2 + (y - 120) ** 2 < 65 ** 2 else None

        _, packet = self.pipe(self.blobs(yellow=(145, 36, 30, 12)), color_of=floor_inside)
        self.assertNotEqual(self.yellow_bearing(packet), v.UNKNOWN_ANGLE)
        self.assertGreaterEqual(packet[17], 40)

    def test_false_goal_shapes_are_rejected(self):
        for box in ((156, 25, 8, 40),    # tall strip: cable, LED bar
                    (158, 40, 3, 3),     # tiny speck
                    (60, 20, 200, 200)):  # covers most of the mirror
            _, packet = self.pipe(self.blobs(blue=box, yellow=box))
            self.assertEqual(self.yellow_bearing(packet), v.UNKNOWN_ANGLE, box)

    def test_sparse_colour_is_rejected(self):
        class Sparse:
            x, y, w, h, pixels, cx, cy = 145, 36, 30, 12, 40, 160, 42

        self.assertIsNone(v.goal_candidate(Sparse(), nav.Mirror(320, 240), 12, nav.cfg))

    def test_goals_in_similar_directions_are_not_trusted(self):
        yellow, blue = [10, None, 90, 10, 0], [60, None, 60, 10, 0]
        v.separate_goals(yellow, blue)
        self.assertEqual((yellow[2], blue[2]), (45, 0))

    def test_single_wide_white_blocks_without_a_retreat(self):
        guard = v.LineGuard(25, 35)
        mask, escape, _, _ = guard.update(True, ALL, lines((0, 15)), thin=set())
        self.assertIsNone(escape)
        self.assertFalse(allows(mask, 0) or allows(mask, 45))
        self.assertTrue(allows(mask, 180))
        self.assertEqual(guard.update(True, ALL, lines((0, 15)), thin=set())[1], 180)  # persists: treat as line

    def test_goals_are_still_confirmed_when_the_camera_is_slow(self):
        frame = self.blobs(yellow=(150, 40, 20, 10))
        with patch.multiple(nav.cfg, IMAGE_SIZE=(320, 240), MIRROR_CENTER=(160, 120), MIRROR_RING=(20, 100)):
            p = v.Pipeline(nav.cfg, nav.Mirror(320, 240), 25, 100)
            for i in range(30):
                packet = p.step(frame, i * 100, True, 80, ALL, [])
        self.assertGreaterEqual(packet[17], 40)

    def test_thin_line_with_turf_beyond_retreats_at_once(self):
        self.assertEqual(v.LineGuard(25, 35).update(True, ALL, lines((0, 15)), thin={0})[1], 180)

    def test_thin_line_needs_narrow_white_followed_by_turf(self):
        samples = [(i, 0, 18 + 2 * i) for i in range(16)]

        def paint(white, after):
            return lambda x, y: "GREEN" if x < 3 else "WHITE" if x < 3 + white else after

        edge_sample = ("WHITE", 24, 3, 0)
        self.assertTrue(nav.line_is_thin(samples, edge_sample, paint(2, "GREEN")))
        self.assertFalse(nav.line_is_thin(samples, edge_sample, paint(10, "GREEN")))  # wide: robot shell
        self.assertFalse(nav.line_is_thin(samples, edge_sample, paint(2, "BLACK")))   # nothing green beyond
        self.assertFalse(nav.line_is_thin(samples, edge_sample, paint(2, None)))

    def test_single_floor_gap_is_filled_only_between_clear_neighbours(self):
        with patch.multiple(nav.cfg, IMAGE_SIZE=(320, 240), MIRROR_CENTER=(160, 120), MIRROR_RING=(20, 100)):
            p = v.Pipeline(nav.cfg, nav.Mirror(320, 240), 25, 100)
        self.assertEqual(p.fill_gap(ALL - {90}, []), ALL)
        self.assertEqual(p.fill_gap(ALL - {90, 105}, []), ALL - {90, 105})
        self.assertEqual(p.fill_gap(ALL - {90}, lines((105, 20))), ALL - {90})
        self.assertEqual(p.fill_gap(ALL - {90}, lines((90, 20))), ALL - {90})

    def test_table_crc_matches_the_bitwise_definition(self):
        def bitwise(data):
            crc = 0xFFFF
            for byte in data:
                crc ^= byte << 8
                for _ in range(8):
                    crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
            return crc

        for n in range(64):
            data = bytes((i * 37 + n * 11) & 0xFF for i in range(n))
            self.assertEqual(v.crc16(data), bitwise(data))

    def test_color_cache_answers_like_the_direct_classification(self):
        class Frame:
            def get_pixel(self, position, *, rgbtuple):
                x = position[0]
                return (x % 7 * 15, x % 5 * 20 - 40, x % 3 * 25 - 30)

        calls = []

        class Lab:
            @staticmethod
            def rgb_to_lab(rgb):
                calls.append(rgb)
                return rgb

        with patch.object(nav, "image", Lab):
            cache = nav.ColorCache()
            expected = [nav.ground_color(Frame(), x, 0) for x in range(600)]
            calls.clear()
            self.assertEqual([cache(Frame(), x, 0) for x in range(600)], expected)
            self.assertEqual(len(calls), len(cache.known))  # each colour classified once, None included
            self.assertIn(None, expected)
            with patch.object(nav, "COLOR_CACHE_LIMIT", 4):
                small = nav.ColorCache()
                self.assertEqual([small(Frame(), x, 0) for x in range(600)], expected)
                self.assertLessEqual(len(small.known), 4)

    def test_golden_packet_shared_with_main(self):
        packet = v.make_packet(7, 0x15, 0xFF0F, None, (350, 31), 1,
                               [0, 120, 90, 25, 0], [180, None, 60, 20, 3], (355, 55), 33)
        golden = Path(__file__).with_name("camera_v2_golden.txt").read_text().split()
        self.assertEqual([int(x, 16) for x in golden], list(packet))


if __name__ == "__main__":
    unittest.main()
