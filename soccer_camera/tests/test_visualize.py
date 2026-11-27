#!/usr/bin/env python3
"""Host tests for the camera overlay, telemetry parser and goal diagnostics."""
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "openmv"))
import competition_vision as v
import goal_wall_navigation as nav
import visualize as viz

GOLDEN = bytes(int(x, 16) for x in Path(__file__).with_name("telemetry_golden.txt").read_text().split())
ALL = set(range(0, 360, 15))


def diff(a, b):
    return a - b


class Blob:
    def __init__(self, x, y, w, h, pixels=None):
        self.x, self.y, self.w, self.h = x, y, w, h
        self.pixels = w * h * 3 // 4 if pixels is None else pixels
        self.cx, self.cy = x + w // 2, y + h // 2


class Image:
    def __init__(self, yellow=(), blue=()):
        self.found = {"YELLOW": list(yellow), "BLUE": list(blue)}
        self.calls = []

    def find_blobs(self, thresholds, **kwargs):
        self.calls.append(("find", None, None))
        return self.found["YELLOW" if thresholds is nav.cfg.YELLOW else "BLUE"]

    def __getattr__(self, name):
        if not name.startswith("draw_"):
            raise AttributeError(name)
        return lambda *args, **kwargs: self.calls.append((name, args, kwargs.get("color")))


def mirror():
    with patch.multiple(nav.cfg, IMAGE_SIZE=(320, 240), MIRROR_CENTER=(160, 120), MIRROR_RING=(20, 100)):
        return nav.Mirror(320, 240)


class TelemetryTests(unittest.TestCase):
    def test_golden_frame_from_main(self):
        link = viz.MainLink()
        link.feed(b"\x00\xd7junk" + GOLDEN[:7], 0)
        self.assertEqual(link.current(0, diff), (None, None))
        link.feed(GOLDEN[7:], 10)
        data, ball = link.current(10, diff)
        self.assertEqual(ball, 271)
        self.assertEqual((data["ball_confidence"], data["direction"], data["power"], data["turn"]), (88, 45, 160, -9))
        self.assertEqual((data["tactic"], data["mode"], data["bno"]), ("ATTACK", "VISION_ONLY", "OFF"))
        self.assertTrue(data["ir_link"] and data["camera_link"] and data["upgrade"])

    def test_old_ir_information_is_not_current(self):
        link = viz.MainLink()
        link.feed(GOLDEN, 1000)
        self.assertEqual(link.current(1100, diff)[1], 271)
        self.assertIsNone(link.current(1190, diff)[1])     # 20 ms old at MAIN + 190 ms here
        self.assertIsNotNone(link.current(1190, diff)[0])  # the rest is still shown
        self.assertEqual(link.current(1500, diff), (None, None))

    def test_corruption_is_rejected(self):
        for i in range(len(GOLDEN)):
            bad = bytearray(GOLDEN)
            bad[i] ^= 1
            link = viz.MainLink()
            link.feed(bad, 0)
            self.assertIsNone(link.data, i)
            link.feed(GOLDEN, 1)
            self.assertIsNotNone(link.data, i)


class OverlayTests(unittest.TestCase):
    def pipeline(self, image=None):
        with patch.multiple(nav.cfg, IMAGE_SIZE=(320, 240), MIRROR_CENTER=(160, 120), MIRROR_RING=(20, 100)):
            p = v.Pipeline(nav.cfg, mirror(), 25, 100)
            p.keep_reports = True
            for i in range(4):
                p.step(image or Image(), i * 100, True, 30, ALL, [])
        return p

    def draw(self, image, telemetry=True, now=10, **options):
        overlay = viz.Overlay(nav.cfg, mirror(), **options)
        if telemetry:
            overlay.link.feed(GOLDEN, 0)
        p = self.pipeline(image)
        del image.calls[:]
        overlay.draw(image, now, p, [(0, "WHITE", 20, 160, 60)], diff, lambda x, y: None)
        overlay.draw(image, now + 1, p, [], diff, lambda x, y: None)
        return overlay

    def texts(self, image):
        return [args[1] for name, args, _ in image.calls if name == "draw_string"]

    def colors(self, image):
        return {color for name, _, color in image.calls if name.startswith("draw_")}

    def test_all_layers_and_values_are_drawn(self):
        image = Image(yellow=[Blob(145, 36, 30, 12)])
        self.draw(image)
        for color in (viz.RED, viz.ORANGE, viz.GREEN, viz.WHITE, viz.CYAN, viz.YELLOW, viz.BLUE):
            self.assertIn(color, self.colors(image))
        text = " | ".join(self.texts(image))
        for part in ("BALL(IR) | 271 deg | conf 88", "MOVE | 45 deg | pwr 160 | turn -9", "ATTACK / VISION_ONLY",
                     "IR OK | CAM OK | BNO OFF", "FPS", "NOCOLOR", "SEEN"):
            self.assertIn(part, text)

    def test_red_line_follows_front_deg_and_arrows_use_the_same_angles(self):
        m = mirror()
        with patch.multiple(nav.cfg, FRONT_DEG=90.0, CLOCKWISE=False):
            self.assertEqual(viz.image_point(nav.cfg, m, 0, 50), (210, 120))   # front: image right
            self.assertEqual(viz.image_point(nav.cfg, m, 90, 50), (160, 70))   # robot right: image up
            self.assertAlmostEqual(m.bearing(*viz.image_point(nav.cfg, m, 135, 50)), 135, delta=1)

    def test_no_telemetry_or_stale_ball_draws_no_orange_arrow(self):
        image = Image()
        self.draw(image, telemetry=False)
        self.assertIn("MAIN: NO DATA", " ".join(self.texts(image)))
        arrows = [c for name, _, c in image.calls if name == "draw_line"]
        self.assertNotIn(viz.GREEN, arrows)
        self.assertNotIn(viz.ORANGE, arrows)
        stale = Image()
        self.draw(stale, now=300)
        self.assertIn("BALL(IR) | --- | conf -", " | ".join(self.texts(stale)))
        lines = [c for name, _, c in stale.calls if name == "draw_line"]
        self.assertNotIn(viz.ORANGE, lines)

    def test_forced_floor_is_labelled(self):
        for forced, label in ((True, "FLOOR OK (FORCED)"), (False, "FLOOR OK")):
            image = Image()
            overlay = viz.Overlay(nav.cfg, mirror())
            p = self.pipeline(image)
            p.forced = forced
            overlay.draw(image, 10, p, [], diff, None)
            self.assertIn(label, self.texts(image))

    def test_rejection_reason_is_shown(self):
        image = Image(yellow=[Blob(156, 25, 8, 40)])
        self.draw(image, debug_goals=True)
        self.assertIn("NG TALL", " ".join(self.texts(image)))

    def test_text_stays_outside_the_mirror_with_the_real_calibration(self):
        real = nav.Mirror(320, 240)
        image = Image(yellow=[])
        overlay = viz.Overlay(nav.cfg, real)
        overlay.link.feed(GOLDEN, 0)
        overlay.draw(image, 10, self.pipeline(), [], diff, None)
        boxes = [args[0] for name, args, _ in image.calls
                 if name == "draw_rectangle" and args[0][3] == 11]
        panel = [b for b in boxes if b[0] in (viz.LEFT_X, viz.RIGHT_X) or b[1] >= viz.BOTTOM_Y]
        self.assertGreater(len(panel), 10)
        for x, y, w, h in panel:
            self.assertTrue(x + w <= 320 and y + h <= 240, (x, y, w, h))
            nearest_x = min(max(real.cx, x), x + w)
            nearest_y = min(max(real.cy, y), y + h)
            self.assertGreater(nav.distance(nearest_x - real.cx, nearest_y - real.cy), real.outer, (x, y, w, h))

    def test_overlay_reuses_the_recognition_and_searches_only_in_debug(self):
        # 42 frames 50 ms apart: two warm-up searches, then one per interval (frames 12, 22, 32, 42).
        for debug, expected in ((False, 0), (True, 6)):
            image = Image()
            overlay = viz.Overlay(nav.cfg, mirror(), debug_goals=debug)
            p = self.pipeline()
            with patch("builtins.print"):
                for i in range(42):
                    overlay.draw(image, i * 50, p, [], diff, None)
            self.assertEqual(sum(1 for name, _, _ in image.calls if name == "find"), expected)

    def test_debug_search_is_bounded_in_time_and_in_frames(self):
        p = self.pipeline()
        for step, frames, expected in ((10, 1000, 2 + 19), (1000, 44, 2 + 14)):  # per interval; every 3rd frame
            image = Image()
            overlay = viz.Overlay(nav.cfg, mirror(), debug_goals=True)
            with patch("builtins.print") as log:
                for i in range(frames):
                    overlay.draw(image, i * step, p, [], diff, None)
            self.assertEqual(sum(1 for name, _, _ in image.calls if name == "find"), expected, step)
            printed = [str(call[0][0]) for call in log.call_args_list]
            for name in ("YELLOW", "BLUE"):  # the slower rate must not silence one colour
                self.assertGreater(sum(line.startswith(name) for line in printed), 3, (step, name))

    def test_hidden_overlay_draws_nothing(self):
        image = Image()
        self.draw(image, show=False, debug_goals=True)
        self.assertEqual([name for name, _, _ in image.calls if name.startswith("draw_")], [])


class GoalDiagnosticTests(unittest.TestCase):
    def reason(self, blob, color=None):
        report = {}
        v.goal_candidate(blob, mirror(), 12, nav.cfg, color, report)
        return report

    def test_every_rejection_has_a_reason_and_numbers(self):
        self.assertEqual(self.reason(Blob(145, 36, 30, 12, 5))["reason"], "SMALL")
        self.assertEqual(self.reason(Blob(0, 0, 30, 12))["reason"], "RING")
        self.assertEqual(self.reason(Blob(60, 5, 200, 150))["reason"], "WIDTH")
        self.assertEqual(self.reason(Blob(156, 25, 8, 40))["reason"], "TALL")
        self.assertEqual(self.reason(Blob(145, 36, 30, 12, 20))["reason"], "SPARSE")
        self.assertEqual(self.reason(Blob(150, 60, 22, 14), lambda x, y: "GREEN")["reason"], "TURF")
        good = self.reason(Blob(145, 36, 30, 12))
        self.assertEqual(good["reason"], "OK")
        for key in ("pixels", "radius", "width", "aspect", "density", "bearing", "box", "quality"):
            self.assertIn(key, good)

    def test_diagnostics_do_not_change_the_decision(self):
        for blob in (Blob(145, 36, 30, 12), Blob(156, 25, 8, 40)):
            self.assertEqual(v.goal_candidate(blob, mirror(), 12, nav.cfg),
                             v.goal_candidate(blob, mirror(), 12, nav.cfg, None, {}))

    def test_accepted_candidate_is_listed_first(self):
        image = Image(yellow=[Blob(60, 5, 200, 150), Blob(145, 36, 30, 12)])
        reports = viz.inspect_goals(image, "YELLOW", nav.cfg, mirror(), None)
        self.assertEqual([r["reason"] for r in reports], ["OK", "WIDTH"])
        self.assertIn("px=", viz.describe("YELLOW", reports[1]))


if __name__ == "__main__":
    unittest.main()
