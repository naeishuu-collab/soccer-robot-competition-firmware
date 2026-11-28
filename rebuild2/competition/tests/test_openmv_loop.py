"""Drive the real OpenMV entry point with fake camera, UART, USB, and clock."""
import runpy
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "openmv"))
import vision
import preview


class EndFrames(Exception):
    pass


class CameraLoopTests(unittest.TestCase):
    def test_version_mismatch_names_module_before_uart_start(self):
        boot = str(Path(vision.__file__).with_name("main.py"))
        for module in (vision, preview, vision.cfg):
            with patch.dict(sys.modules, {name: SimpleNamespace() for name in ("image", "pyb", "sensor")}), \
                    patch.object(module, "PRODUCTION_API", 3):
                with self.assertRaisesRegex(RuntimeError, module.__name__ + " API mismatch"):
                    runpy.run_path(boot)

    def test_white_line_transmits_before_optional_goal_work(self):
        self.exercise_loop(None)

    def test_distance_calibration_clears_ready_and_sends_before_target_search(self):
        self.exercise_loop(45)

    def test_disabled_goal_search(self):
        self.exercise_loop(None, False)

    def exercise_loop(self, calibration_cm, enabled=True):
        entries = []
        logs = []
        commands = iter((b"b", b"l", b"i", b"g", b"i"))
        clock = 0
        frames = 0
        packets = []
        draws = []
        green = (37, -9, 5)

        class Frame:
            def width(self):
                return 320

            def height(self):
                return 240

            def get_pixel(self, point, rgbtuple):
                return green

            def draw_string(self, *args, **kwargs):
                draws.append((len(packets), entries[-1]))

            def find_blobs(self, thresholds, **kwargs):
                self.assert_pristine()
                if calibration_cm is not None:
                    assert thresholds == [vision.cfg.WHITE]
                    entries.append("calibration")
                    return [SimpleNamespace(rect=(220, 128, 4, 8), cx=222, cy=132, pixels=18)]
                entries.append("yellow" if thresholds == vision.cfg.YELLOW else "both" if thresholds == vision.GOAL_THRESHOLDS else "blue")
                if len(packets) == 1 and thresholds == vision.GOAL_THRESHOLDS:
                    raise TypeError("simulated optional goal API error")
                return []

            def assert_pristine(self):
                assert not any(n == len(packets) for n, _ in draws)

        def snapshot():
            nonlocal frames, clock
            frames += 1
            if frames > 6:
                raise EndFrames()
            clock += 45000
            return Frame()

        def write(packet):
            entries.append("uart")
            packets.append(bytes(packet))
            if len(packets) == 3:
                raise OSError("simulated temporary UART failure")
            return len(packet)

        def noop(*args, **kwargs):
            pass

        usb = SimpleNamespace(any=lambda: True, isconnected=lambda: True,
                              read=lambda _size: next(commands, None),
                              debug_mode_enabled=lambda: False,
                              send=lambda data, timeout: logs.append(data.decode()) or len(data))
        sensor = SimpleNamespace(snapshot=snapshot, reset=noop, RGB565=1, QVGA=2, set_pixformat=noop,
                                 set_framesize=noop, set_auto_exposure=noop, set_auto_gain=noop,
                                 set_auto_whitebal=noop, skip_frames=noop, get_rgb_gain_db=lambda: 0)
        modules = {
            "image": SimpleNamespace(rgb_to_lab=lambda rgb: rgb),
            "pyb": SimpleNamespace(UART=lambda *args, **kwargs: SimpleNamespace(write=write),
                                   USB_VCP=lambda: usb, LED=lambda _n: SimpleNamespace(off=noop)),
            "sensor": sensor,
            "time": SimpleNamespace(ticks_us=lambda: clock, ticks_ms=lambda: clock // 1000,
                                    ticks_diff=lambda a, b: a-b, ticks_add=lambda a, b: a+b),
        }
        with patch.dict(sys.modules, modules), patch.object(vision.cfg, "BENCH_DISTANCE_CM", calibration_cm), \
                patch.object(vision.cfg, "ENABLE_GOAL_CORRECTION", enabled):
            with self.assertRaises(EndFrames):
                runpy.run_path(str(Path(vision.__file__).with_name("main.py")))
        if calibration_cm is not None:
            self.assertEqual(entries, ["uart", "calibration"] * 5)
            self.assertTrue(all(p[3] == 8 and p[4:8] == bytes(4) for p in packets))
            self.assertTrue(any("CALIBRATION_CM=45" in line for line in logs))
            self.assertTrue(any("TARGET_XY=(222, 132)" in line and "SEGMENT=" in line for line in logs))
            return
        self.assertEqual(entries, ["uart", "both", "uart", "uart", "uart", "uart"] if enabled else ["uart"] * 5)
        self.assertTrue(all(p[3] == 9 and p[4:6] == b"\xff\xff" for p in packets))
        self.assertTrue(all(p[20] == 3 for p in packets))
        self.assertEqual([p[2] for p in packets], list(range(5)))
        self.assertEqual(draws[0], (1, "both" if enabled else "uart"))
        self.assertTrue(any("DARK_MASK=0000" in line and "SELF_MASK=0000" in line for line in logs))
        self.assertTrue(any("LINE_CLASS=NO_CURRENT_LINE CLASS_CONFIDENCE=UNASSESSED" in line for line in logs))
        self.assertIn("UART_PARTIAL=1", logs[-1])
        self.assertIn("FRAME_LATE=0", logs[-1])
        self.assertIn("GOAL_ERROR=1/find_goals/TypeError" if enabled else "GOAL_ERROR=0", logs[-1])
