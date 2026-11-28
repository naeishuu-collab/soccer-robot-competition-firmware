"""Drive the real OpenMV entry point with fake camera, UART, USB, and clock."""
import runpy
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "openmv"))
import vision


class EndFrames(Exception):
    pass


class CameraLoopTests(unittest.TestCase):
    def test_white_line_transmits_before_optional_goal_work(self):
        entries = []
        logs = []
        commands = iter((b"b", b"l", b"i", b"g", b"i"))
        clock = 0
        frames = 0
        packets = []
        green = (37, -9, 5)

        class Frame:
            def width(self):
                return 320

            def height(self):
                return 240

            def get_pixel(self, point, rgbtuple):
                return green

            def find_blobs(self, thresholds, **kwargs):
                entries.append("yellow" if thresholds == vision.cfg.YELLOW else "blue")
                return []

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
                              send=lambda data, timeout: logs.append(data.decode()))
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
        with patch.dict(sys.modules, modules):
            with self.assertRaises(EndFrames):
                runpy.run_path(str(Path(vision.__file__).with_name("main.py")))
        self.assertEqual(entries, ["uart", "yellow", "uart", "yellow", "blue", "uart", "uart", "uart", "yellow"])
        self.assertTrue(all(p[3] == 1 and p[4:6] == b"\xff\xff" for p in packets))
        self.assertEqual([p[2] for p in packets], list(range(5)))
        self.assertIn("UART_PARTIAL=1", logs[-1])
        self.assertIn("FRAME_LATE=0", logs[-1])
