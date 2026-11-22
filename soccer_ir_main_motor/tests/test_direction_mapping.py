from __future__ import annotations

import json
import math
import re
import unittest
from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
ROOT = PROJECT.parent
SKETCHES = [
    PROJECT / "common" / "src" / "BoardConfig.ino",
    PROJECT / "main" / "main.ino",
    PROJECT / "ir" / "ir.ino",
    PROJECT / "motor" / "motor.ino",
    PROJECT / "line" / "line.ino",
]
CONTROL_SOURCES = [PROJECT / "common" / "src" / "RobotControl.ino", *SKETCHES[1:]]
EXPECTED_WHEELS = [
    (1, -1, 45, 1),    # RIGHT_REAR
    (-1, 1, 225, 1),   # LEFT_FRONT
    (1, 1, 135, 1),    # RIGHT_FRONT
    (-1, -1, 315, 1),  # LEFT_REAR
]


def int_array(source: str, name: str) -> list[int]:
    match = re.search(
        rf"const\s+\w+\s+{re.escape(name)}\[[^\]]+\]\s*=\s*\{{(.*?)\}};",
        source,
        re.DOTALL,
    )
    if not match:
        raise AssertionError(f"array {name} not found")
    return [int(value) for value in re.findall(r"-?\d+", match.group(1))]


def wheel_config(source: str) -> list[tuple[int, int, int, int]]:
    match = re.search(
        r"constexpr\s+WheelConfig\s+kMotorWheelConfig\[4\]\s*=\s*\{(.*?)\};",
        source,
        re.DOTALL,
    )
    if not match:
        raise AssertionError("kMotorWheelConfig not found")
    rows = re.findall(
        r"\{\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*\}",
        match.group(1),
    )
    return [tuple(map(int, row)) for row in rows]


def omni_outputs(motion: tuple[float, float, float]) -> list[float]:
    x, y, turn = motion
    outputs = []
    for px, py, angle_deg, polarity in EXPECTED_WHEELS:
        angle = math.radians(angle_deg)
        dx, dy = math.cos(angle), math.sin(angle)
        rotation = -py * dx + px * dy
        outputs.append((x * dx + y * dy + turn * rotation) * polarity)
    largest = max(abs(value) for value in outputs)
    if largest > 1:
        outputs = [value / largest for value in outputs]
    return outputs


class DirectionMappingTests(unittest.TestCase):
    def test_ir_drive_is_testable_at_known_motor_pwm_and_requires_arm(self):
        for path in CONTROL_SOURCES:
            with self.subTest(path=path.relative_to(PROJECT)):
                source = path.read_text(encoding="utf-8")
                self.assertIn("constexpr uint8_t kIrTestPwmCap = 128;", source)
                self.assertIn("motion.x = x * (0.50f / projection);", source)
                self.assertIn("bool autoStartPending_ = false;", source)

    def test_all_inlined_wheel_tables_match_zip_based_mapping(self):
        for path in SKETCHES:
            with self.subTest(path=path.relative_to(PROJECT)):
                source = path.read_text(encoding="utf-8")
                self.assertEqual(wheel_config(source), EXPECTED_WHEELS)
                self.assertIn("kMotorWheelConfigConfigured = false", source)
                self.assertIn("kCompetitionArmed = false", source)

    def test_ir_gpio_cardinals_map_to_clockwise_wire_angles(self):
        source = (PROJECT / "ir" / "ir.ino").read_text(encoding="utf-8")
        pins = int_array(source, "sensorPins")
        cosine = int_array(source, "cosTable")
        sine = int_array(source, "sinTable")
        self.assertEqual(len(pins), 12)
        self.assertEqual(len(cosine), len(sine))
        self.assertRegex(
            source,
            r"rawAngle\s*=\s*normalizeDegrees\(360\.0f\s*-\s*estimate\.angleDeg\)",
        )

        actual = {}
        for pin in (18, 21, 24, 27):
            index = pins.index(pin)
            angle = math.degrees(math.atan2(sine[index], cosine[index])) % 360
            actual[pin] = round((360 - angle) % 360) % 360
        self.assertEqual(actual, {18: 0, 21: 270, 24: 180, 27: 90})

    def test_omni_projection_has_expected_relative_wheel_signs(self):
        for path in CONTROL_SOURCES:
            source = path.read_text(encoding="utf-8")
            self.assertIn(
                "const float rotation = -w.positionY * dx + w.positionX * dy;",
                source,
            )
            self.assertIn(
                "output[i] = (motion.x * dx + motion.y * dy + motion.turn * rotation) * w.polarity;",
                source,
            )
        cases = [
            ((0, 1, 0), [1, -1, 1, -1]),     # forward
            ((1, 0, 0), [1, -1, -1, 1]),     # right
            ((0, -1, 0), [-1, 1, -1, 1]),    # backward
            ((-1, 0, 0), [-1, 1, 1, -1]),    # left
            ((0, 0, 1), [1, 1, 1, 1]),       # positive/CCW turn
        ]
        for motion, expected_signs in cases:
            with self.subTest(motion=motion):
                outputs = omni_outputs(motion)
                signs = [1 if value > 1e-6 else -1 if value < -1e-6 else 0 for value in outputs]
                self.assertEqual(signs, expected_signs)

    def test_workspace_is_self_contained_and_default_build_excludes_line(self):
        workspace = json.loads(
            (ROOT / "サッカーロボット_カメラなし.code-workspace").read_text(encoding="utf-8")
        )
        self.assertEqual(
            [folder["path"] for folder in workspace["folders"]],
            ["soccer_ir_main_motor"],
        )
        tasks = {task["label"]: task for task in workspace["tasks"]["tasks"]}
        default = next(
            task for task in tasks.values()
            if task.get("group", {}).get("isDefault") is True
        )
        self.assertEqual(
            default["dependsOn"],
            [
                "コンパイル: MAIN (RP2350B)",
                "コンパイル: IR (RP2350A)",
                "コンパイル: MOTOR (RP2350A)",
            ],
        )
        for board in ("MAIN", "IR", "MOTOR", "LINE"):
            chip = "RP2350B" if board in ("MAIN", "LINE") else "RP2350A"
            compile_task = tasks[f"コンパイル: {board} ({chip})"]
            self.assertTrue((PROJECT / board.lower() / f"{board.lower()}.ino").is_file())
            self.assertIn(chr(36) + "{workspaceFolder}/" + board.lower(), compile_task["args"])

    def test_main_has_no_camera_uart_code(self):
        source = (PROJECT / "main" / "main.ino").read_text(encoding="utf-8")
        self.assertNotRegex(source, r"(?i)openmv|serial2")


if __name__ == "__main__":
    unittest.main()
