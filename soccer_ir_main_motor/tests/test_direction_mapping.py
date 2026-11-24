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


def floor_nonzero_pwm(values: list[int], floor: int) -> list[int]:
    return [0 if value == 0 else (1 if value > 0 else -1) * max(abs(value), floor) for value in values]


def scale_to_peak(values: list[int], peak: int) -> list[int]:
    largest = max((abs(value) for value in values), default=0)
    if largest == 0:
        return values[:]
    return [0 if value == 0 else (1 if value > 0 else -1) * math.floor(abs(value) * peak / largest + 0.5) for value in values]


def first_ramp_output(requested: int, floor: int, step: int) -> int:
    if abs(requested) >= floor:
        return (1 if requested > 0 else -1) * floor
    return (1 if requested > 0 else -1) * min(abs(requested), step) if requested else 0


class DirectionMappingTests(unittest.TestCase):
    def test_ir_drive_is_testable_at_known_motor_pwm_and_requires_arm(self):
        for path in CONTROL_SOURCES:
            with self.subTest(path=path.relative_to(PROJECT)):
                source = path.read_text(encoding="utf-8")
                self.assertIn("constexpr uint8_t kIrTestPwmCap = 210;", source)
                self.assertIn("constexpr uint8_t kIrTestPwmFloor = 120;", source)
                if path == PROJECT / "main" / "main.ino":
                    self.assertIn(
                        "inline bool cardinalMotionForBearing(float directionDeg, float magnitude, Motion& motion)",
                        source,
                    )
                    self.assertIn("if (directionDeg < 45.0f || directionDeg >= 315.0f)", source)
                    self.assertIn("else if (directionDeg < 135.0f)", source)
                    self.assertIn("else if (directionDeg < 225.0f)", source)
                    self.assertIn(
                        "return cardinalMotionForBearing(static_cast<float>(directionDeg), 0.50f, motion);",
                        source,
                    )
                else:
                    self.assertIn("motion.x = x * (0.50f / projection);", source)
                self.assertIn("bool autoStartPending_ = false;", source)
                self.assertIn("inline void floorMotorCommandValues(int16_t values[4], uint8_t minPwm)", source)
                self.assertIn("inline void scaleMotorCommandValuesToPeak(int16_t values[4], uint8_t peakPwm)", source)
        self.assertIn("scaleMotorCommandValuesToPeak(values, kIrTestPwmCap);", (PROJECT / "main/main.ino").read_text(encoding="utf-8"))

    def test_pwm_floor_preserves_zero_and_sign(self):
        values = [0, 1, 119, 120, 210, -1, -119, -120, -210]
        self.assertEqual(
            floor_nonzero_pwm(values, 120),
            [0, 120, 120, 120, 210, -120, -120, -120, -210],
        )
        self.assertIn("floorMotorCommandValues(values, kIrTestPwmFloor);", (PROJECT / "main/main.ino").read_text(encoding="utf-8"))

    def test_pwm_scaling_reaches_cap_before_floor(self):
        scaled = scale_to_peak([64, -32, 0, 16], 210)
        self.assertEqual(scaled, [210, -105, 0, 53])
        self.assertEqual(floor_nonzero_pwm(scaled, 120), [210, -120, 0, 120])

    def test_motor_skips_the_ramp_below_start_floor(self):
        motor_source = (PROJECT / "motor" / "motor.ino").read_text(encoding="utf-8")
        self.assertIn("if (current == 0 && (requested >= startFloor || requested <= -startFloor))", motor_source)
        self.assertIn("motorOutput(index, current);", motor_source)
        self.assertEqual(first_ramp_output(210, 120, 5), 120)
        self.assertEqual(first_ramp_output(-210, 120, 5), -120)
        self.assertEqual(first_ramp_output(80, 120, 5), 5)
        self.assertEqual(first_ramp_output(0, 120, 5), 0)

    def test_all_inlined_wheel_tables_match_zip_based_mapping(self):
        for path in SKETCHES:
            with self.subTest(path=path.relative_to(PROJECT)):
                source = path.read_text(encoding="utf-8")
                self.assertEqual(wheel_config(source), EXPECTED_WHEELS)
                self.assertIn("constexpr int8_t kMotorIn1Pins[4] = {6, 12, 11, 9};", source)
                self.assertIn("constexpr int8_t kMotorIn2Pins[4] = {7, 13, 10, 8};", source)
                self.assertIn("kMotorWheelConfigConfigured = false", source)
                self.assertIn("kCompetitionArmed = false", source)

    def test_motor_driver_keeps_verified_rr_output_polarity(self):
        source = (PROJECT / "motor" / "motor.ino").read_text(encoding="utf-8")
        self.assertIn("constexpr int8_t kMotorOutputPolarity[4] = {-1, 1, 1, 1};", source)

    def test_ir_auto_requires_explicit_arm_but_allows_unverified_wheel_test(self):
        source = (PROJECT / "main" / "main.ino").read_text(encoding="utf-8")
        self.assertIn("AutoMode autoMode = kAutoStopped;", source)
        self.assertNotIn("irTestController.enableAutoStart();", source)
        self.assertNotIn("irTestController.armOnFirstFreshBall(", source)
        self.assertIn("constexpr bool kIrTestWheelMappingVerified = false;", source)
        self.assertIn("constexpr int8_t kBnoHeadingTurnSign = 0;", source)
        self.assertIn("if (autoMode == kAutoIrTest && headingFresh &&", source)
        self.assertNotIn("kIrTestWheelMappingVerified &&", source)
        self.assertNotIn("kBnoHeadingTurnSign != 0 &&", source)
        self.assertIn(
            "motorMotion.turn = headingTurn * static_cast<float>(kBnoHeadingTurnSign);",
            source,
        )
        self.assertIn("irTestHeadingTargetDeg = headingSample.headingDeg;", source)
        self.assertIn("irTestController.armed() && !input.headingValid", source)

    def test_manual_motor_command_times_out_and_queues_stop(self):
        source = (PROJECT / "main" / "main.ino").read_text(encoding="utf-8")
        self.assertIn("constexpr uint32_t kMotorConsoleTimeoutMs = 300;", source)
        self.assertIn(
            "motorCommandExpired(nowMs, lastMotorConsoleCommandMs, kMotorConsoleTimeoutMs)",
            source,
        )
        self.assertIn("motorStopPending = true;", source)
        self.assertIn('Serial.println("MOTOR_CMD_TIMEOUT; stopped");', source)

    def test_main_motor_direct_wheel_protocol_matches(self):
        main = (PROJECT / "main" / "main.ino").read_text(encoding="utf-8")
        motor = (PROJECT / "motor" / "motor.ino").read_text(encoding="utf-8")
        for declaration in (
            "constexpr uint8_t kMotorCommandStart = 0xAA;",
            "constexpr uint8_t kMotorCommandWheelCount = 4;",
            "constexpr uint8_t kMotorCommandEnd = 0x55;",
            "constexpr uint8_t kMotorCommandFrameSize = 12;",
        ):
            self.assertIn(declaration, main)
            self.assertIn(declaration, motor)
        self.assertIn("out[2 + 2 * i] = static_cast<uint8_t>(raw);", main)
        self.assertIn("out[3 + 2 * i] = static_cast<uint8_t>(raw >> 8);", main)
        self.assertIn("out[kMotorCommandFrameSize - 2] = checksum;", main)
        self.assertIn("out[kMotorCommandFrameSize - 1] = kMotorCommandEnd;", main)
        self.assertIn("buffer_[2 + 2 * i]", motor)
        self.assertIn("buffer_[3 + 2 * i]", motor)
        self.assertIn("checksum != buffer_[kMotorCommandFrameSize - 2]", motor)
        self.assertIn("buffer_[kMotorCommandFrameSize - 1] != kMotorCommandEnd", motor)

    def test_ir_gpio_cardinals_map_to_clockwise_wire_angles(self):
        source = (PROJECT / "ir" / "ir.ino").read_text(encoding="utf-8")
        pins = int_array(source, "sensorPins")
        cosine = int_array(source, "cosTable")
        sine = int_array(source, "sinTable")
        self.assertEqual(len(pins), 12)
        self.assertEqual(len(cosine), len(sine))
        self.assertRegex(
            source,
            r"rawAngle\s*=\s*calibratedDirection\s*>=\s*0[\s\S]*?normalizeDegrees\(360\.0f\s*-\s*estimate\.angleDeg\)",
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
        for board in ("MAIN", "IR", "MOTOR"):
            chip = "RP2350B" if board in ("MAIN", "LINE") else "RP2350A"
            compile_task = tasks[f"コンパイル: {board} ({chip})"]
            self.assertTrue((PROJECT / board.lower() / f"{board.lower()}.ino").is_file())
            self.assertIn(chr(36) + "{workspaceFolder}/" + board.lower(), compile_task["args"])
        self.assertTrue((PROJECT / "line" / "line.ino").is_file())
        self.assertFalse(any("LINE" in label for label in tasks))

    def test_main_has_no_camera_uart_code(self):
        source = (PROJECT / "main" / "main.ino").read_text(encoding="utf-8")
        self.assertNotRegex(source, r"(?i)openmv|serial2")


if __name__ == "__main__":
    unittest.main()
