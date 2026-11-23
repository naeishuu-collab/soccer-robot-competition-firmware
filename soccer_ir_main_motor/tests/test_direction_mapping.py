from __future__ import annotations

import math
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
COMMON = PROJECT / "common" / "src"
CONTROL_SOURCES = [COMMON / "RobotControl.h", PROJECT / "line" / "line.ino"]
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
    def test_ir_test_requires_explicit_arm_without_marking_mapping_verified(self):
        tuning = (COMMON / "RobotTuning.h").read_text(encoding="utf-8")
        control = (COMMON / "RobotControl.h").read_text(encoding="utf-8")
        main = (PROJECT / "main/main.ino").read_text(encoding="utf-8")
        board = (COMMON / "BoardConfig.h").read_text(encoding="utf-8")
        self.assertRegex(tuning, r"constexpr uint8_t kIrTestPwmCap = \d+;")
        self.assertRegex(tuning, r"constexpr uint8_t kIrTestPwmFloor = \d+;")
        self.assertIn("constexpr int8_t kBnoHeadingTurnSign = 0;", tuning)
        self.assertIn("if (directionDeg < 45.0f || directionDeg >= 315.0f)", control)
        self.assertIn("else if (directionDeg < 135.0f)", control)
        self.assertIn("else if (directionDeg < 225.0f)", control)
        self.assertIn(
            "return cardinalMotionForBearing(static_cast<float>(directionDeg), kIrTestDriveMagnitude, motion);",
            control,
        )
        self.assertIn("AutoMode autoMode = kAutoStopped;", main)
        self.assertNotIn("irTestController.enableAutoStart()", main)
        self.assertIn("autoMode == kAutoIrTest && headingFresh", main)
        self.assertIn("irTestController.arm(irFresh, irBallFound, irDirection)", main)
        self.assertIn("kMotorWheelConfigConfigured = false", board)
        self.assertIn("kCompetitionArmed = false", board)
        self.assertIn("scaleMotorCommandValuesToPeak(values, kIrTestPwmCap);", (PROJECT / "motor/motor.ino").read_text(encoding="utf-8"))

    def test_manual_motor_command_times_out_and_motor_watchdog_remains(self):
        tuning = (COMMON / "RobotTuning.h").read_text(encoding="utf-8")
        main = (PROJECT / "main/main.ino").read_text(encoding="utf-8")
        motor = (PROJECT / "motor/motor.ino").read_text(encoding="utf-8")
        self.assertIn("constexpr uint32_t kMotorConsoleTimeoutMs = 300;", tuning)
        self.assertIn("motorConsoleManualActive &&", main)
        self.assertIn("motorConsoleCommandSeen = false;", main)
        self.assertIn('Serial.println("MOTOR_CMD_TIMEOUT; stopped")', main)
        self.assertIn("constexpr uint32_t kCommandTimeoutMs = 100;", tuning)
        self.assertIn("if (kSoftwareWatchdogEnabled && hasCommand", motor)

    def test_bno_loss_disarms_and_diagnostics_can_be_disabled(self):
        main = (PROJECT / "main/main.ino").read_text(encoding="utf-8")
        protocol = (COMMON / "RobotProtocol.h").read_text(encoding="utf-8")
        self.assertIn("irTestController.armed() && !input.headingValid", main)
        self.assertIn("BNO lost/reinitializing", main)
        self.assertIn('consumeConsoleWord(p, "DIAG")', protocol)
        self.assertIn('Serial.println(diagnosticsEnabled ? "DIAG ON" : "DIAG OFF")', main)
        self.assertIn("if (diagnosticsEnabled &&", main)

    def test_pwm_floor_preserves_zero_and_sign(self):
        values = [0, 1, 119, 120, 210, -1, -119, -120, -210]
        self.assertEqual(
            floor_nonzero_pwm(values, 120),
            [0, 120, 120, 120, 210, -120, -120, -120, -210],
        )
        self.assertIn("floorMotorCommandValues(values, kIrTestPwmFloor);", (PROJECT / "motor/motor.ino").read_text(encoding="utf-8"))

    def test_main_sends_motion_and_motor_allocates_pwm(self):
        main = (PROJECT / "main/main.ino").read_text(encoding="utf-8")
        motor = (PROJECT / "motor/motor.ino").read_text(encoding="utf-8")
        sender = main.split("bool sendMotorCommand(", 1)[1].split("void pollMotorConsole(", 1)[0]
        self.assertIn("encodeMotorMotionFrame(motorMotion, autoMode, bytes, sizeof(bytes))", sender)
        self.assertIn("encodeMotorCommandFrame(motorConsoleValues, bytes, sizeof(bytes))", sender)
        self.assertIn("encodeMotorCommandFrame(stop, bytes, sizeof(bytes))", sender)
        self.assertNotIn("motorCommandValuesForMotion(", sender)
        self.assertIn("type == kMotorMotionCommand && !allocateMotionPwm(next)", motor)

    @unittest.skipUnless(shutil.which("c++"), "C++ compiler required")
    def test_motion_frame_round_trip(self):
        main = (PROJECT / "main/main.ino").read_text(encoding="utf-8")
        motor = (PROJECT / "motor/motor.ino").read_text(encoding="utf-8")
        start = main.index("uint8_t encodeMotorMotionFrame(")
        encoder = main[start:main.index("bool sendMotorCommand(", start)]
        start = motor.index("bool allocateMotionPwm(")
        allocator = motor[start:motor.index("void receiveCommands(", start)]
        program = '#include "BoardConfig.h"\n#include <cassert>\nusing namespace soccer;\n'
        program += encoder + allocator + """
int main() {
    Motion motion;
    motion.y = 0.5f;
    uint8_t frame[kMotorCommandFrameSize];
    MotorCommandFrameParser motorParser;
    SharedBusIrRequestParser irParser;
    int16_t values[kMotorCommandWheelCount]{};
    uint8_t type = 0;
    const AutoMode modes[] = {kAutoIrTest, kAutoFloor, kAutoWheelLift};
    for (AutoMode mode : modes) {
        assert(encodeMotorMotionFrame(motion, mode, frame, sizeof(frame)) == sizeof(frame));
        assert(frame[1] == 0x05 && frame[4] == 0 && frame[5] == 0x20);
        for (unsigned i = 0; i < sizeof(frame); ++i) {
            assert(!irParser.feed(frame[i], 0));
            assert(motorParser.feed(frame[i], 0, values, type) ==
                   (i + 1 == sizeof(frame) ? kMotorFrameReady : kMotorFramePending));
        }
        assert(irParser.feed(0x01, 0));
        assert(type == kMotorMotionCommand && values[3] == mode);
        assert(allocateMotionPwm(values));
        const int cap = mode == kAutoIrTest ? kIrTestPwmCap :
                        mode == kAutoFloor ? kAutoFloorPwmCap : kAutoWheelLiftPwmCap;
        assert(values[0] == cap && values[1] == -cap && values[2] == cap && values[3] == -cap);
    }
    frame[8] ^= 1;
    for (unsigned i = 0; i < sizeof(frame); ++i)
        assert(motorParser.feed(frame[i], 0, values, type) ==
               (i + 1 == sizeof(frame) ? kMotorFrameInvalid : kMotorFramePending));
    frame[10] ^= frame[8] ^ 0x05;
    frame[8] = 0x05;
    frame[10] ^= 1;  // Repair the previous checksum corruption.
    for (unsigned i = 0; i < sizeof(frame); ++i)
        assert(motorParser.feed(frame[i], 0, values, type) ==
               (i + 1 == sizeof(frame) ? kMotorFrameInvalid : kMotorFramePending));
    assert(motorParser.stats().checksumErrors == 1 && motorParser.stats().rangeErrors == 1);
    motion.y = 1.01f;
    assert(encodeMotorMotionFrame(motion, kAutoIrTest, frame, sizeof(frame)) == 0);
    const int16_t manual[] = {128, -128, 256, 0};
    assert(encodeMotorCommandFrame(manual, frame, sizeof(frame)) == sizeof(frame));
    for (unsigned i = 0; i < sizeof(frame); ++i)
        assert(motorParser.feed(frame[i], 0, values, type) ==
               (i + 1 == sizeof(frame) ? kMotorFrameReady : kMotorFramePending));
    assert(type == kMotorCommandWheelCount);
    for (unsigned i = 0; i < 4; ++i) assert(values[i] == manual[i]);
}
"""
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / "motion-test"
            subprocess.run(
                ["c++", "-std=c++17", "-I", str(COMMON),
                 "-x", "c++", "-o", str(binary), "-"],
                input=program, text=True, check=True,
            )
            subprocess.run([str(binary)], check=True)

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

    def test_wheel_tables_match_zip_based_mapping(self):
        for path in (COMMON / "BoardConfig.h", PROJECT / "line" / "line.ino"):
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
            r"rawAngle\s*=\s*calibratedDirection[\s\S]*?[:]\s*normalizeDegrees\(360\.0f\s*-\s*estimate\.angleDeg\)",
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

    def test_sketches_use_one_shared_configuration(self):
        for name in ("main", "ir", "motor"):
            folder = PROJECT / name
            source = (folder / f"{name}.ino").read_text(encoding="utf-8")
            src = folder / "src"
            self.assertIn('#include "src/BoardConfig.h"', source)
            self.assertIn('#include "src/Rs485.h"', source)
            if src.is_dir():
                self.assertEqual(src.resolve(), COMMON)
            else:
                self.assertEqual(src.read_text(encoding="utf-8").strip().replace("\\", "/"), "../common/src")
            self.assertTrue((COMMON / "BoardConfig.h").is_file())

    def test_main_has_no_camera_uart_code(self):
        source = (PROJECT / "main" / "main.ino").read_text(encoding="utf-8")
        self.assertNotRegex(source, r"(?i)openmv|serial2")


if __name__ == "__main__":
    unittest.main()
