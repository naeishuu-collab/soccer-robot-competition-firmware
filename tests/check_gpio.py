#!/usr/bin/env python3
"""Check package ranges, configured line faults, and reported-versus-unknown board routes."""
import json
import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG = json.loads((ROOT / "config/gpio_profiles.json").read_text(encoding="utf-8"))
CUSTOM = CONFIG["custom_rp2354"]
PICO2 = CONFIG["standard_pico2_reference"]
LEGACY_MOTOR = CONFIG["legacy_reference"]["MOTOR"]
LINE_FAULTS = CONFIG["line_sensor_faults"]
MOTOR_REPORTED = CONFIG["motor_user_reported"]


def node_pins(node):
    pins = []
    for key, value in node.items():
        if isinstance(value, int) and value >= 0:
            pins.append((value, key))
        elif isinstance(value, list):
            pins.extend((pin, f"{key}[{i}]") for i, pin in enumerate(value) if isinstance(pin, int) and pin >= 0)
    return pins


def validate_profile(profile):
    errors = []
    package_counts = profile.get("chip_gpio_count", {})
    for node_name, node in profile["nodes"].items():
        seen = {}
        maximum = package_counts.get(node.get("chip"), 0) - 1
        for pin, signal in node_pins(node):
            if maximum < 0 or pin > maximum:
                errors.append(f"{node_name}: GPIO{pin} is outside {node.get('chip', 'unknown')} package range")
            if pin in seen:
                errors.append(f"{node_name}: GPIO{pin} collision ({seen[pin]} and {signal})")
            seen[pin] = signal
    return errors


def validate_motor_routing(node):
    errors = []
    reserved = {node.get("UART_TX"), node.get("UART_RX"), node.get("RS485_DE")}
    shared_de = node.get("DE_SHARED_INPUT_GPIO")
    if isinstance(shared_de, str) and shared_de.isdigit():
        reserved.add(int(shared_de))
    pwm_a = node.get("PWM_A", [])
    pwm_b = node.get("PWM_B", [])
    if len(pwm_a) != 4 or len(pwm_b) != 4:
        return ["MOTOR: expected four PWMh/PWMl channel pairs"]
    used = set()
    for channel, pin in enumerate(pwm_a + pwm_b):
        if not isinstance(pin, int) or pin < 0 or pin >= 30:
            errors.append(f"MOTOR: PWM GPIO {pin} is outside RP2354A package range")
        elif pin in reserved:
            errors.append(f"MOTOR: PWM GPIO{pin} conflicts with a reserved RS485 pin")
        if isinstance(pin, int) and pin >= 0:
            if pin in used:
                errors.append(f"MOTOR: duplicate PWM GPIO{pin}")
            used.add(pin)
    return errors


def valid_motor_legacy_reference(record):
    old = record.get("old_reference_only", {})
    return (record.get("status") == "LEGACY_REFERENCE" and
            record.get("use_as_current_configuration") is False and
            len(old.get("PWM_A", [])) == 4 and len(old.get("PWM_B", [])) == 4)


def custom_route_status(_node, _pin):
    # Chip bonding is not carrier routing; only the actual schematic/netlist can resolve this.
    return "UNKNOWN"


def standard_pico2_status(pin):
    if pin in PICO2["exposed_gpio"]:
        return "HEADER_GPIO"
    if str(pin) in PICO2["board_functions"]:
        return "BOARD_FUNCTION"
    return "NOT_HEADER_GPIO"


def extract_array(path, symbol):
    source = path.read_text(encoding="utf-8")
    match = re.search(rf"\b{re.escape(symbol)}\s*\[[^]]*\]\s*=\s*\{{([^}}]*)\}}", source, re.S)
    if not match:
        raise ValueError(f"Could not find {symbol} in {path}")
    return [int(value) for value in re.findall(r"-?\d+", match.group(1))]


def extract_bool_array(path, symbol):
    source = path.read_text(encoding="utf-8")
    match = re.search(rf"\b{re.escape(symbol)}\s*\[[^]]*\]\s*=\s*\{{([^}}]*)\}}", source, re.S)
    if not match:
        raise ValueError(f"Could not find {symbol} in {path}")
    return [value == "true" for value in re.findall(r"\b(?:true|false)\b", match.group(1))]


def extract_motor_driver_pins(path):
    source = path.read_text(encoding="utf-8")
    match = re.search(r"\bkDriver\s*\[4\]\s*=\s*\{(.*?)\};", source, re.S)
    if not match:
        raise ValueError(f"Could not find kDriver[4] in {path}")
    return [(int(a), int(b)) for a, b in re.findall(
        r"\{\s*(-?\d+)\s*,\s*(-?\d+)(?:\s*,[^}]*)?\s*\}", match.group(1))]


def extract_gpio_mask(source, symbol):
    match = re.search(rf"\b{re.escape(symbol)}\s*=\s*(.*?);", source, re.S)
    if not match:
        raise ValueError(f"Could not find {symbol}")
    return sum(1 << int(pin) for pin in re.findall(r"1u\s*<<\s*(\d+)", match.group(1)))


def verify_active_sources():
    ir_copy = ROOT / "ir/ir.ino"
    config = ROOT / "common/src/BoardConfig.h"
    expected = CUSTOM["nodes"]
    errors = []
    if extract_array(ir_copy, "sensorPins") != expected["IR"]["SENSORS"]:
        errors.append("IR sensor map differs from the preserved configuration")
    if extract_array(config, "kLineSensorPins") != expected["LINE"]["SENSORS"]:
        errors.append("LINE sensor map differs from the project GPIO2..25 mapping")
    enabled_indices = LINE_FAULTS["enabled_indices"]
    if extract_bool_array(config, "kLineSensorEnabled") != [i in enabled_indices for i in range(24)]:
        errors.append("LINE enable mask differs from the 18 user-reported working sensors")
    ir_source = ir_copy.read_text(encoding="utf-8")
    if "estimateIrDirection(" not in ir_source or "calculatePrediction()" not in ir_source or "predictIr(" not in ir_source:
        errors.append("IR direction confidence and trend prediction must remain present")
    if "directionCoherence" not in ir_source or "lastSensorSampleMs" not in ir_source:
        errors.append("IR confidence/freshness diagnostics must remain present")
    debug_start = ir_source.find("void printDebug()")
    setup_start = ir_source.find("// SETUP", debug_start)
    debug_source = ir_source[debug_start:setup_start] if debug_start >= 0 and setup_start >= 0 else ""
    if len(re.findall(r"Serial\.availableForWrite\(\)\s*>=\s*128", debug_source)) != 4:
        errors.append("IR debug output must skip each chunk unless its USB transmit buffer has room")
    if not re.search(r"if\s*\(\s*Serial\s*&&\s*Serial\.availableForWrite\(\)\s*>=\s*64\s*\)", ir_source):
        errors.append("IR startup diagnostics must not wait for an unread USB console")

    source = config.read_text(encoding="utf-8")
    main_source = (ROOT / "main/main.ino").read_text(encoding="utf-8")
    line_source = (ROOT / "line/line.ino").read_text(encoding="utf-8")
    motor_source = (ROOT / "motor/motor.ino").read_text(encoding="utf-8")
    motor_channels = MOTOR_REPORTED["channels"]
    expected_driver = [(motor_channels[name]["IN1"], motor_channels[name]["IN2"])
                       for name in MOTOR_REPORTED["channel_order"]]
    if extract_motor_driver_pins(ROOT / "motor/motor.ino") != expected_driver:
        errors.append("MOTOR driver table differs from the user-reported current-revision mapping")
    if "board::kLineSensorPins[i]" not in line_source or "board::kLineSensorEnabledMask" not in line_source:
        errors.append("LINE input scan/calculation must use the shared pin map and failed-sensor mask")
    if "board::kLineSensorEnabledMask" not in main_source:
        errors.append("MAIN must validate LINE packets with the same failed-sensor mask")
    if "kBnoOffsetsVerified = false" not in source or not re.search(r"if\s*\(\s*board::kBnoOffsetsVerified\s*\)\s*devices->sensor->setSensorOffsets\(offsets\);", main_source):
        errors.append("unverified BNO055 offsets must remain disabled until calibrated")
    for header in ("RobotControl.h", "Rs485.h"):
        local_header = ROOT / "motor" / header
        common_header = ROOT / "common/src" / header
        if local_header.read_bytes() != common_header.read_bytes():
            errors.append(f"MOTOR sketch-local {header} differs from the tested common header")
    constants = {
        "kMainRs485TxPin": expected["MAIN"]["UART_TX"], "kMainRs485RxPin": expected["MAIN"]["UART_RX"],
        "kMainRs485DePin": expected["MAIN"]["RS485_DE"], "kIrRs485TxPin": expected["IR"]["UART_TX"],
        "kIrRs485RxPin": expected["IR"]["UART_RX"], "kIrRs485DePin": expected["IR"]["RS485_DE"],
        "kLineRs485TxPin": expected["LINE"]["UART_TX"], "kLineRs485RxPin": expected["LINE"]["UART_RX"],
        "kLineRs485DePin": expected["LINE"]["RS485_DE"], "kMotorRs485TxPin": -1,
        "kMotorRs485RxPin": -1, "kMotorRs485DePin": -1, "kMotorRs485SharedDePin": -1,
        "kBnoSdaPin": expected["MAIN"]["BNO_SDA"], "kBnoSclPin": expected["MAIN"]["BNO_SCL"],
    }
    for name, value in constants.items():
        match = re.search(rf"constexpr\s+int8_t\s+{name}\s*=\s*(-?\d+)", source)
        if not match or int(match.group(1)) != value:
            errors.append(f"{name} differs from the verified profile or must remain UNKNOWN")
    for name, value in (("kMotorTestIn1CandidatePin", 13), ("kMotorTestIn2CandidatePin", 12)):
        match = re.search(rf"constexpr\s+int8_t\s+{name}\s*=\s*(-?\d+)", source)
        if not match or int(match.group(1)) != value:
            errors.append(f"{name} must preserve the user-reported test candidate only")
    for setting in ("kLinePolarityConfigured", "kLineFrontOffsetConfigured", "kLineAnglesConfigured", "kMotorHardwareConfigured",
                    "kCompetitionArmed", "kKickerHardwareConfigured", "kKickerRequestPolicyConfigured",
                    "kKickerHeadingWindowConfigured", "kDribblerHardwareConfigured",
                    "kMotorRs485PinsVerified"):
        if not re.search(rf"constexpr\s+bool\s+{setting}\s*=\s*false", source):
            errors.append(f"safety/configuration gate {setting} must remain false until confirmed")
    for name in ("kMotorArmPin", "kMotorEStopPin", "kMotorClearFaultPin",
                 "kMainArmPin", "kMainEStopPin", "kMainClearFaultPin"):
        if not re.search(rf"constexpr\s+int8_t\s+{name}\s*=\s*-1", source):
            errors.append(f"unverified safety input {name} must stay unassigned")
    if not re.search(r"constexpr\s+uint32_t\s+kMotorPwmFrequencyHz\s*=\s*kTb67h450PwmFrequencyHz", source):
        errors.append("MOTOR PWM frequency must match the user-reported 20 kHz test")
    if not re.search(r"constexpr\s+bool\s+kLineDebounceConfigured\s*=\s*false", source):
        errors.append("LINE debounce stays disabled until the physical edge timing is measured")
    if "lineDebouncer.update(enabledState, board::kLineDebounceSamples)" not in line_source:
        errors.append("LINE scan must pass through the configurable debounce layer")
    if not re.search(r"constexpr\s+uint8_t\s+kMotorGpioCount\s*=\s*0", source):
        errors.append("MOTOR package count must stay unknown until the complete package/route map is verified")
    if "constexpr bool kMotorSafetyInputBiasVerified = false" not in source or \
       "constexpr bool kMainSafetyInputBiasVerified = false" not in source:
        errors.append("safety inputs require verified fail-safe electrical bias")
    if "kickerRequestAllowed(board::kKickerRequestPolicyConfigured" not in main_source or \
       "kickerController.update(kickerInput, nowMs)" not in main_source:
        errors.append("MAIN must route its gated kick policy through the safe KickerController")
    try:
        if "MOTOR" in expected:
            errors.append("LEGACY_REFERENCE MOTOR data must not appear in active nodes")
        if not valid_motor_legacy_reference(LEGACY_MOTOR):
            errors.append("MOTOR old design must be explicitly classified LEGACY_REFERENCE")
        if validate_motor_routing(LEGACY_MOTOR["old_reference_only"]):
            errors.append("LEGACY_REFERENCE motor route fixture is internally inconsistent")
        expected_pwm_mask = sum(1 << pin for pair in expected_driver for pin in pair)
        if extract_gpio_mask(source, "kMotorPwmPinMask") != expected_pwm_mask:
            errors.append("MOTOR PWM whitelist differs from the user-reported current-revision mapping")
        if extract_gpio_mask(source, "kMotorReservedPinMask") != 0:
            errors.append("MOTOR reserved pins must not be inferred from legacy design data")
        if not re.search(r"constexpr\s+uint64_t\s+kMotorSafetyReservedPinMask\s*=\s*kMotorPwmPinMask", source):
            errors.append("MOTOR safety inputs must not reuse a mapped motor output pin")
        if not re.search(r"constexpr\s+bool\s+kWheelsConfigured\s*=\s*false", motor_source):
            errors.append("MOTOR wheel geometry and polarity must remain unconfigured")
        if "if (motorRs485PinsReady())" not in motor_source:
            errors.append("MOTOR UART setup and receive must remain behind the verified-pin gate")
        if "tb67h450OutputForWheel" not in motor_source or "kTb67h450ReversalDwellMs" not in motor_source:
            errors.append("MOTOR output must use the TB67H450 adapter and 200 ms reversal dwell")
        if "gpio_put_masked64" not in motor_source or "selectDriverSioLow(channel)" not in motor_source:
            errors.append("MOTOR brake transitions must preload both SIO outputs low and switch them together")
        if "commandPriority.accept(next, enable)" not in motor_source:
            errors.append("MOTOR must retain STOP priority until its output tick")
    except ValueError as error:
        errors.append(str(error))
    sketches = (ROOT / "main/main.ino", ROOT / "ir/ir.ino", ROOT / "line/line.ino", ROOT / "motor/motor.ino")
    if any(re.search(r"SerialPIO|\bsendRs485\s*\(", path.read_text(encoding="utf-8")) for path in sketches):
        errors.append("competition sketches must use the common hardware-UART RS485 driver")
    return errors


class GpioCheckerTests(unittest.TestCase):
    def test_custom_rp2354_package_ranges_have_no_collisions(self):
        self.assertEqual(validate_profile(CUSTOM), [])

    def test_custom_carrier_routing_stays_unknown(self):
        self.assertEqual(custom_route_status("IR", 29), "UNKNOWN")
        self.assertEqual(custom_route_status("LINE", 25), "UNKNOWN")

    def test_standard_pico2_is_a_separate_reference(self):
        self.assertNotIn(29, PICO2["exposed_gpio"])
        self.assertEqual(standard_pico2_status(29), "BOARD_FUNCTION")
        self.assertEqual(custom_route_status("IR", 29), "UNKNOWN")

    def test_checker_catches_collision_and_package_range(self):
        profile = {"chip_gpio_count": {"RP2354A": 30}, "nodes": {"MAIN": {"chip": "RP2354A", "TX": 2, "DE": 2, "BAD": 30}}}
        errors = validate_profile(profile)
        self.assertTrue(any("collision" in error for error in errors))
        self.assertTrue(any("outside RP2354A package range" in error for error in errors))

    def test_motor_legacy_reference_is_not_active_routing(self):
        self.assertNotIn("MOTOR", CUSTOM["nodes"])
        self.assertTrue(valid_motor_legacy_reference(LEGACY_MOTOR))
        self.assertEqual(validate_motor_routing(LEGACY_MOTOR["old_reference_only"]), [])
        self.assertEqual(extract_motor_driver_pins(ROOT / "motor/motor.ino"), [(13, 12), (11, 10), (9, 8), (7, 6)])

    def test_reported_motor_map_is_unique_and_still_gated(self):
        pins = [pin for pair in extract_motor_driver_pins(ROOT / "motor/motor.ino") for pin in pair]
        self.assertEqual(len(pins), 8)
        self.assertEqual(len(set(pins)), 8)
        config = (ROOT / "common/src/BoardConfig.h").read_text(encoding="utf-8")
        motor = (ROOT / "motor/motor.ino").read_text(encoding="utf-8")
        self.assertIn("kMotorHardwareConfigured = false", config)
        self.assertIn("kWheelsConfigured = false", motor)

    def test_line_fault_mask_matches_reported_indices(self):
        enabled = extract_bool_array(ROOT / "common/src/BoardConfig.h", "kLineSensorEnabled")
        self.assertEqual(len(enabled), 24)
        self.assertEqual([i for i, value in enumerate(enabled) if not value], LINE_FAULTS["failed_indices"])

    def test_candidate_line_angles_match_code_and_remain_disabled(self):
        expected = [11, 21, 41, 51, 71, 81, 101, 111, 131, 141, 161, 171,
                    191, 201, 221, 231, 251, 261, 281, 291, 311, 321, 341, 351]
        config = ROOT / "common/src/BoardConfig.h"
        self.assertEqual(extract_array(config, "kLineBoardAngleCdeg"), [angle * 100 for angle in expected])
        source = config.read_text(encoding="utf-8")
        self.assertIn("kLineFrontOffsetConfigured = false", source)
        self.assertIn("kLineAnglesConfigured = false", source)
        self.assertIn("kLinePolarityConfigured = false", source)

    def test_ir_console_debug_skips_short_usb_buffers(self):
        source = (ROOT / "ir/ir.ino").read_text(encoding="utf-8")
        debug = source[source.index("void printDebug()"):source.index("// SETUP")]
        self.assertEqual(len(re.findall(r"Serial\.availableForWrite\(\)\s*>=\s*128", debug)), 4)
        self.assertIn("if (!Serial) return;", debug)

    def test_motor_gpio25_and_rs485_pins_are_rejected_for_pwm(self):
        node = {"UART_TX": 0, "UART_RX": 1, "RS485_DE": 2, "DE_SHARED_INPUT_GPIO": "25",
                "PWM_A": [25, 13, 11, 9], "PWM_B": [6, 12, 10, 8]}
        errors = validate_motor_routing(node)
        self.assertTrue(any("GPIO25 conflicts" in error for error in errors))
        node["PWM_A"] = [7, 13, 11, 9]
        node["PWM_B"] = [2, 12, 10, 8]
        errors = validate_motor_routing(node)
        self.assertTrue(any("GPIO2 conflicts" in error for error in errors))

    def test_active_source_maps_and_motor_headers_are_consistent(self):
        self.assertEqual(verify_active_sources(), [])

def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--self-test":
        result = unittest.TextTestRunner(stream=sys.stdout).run(
            unittest.defaultTestLoader.loadTestsFromTestCase(GpioCheckerTests))
        if not result.wasSuccessful():
            raise SystemExit(1)
        return
    source_errors = verify_active_sources()
    if source_errors:
        print("FAIL source/GPIO check: " + "; ".join(source_errors))
        raise SystemExit(1)
    errors = validate_profile(CUSTOM)
    if errors:
        print("FAIL package/collision check: " + "; ".join(errors))
        raise SystemExit(1)
    print("PASS IR coherence/prediction; 18 LINE sensors enabled and 6 failed sensors excluded")
    print("PASS project IR/LINE pin maps; physical continuity and LINE angles still need measurement")
    print("PASS user-reported MOTOR PWM map matches firmware; RS485/safety routes are unknown and output stays disabled")
    print("LEGACY_REFERENCE: old MOTOR design remains separate from the user-reported current-revision map")


if __name__ == "__main__":
    main()

