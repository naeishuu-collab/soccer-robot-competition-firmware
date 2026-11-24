#!/usr/bin/env python3
"""Check the deployed IR and MAIN pin assignments and command policy."""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ChaseConfiguration(unittest.TestCase):
    def test_ir_sensor_and_bus_pins(self):
        source = (ROOT / "ir/ir.ino").read_text(encoding="utf-8")
        pins = re.search(r"sensorPins\[SENSOR_COUNT\]\s*=\s*\{([^}]+)\}", source)
        self.assertIsNotNone(pins)
        self.assertEqual([int(n) for n in re.findall(r"\d+", pins.group(1))], list(range(18, 30)))
        for name, value in (("RS485_TX_PIN", 0), ("RS485_RX_PIN", 1), ("RS485_DE_PIN", 2)):
            self.assertRegex(source, rf"\b{name}\s*=\s*{value}\s*;")

    def test_main_board_pins_and_no_auto_command(self):
        source = (ROOT / "main/main.ino").read_text(encoding="utf-8")
        for name, value in (("PIN_RS485_TX", 0), ("PIN_RS485_RX", 1), ("PIN_RS485_EN", 8)):
            self.assertRegex(source, rf"\b{name}\s*=\s*{value}\s*;")
        self.assertRegex(source, r"Serial2\.setTX\(2\)")
        self.assertRegex(source, r"Serial2\.setRX\(3\)")
        self.assertRegex(source, r"IR_FRONT_DEGREES\s*=\s*0\s*;")
        self.assertRegex(source, r"IR_ANGLE_SIGN\s*=\s*-1\s*;")
        self.assertNotRegex(source, re.compile(r"\bAUTO\b|\bARM\b", re.I))

    def test_kicker_output_is_initialized_low(self):
        source = (ROOT / "main/main.ino").read_text(encoding="utf-8")
        self.assertRegex(source, r"digitalWrite\(11,\s*LOW\);\s*pinMode\(11,\s*OUTPUT\);")
        self.assertNotRegex(source, r"digitalWrite\(11,\s*HIGH\)")


if __name__ == "__main__":
    unittest.main()
