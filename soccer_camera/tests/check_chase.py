#!/usr/bin/env python3
"""Check the deployed IR/MOTOR sketches and board pin assignments against the tested ZIP."""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ChaseConfiguration(unittest.TestCase):
    def test_ir_and_motor_match_tested_zip(self):
        for board in ("ir", "motor"):
            with self.subTest(board=board):
                self.assertEqual((ROOT / board / f"{board}.ino").read_bytes(),
                                 (ROOT / "chase_verified" / board / f"{board}.ino").read_bytes())

    def test_main_board_pins_and_no_auto_command(self):
        source = (ROOT / "main/main.ino").read_text(encoding="utf-8")
        for name, value in (("PIN_RS485_TX", 0), ("PIN_RS485_RX", 1), ("PIN_RS485_EN", 8)):
            self.assertRegex(source, rf"\b{name}\s*=\s*{value}\s*;")
        self.assertRegex(source, r"Serial2\.setTX\(2\)")
        self.assertRegex(source, r"Serial2\.setRX\(3\)")
        self.assertNotRegex(source, re.compile(r"\bAUTO\b|\bARM\b", re.I))

    def test_kicker_output_is_initialized_low(self):
        source = (ROOT / "main/main.ino").read_text(encoding="utf-8")
        self.assertRegex(source, r"digitalWrite\(11,\s*LOW\);\s*pinMode\(11,\s*OUTPUT\);")
        self.assertNotRegex(source, r"digitalWrite\(11,\s*HIGH\)")


if __name__ == "__main__":
    unittest.main()
