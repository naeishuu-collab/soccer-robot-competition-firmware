"""Windows BASIC_CHASE: hold space for a single short, supervised run."""
import argparse
import ctypes
import time

import serial

parser = argparse.ArgumentParser()
parser.add_argument("port", help="MAIN USB serial port, e.g. COM5")
parser.add_argument("stage", choices=("1", "2", "3"), help="PWM cap: 140, 175, 210")
args = parser.parse_args()

with serial.Serial(args.port, 115200, timeout=0, write_timeout=0.2) as port:
    try:
        for command in (b"s", args.stage.encode(), b"g"):
            port.write(command)
            port.flush()
            time.sleep(0.15)
        print("Release SPACE, then hold it to run for up to 1.8 seconds; release to stop.", flush=True)
        while ctypes.windll.user32.GetAsyncKeyState(0x20) & 0x8000:
            time.sleep(0.02)
        while not ctypes.windll.user32.GetAsyncKeyState(0x20) & 0x8000:
            time.sleep(0.02)
        start = time.monotonic()
        while time.monotonic() - start < 1.8 and ctypes.windll.user32.GetAsyncKeyState(0x20) & 0x8000:
            port.write(b"R")
            port.flush()
            time.sleep(0.05)
    finally:
        port.write(b"s")
        port.flush()
