"""Windows serial trial: hold space for a single short, supervised run."""
import argparse
import ctypes
import time

import serial

parser = argparse.ArgumentParser()
parser.add_argument("port", help="MAIN USB serial port, e.g. COM5")
parser.add_argument("mode", choices=("p", "y", "b"), help="PD chase, yellow goal, or blue goal")
args = parser.parse_args()

with serial.Serial(args.port, 115200, timeout=0, write_timeout=0.2) as port:
    try:
        for command in (b"s", args.mode.encode()):
            port.write(command)
            port.flush()
            time.sleep(0.15)
        print("Release SPACE, then hold it to run for after a 2-second wait, for up to 1.8 seconds; release to stop.", flush=True)
        while ctypes.windll.user32.GetAsyncKeyState(0x20) & 0x8000:
            time.sleep(0.02)
        while not ctypes.windll.user32.GetAsyncKeyState(0x20) & 0x8000:
            time.sleep(0.02)
        port.write(b"g")
        port.flush()
        start = time.monotonic()
        while time.monotonic() - start < 3.8 and ctypes.windll.user32.GetAsyncKeyState(0x20) & 0x8000:
            time.sleep(0.05)
    finally:
        port.write(b"s")
        port.flush()
