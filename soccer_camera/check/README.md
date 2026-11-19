# RS485 RX/TX check

These are standalone Arduino sketches with no project `.h` files. The TX sketch sends a numbered probe only after USB serial receives `T`; `S` stops it. The RX sketch only echoes a correctly formatted probe. A successful exact echo checks both TX and RX on both boards.

Defaults follow the firmware pin map: TX sketch on MAIN uses GP0/GP1/DE GP8; RX sketch on MOTOR uses GP0/GP1/DE GP2. IR/MOTOR use DE GP2, LINE uses GP27, and MAIN uses GP8 in the current configuration. Physical pin continuity is not verified; change `kRs485DePin` for the board being tested and confirm its actual wiring first. Both sketches use 115200 baud and active-high DE.

Connect only the test transmitter and receiver to the RS485 link during this test, with A/B polarity and signal ground checked against the transceiver boards. Keep motor power disconnected. Open the TX USB serial monitor at 115200, send `T`, and confirm each probe is echoed as `PASS`; `S` stops transmission. The RX USB serial monitor shows accepted probes and echo activity. A timeout points to wiring, DE, baud, or A/B polarity; it does not identify which one.

Compile without uploading:

```powershell
arduino-cli compile --fqbn rp2040:rp2040:generic_rp2350:variantchip=RP2350A,flash=2097152_0 check/rs485_tx_check
arduino-cli compile --fqbn rp2040:rp2040:generic_rp2350:variantchip=RP2350A,flash=2097152_0 check/rs485_rx_check
```
