# Current communication protocol

No protocol change was made in this branch. These are source-audited formats; hardware timing and wiring
still require the bench checklist.

| Link | Direction / transport | Frame |
|---|---|---|
| IR | MAIN polls, IR answers on shared half-duplex RS-485, 115200 baud | Raw request `A5 5A 01 FE`; raw response `AA`, 16-bit big-endian angle, 16-bit big-endian detect count, `55`. Filtered request `A5 5A 02 FD`; response is the 9-byte `ir_logic.h` format with CRC-8. |
| MOTOR | MAIN to MOTOR, RS-485, 115200 baud | 12 bytes, `A5` start, command type, sequence, encoded values, CRC-7. Type 3 carries direction/power/turn. Type 4 carries 4 signed PWM values encoded with offset 210 (valid encoded range 0..420). |
| Camera | OpenMV to MAIN, UART | Legacy 8-byte packet at 9600 baud when `PROTOCOL=1`; extended 32-byte protocol version 3 at 115200 when `PROTOCOL=2`. MAIN validates framing/checksum and field values. |
| MOTOR feedback | MOTOR to MAIN | No reply frame exists. Do not infer received/applied MOTOR state from MAIN transmit diagnostics. |

MAIN is the only RS-485 request initiator. IR only drives the bus after a complete valid request and returns
to receive mode after its response. Adding a MOTOR reply would create a second responder and has not been
implemented because collision timing, transaction order and legacy compatibility are not validated.

## Failure handling already in code

MAIN ages IR and camera data. MOTOR rejects malformed/CRC-invalid/out-of-range packets and stops after its
receive watchdog expires. The IR request parser only samples on a complete recognized request. UART frame
parsers handle bytewise input; interoperability under dropped/partial frames remains a bench test.

## Compatibility

Build MAIN, IR and MOTOR from the same commit. The protocol itself is unchanged, but software route
compatibility still depends on `ROBOT_UPGRADE`, `ROBOT_RAW_IR`, `ROBOT_ZIP_TRACK`, `ROBOT_ZIP_COMPAT`,
MOTOR `ROBOT_UPGRADE`, and OpenMV `PROTOCOL`. These are not negotiated on the wire. Check the build ID and
settings locally on each board before a bench run.
