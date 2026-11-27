# Bench and hardware test guide (no flash performed)

## Before powering motors

1. Confirm the tournament's 2026 local rule set, robot dimensions/mass, ball-capture geometry and allowed IR hardware. The international reference is linked in `BASELINE_AUDIT_2026-11-27_JA.md`.
2. Label MAIN, IR, MOTOR and camera with board role, firmware SHA and config ID. Check all builds use the same commit and compatible route settings.
3. Raise all four wheels. Keep a reachable power disconnect. Verify no kicker/dribbler automatic output is enabled.
4. Use 115200 baud for MAIN/IR/MOTOR USB serial. OpenMV diagnostics use the OpenMV USB console. Do not connect a second RS-485 transmitter.
5. Run `tests/compile_arduino.ps1` first. It compiles MAIN RP2354B, IR RP2354A, LINE RP2354B (compile-only), and MOTOR RP2354A. It does not upload.

## Test order

1. **MOTOR isolated:** with MAIN not transmitting, use MOTOR `i`+Enter. For the existing `diag` command, keep the wheels raised and follow the individual wheel controls in `UPGRADE_2026-11-26_JA.md`. Confirm the stop/timeout behavior before lowering wheels. `APPLIED` is a bridge command, not measured rotation.
2. **IR isolated:** send the existing MAIN polling build only after wiring is checked. IR `i` shows the latest 12 sensor counters and request/reply totals; move the ball around the sensor ring and compare expected bearing.
3. **OpenMV isolated:** enable `DEBUG_STATS=True`; use `DEBUG_FORMAT="CSV"` only when capturing records. Check camera `ready`, setup reason, FPS, stage timing, line mask/escape, yellow/blue bearings, UART partial/failure counters. Verify view and mirror orientation with wheels raised. Do not tune calibration from synthetic PC images.
4. **Links:** MAIN `i` reports IR replies/timeouts, camera packet errors/gaps and transmitted command; MOTOR `i` reports received sequence, CRC/BAD/timeout, target/current/applied outputs. Correlate timestamps over USB. MAIN cannot observe MOTOR state over the current protocol.
5. **Stop injection:** disconnect/stop one source at a time with wheels raised. Verify stale camera/IR leads to MAIN zero command where required and loss of MAIN frames leads to MOTOR timeout stop. Restore links and verify an explicit MAIN `g` only after the source is healthy.
6. **Chase:** use a flat, bounded, line-safe test area only after the above pass. Start at low-duration exposure with a reachable disconnect. The ZIP reference path requires fresh camera safety frames; camera-less tracking is not a competition mode.
7. **Line and goal:** record physical camera images and per-direction line decisions at measured locations. Test straight/diagonal/multiple-edge cases and yellow/blue goals in match lighting. Only then consider enabling experimental features.

## Stop and recovery

Use MAIN `d` or `s` for a soft stop. MOTOR watchdog behavior is independent. If a command stream remains ambiguous, remove motor power first; do not troubleshoot by changing GPIO, wheel order, polarity, protocol baud or camera exposure. Save all four boards' snapshots before resetting so the last status is retained in the USB logs.

All actual driving, threshold calibration, PWM response, RS-485 collision behavior, OpenMV FPS and rule compliance are `PENDING_HARDWARE` until recorded on the target hardware.
