# Phase 9 — MOTOR and transport coverage

No RS-485 frame layout, baud rate, GPIO, wheel order or motor polarity was changed. MAIN remains the sole
RS-485 initiator; IR only responds to its poll. MOTOR continues to validate type/CRC/range, reject invalid
commands, wait before a reversal, ramp within its existing policy, and stop after the receive watchdog.
There is no MOTOR-to-MAIN response; two USB consoles are required for transmit/receive comparison.

Host tests in `test_chase_motor.cpp`, `test_chase_mixer.cpp`, `test_motor_model.cpp`, core and integration
cover direct signed PWM, wheel mapping, bounds, reversal and timeout behavior. All Arduino sketches
compile without upload. These checks do not establish physical RPM, braking distance or RS-485 turnaround
timing. No new global PWM cap was added; ZIP reference remains 175 nominal / 140 nonzero minimum / 210
maximum. Phase 9 is `PARTIAL`; bus timing and wheel response are `PENDING_HARDWARE`.
