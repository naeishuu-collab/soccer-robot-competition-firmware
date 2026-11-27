# Firmware architecture

## Board responsibilities

- **IR (`soccer_camera/ir/ir.ino`)** samples the 12 receiver inputs, calculates the ball bearing/strength and answers MAIN's polled RS-485 request. It does not select tactics or drive motors.
- **OpenMV (`soccer_camera/openmv/goal_wall_navigation.py`, `competition_vision.py`)** samples the omnidirectional image, determines observed floor/line sectors and yellow/blue goal bearings, attaches freshness/status, and sends a UART frame. It does not issue motor commands.
- **MAIN (`soccer_camera/main/main.ino`, `main_logic.h`, `zip_chase_pd.h`)** polls IR, receives camera frames, rejects stale/unsafe sensor states, chooses a motion/state, and emits the final drive frame. The ZIP reference PD remains separate from the expanded `brain::Brain` route.
- **MOTOR (`soccer_camera/motor/motor.ino`, `motor_logic.h`)** validates commands, mixes 4-wheel PWM, enforces reverse waiting/ramp/watchdog stop and reports locally over USB. It has no tactics.

## Data and call path

```text
IR sensors -> IR sample/angle -> polled RS-485 response -> MAIN receiver
OpenMV image -> floor scan + goal tracker -> UART packet -> MAIN camera parser
MAIN sensor freshness + safety mask -> ZIP PD or brain::Brain::step()
 -> one MAIN sendDrive/sendPwm path -> RS-485 command -> MOTOR frame validation
 -> wheel mixer/reverse protection/ramp -> H-bridge PWM command
```

`MOTOR` does not return status to MAIN. USB snapshots on each board are the only current way to compare
MAIN transmit values with MOTOR receive/target/applied command values. Applied PWM is not wheel-speed feedback.

## Active versus reference code

At the requested baseline, MAIN/MOTOR compile with `ROBOT_UPGRADE=1`, `ROBOT_ZIP_TRACK=0`,
`ROBOT_ZIP_COMPAT=0`; OpenMV uses `PROTOCOL=2`. This change makes `ROBOT_ZIP_TRACK=1` the default,
so the top-level MAIN loop selects `chaseLoop()` before the expanded `upgradeLoop()`. The filtered PD
candidate remains disabled. `ZipChasePD` is checked against an independent 10,000-input transcription;
the original ZIP file is not checked in, so source equivalence is not established. Software tests do not
certify hardware. `soccer_ir_main_line`, `chase_*`, and `legacy_before_chase` remain historical/test sources.

The active chase path now allows a fresh camera-confirmed 0°/180° line escape without an IR reply.
Other escape bearings still stop. See [STOP_CONDITIONS.md](STOP_CONDITIONS.md) for exact state gates and
the conservative `zipPathClear()` rule.

## Single-controller limits

`brain::Brain::step()` returns one tactic/output object; `upgradeLoop()` sends that output. The top-level
loop selects a single compile-time path. USB runtime mode switching is not implemented yet. Separate
diagnostic modes must not be assumed to exist merely because mode names are in `RuntimeConfig.h`.
