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

At the audited baseline, MAIN/MOTOR compile with `ROBOT_UPGRADE=1`, `ROBOT_ZIP_TRACK=0`,
`ROBOT_ZIP_COMPAT=0`; OpenMV uses `PROTOCOL=2`. That selects the expanded MAIN path on boot. Its current
software tests do not certify hardware. `ZipChasePD` is the saved output reference; the new filtered
candidate is disabled by default. Search, orbit and attack in `brain::Brain` are compile-gated and default
off; direct approach/alignment remains available. `soccer_ir_main_line`, `chase_*`, and `legacy_before_chase` are separate
historical/test sources, not the documented current flash target.

## Single-controller limits

`brain::Brain::step()` returns one tactic/output object; `upgradeLoop()` sends that output. The top-level
loop selects a single compile-time path. USB runtime mode switching is not implemented yet. Separate
diagnostic modes must not be assumed to exist merely because mode names are in `RuntimeConfig.h`.
