# Test modes

## Implemented controls

`RuntimeConfig.h` defines the named mode enum, parser and `ModeGate`. Host tests verify that mode selection
disarms motion, invalidates previous sensor freshness, increments the generation, and requires a fresh
sensor acknowledgement plus explicit resume. Invalid config or interlocks deny motion.

The gate is **not yet wired to the MAIN USB command loop**. No `mode ...` or `resume` serial command exists.
Do not use the enum as evidence that a bench mode can drive the physical robot.

| Requested mode | State |
|---|---|
| STOP | Existing MAIN `d` or `s` soft stop; `g` resumes the current firmware route |
| MOTOR_TEST | Existing MOTOR `diag` command, bench only; keep wheels raised |
| COMM_TEST | Partial: inspect each board's local USB diagnostics; no automatic end-to-end mode |
| IR_TEST | Existing IR `i` snapshot and `v` continuous output |
| CAMERA_TEST | OpenMV `DEBUG_STATS=True`, `VISUALIZE=True` only on the bench |
| CHASE_ONLY | ZIP PD build route exists behind `ROBOT_ZIP_TRACK`; still requires fresh camera safety data |
| LINE_TEST | OpenMV diagnostics; no independent MAIN mode command |
| SEARCH_TEST | Not implemented |
| GOAL_TEST | OpenMV goal diagnostics/overlay; no MAIN mode command |
| ORBIT_TEST | Not implemented |
| ATTACK_TEST | Not implemented |
| COMPETITION | Current compile-time firmware route; not hardware validated |

## Commands that exist today

- MAIN USB: `d` or `s` stops, `g` resumes, `i` prints diagnostics, `v` toggles viewer telemetry.
- IR USB: `i` prints a snapshot, `v` toggles continuous filtered diagnostics.
- MOTOR USB: enter `i` then newline for status; existing `diag` command starts/stops the local motor test.
- OpenMV: set `DEBUG_STATS`, `DEBUG_FORMAT`, `VISUALIZE`, and `GOAL_DEBUG` in `camera_calibration.py` and restart the script.

There is no live mode-switch command. Before a firmware mode router is added, changing compile-time route
requires rebuilding all compatible boards together. Never enable a drive test with wheels on the floor.
