# Competition stop and motion conditions

## MAIN motion states

The competition candidate selects `ROBOT_ZIP_TRACK=1` by default. Its motion states are `CHASE`, `ESCAPE`, and `STOP`.

| State / reason | Entry condition | Recovery |
| --- | --- | --- |
| `STOP / MANUAL_STOP` | USB `d` or `s` sets the hold latch | USB `g` |
| `STOP / CAMERA_TIMEOUT` | No accepted camera packet, packet older than 200 ms, camera not ready, or camera reports slow | A fresh, ready camera packet; manual hold still requires `g` |
| `STOP / LINE_BLOCKED` | Camera reports a line but no supported forward/reverse escape direction has a clear sector | Camera reports a supported clear escape direction |
| `STOP / IR_TIMEOUT` | No accepted IR reply within 100 ms while chasing | A fresh valid IR reply |
| `STOP / BALL_LOST` | IR strength is zero while chasing | A positive-strength IR reply |
| `STOP / NO_SAFE_DIRECTION` | ZIP PWM is outside the conservative mask rule | The required sector(s) become clear |
| `ESCAPE` | Fresh ready camera reports escape at 0 or 180 degrees and its corresponding 16-sector mask bit is clear | Camera clears escape; chase then needs fresh IR |
| `CHASE` | Camera is fresh/ready, no escape is active, IR is fresh and strength is nonzero | Any stop condition above |

Escape does not require IR. The current direct-PWM escape implementation supports only 0° and 180° and commands all four wheels at +175 or -175. Other escape bearings remain stopped. Camera freshness and readiness are still mandatory.

MAIN diagnostics are requested over USB with `i`; output includes the state, stop reason, PD values, calculated/safety/transmitted PWM, camera age, IR age, and safety mask. MOTOR has independent USB diagnostics. MAIN receives no MOTOR status reply, so MAIN logs cannot confirm applied motor output.

## Conservative path check

`zipPathClear()` requires a matching 8-wheel grouping. Equal signed groups check the corresponding forward or reverse sector. A split group is treated as a turn and requires all 16 sectors. This can cause nuisance stops. No new rule is enabled because the checked-in repository does not establish wheel-to-sector motion, footprint sweep, camera blind spots, or stopping distance. The old rule can be restored from the parent commit (`63cffa17e4f71df638646ffabf6585a3e2a44489`) with `git revert <change-commit>` or by checking out that commit in a separate worktree.

## ZIP reference limits

`soccer_camera/main/zip_chase_pd.h` uses Kp=2.0, Kd=0.2, base PWM 175, turn clamp ±250, and nonzero wheel duty 140..210. `tests/test_zip_chase_pd.cpp` compares against an independent transcription for 10,000 inputs. The source `chase (4).zip` itself is not present in this repository, so this is software regression evidence, not a byte-for-byte or source-level comparison with the ZIP original. The filtered PD class remains compile-disabled by default.

## Hardware pending

`PENDING_HARDWARE`: verify wheel order and polarity, 175 escape movement, 140 minimum duty, camera sector orientation, stopping distance, reverse dwell, and line escape on the real robot with wheels raised first. Do not treat host tests as a hardware safety certification.
