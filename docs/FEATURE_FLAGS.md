# Feature flags and build identity

## Current compile-time flags

The shared compile-time contract is in `soccer_camera/libraries/RobotDiagnostics/src/RuntimeConfig.h`.
Edit the `ENABLE_*` values at build time. The compiler rejects missing mandatory drive inputs and invalid
dependencies. BNO055 and the separate LINE board are forbidden by a compile-time assertion. This header
does not yet replace every legacy selector (`ROBOT_UPGRADE`, `ROBOT_ZIP_TRACK`, `ROBOT_ZIP_COMPAT`), so
the diagnostic snapshot must be checked against those actual selectors before a build is used.

| Flag | Default | Meaning / restriction |
|---|---:|---|
| `ENABLE_IR` | 1 | Required ball sensor link |
| `ENABLE_MOTOR` | 1 | Required motor link |
| `ENABLE_PD_CHASE` | 1 | Baseline chase capability |
| `ENABLE_CAMERA` | 1 | OpenMV link; competition motion requires freshness |
| `ENABLE_LINE_ESCAPE` | 1 | Requires camera |
| `ENABLE_OBSTACLE_AVOID` | 0 | Rejected: no independently validated obstacle input |
| `ENABLE_BALL_SEARCH` | 0 | Compile-time gated in expanded Brain; requires hardware validation |
| `ENABLE_GOAL_DETECTION` | 1 | Camera capability; goal data is not required for ball chase |
| `ENABLE_ORBIT` | 0 | Compile-time gated; requires camera, goal detection and PD |
| `ENABLE_ATTACK` | 0 | Compile-time gated; requires camera, goal detection and PD |
| `ENABLE_DIAGNOSTICS` | 1 | Diagnostic build capability |
| `ENABLE_EXPERIMENTAL_PD_FILTER` | 0 | Separate variable-dt/filtered-D candidate; reference ZIP PD stays default |

The build identity is `ROBOT_CONFIG_ID` / `CONFIG_ID = RCJ-IR-SW-2026-11-27-A`. Arduino builds also print
the injected Git SHA when built with `tests/compile_arduino.ps1`; a manual build may show `unembedded`.
OpenMV cannot query Git and requires `GIT_COMMIT` to be updated when copied.

The configuration rejects competition profiles without IR, MOTOR, PD, fresh camera input or line escape.
Those safety/core flags can be edited, but disabling them makes the build invalid. Search/orbit/attack are
compile-time gated in `brain::Brain` and default off. The filtered PD candidate is compile-time selectable
but default off. `ENABLE_DIAGNOSTICS` is currently an identity/capability flag and does not strip all log
calls. The legacy route switches still choose the active control path; see `BASELINE_AUDIT_2026-11-27_JA.md`.
