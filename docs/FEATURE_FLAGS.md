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
| `ENABLE_BALL_SEARCH` | 0 | Experimental, not connected to the active control loop |
| `ENABLE_GOAL_DETECTION` | 1 | Camera capability; goal data is not required for ball chase |
| `ENABLE_ORBIT` | 0 | Requires camera, goal detection and PD; not active |
| `ENABLE_ATTACK` | 0 | Requires camera, goal detection and PD; not active |
| `ENABLE_DIAGNOSTICS` | 1 | Diagnostic build capability |

The build identity is `ROBOT_CONFIG_ID` / `CONFIG_ID = RCJ-IR-SW-2026-11-27-A`. Arduino builds also print
the injected Git SHA when built with `tests/compile_arduino.ps1`; a manual build may show `unembedded`.
OpenMV cannot query Git and requires `GIT_COMMIT` to be updated when copied.

These flags are currently a dependency-checked build contract, not a complete runtime feature router.
The legacy route switches still choose the active control path; see `BASELINE_AUDIT_2026-11-27_JA.md`.
Experimental or hardware-dependent capabilities remain disabled in the active control path.
