# Phase 6 — Ball search coverage

## Current software behavior

The expanded `brain::Brain` distinguishes IR link freshness/sensor faults from a valid link with no ball.
Its existing ball tracker has WAIT/TRACK/SEARCH/REACQUIRE states and a bounded last-bearing patrol. The
new build configuration gates search with `ENABLE_BALL_SEARCH`, default 0. With search disabled, an
unacquired/lost ball yields a safe stop; line escape and camera freshness are checked first. The camera
never fabricates a ball detection.

## Software evidence

`test_upgrade.cpp` exercises ball state transitions and loss/reacquisition; the integration suite injects
IR link loss and stale sensor data. The test runner enables the tactic flags for the full-path regression
profile and separately builds the default-disabled feature profile.

## Limits

No robot motion or ball loss at match speed was measured. The default disables search until the sweep
direction and line response are verified on the field. Phase 6 is `PARTIAL`; hardware timing is
`PENDING_HARDWARE`.
