# Phase 8 — Orbit and attack coverage

The expanded `brain::Brain` contains approach, align, orbit, attack, recover and own-goal veto branches.
It consumes relative IR ball bearing and fresh goal data; IR strength is not used as calibrated distance.
The kicker output remains initialized low and no new automatic kicker/dribbler behavior was added.

`ENABLE_ORBIT` and `ENABLE_ATTACK` now compile-gate their corresponding branches and default to 0. With
these off, the expanded controller uses direct approach/alignment and never claims an attack state.
`test_upgrade.cpp` exercises orbit/align/attack/recovery and own-goal avoidance when the full tactic test
profile is enabled. The separate default-profile test asserts these branches remain out of the default
build.

Phase 8 is `PARTIAL`: host state logic is covered, but approach geometry, heading assumptions, ball
reacquisition, line interaction, scoring and tournament legality have not been verified on hardware.
