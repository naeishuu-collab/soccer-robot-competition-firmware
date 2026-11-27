# Phase 10 — Integrated behavior coverage

The expanded route has one `brain::Brain::step()` decision result per update and one `upgradeLoop()` drive
send path. Its priority order checks link/data validity, usable line mask and escape before ball tactics.
Search/orbit/attack compile gates are separate, and the top-level loop selects one route at compile time.
The ZIP route also has a single PWM send point. MOTOR handles only drive execution and safety.

Host coverage includes `test_integration.cpp`, `test_upgrade.cpp`, `test_loop.cpp`, `test_loop_legacy.cpp`,
`test_zip_loop.cpp`, and the 10,000-input ZIP reference comparison. The code has no USB runtime mode router
yet; mode names in `RuntimeConfig.h` do not constitute operational test modes. Main-to-MOTOR stop reasons
and MOTOR-local stops are not merged because the protocol has no reply channel.

Phase 10 is `PARTIAL`. Software route selection and state logic are host-tested; actual transition latency,
stop delivery while the IR bus is occupied and physical line/drive arbitration are `PENDING_HARDWARE`.
