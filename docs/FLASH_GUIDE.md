# Build and flash targets

## Source targets

| Board | Source to compile/copy | Build settings in current repository |
|---|---|---|
| MAIN RP2354B | `soccer_camera/main/` (Arduino sketch `main.ino`) | Generic RP2350, variant `RP2350B`, 2 MB; source selectors in `main.ino` |
| IR RP2354A | `soccer_camera/ir/` (`ir.ino`, `ir_logic.h`) | Generic RP2350, variant `RP2350A`, 2 MB |
| MOTOR RP2354A | `soccer_camera/motor/` (`motor.ino`, `motor_logic.h`) | Generic RP2350, variant `RP2350A`, 2 MB |
| OpenMV H7 Plus | `soccer_camera/openmv/goal_wall_navigation.py`, `competition_vision.py`, `camera_calibration.py` | `PROTOCOL=2` is paired to MAIN's extended parser; `GIT_COMMIT` must be manually recorded |

The `line/`, `soccer_ir_main_line/`, `chase_*`, `legacy_before_chase/`, and backup folders are not current
competition flash targets. LINE is not used in the required configuration.

## Compile only

Run from the repository root:

```powershell
.\soccer_camera\tests\compile_arduino.ps1
```

This injects the current short Git SHA and compiles the four RP2350 sketches without upload. For a manual
build, use the same core `rp2040:rp2040 6.3.2`, Generic RP2350 variants and 2 MB setting recorded in
`tests/arduino_compile_current.txt`. Manual builds can report `unembedded`.

## Flashing

**No firmware was uploaded in this task.** Flash only after `HARDWARE_TEST_GUIDE.md` passes and a known-good
rollback image is available. Use the board-specific target above and the locally verified COM port; do not
copy a generated binary from a different variant. For OpenMV, copy the three matching files together and
set `GIT_COMMIT` / `CONFIG_ID` before reset. After every write, read the version/config identity from USB
and verify the matching board set before enabling motor power.
