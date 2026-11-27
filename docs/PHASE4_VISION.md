# Phase 4 — OpenMV processing and diagnostic coverage

## Current software path

`goal_wall_navigation.py:main_loop()` captures an image, scans floor/edges before optional goal work, runs
`competition_vision.Pipeline.step()`, then sends the packet before optional display drawing. The V2 path
checks one goal color per selected interval and skips/defers goal work when the line data would exceed its
freshness budget. `fast_scan.py` is used only after startup/self-check conditions; otherwise the reference
scan remains available. RGB classification caching and bounded stage-time summaries already exist.

This branch adds UART write attempt/completion/partial/failure/expiry/unknown counters and opt-in two-second
CSV records with firmware/config identity and capture/scan/goal/guard/UART/total timing summaries. Startup
messages remain textual. It does not change exposure, gain, LAB thresholds, mirror geometry, or floor calibration.

## Software evidence

- `test_fast_scan.py`: random-floor equivalence, mismatch fallback, unknown-floor refusal, line checks,
  fixed telemetry budget, stage reporting, and frame freshness.
- `test_competition_vision.py`: per-color scheduling, line freshness priority, goal shapes/bearings,
  tracking expiry, UART partial writes, expiry and injected OSError/no-progress.
- Python syntax and 110 Python tests pass on host.

## Limits

Host runtime is not OpenMV FPS. No camera hardware, recorded match frame, USB rendering load or UART wire
timing was measured. The reported yellow candidate issue cannot be resolved without actual camera frames.
Phase 4 is `PARTIAL`; device timing and image calibration are `PENDING_HARDWARE`.
