# Phase 7 — Goal recognition coverage

## Current software behavior

OpenMV tracks yellow and blue candidates separately, estimates mirror-relative bearing, confidence and
freshness, filters sudden jumps, rejects similar-direction color detections, and emits target-goal
configuration. MAIN uses fresh goal bearings when available; goal loss falls back to its ball-tracking
branch. Absolute heading is not claimed from a single goal. Without BNO055, only the existing two-goal
relative estimate is available and remains unverified.

## Software evidence

`test_competition_vision.py` covers candidate shape/density, false objects, two-goal separation, freshness,
angle wrapping and slow processing. `test_openmv_logic.py` covers blue/yellow LAB sample candidates and
wire encoding. The pipeline tests use host fakes/synthetic blobs, not camera images.

## Limits

The yellow-only field issue is not diagnosed conclusively because no recorded target-camera image or live
OpenMV feed was available. Thresholds/ROI/calibration were left unchanged. Phase 7 is `PARTIAL`; color
performance, target-goal choice and field-relative orientation are `PENDING_HARDWARE`.
