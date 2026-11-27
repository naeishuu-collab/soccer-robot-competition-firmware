# Phase 5 — Line escape coverage

## Current software behavior

OpenMV `LineGuard.update()` builds a 16-sector mask only from observed floor, keeps unknown rays closed,
closes approach headings near a line, merges multiple boundaries, and chooses an exact 45-degree retreat
when an open path exists. Close-line confirmation and release use neighbour/frame/time hysteresis. If the
floor is not ready or no safe retreat exists, the result has no open motion direction. MAIN's expanded
controller requires fresh ready camera data and gives a line escape priority over ball tactics.

## Software evidence

`test_competition_vision.py` covers unknown floor, all-clear floor, front/back and corner escape, multiple
edges, unreadable rays, no safe escape, direction continuity and release hysteresis. `test_fast_scan.py`
covers unclassified pixels and white-line detection without green floor classification.

## Limits

The threshold distances and sector geometry are software constants, not measured robot stopping distance.
Mirror dead zones, field boundary, chassis dimensions during rotation and diagonal traction were not
verified. Do not relax the mask or treat unobserved floor as safe. Phase 5 remains `PARTIAL` and physical
clearance calibration is `PENDING_HARDWARE`.
