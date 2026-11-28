"""Calibration retained from 968bc52. Verify mirror orientation, LAB and ground distances on hardware."""

FRAME_SIZE = "QVGA"

IMAGE_SIZE = (320, 240)

MIRROR_CENTER = (171, 129)

MIRROR_RING = (29, 77)

FRONT_DEG = 86.0

CLOCKWISE = False

GROUND_POINTS = ((50, 18), (74, 50))

TARGET_GOAL = "YELLOW"

GOAL_MIN_WIDTH_DEG = 6

GOAL_MAX_WIDTH_DEG = 150

GOAL_MIN_ASPECT = 0.8

GOAL_MIN_DENSITY = 0.2

EXPOSURE_US = 45000
# Bench presets reuse the base thresholds only for comparison. Recalibrate before field use.
EXPOSURE_PRESETS_US = (45000, 30000, 20000, 15000, 10000, 5000)
BENCH_EXPOSURE_US = None
# Optional QVGA crop preserving pixel scale, mirror ring and the floor rays.
BENCH_WINDOW = None  # Try (80, 48, 176, 168).
BENCH_GAIN_DB = None
GOAL_X_STRIDE = 1  # Try 2 on the bench, especially with small distant goals.
VISUALIZATION_MODE = "LIGHT"
# Full-frame reference reported by the operator. UART READY stays clear.
BENCH_BASELINE = False

GAIN_DB = 6.0

RGB_GAIN_DB = None

YELLOW = [(30, 62, 5, 24, 26, 64), (79, 86, -3, 7, 57, 70),
          (69, 88, 18, 32, 22, 39), (62, 88, 14, 32, 40, 55),
          (80, 100, -15, 5, 45, 75)]

BLUE = [(12, 65, -3, 30, -65, -18),
        (4, 10, -5, 2, -18, -10)]

GREEN = (12, 75, -65, -10, -15, 55)

GREEN_LOW_SATURATION = (33, 48, -13, -7, 2, 11)

WHITE = (72, 100, -12, 12, -15, 15)
WHITE_CANDIDATE = (85, 100, -8, 8, -8, 8)
BENCH_WHITE_CANDIDATE = False  # Candidate-only comparison always clears UART READY.
MIN_VISIBLE_L = 12
MAX_VISIBLE_L = 97
# Calibrated QVGA rectangles only; an occluded safety sample makes its ray unknown.
LINE_SELF_MASKS = ()
# (body radius cm, speed limit cm/s, worst pipeline ms, measured brake cm, margin cm)
MEASURED_MOTION = None
BENCH_FRAME_SIZE = None  # "QQVGA" derives a pixel-scaled comparison; UART READY is clear.
BENCH_WHITE_BACKEND = None  # "BLOBS" or "EDGES"; run after the safety packet for timing only.

BLUE_MIN_PIXELS = 8

YELLOW_MIN_PIXELS = 12
