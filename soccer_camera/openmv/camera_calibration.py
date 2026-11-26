"""Measurements for the H7 Plus with its competition mirror installed."""

FRAME_SIZE = "QVGA"
# Enter (image_width, image_height), (center_x, center_y), and (inner_px, outer_px).
IMAGE_SIZE = (320, 240)
MIRROR_CENTER = (180, 132)
MIRROR_RING = (40, 100)
FRONT_DEG = 297.0  # Clockwise from image up to the robot's forward direction.
CLOCKWISE = False  # False if moving a target to the robot's right decreases its image angle.

# (radius_px, floor_distance_cm), ordered by increasing distance, measured from mirror axis.
GROUND_POINTS = ((50, 18), (74, 50))
TARGET_GOAL = "YELLOW"  # Set to "YELLOW" or "BLUE" before each half.

# Capture settings are shared by all four scripts. None uses automatic exposure and gain.
EXPOSURE_US = 45000
GAIN_DB = 6.0
RGB_GAIN_DB = None  # None freezes the white balance measured after warm-up.

# Yellow sample at 45 ms: LAB (78–79, 24–26, 29–30).
YELLOW = [(30, 62, 5, 24, 26, 64), (79, 86, -3, 7, 57, 70),
          (69, 88, 18, 32, 22, 39)]
# Additional sample: LAB L=21..22, A=8..9, B=-34..-33.
BLUE = [(18, 25, 5, 12, -38, -30), (39, 42, 3, 7, -45, -41), (19, 25, 13, 20, -46, -40),
        (4, 10, -5, 2, -18, -10)]
BLACK = (5, 32, -20, 5, -5, 15)
GREEN = (12, 75, -65, -10, -15, 55)
WHITE = (72, 100, -12, 12, -15, 15)
GOAL_MIN_PIXELS = 30  # Check both goals at the farthest usable distance.
