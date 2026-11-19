"""Measurements for the H7 Plus with its competition mirror installed."""

FRAME_SIZE = "QVGA"
# Enter (image_width, image_height), (center_x, center_y), and (inner_px, outer_px).
IMAGE_SIZE = None
MIRROR_CENTER = None
MIRROR_RING = None
FRONT_DEG = 0.0  # Clockwise from image up to the kicker's direction.
CLOCKWISE = True  # False if moving a target to the robot's right decreases its image angle.

# (radius_px, floor_distance_cm), ordered by increasing distance, measured from mirror axis.
GROUND_POINTS = ()
TARGET_GOAL = None  # Set to "YELLOW" or "BLUE" before each half.

# Capture settings are shared by all four scripts. None uses automatic exposure and gain.
EXPOSURE_US = None
GAIN_DB = None
RGB_GAIN_DB = None  # None freezes the white balance measured after warm-up.

# Starting thresholds from the repository. Measure these through the mounted mirror.
YELLOW = [(79, 86, -3, 7, 57, 70)]
BLUE = [(39, 42, 3, 7, -45, -41), (19, 25, 13, 20, -46, -40)]
BLACK = (5, 32, -20, 5, -5, 15)
GREEN = (20, 75, -65, -10, -15, 55)
WHITE = (72, 100, -12, 12, -15, 15)
GOAL_MIN_PIXELS = 30  # Check both goals at the farthest usable distance.
