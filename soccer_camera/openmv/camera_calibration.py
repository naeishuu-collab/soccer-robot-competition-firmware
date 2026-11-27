"""H7 Plus＋全周ミラー用の校正値。goal_wall_navigation.pyが読み、検出・距離・UART設定へ反映する。
値はカメラ設置状態に依存する。特にミラー中心/半径、前方角、色しきい値、床距離は実機で測る。
competition_vision.pyへ切り替える場合はPROTOCOLとMAIN側ROBOT_UPGRADE設定も対応させる。
"""

FRAME_SIZE = "QVGA"
# Enter (image_width, image_height), (center_x, center_y), and (inner_px, outer_px).
IMAGE_SIZE = (320, 240)
MIRROR_CENTER = (171, 129)  # Approximate values measured from the supplied preview.
MIRROR_RING = (29, 77)  # Inner and outer radii in QVGA pixels.
MIRROR_AUTO_DETECT = False
# Clockwise from image up to the robot's forward direction. 86 was estimated from the camera image on
# 2026-11-27 (previously 297) and is NOT verified: check that the red line points at the robot's front.
FRONT_DEG = 86.0
CLOCKWISE = False  # False if moving a target to the robot's right decreases its image angle.

# (radius_px, floor_distance_cm), ordered by increasing distance, measured from mirror axis.
GROUND_POINTS = ((50, 18), (74, 50))
TARGET_GOAL = "YELLOW"  # Set to "YELLOW" or "BLUE" before each half.

# 1 = legacy 8-byte UART at 9600 (default). 2 = extended packet (wire version 3) at 115200; needs MAIN
# ROBOT_UPGRADE=1 from the same commit and competition_vision.py on the camera.
PROTOCOL = 2
# Overlay and text in the OpenMV IDE (needs visualize.py). Set True on the bench; keep False for matches
# because the extra search and drawing lower the frame rate.
VISUALIZE = True
# Every goal colour candidate with its numbers and rejection reason (drawn and printed). Slow: bench only.
GOAL_DEBUG = True
LINE_BLOCK_CM = 35  # Protocol 2: headings toward a line closer than this are closed early. Unverified.
# (radius_px, distance_cm) table measured for the goals. Empty = distance unknown.
# Never copy GROUND_POINTS here without measuring. The camera does not detect the ball at all.
GOAL_DISTANCE = ()
# Goal shape limits (mirror image). Unverified: tune with real goals and robots in view.
GOAL_MIN_WIDTH_DEG = 6
GOAL_MAX_WIDTH_DEG = 150
GOAL_MIN_ASPECT = 0.8   # arc length / radial height
GOAL_MIN_DENSITY = 0.2  # color pixels / bounding box
FILL_SINGLE_GAP = True  # reopen a single 15 degree floor gap (small object) between clear neighbours


# Capture settings are shared by all four scripts. None uses automatic exposure and gain.
EXPOSURE_US = 45000
GAIN_DB = 6.0
RGB_GAIN_DB = None  # None freezes the white balance measured after warm-up.

# Yellow sample at 45 ms: LAB (78–79, 24–26, 29–30).
YELLOW = [(30, 62, 5, 24, 26, 64), (79, 86, -3, 7, 57, 70),
          (69, 88, 18, 32, 22, 39), (62, 88, 14, 32, 40, 55)]
# Additional sample: LAB L=21..22, A=8..9, B=-34..-33.
# Additional blue sample: L=43..45, A=10..11, B=-39..-38.
# Mid-brightness blue sample: L=28..29, A=9..10, B=-35..-34.
BLUE = [(12, 65, -3, 30, -65, -18),
        (4, 10, -5, 2, -18, -10)]
BLACK = (5, 32, -20, 5, -5, 15)
GREEN = (12, 75, -65, -10, -15, 55)
# Turf sample: L=34..41, A=-10..-9, B=5..7.
GREEN_LOW_SATURATION = (33, 48, -13, -7, 2, 11)
WHITE = (72, 100, -12, 12, -15, 15)
GOAL_MIN_PIXELS = 30  # Check both goals at the farthest usable distance.
BLUE_MIN_PIXELS = 8
YELLOW_MIN_PIXELS = 12
