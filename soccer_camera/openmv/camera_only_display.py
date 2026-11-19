"""Camera-only display. Keep goal_wall_navigation.py and camera_calibration.py on the camera."""
from goal_wall_navigation import main_loop

SEND_UART = False

if __name__ == "__main__":
    main_loop(send_uart=SEND_UART, display=True)
