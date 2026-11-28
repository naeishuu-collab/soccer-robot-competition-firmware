"""OpenMV H7 Plus boot entry. White-line UART output precedes optional goal work."""
import gc
import image
import pyb
import sensor
import time
import camera_calibration as cfg
import vision


def initialize_camera():
    for led in (1, 2, 3, 4):
        pyb.LED(led).off()
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(getattr(sensor, cfg.FRAME_SIZE))
    sensor.set_auto_exposure(True)
    sensor.set_auto_gain(True)
    sensor.set_auto_whitebal(True)
    sensor.skip_frames(time=1500)
    gain = sensor.get_rgb_gain_db() if cfg.RGB_GAIN_DB is None else cfg.RGB_GAIN_DB
    sensor.set_auto_whitebal(False, rgb_gain_db=gain)
    sensor.set_auto_exposure(False, exposure_us=cfg.EXPOSURE_US)
    sensor.set_auto_gain(False, gain_db=cfg.GAIN_DB)
    sensor.skip_frames(time=300)


def main():
    initialize_camera()
    uart = pyb.UART(3, 115200, timeout=10, timeout_char=2)
    usb = pyb.USB_VCP()
    frame = sensor.snapshot()
    mirror = vision.Mirror(frame.width(), frame.height())
    scanner = vision.Scanner(mirror, frame.width(), image.rgb_to_lab)
    if mirror.rays:
        try:
            scanner.prepare(frame)  # This frame is overwritten while constructing the table.
            scanner.fast = all(scanner.verify(sensor.snapshot()) for _ in range(3))
        except (AttributeError, TypeError, ValueError, MemoryError, RuntimeError):
            scanner.fast = False
        if not scanner.fast:
            scanner.table = None
        gc.collect()

    guard = vision.LineGuard(time.ticks_diff)
    goals = [None, None]
    colors = ((cfg.YELLOW, cfg.YELLOW_MIN_PIXELS), (cfg.BLUE, cfg.BLUE_MIN_PIXELS))
    active_color = 0
    sequence = sent = partial = late = frames = 0
    maxima = [0] * 5
    last_report = time.ticks_ms()
    last_goal = time.ticks_add(last_report, -80)
    goal_mode = b"g"
    while True:
        start = time.ticks_us()
        frame = sensor.snapshot()
        captured = time.ticks_us()
        now = time.ticks_ms()
        ready = bool(mirror.rays and (frame.width(), frame.height()) == cfg.IMAGE_SIZE)
        visible, lines = scanner.scan(frame) if ready else (set(), {})
        mask, escape, nearest = guard.update(visible, lines, now)
        guarded = time.ticks_us()
        frame_ms = (time.ticks_diff(guarded, start) + 999) // 1000
        slow = frame_ms > vision.MAX_FRAME_MS
        late += int(slow)
        goal_values = tuple(vision.goal_fields(g, now, time.ticks_diff) for g in goals)
        data = vision.packet(sequence, ready, bool(guard.near), mask, escape, nearest, goal_values, frame_ms)
        uart_start = time.ticks_us()
        try:
            written = uart.write(data)
        except OSError:
            written = 0
        if written == len(data):
            sent += 1
        else:
            partial += 1
        uart_end = time.ticks_us()
        sequence = (sequence + 1) & 255
        if ready and not slow and goal_mode != b"l":
            bench_both = goal_mode == b"b"
            if bench_both or time.ticks_diff(now, last_goal) >= 80:
                for color in ((0, 1) if bench_both else (active_color,)):
                    thresholds, minimum = colors[color]
                    candidate = vision.find_goal(frame, mirror, thresholds, minimum, scanner.color)
                    if candidate is not None:
                        goals[color] = (candidate, now)
                active_color ^= 1
                last_goal = now
        end = time.ticks_us()
        frames += 1
        times = (time.ticks_diff(captured, start), time.ticks_diff(guarded, captured),
                 time.ticks_diff(uart_end, uart_start), time.ticks_diff(end, uart_end),
                 time.ticks_diff(end, start))
        for index in range(5):
            maxima[index] = max(maxima[index], times[index])

        command = usb.read(1) if usb.any() else None
        if command in (b"l", b"g", b"b"):  # bench comparison
            goal_mode = command
            if goal_mode == b"l":
                goals = [None, None]
        elapsed = time.ticks_diff(now, last_report)
        if usb.isconnected() and (elapsed >= 2000 or command == b"i"):
            text = ("FPS=%.1f FRAME_MS=%d CAPTURE_US=%d LINE_US=%d UART_US=%d GOAL_US=%d TOTAL_US=%d "
                    "MAX_US=%s LINE=%s SAFE_MASK=%04x READY=%d SLOW=%d FAST=%d "
                    "YELLOW=%s BLUE=%s UART_TX=%d UART_PARTIAL=%d FRAME_LATE=%d\n") % (
                        frames * 1000 / max(1, elapsed), frame_ms, *times, maxima, nearest,
                        mask if ready and not slow else 0, ready, slow, scanner.fast,
                        goal_values[0], goal_values[1], sent, partial, late)
            try:
                usb.send(text.encode(), timeout=0)
            except OSError:
                pass
            last_report, frames = now, 0
            maxima = [0] * 5
        elif elapsed >= 2000:
            last_report, frames = now, 0
            maxima = [0] * 5


main()
