"""OpenMV H7 Plus. Publish line safety before a bounded goal work unit and the IDE overlay."""
import gc
import image
import pyb
import sensor
import time
import camera_calibration as cfg
import preview
import vision

try:
    import omv
    disable_preview = getattr(omv, "disable_fb", None)
except ImportError:
    disable_preview = None


def initialize_camera():
    for led in (1, 2, 3, 4):
        pyb.LED(led).off()
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(getattr(sensor, cfg.FRAME_SIZE))
    window = None
    if cfg.BENCH_WINDOW is not None:
        sensor.set_windowing(cfg.BENCH_WINDOW)
        window = sensor.get_windowing()  # Use the driver's actual aligned crop.
    sensor.set_auto_exposure(True)
    sensor.set_auto_gain(True)
    sensor.set_auto_whitebal(True)
    sensor.skip_frames(time=1500)
    gain = sensor.get_rgb_gain_db() if cfg.RGB_GAIN_DB is None else cfg.RGB_GAIN_DB
    sensor.set_auto_whitebal(False, rgb_gain_db=gain)
    exposure = cfg.EXPOSURE_US if cfg.BENCH_EXPOSURE_US is None else cfg.BENCH_EXPOSURE_US
    if cfg.BENCH_EXPOSURE_US is not None and exposure not in cfg.EXPOSURE_PRESETS_US:
        raise ValueError("unsupported bench exposure")
    sensor.set_auto_exposure(False, exposure_us=exposure)
    sensor.set_auto_gain(False, gain_db=cfg.GAIN_DB if cfg.BENCH_GAIN_DB is None else cfg.BENCH_GAIN_DB)
    sensor.skip_frames(time=300)
    get_exposure = getattr(sensor, "get_exposure_us", None)
    return window, exposure, get_exposure() if get_exposure else -1


def local_roi(full, box, padding):
    fx, fy, fw, fh = full
    x, y, w, h = box
    left, top = max(fx, x - padding), max(fy, y - padding)
    return left, top, min(fx + fw, x + w + padding) - left, min(fy + fh, y + h + padding) - top


def goal_tiles(full):
    x, y, w, h = full
    # Four overlapping half-ring strips let normal goals cross quadrant boundaries.
    sw, sh = 2 * w // 3, 2 * h // 3
    return (x, y, w, sh), (x + w - sw, y, sw, h), (x, y + h - sh, w, sh), (x, y, sw, h)


def main():
    window, exposure, actual_exposure = initialize_camera()
    uart = pyb.UART(3, 115200, timeout=10, timeout_char=2)
    usb = pyb.USB_VCP()
    ide_connected = getattr(usb, "debug_mode_enabled", None)
    frame = sensor.snapshot()
    mirror = vision.Mirror(frame.width(), frame.height(), window)
    scanner = vision.Scanner(mirror, frame.width(), image.rgb_to_lab)
    if mirror.rays:
        try:
            scanner.prepare(frame)
            scanner.fast = all(scanner.verify(sensor.snapshot()) for _ in range(3))
        except (AttributeError, TypeError, ValueError, MemoryError, RuntimeError):
            scanner.fast = False
        if not scanner.fast:
            scanner.table = None
        gc.collect()

    guard = vision.LineGuard(time.ticks_diff)
    overlay = preview.Overlay(mirror)
    timings = preview.Timings()
    tiles = goal_tiles(mirror.roi) if mirror.roi else ()
    goals, current = [None, None], [None, None]
    reports = [[], []]
    boxes = [None, None]
    attempts, tile_index = [0, 0], [0, 0]
    last_roi = [None, None]
    goal_seen_us = [None, None]
    colors = ((cfg.YELLOW, cfg.YELLOW_MIN_PIXELS), (cfg.BLUE, cfg.BLUE_MIN_PIXELS))
    mode = cfg.VISUALIZATION_MODE
    if mode not in ("OFF", "LIGHT", "FULL"):
        mode = "LIGHT"
    if disable_preview:
        disable_preview(mode == "OFF")
    bench = cfg.BENCH_EXPOSURE_US is not None or cfg.BENCH_GAIN_DB is not None or window is not None
    active_color = sequence = sent = partial = late = frames = interval_late = goal_skips = 0
    goal_errors = draw_errors = 0
    goal_error = draw_error = "none"
    last_report = time.ticks_ms()
    last_tx_us = None
    fps = 0.0
    goal_mode = b"g"
    yellow_probe = False
    last_goal_ms = None
    goal_interval_ms = 60
    packet_data = bytearray(23)
    while True:
        start = time.ticks_us()
        frame = sensor.snapshot()
        captured = time.ticks_us()
        now = time.ticks_ms()
        ready = bool(mirror.rays and (frame.width(), frame.height()) == mirror.size)
        visible, lines = scanner.scan(frame) if ready else (set(), {})
        scanned = time.ticks_us()
        mask, escape, nearest = guard.update(visible, lines, now)
        guarded = time.ticks_us()
        frame_ms = (time.ticks_diff(guarded, start) + 999) // 1000
        slow = frame_ms > vision.MAX_FRAME_MS
        late += int(slow)
        goal_values = tuple(vision.goal_fields(g, now, time.ticks_diff) for g in goals)
        data = vision.packet(sequence, ready and not bench, bool(guard.near), mask, escape, nearest,
                             goal_values, frame_ms, packet_data)
        uart_start = time.ticks_us()
        try:
            written = uart.write(data)
        except OSError:
            written = 0
        uart_end = time.ticks_us()
        gap = time.ticks_diff(uart_end, last_tx_us) if last_tx_us is not None else 0
        if written == len(data):
            sent += 1
            if gap:
                timings.add(8, gap)
                interval_late += int(gap > 100000)
            last_tx_us = uart_end
        else:
            partial += 1
        sequence = (sequence + 1) & 255

        current[0] = current[1] = None
        skip = frame_ms >= 75 or gap >= 85000
        due = last_goal_ms is None or time.ticks_diff(now, last_goal_ms) >= goal_interval_ms
        probing = ready and not slow and not skip and yellow_probe
        if ready and not slow and (probing or (goal_mode != b"l" and (goal_mode == b"b" or (due and not skip)))):
            goal_interval_ms = 60
            last_goal_ms = now
            for color in ((0,) if probing else (0, 1) if goal_mode == b"b" else (active_color,)):
                thresholds, minimum = colors[color]
                step = attempts[color]
                if probing:
                    roi = last_roi[0] or tiles[0]
                elif goal_mode in (b"b", b"h") or step % 8 == 7:
                    roi = mirror.roi  # Periodically retain the baseline whole-ring search.
                elif boxes[color] and goal_values[color][0] != 65535 and step % 2:
                    roi = local_roi(mirror.roi, boxes[color], 16)
                else:
                    roi = tiles[tile_index[color]]
                    tile_index[color] = (tile_index[color] + 1) % len(tiles)
                last_roi[color] = roi
                if not probing:
                    attempts[color] += 1
                begin = time.ticks_us()
                try:
                    candidate = vision.find_goal(frame, mirror, thresholds, minimum, scanner.goal_color,
                                                 roi, 2 if probing else cfg.GOAL_X_STRIDE, reports[color], probing)
                except (AttributeError, TypeError, ValueError, MemoryError, OSError, RuntimeError) as exc:
                    goal_errors += 1
                    goal_error = type(exc).__name__
                    reports[color].clear()
                    candidate = None
                    goal_interval_ms = 160
                goal_done = time.ticks_us()
                duration = time.ticks_diff(goal_done, begin)
                timings.add(4 + color, duration)
                # Native find_blobs is synchronous. Increase the quiet time after an overrun.
                if duration > 12000:
                    goal_interval_ms = 160
                if candidate is not None and not probing:
                    box, center = reports[color][-2:]
                    current[color] = (candidate, box, center)
                    boxes[color] = box
                    goals[color] = (candidate, now)
                    if goal_seen_us[color] is not None:
                        timings.add(9 + color, time.ticks_diff(goal_done, goal_seen_us[color]))
                    goal_seen_us[color] = goal_done
            if probing:
                yellow_probe = False
            else:
                active_color ^= 1
        elif ready and goal_mode != b"l" and due:
            goal_skips += 1
        scanner.frame_buffer = None

        before_draw = time.ticks_us()
        if mode != "OFF" and (ide_connected is None or ide_connected()) and hasattr(frame, "draw_string"):
            try:
                overlay.draw(frame, mode, now, time.ticks_diff, fps, frame_ms, guard, lines,
                             mask if ready and not slow else 0, escape if ready and not slow else 0,
                             nearest, current, goal_values)
            except (AttributeError, TypeError, ValueError, MemoryError, OSError, RuntimeError) as exc:
                draw_errors += 1
                draw_error = type(exc).__name__
                mode = "OFF"
        end = time.ticks_us()
        for index, us in ((0, time.ticks_diff(captured, start)), (1, time.ticks_diff(scanned, captured)),
                          (2, time.ticks_diff(guarded, scanned)), (3, time.ticks_diff(uart_end, uart_start)),
                          (6, time.ticks_diff(end, before_draw)), (7, time.ticks_diff(end, start))):
            timings.add(index, us)
        frames += 1

        command = usb.read(1) if usb.any() else None
        if command in (b"0", b"1", b"2"):
            mode = ("OFF", "LIGHT", "FULL")[command[0] - 48]
            if disable_preview:
                disable_preview(mode == "OFF")
        if command == b"d":
            yellow_probe = True
        if command in (b"l", b"g", b"b", b"h"):
            goal_mode = command
            if goal_mode == b"l":
                goals[0] = goals[1] = None
        elapsed = time.ticks_diff(now, last_report)
        if elapsed >= 2000 or (command == b"i" and usb.isconnected()) or (probing and usb.isconnected()):
            fps = frames * 1000 / max(1, elapsed)
            if usb.isconnected():
                try:
                    summary = ("FPS=%.1f MODE=%s IDE=%s EXPOSURE_US=%d ACTUAL_EXPOSURE_US=%d SIZE=%s BENCH=%d FRAME_MS=%d "
                               "LINE=%s SAFE_MASK=%04x READY=%d FAST=%d UART_TX=%d UART_PARTIAL=%d "
                               "FRAME_LATE=%d INTERVAL_LATE=%d GOAL_SKIP=%d GOAL_ERROR=%d/%s DRAW_ERROR=%d/%s\n") % (
                                   fps, mode, ide_connected() if ide_connected else "unknown", exposure, actual_exposure,
                                   (frame.width(), frame.height()), bench, frame_ms, nearest,
                                   mask if ready and not slow and not bench else 0, ready and not bench, scanner.fast,
                                   sent, partial, late, interval_late, goal_skips, goal_errors, goal_error, draw_errors, draw_error)
                    usb.send((timings.report() + "\n").encode(), timeout=0)
                    if command == b"i" or probing:
                        usb.send(("GOALS=%s ROI=%s STRIDE=%d MIN_PIXELS=%s "
                                  "COUNTS(raw,small,clip,ring,shape,turf,accepted)=%s\n" %
                                  (goal_values, last_roi, cfg.GOAL_X_STRIDE, (cfg.YELLOW_MIN_PIXELS,
                                   cfg.BLUE_MIN_PIXELS), (reports[0][:7], reports[1][:7]))).encode(), timeout=0)
                    usb.send(summary.encode(), timeout=0)
                except OSError:
                    pass
            last_report, frames = now, 0
            timings = preview.Timings()


main()
