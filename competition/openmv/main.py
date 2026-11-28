"""OpenMV H7 Plus production entry. Safety UART always precedes goals and drawing."""
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

PRODUCTION_API = 9
BUILD = "rcj-openmv-competition-9"
CAPTURE_ERRORS = (AttributeError, TypeError, ValueError, MemoryError, OSError, RuntimeError)
BASELINE = ((30, 100, -30, 30, 30, 127), (10, 80, -30, 50, -128, -20))


def initialize_camera():
    for led in (1, 2, 3, 4):
        pyb.LED(led).off()
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    if cfg.BENCH_FRAME_SIZE not in (None, "QQVGA") or (cfg.BENCH_FRAME_SIZE and cfg.BENCH_WINDOW):
        raise ValueError("Use either the QVGA crop or the QQVGA comparison")
    sensor.set_framesize(getattr(sensor, cfg.BENCH_FRAME_SIZE or cfg.FRAME_SIZE))
    set_buffers = getattr(sensor, "set_framebuffers", None)
    if set_buffers and not cfg.BENCH_BASELINE:
        try:
            set_buffers(3)
        except CAPTURE_ERRORS as exc:
            print("FRAME_BUFFERS=driver_default %s" % type(exc).__name__)
    window = None
    if cfg.BENCH_WINDOW is not None:
        sensor.set_windowing(cfg.BENCH_WINDOW)
        window = sensor.get_windowing()
    exposure = cfg.EXPOSURE_US if cfg.BENCH_EXPOSURE_US is None else cfg.BENCH_EXPOSURE_US
    if cfg.BENCH_BASELINE:
        sensor.skip_frames(time=2000)
        sensor.set_auto_gain(False)
        sensor.set_auto_whitebal(False)
        exposure = -1  # The reference uses automatic exposure.
    else:
        if cfg.BENCH_EXPOSURE_US is not None and exposure not in cfg.EXPOSURE_PRESETS_US:
            raise ValueError("unsupported bench exposure")
        sensor.set_auto_exposure(True)
        sensor.set_auto_gain(True)
        sensor.set_auto_whitebal(True)
        sensor.skip_frames(time=1500)
        gain = cfg.BENCH_RGB_GAIN_DB or cfg.RGB_GAIN_DB or sensor.get_rgb_gain_db()
        sensor.set_auto_whitebal(False, rgb_gain_db=gain)
        sensor.set_auto_exposure(False, exposure_us=exposure)
        sensor.set_auto_gain(False, gain_db=cfg.GAIN_DB if cfg.BENCH_GAIN_DB is None else cfg.BENCH_GAIN_DB)
        sensor.skip_frames(time=300)
    get_exposure = getattr(sensor, "get_exposure_us", None)
    return window, exposure, get_exposure() if get_exposure else -1


def goal_tiles(full):
    x, y, w, h = full
    sw, sh = 2 * w // 3, 2 * h // 3
    return (x, y, w, sh), (x + w - sw, y, sw, h), (x, y + h - sh, w, sh), (x, y, sw, h)


def paired_roi(full, first, second):
    fx, fy, fw, fh = full
    left = max(fx, min(first[0], second[0]) - 16)
    top = max(fy, min(first[1], second[1]) - 16)
    right = min(fx + fw, max(first[0] + first[2], second[0] + second[2]) + 16)
    bottom = min(fy + fh, max(first[1] + first[3], second[1] + second[3]) + 16)
    return left, top, right - left, bottom - top


def reference_goals(frame, overlay):
    # Bench-only overlay with the operator's successful API and provisional LAB values.
    found = [frame.find_blobs([threshold], pixels_threshold=30, area_threshold=30, merge=True)
             for threshold in BASELINE]
    for color, blobs in enumerate(found):
        for blob in blobs:
            rect = blob.rect
            overlay.primitive(frame, "draw_rectangle", rect() if callable(rect) else rect, preview.COLORS[color])


def report_to_console(usb, ide, lines):
    # IDE consumes Python stdout separately from a raw USB terminal.
    raw_terminal = ide is False and usb.isconnected()
    for line in lines:
        if raw_terminal:
            try:
                data = (line + "\n").encode()
                if usb.send(data, timeout=0) == len(data):
                    continue
            except OSError:
                pass
        print(line)


def white_comparison(frame, roi, backend):
    if backend == "BLOBS":
        return len(frame.find_blobs([cfg.WHITE_CANDIDATE if cfg.BENCH_WHITE_CANDIDATE else cfg.WHITE],
                                    roi=roi, pixels_threshold=2, area_threshold=2))
    sample = frame.copy(roi=roi)
    edges = sample.to_grayscale()
    edges.find_edges(image.EDGE_CANNY)
    return None


def main():
    for module in (vision, preview, cfg):
        if getattr(module, "PRODUCTION_API", 0) != PRODUCTION_API:
            raise RuntimeError("%s API mismatch; copy all 4 OpenMV files (expected %d, got %s)" %
                               (module.__name__, PRODUCTION_API, getattr(module, "PRODUCTION_API", None)))
    window, exposure, actual_exposure = initialize_camera()
    uart = pyb.UART(3, 115200, timeout=10, timeout_char=2)
    usb = pyb.USB_VCP()
    ide_connected = getattr(usb, "debug_mode_enabled", None)
    frame = sensor.snapshot()
    scale = 0.5 if cfg.BENCH_FRAME_SIZE == "QQVGA" else 1
    mirror = vision.Mirror(frame.width(), frame.height(), window, scale)
    clearance, block = vision.safety_distances(cfg.MEASURED_MOTION)
    scanner = vision.Scanner(mirror, frame.width(), image.rgb_to_lab, block, cfg.BENCH_WHITE_CANDIDATE)
    if cfg.BENCH_WHITE_BACKEND not in (None, "BLOBS", "EDGES"):
        raise ValueError("Invalid white backend comparison")
    if mirror.rays and not cfg.BENCH_BASELINE:
        try:
            scanner.prepare(frame)
            scanner.fast = all(scanner.verify(sensor.snapshot()) for _ in range(3))
        except CAPTURE_ERRORS as exc:
            print("SCAN_BACKEND=reference prepare/verify %s:%s" % (type(exc).__name__, exc))
        if not scanner.fast:
            scanner.table = None
        gc.collect()

    body_cm = 0 if cfg.MEASURED_MOTION is None else cfg.MEASURED_MOTION[0]
    guard = vision.LineGuard(time.ticks_diff, clearance, block, body_cm)
    overlay = preview.Overlay(mirror)
    timings = preview.Timings()
    tiles = goal_tiles(mirror.roi) if mirror.roi else ()
    goals, current = [None, None], [None, None]
    reports = [[], []]
    boxes = [None, None]
    goal_seen_us = [None, None]
    colors = ((cfg.YELLOW, cfg.YELLOW_MIN_PIXELS), (cfg.BLUE, cfg.BLUE_MIN_PIXELS))
    mode = cfg.VISUALIZATION_MODE
    if mode not in ("OFF", "LIGHT", "FULL"):
        mode = "LIGHT"
    if disable_preview:
        disable_preview(mode == "OFF")
    calibration_cm = cfg.BENCH_DISTANCE_CM
    if calibration_cm not in (None, 20, 30, 45):
        raise ValueError("BENCH_DISTANCE_CM must be None, 20, 30 or 45")
    bench = (calibration_cm is not None or cfg.BENCH_BASELINE or cfg.BENCH_EXPOSURE_US is not None or cfg.BENCH_GAIN_DB is not None
             or cfg.BENCH_RGB_GAIN_DB is not None or window is not None or cfg.BENCH_FRAME_SIZE is not None or cfg.BENCH_WHITE_CANDIDATE
             or cfg.BENCH_WHITE_BACKEND is not None)
    comparison = None
    targets = ()
    empty_visible, empty_lines = set(), {}
    sequence = sent = partial = late = frames = interval_late = goal_skips = goal_passes = 0
    goal_errors = 0
    goal_error = "none"
    last_report = time.ticks_ms()
    last_tx_us = None
    fps = 0.0
    goal_mode = b"g"
    yellow_probe = False
    last_goal_ms = None
    goal_interval_ms = 45
    last_roi = None
    packet_data = bytearray(23)
    print("BUILD=%s MODE=%s MAIN=%s VISION=%s PREVIEW=%s CAL=%s" % (
        BUILD, mode, globals().get("__file__", "<stdin>"), getattr(vision, "__file__", "?"),
        getattr(preview, "__file__", "?"), getattr(cfg, "__file__", "?")))

    while True:
        start = time.ticks_us()
        frame = sensor.snapshot()
        captured = time.ticks_us()
        now = time.ticks_ms()
        ready = bool(not cfg.BENCH_BASELINE and mirror.rays and block <= mirror.horizon
                     and (frame.width(), frame.height()) == mirror.size)
        visible, lines = scanner.scan(frame) if ready else (empty_visible, empty_lines)
        scanned = time.ticks_us()
        mask, escape, nearest = guard.update(visible, lines, now, scanner.runs if ready else None, mirror.rays,
                                            scanner.censored if ready else ())
        guarded = time.ticks_us()
        frame_ms = (time.ticks_diff(guarded, start) + 999) // 1000
        slow = frame_ms > vision.MAX_FRAME_MS
        late += int(slow)
        fields = (vision.goal_fields(goals[0], now, time.ticks_diff),
                  vision.goal_fields(goals[1], now, time.ticks_diff))
        data = vision.packet(sequence, ready and not bench, guard.escape_requested, mask, escape, nearest,
                             fields, frame_ms, packet_data, guard.blocked)
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
        timings.add(12, time.ticks_diff(uart_end, scanned))

        if calibration_cm is not None:
            try:
                targets = vision.calibration_targets(frame, mirror)
            except CAPTURE_ERRORS as exc:
                targets = ()
                comparison = "calibration_targets/%s:%s" % (type(exc).__name__, str(exc)[:80])

        if ready and cfg.BENCH_WHITE_BACKEND:
            started = time.ticks_us()
            try:
                comparison = white_comparison(frame, mirror.roi, cfg.BENCH_WHITE_BACKEND)
            except CAPTURE_ERRORS as exc:
                comparison = "%s:%s" % (type(exc).__name__, str(exc)[:80])
            timings.add(13, time.ticks_diff(time.ticks_us(), started))

        current[0] = current[1] = None
        skip = frame_ms >= 75 or gap >= 85000
        due = last_goal_ms is None or time.ticks_diff(now, last_goal_ms) >= goal_interval_ms
        probing = ready and not skip and yellow_probe
        if calibration_cm is not None:
            pass  # Show white targets and production scan from the same frame.
        elif cfg.BENCH_BASELINE and mode != "OFF" and goal_mode != b"l":
            try:
                reference_goals(frame, overlay)
            except CAPTURE_ERRORS as exc:
                goal_errors += 1
                goal_error = "reference_goals/%s:%s" % (type(exc).__name__, str(exc)[:80])
        elif ready and not slow and (probing or (goal_mode != b"l" and (goal_mode == b"b" or (due and not skip)))):
            goal_interval_ms = 45
            last_goal_ms = now
            roi = mirror.roi
            if goal_mode == b"g" and not probing and goal_passes % 4:
                if all(boxes[i] is not None and fields[i][0] != 65535 for i in range(2)):
                    roi = paired_roi(roi, boxes[0], boxes[1])
                else:
                    roi = tiles[(goal_passes - 1 - goal_passes // 4) % len(tiles)]
            last_roi = roi
            begin = time.ticks_us()
            operation = "find_goals"
            candidates = [None, None]
            try:
                if probing or goal_mode == b"b":
                    for color in ((0,) if probing else (0, 1)):
                        thresholds, minimum = colors[color]
                        operation = "find_goal.YELLOW" if color == 0 else "find_goal.BLUE"
                        started = time.ticks_us()
                        candidates[color] = vision.find_goal(frame, mirror, thresholds, minimum, scanner.goal_color,
                                                             roi, 2 if probing else cfg.GOAL_X_STRIDE, reports[color], probing)
                        timings.add(4 + color, time.ticks_diff(time.ticks_us(), started))
                else:
                    candidates = vision.find_goals(frame, mirror, roi, cfg.GOAL_X_STRIDE, scanner.goal_color, reports)
                    timings.add(11, time.ticks_diff(time.ticks_us(), begin))
            except CAPTURE_ERRORS as exc:
                goal_errors += 1
                goal_error = "%s/%s:%s" % (operation, type(exc).__name__, str(exc)[:80])
                goal_interval_ms = 160
            done = time.ticks_us()
            if time.ticks_diff(done, begin) > 12000:
                goal_interval_ms = 160
            if probing:
                yellow_probe = False
            else:
                goal_passes += 1
                for color, candidate in enumerate(candidates):
                    if candidate is not None:
                        box, center = reports[color][-2:]
                        current[color] = (candidate, box, center)
                        boxes[color] = box
                        goals[color] = (candidate, now)
                        if goal_seen_us[color] is not None:
                            timings.add(9 + color, time.ticks_diff(done, goal_seen_us[color]))
                        goal_seen_us[color] = done
        elif ready and goal_mode != b"l" and due:
            goal_skips += 1
        before_draw = time.ticks_us()
        if mode != "OFF":
            try:
                if calibration_cm is not None:
                    overlay.calibration(frame, calibration_cm, targets)
                overlay.draw(frame, mode, now, time.ticks_diff, fps, frame_ms, guard, lines,
                             mask if ready and not slow else 0, escape if ready and not slow else 0,
                             nearest, current, fields, visible,
                             time.ticks_diff(scanned, captured), gap)
            except CAPTURE_ERRORS as exc:
                overlay.errors += 1
                overlay.last_error = "Overlay.draw/%s:%s" % (type(exc).__name__, str(exc)[:80])
        end = time.ticks_us()
        for index, us in ((0, time.ticks_diff(captured, start)), (1, time.ticks_diff(scanned, captured)),
                          (2, time.ticks_diff(guarded, scanned)), (3, time.ticks_diff(uart_end, uart_start)),
                          (6, time.ticks_diff(end, before_draw)), (7, time.ticks_diff(end, start))):
            timings.add(index, us)
        frames += 1

        command = usb.read(1) if usb.any() else None
        if command in (b"0", b"1", b"2"):
            mode = ("OFF", "LIGHT", "FULL")[command[0] - 48]
            overlay.disabled.clear()
            if disable_preview:
                disable_preview(mode == "OFF")
        if command == b"d":
            yellow_probe = True
        if command in (b"l", b"g", b"b", b"h"):
            goal_mode = command
            if goal_mode == b"l":
                goals[0] = goals[1] = None
        elapsed = time.ticks_diff(now, last_report)
        if (cfg.AUTO_REPORT_MS and elapsed >= cfg.AUTO_REPORT_MS) or command == b"i" or probing:
            fps = frames * 1000 / max(1, elapsed)
            ide = ide_connected() if ide_connected else None
            summary = ("BUILD=%s FPS=%.1f MODE=%s IDE=%s EXPOSURE_US=%d ACTUAL_EXPOSURE_US=%d BENCH=%d "
                       "FRAME_MS=%d READY=%d FAST=%d SAFE_MASK=%04x FLOOR=ASSUMED OBSERVED_MASK=%04x LINE_MASK=%04x UNKNOWN_MASK=%04x "
                       "ESCAPE_REQUEST=%d ESCAPE_AWAY_MASK=%04x ESCAPE_MASK=%04x LINE=%s UART_TX=%d UART_PARTIAL=%d "
                       "WHITE_RAYS=%d STRICT_RAYS=%d CLOSING_CM_S=%s TTC_MS=%s RANGE_CM=%s WHITE_BENCH=%s "
                       "FRAME_LATE=%d INTERVAL_LATE=%d GOAL_SKIP=%d GOAL_ERROR=%d/%s DRAW_ERROR=%d/%s") % (
                           BUILD, fps, mode, ide, exposure, actual_exposure, bench, frame_ms, ready and not bench,
                           scanner.fast, mask if ready and not slow and not bench else 0, guard.coverage,
                           guard.blocked, guard.unknown, guard.escape_requested, guard.escape_away, escape if ready and not slow and not bench else 0,
                           nearest, sent, partial, len(lines), scanner.strict_hits, guard.closing_cm_s, guard.ttc_ms,
                           (clearance, block), comparison, late, interval_late, goal_skips, goal_errors, goal_error,
                           overlay.errors, overlay.last_error)
            radius = "-" if guard.line_xy is None else "%.1f" % mirror.pixel_radius(*guard.line_xy)
            output = ["LINE_ANGLE=%s LINE_DISTANCE=%s LINE_PIXEL_RADIUS=%s DISTANCE_VALID=%d DISTANCE_SOURCE=%s" %
                      (guard.line_angle, guard.line_distance, radius, guard.line_distance is not None,
                       "TABLE_ESTIMATE" if guard.line_distance is not None else "UNKNOWN")]
            output.append("LINE_CLASS=%s CLASS_CONFIDENCE=UNASSESSED CLASS_REASON=NO_VERIFIED_FIELD_LOCALIZATION" %
                          ("UNKNOWN_LINE" if lines else "NO_CURRENT_LINE"))
            dark, clipped, self_mask = vision.observation_reasons(scanner.reasons if ready else {})
            output.append("DARK_MASK=%04x CLIPPED_MASK=%04x SELF_MASK=%04x CONFIRMED_BLOCK=%04x "
                          "SUSPECTED_BLOCK=%04x HELD_BLOCK=%04x ESC_UNKNOWN=%04x ESC_LINE=%04x" %
                          (dark, clipped, self_mask, guard.confirmed_blocked, guard.ambiguous_blocked,
                           guard.held_blocked, guard.escape_away & guard.unknown, guard.escape_away & guard.blocked))
            if calibration_cm is not None:
                center = (mirror.cx, mirror.cy) if mirror.roi else None
                output.append("CALIBRATION_CM=%d MIRROR_CENTER=%s TARGETS=%d" % (calibration_cm, center, len(targets)))
                for _, _, xy, r, estimated, segment in targets:
                    output.append("TARGET_XY=%s PIXEL_RADIUS=%.2f ESTIMATED_CM=%s SEGMENT=%s DISTANCE_VALID=%d" %
                                  (xy, r, estimated, segment, segment is not None))
            output.append(timings.report())
            if command == b"i" or probing:
                output.append("GEOMETRY_READY=%d CENSORED=%s HELD=%s" %
                              (ready, sorted(scanner.censored) if ready else [], sorted(guard.near_history)))
                output.append("HELD_AGE_MS_AND_CLEAR_FRAMES=%s RELEASE_AFTER_MS=100 CLEAR_FRAMES=2" %
                              [(b, time.ticks_diff(now, last), count)
                               for b, (last, count) in sorted(guard.near_history.items())])
                rays = "".join("W" if b in guard.confirmed else "w" if b in lines else "O" if b in visible else "?"
                               for b in range(0, 360, 15))
                output.append("RAYS=%s WHITE_CM=%s REJECTED_SELF_RAYS=%s ROI=%s GOALS=%s COUNTS(raw,small,clip,ring,shape,turf,accepted)=%s" %
                              (rays, lines, [b for b in scanner.reasons if ready and scanner.reasons[b] & 4], last_roi, fields, (reports[0][:7], reports[1][:7])))
            output.append(summary)
            report_to_console(usb, ide, output)
            last_report, frames = now, 0
            timings.reset()


main()
