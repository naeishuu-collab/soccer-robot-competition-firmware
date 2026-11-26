"""LAB and exposure diagnostics. Copy both competition Python modules beside this script."""
from goal_wall_navigation import (GOAL_COLORS, Mirror, ball_like, blob_value, cfg, draw_goal, draw_mirror,
                                  find_color_blob, init_camera, sensor, time)

# Optional named ROIs in raw image pixels. Sample the inside of each target.
SAMPLE_ROIS = ()  # (("yellow", (x, y, width, height)), ...)
PRINT_INTERVAL_MS = 500


def main():
    init_camera()
    img = sensor.snapshot()
    width, height = img.width(), img.height()
    mirror = Mirror(width, height, img)
    clock = time.clock()
    last_print = time.ticks_ms()
    print("frame={}x{} ring={} center=({}, {}) radii=({}, {}) auto={} ground={}".format(
        width, height, mirror.ring_ready, mirror.cx, mirror.cy, mirror.inner, mirror.outer,
        mirror.detected, bool(mirror.rays)))
    while True:
        clock.tick()
        img = sensor.snapshot()
        now = time.ticks_ms()
        log = time.ticks_diff(now, last_print) >= PRINT_INTERVAL_MS
        detections = []
        for name, thresholds in (("YELLOW", cfg.YELLOW), ("BLUE", cfg.BLUE)):
            blob, counts = find_color_blob(img, mirror, thresholds,
                                           getattr(cfg, "BLUE_MIN_PIXELS", 8) if name == "BLUE" else
                                           getattr(cfg, "YELLOW_MIN_PIXELS", 12), None, name == "YELLOW")
            if blob is not None:
                detections.append((name, blob))
            if log:
                rect = blob_value(blob, "rect") if blob is not None else None
                print("{} components/small/masked={} selected={}".format(name, counts, rect))
        ball_thresholds = getattr(cfg, "BALL", None)
        ball, ball_counts = (find_color_blob(img, mirror, ball_thresholds, 10, ball_like, True)
                             if ball_thresholds else (None, [0, 0, 0]))
        if log:
            print("BALL components/small/masked={} selected={}".format(
                ball_counts, blob_value(ball, "rect") if ball is not None else None))
        if ball is not None:
            detections.append(("BALL", ball))
        if log:
            last_print = now
            print("fps={:.1f} exposure_us={} gain_db={} rgb_gain={}".format(
                clock.fps(), sensor.get_exposure_us(), sensor.get_gain_db(), sensor.get_rgb_gain_db()))
            for name, rect in SAMPLE_ROIS:
                x, y, w, h = rect
                if x < 0 or y < 0 or w <= 0 or h <= 0 or x + w > width or y + h > height:
                    print("{}: ROI outside image".format(name))
                    continue
                histogram = img.get_histogram(roi=rect)
                lo, hi = histogram.get_percentile(0.05), histogram.get_percentile(0.95)
                print("{}: LAB p05–p95 {}".format(
                    name, tuple(value for channel in ("l_value", "a_value", "b_value")
                                for value in (blob_value(lo, channel), blob_value(hi, channel)))))
        for name, rect in SAMPLE_ROIS:
            x, y, w, h = rect
            if x >= 0 and y >= 0 and w > 0 and h > 0 and x + w <= width and y + h <= height:
                img.draw_rectangle(rect, color=GOAL_COLORS.get(name.upper(), (255, 255, 255)))
        for name, blob in detections:
            draw_goal(img, blob, name)
        draw_mirror(img, mirror)


if __name__ == "__main__":
    main()
