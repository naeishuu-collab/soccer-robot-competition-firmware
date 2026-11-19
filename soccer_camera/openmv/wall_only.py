"""Wall diagnostic and legacy 0x02/0x14 sender."""
from goal_wall_navigation import (Mirror, blob_value, cfg, draw_mirror, find_color_blob,
                                  init_camera, pyb, scan_field, sensor, time)

STREAM_INTERVAL_MS = 200
WALL_MIN_PIXELS = 150


def main():
    init_camera()
    frame = sensor.snapshot()
    mirror = Mirror(frame.width(), frame.height())
    uart = pyb.UART(3, 9600)
    last_stream = time.ticks_ms()
    while True:
        img = sensor.snapshot()
        wall, counts = find_color_blob(img, mirror, [cfg.BLACK], WALL_MIN_PIXELS)
        visible, boundaries = scan_field(img, mirror) if mirror.rays else ((), [])
        now = time.ticks_ms()
        if time.ticks_diff(now, last_stream) >= STREAM_INTERVAL_MS:
            last_stream = now
            if wall is None:
                wx = wy = 255
                size = 0
            else:
                wx = blob_value(wall, "cx") * 255 // img.width()
                wy = blob_value(wall, "cy") * 255 // img.height()
                size = min(65535, blob_value(wall, "pixels"))
            uart.write(bytes((0x02, 255, 255, 255, 255, wx, wy)))
            uart.write(bytes((0x14, wx, wy, size >> 8, size & 255, 255, 255)))
            print("BLACK components/small/masked={} legacy_pixels={} green_rays={} edges={}".format(
                counts, size, len(visible), [(angle, color, round(cm, 1)) for angle, color, cm, _, _ in boundaries]))
        if wall is not None:
            img.draw_edges(blob_value(wall, "corners"), color=(255, 80, 80), thickness=3)
        for _, color, _, x, y in boundaries:
            img.draw_cross(x, y, color=(255, 80, 80) if color == "BLACK" else (255, 255, 255))
        draw_mirror(img, mirror)


if __name__ == "__main__":
    main()
