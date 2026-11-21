"""Wall diagnostic and legacy 0x02/0x14 sender."""
from goal_wall_navigation import (Mirror, blob_value, cfg, draw_mirror, find_color_blob,
                                  init_camera, pyb, scan_field, sensor, time)

STREAM_INTERVAL_MS = 200
WALL_MIN_PIXELS = 150
WALL_KEEP_PIXELS = 100


class WallTrack:
    def __init__(self):
        self.x = self.y = None

    def update(self, blob, width, height):
        if blob is None:
            self.x = self.y = None
            return None
        x, y = blob_value(blob, "cx"), blob_value(blob, "cy")
        pixels = blob_value(blob, "pixels")
        continuous = (self.x is not None and
                      ((x - self.x) / width) ** 2 + ((y - self.y) / height) ** 2 <= 0.0625)
        if pixels < (WALL_KEEP_PIXELS if continuous else WALL_MIN_PIXELS):
            self.x = self.y = None
            return None
        if continuous:
            self.x += 0.4 * (x - self.x)
            self.y += 0.4 * (y - self.y)
        else:
            self.x, self.y = x, y
        return (max(0, min(255, round(self.x * 255 / width))),
                max(0, min(255, round(self.y * 255 / height))), min(65535, pixels))


def main():
    init_camera()
    frame = sensor.snapshot()
    mirror = Mirror(frame.width(), frame.height())
    uart = pyb.UART(3, 9600)
    last_stream = time.ticks_ms()
    track = WallTrack()
    while True:
        img = sensor.snapshot()
        wall, counts = find_color_blob(img, mirror, [cfg.BLACK], WALL_KEEP_PIXELS)
        tracked = track.update(wall, img.width(), img.height())
        visible, boundaries = scan_field(img, mirror) if mirror.rays else ((), [])
        now = time.ticks_ms()
        if time.ticks_diff(now, last_stream) >= STREAM_INTERVAL_MS:
            last_stream = now
            wx, wy, size = tracked if tracked is not None else (255, 255, 0)
            uart.write(bytes((0x02, 255, 255, 255, 255, wx, wy)))
            uart.write(bytes((0x14, wx, wy, size >> 8, size & 255, 255, 255)))
            print("BLACK components/small/masked={} legacy_pixels={} green_rays={} edges={}".format(
                counts, size, len(visible), [(angle, color, round(cm, 1)) for angle, color, cm, _, _ in boundaries]))
        if tracked is not None:
            img.draw_edges(blob_value(wall, "corners"), color=(255, 80, 80), thickness=3)
            img.draw_cross((round(track.x), round(track.y)), color=(255, 255, 0))
        for _, color, _, x, y in boundaries:
            img.draw_cross((x, y), color=(255, 80, 80) if color == "BLACK" else (255, 255, 255))
        draw_mirror(img, mirror)


if __name__ == "__main__":
    main()
