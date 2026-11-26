"""Wall diagnostic and legacy 0x02/0x14 sender."""
from goal_wall_navigation import (Mirror, blob_value, cfg, draw_mirror, find_color_blob,
                                  init_camera, pyb, scan_field, sensor, time)

STREAM_INTERVAL_MS = 200
WALL_MIN_PIXELS = 150
WALL_KEEP_PIXELS = 100
USE_MIRROR = False  # Set True when the omnidirectional mirror is installed.
SHOW_FIELD_EDGES = False  # Enable calibrated floor-ray diagnostics when needed.
LOG_INTERVAL_MS = 1000


def find_rectangular_wall(img):
    counts = [0, 0, 0]
    best = None
    for blob in img.find_blobs([cfg.BLACK], pixels_threshold=WALL_KEEP_PIXELS,
                               area_threshold=WALL_KEEP_PIXELS):
        counts[0] += 1
        pixels = blob_value(blob, "pixels")
        corners = blob_value(blob, "corners")
        area2 = abs(sum(corners[i][0] * corners[(i + 1) % 4][1] -
                        corners[(i + 1) % 4][0] * corners[i][1] for i in range(4)))
        # shortcut: dark rectangular objects also pass, add field context after calibration.
        if area2 == 0 or 2 * pixels < 0.55 * area2:
            counts[2] += 1
        elif best is None or pixels > blob_value(best, "pixels"):
            best = blob
    return best, counts


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
    last_log = last_stream
    frames = 0
    while True:
        img = sensor.snapshot()
        frames += 1
        wall, counts = (find_color_blob(img, mirror, [cfg.BLACK], WALL_KEEP_PIXELS)
                        if USE_MIRROR else find_rectangular_wall(img))
        tracked = track.update(wall, img.width(), img.height())
        visible, boundaries = (), []
        if USE_MIRROR and SHOW_FIELD_EDGES and mirror.rays:
            visible, boundaries = scan_field(img, mirror, include_wall=True)
        now = time.ticks_ms()
        if time.ticks_diff(now, last_stream) >= STREAM_INTERVAL_MS:
            last_stream = now
            wx, wy, size = tracked if tracked is not None else (255, 255, 0)
            uart.write(bytes((0x02, 255, 255, 255, 255, wx, wy)))
            uart.write(bytes((0x14, wx, wy, size >> 8, size & 255, 255, 255)))
        elapsed = time.ticks_diff(now, last_log)
        if elapsed >= LOG_INTERVAL_MS:
            print("mode={} fps={:.1f} BLACK counts={} green_rays={} edges={}".format(
                "mirror" if USE_MIRROR else "direct", frames * 1000 / elapsed, counts,
                len(visible), len(boundaries)))
            last_log = now
            frames = 0
        if tracked is not None:
            img.draw_edges(blob_value(wall, "corners"), color=(255, 80, 80), thickness=3)
            img.draw_cross((round(track.x), round(track.y)), color=(255, 255, 0))
        for _, color, _, x, y in boundaries:
            img.draw_cross((x, y), color=(255, 80, 80) if color == "BLACK" else (255, 255, 255))
        if USE_MIRROR:
            draw_mirror(img, mirror)


if __name__ == "__main__":
    main()
