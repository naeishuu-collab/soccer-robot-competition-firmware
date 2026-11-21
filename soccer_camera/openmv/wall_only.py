"""Wall diagnostic and legacy 0x02/0x14 sender."""
from goal_wall_navigation import (Mirror, blob_value, cfg, draw_mirror, find_color_blob,
                                  init_camera, pyb, scan_field, sensor, time)

STREAM_INTERVAL_MS = 200
WALL_MIN_PIXELS = 150
WALL_KEEP_PIXELS = 100
USE_MIRROR = False  # Set True when the omnidirectional mirror is installed.
RECT_THRESHOLD = 10000


def find_rectangular_wall(img):
    rects = img.find_rects(threshold=RECT_THRESHOLD)
    counts = [0, 0, 0]
    best = None
    for blob in img.find_blobs([cfg.BLACK], pixels_threshold=WALL_KEEP_PIXELS,
                               area_threshold=WALL_KEEP_PIXELS):
        counts[0] += 1
        x, y, w, h = blob_value(blob, "rect")
        pixels = blob_value(blob, "pixels")
        touches_edge = x <= 0 or y <= 0 or x + w >= img.width() or y + h >= img.height()
        # A cropped wall may not have four visible corners.
        matched = touches_edge and pixels >= 0.55 * w * h
        for rect in rects:
            rx, ry = blob_value(rect, "x"), blob_value(rect, "y")
            rw, rh = blob_value(rect, "w"), blob_value(rect, "h")
            overlap = max(0, min(x + w, rx + rw) - max(x, rx)) * max(0, min(y + h, ry + rh) - max(y, ry))
            if overlap >= 0.5 * w * h and overlap >= 0.5 * rw * rh:
                matched = True
                break
        if not matched:
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
    while True:
        img = sensor.snapshot()
        wall, counts = (find_color_blob(img, mirror, [cfg.BLACK], WALL_KEEP_PIXELS)
                        if USE_MIRROR else find_rectangular_wall(img))
        tracked = track.update(wall, img.width(), img.height())
        visible, boundaries = scan_field(img, mirror) if USE_MIRROR and mirror.rays else ((), [])
        now = time.ticks_ms()
        if time.ticks_diff(now, last_stream) >= STREAM_INTERVAL_MS:
            last_stream = now
            wx, wy, size = tracked if tracked is not None else (255, 255, 0)
            uart.write(bytes((0x02, 255, 255, 255, 255, wx, wy)))
            uart.write(bytes((0x14, wx, wy, size >> 8, size & 255, 255, 255)))
            print("mode={} BLACK counts={} legacy_pixels={} green_rays={} edges={}".format(
                "mirror" if USE_MIRROR else "direct", counts, size, len(visible), [(angle, color, round(cm, 1)) for angle, color, cm, _, _ in boundaries]))
        if tracked is not None:
            img.draw_edges(blob_value(wall, "corners"), color=(255, 80, 80), thickness=3)
            img.draw_cross((round(track.x), round(track.y)), color=(255, 255, 0))
        for _, color, _, x, y in boundaries:
            img.draw_cross((x, y), color=(255, 80, 80) if color == "BLACK" else (255, 255, 255))
        if USE_MIRROR:
            draw_mirror(img, mirror)


if __name__ == "__main__":
    main()
