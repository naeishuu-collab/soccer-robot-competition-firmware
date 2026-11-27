"""床と白線の走査の高速版。goal_wall_navigation.scan_field() と同じ結果を、画素ごとの確保なしで返す。

仕組み: 起動時にRGB565の全65536色を一度だけ分類して表にし、走査点の画素位置も一度だけ計算する。
毎フレームは viper の関数が画像バッファを直接読む。使う前に実フレームで従来版と一致することを確認し、
一致しなければ呼び出し側は従来版を使い続ける。
"""
import sys
from array import array

try:
    import micropython
except ImportError:
    class micropython:  # host: the code runs as ordinary Python
        native = staticmethod(lambda function: function)
        viper = staticmethod(lambda function: function)

if sys.implementation.name != "micropython":
    # Inside viper functions ptr8/16/32 are built in; these stand in for them on a PC.
    def ptr8(buffer):
        return buffer

    def ptr16(buffer):
        return memoryview(buffer).cast("H")

    def ptr32(buffer):
        return buffer

# Classes in the table. 0 is never stored.
GREEN, WHITE, BLACK, OTHER = 1, 2, 3, 4
CODES = {"GREEN": GREEN, "WHITE": WHITE, "BLACK": BLACK, None: OTHER}
NAMES = (None, "GREEN", "WHITE", "BLACK", None)
SHADES = 65536
ROW = 256  # colours written into one image row while building the table


@micropython.viper
def fill_row(frame, base: int):
    pixels = ptr16(frame)
    i = 0
    while i < 256:
        pixels[i] = base + i
        i += 1


@micropython.viper
def code_at(frame, lut, offset: int) -> int:
    pixels = ptr16(frame)
    table = ptr8(lut)
    return table[pixels[offset]]


@micropython.viper
def scan_kernel(frame, packed, lut, span: int) -> int:
    """scan_ray()と同じ状態遷移。packed[i] = 画素位置*2 + (回避距離以内なら1)。span = 開始 | 終了<<16。

    戻り値 = 床が見える(0/1) | (確認した白線の始点の番号+1) << 1（線がなければ0）。
    """
    pixels = ptr16(frame)
    samples = ptr32(packed)
    table = ptr8(lut)
    start = span & 0xFFFF
    end = span >> 16
    green_run = 0
    ground = 0
    previous = -1
    bridge = 0
    i = start
    while i < end:
        sample = samples[i]
        color = int(table[pixels[sample >> 1]])
        index = i - start
        if sample & 1:
            if color != 1:
                ground = 0
        if previous >= 0:
            if color == 2:  # a second white sample confirms the boundary
                return ground | ((previous + 1) << 1)
        previous = -1
        if color == 2:
            if green_run >= 2:
                previous = index
            elif bridge:
                previous = index
        # A mixed edge pixel may be neither turf nor white.
        bridge = 0
        if color == 4:
            if green_run >= 2:
                bridge = 1
        if color == 1:
            green_run += 1
        else:
            green_run = 0
        if index == 1:
            if green_run >= 2:
                ground = 1
            else:
                ground = 0
        i += 1
    return ground


def build_table(img, classify):
    """画像の1行を書き換えながら全RGB565色を分類する。classify(rgb) は "GREEN" などを返す。

    get_pixel() が読む色をそのまま分類するので、バイト順などの前提を持たない。
    """
    if img.width() < ROW:
        raise ValueError("image too narrow")
    table = bytearray(SHADES)
    buffer = img.bytearray()
    get_pixel = img.get_pixel
    for base in range(0, SHADES, ROW):
        fill_row(buffer, base)
        for x in range(ROW):
            table[base + x] = CODES[classify(get_pixel((x, 0), rgbtuple=True))]
    return table


class FastScanner:
    def __init__(self, rays, width, height, table, clearance_cm, is_thin):
        """rays: Mirror.rays。is_thin: goal_wall_navigation.line_is_thin。"""
        self.table = table
        self.width = width
        self.height = height
        self.rays = rays
        self.is_thin = is_thin
        packed = []
        self.spans = []
        for bearing, samples in rays:
            start = len(packed)
            for x, y, cm in samples:
                if not (0 <= x < width and 0 <= y < height):
                    raise ValueError("sample outside the image")
                packed.append((y * width + x) * 2 + (1 if cm <= clearance_cm else 0))
            self.spans.append((bearing, samples, start | (len(packed) << 16)))
        self.packed = array("i", packed)
        self.buffer = None

    def color_at(self, x, y):
        return NAMES[code_at(self.buffer, self.table, y * self.width + x)]

    def scan(self, img, thin=None):
        """scan_field(img, mirror, thin=thin) と同じ (visible, boundaries)。"""
        buffer = self.buffer = img.bytearray()  # the frame buffer address changes between frames
        packed, table, kernel = self.packed, self.table, scan_kernel
        visible = set()
        boundaries = []
        for bearing, samples, span in self.spans:
            result = kernel(buffer, packed, table, span)
            if result & 1:
                visible.add(bearing)
            if result > 1:
                x, y, cm = samples[(result >> 1) - 1]
                edge = ("WHITE", cm, x, y)
                boundaries.append((bearing, "WHITE", cm, x, y))
                if thin is not None and self.is_thin(samples, edge, self.color_at):
                    thin.add(bearing)
        return visible, boundaries

    def matches(self, img, classify, reference):
        """実フレームで、全走査点の色と走査結果が従来版と一致するか。reference(img, thin) は従来版。"""
        self.buffer = img.bytearray()
        for _, samples, _ in self.spans:
            for x, y, _ in samples:
                if self.color_at(x, y) != classify(img.get_pixel((x, y), rgbtuple=True)):
                    return False
        fast_thin, slow_thin = set(), set()
        return self.scan(img, fast_thin) == reference(img, slow_thin) and fast_thin == slow_thin
