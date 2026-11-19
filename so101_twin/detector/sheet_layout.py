"""Geometry of the printed A4 grid sheet.

One source of truth for the PDF generator and the detector, so they can
never disagree.

Sheet coordinates are millimetres. Origin is the top-left corner of the
A4 page held upright (portrait), x to the right, y down. The bottom edge
of the page faces the robot. Seen from behind the robot, the page reads
upright: cells 1-2-3 are the far row, cells 7-8-9 the near row.

The marker IDs tell the detector which layout is in the photo, so a 55 mm
sheet can never be read as a 60 mm sheet by mistake.
"""

PAGE_W = 210.0
PAGE_H = 297.0

MARKER_DICT = "DICT_4X4_50"   # OpenCV ArUco dictionary
MARKER_SIZE = 35.0            # black square edge, mm
MARKER_MARGIN = 12.0          # page edge to marker, mm

# cell size (mm) -> marker IDs (top-left, top-right, bottom-right, bottom-left)
LAYOUTS = {
    60: (0, 1, 2, 3),
    55: (4, 5, 6, 7),
    50: (8, 9, 10, 11),
}
DEFAULT_CELL = 60

BLOCK_SIZE = 25.0             # cube edge, mm (placement outline on the sheet)


class Layout:
    def __init__(self, cell_mm=DEFAULT_CELL):
        if cell_mm not in LAYOUTS:
            raise ValueError(f"unknown cell size {cell_mm}, use one of {sorted(LAYOUTS)}")
        self.cell = float(cell_mm)
        self.ids = LAYOUTS[cell_mm]
        self.side = 3 * self.cell
        self.x0 = (PAGE_W - self.side) / 2
        self.y0 = (PAGE_H - self.side) / 2

    # ---- markers ---------------------------------------------------
    def marker_origin(self, slot):
        """Top-left corner (mm) of the marker in slot 0..3 (TL, TR, BR, BL)."""
        m, s = MARKER_MARGIN, MARKER_SIZE
        left, right = m, PAGE_W - m - s
        top, bottom = m, PAGE_H - m - s
        return [(left, top), (right, top), (right, bottom), (left, bottom)][slot]

    def marker_corners(self, marker_id):
        """The 4 corners (mm) in ArUco order: TL, TR, BR, BL."""
        slot = self.ids.index(marker_id)
        x, y = self.marker_origin(slot)
        s = MARKER_SIZE
        return [(x, y), (x + s, y), (x + s, y + s), (x, y + s)]

    # ---- cells -----------------------------------------------------
    def cell_rect(self, n):
        """(x, y, w, h) in mm of cell n = 1..9."""
        r, c = divmod(n - 1, 3)
        return (self.x0 + c * self.cell, self.y0 + r * self.cell, self.cell, self.cell)

    def cell_center(self, n):
        x, y, w, h = self.cell_rect(n)
        return (x + w / 2, y + h / 2)

    def locate(self, x, y):
        """Map a sheet point (mm) to (cell or None, margin_mm).

        margin_mm is the distance to the nearest cell border, so a small
        value means the call is close and may be wrong.
        """
        u, v = x - self.x0, y - self.y0
        if not (0 <= u < self.side and 0 <= v < self.side):
            return None, 0.0
        c, r = int(u // self.cell), int(v // self.cell)
        du = min(u - c * self.cell, (c + 1) * self.cell - u)
        dv = min(v - r * self.cell, (r + 1) * self.cell - v)
        return r * 3 + c + 1, min(du, dv)

    def describe(self):
        return {
            "cell_mm": self.cell,
            "grid_mm": [self.x0, self.y0, self.side, self.side],
            "marker_ids": list(self.ids),
            "marker_mm": MARKER_SIZE,
        }


def layout_for_ids(ids):
    """Pick the layout whose marker IDs appear most often among `ids`."""
    best, count = None, 0
    for cell, lids in LAYOUTS.items():
        k = sum(1 for i in ids if i in lids)
        if k > count:
            best, count = cell, k
    return (Layout(best), count) if best else (None, 0)
