"""Find the sheet in a photo and map image points to grid cells.

The table is flat, so 4 known points are enough to map any photo pixel to
millimetres on the sheet (a homography). Each ArUco marker gives 4 known
corners. No camera calibration and no camera pose are needed, so this
works with a moving head-mounted camera.
"""
import cv2
import numpy as np

from sheet_layout import MARKER_DICT, Layout, layout_for_ids

PX_PER_MM = 4.0      # resolution of the rectified (top-down) grid image
PAD_MM = 8.0         # border kept around the 3x3 grid in that image
UNSURE_MM = 6.0      # a point closer than this to a cell border is flagged


def _detector():
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, MARKER_DICT))
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(d, p)


_DET = _detector()


class SheetView:
    """The sheet as seen in one photo."""

    def __init__(self, layout, H_img2sheet, markers, reproj_mm):
        self.layout = layout
        self.H = H_img2sheet
        self.markers = markers          # {id: 4x2 image corners}
        self.reproj_mm = reproj_mm      # fit error, mm (0 if exactly determined)
        L = layout
        self.rect_origin = (L.x0 - PAD_MM, L.y0 - PAD_MM)
        size = int(round((L.side + 2 * PAD_MM) * PX_PER_MM))
        self.rect_size = (size, size)
        S = np.array([[PX_PER_MM, 0, -self.rect_origin[0] * PX_PER_MM],
                      [0, PX_PER_MM, -self.rect_origin[1] * PX_PER_MM],
                      [0, 0, 1]], dtype=np.float64)
        self.H_img2rect = S @ self.H

    # ---- coordinate helpers ---------------------------------------------
    def rect_to_sheet(self, u, v):
        return (self.rect_origin[0] + u / PX_PER_MM, self.rect_origin[1] + v / PX_PER_MM)

    def sheet_to_rect(self, x, y):
        return ((x - self.rect_origin[0]) * PX_PER_MM, (y - self.rect_origin[1]) * PX_PER_MM)

    def img_to_sheet(self, pts):
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(pts, self.H).reshape(-1, 2)

    def sheet_to_img(self, pts):
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(pts, np.linalg.inv(self.H)).reshape(-1, 2)

    def rectify(self, img):
        """Top-down image of the grid area, PX_PER_MM pixels per mm."""
        return cv2.warpPerspective(img, self.H_img2rect, self.rect_size,
                                   flags=cv2.INTER_LINEAR, borderValue=(255, 255, 255))

    def cell_of(self, x_mm, y_mm):
        cell, margin = self.layout.locate(x_mm, y_mm)
        return cell, margin, (cell is not None and margin < UNSURE_MM)


def find_sheet(img, cell_mm=None):
    """Detect markers and fit the homography.

    Returns (SheetView or None, info dict). Needs at least 1 marker of the
    layout, 2 or more is much more accurate across the whole grid.
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    corners, ids, _ = _DET.detectMarkers(gray)
    found = [] if ids is None else [int(i) for i in ids.ravel()]
    info = {"markers_found": sorted(found), "warnings": []}
    if not found:
        info["error"] = "no ArUco markers found"
        return None, info

    if cell_mm:
        layout = Layout(cell_mm)
    else:
        layout, _ = layout_for_ids(found)
        if layout is None:
            info["error"] = f"markers {sorted(found)} do not belong to any grid sheet"
            return None, info

    img_pts, sheet_pts, markers = [], [], {}
    for c, i in zip(corners, found):
        if i not in layout.ids or i in markers:
            continue
        markers[i] = c.reshape(4, 2)
        img_pts.extend(markers[i].tolist())
        sheet_pts.extend(layout.marker_corners(i))
    info["markers_used"] = sorted(markers)
    info["layout"] = layout.describe()
    if not markers:
        info["error"] = f"no markers of the {int(layout.cell)} mm sheet found"
        return None, info

    src = np.array(img_pts, dtype=np.float64)
    dst = np.array(sheet_pts, dtype=np.float64)
    if len(markers) == 1:
        H, _ = cv2.findHomography(src, dst, 0)
        info["warnings"].append("only 1 marker visible: cell mapping far from it is less accurate")
    else:
        H, _ = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
    if H is None:
        info["error"] = "homography fit failed"
        return None, info

    proj = cv2.perspectiveTransform(src.reshape(-1, 1, 2), H).reshape(-1, 2)
    err = float(np.sqrt(np.mean(np.sum((proj - dst) ** 2, axis=1))))
    info["reproj_mm"] = round(err, 2)
    if err > 2.0:
        info["warnings"].append(f"marker fit error {err:.1f} mm: is the sheet flat and printed at 100 %?")
    return SheetView(layout, H, markers, err), info


# ---- drawing ---------------------------------------------------------------
BGR = {
    "red": (40, 40, 220), "green": (60, 170, 60), "blue": (210, 110, 30),
    "yellow": (0, 210, 240), "orange": (0, 140, 255), "purple": (170, 60, 150),
    "white": (235, 235, 235), "black": (30, 30, 30), "pink": (180, 120, 255),
}


def color_bgr(name):
    return BGR.get(name, (128, 128, 128))


def draw_rectified(rect, view, blocks):
    """Annotate the top-down image: grid, cell numbers, detections."""
    out = rect.copy()
    L = view.layout
    for i in range(4):
        a = view.sheet_to_rect(L.x0 + i * L.cell, L.y0)
        b = view.sheet_to_rect(L.x0 + i * L.cell, L.y0 + L.side)
        cv2.line(out, tuple(map(int, a)), tuple(map(int, b)), (255, 0, 255), 1, cv2.LINE_AA)
        a = view.sheet_to_rect(L.x0, L.y0 + i * L.cell)
        b = view.sheet_to_rect(L.x0 + L.side, L.y0 + i * L.cell)
        cv2.line(out, tuple(map(int, a)), tuple(map(int, b)), (255, 0, 255), 1, cv2.LINE_AA)
    for n in range(1, 10):
        x, y, _, _ = L.cell_rect(n)
        u, v = view.sheet_to_rect(x + 3, y + 12)
        cv2.putText(out, str(n), (int(u), int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2, cv2.LINE_AA)
    for b in blocks:
        col = color_bgr(b["color"])
        if b.get("box_rect"):
            x1, y1, x2, y2 = map(int, b["box_rect"])
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 0, 0), 4)
            cv2.rectangle(out, (x1, y1), (x2, y2), col, 2)
        u, v = map(int, b["point_rect"])
        cv2.circle(out, (u, v), 7, (0, 0, 0), -1)
        cv2.circle(out, (u, v), 5, col, -1)
        label = f'{b["color"]} -> {b["cell"] if b["cell"] else "off grid"}'
        if b.get("unsure"):
            label += " ?"
        cv2.putText(out, label, (u + 10, v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(out, label, (u + 10, v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def draw_photo(img, view, info):
    """Annotate the original photo: markers used and the grid outline."""
    out = img.copy()
    if view is None:
        for line, txt in enumerate([info.get("error", "")]):
            cv2.putText(out, txt, (20, 40 + 30 * line), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        return out
    for i, c in view.markers.items():
        pts = c.astype(int).reshape(-1, 1, 2)
        cv2.polylines(out, [pts], True, (0, 255, 0), 3)
        cv2.putText(out, f"ID {i}", tuple(c[0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    L = view.layout
    for i in range(4):
        for a, b in (((L.x0 + i * L.cell, L.y0), (L.x0 + i * L.cell, L.y0 + L.side)),
                     ((L.x0, L.y0 + i * L.cell), (L.x0 + L.side, L.y0 + i * L.cell))):
            p = view.sheet_to_img([a, b]).astype(int)
            cv2.line(out, tuple(p[0]), tuple(p[1]), (255, 0, 255), 2, cv2.LINE_AA)
    return out
