"""Generate the printable A4 grid sheet with ArUco markers.

    python make_grid_sheet.py                 # 60 mm cells -> grid_sheet_60mm.pdf
    python make_grid_sheet.py --cell 55       # 55 mm cells
    python make_grid_sheet.py --cell 50 -o my_sheet.pdf

Print at 100 % ("Actual size"), never "Fit to page". Then measure the
100 mm scale bar with a ruler.
"""
import argparse
import os

import cv2
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas

from sheet_layout import (BLOCK_SIZE, LAYOUTS, MARKER_DICT, MARKER_SIZE,
                          PAGE_H, PAGE_W, Layout)

HERE = os.path.dirname(os.path.abspath(__file__))


def marker_bits(marker_id):
    """6x6 bit matrix (1 = black) including the 1-bit black border."""
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, MARKER_DICT))
    img = cv2.aruco.generateImageMarker(d, marker_id, 6, borderBits=1)
    return [[1 if img[r, c] < 128 else 0 for c in range(6)] for r in range(6)]


def build(cell, out):
    L = Layout(cell)
    c = canvas.Canvas(out, pagesize=(PAGE_W * mm, PAGE_H * mm))
    c.setTitle(f"SO-101 pick grid, {cell} mm cells")

    def X(x):
        return x * mm

    def Y(y):          # sheet y runs down, PDF y runs up
        return (PAGE_H - y) * mm

    # ---- ArUco markers (vector squares, crisp at any printer) ----------
    c.setFillColorRGB(0, 0, 0)
    c.setStrokeColorRGB(0, 0, 0)
    bit = MARKER_SIZE / 6
    for slot, mid in enumerate(L.ids):
        ox, oy = L.marker_origin(slot)
        bits = marker_bits(mid)
        for r in range(6):
            for k in range(6):
                if bits[r][k]:
                    # tiny overlap avoids hairline gaps between squares
                    c.rect(X(ox + k * bit), Y(oy + (r + 1) * bit),
                           (bit + 0.05) * mm, (bit + 0.05) * mm, stroke=0, fill=1)
        c.setFillColorRGB(0.55, 0.55, 0.55)
        c.setFont("Helvetica", 6)
        label_y = oy + MARKER_SIZE + 3.2 if slot < 2 else oy - 1.6
        c.drawString(X(ox), Y(label_y), f"ID {mid}")
        c.setFillColorRGB(0, 0, 0)

    # ---- grid ---------------------------------------------------------
    c.setStrokeColorRGB(0.2, 0.2, 0.2)
    c.setLineWidth(0.8 * mm)
    for i in range(4):
        x = L.x0 + i * L.cell
        y = L.y0 + i * L.cell
        c.line(X(x), Y(L.y0), X(x), Y(L.y0 + L.side))
        c.line(X(L.x0), Y(y), X(L.x0 + L.side), Y(y))

    # cell numbers, block outline, centre cross
    for n in range(1, 10):
        x, y, w, h = L.cell_rect(n)
        cx, cy = L.cell_center(n)
        c.setFillColorRGB(0.5, 0.5, 0.5)
        c.setFont("Helvetica-Bold", 11)
        c.drawString(X(x + 3), Y(y + 6.5), str(n))
        half = BLOCK_SIZE / 2 + 1.0          # 1 mm play on each side
        c.setStrokeColorRGB(0.72, 0.72, 0.72)
        c.setLineWidth(0.3 * mm)
        c.setDash(2 * mm, 1.5 * mm)
        c.rect(X(cx - half), Y(cy + half), 2 * half * mm, 2 * half * mm, stroke=1, fill=0)
        c.setDash()
        c.line(X(cx - 2.5), Y(cy), X(cx + 2.5), Y(cy))
        c.line(X(cx), Y(cy - 2.5), X(cx), Y(cy + 2.5))

    # ---- 100 mm scale bar (top, between the markers) ----------------
    bx, by = (PAGE_W - 100) / 2, 24.0
    c.setStrokeColorRGB(0, 0, 0)
    c.setLineWidth(0.4 * mm)
    c.line(X(bx), Y(by), X(bx + 100), Y(by))
    for t in range(0, 101, 10):
        tick = 3.0 if t % 50 == 0 else 1.6
        c.line(X(bx + t), Y(by - tick), X(bx + t), Y(by))
    c.setFillColorRGB(0, 0, 0)
    c.setFont("Helvetica", 7.5)
    c.drawCentredString(X(PAGE_W / 2), Y(by + 5), "This bar must measure exactly 100 mm.")
    c.drawCentredString(X(PAGE_W / 2), Y(by + 9), "Print at 100 % (Actual size), not Fit to page.")

    # ---- robot side label (bottom) ------------------------------------
    c.setFillColorRGB(0.35, 0.35, 0.35)
    c.setFont("Helvetica-Bold", 10)
    c.drawCentredString(X(PAGE_W / 2), Y(PAGE_H - 30), "ROBOT SIDE")
    ax, ay = PAGE_W / 2, PAGE_H - 26
    c.setStrokeColorRGB(0.35, 0.35, 0.35)
    c.setLineWidth(0.5 * mm)
    c.line(X(ax), Y(ay), X(ax), Y(ay + 9))
    p = c.beginPath()
    p.moveTo(X(ax - 2.5), Y(ay + 6))
    p.lineTo(X(ax), Y(ay + 10))
    p.lineTo(X(ax + 2.5), Y(ay + 6))
    p.close()
    c.drawPath(p, stroke=0, fill=1)
    c.setFont("Helvetica", 6.5)
    c.drawCentredString(X(PAGE_W / 2), Y(PAGE_H - 13),
                        f"SO-101 pick grid, {cell} mm cells, {MARKER_DICT} IDs {L.ids[0]}-{L.ids[3]}")

    c.showPage()
    c.save()
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cell", type=int, default=60, choices=sorted(LAYOUTS))
    ap.add_argument("-o", "--out", default=None)
    a = ap.parse_args()
    out = a.out or os.path.join(HERE, f"grid_sheet_{a.cell}mm.pdf")
    print("wrote", build(a.cell, out))


if __name__ == "__main__":
    main()
