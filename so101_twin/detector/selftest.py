"""Self-test with synthetic photos (no robot, no camera, no printer needed).

Renders the grid sheet with 3D cubes through a pinhole camera from several
viewpoints, runs the full pipeline and checks every cube lands in its cell.

    python selftest.py                     # colour backend, quick
    python selftest.py --backend locate    # same photos through LocateAnything
    python selftest.py --save              # also write the synthetic photos to scans/
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np

from backends import make_backend
from locate_service import SCAN_DIR, print_summary, run_scan
from sheet_layout import BLOCK_SIZE, MARKER_DICT, MARKER_SIZE, PAGE_H, PAGE_W, Layout

TEX = 4.0   # texture px per mm
CUBE_BGR = {"red": (40, 40, 205), "green": (70, 160, 40), "blue": (190, 90, 20), "yellow": (30, 200, 235)}


def sheet_texture(L):
    w, h = int(PAGE_W * TEX), int(PAGE_H * TEX)
    img = np.full((h, w, 3), 245, np.uint8)
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, MARKER_DICT))
    s = int(MARKER_SIZE * TEX)
    for slot, mid in enumerate(L.ids):
        x, y = L.marker_origin(slot)
        m = cv2.aruco.generateImageMarker(d, mid, s, borderBits=1)
        img[int(y * TEX):int(y * TEX) + s, int(x * TEX):int(x * TEX) + s] = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
    for i in range(4):
        a = int((L.x0 + i * L.cell) * TEX)
        b = int((L.y0 + i * L.cell) * TEX)
        cv2.line(img, (a, int(L.y0 * TEX)), (a, int((L.y0 + L.side) * TEX)), (50, 50, 50), 3)
        cv2.line(img, (int(L.x0 * TEX), b), (int((L.x0 + L.side) * TEX), b), (50, 50, 50), 3)
    return img


def look_at(eye, target, up=(0, -1, 0)):
    eye, target, up = map(lambda v: np.asarray(v, float), (eye, target, up))
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, up)
    if np.linalg.norm(x) < 1e-6:
        x = np.cross(z, (1.0, 0, 0))
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.stack([x, y, z])          # world -> camera
    return R, -R @ eye


def render(L, cubes, eye, size=(1280, 960), f=820, seed=0):
    """World frame = sheet mm, z up out of the paper (towards the camera)."""
    W, Hh = size
    K = np.array([[f, 0, W / 2], [0, f, Hh / 2], [0, 0, 1.0]])
    target = (L.x0 + L.side / 2, L.y0 + L.side / 2, 0)
    # sheet frame: x right, y down, z into the table, so the camera has z < 0
    R, t = look_at(eye, target)
    rng = np.random.default_rng(seed)
    bg = np.clip(rng.normal(120, 12, (Hh, W, 3)), 0, 255).astype(np.uint8)
    bg = cv2.GaussianBlur(bg, (0, 0), 3)
    tex = sheet_texture(L)
    # plane z=0: image = K [r1 r2 t] [x y 1]; texture px -> mm is /TEX
    Hp = K @ np.column_stack([R[:, 0], R[:, 1], t]) @ np.diag([1 / TEX, 1 / TEX, 1])
    warped = cv2.warpPerspective(tex, Hp, (W, Hh), flags=cv2.INTER_AREA)
    mask = cv2.warpPerspective(np.full(tex.shape[:2], 255, np.uint8), Hp, (W, Hh))
    img = np.where(mask[..., None] > 0, warped, bg)

    def proj(p):
        pc = R @ np.asarray(p, float) + t
        return (K @ (pc / pc[2]))[:2]

    eye_v = np.asarray(eye, float)
    faces = []
    for color, (cx, cy) in cubes:
        h = BLOCK_SIZE / 2
        z = -BLOCK_SIZE          # cube top is 25 mm above the paper
        base = [(cx - h, cy - h, 0), (cx + h, cy - h, 0), (cx + h, cy + h, 0), (cx - h, cy + h, 0)]
        top = [(x, y, z) for x, y, _ in base]
        col = np.array(CUBE_BGR[color], float)
        for i in range(4):
            j = (i + 1) % 4
            quad = [base[i], base[j], top[j], top[i]]
            c = np.mean(quad, axis=0)
            faces.append((np.linalg.norm(c - eye_v), quad, col * 0.65))
        faces.append((np.linalg.norm(np.mean(top, axis=0) - eye_v) - 1, top, col))
    for _, quad, col in sorted(faces, key=lambda f: -f[0]):
        pts = np.array([proj(p) for p in quad], np.int32)
        cv2.fillConvexPoly(img, pts, tuple(int(v) for v in col), cv2.LINE_AA)
    img = cv2.GaussianBlur(img, (0, 0), 0.8)
    img = np.clip(img.astype(float) + rng.normal(0, 4, img.shape), 0, 255).astype(np.uint8)
    return img


def scenarios(L):
    c = {n: L.cell_center(n) for n in range(1, 10)}
    cx, cy = L.x0 + L.side / 2, L.y0 + L.side / 2
    j = lambda n, dx, dy: (c[n][0] + dx, c[n][1] + dy)
    return [
        ("straight down", (cx, cy, -520), [("red", c[1]), ("green", c[5]), ("blue", c[9]), ("yellow", c[3])]),
        ("from robot side, 25 deg", (cx, cy + 520 * math.tan(math.radians(25)), -520),
         [("red", c[7]), ("green", c[2]), ("blue", c[6]), ("yellow", c[4])]),
        ("from the side, 30 deg, rotated", (cx - 300, cy + 150, -480),
         [("red", c[8]), ("green", j(1, 6, -5)), ("blue", c[3]), ("yellow", j(5, -8, 8))]),
        ("close, 35 deg", (cx + 120, cy + 300, -380), [("red", c[9]), ("green", c[4]), ("blue", c[2])]),
    ]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", default="color", choices=["color", "locate"])
    ap.add_argument("--model", default="nvidia/LocateAnything-3B")
    ap.add_argument("--cell", type=int, default=60)
    ap.add_argument("--save", action="store_true")
    a = ap.parse_args()
    L = Layout(a.cell)
    backend = make_backend(a.backend, model=a.model)
    fails = 0
    for k, (name, eye, cubes) in enumerate(scenarios(L)):
        img = render(L, cubes, eye, seed=k)
        if a.save:
            os.makedirs(SCAN_DIR, exist_ok=True)
            cv2.imwrite(os.path.join(SCAN_DIR, f"selftest_{k}.jpg"), img)
        res = run_scan(img, backend, [c for c, _ in cubes], save=a.save)
        want = {str(L.locate(*p)[0]): col for col, p in cubes}
        got = {n: col for n, col in (res.get("cells") or {}).items() if col}
        ok = res["ok"] and got == want
        fails += not ok
        print(f"[{'PASS' if ok else 'FAIL'}] {name}")
        if not ok:
            print("   want", want, "\n   got ", got)
        print_summary(res)
    print("all passed" if not fails else f"{fails} failed")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
