"""Block detector service for the SO-101 pick demo.

Photo in, "which colour block is in which of the 9 cells" out.

    # one photo from the command line
    python locate_service.py --image photo.jpg
    # HTTP service with a test page at http://127.0.0.1:8002
    python locate_service.py
    # no GPU model, plain colour thresholds (fallback / testing)
    python locate_service.py --backend color

Pipeline: ArUco markers -> homography -> top-down grid image ->
LocateAnything (one query per colour) -> sheet mm -> cell 1..9.
"""
import argparse
import base64
import datetime as dt
import json
import os
import re
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

from backends import make_backend, measured_color
from vision import PX_PER_MM, draw_photo, draw_rectified, find_sheet

HERE = os.path.dirname(os.path.abspath(__file__))
SCAN_DIR = os.path.join(HERE, "scans")
PAGE = os.path.join(HERE, "web", "detector.html")
DEFAULT_COLORS = ["green", "yellow", "blue", "red"]   # demo set: 3 green, 2 yellow, 2 blue, 2 red
# The exact model version tested on 19 Nov 2026. Pinning it means a later
# update by NVIDIA cannot change the detector's behaviour before the demo.
MODEL_REVISION = "c32291ca5e996f5a7a485845b4f57a233936bba0"
MAX_BLOCK_MM = 70.0       # a detection box larger than this is not one cube
MIN_BLOCK_MM = 10.0
MAX_BODY = 25 * 2**20
_SCAN_RE = re.compile(r"^scan_[0-9_]+_(photo|grid|annotated|result)\.(jpg|json)$")


def decode_image(data):
    arr = np.frombuffer(data, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("could not decode image (send JPEG or PNG)")
    return img


def parse_expect(text):
    """'blue:3,black:2,yellow:1' -> {'blue': 3, 'black': 2, 'yellow': 1}"""
    if not text:
        return None
    if isinstance(text, dict):
        return {str(k).strip().lower(): int(v) for k, v in text.items()}
    out = {}
    for part in str(text).split(","):
        if part.strip():
            c, n = part.split(":")
            out[c.strip().lower()] = int(n)
    return out or None


def run_scan(img, backend, colors=None, cell_mm=None, save=True, expect=None):
    """expect: optional {colour: count}. Then the answer says whether the counts match."""
    colors = [c.strip().lower() for c in (colors or DEFAULT_COLORS) if c.strip()]
    expect = parse_expect(expect)
    if expect:                       # always look for every colour we expect
        colors += [c for c in expect if c not in colors]
    t0 = time.time()
    res = {"ok": False, "backend": backend.name, "colors": colors,
           "image_size": [int(img.shape[1]), int(img.shape[0])], "warnings": []}
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    files = {}

    def store(kind, image):
        if save:
            os.makedirs(SCAN_DIR, exist_ok=True)
            name = f"scan_{stamp}_{kind}.jpg"
            cv2.imwrite(os.path.join(SCAN_DIR, name), image, [cv2.IMWRITE_JPEG_QUALITY, 90])
            files[kind] = name

    store("photo", img)
    view, info = find_sheet(img, cell_mm)
    res.update({k: v for k, v in info.items() if k != "warnings"})
    res["warnings"] += info["warnings"]
    t_sheet = time.time()
    if view is None:
        store("annotated", draw_photo(img, None, info))
        res["files"], res["timing_s"] = files, {"total": round(time.time() - t0, 3)}
        return res

    rect = view.rectify(img)
    store("grid", rect)
    dets = backend.detect(rect, colors)
    t_det = time.time()

    blocks, rejected, raw = [], [], {}
    for d in dets:
        raw.setdefault(d["color"], d.get("raw", ""))
        if d.get("point") is None:
            continue
        u, v = d["point"]
        x, y = view.rect_to_sheet(u, v)
        b = {"color": d["color"], "point_rect": [round(u, 1), round(v, 1)],
             "sheet_mm": [round(x, 1), round(y, 1)],
             "box_rect": [round(t, 1) for t in d["box"]] if d.get("box") else None}
        if d.get("box"):
            wmm = (d["box"][2] - d["box"][0]) / PX_PER_MM
            hmm = (d["box"][3] - d["box"][1]) / PX_PER_MM
            b["size_mm"] = [round(wmm, 1), round(hmm, 1)]
            if max(wmm, hmm) > MAX_BLOCK_MM or max(wmm, hmm) < MIN_BLOCK_MM:
                b["reason"] = "box size not a 25 mm cube"
                rejected.append(b)
                continue
        cell, margin, unsure = view.cell_of(x, y)
        b.update(cell=cell, margin_mm=round(margin, 1), unsure=bool(unsure))
        b["seen_color"] = measured_color(rect, d.get("box"), d["point"])
        if cell is None:
            b["reason"] = "outside the 3x3 grid"
            rejected.append(b)
            continue
        blocks.append(b)

    # One block per cell. Several boxes of the same colour in one cell are the
    # same block seen twice: keep the most central one, no warning. Different
    # colours in one cell: trust the measured pixel colour, and warn.
    by_cell = {}
    for b in blocks:
        by_cell.setdefault(b["cell"], []).append(b)
    final = []
    for cell, claims in sorted(by_cell.items()):
        claims.sort(key=lambda b: -b["margin_mm"])
        if len({b["color"] for b in claims}) == 1:
            keep = claims[0]
        else:
            match = [b for b in claims if b["seen_color"] == b["color"]]
            keep = match[0] if match else claims[0]
            res["warnings"].append(f"cell {cell}: {', '.join(sorted({b['color'] for b in claims}))} "
                                   f"all claimed it, kept {keep['color']}")
        final.append(keep)
        for b in claims:
            if b is not keep:
                b["reason"] = f"cell {cell} already holds {keep['color']}"
                rejected.append(b)

    for b in final:
        if b["unsure"]:
            res["warnings"].append(f"{b['color']} in cell {b['cell']} is {b['margin_mm']} mm from a border, check it")
        if b["seen_color"] and b["seen_color"] != b["color"]:
            res["warnings"].append(f"cell {b['cell']}: model said {b['color']}, pixels look {b['seen_color']}")

    # Several blocks of one colour are normal. Check the counts if we know them.
    counts = {}
    for b in final:
        counts[b["color"]] = counts.get(b["color"], 0) + 1
    res["counts"] = counts
    if expect:
        res["expect"] = expect
        for c, n in expect.items():
            if counts.get(c, 0) != n:
                res["warnings"].append(f"expected {n} {c}, found {counts.get(c, 0)}")
        for c, n in counts.items():
            if c not in expect:
                res["warnings"].append(f"found {n} {c}, expected none")
        res["counts_ok"] = all(counts.get(c, 0) == n for c, n in expect.items()) and \
            all(c in expect for c in counts)
    else:
        for c in colors:
            if c not in counts:
                res["warnings"].append(f"no {c} block found")

    res["blocks"] = final
    res["rejected"] = rejected
    res["cells"] = {str(n): None for n in range(1, 10)}
    for b in final:
        res["cells"][str(b["cell"])] = b["color"]
    res["raw"] = raw
    res["ok"] = True
    store("annotated", draw_rectified(rect, view, final))
    res["files"] = files
    res["timing_s"] = {"sheet": round(t_sheet - t0, 3), "detect": round(t_det - t_sheet, 3),
                       "total": round(time.time() - t0, 3)}
    if save:
        with open(os.path.join(SCAN_DIR, f"scan_{stamp}_result.json"), "w") as f:
            json.dump(res, f, indent=1)
    return res


def print_summary(res):
    if not res["ok"]:
        print("FAILED:", res.get("error"))
    else:
        cells = res["cells"]
        print(f"layout {int(res['layout']['cell_mm'])} mm, markers {res['markers_used']}, "
              f"fit error {res['reproj_mm']} mm, {res['timing_s']['total']} s")
        for r in range(3):
            print("  " + " | ".join(f"{n}:{(cells[str(n)] or '.'):<6}" for n in range(r * 3 + 1, r * 3 + 4)))
    for w in res["warnings"]:
        print("  warning:", w)
    if res.get("files"):
        print("  saved:", ", ".join(os.path.join("scans", f) for f in res["files"].values()))


# ---- HTTP --------------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    backend = None
    colors = DEFAULT_COLORS
    cell_mm = None
    expect = None

    def log_message(self, fmt, *args):
        pass

    def _send(self, code, body, ctype="application/json"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            with open(PAGE, "rb") as f:
                return self._send(200, f.read(), "text/html; charset=utf-8")
        if path == "/api/health":
            return self._send(200, {"ok": True, "backend": self.backend.name, "colors": self.colors,
                                    "cell_mm": self.cell_mm, "expect": self.expect})
        if path.startswith("/scans/"):
            name = path[len("/scans/"):]
            fp = os.path.join(SCAN_DIR, name)
            if _SCAN_RE.match(name) and os.path.isfile(fp):
                with open(fp, "rb") as f:
                    ctype = "image/jpeg" if name.endswith(".jpg") else "application/json"
                    return self._send(200, f.read(), ctype)
        self._send(404, {"error": "not found"})

    def do_POST(self):
        url = urlparse(self.path)
        if url.path != "/api/detect":
            return self._send(404, {"error": "not found"})
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0 or n > MAX_BODY:
            return self._send(413 if n > MAX_BODY else 400, {"error": "missing or too large body"})
        body = self.rfile.read(n)
        q = parse_qs(url.query)
        colors = q["colors"][0].split(",") if "colors" in q else None
        cell = int(q["cell"][0]) if "cell" in q else None
        expect = q["expect"][0] if "expect" in q else None
        try:
            if (self.headers.get("Content-Type") or "").startswith("application/json"):
                req = json.loads(body)
                data = req.get("image", "")
                if "," in data[:100]:
                    data = data.split(",", 1)[1]        # strip a data: URL prefix
                body = base64.b64decode(data)
                colors = req.get("colors") or colors
                cell = req.get("cell_mm") or cell
                expect = req.get("expect") or expect
            img = decode_image(body)
            res = run_scan(img, self.backend, colors or self.colors, cell or self.cell_mm,
                           expect=expect or self.expect)
        except Exception as e:  # report, keep serving
            return self._send(400, {"ok": False, "error": f"{type(e).__name__}: {e}"})
        print_summary(res)
        self._send(200, res)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--image", nargs="*", help="run on these photos and exit")
    ap.add_argument("--backend", choices=["locate", "color"], default="locate")
    ap.add_argument("--model", default="nvidia/LocateAnything-3B", help="HF id or local folder")
    ap.add_argument("--revision", default=MODEL_REVISION,
                    help="model commit to use (default: the tested one). 'latest' = newest on the Hub")
    ap.add_argument("--colors", default=",".join(DEFAULT_COLORS))
    ap.add_argument("--expect", default=None, help="expected counts, e.g. green:3,blue:2,red:1")
    ap.add_argument("--phrase", default="{color} cube", help="text query, {color} is filled in")
    ap.add_argument("--mode", choices=["detect", "point"], default="detect")
    ap.add_argument("--greedy", action="store_true", help="no sampling (repeatable answers)")
    ap.add_argument("--temperature", type=float, default=0.7)
    ap.add_argument("--cell", type=int, default=None, help="force a layout (60/55/50), default: from marker IDs")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8002)
    a = ap.parse_args()

    colors = [c.strip() for c in a.colors.split(",") if c.strip()]
    revision = None if a.revision == "latest" else a.revision
    backend = make_backend(a.backend, model=a.model, revision=revision, phrase=a.phrase, mode=a.mode,
                           greedy=a.greedy, temperature=a.temperature)
    expect = parse_expect(a.expect)
    if a.image:
        for p in a.image:
            print(f"== {p}")
            with open(p, "rb") as f:
                res = run_scan(decode_image(f.read()), backend, colors, a.cell, expect=expect)
            print_summary(res)
            if a.backend == "locate":
                for c, r in res.get("raw", {}).items():
                    print(f"  raw[{c}]: {r[:300]!r}")
        return

    Handler.backend, Handler.colors, Handler.cell_mm, Handler.expect = backend, colors, a.cell, expect
    srv = ThreadingHTTPServer((a.host, a.port), Handler)
    print(f"[detector] backend={backend.name} colors={colors} on http://{a.host}:{a.port}")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    sys.exit(main())
