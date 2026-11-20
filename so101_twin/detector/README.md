# Block detector (LocateAnything-3B + printed grid)

Photo in, "which colour block is in which of the 9 cells" out.

```
photo -> 4 ArUco markers -> homography -> top-down grid image
      -> LocateAnything-3B ("red cube", "green cube", ...) -> mm on the sheet -> cell 1..9
```

The table is flat, so the markers alone map any photo pixel to millimetres
on the sheet. No camera calibration and no camera pose are needed. That is
why a moving head-mounted camera (Quest 3) works. At least 1 marker must be
visible, 2 or more is much better.

## Files

| File | What it is |
|---|---|
| `grid_sheet_60mm.pdf` | The sheet to print (recommended). `55mm` and `50mm` versions too. |
| `make_grid_sheet.py` | Regenerates the PDFs. |
| `sheet_layout.py` | Sheet geometry. Shared by the generator and the detector. |
| `vision.py` | Marker detection, homography, top-down image, cell lookup. |
| `backends.py` | LocateAnything backend and a plain colour backend (no GPU). |
| `locate_service.py` | HTTP service (port 8002) and command line tool. |
| `web/detector.html` | Test page: drop a photo, see the 3x3 result. |
| `selftest.py` | Synthetic 3D test photos, checks the whole pipeline. |
| `setup_env.sh` | One-time environment setup. |

## The sheet

1. Print `grid_sheet_60mm.pdf` on plain white A4 at **100 % / Actual size**. Never "Fit to page".
2. Measure the bar at the top. It must be 100 mm (99 to 101 is fine).
3. Matte paper. Gloss or glossy lamination reflects light and hides the markers.
4. Do not fold the sheet, and do not put tape or anything else over the markers or the white border around them.
5. "ROBOT SIDE" faces the robot. Tape all 4 edges flat to the table, and mark where the sheet and the robot base sit. The 9 taught pick motions are only valid while neither moves.

Each sheet size has its own marker IDs (60 mm: 0-3, 55 mm: 4-7, 50 mm: 8-11).
The detector reads the IDs and picks the right geometry by itself.

## Setup (once)

```bash
cd so101_twin/detector
bash setup_env.sh          # separate env "locany", PyTorch cu128, model pins, model download
conda activate locany      # or the line setup_env.sh prints at the end
```

If the download says 401 or 403: open the model page on Hugging Face, accept the
licence, run `hf auth login`, and run `bash setup_env.sh` again.

## Test

```bash
python selftest.py                     # pipeline check, colour backend, no GPU (seconds)
python selftest.py --backend locate    # the same synthetic photos through LocateAnything
python locate_service.py --image my_photo.jpg      # one real photo, prints the 3x3 result
python locate_service.py                            # service + test page: http://127.0.0.1:8002
python locate_service.py --backend color            # fallback without the model
```

Every scan saves the photo, the top-down image, the annotated image and the
JSON result in `scans/`.

Useful options: `--colors green,yellow,blue,red`, `--expect green:3,blue:2,red:1`,
`--phrase "{color} block"`, `--mode point` (point queries instead of boxes),
`--greedy` (no sampling).

## Several blocks of one colour

Normal and supported. Give the expected counts and the answer says whether
they match (`counts`, `counts_ok`, plus a warning such as "expected 3 blue, found 2"):

```bash
python locate_service.py --expect green:3,blue:2,red:1
```

The test page has an "Expected" field for the same thing.

## Pinned model version

The detector loads LocateAnything-3B at commit
`c32291ca5e996f5a7a485845b4f57a233936bba0`, the version tested on 19 Nov 2026.
The pin covers the weights and the model's own Python code, so an update on
Hugging Face cannot change behaviour before the demo. `--revision latest`
uses the newest version instead.

For the demo day, run fully offline so a slow network cannot delay start-up:

```bash
export HF_HUB_OFFLINE=1
python locate_service.py --expect green:3,blue:2,red:1
```

## API

`POST /api/detect` with either
- the raw JPEG/PNG as the body (`Content-Type: image/jpeg`), optional `?colors=red,blue&expect=blue:3,black:2`, or
- JSON `{"image": "<base64 or data URL>", "colors": ["red", "blue"], "expect": {"blue": 3, "black": 2}}`.

Answer (short form):

```json
{"ok": true, "cells": {"1": "red", "2": null, "...": "..."},
 "blocks": [{"color": "red", "cell": 1, "sheet_mm": [45.2, 88.1], "margin_mm": 21.4, "unsure": false}],
 "warnings": [], "markers_used": [0, 1, 2, 3], "reproj_mm": 0.3, "timing_s": {"total": 1.2}}
```

`unsure: true` means the block centre is within 6 mm of a cell border.

## Taking good photos

- Look roughly straight down at the sheet. Block tops sit 25 mm above the paper, so an angled view shifts them.
- Park the arm out of the way before scanning, so it hides neither blocks nor markers.
- Put each block on its dashed outline.
