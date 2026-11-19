"""Block finders. Both work on the rectified top-down grid image.

LocateAnythingBackend  NVIDIA LocateAnything-3B, one query per colour.
ColorBackend           plain HSV colour thresholds. Needs no GPU. Used as a
                       fallback and for testing the rest of the pipeline.

Every backend returns a list of dicts:
    {"color": str, "box": [x1, y1, x2, y2] or None, "point": [u, v], "raw": str}
in pixels of the image it was given.
"""
import re
import threading
import time

import cv2
import numpy as np

from vision import PX_PER_MM

# Hue ranges on OpenCV's 0-179 scale.
HUE_RANGES = {
    "red": [(0, 8), (170, 179)],
    "orange": [(9, 19)],
    "yellow": [(20, 34)],
    "green": [(35, 85)],
    "blue": [(90, 130)],
    "purple": [(131, 160)],
    "pink": [(161, 169)],
}
SAT_MIN, VAL_MIN = 90, 60
DARK_MAX = 85          # "black": brightness below this, any saturation
BLOCK_MM = 25.0


def measured_color(img_bgr, box=None, point=None, names=None):
    """Name the dominant colour inside a box (or around a point).

    Saturated pixels give a hue name. If few pixels are saturated but most
    are dark, the answer is "black". Otherwise None (unsure).
    """
    h, w = img_bgr.shape[:2]
    if box is not None:
        x1, y1, x2, y2 = [int(round(v)) for v in box]
        # central 60 % of the box, mostly the cube's top face
        cx, cy, hw, hh = (x1 + x2) / 2, (y1 + y2) / 2, (x2 - x1) * 0.3, (y2 - y1) * 0.3
        x1, x2, y1, y2 = int(cx - hw), int(cx + hw), int(cy - hh), int(cy + hh)
    else:
        r = int(BLOCK_MM * PX_PER_MM * 0.25)
        x1, y1, x2, y2 = int(point[0]) - r, int(point[1]) - r, int(point[0]) + r, int(point[1]) + r
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, max(x1 + 1, x2)), min(h, max(y1 + 1, y2))
    hsv = cv2.cvtColor(img_bgr[y1:y2, x1:x2], cv2.COLOR_BGR2HSV).reshape(-1, 3)
    sat = hsv[(hsv[:, 1] >= SAT_MIN) & (hsv[:, 2] >= VAL_MIN)]
    if len(sat) < 0.15 * len(hsv):
        dark = int((hsv[:, 2] < DARK_MAX).sum())
        if dark >= 0.4 * len(hsv) and (not names or "black" in names):
            return "black"
        return None
    best, count = None, 0
    for name, ranges in HUE_RANGES.items():
        if names and name not in names:
            continue
        k = sum(int(((sat[:, 0] >= lo) & (sat[:, 0] <= hi)).sum()) for lo, hi in ranges)
        if k > count:
            best, count = name, k
    return best


class ColorBackend:
    name = "color"

    def __init__(self, **_):
        self.ready = True

    def detect(self, img_bgr, colors, grid_rect_px=None):
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        side = BLOCK_MM * PX_PER_MM
        amin, amax = 0.25 * side * side, 3.0 * side * side
        out = []
        for color in colors:
            if color not in HUE_RANGES and color != "black":
                continue
            mask = np.zeros(hsv.shape[:2], np.uint8)
            if color == "black":
                # dark pixels; the opening below removes thin grid lines and digits
                mask |= cv2.inRange(hsv, (0, 0, 0), (179, 255, DARK_MAX - 1))
            else:
                for lo, hi in HUE_RANGES[color]:
                    mask |= cv2.inRange(hsv, (lo, SAT_MIN, VAL_MIN), (hi, 255, 255))
            k = 9 if color == "black" else 5
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((k, k), np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
            n, _, stats, cent = cv2.connectedComponentsWithStats(mask)
            for i in range(1, n):
                x, y, w, h, a = stats[i]
                if not (amin <= a <= amax):
                    continue
                if max(w, h) > 2.6 * min(w, h):      # long thin strip, not a cube
                    continue
                out.append({"color": color, "box": [float(x), float(y), float(x + w), float(y + h)],
                            "point": [float(cent[i][0]), float(cent[i][1])], "raw": f"area={int(a)}"})
        return out


# ---- LocateAnything ---------------------------------------------------------
_BOX_RE = re.compile(r"<box>(.*?)</box>", re.S)
_INT_RE = re.compile(r"\d+(?:\.\d+)?")


def parse_locate_output(text, width, height):
    """Turn model text into boxes/points in pixels.

    The model card shows <box><x1><y1><x2><y2></box> for boxes and
    <box><x><y></box> for points, values 0..1000. This parser also accepts
    spaces or commas between numbers. Slots holding <null> are ignored.
    """
    sx, sy = width / 1000.0, height / 1000.0
    boxes, points = [], []
    for body in _BOX_RE.findall(text or ""):
        nums = [float(v) for v in _INT_RE.findall(body.replace("null", ""))]
        if len(nums) >= 4:
            x1, y1, x2, y2 = nums[:4]
            boxes.append([min(x1, x2) * sx, min(y1, y2) * sy, max(x1, x2) * sx, max(y1, y2) * sy])
        elif len(nums) == 2:
            points.append([nums[0] * sx, nums[1] * sy])
    return boxes, points


class LocateAnythingBackend:
    name = "locate"
    DETECT = "Locate all the instances that matches the following description: {phrase}."
    POINT = "Point to: {phrase}."

    def __init__(self, model="nvidia/LocateAnything-3B", revision=None, phrase="{color} cube", mode="detect",
                 greedy=False, temperature=0.7, max_new_tokens=1024, generation_mode="hybrid", **_):
        import torch
        from transformers import AutoModel, AutoProcessor, AutoTokenizer
        self.torch = torch
        self.phrase, self.mode = phrase, mode
        self.greedy, self.temperature = greedy, temperature
        self.max_new_tokens, self.generation_mode = max_new_tokens, generation_mode
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.dtype = torch.bfloat16 if self.device == "cuda" else torch.float32
        if self.device == "cpu":
            print("[locate] WARNING: no CUDA GPU found, running on CPU (very slow)")
        t = time.time()
        # revision pins both the weights and the model's own Python code files
        pin = {"revision": revision} if revision else {}
        self.tokenizer = AutoTokenizer.from_pretrained(model, trust_remote_code=True, **pin)
        self.processor = AutoProcessor.from_pretrained(model, trust_remote_code=True, **pin)
        self.model = AutoModel.from_pretrained(model, dtype=self.dtype, trust_remote_code=True,
                                               **pin).to(self.device).eval()
        self.lock = threading.Lock()
        self.ready = True
        print(f"[locate] model loaded on {self.device} in {time.time() - t:.1f} s "
              f"(revision {revision or 'latest'})")
        if self.device == "cuda":
            print(f"[locate] GPU memory in use: {torch.cuda.memory_allocated() / 2**30:.1f} GB")

    def ask(self, pil_image, question):
        torch = self.torch
        messages = [{"role": "user", "content": [
            {"type": "image", "image": pil_image},
            {"type": "text", "text": question},
        ]}]
        text = self.processor.py_apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
        images, videos = self.processor.process_vision_info(messages)
        inputs = self.processor(text=[text], images=images, videos=videos, return_tensors="pt").to(self.device)
        kw = dict(
            pixel_values=inputs["pixel_values"].to(self.dtype),
            input_ids=inputs["input_ids"],
            attention_mask=inputs["attention_mask"],
            image_grid_hws=inputs.get("image_grid_hws", None),
            tokenizer=self.tokenizer,
            max_new_tokens=self.max_new_tokens,
            use_cache=True,
            generation_mode=self.generation_mode,
            repetition_penalty=1.1,
        )
        if self.greedy:
            kw.update(do_sample=False)
        else:
            kw.update(do_sample=True, temperature=self.temperature, top_p=0.9)
        with self.lock, torch.no_grad():
            resp = self.model.generate(**kw)
        return resp[0] if isinstance(resp, tuple) else resp

    def detect(self, img_bgr, colors, grid_rect_px=None):
        from PIL import Image
        h, w = img_bgr.shape[:2]
        pil = Image.fromarray(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
        out = []
        for color in colors:
            phrase = self.phrase.format(color=color)
            tmpl = self.POINT if self.mode == "point" else self.DETECT
            raw = self.ask(pil, tmpl.format(phrase=phrase))
            raw = raw if isinstance(raw, str) else str(raw)
            boxes, points = parse_locate_output(raw, w, h)
            for b in boxes:
                out.append({"color": color, "box": b, "point": [(b[0] + b[2]) / 2, (b[1] + b[3]) / 2], "raw": raw})
            for p in points:
                out.append({"color": color, "box": None, "point": p, "raw": raw})
            if not boxes and not points:
                out.append({"color": color, "box": None, "point": None, "raw": raw})
        return out


def make_backend(name, **kw):
    if name == "color":
        return ColorBackend(**kw)
    if name == "locate":
        return LocateAnythingBackend(**kw)
    raise ValueError(f"unknown backend {name}")
