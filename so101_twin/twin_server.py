"""
Browser-based digital twin teleop for the SO-101.

    python twin_server.py --port /dev/ttyACM0 --id asherbot
    python twin_server.py --sim                      # no robot, fake arm (try the UI safely)

then open http://127.0.0.1:8000 in a browser.

Architecture
------------
    browser (Three.js twin)  --HTTP-->  this server  --serial-->  SO-101
        drag a link  -> POST /api/target   (sets TARGET only)
        poll 20 Hz   -> GET  /api/state    (actual / cmd / target)

One background thread owns the serial bus and runs the same loop as
knob_teleop.py: every tick, `cmd` chases `target` at no more than
"max speed" deg/s, then cmd is sent and the arm is read back.
HTTP threads never touch the bus; they only change `target` under a lock.
So however fast you fling the mouse, the motors move at <= max speed.

Recordings go to so101_twin/recorded_motions/ and the Replay button plays them
back (one or several in a row) through the same speed cap.
Recording writes the same JSON-lines format as teleop_and_record.py /
knob_teleop.py, so replay_motion.py and block_trigger_watcher.py still work.

Sorting demo (see sorting.py and web/teach.html):
    /teach.html   teach a hover and a grab pose for every cell by moving the arm by hand
                  (arm torque off, gripper held at its teach value), plus the Home pose
    sequencer     pick: hover -> grab -> close gripper -> hover
                  place: hover -> grab -> open gripper -> hover; at the end: Home
    Poses are saved in taught_poses.json. Every sequencer move runs through the
    same speed cap, as one straight line in joint space (all joints arrive together).
    Scan          POST a photo to /api/scan: it goes to the detector service
                  (detector/locate_service.py, --detector URL), and a valid 3 green + 2 + 1
                  result becomes the board, with a plan ready for Run.

Only the standard library is needed (plus lerobot when not in --sim).
"""

import argparse
import json
import mimetypes
import os
import re
import threading
import time
import urllib.error
import urllib.request

import sorting
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

LOOP_HZ = 30.0
SPEED_CEILING = 90.0   # hard upper limit for max speed (deg/s; gripper %/s)
DEFAULT_SPEED = 30.0
ENCODER_MAX = 4095     # STS3215 resolution - 1

HERE = os.path.dirname(os.path.abspath(__file__))
WEB_ROOT = os.path.join(HERE, "web")
URDF_ROOT = os.path.join(HERE, "urdf")
TWIN_CALIB = os.path.join(HERE, "twin_calib.json")   # visual-only: how LeRobot degrees map onto the URDF
RECORD_DIR = os.path.join(HERE, "recorded_motions")  # every recording goes here; replay picks from here

REPLAY_LAG_TOL = 2.0   # deg: replay clock pauses while cmd is further than this behind the recording
SETTLE_TOL = 0.05      # deg: "cmd has reached the target"
_NAME_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._ -]{0,99}\.jsonl")

# ---- sorting demo ----------------------------------------------------------
POSES_FILE = os.path.join(HERE, "taught_poses.json")
ARM = JOINTS[:5]                       # everything except the gripper
# Home pose from the demo screenshot of 19 Nov 2026. Re-teach it on the teach page if needed.
DEFAULT_HOME = {"shoulder_pan": 1.2, "shoulder_lift": -1.3, "elbow_flex": 0.3,
                "wrist_flex": -62.6, "wrist_roll": 18.3}
# Gripper % values: teach = held while teaching, open = release / approach,
# grip = close on a block, home = at the Home pose.
DEFAULT_GRIPPER = {"teach": 19.5, "open": 19.5, "grip": 15.0, "home": 1.0}
SLOT_RE = re.compile(r"^(home|[1-9]\.(hover|grab))$")
ARRIVE_TOL = 3.0       # deg: a pose counts as reached when every arm joint is this close
# The sequencer never stops for a missed pose (servos can sit a few degrees off, and this setup
# has no collisions). After a motion it waits for the arm to catch up, but only until the arm
# stops moving or ARRIVE_WAIT has passed, then it carries on.
ARRIVE_WAIT = 1.0      # s: longest wait for the arm to catch up after a motion
STILL_DEG = 0.15       # deg per loop tick: below this the arm counts as not moving
STILL_S = 0.25         # s: not moving for this long = settled, carry on
SORT_ALGORITHM = "greedy_simple"   # the one planner the demo uses
DETECTOR_URL = "http://127.0.0.1:8002"   # detector/locate_service.py
SCAN_COLORS = "green,yellow,blue,red"    # demo block set
MAX_SCAN_BYTES = 20 * 2**20
GRIP_DWELL = 0.6       # s: wait after a gripper command (the block stops the jaw early)
TORQUE_CHECK_S = 1.0   # s: while the arm is free, re-read Torque_Enable this often and fix it if it came back
COLOR_RE = re.compile(r"^[a-z]{1,12}$")


def load_poses():
    """taught_poses.json -> {"gripper", "home", "home_taught", "cells", "taught"} with defaults."""
    try:
        with open(POSES_FILE) as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError):
        data = {}
    out = {"gripper": dict(DEFAULT_GRIPPER), "home": dict(DEFAULT_HOME), "home_taught": False,
           "cells": {}, "taught": dict(data.get("taught") or {})}
    for k, v in (data.get("gripper") or {}).items():
        if k in DEFAULT_GRIPPER:
            out["gripper"][k] = min(max(float(v), 0.0), 100.0)
    home = data.get("home")
    if isinstance(home, dict) and all(j in home for j in ARM):
        out["home"] = {j: float(home[j]) for j in ARM}
        out["home_taught"] = bool(data.get("home_taught", True))
    for n in range(1, 10):
        c = (data.get("cells") or {}).get(str(n)) or {}
        for kind in ("hover", "grab"):
            pose = c.get(kind)
            if isinstance(pose, dict) and all(j in pose for j in ARM):
                out["cells"].setdefault(str(n), {})[kind] = {j: float(pose[j]) for j in ARM}
    return out


def save_poses(poses):
    tmp = POSES_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(poses, f, indent=1)
    os.replace(tmp, POSES_FILE)      # atomic: a crash never leaves half a file


def motion_path(name, default_if_empty=False):
    """Map a user-supplied name to a file inside RECORD_DIR. Never allows paths outside it."""
    name = (name or "").strip()
    if "/" in name or "\\" in name:
        raise ValueError("Give a file name only, not a path: files always live in recorded_motions/.")
    if not name and default_if_empty:
        name = time.strftime("motion_%Y%m%d_%H%M%S")
    if name and not name.endswith(".jsonl"):
        name += ".jsonl"
    if not _NAME_RE.fullmatch(name):
        raise ValueError("Use letters, digits, space, '.', '_' or '-' for the file name.")
    return name, os.path.join(RECORD_DIR, name)


def load_motion(name, limits):
    """Read a recording into [(t, {joint: deg}), ...] with t starting at 0, clamped to the joint limits."""
    _, path = motion_path(name)
    frames = []
    with open(path) as f:
        for line in f:
            if not line.strip():
                continue
            try:
                e = json.loads(line)
                act = {j: float(e["action"][f"{j}.pos"]) for j in JOINTS if f"{j}.pos" in e["action"]}
                frames.append((float(e["t"]), act))
            except (ValueError, KeyError, TypeError):
                continue                      # skip a malformed line rather than refusing the file
    if not frames:
        raise ValueError(f"{name} has no usable frames")
    frames.sort(key=lambda fr: fr[0])
    t0 = frames[0][0]
    out = []
    for t, act in frames:
        out.append((t - t0, {j: min(max(v, limits[j][0]), limits[j][1]) for j, v in act.items()}))
    return out


def list_motions():
    items = []
    if not os.path.isdir(RECORD_DIR):
        return items
    for fn in os.listdir(RECORD_DIR):
        if not _NAME_RE.fullmatch(fn):
            continue
        p = os.path.join(RECORD_DIR, fn)
        frames, first, last = 0, None, None
        try:
            with open(p) as f:
                for line in f:
                    if not line.strip():
                        continue
                    try:
                        t = float(json.loads(line)["t"])
                    except (ValueError, KeyError, TypeError):
                        continue
                    frames += 1
                    first = t if first is None else first
                    last = t
        except OSError:
            continue
        items.append({"name": fn, "frames": frames,
                      "duration": (last - first) if frames else 0.0,
                      "mtime": os.path.getmtime(p), "size": os.path.getsize(p)})
    items.sort(key=lambda it: it["mtime"], reverse=True)
    return items


def load_twin_calib():
    """{joint: {"sign": +-1, "offset": deg}}. Affects only the drawing, never the motors."""
    try:
        with open(TWIN_CALIB) as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError):
        return {}


def save_twin_calib(body):
    clean = {}
    for j in JOINTS:
        c = body.get(j) or {}
        clean[j] = {"sign": -1 if float(c.get("sign", 1)) < 0 else 1,
                    "offset": float(c.get("offset", 0.0))}
    with open(TWIN_CALIB, "w") as f:
        json.dump(clean, f, indent=2)


# --------------------------------------------------------------------------
# Robot backends
# --------------------------------------------------------------------------
def joint_limits(robot, joint):
    """Same as knob_teleop.py: +/- half the calibrated span (degrees mode)."""
    if joint == "gripper":
        return 0.0, 100.0
    cal = (getattr(robot, "calibration", None) or {}).get(joint)
    if cal is None:
        return -100.0, 100.0
    half = (cal.range_max - cal.range_min) / 2.0 * 360.0 / ENCODER_MAX
    return -half, half


class _SimBus:
    """Torque on/off, like lerobot's FeetechMotorsBus.enable_torque / disable_torque."""

    def __init__(self, sim):
        self.sim = sim

    def disable_torque(self, motors=None, num_retry=0):
        self.sim.free |= set([motors] if isinstance(motors, str) else (motors or JOINTS))

    def enable_torque(self, motors=None, num_retry=0):
        self.sim.free -= set([motors] if isinstance(motors, str) else (motors or JOINTS))

    def read(self, data_name, motor, **kw):
        assert data_name == "Torque_Enable"
        return 0 if motor in self.sim.free else 1


class SimRobot:
    """Stand-in for SO101Follower: follows commands with a little lag."""

    calibration = None

    def __init__(self):
        self.pos = {j: 0.0 for j in JOINTS}
        self.pos.update({"shoulder_pan": 5.1, "shoulder_lift": -2.2, "elbow_flex": 0.6, "wrist_flex": 90.5, "wrist_roll": 14.2, "gripper": 2.0})
        self.cmd = dict(self.pos)
        self.free = set()                      # joints with torque off
        self.bus = _SimBus(self)

    def connect(self):
        pass

    def hand(self, values):
        """--sim only: pretend a hand moved the free joints (for testing teach mode)."""
        for j, v in values.items():
            if j in self.free:
                self.pos[j] = float(v)

    def get_observation(self):
        for j in JOINTS:                       # first-order lag, like a real servo
            if j not in self.free:             # torque off: the joint stays where the hand left it
                self.pos[j] += 0.35 * (self.cmd[j] - self.pos[j])
        return {f"{j}.pos": v for j, v in self.pos.items()}

    def send_action(self, action):
        for j in JOINTS:                       # like lerobot: only the motors in the action get a goal
            if f"{j}.pos" in action:
                self.cmd[j] = float(action[f"{j}.pos"])

    def disconnect(self):
        pass


# --------------------------------------------------------------------------
# Controller: the only thing that talks to the bus
# --------------------------------------------------------------------------
class Controller:
    def __init__(self, robot, max_speed):
        self.robot = robot
        self.lock = threading.Lock()
        self.period = 1.0 / LOOP_HZ
        self.speed = min(max(max_speed, 1.0), SPEED_CEILING)
        self.error = ""
        self.rec_file = None
        self.rec_path = ""
        self.rec_t0 = 0.0
        self.replay = None     # dict while a replay is running, see start_replay()
        self.replay_msg = ""   # last replay outcome, shown in the UI
        self.running = True

        # sorting demo
        self.poses = load_poses()
        self.poses_version = 0
        self.teach = None      # {"free": bool} while teach mode is on
        self.teach_msg = ""
        self.requests = []     # torque on/off jobs for the loop thread (it owns the bus)
        self.program = None    # dict while the sequencer runs, see _start_program()
        self.program_msg = ""
        self.board = {str(n): None for n in range(1, 10)}   # what is in each cell (twin + planner)
        self.holding = None    # colour of the block in the gripper, if any
        self.sort = None       # last plan from sorting.plan(), plus progress
        self.scan = None       # last scan result (see scan_image)
        self.detector_url = DETECTOR_URL
        self.keep_scans = False   # False: a scan's photo is deleted when its sort finishes or a new scan replaces it
        self._torque_checked = 0.0

        # Read where the arm actually is BEFORE sending anything, so nothing jumps.
        obs = robot.get_observation()
        self.start_pose = {j: float(obs[f"{j}.pos"]) for j in JOINTS}
        self.actual = dict(self.start_pose)
        self.actual_t = time.monotonic()   # when `actual` was read from the motors (s); lets clients compute speeds
        self.cmd = dict(self.start_pose)
        self.target = dict(self.start_pose)
        self.limits = {}
        for j in JOINTS:
            lo, hi = joint_limits(robot, j)
            v = self.start_pose[j]
            self.limits[j] = (min(lo, v), max(hi, v))

        self.thread = threading.Thread(target=self._loop, daemon=True)

    # ---- commands from HTTP threads ---------------------------------------
    def busy_reason(self):
        """Why manual targets are refused right now, or '' if they are allowed. Lock held by caller."""
        if self.replay is not None:
            return "Replay in progress: press STOP to take control"
        if self.program is not None:
            return f"{self.program['name']} in progress: press STOP to take control"
        if self.teach and self.teach["free"]:
            return "Teach mode: the arm is free (torque off). Press Lock first"
        return ""

    def set_targets(self, values):
        """Returns '' if applied, else the reason it was refused (nothing changes then)."""
        with self.lock:
            why = self.busy_reason()
            if why:
                return why
            for j, v in values.items():
                if j in self.limits:
                    if j == "gripper" and self.teach:      # the gripper is held while teaching
                        continue
                    lo, hi = self.limits[j]
                    self.target[j] = min(max(float(v), lo), hi)
            return ""

    def set_speed(self, v):
        with self.lock:
            self.speed = min(max(float(v), 1.0), SPEED_CEILING)

    def stop(self):
        with self.lock:
            if self.replay is not None:
                self.replay_msg = f"Replay of {self.replay['names'][self.replay['k']]} stopped"
            self.replay = None
            if self.program is not None:
                p = self.program
                label = p["steps"][min(p["i"], len(p["steps"]) - 1)]["label"]
                self.program_msg = f"{p['name']} stopped at: {label}"
                self.program = None
            self.target = dict(self.cmd)
            arm_free = bool(self.teach and self.teach["free"])
        if arm_free:                               # STOP while teaching = torque back on, nothing saved
            self._request("lock", wait=False)

    def go_start(self):
        with self.lock:
            if self.busy_reason():
                return False
            self.target = dict(self.start_pose)
            return True

    # ---- teach mode (sorting demo) -------------------------------------------
    def _request(self, kind, wait=True, **kw):
        """Queue a torque job for the loop thread; optionally wait for its result."""
        req = {"kind": kind, "event": threading.Event(), "result": (500, "no answer from the control loop"), **kw}
        with self.lock:
            self.requests.append(req)
        if wait:
            req["event"].wait(timeout=3.0)
        return req["result"]

    def teach_start(self):
        with self.lock:
            if self.replay is not None or self.program is not None or self.rec_file is not None:
                return 409, "Stop the replay, program or recording first."
            if self.teach is None:
                self.teach = {"free": False}
            self.target["gripper"] = self.poses["gripper"]["teach"]
            self.teach_msg = "Teach mode on. Gripper held at %.1f %%." % self.poses["gripper"]["teach"]
            return 200, self.teach_msg

    def teach_stop(self):
        with self.lock:
            if self.teach is None:
                return 200, "Teach mode is already off."
            free = self.teach["free"]
        if free:
            code, msg = self._request("lock")
            if code != 200:
                return code, msg
        with self.lock:
            self.teach = None
            self.teach_msg = "Teach mode off."
            return 200, self.teach_msg

    def teach_free(self):
        with self.lock:
            if self.teach is None:
                return 409, "Turn teach mode on first."
            if self.program is not None:
                return 409, "Wait for the motion to finish (or press STOP)."
            if self.teach["free"]:
                return 200, "The arm is already free."
        return self._request("free")

    def teach_lock(self, slot):
        if slot and not SLOT_RE.fullmatch(slot):
            return 400, f"Unknown pose slot {slot!r}"
        with self.lock:
            if self.teach is None:
                return 409, "Turn teach mode on first."
            if self.program is not None:
                return 409, "Wait for the motion to finish (or press STOP)."
        return self._request("lock", slot=slot or None)

    def _save_slot(self, slot, pose):
        """Lock held by caller."""
        pose = {j: round(pose[j], 2) for j in ARM}
        if slot == "home":
            self.poses["home"] = pose
            self.poses["home_taught"] = True
        else:
            n, kind = slot.split(".")
            self.poses["cells"].setdefault(n, {})[kind] = pose
        self.poses["taught"][slot] = time.strftime("%Y-%m-%d %H:%M:%S")
        save_poses(self.poses)
        self.poses_version += 1

    def set_gripper_values(self, body):
        with self.lock:
            for k in DEFAULT_GRIPPER:
                if k in body:
                    self.poses["gripper"][k] = min(max(float(body[k]), 0.0), 100.0)
            save_poses(self.poses)
            self.poses_version += 1
            if self.teach and self.program is None:
                self.target["gripper"] = self.poses["gripper"]["teach"]
            return dict(self.poses["gripper"])

    def slot_pose(self, slot):
        """Lock held by caller."""
        if slot == "home":
            return self.poses["home"]
        n, kind = slot.split(".")
        return (self.poses["cells"].get(n) or {}).get(kind)

    def _torque(self, on):
        bus = getattr(self.robot, "bus", None)
        if bus is None:
            raise RuntimeError("this robot has no bus: cannot switch torque")
        (bus.enable_torque if on else bus.disable_torque)(ARM)

    def _torque_on_motors(self):
        """Arm motors whose Torque_Enable reads 1 (loop thread only). None if unreadable."""
        bus = getattr(self.robot, "bus", None)
        if bus is None or not hasattr(bus, "read"):
            return None
        return [j for j in ARM if int(bus.read("Torque_Enable", j)) != 0]

    def _check_free(self):
        """Loop thread, arm free: if any arm motor has torque again, switch it off again."""
        try:
            on = self._torque_on_motors()
        except Exception as e:
            with self.lock:
                self.teach_msg = f"Could not read torque state: {e}"
            return
        if on:
            self.robot.bus.disable_torque(on)
            with self.lock:
                self.teach_msg = "Torque came back on " + ", ".join(on) + ": switched off again."

    def _run_request(self, req, actual):
        """Loop thread only (it owns the serial bus)."""
        try:
            if req["kind"] == "free":
                self._torque(False)
                on = self._torque_on_motors()
                if on:                              # one retry, then tell the user exactly which motor
                    self.robot.bus.disable_torque(on)
                    on = self._torque_on_motors()
                with self.lock:
                    if self.teach is not None:
                        self.teach["free"] = True
                    self._torque_checked = time.perf_counter()
                    if on:
                        self.teach_msg = "Torque is still on for: " + ", ".join(on) + ". Tell Claude."
                    else:
                        checked = " (torque off confirmed on all 5 motors)" if on is not None else ""
                        self.teach_msg = f"Arm is free{checked}: hold it and move it by hand, then press Lock."
                    req["result"] = (200, self.teach_msg)
            elif req["kind"] == "lock":
                with self.lock:             # goal = where the arm is now, so torque-on causes no jump
                    for j in ARM:
                        self.cmd[j] = self.target[j] = actual[j]
                    action = {f"{j}.pos": self.cmd[j] for j in JOINTS}
                self.robot.send_action(action)
                self._torque(True)
                with self.lock:
                    if self.teach is not None:
                        self.teach["free"] = False
                    slot = req.get("slot")
                    if slot:
                        self._save_slot(slot, actual)
                        self.teach_msg = f"Saved {slot}: " + ", ".join(f"{actual[j]:.1f}" for j in ARM)
                    else:
                        self.teach_msg = "Arm locked (torque on). Nothing saved."
                    req["result"] = (200, self.teach_msg)
        except Exception as e:
            req["result"] = (500, f"{req['kind']} failed: {e}")
            with self.lock:
                self.error = str(e)
        finally:
            req["event"].set()

    # ---- sequencer (sorting demo) -----------------------------------------------
    def _arm(self, pose, label):
        return {"do": "arm", "pose": dict(pose), "label": label}

    def _grip(self, value, label):
        return {"do": "grip", "value": float(value), "label": label}

    def _event(self, event, label, **kw):
        return {"do": "event", "event": event, "label": label, **kw}

    def _need(self, slots):
        """Lock held by caller. Returns the list of missing pose slots."""
        return [s for s in slots if not self.slot_pose(s)]

    def _move_steps(self, n, src, dst):
        """One block from src to dst. Lock held by caller."""
        c, g = self.poses["cells"], self.poses["gripper"]
        hs, gs = c[str(src)]["hover"], c[str(src)]["grab"]
        hd, gd = c[str(dst)]["hover"], c[str(dst)]["grab"]
        tag = f"Move {n}: " if n else ""
        return [
            self._event("move", f"{tag}cell {src} to cell {dst}", n=n, src=src, dst=dst),
            self._arm(hs, f"{tag}hover over cell {src}"),
            self._arm(gs, f"{tag}lower to the block in cell {src}"),
            self._grip(g["grip"], f"{tag}close gripper ({g['grip']:.1f} %)"),
            self._event("pick", f"{tag}picked", cell=src),
            self._arm(hs, f"{tag}lift back over cell {src}"),
            self._arm(hd, f"{tag}hover over cell {dst}"),
            self._arm(gd, f"{tag}lower into cell {dst}"),
            self._grip(g["open"], f"{tag}open gripper ({g['open']:.1f} %)"),
            self._event("place", f"{tag}placed", cell=dst),
            self._arm(hd, f"{tag}lift back over cell {dst}"),
        ]

    def _start_program(self, name, steps, allow_teach=False):
        """Lock held by caller. Returns (code, msg)."""
        if self.replay is not None or self.program is not None:
            return 409, "Something is already moving: wait or press STOP."
        if self.rec_file is not None:
            return 409, "Stop recording first."
        if self.teach is not None and (self.teach["free"] or not allow_teach):
            return 409, ("Lock the arm first." if self.teach["free"] else "Leave teach mode first.")
        self.program = {"name": name, "steps": steps, "i": 0, "move": None}
        self.program_msg = ""
        return 200, f"{name}: {len(steps)} steps"

    def run_moves(self, moves, name):
        """moves: [(src, dst), ...]. Opens the gripper, does the moves, ends at Home."""
        with self.lock:
            slots = ["home"] + [f"{c}.{k}" for m in moves for c in m for k in ("hover", "grab")]
            missing = sorted(set(self._need(slots)))
            if missing:
                return 409, "Teach these poses first: " + ", ".join(missing)
            g = self.poses["gripper"]
            steps = [self._grip(g["open"], f"open gripper ({g['open']:.1f} %)")]
            for n, (src, dst) in enumerate(moves, 1):
                steps += self._move_steps(n if len(moves) > 1 else 0, src, dst)
            steps += [self._arm(self.poses["home"], "return to Home"),
                      self._grip(g["home"], f"gripper to {g['home']:.1f} % at Home")]
            return self._start_program(name, steps)

    def go_home(self):
        with self.lock:
            return self._start_program("Go Home", [self._arm(self.poses["home"], "move to Home")],
                                       allow_teach=True)

    def goto_slot(self, slot):
        if not SLOT_RE.fullmatch(slot or ""):
            return 400, f"Unknown pose slot {slot!r}"
        with self.lock:
            pose = self.slot_pose(slot)
            if not pose:
                return 409, f"{slot} is not taught yet."
            return self._start_program(f"Go to {slot}", [self._arm(pose, f"move to {slot}")], allow_teach=True)

    def test_cell(self, n):
        """hover -> grab -> hover, gripper untouched. For checking taught poses."""
        with self.lock:
            missing = self._need([f"{n}.hover", f"{n}.grab"])
            if missing:
                return 409, "Teach these poses first: " + ", ".join(missing)
            c = self.poses["cells"][str(n)]
            steps = [self._arm(c["hover"], f"hover over cell {n}"),
                     self._arm(c["grab"], f"lower to grab height in cell {n}"),
                     self._arm(c["hover"], f"back over cell {n}")]
            return self._start_program(f"Test cell {n}", steps, allow_teach=True)

    def scan_image(self, data, ctype, source):
        """Photo -> detector -> board. Valid only with 3 green + 2 + 1 blocks. Returns (code, body)."""
        with self.lock:
            if self.program is not None:
                return 409, {"msg": "Wait for the robot to finish (or press STOP) before scanning."}
            prev = self.scan                       # its photo is deleted once this scan is in
            self.scan = {"busy": True, "ok": False, "source": source, "msg": "Scanning..."}
        t0 = time.time()
        url = f"{self.detector_url}/api/detect?colors={SCAN_COLORS}"
        req = urllib.request.Request(url, data=data, method="POST",
                                     headers={"Content-Type": ctype or "image/jpeg"})
        try:
            with urllib.request.urlopen(req, timeout=60) as r:
                det = json.loads(r.read())
        except urllib.error.HTTPError as e:
            try:
                det = json.loads(e.read())
            except Exception:
                det = {"ok": False, "error": f"detector answered HTTP {e.code}"}
        except (urllib.error.URLError, OSError) as e:
            msg = (f"Detector not reachable at {self.detector_url}. "
                   f"Start detector/locate_service.py first. ({getattr(e, 'reason', e)})")
            with self.lock:
                self.scan = {"busy": False, "ok": False, "source": source, "msg": msg,
                             "time": time.strftime("%H:%M:%S"), "files": (prev or {}).get("files") or {}}
            return 503, {"msg": msg, "scan": self.scan}
        scan = {"busy": False, "ok": False, "source": source, "time": time.strftime("%H:%M:%S"),
                "cells": det.get("cells") or {}, "counts": det.get("counts") or {},
                "warnings": det.get("warnings") or [], "markers": det.get("markers_used") or [],
                "image_size": det.get("image_size"), "seconds": round(time.time() - t0, 1),
                "files": det.get("files") or {}}
        found = ", ".join(f"{n} {c}" for c, n in sorted(scan["counts"].items(), key=lambda kv: -kv[1]))
        if not det.get("ok"):
            err = det.get("error") or "the detector could not read the photo"
            scan["msg"] = f"Scan failed: {err}." + (" Keep all 4 corner markers in view." if "marker" in err else "")
        else:
            try:
                sorting.infer_roles(sorting.normalise_board(scan["cells"]))
                scan["ok"] = True
            except sorting.PlanError as e:
                scan["msg"] = f"{e} Rescan."
        if scan["ok"]:
            try:
                self.set_board(scan["cells"])
                plan = self.make_plan()
                scan["msg"] = (f"Found {found}. " + (plan["summary"]["short"] + " Press Run."
                                                     if plan["moves"] else "Already sorted."))
            except (ValueError, sorting.PlanError) as e:
                scan["ok"], scan["msg"] = False, str(e)
        with self.lock:
            self.scan = scan
        if prev and prev.get("files"):            # the previous photo is no longer needed
            self._forget_scan(prev)
        return (200 if scan["ok"] else 422), {"msg": scan["msg"], "scan": scan}

    def _forget_scan(self, scan):
        """Ask the detector to delete this scan's photo and result files (in the background)."""
        files = list((scan.get("files") or {}).values())
        scan["files"] = {}
        if self.keep_scans or not files:
            return
        def job():
            req = urllib.request.Request(f"{self.detector_url}/api/forget", method="POST",
                                         data=json.dumps({"files": files}).encode(),
                                         headers={"Content-Type": "application/json"})
            try:
                with urllib.request.urlopen(req, timeout=10) as r:
                    gone = json.loads(r.read()).get("deleted", [])
                print(f"[scan] deleted {len(gone)} file(s) of the used photo")
            except Exception as e:                  # detector stopped: nothing to delete there now
                print(f"[scan] could not delete the photo files: {e}")
        threading.Thread(target=job, daemon=True).start()

    def set_board(self, cells):
        clean = {str(n): None for n in range(1, 10)}
        for k, v in (cells or {}).items():
            if str(k) not in clean:
                raise ValueError(f"cell {k} does not exist")
            if v:
                v = str(v).strip().lower()
                if not COLOR_RE.fullmatch(v):
                    raise ValueError(f"bad colour name {v!r}")
                clean[str(k)] = v
        with self.lock:
            if self.program is not None:
                raise ValueError("The board cannot change while the robot is moving.")
            self.board = clean
            self.holding = None
            self.sort = None
        return clean

    def make_plan(self, algorithm=SORT_ALGORITHM):
        with self.lock:
            board = dict(self.board)
        p = sorting.plan(board, algorithm)        # raises sorting.PlanError with a readable message
        p["current"] = None
        p["done"] = 0
        with self.lock:
            self.sort = p
        return p

    def run_sort(self, algorithm=SORT_ALGORITHM):
        p = self.make_plan(algorithm)
        if not p["moves"]:
            return 200, "Already sorted: nothing to do.", p
        code, msg = self.run_moves([(m["src"], m["dst"]) for m in p["moves"]], "Sort")
        return code, msg, p

    def _program_tick(self, dt):
        """Inside the control loop, lock held: advance the sequencer."""
        p = self.program
        while p["i"] < len(p["steps"]):
            st = p["steps"][p["i"]]
            if st["do"] == "event":           # instant: update the board, then go on
                ev = st["event"]
                if ev == "move":
                    p["move"] = st["n"] or None
                    if self.sort is not None and st["n"]:
                        self.sort["current"] = st["n"]
                elif ev == "pick":
                    self.holding = self.board.get(str(st["cell"])) or "block"
                    self.board[str(st["cell"])] = None
                elif ev == "place":
                    self.board[str(st["cell"])] = self.holding
                    self.holding = None
                    if self.sort is not None and p["move"]:
                        self.sort["done"] = p["move"]
                p["i"] += 1
                continue
            if "_t" not in st:                 # first tick of this step
                st["_t"] = 0.0
                if st["do"] == "arm":
                    st["_from"] = {j: self.cmd[j] for j in ARM}
                    span = max(abs(st["pose"][j] - self.cmd[j]) for j in ARM)
                    st["_T"] = span / max(self.speed, 1e-3)
                else:
                    self.target["gripper"] = st["value"]
            st["_t"] += dt
            if st["do"] == "arm":
                T = st["_T"]
                s = 1.0 if T <= 0 else min(st["_t"] / T, 1.0)
                for j in ARM:                  # straight line in joint space: all joints arrive together
                    self.target[j] = st["_from"][j] + s * (st["pose"][j] - st["_from"][j])
                if s < 1.0 or max(abs(self.target[j] - self.cmd[j]) for j in ARM) > SETTLE_TOL:
                    return
                err = max(abs(self.actual[j] - st["pose"][j]) for j in ARM)
                if err > ARRIVE_TOL:           # not there yet: wait while it is still moving, never abort
                    prev = st.get("_prev")
                    moving = prev is not None and max(abs(self.actual[j] - prev[j]) for j in ARM) > STILL_DEG
                    st["_prev"] = dict(self.actual)
                    st["_still"] = 0.0 if moving or prev is None else st.get("_still", 0.0) + dt
                    if st["_still"] < STILL_S and st["_t"] - T < ARRIVE_WAIT:
                        return
                p["worst"] = max(p.get("worst", 0.0), err)
            else:
                if abs(self.cmd["gripper"] - st["value"]) > SETTLE_TOL:
                    st["_t"] = 0.0             # dwell counts from when the command arrives
                    return
                if st["_t"] < GRIP_DWELL:
                    return
            p["i"] += 1
        worst = p.get("worst", 0.0)
        self.program_msg = f"{p['name']} finished." + (
            f" Largest pose error {worst:.1f} deg (carried on)." if worst > ARRIVE_TOL else "")
        self.program = None
        if p["name"] == "Sort" and self.scan and self.scan.get("files"):   # demo done: delete its photo
            self._forget_scan(self.scan)
            self.scan["msg"] = "Sorted. The photo has been deleted." if not self.keep_scans else "Sorted."

    def _program_status(self):
        p = self.program
        if p is None:
            return {"active": False, "msg": self.program_msg}
        st = p["steps"][min(p["i"], len(p["steps"]) - 1)]
        return {"active": True, "name": p["name"], "step": p["i"] + 1, "count": len(p["steps"]),
                "label": st["label"], "move": p["move"]}

    def start_recording(self, name, overwrite):
        try:
            name, path = motion_path(name, default_if_empty=True)
        except ValueError as e:
            return 400, str(e), ""
        if os.path.exists(path) and not overwrite:
            return 409, f"{name} exists", name
        with self.lock:
            if self.rec_file is not None:
                return 400, "Already recording.", name
            if self.replay is not None:
                return 400, "Stop the replay before recording.", name
            os.makedirs(RECORD_DIR, exist_ok=True)
            self.rec_file = open(path, "w")
            self.rec_path = name
            self.rec_t0 = time.perf_counter()
        return 200, f"Recording to recorded_motions/{name} ...", name

    # ---- replay -----------------------------------------------------------
    def start_replay(self, names):
        """Play one or more recordings in order, through the same speed cap as everything else.

        Each file starts with an 'approach' phase: the arm moves to the file's first frame at
        max speed. Then 'play': the target follows the recording in real time. If the recording
        is faster than the current max speed, the replay clock pauses until the arm catches up,
        so the path is kept and only the timing stretches.
        """
        if not names:
            return 400, "Pick at least one recording."
        try:
            motions = [load_motion(n, self.limits) for n in names]
            names = [motion_path(n)[0] for n in names]
        except (ValueError, OSError) as e:
            return 400, f"Cannot replay: {e}"
        with self.lock:
            if self.rec_file is not None:
                return 400, "Stop recording before replaying."
            if self.program is not None or self.teach is not None:
                return 409, "Leave teach mode / stop the running program before replaying."
            self.replay = {"names": names, "motions": motions, "k": 0,
                           "phase": "approach", "clock": 0.0, "i": 0}
            self.replay_msg = ""
        total = sum(m[-1][0] for m in motions)
        return 200, f"Replaying {len(names)} file(s), {total:.1f} s of motion"

    def _replay_step(self, dt):
        """Called inside the control loop with the lock held: moves self.target along the recording."""
        r = self.replay
        frames = r["motions"][r["k"]]
        lag = max(abs(self.target[j] - self.cmd[j]) for j in JOINTS)

        if r["phase"] == "approach":
            self.target.update(frames[0][1])
            if max(abs(self.target[j] - self.cmd[j]) for j in JOINTS) < SETTLE_TOL:
                r.update(phase="play", clock=0.0, i=0)
            return

        if lag < REPLAY_LAG_TOL:                  # only let time run while the arm keeps up
            r["clock"] += dt
        t = r["clock"]
        while r["i"] + 1 < len(frames) and frames[r["i"] + 1][0] <= t:
            r["i"] += 1
        i = r["i"]
        if i + 1 < len(frames):                   # interpolate between neighbouring frames
            (ta, a), (tb, b) = frames[i], frames[i + 1]
            w = 0.0 if tb <= ta else (t - ta) / (tb - ta)
            self.target.update({j: a[j] + w * (b.get(j, a[j]) - a[j]) for j in a})
            return

        self.target.update(frames[-1][1])         # last frame: wait to arrive, then next file
        if lag < SETTLE_TOL:
            if r["k"] + 1 < len(r["motions"]):
                r.update(k=r["k"] + 1, phase="approach", clock=0.0, i=0)
            else:
                self.replay_msg = "Replay finished: " + ", ".join(r["names"])
                self.replay = None

    def _replay_status(self):
        r = self.replay
        if r is None:
            return {"active": False, "msg": self.replay_msg}
        frames = r["motions"][r["k"]]
        dur = frames[-1][0]
        return {"active": True, "file": r["names"][r["k"]], "index": r["k"], "count": len(r["names"]),
                "phase": r["phase"], "t": r["clock"], "duration": dur,
                "progress": 0.0 if r["phase"] == "approach" else (min(r["clock"] / dur, 1.0) if dur > 0 else 1.0)}

    def stop_recording(self):
        with self.lock:
            if self.rec_file is None:
                return "Not recording."
            self.rec_file.close()
            self.rec_file = None
            return f"Saved recorded_motions/{self.rec_path}"

    def snapshot(self):
        with self.lock:
            return {
                "actual": dict(self.actual),
                "t": self.actual_t,
                "cmd": dict(self.cmd),
                "target": dict(self.target),
                "speed": self.speed,
                "recording": self.rec_file is not None,
                "rec_path": self.rec_path,
                "replay": self._replay_status(),
                "error": self.error,
                "teach": None if self.teach is None else {"free": self.teach["free"], "msg": self.teach_msg},
                "program": self._program_status(),
                "board": dict(self.board),
                "holding": self.holding,
                "sort": self.sort,
                "scan": self.scan,
                "poses_version": self.poses_version,
            }

    def config(self):
        return {
            "joints": JOINTS,
            "limits": {j: list(v) for j, v in self.limits.items()},
            "start_pose": self.start_pose,
            "speed_ceiling": SPEED_CEILING,
            "loop_hz": LOOP_HZ,
            "twin_calib": load_twin_calib(),
            "algorithms": sorting.ALGORITHMS,
            "targets": {k: list(v) for k, v in sorting.DEFAULT_TARGETS.items()},
        }

    def poses_info(self):
        with self.lock:
            return {"version": self.poses_version, **self.poses}

    # ---- control loop -----------------------------------------------------
    def _loop(self):
        last = time.perf_counter()
        while self.running:
            t0 = time.perf_counter()
            dt = min(t0 - last, 0.1)
            last = t0
            try:
                with self.lock:
                    if self.replay is not None:
                        self._replay_step(dt)
                    if self.program is not None:
                        self._program_tick(dt)
                    arm_free = self.teach is not None and self.teach["free"]
                    if arm_free:
                        for j in ARM:          # torque off: remember where the hand put the arm
                            self.cmd[j] = self.target[j] = self.actual[j]
                    max_step = self.speed * dt
                    for j in JOINTS:
                        diff = self.target[j] - self.cmd[j]
                        self.cmd[j] += max(-max_step, min(max_step, diff))
                    action = {f"{j}.pos": self.cmd[j] for j in JOINTS}

                # While the arm is free, send a goal to the gripper only. A goal written to a
                # torque-off STS3215 can switch its torque back on, which makes the arm stiff.
                self.robot.send_action({"gripper.pos": action["gripper.pos"]} if arm_free else action)
                obs = self.robot.get_observation()
                actual = {j: float(obs[f"{j}.pos"]) for j in JOINTS}

                with self.lock:
                    self.actual = actual
                    self.actual_t = time.monotonic()
                    self.error = ""
                    if self.rec_file is not None:
                        entry = {"t": t0 - self.rec_t0, "action": action}
                        self.rec_file.write(json.dumps(entry) + "\n")
                    jobs, self.requests = self.requests, []
                for req in jobs:                   # torque on/off: only this thread touches the bus
                    self._run_request(req, actual)
                if arm_free and not jobs and t0 - self._torque_checked > TORQUE_CHECK_S:
                    self._torque_checked = t0
                    self._check_free()
            except Exception as e:   # keep looping; surface the problem in the UI
                with self.lock:
                    self.error = str(e)

            time.sleep(max(0.0, self.period - (time.perf_counter() - t0)))

    def shutdown(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.stop_recording()


# --------------------------------------------------------------------------
# HTTP
# --------------------------------------------------------------------------
def make_handler(ctrl):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):   # keep the terminal quiet (20 Hz polling)
            pass

        def _json(self, code, obj):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        @staticmethod
        def _cm(result):
            code, msg = result
            return code, {"msg": msg}

        def _file(self, root, rel):
            path = os.path.realpath(os.path.join(root, rel))
            if not path.startswith(os.path.realpath(root) + os.sep) or not os.path.isfile(path):
                self.send_error(404)
                return
            ctype = mimetypes.guess_type(path)[0] or "application/octet-stream"
            with open(path, "rb") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            p = urlparse(self.path).path
            if p == "/api/state":
                return self._json(200, ctrl.snapshot())
            if p == "/api/config":
                return self._json(200, ctrl.config())
            if p == "/api/recordings":
                return self._json(200, {"files": list_motions()})
            if p == "/api/poses":
                return self._json(200, ctrl.poses_info())
            if p == "/teach":
                return self._file(WEB_ROOT, "teach.html")
            if p.startswith("/urdf/"):
                return self._file(URDF_ROOT, p[len("/urdf/"):])
            return self._file(WEB_ROOT, "index.html" if p == "/" else p.lstrip("/"))

        def do_POST(self):
            p = urlparse(self.path).path
            n = int(self.headers.get("Content-Length") or 0)
            if p == "/api/scan":                     # raw JPEG/PNG body, not JSON
                if n <= 0 or n > MAX_SCAN_BYTES:
                    return self._json(413 if n > 0 else 400, {"msg": "send one photo (JPEG or PNG, under 20 MB)"})
                data = self.rfile.read(n)
                q = parse_qs(urlparse(self.path).query)
                source = (q.get("source") or ["photo"])[0][:20]
                return self._json(*ctrl.scan_image(data, self.headers.get("Content-Type"), source))
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except json.JSONDecodeError:
                return self._json(400, {"msg": "bad json"})

            if p == "/api/target":
                why = ctrl.set_targets(body)
                if why:
                    return self._json(409, {"msg": why})
                return self._json(200, {"ok": True})
            if p == "/api/speed":
                ctrl.set_speed(body.get("speed", DEFAULT_SPEED))
                return self._json(200, {"ok": True})
            if p == "/api/stop":
                ctrl.stop()
                return self._json(200, {"msg": "Stopped (target = current command)"})
            if p == "/api/go_start":
                if not ctrl.go_start():
                    return self._json(409, {"msg": "Busy (replay, program or free arm): press STOP first"})
                return self._json(200, {"msg": "Returning to start pose"})
            if p == "/api/record/start":
                code, msg, name = ctrl.start_recording(body.get("file", ""), bool(body.get("overwrite")))
                return self._json(code, {"msg": msg, "file": name})
            if p == "/api/record/stop":
                return self._json(200, {"msg": ctrl.stop_recording()})
            if p == "/api/replay/start":
                files = body.get("files") or []
                if not isinstance(files, list) or not all(isinstance(f, str) for f in files):
                    return self._json(400, {"msg": "files must be a list of names"})
                code, msg = ctrl.start_replay(files)
                return self._json(code, {"msg": msg})
            if p == "/api/replay/stop":
                ctrl.stop()
                return self._json(200, {"msg": "Replay stopped"})
            # ---- sorting demo: teaching --------------------------------------
            if p == "/api/teach/start":
                return self._json(*self._cm(ctrl.teach_start()))
            if p == "/api/teach/stop":
                return self._json(*self._cm(ctrl.teach_stop()))
            if p == "/api/teach/free":
                return self._json(*self._cm(ctrl.teach_free()))
            if p == "/api/teach/lock":
                code, msg = ctrl.teach_lock(body.get("slot") or "")
                return self._json(code, {"msg": msg, "poses": ctrl.poses_info()})
            if p == "/api/teach/goto":
                return self._json(*self._cm(ctrl.goto_slot(str(body.get("slot", "")))))
            if p == "/api/teach/test_cell":
                try:
                    n = int(body.get("cell"))
                    assert 1 <= n <= 9
                except (TypeError, ValueError, AssertionError):
                    return self._json(400, {"msg": "cell must be 1 to 9"})
                return self._json(*self._cm(ctrl.test_cell(n)))
            if p == "/api/poses/gripper":
                try:
                    return self._json(200, {"msg": "saved", "gripper": ctrl.set_gripper_values(body)})
                except (TypeError, ValueError) as e:
                    return self._json(400, {"msg": str(e)})
            if p == "/api/go_home":
                return self._json(*self._cm(ctrl.go_home()))
            if p == "/api/sim/hand":                 # --sim only: pretend to move the free arm by hand
                if not isinstance(ctrl.robot, SimRobot):
                    return self._json(404, {"msg": "only with --sim"})
                ctrl.robot.hand({k: v for k, v in body.items() if k in JOINTS})
                return self._json(200, {"ok": True})
            # ---- sorting demo: board, plan, run ---------------------------------
            if p == "/api/board":
                try:
                    board = ctrl.set_board(body.get("cells"))
                    if ctrl.scan:                    # the board no longer comes from the last scan
                        ctrl._forget_scan(ctrl.scan)
                    ctrl.scan = None
                    return self._json(200, {"board": board})
                except (ValueError, TypeError, AttributeError) as e:
                    return self._json(400, {"msg": str(e)})
            if p in ("/api/sort/plan", "/api/sort/run"):
                alg = str(body.get("algorithm") or SORT_ALGORITHM)
                try:
                    if p == "/api/sort/plan":
                        plan = ctrl.make_plan(alg)
                        return self._json(200, {"msg": plan["summary"]["short"], "plan": plan})
                    code, msg, plan = ctrl.run_sort(alg)
                    return self._json(code, {"msg": msg, "plan": plan})
                except sorting.PlanError as e:
                    return self._json(400, {"msg": str(e)})
            if p == "/api/sort/move":
                try:
                    src, dst = int(body.get("src")), int(body.get("dst"))
                    assert 1 <= src <= 9 and 1 <= dst <= 9 and src != dst
                except (TypeError, ValueError, AssertionError):
                    return self._json(400, {"msg": "src and dst must be different cells 1 to 9"})
                return self._json(*self._cm(ctrl.run_moves([(src, dst)], f"Move cell {src} to {dst}")))
            if p == "/api/twin_calib":
                try:
                    save_twin_calib(body)
                except (OSError, ValueError, TypeError, AttributeError) as e:
                    return self._json(400, {"msg": str(e)})
                return self._json(200, {"msg": "saved"})
            self.send_error(404)

    return Handler


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port, e.g. /dev/ttyACM0")
    ap.add_argument("--id", help="calibration id, e.g. asherbot")
    ap.add_argument("--sim", action="store_true", help="no hardware: drive a fake arm")
    ap.add_argument("--max-speed", type=float, default=DEFAULT_SPEED,
                    help=f"initial speed cap in deg/s (hard ceiling {SPEED_CEILING:.0f})")
    ap.add_argument("--host", default="127.0.0.1",
                    help="bind address. Keep 127.0.0.1 unless you WANT other machines to move the arm")
    ap.add_argument("--http-port", type=int, default=8000)
    ap.add_argument("--detector", default=DETECTOR_URL,
                    help="block detector service for Scan (detector/locate_service.py)")
    ap.add_argument("--keep-scans", action="store_true",
                    help="keep scan photos (default: delete a photo when its sort finishes or a new scan replaces it)")
    args = ap.parse_args()

    if args.sim:
        robot = SimRobot()
    else:
        if not (args.port and args.id):
            ap.error("--port and --id are required unless --sim is given")
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
        robot = SO101Follower(SO101FollowerConfig(port=args.port, id=args.id))
    robot.connect()   # may prompt in the terminal if calibration is missing

    try:
        os.makedirs(RECORD_DIR, exist_ok=True)
        ctrl = Controller(robot, args.max_speed)
        ctrl.detector_url = args.detector.rstrip("/")
        ctrl.keep_scans = args.keep_scans
    except Exception:
        robot.disconnect()
        raise
    print("Start pose:", "  |  ".join(f"{j}: {v:.1f}" for j, v in ctrl.start_pose.items()))
    ctrl.thread.start()

    server = ThreadingHTTPServer((args.host, args.http_port), make_handler(ctrl))
    server.daemon_threads = True
    print(f"Digital twin at http://{args.host}:{args.http_port}  ({'SIM' if args.sim else args.port})")
    print("Ctrl+C to quit (this releases motor torque, so support the arm).")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        ctrl.shutdown()
        robot.disconnect()   # releases torque (LeRobot default)


if __name__ == "__main__":
    main()
