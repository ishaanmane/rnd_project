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

Only the standard library is needed (plus lerobot when not in --sim).
"""

import argparse
import json
import mimetypes
import os
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

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


class SimRobot:
    """Stand-in for SO101Follower: follows commands with a little lag."""

    calibration = None

    def __init__(self):
        self.pos = {j: 0.0 for j in JOINTS}
        self.pos.update({"shoulder_pan": 5.1, "shoulder_lift": -2.2, "elbow_flex": 0.6, "wrist_flex": 90.5, "wrist_roll": 14.2, "gripper": 2.0})
        self.cmd = dict(self.pos)

    def connect(self):
        pass

    def get_observation(self):
        for j in JOINTS:                       # first-order lag, like a real servo
            self.pos[j] += 0.35 * (self.cmd[j] - self.pos[j])
        return {f"{j}.pos": v for j, v in self.pos.items()}

    def send_action(self, action):
        for j in JOINTS:
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
    def set_targets(self, values):
        """Returns False (and changes nothing) while a replay owns the arm."""
        with self.lock:
            if self.replay is not None:
                return False
            for j, v in values.items():
                if j in self.limits:
                    lo, hi = self.limits[j]
                    self.target[j] = min(max(float(v), lo), hi)
            return True

    def set_speed(self, v):
        with self.lock:
            self.speed = min(max(float(v), 1.0), SPEED_CEILING)

    def stop(self):
        with self.lock:
            if self.replay is not None:
                self.replay_msg = f"Replay of {self.replay['names'][self.replay['k']]} stopped"
            self.replay = None
            self.target = dict(self.cmd)

    def go_start(self):
        with self.lock:
            self.replay = None
            self.target = dict(self.start_pose)

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
            }

    def config(self):
        return {
            "joints": JOINTS,
            "limits": {j: list(v) for j, v in self.limits.items()},
            "start_pose": self.start_pose,
            "speed_ceiling": SPEED_CEILING,
            "loop_hz": LOOP_HZ,
            "twin_calib": load_twin_calib(),
        }

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
                    max_step = self.speed * dt
                    for j in JOINTS:
                        diff = self.target[j] - self.cmd[j]
                        self.cmd[j] += max(-max_step, min(max_step, diff))
                    action = {f"{j}.pos": self.cmd[j] for j in JOINTS}

                self.robot.send_action(action)
                obs = self.robot.get_observation()
                actual = {j: float(obs[f"{j}.pos"]) for j in JOINTS}

                with self.lock:
                    self.actual = actual
                    self.actual_t = time.monotonic()
                    self.error = ""
                    if self.rec_file is not None:
                        entry = {"t": t0 - self.rec_t0, "action": action}
                        self.rec_file.write(json.dumps(entry) + "\n")
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
            if p.startswith("/urdf/"):
                return self._file(URDF_ROOT, p[len("/urdf/"):])
            return self._file(WEB_ROOT, "index.html" if p == "/" else p.lstrip("/"))

        def do_POST(self):
            p = urlparse(self.path).path
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except json.JSONDecodeError:
                return self._json(400, {"msg": "bad json"})

            if p == "/api/target":
                if not ctrl.set_targets(body):
                    return self._json(409, {"msg": "Replay in progress: press STOP to take control"})
                return self._json(200, {"ok": True})
            if p == "/api/speed":
                ctrl.set_speed(body.get("speed", DEFAULT_SPEED))
                return self._json(200, {"ok": True})
            if p == "/api/stop":
                ctrl.stop()
                return self._json(200, {"msg": "Stopped (target = current command)"})
            if p == "/api/go_start":
                ctrl.go_start()
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
