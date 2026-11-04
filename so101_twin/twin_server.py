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

Recording writes the same JSON-lines format as teleop_and_record.py /
knob_teleop.py, so replay_motion.py and block_trigger_watcher.py still work.

Only the standard library is needed (plus lerobot when not in --sim).
"""

import argparse
import json
import mimetypes
import os
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
        self.running = True

        # Read where the arm actually is BEFORE sending anything, so nothing jumps.
        obs = robot.get_observation()
        self.start_pose = {j: float(obs[f"{j}.pos"]) for j in JOINTS}
        self.actual = dict(self.start_pose)
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
        with self.lock:
            for j, v in values.items():
                if j in self.limits:
                    lo, hi = self.limits[j]
                    self.target[j] = min(max(float(v), lo), hi)

    def set_speed(self, v):
        with self.lock:
            self.speed = min(max(float(v), 1.0), SPEED_CEILING)

    def stop(self):
        with self.lock:
            self.target = dict(self.cmd)

    def go_start(self):
        with self.lock:
            self.target = dict(self.start_pose)

    def start_recording(self, path, overwrite):
        path = path.strip()
        if not path:
            return 400, "Enter a file name to record to."
        if os.path.exists(path) and not overwrite:
            return 409, f"{path} exists"
        with self.lock:
            if self.rec_file is not None:
                return 400, "Already recording."
            self.rec_file = open(path, "w")
            self.rec_path = path
            self.rec_t0 = time.perf_counter()
        return 200, f"Recording to {path} ..."

    def stop_recording(self):
        with self.lock:
            if self.rec_file is None:
                return "Not recording."
            self.rec_file.close()
            self.rec_file = None
            return f"Saved {self.rec_path}"

    def snapshot(self):
        with self.lock:
            return {
                "actual": dict(self.actual),
                "cmd": dict(self.cmd),
                "target": dict(self.target),
                "speed": self.speed,
                "recording": self.rec_file is not None,
                "rec_path": self.rec_path,
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
                ctrl.set_targets(body)
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
                code, msg = ctrl.start_recording(body.get("file", ""), bool(body.get("overwrite")))
                return self._json(code, {"msg": msg})
            if p == "/api/record/stop":
                return self._json(200, {"msg": ctrl.stop_recording()})
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
