"""
Knob (dial) teleop GUI for the SO-101 -- no leader arm, no keyboard, no pynput.

Each motor gets a speedometer-style dial:
    white needle  = where the motor actually is right now
    orange marker = where you told it to go (drag the dial with the mouse)
    grey dot      = the angle the motor had when the app started

Safety: the commanded position chases the target at no more than
"Max speed" (deg/s; gripper: units/s), however fast the mouse moves.
The slider can never exceed SPEED_CEILING.

Optional recording writes the same JSON-lines format as teleop_and_record.py,
so replay_motion.py and block_trigger_watcher.py work on the files unchanged.

Run:
    python knob_teleop_gui.py --port /dev/ttyACM0 --id asherbot
    python knob_teleop_gui.py --port /dev/ttyACM0 --id asherbot --max-speed 20

Only needs tkinter (ships with most Python installs; works under Wayland
through XWayland).
"""

import argparse
import json
import math
import os
import time
import tkinter as tk
from tkinter import messagebox

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
UNITS = {j: "\u00b0" for j in JOINTS}
UNITS["gripper"] = "%"

LOOP_HZ = 30.0
SPEED_CEILING = 90.0   # hard upper limit for the speed slider (deg/s)
DEFAULT_SPEED = 30.0   # starting value of the speed slider (deg/s)
ENCODER_MAX = 4095     # STS3215 resolution - 1

BG = "#1e1f24"
PANEL = "#272932"
TRACK = "#3a3c46"
ACCENT = "#4aa3ff"
TARGET = "#ff9f1a"
FG = "#e8e8e8"
DIM = "#8a8d99"


def joint_limits(robot, joint):
    """Dial range for a joint, derived from the robot's calibration.

    In degrees mode LeRobot maps raw ticks to (raw - mid) * 360 / 4095, so the
    reachable range is +/- half of the calibrated span. The gripper is 0..100.
    """
    if joint == "gripper":
        return 0.0, 100.0
    cal = (getattr(robot, "calibration", None) or {}).get(joint)
    if cal is None:
        return -100.0, 100.0
    half = (cal.range_max - cal.range_min) / 2.0 * 360.0 / ENCODER_MAX
    return -half, half


class Dial(tk.Canvas):
    START = 225.0   # math-convention angle of the minimum (lower left)
    SWEEP = 270.0   # clockwise sweep to the maximum (lower right)

    def __init__(self, master, name, vmin, vmax, start_value, unit, on_target, dim=230):
        super().__init__(master, width=dim, height=dim - 15, bg=PANEL, highlightthickness=0)
        self.name = name
        self.vmin, self.vmax = vmin, vmax
        self.unit = unit
        self.start_value = start_value
        self.on_target = on_target
        self.dim = dim
        self.cx = dim / 2
        self.cy = dim / 2 + 5
        self.r = dim * 0.34
        self.actual = start_value
        self.target = start_value
        self._last_frac = None
        self.bind("<Button-1>", self._press)
        self.bind("<B1-Motion>", self._motion)
        self.redraw()

    # ---- geometry helpers -------------------------------------------------
    def _frac(self, value):
        value = min(max(value, self.vmin), self.vmax)
        return (value - self.vmin) / (self.vmax - self.vmin)

    def _pos(self, frac, radius):
        a = math.radians(self.START - frac * self.SWEEP)
        return self.cx + radius * math.cos(a), self.cy - radius * math.sin(a)

    # ---- mouse ------------------------------------------------------------
    def _press(self, event):
        self._last_frac = None
        self._handle(event)

    def _motion(self, event):
        self._handle(event)

    def _handle(self, event):
        dx, dy = event.x - self.cx, event.y - self.cy
        if math.hypot(dx, dy) < 10:
            return
        theta = math.degrees(math.atan2(-dy, dx))
        cw = (self.START - theta) % 360.0          # degrees clockwise from the minimum
        if cw > self.SWEEP:                         # mouse is in the dead zone at the bottom
            cw = self.SWEEP if cw <= self.SWEEP + (360.0 - self.SWEEP) / 2 else 0.0
        frac = cw / self.SWEEP
        # Don't let the target wrap from max to min when the cursor crosses the dead zone.
        if self._last_frac is not None and abs(frac - self._last_frac) > 0.6:
            return
        self._last_frac = frac
        self.on_target(self.name, self.vmin + frac * (self.vmax - self.vmin))

    # ---- drawing ----------------------------------------------------------
    def set_state(self, actual, target):
        if abs(actual - self.actual) > 0.05 or abs(target - self.target) > 0.05:
            self.actual, self.target = actual, target
            self.redraw()

    def redraw(self):
        self.delete("all")
        cx, cy, r = self.cx, self.cy, self.r
        box = (cx - r, cy - r, cx + r, cy + r)

        self.create_text(cx, 12, text=self.name.replace("_", " ").title(), fill=FG,
                         font=("TkDefaultFont", 11, "bold"))
        self.create_arc(*box, start=self.START, extent=-self.SWEEP, style="arc", width=14, outline=TRACK)

        af = self._frac(self.actual)
        if af > 0.002:
            self.create_arc(*box, start=self.START, extent=-af * self.SWEEP, style="arc",
                            width=14, outline=ACCENT)

        for i in range(11):
            f = i / 10
            x1, y1 = self._pos(f, r + 11)
            x2, y2 = self._pos(f, r + (20 if i % 5 == 0 else 16))
            self.create_line(x1, y1, x2, y2, fill=DIM, width=2 if i % 5 == 0 else 1)

        sx, sy = self._pos(self._frac(self.start_value), r)
        self.create_oval(sx - 4, sy - 4, sx + 4, sy + 4, fill=DIM, outline=PANEL)

        tf = self._frac(self.target)
        tx1, ty1 = self._pos(tf, r - 12)
        tx2, ty2 = self._pos(tf, r + 12)
        self.create_line(tx1, ty1, tx2, ty2, fill=TARGET, width=4)

        nx, ny = self._pos(af, r - 8)
        self.create_line(cx, cy, nx, ny, fill="white", width=3)
        self.create_oval(cx - 7, cy - 7, cx + 7, cy + 7, fill=FG, outline=PANEL)

        self.create_text(cx, cy + r * 0.50, text=f"{self.actual:.1f}{self.unit}", fill=FG,
                         font=("TkDefaultFont", 15, "bold"))
        self.create_text(cx, cy + r * 0.50 + 20, text=f"target {self.target:.1f}{self.unit}",
                         fill=TARGET, font=("TkDefaultFont", 9))
        lx, ly = self._pos(0.0, r * 0.95)
        rx, ry = self._pos(1.0, r * 0.95)
        self.create_text(lx, ly + 22, text=f"{self.vmin:.0f}", fill=DIM, font=("TkDefaultFont", 8))
        self.create_text(rx, ry + 22, text=f"{self.vmax:.0f}", fill=DIM, font=("TkDefaultFont", 8))


class App:
    def __init__(self, root, robot, max_speed):
        self.root = root
        self.robot = robot
        self.period = 1.0 / LOOP_HZ
        self.rec_file = None
        self.rec_t0 = 0.0
        self._after_id = None

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

        root.title("SO-101 knob teleop")
        root.configure(bg=BG)

        grid = tk.Frame(root, bg=BG)
        grid.pack(padx=10, pady=(10, 4))
        self.dials = {}
        for i, j in enumerate(JOINTS):
            lo, hi = self.limits[j]
            d = Dial(grid, j, lo, hi, self.start_pose[j], UNITS[j], self.set_target)
            d.grid(row=i // 3, column=i % 3, padx=5, pady=5)
            self.dials[j] = d

        controls = tk.Frame(root, bg=BG)
        controls.pack(fill="x", padx=14, pady=4)

        self.speed = tk.DoubleVar(value=min(max_speed, SPEED_CEILING))
        tk.Label(controls, text="Max speed (deg/s)", bg=BG, fg=FG).grid(row=0, column=0, sticky="w")
        tk.Scale(controls, from_=1, to=SPEED_CEILING, resolution=1, orient="horizontal",
                 variable=self.speed, length=260, bg=BG, fg=FG, troughcolor=TRACK,
                 highlightthickness=0).grid(row=0, column=1, padx=8)

        tk.Button(controls, text="Stop", width=8, command=self.stop).grid(row=0, column=2, padx=4)
        tk.Button(controls, text="Go to start pose", command=self.go_start).grid(row=0, column=3, padx=4)

        rec = tk.Frame(root, bg=BG)
        rec.pack(fill="x", padx=14, pady=4)
        tk.Label(rec, text="Record to", bg=BG, fg=FG).pack(side="left")
        self.fname = tk.StringVar(value="recording.jsonl")
        tk.Entry(rec, textvariable=self.fname, width=28).pack(side="left", padx=8)
        self.rec_btn = tk.Button(rec, text="Record", width=14, command=self.toggle_record)
        self.rec_btn.pack(side="left")

        self.status = tk.StringVar()
        tk.Label(root, textvariable=self.status, bg=BG, fg=DIM, anchor="w", justify="left",
                 wraplength=700).pack(fill="x", padx=14, pady=(4, 0))
        tk.Label(root, text="white needle = actual    orange = target    grey dot = start angle",
                 bg=BG, fg=DIM, anchor="w").pack(fill="x", padx=14, pady=(0, 10))

        start_txt = "  |  ".join(f"{j}: {self.start_pose[j]:.1f}{UNITS[j]}" for j in JOINTS)
        self.status.set("Started at -> " + start_txt)
        print("Start pose:", start_txt)

        root.protocol("WM_DELETE_WINDOW", self.close)
        self.last = time.perf_counter()
        self._after_id = root.after(int(self.period * 1000), self.tick)

    # ---- user actions -----------------------------------------------------
    def set_target(self, joint, value):
        lo, hi = self.limits[joint]
        self.target[joint] = min(max(value, lo), hi)

    def stop(self):
        self.target = dict(self.cmd)

    def go_start(self):
        self.target = dict(self.start_pose)

    def toggle_record(self):
        if self.rec_file is None:
            path = self.fname.get().strip()
            if not path:
                self.status.set("Enter a file name to record to.")
                return
            if os.path.exists(path) and not messagebox.askyesno("Overwrite?", f"{path} exists. Overwrite it?"):
                return
            self.rec_file = open(path, "w")
            self.rec_t0 = time.perf_counter()
            self.rec_btn.config(text="Stop recording", bg="#c0392b")
            self.status.set(f"Recording to {path} ...")
        else:
            self._stop_recording()

    def _stop_recording(self):
        if self.rec_file is not None:
            self.rec_file.close()
            self.rec_file = None
            self.rec_btn.config(text="Record", bg=self.root.cget("bg"))
            self.status.set(f"Saved {self.fname.get().strip()}")

    # ---- control loop -----------------------------------------------------
    def tick(self):
        t0 = time.perf_counter()
        dt = min(t0 - self.last, 0.1)
        self.last = t0
        try:
            max_step = self.speed.get() * dt
            for j in JOINTS:
                diff = self.target[j] - self.cmd[j]
                self.cmd[j] += max(-max_step, min(max_step, diff))

            self.robot.send_action({f"{j}.pos": self.cmd[j] for j in JOINTS})

            obs = self.robot.get_observation()
            self.actual = {j: float(obs[f"{j}.pos"]) for j in JOINTS}

            if self.rec_file is not None:
                entry = {"t": t0 - self.rec_t0, "action": {f"{j}.pos": self.cmd[j] for j in JOINTS}}
                self.rec_file.write(json.dumps(entry) + "\n")

            for j in JOINTS:
                self.dials[j].set_state(self.actual[j], self.target[j])
        except Exception as e:  # keep the UI alive; show what went wrong
            self.status.set(f"Error: {e}")

        elapsed_ms = (time.perf_counter() - t0) * 1000
        delay = max(1, int(self.period * 1000 - elapsed_ms))
        self._after_id = self.root.after(delay, self.tick)

    def close(self):
        if self._after_id is not None:
            self.root.after_cancel(self._after_id)
        self._stop_recording()
        try:
            self.robot.disconnect()   # NOTE: this releases motor torque (LeRobot default)
        finally:
            self.root.destroy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="e.g. /dev/ttyACM0")
    parser.add_argument("--id", required=True, help="calibration id, e.g. asherbot")
    parser.add_argument("--max-speed", type=float, default=DEFAULT_SPEED,
                        help=f"initial speed cap in deg/s (hard ceiling {SPEED_CEILING:.0f})")
    args = parser.parse_args()

    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    robot = SO101Follower(SO101FollowerConfig(port=args.port, id=args.id))
    robot.connect()   # may prompt in the terminal if calibration is missing

    root = tk.Tk()
    try:
        App(root, robot, args.max_speed)
    except Exception:
        robot.disconnect()
        raise
    root.mainloop()


if __name__ == "__main__":
    main()