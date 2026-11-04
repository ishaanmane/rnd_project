# SO-101 Digital Twin & Teleoperation

A research project exploring **leader-arm-free teleoperation** for the open-source [SO-101 robotic arm](https://github.com/TheRobotStudio/SO-ARM100). Instead of requiring a physical leader arm, the robot is controlled through a web-based 3D digital twin, keyboard controls, a knob-based GUI, or an XR headset (Meta Quest 3) via NVIDIA Isaac Teleop — all while supporting motion recording for dataset collection.

---

## Features

| Approach | Script | Input | Requires Display? |
|---|---|---|---|
| **Browser digital twin** | `so101_twin/twin_server.py` | Drag the 3D model in your browser | No (serves a web page) |
| **Keyboard teleop** | `keyboard_teleop_so101.py` | Terminal keypresses (works over SSH) | No |
| **Knob (dial) GUI** | `knob_teleop.py` | Tkinter speedometer dials + mouse | Yes (X11 / XWayland) |
| **XR controller (Quest 3)** | `lerobot/run_xr_teleop.py` | Meta Quest 3 via NVIDIA Isaac Teleop | CloudXR runtime |

All teleoperation methods write the same **JSON-lines recording format**, so `replay_motion.py` works with recordings from any of them.

---

## Repository Structure

```
ishaan_rnd/
├── README.md                       # ← you are here
├── environment.yml                 # conda environment (Python 3.12, lerobot, isaacteleop, etc.)
│
├── keyboard_teleop_so101.py        # terminal-based keyboard teleop (no GUI, SSH-friendly)
├── knob_teleop.py                  # tkinter knob/dial GUI for joint-level control
├── replay_motion.py                # replay a recorded .jsonl motion file on the robot
│
├── so101_twin/                     # browser-based 3D digital twin
│   ├── twin_server.py              # Python HTTP server + 30 Hz control loop
│   ├── twin_calib.json             # visual alignment offsets (twin ↔ real arm)
│   ├── web/
│   │   ├── index.html              # Three.js twin + control panel (single-file SPA)
│   │   └── vendor/                 # three.js r160 + urdf-loader 0.13 (works offline)
│   ├── urdf/
│   │   ├── so101.urdf              # SO-101 URDF from TheRobotStudio/SO-ARM100
│   │   └── assets/*.stl            # mesh files
│   └── README.md                   # detailed twin-specific docs
│
└── lerobot/                        # LeRobot fork with Isaac Teleop integration
    ├── run_xr_teleop.py            # quick-start script to bypass draccus CLI issues
    ├── IsaacTeleop/                 # NVIDIA Isaac Teleop (cloned)
    └── examples/
        └── isaac_teleop_to_so101/  # XR controller → SO-101 example (teleop + record)
```

---

## Setup

### Prerequisites

- Linux workstation
- SO-101 follower arm, calibrated with `lerobot-calibrate`
- (Optional) Meta Quest 3 or other CloudXR-capable headset for XR teleoperation

### Environment

The full conda environment is captured in `environment.yml`. To recreate it:

```bash
conda env create -f environment.yml
conda activate ishaan_rnd
```

Key dependencies include:
- **[LeRobot](https://github.com/huggingface/lerobot)** `0.6.1` — robot control, calibration, dataset recording
- **[isaacteleop](https://nvidia.github.io/IsaacTeleop)** `1.3.131` — NVIDIA Isaac Teleop (XR path)
- **PyTorch**, **torchvision** — for LeRobot internals
- **tkinter** — for the knob GUI (ships with most Python installs)

---

## Usage

### 1. Browser Digital Twin

The flagship approach — drag the 3D arm in your browser and the real robot follows at a safe, capped speed.

```bash
# Try with a simulated (fake) arm first
python so101_twin/twin_server.py --sim

# Connect to the real arm
python so101_twin/twin_server.py --port /dev/ttyACM0 --id asherbot

# With a custom speed cap
python so101_twin/twin_server.py --port /dev/ttyACM0 --id asherbot --max-speed 20
```

Then open **http://127.0.0.1:8000** in a browser.

- **Solid arm** = actual motor positions (read back every tick)
- **Orange ghost** = target — drag it to move joints
- **Space / STOP** = freeze in place, **Start pose** = return to initial pose
- **Record** writes a `.jsonl` file compatible with `replay_motion.py`

See [`so101_twin/README.md`](so101_twin/README.md) for twin-specific docs (alignment, safety notes, etc.).

### 2. Keyboard Teleop

Minimal, SSH-friendly — no GUI needed.

```bash
python keyboard_teleop_so101.py --port /dev/ttyACM0 --id asherbot
```

| Keys | Joint |
|------|-------|
| `q` / `a` | Shoulder Pan +/− |
| `w` / `s` | Shoulder Lift +/− |
| `e` / `d` | Elbow Flex +/− |
| `r` / `f` | Wrist Flex +/− |
| `t` / `g` | Wrist Roll +/− |
| `y` / `h` | Gripper +/− |
| `x` | Quit |

Each keypress moves the joint by 2° (configurable via `STEP_DEG`).

### 3. Knob (Dial) GUI

A Tkinter-based interface with speedometer-style dials for each joint.

```bash
python knob_teleop.py --port /dev/ttyACM0 --id asherbot
python knob_teleop.py --port /dev/ttyACM0 --id asherbot --max-speed 20
```

- **White needle** = actual motor position
- **Orange marker** = your target (drag the dial with the mouse)
- **Grey dot** = position at startup
- Adjustable max speed slider with hard ceiling of 90 °/s
- Built-in recording to `.jsonl`

### 4. XR Teleoperation (Meta Quest 3 + Isaac Teleop)

Uses NVIDIA Isaac Teleop to map a Meta Quest 3 controller's grip pose to the SO-101 end-effector via Cartesian IK, with a squeeze-to-engage clutch.

```bash
# From inside the lerobot/ directory:
python run_xr_teleop.py
```

> **Note**: Edit `ROBOT_PORT` and `ROBOT_ID` in `run_xr_teleop.py` to match your setup.

For full documentation, installation steps, CloudXR setup, and dataset recording with XR, see [`lerobot/examples/isaac_teleop_to_so101/README.md`](lerobot/examples/isaac_teleop_to_so101/README.md).

---

## Recording & Replay

All teleoperation scripts can record motions as JSON-lines files (`.jsonl`). Each line contains:

```json
{"t": 0.033, "action": {"shoulder_pan.pos": 5.1, "shoulder_lift.pos": -2.2, ...}}
```

To replay a recorded motion on the robot:

```bash
python replay_motion.py --port /dev/ttyACM0 --id asherbot --file recording.jsonl
```

The replay includes a smooth ease-in from the arm's current position to the recording's first frame.

---

## Safety Notes

- The digital twin server binds to `127.0.0.1` by default. Use `--host 0.0.0.0` only if you intentionally want other machines on the network to control the arm.
- **Ctrl+C** disconnects from the robot, which releases motor torque (LeRobot default) — **support the arm before quitting**.
- All teleoperation modes enforce a configurable speed ceiling (default 30 °/s, hard max 90 °/s) so the motors can never move faster than you intend.

---

## Acknowledgements

- [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) — SO-101 hardware design & URDF
- [Hugging Face LeRobot](https://github.com/huggingface/lerobot) — robot control framework & dataset tooling
- [NVIDIA Isaac Teleop](https://github.com/NVIDIA/IsaacTeleop) — XR teleoperation framework
