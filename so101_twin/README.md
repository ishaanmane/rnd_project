# SO-101 browser digital twin

Drag the 3D arm in your browser; the real SO-101 follows at a capped speed.

```
so101_twin/
  twin_server.py      # owns the serial bus, runs the 30 Hz speed-limited loop, serves the page
  web/index.html      # Three.js twin + control panel
  web/vendor/         # three.js 0.160 + urdf-loader 0.13 (works offline)
  urdf/so101.urdf     # official so101_new_calib.urdf from TheRobotStudio/SO-ARM100
  urdf/assets/*.stl   # its meshes
```

## Run

```bash
cd ~/asherbot/so101_twin
python twin_server.py --sim                                   # try it with no robot first
python twin_server.py --port /dev/ttyACM0 --id asherbot       # real arm
python twin_server.py --port /dev/ttyACM0 --id asherbot --max-speed 20
```

Open http://127.0.0.1:8000. Run it from the same conda env as lerobot.
Only the standard library is needed on top of lerobot.

## Using it

- **Solid arm** = where the motors actually are (read back every tick).
- **Orange ghost** = target. Hover a part of the ghost: it lights up and shows the joint name.
  Drag it to rotate that joint. The jaw drags the gripper.
- Drag empty space to orbit, scroll to zoom. Side panel sliders give fine control.
- **Space / STOP** freezes the target where the arm currently is. **Start pose** goes back to
  the pose the arm had when the server started.
- **Record** saves to `so101_twin/recorded_motions/<name>.jsonl` (leave the name blank for a
  timestamped one). Same JSON-lines format as `teleop_and_record.py`, so `replay_motion.py` and
  `block_trigger_watcher.py` work on these files unchanged.
- **Replay…** lists everything in `recorded_motions/`. Click one or more files; they play in the
  order you clicked them. Each file first moves to its starting pose, then plays in real time.
  Replay obeys the max-speed slider: if a recording is faster than the cap, the replay slows down
  (same path, longer duration) instead of moving faster. While replaying, dragging and sliders are
  locked. **STOP / Space / Stop replay** ends it and hands control back. To replay old recordings,
  copy their `.jsonl` files into `recorded_motions/`.

## Aligning the twin with the real arm

LeRobot's 0 deg is whatever pose the arm held at the "middle of range" step of `lerobot-calibrate`.
The URDF's 0 deg is its own "L" pose (upper arm vertical, forearm horizontal). They differ, so each
joint needs `urdf_deg = sign * lerobot_deg + offset`. This lives in `twin_calib.json` and only
changes the drawing, never the motors.

The shipped file (shoulder_lift -90, elbow_flex +90) matches asherbot's folded rest pose.
To redo it: click **Align twin to real arm**, the motors lock, then drag the twin or use the
+-90 buttons until it looks like the real arm, and **Save**.

Then check direction: max speed ~10 deg/s, move one joint a little with its slider. If the twin turns
the opposite way, open Align again and tick **flip** for that joint (the current pose is kept).

## AR on a Meta Quest 3 (USB, Phase 0)

The same page runs in the Quest Browser as a WebXR passthrough session. WebXR only works on
https or `http://localhost`, so the Quest reaches this PC through `adb reverse` over a USB cable.
Nothing goes over the network and the server stays bound to 127.0.0.1.

One-time setup:
1. Meta Horizon phone app: Devices -> your Quest 3 -> Headset settings -> Developer Mode ON
   (needs a free Meta developer account). Reboot the headset.
2. `sudo apt install adb`
3. Connect the Quest to the PC with a data-capable USB-C cable, put it on, and accept
   "Allow USB debugging" (tick "Always allow").

Every session:
```bash
python twin_server.py --sim                       # or --port /dev/ttyACM0 --id asherbot
./quest_connect.sh                                # re-run after unplugging or rebooting the headset
```
In the Quest Browser open `http://localhost:8000` and press **Enter AR**. The twin appears about
0.5 m in front of you. Pull a trigger (or pinch) to put the robot's base where your controller tip is,
with the arm facing the way the controller points. Press the Meta button to leave AR.

Debugging: open Google Chrome on the PC at `chrome://inspect/#devices` to see the Quest page's console.

### Controls in AR

A help panel floats just below your line of sight and always shows the next step.

| Controller | Hands (wrist bar above the LEFT wrist, poke with the right index finger) | Does |
|---|---|---|
| **Y** | **Menu** | Open / close the AR menu |
| **B** | **Holo** | Hologram on / off. Labels and arcs stay, so a replay shows only the motor readings |
| left stick click | **Labels** | Labels: all -> moving only -> off |
| point + **trigger** | poke, or point a hand ray + pinch | Press a button on the AR menu |
| **A** (while a replay runs) | **STOP** (wrist bar, only while a replay runs) | Stop the replay |
| **X** (when not aligning) | **Help** | Hide / show the help panel (remembered). While hidden, a new message still pops up for 4 s |

**The AR menu** opens about 30 degrees to the left of where you are looking, so the robot ahead stays
visible: your recordings (tap files to select; they play in the order you tap, numbered), **Play**, a red
**STOP**, speed **- / +**, **Help panel**, **Hologram**, **Labels**, **Realign...** and **Forget...**. The
status line at the bottom shows the replay in progress. **Play closes the menu** so it is out of your way;
press Y (or Menu) to bring it back, and A (or the wrist bar's STOP) stops the replay.

### Alignment is locked once saved

After aligning, nothing you do by accident can move the hologram: trigger, pinch, grip and sticks are
ignored and the panel says "Alignment is locked". To change it:

1. Menu > **Realign...**, then tap it **again within 4 s** to confirm.
2. You are now editing. The old alignment is kept, untouched, in the background.
3. **Done** (A, or the wrist bar) saves the new alignment and locks it again.
   **Cancel** (X, or the wrist bar) puts the hologram back exactly where the saved alignment was.

**Forget...** also needs a second tap. The first-ever alignment needs no unlock: A, or the wrist bar's
**Align**, starts it directly.

### Aligning (first time, or after Realign)

The twin is drawn from the robot's live joint angles, so its elbow-flex motor sits exactly where the real
one does relative to the base. Put the hologram's motor on the real motor and the whole arm is aligned.
(To use another motor, change `ALIGN_MOTOR` in web/index.html; the comment above it lists the
shoulder-lift values.)

1. Stand behind the robot so you can see the real elbow-flex motor. First time only: pull the trigger
   (or pinch) near the robot to drop the hologram roughly in place.
2. Start aligning (see above). The hologram's elbow motor glows cyan and the arm turns see-through.
3. **Grab the glowing motor**: squeeze the grip with the controller on it, or pinch it. The hologram
   follows your hand 1:1. Move it onto the real motor and twist to turn it. Let go. Repeat as needed.
4. Fine-tune: controller = hold grip + sticks (right stick turns about the motor and raises/lowers, left
   stick slides). Hands = wrist bar **Tune**, then pinch-drag (right hand slides, left hand sideways turns,
   both at 1/3 scale).
5. **Done** to save, or **Cancel** to undo.

Controller-only alternative: while aligning, click the right thumbstick for the 2-corner method. Green
**1** and magenta **2** mark the two front corners of the base plate; put the blue dot on each real corner
and pull the trigger. The panel reports span error and height diff (under ~5 mm is good).

The saved alignment is a WebXR persistent anchor: the next AR session puts the hologram back by itself
(look at the robot for a moment). Only one anchor is ever kept, so the Quest's per-site limit is never hit.

### Motor labels during motion (Phase 2)

In AR every motor gets a floating label with its name and live angle. When a motor is engaged
(turning faster than 3 deg/s, or more than 1.5 deg between two readings):

- its servo **glows orange** on the hologram and its label grows,
- the label shows **Δ** (degrees moved since this motion started) and the current speed,
- an **orange arc** at the joint sweeps from where the motion started to where the joint is now,
- when it stops, the label keeps "moved +X" for 3 seconds, then shrinks back.

The help panel also shows the replay in progress (file, n/N, time). Labels: **left thumbstick click**
or wrist bar **Labels** cycles *all -> moving only -> off*. Tip: hologram off (B) plus labels gives pure
annotations floating on the real arm.

Speeds use the server's timestamp of each motor reading (`t` in `/api/state`), so Wi-Fi or frame-rate
hiccups do not distort them.

## Safety notes

- The server binds to 127.0.0.1 only. `--host 0.0.0.0` lets anyone on the network move the arm.
- Ctrl+C disconnects, which releases torque (LeRobot default): support the arm first.
- If the browser tab closes, the arm just holds its last target; nothing keeps moving.
