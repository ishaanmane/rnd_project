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
- **Record** writes the same JSON-lines format as `teleop_and_record.py`, so `replay_motion.py`
  and `block_trigger_watcher.py` work on it unchanged. Paths are relative to where you ran the server.

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

## Safety notes

- The server binds to 127.0.0.1 only. `--host 0.0.0.0` lets anyone on the network move the arm.
- Ctrl+C disconnects, which releases torque (LeRobot default): support the arm first.
- If the browser tab closes, the arm just holds its last target; nothing keeps moving.
