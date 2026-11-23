# CLAUDE.md: SO-101 digital twin, AR and sorting demo

Handover from the previous Claude session (Nov 2026). Read this first. The README.md files
(`./README.md`, `./detector/README.md`) are the user manuals. This file is the engineering context.

## Working with the user

- Name: Niraj (MTech IEOR, IIT Bombay). In the previous session he asked to be called "Gami Sama".
  Ask him if he wants that again.
- Reply in English, in short simple sentences (one idea each). Technical terms are welcome.
- He wants honest feasibility assessments. Say plainly what is verified on hardware and what is not.
  Never claim something works on the real robot or the Quest unless he confirmed it.
- Prose he will paste anywhere (abstracts, poster text): no semicolons.
- No emoji anywhere in the AR UI.
- Explain the "why" behind design choices. He wants to understand, not just receive code.
- Previous session ran in a cloud sandbox and shipped zips of changed files. Here you work in the
  repo directly: edit in place. Do not commit unless he asks.

## Machine and environments

- Ubuntu PC, NVIDIA RTX 5060 Ti 16 GB, driver 595 (CUDA 13.2). Miniforge conda.
- Project: `~/ishaan_rnd/so101_twin` (GitHub repo `ishaanmane/rnd_project`, folder `so101_twin`).
- Two conda envs, kept separate on purpose:
  - `lerobot` (Python 3.12, lerobot 0.6.1 editable from `~/asherbot/lerobot`, numpy 2.2.6,
    torch 2.11 cu130, huggingface_hub 1.26). Runs `twin_server.py`.
    It has BOTH opencv-python 5.0 and opencv-python-headless 4.13 installed (same `cv2`). Works today, a known suspect if cv2 acts up.
  - `locany` (Python 3.10, transformers 4.57.1, numpy 1.25.0, opencv-python-headless 4.11.0.86,
    torch cu128). Runs the detector. Made by `detector/setup_env.sh`.
    Never install the detector pins into `lerobot`: numpy 1.25 / hub < 1.0 would break LeRobot.
  - The two only talk over HTTP (8001 <-> 8002), so `twin_server.py` stays standard-library only.
- Robot: SO-101 follower ("asherbot"), Feetech STS3215 servos on `/dev/ttyACM0`.
- Headset: Meta Quest 3, wired via `adb reverse` (IIT Bombay network, no Wi-Fi setup).

## How to run

```bash
# Terminal 1: robot + web UI (lerobot env)
cd ~/ishaan_rnd/so101_twin
python twin_server.py --port /dev/ttyACM0 --id asherbot --http-port 8001
#   --sim (no hardware)  --detector URL (default http://127.0.0.1:8002)  --keep-scans  --max-speed N
# Terminal 2: block detector (locany env)
cd ~/ishaan_rnd/so101_twin/detector && python locate_service.py          # test page http://127.0.0.1:8002
#   --backend color (no GPU fallback)  --expect green:3,blue:2,red:1  --greedy  --revision latest
# Terminal 3: Quest over USB
./quest_connect.sh 8001        # adb reverse tcp:8001; then open http://localhost:8001 on the Quest
```

PC UI: http://127.0.0.1:8001 . Teach page: http://127.0.0.1:8001/teach .
For demo day: `export HF_HUB_OFFLINE=1` before starting the detector (model is cached).

## What exists (all working unless the status section says otherwise)

1. Browser digital twin (three.js 0.160 + urdf-loader 0.13.1, vendored in `web/vendor`).
   Drag the orange ghost to command joints, sliders, STOP, start pose, record/replay
   (`recorded_motions/*.jsonl`), speed cap.
2. AR on Quest 3 (WebXR immersive-ar, local-floor). Hologram aligned to the real arm (elbow-flex
   grab method or 2-point corner method), locked, saved with persistent anchors. Live motor labels
   (Phase 2: which motor moves and by how much). Hands and controllers both supported.
3. Block detector (`detector/`): A4 sheet with 4 ArUco markers, homography from the markers
   (no camera calibration), NVIDIA LocateAnything-3B, one query per colour, cell 1..9.
4. Sorting demo: teach 18 poses by hand, planner, pick/place sequencer, twin shows sheet, blocks,
   plan arrows and reasons. Works on the real robot.
5. AR game-style menu: Homepage (Sort / Settings), Sort screen (Take picture, Run, STOP, Arm Home,
   planner row Greedy / BFS / Dijkstra),
   overlay on the real grid, draggable menus, take-picture review flow.

## Files

| File | Role |
|---|---|
| `twin_server.py` | Stdlib HTTP server + 30 Hz control loop (the only thread touching the serial bus). Teach mode, poses, sequencer, board, scan forwarding, record/replay |
| `sorting.py` | Planner (stdlib): Greedy, BFS, A*, Dijkstra (robot time, `TimeModel`). `python sorting.py test` checks all 5040 layouts (~4 min, Dijkstra too when taught_poses.json loads) |
| `sort_runs.csv` | Written by the server: one line per finished Sort (algorithm, board, moves, predicted vs measured seconds, speed). Data for the report |
| `web/index.html` | Twin page + all AR code (big single module script) |
| `web/board.js` | Twin/AR sorting view: sheet, blocks, outlines, rings, arrows, banner, grid fit, PC "Scan photo" |
| `web/teach.html` | Teach page (Free arm / Lock and save, Test cell, gripper values; payload poses + payload gripper/container) |
| `web/payload.js` | 2D page Payload panel (weight/speed, Run, RELEASE, hold, per-motor table) |
| `payload_model.py` | Stdlib static gravity-torque model from the URDF (masses, COMs, tool point) -> % of 30 kg·cm stall per joint |
| `payload_poses.json` | Payload test poses (hover, grab, lift, left, right), payload gripper open/grip, container mass. User's own data. Separate from taught_poses.json |
| `payload_runs/` | One CSV per payload run (every control tick: target, actual, load, current, temp, status, model) + `summary.csv` |
| `web/questcam.js` | Camera capture for Take picture (camera choice, black-frame check) |
| `twin_calib.json` | Visual mapping LeRobot deg -> URDF deg (`urdf = sign*lerobot + offset`). User's own values, do not overwrite |
| `taught_poses.json` | Taught Home + 9 x (hover, grab) poses + gripper values. User's own data, do not overwrite |
| `quest_connect.sh` | adb checks + `adb reverse` |
| `urdf/so101.urdf` | Arm model (so101_new_calib content). `gripper_frame_link` = fingertip frame used for FK |
| `detector/locate_service.py` | Detector HTTP service (8002) + CLI (`--image photo.jpg`) |
| `detector/vision.py`, `sheet_layout.py`, `backends.py` | Markers/homography/cells, sheet geometry, LocateAnything + HSV colour backend |
| `detector/make_grid_sheet.py`, `grid_sheet_60mm.pdf` | Printable sheet (60 mm cells in use; 55/50 mm variants) |
| `detector/selftest.py` | Synthetic 3D photos through the whole pipeline (`--backend locate` for the model) |

## twin_server.py: key facts

- Units: LeRobot degrees mode for joints, gripper 0..100 %. Joints: shoulder_pan, shoulder_lift,
  elbow_flex, wrist_flex, wrist_roll, gripper. `ARM` = first five.
- Loop: `cmd` chases `target` at <= speed (deg/s, gripper %/s), `SPEED_CEILING` 90, default 30.
- Torque jobs (`self.requests`) are run by the loop thread only. LeRobot 0.6.1 API used:
  `robot.bus.disable_torque(ARM)`, `robot.bus.enable_torque(ARM)`, `robot.bus.read("Torque_Enable", motor)`.
- Teach mode: arm torque off, gripper held at teach value (19.5 %).
  IMPORTANT: while the arm is free, the loop sends a goal to the GRIPPER ONLY. Writing Goal_Position
  to a torque-off STS3215 switched its torque back on (arm felt stiff). Lock = set goal = present
  position, send, then enable torque (no jump). A 1 s watchdog re-reads Torque_Enable and fixes it.
- Sequencer (`self.program`): steps arm / grip / event. Arm moves are synchronised straight lines in
  joint space. A missed pose NEVER aborts (jittery servos, user said no collisions in this setup):
  wait until settled (`STILL_*`) or `ARRIVE_WAIT` 1 s, then carry on, report worst error at the end.
- Move = event move, hover src, grab src, grip 15 %, event pick, hover src, hover dst, grab dst,
  open 19.5 %, event place, hover dst. A run opens the gripper first and ends at Home, gripper 1 %.
- `DEFAULT_HOME` came from the user's screenshot (pan 1.2, lift -1.3, elbow 0.3, wrist_flex -62.6,
  roll 18.3). It can be re-taught on the teach page.
- Planner choice (added 23 Nov 2026, user's request): Greedy (simple), BFS, Dijkstra. The server
  keeps it (`Controller.sort_alg`, default `SORT_ALGORITHM = "greedy_simple"`, list in
  `DEMO_ALGORITHMS`), so scan, Plan and Run all use the same one and both pages agree.
  `POST /api/sort/algorithm {"algorithm"}` sets it and re-plans the board at once (409 while the robot
  moves). 2D page: "Planner" dropdown. AR Sort screen: Greedy / BFS / Dijkstra toggle row (lit = in use).
  `make_plan` refuses while a program runs (it used to overwrite the running plan's progress).
- Dijkstra minimises robot SECONDS, not moves: node = (board, arm position 0=Home or 1..9 hover),
  edge = time of one block move from the taught poses at the current speed cap (`sorting.TimeModel`
  mirrors the sequencer: straight joint-space lines at the cap, gripper change / speed + GRIP_DWELL,
  Home to Home; settle waits not counted). ~0.2-0.3 s per plan. Every plan now has
  `summary.seconds` (predicted); a finished Sort shows "Took X s (predicted Y s)" and is logged to
  `sort_runs.csv`. Dijkstra needs all 9 cells taught (clear error otherwise).
- Scan: `POST /api/scan` (raw JPEG/PNG body, `?source=`) -> detector `/api/detect?colors=green,yellow,blue,red`
  -> must be 3 + 2 + 1 with green as the 3 -> sets board and plans. Photo files are deleted
  (detector `/api/forget`) when the sort finishes, a newer scan replaces it, or the board is edited
  by hand, unless `--keep-scans`.
- Endpoints: GET `/api/state /api/config /api/recordings /api/poses /teach /urdf/*`.
  POST `/api/target /api/speed /api/stop /api/go_start /api/go_home /api/record/start /api/record/stop
  /api/replay/start /api/replay/stop /api/twin_calib /api/teach/{start,stop,free,lock,goto,test_cell}
  /api/poses/gripper /api/board /api/sort/{algorithm,plan,run,move} /api/scan /api/sim/hand` (sim only).
  `/api/target` returns 409 with a reason while a replay/program runs or the arm is free.

## Payload experiment (added 23 Nov 2026)
- Hardware: follower on 12 V STS3215 (30 kg·cm stall). Payload = 3D printed 5x5x2.5 cm container in cell 2
  with hammer heads (100 g x1, 200 g x2): runs at 0/100/200/300/400/500 g x 10/20/30 deg/s.
- Sequence (`run_payload`): Home -> open -> hover -> grab -> grip -> hover -> lift -> HOLD 5 s -> left -> right
  -> lift -> hover -> grab -> open -> hover -> Home. Speed mode applies to all motors for the run, then the old
  speed comes back (`/api/speed` returns 409 during the run). left/right: ONLY shoulder_pan from the taught pose,
  other joints copied from lift (`payload_pose`), so the dance moves only the base.
- HOLD PASS: every arm joint within HOLD_TOL = 5 deg of lift, no arm servo Status overload/overcurrent bit, no
  slip (gripper closes > SLIP_PCT = 3 % further than where the container stopped it). FAIL -> dance skipped,
  container put back. HOLD_TOL is a guess to tune after the first real runs (sorting runs already showed 3.3 deg).
- RELEASE (`/api/payload/release`, big red button on both pages, R key on the 2D page): program cleared, arm
  target = cmd (frozen), gripper cmd AND target set to payload open (no speed ramp: jaw opens next tick).
- Telemetry (real robot only, `--no-telemetry` to disable): one `bus.sync_read(reg, normalize=False)` per tick,
  cycle Load, Current, Status, Load, Current, Temp/Voltage. Load = |raw|/10 % (LeRobot decodes the sign, bit 10),
  current = raw*6.5 mA, voltage raw/10. 30 failed reads in a row -> telemetry switches itself off.
  NOT yet run on the real servos: check the numbers react by pushing the locked arm by hand.
- Display: `state.loads` = real readings if telemetry, else the model (`source`). Servo colours (index.html
  `paintLoad`/`paintServo`): green < 50, orange < 80, red, flashing = Status flag or >= 100 %. All servo
  recolouring goes through `paintServo` and restores `mesh.userData.origMat` (align glow, moving glow, load).
- Model (verified against an independent numpy version, 1e-14): static only, URDF masses (632 g total), payload
  as a point at gripper_frame_link + container mass. shoulder_pan always ~0 (vertical axis): base load in the
  dance is dynamic only. Pre-check before Run: peak along the carried path; > 100 % needs a confirm press.
- LeRobot caps the GRIPPER at 50 % torque/current on every connect (so_follower.configure): the container may
  slip before an arm motor overloads.
- Logs: per-run CSV + `summary.csv` (result, hold error/joint, slip, flags, peak/hold-mean load per joint,
  model at lift, peak current, dance peak on shoulder_pan, max temp).

## Sorting demo rules

- Blocks: 3 green, 2 yellow, 2 blue, 2 red available. Each run: 3 green (A) + 2 of one colour (B) +
  1 of another (C). Green is always A (`MAIN_COLOR`), a board without 3 green is refused.
- Goal: A -> cells 1, 4, 7. B -> 2, 8. C -> 6. Cells 3, 5, 9 end empty (parking).
- Cell numbering: page upright seen from behind the robot, "ROBOT SIDE" edge at the robot.
  1-2-3 far row, 7-8-9 near row.
- Theory (verified on all 5040 layouts): fewest moves = misplaced + loops. Greedy simple: 0-2 extra
  moves on 991 layouts, max plan length 9. BFS/A* always optimal in moves.
- Robot time (model, taught poses of 20 Nov, 30 deg/s, all 5040 layouts): Greedy mean 52.4 s (worst
  84.6), BFS 50.9 s (72.5), Dijkstra 49.2 s (70.9). Dijkstra beats Greedy on 4019 layouts (mean 3.2 s,
  5.5 %, best 21.2 s) and BFS on 3633 (mean 1.7 s, best 11.2 s). On 75 layouts the fastest plan uses
  one move MORE than the minimum. Longest Dijkstra plan: 7 moves. Not yet measured on the robot.
- Gripper: contact between 15.5 and 18.5 %. User chose grip 15 %, open/teach 19.5 % (tight around a
  25 mm cube: raise "Open" if jaws push blocks, no re-teach needed).

## Detector facts

- Model `nvidia/LocateAnything-3B`, pinned `MODEL_REVISION = c32291ca5e996f5a7a485845b4f57a233936bba0`
  (pins weights AND remote code). 7.2 GB VRAM, loads in ~10 s, ~0.55 s per colour query.
- Prompt: "Locate all the instances that matches the following description: {color} cube."
  Output `<box><x1><y1><x2><y2></box>`, 0..1000. Tolerant parser in `backends.py`.
- Runs on the rectified top-down grid image (4 px/mm). Sheet layouts: 60 mm cells = marker IDs 0-3,
  55 mm = 4-7, 50 mm = 8-11 (DICT_4X4_50, 35 mm markers). The detector picks the layout from the IDs.
- Real results: printed sheet fit error 0.18 mm, 5/5 identical answers on repeat runs, all colours
  incl. black correct on a phone photo. Endpoints: `/api/detect`, `/api/forget`, `/api/health`, `/`.

## AR facts (web/index.html)

- Screens: `ar.screen` = 'home' | 'settings' | 'sort'. `homePanel`, `arMenu` (Settings), `sortPanel`.
  Homepage opens on the first AR frame (after anchor restore). Y toggles the current screen.
  Entering Sort turns the hologram off (restored on leaving unless the user toggled it).
- Menus are draggable by the bar under them (controller/hand ray + trigger/pinch, or pinch the bar).
  Position saved relative to the robot in localStorage `so101-twin-menu`.
- Overlay (Sort screen only, `board.js`): paper and virtual cubes hidden, coloured outline per
  detected block, orange ring = pick, green ring = place, arrows, banner past the far row showing
  "Move n of N", the reason, and the current step.
- Grid placement in the twin: FK of taught grab poses (invisible arm clone, `gripper_frame_link`),
  2D Kabsch fit to the 60 mm layout. User's real fit error: 2.8 mm.
- Take picture flow: capture -> large preview -> "Use this photo" uploads / "Click again" retakes.
- Controls: Y menu, B hologram, L3 labels, A = STOP while the robot moves (else Done/align), X help.
  Wrist bar for hands. Debug handle: `window.twinAR`, `window.twin`, `window.twin.sorting`.

## Status (as of 22 Nov 2026)

Confirmed by the user on real hardware:
- Twin control, record/replay, AR alignment and anchors, motor labels, AR menu.
- Teaching all 9 cells (after the torque fix), grid fit 2.8 mm, full sort on the real robot.
- Detector on phone photos and on a Quest Camera-app photo uploaded at localhost:8002.
- Menu dragging (bar visible and used in his screenshot).

Open problem being worked on: **Take picture in AR gives a black image.**
- Cause found: the browser only offered "camera 0, facing front", which is the Quest AVATAR camera
  (virtual, never sees the room). The passthrough cameras ("camera 1/2, facing back") are offered
  only with the Browser's **Headset cameras** permission. Quest's prompt has "Virtual cameras" ticked
  and "Headset cameras" unticked, and does not ask again.
- Fix sent (not yet confirmed by the user): lock icon left of the address bar on localhost:8001 >
  Headset cameras > Allow, or Settings > Privacy & safety > Browser > Headset cameras ON, then
  reload. Needs Quest 3/3S, Horizon OS v74+, Quest Browser 40.1+. Code now refuses the avatar
  camera and black frames with that message, reads frames via ImageCapture.grabFrame, keeps the
  <video> in the DOM (2 px, opacity 0.01) because Quest Browser stops painting the page in AR.
- First thing to ask: what does "Allow camera" on the 2D page say now, and what is the grey line
  under the result after Take picture (should be "camera 1, facing back · WxH · s")?
- Proven fallback: photo with the Quest Camera app, then on the 2D page (outside AR) "Scan photo…".

Not yet verified on the device: hand-pinch dragging of menus, the headset camera after the
permission change, motor labels possibly cluttering the Sort screen (left as is, not asked).

Possible next steps (only if he asks): closed-loop re-scan after a run, making the panel/overlay
cleaner for the demo, the deferred "QR/vision tag" alignment research idea.

## Testing without hardware (how the previous session verified things)

- `python twin_server.py --sim --http-port 80xx`. `/api/sim/hand` moves free joints in sim.
- Detector without GPU: `--backend color`. Synthetic photos: `detector/selftest.py` (`render()`).
- AR in a headless browser: Playwright + Chromium (`--use-gl=swiftshader --enable-unsafe-swiftshader`)
  + IWER (`npm i iwer@2`, inject `build/iwer.js`, `new IWER.XRDevice(IWER.metaQuest3).installRuntime({forceInstall:true})`).
  Very slow (well under 1 fps): wait on conditions, not fixed sleeps. Controllers:
  `__xrd.controllers.right.position.set(..)`, `.updateButtonValue('trigger', 1)`.
- Fake camera: `--use-fake-device-for-media-stream --use-fake-ui-for-media-stream
  --use-file-for-fake-video-capture=file.y4m` (make the y4m from a photo with ffmpeg).
- For a meaningful grid in sim, taught poses must be physically consistent (previous session made
  them with a small IK search on the twin and saved them via the teach API).
- Gotchas: `pkill -f` can kill its own shell (use a ps/awk filter), stale servers hold ports,
  hard-reload the browser (Ctrl+Shift+R) after changing JS.

## History in one paragraph

Built in one long session (Nov 2026): browser twin replacing the leader arm, then AR on Quest
(Phase 0 setup, Phase 1 alignment incl. hands and elbow-flex grab, anchor fix for "maximum number of
anchors", Phase 2 motor labels, AR menu with lock, tidy-up without emoji), then the LocateAnything
detector and A4 sheet, then the sorting demo (the original plan was vertical-rack stacking, dropped
for sorting), teach mode, the torque-stiffness fix, green colour rule, greedy-only planner,
never-abort arrival, AR game menu, draggable menus, take-picture flow and the headset-camera
diagnosis. A POMS 2027 abstract and an IEOR Golden Jubilee poster panel were written about it.
23 Nov 2026 (new session, report due 24 Nov): planner choice Greedy / BFS / Dijkstra (robot time) on
both pages, predicted vs measured sort time (first real run: Dijkstra 70.0 s predicted, 74.5 s measured).
Then the payload experiment (section above): teach page slots, server test + telemetry + RELEASE + logs,
2D Payload panel with load-coloured servos, AR "Payload Testing" screen. Tested in --sim and with a fake
servo bus (pass, sag fail, slip, overload flag, release, stop); not yet on the real robot or in the headset.
User wants to be called "Gami Sama".
