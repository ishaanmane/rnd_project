"""
Replay a recorded SO-101 motion (JSON-lines file from teleop_and_record.py).

Run:
    python replay_motion.py --port /dev/ttyACM0 --id asherbot --file my_motion.jsonl
"""

import argparse
import json
import time

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

EASE_IN_STEPS = 30
EASE_IN_STEP_DELAY = 0.02  # ~0.6s total ease-in


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--id", required=True)
    parser.add_argument("--file", required=True)
    args = parser.parse_args()

    with open(args.file) as f:
        entries = [json.loads(line) for line in f if line.strip()]

    if not entries:
        print("No recorded frames found.")
        return

    config = SO101FollowerConfig(port=args.port, id=args.id)
    robot = SO101Follower(config)
    robot.connect()

    try:
        # Ease from wherever the arm currently is into the recording's first
        # frame, instead of snapping straight there.
        obs = robot.get_observation()
        current = {k: v for k, v in obs.items() if k.endswith(".pos")}
        first_action = entries[0]["action"]

        for i in range(1, EASE_IN_STEPS + 1):
            alpha = i / EASE_IN_STEPS
            interp = {
                joint: current[joint] + alpha * (target - current[joint])
                for joint, target in first_action.items()
                if joint in current
            }
            robot.send_action(interp)
            time.sleep(EASE_IN_STEP_DELAY)

        print(f"Replaying {len(entries)} frames from {args.file}")

        t_start = time.perf_counter()
        for entry in entries:
            target_t = entry["t"]
            now = time.perf_counter() - t_start
            sleep_time = target_t - now
            if sleep_time > 0:
                time.sleep(sleep_time)
            robot.send_action(entry["action"])

        print("Replay complete.")

    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()