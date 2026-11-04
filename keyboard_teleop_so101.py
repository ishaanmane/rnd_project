"""
Terminal-based keyboard teleop for SO-101 — no X11 / pynput required.
Reads raw keypresses from stdin (works fine over plain SSH).

Run:
    python keyboard_teleop_so101.py --port /dev/ttyACM0 --id asherbot

Controls:
    q/a : shoulder_pan   +/-
    w/s : shoulder_lift  +/-
    e/d : elbow_flex     +/-
    r/f : wrist_flex     +/-
    t/g : wrist_roll     +/-
    y/h : gripper        +/-
    x   : quit
"""

import argparse
import sys
import termios
import tty

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

STEP_DEG = 2.0  # degrees per keypress; tune to taste

KEY_MAP = {
    "q": ("shoulder_pan.pos", +1),
    "a": ("shoulder_pan.pos", -1),
    "w": ("shoulder_lift.pos", +1),
    "s": ("shoulder_lift.pos", -1),
    "e": ("elbow_flex.pos", +1),
    "d": ("elbow_flex.pos", -1),
    "r": ("wrist_flex.pos", +1),
    "f": ("wrist_flex.pos", -1),
    "t": ("wrist_roll.pos", +1),
    "g": ("wrist_roll.pos", -1),
    "y": ("gripper.pos", +1),
    "h": ("gripper.pos", -1),
}


def read_key():
    """Read a single keypress from stdin without waiting for Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="e.g. /dev/ttyACM0")
    parser.add_argument("--id", required=True, help="calibration id, e.g. asherbot")
    args = parser.parse_args()

    config = SO101FollowerConfig(port=args.port, id=args.id)
    robot = SO101Follower(config)
    robot.connect()

    print("Connected. Use q/a w/s e/d r/f t/g y/h to move joints, x to quit.")

    try:
        # Start from the robot's current position, not zero.
        obs = robot.get_observation()
        pos = {k: v for k, v in obs.items() if k.endswith(".pos")}

        while True:
            key = read_key()
            if key == "x":
                print("\nExiting.")
                break

            mapping = KEY_MAP.get(key)
            if mapping is None:
                continue

            joint, direction = mapping
            if joint not in pos:
                print(f"\nWarning: '{joint}' not in robot's motors: {list(pos.keys())}")
                continue

            pos[joint] += direction * STEP_DEG
            robot.send_action({joint: pos[joint]})
            print(f"\r{joint} -> {pos[joint]:.1f}    ", end="", flush=True)

    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()