#!/usr/bin/env python3
"""Bypass draccus CLI parsing (broken choice-decoding) and call the teleop loop directly."""

import time

from lerobot.configs.types import FeatureType  # noqa: F401 (harmless import safety)
from lerobot.robots import RobotConfig
from lerobot.robots.so_follower import SOFollowerConfig  # noqa: F401  (registers so101_follower)
from lerobot.utils.robot_utils import precise_sleep

from examples.isaac_teleop_to_so101.common import FPS, HoldLatch, build_device
from examples.isaac_teleop_to_so101.isaac_teleop import XRControllerConfig
from examples.isaac_teleop_to_so101.teleoperate import TeleoperateConfig

# --- EDIT THESE TO MATCH YOUR SETUP ---
ROBOT_PORT = "/dev/ttyACM0"
ROBOT_ID = "asherbot"
# ----------------------------------------

robot_config_cls = RobotConfig.get_choice_class("so101_follower")
robot_cfg = robot_config_cls(port=ROBOT_PORT, id=ROBOT_ID)

teleop_cfg = XRControllerConfig()

cfg = TeleoperateConfig(teleop=teleop_cfg, robot=robot_cfg)

robot, device, motor_names = build_device(cfg)
hold = HoldLatch(motor_names)
try:
    while True:
        t0 = time.perf_counter()
        obs = robot.get_observation()
        action = hold.resolve(device.compute(obs), obs)
        robot.send_action(action)
        precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))
except KeyboardInterrupt:
    pass
finally:
    try:
        device.cleanup()
    finally:
        robot.disconnect()