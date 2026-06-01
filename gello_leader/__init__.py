"""Minimal GELLO leader-arm reader.

This package contains only the parts needed to:
  1. read raw joint angles from the Dynamixel servos of a GELLO leader arm,
  2. apply calibration (offsets / signs / gripper range) to map them into the
     follower (e.g. Franka Panda) joint space,
  3. hand the processed joint vector off to your own robot interface.

It is intentionally self-contained and does NOT depend on the full
`gello_software` package, ROS, the follower robot drivers, or gravity
compensation. The only third-party requirements are `numpy` and
`dynamixel-sdk`.
"""

from .calibration import LeaderCalibration
from .dynamixel_driver import DynamixelDriver
from .leader_arm import LeaderArm

__all__ = ["DynamixelDriver", "LeaderArm", "LeaderCalibration"]
