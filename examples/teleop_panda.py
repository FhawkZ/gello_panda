"""Example teleop loop: read leader -> process -> send to your Franka Panda.

This is the integration skeleton. The leader-reading and calibration parts are
fully implemented; the `PandaInterface` is a stub you replace with your own
robot API (libfranka / franka_ros2 / Polymetis / a network bridge / ...).

Mapping reminder (what GELLO sends to a Panda):
  * arm:     7 joint angles in radians, same order as Franka's getJointPositions
  * gripper: a value in [0, 1]; 0 = open, 1 = closed.
             GELLO sets width = MAX_OPEN * (1 - gripper), with MAX_OPEN = 0.09 m.

The loop also rate-limits the per-step joint change (like GELLO's run_env) so a
large initial pose difference between leader and follower cannot command a jump.

    python examples/teleop_panda.py --calib configs/panda_calib.json
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from gello_leader import LeaderArm, LeaderCalibration  # noqa: E402

PANDA_MAX_OPEN_M = 0.09  # Franka Hand max width, matches GELLO's PandaRobot


class PandaInterface:
    """Replace the bodies below with calls into your own Panda stack."""

    def __init__(self) -> None:
        # TODO: connect to your robot here (libfranka / ROS2 / Polymetis / ...).
        pass

    def get_joint_positions(self) -> np.ndarray:
        """Return the robot's current 7 joint angles (rad)."""
        # TODO: query the real robot. Placeholder returns the Panda home-ish pose.
        return np.array([0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0])

    def command_joint_positions(self, q_arm: np.ndarray) -> None:
        """Send 7 desired joint angles (rad) to the robot."""
        # TODO: e.g. robot.update_desired_joint_positions(q_arm)
        pass

    def command_gripper(self, gripper_01: float) -> None:
        """Send a normalised gripper command (0 = open, 1 = closed)."""
        width = PANDA_MAX_OPEN_M * (1.0 - float(gripper_01))
        # TODO: e.g. gripper.goto(width=width, speed=..., force=...)
        _ = width
        pass


def main() -> None:
    p = argparse.ArgumentParser(description="Leader -> Panda teleop skeleton")
    p.add_argument("--calib", required=True, help="Path to calibration JSON")
    p.add_argument("--hz", type=float, default=100.0)
    p.add_argument("--alpha", type=float, default=1.0, help="Leader smoothing (1=off)")
    p.add_argument(
        "--max-delta",
        type=float,
        default=0.05,
        help="Max per-step change per joint (rad), like GELLO run_env.",
    )
    p.add_argument(
        "--max-start-delta",
        type=float,
        default=0.8,
        help="Refuse to start if leader/follower differ more than this (rad).",
    )
    args = p.parse_args()

    calib = LeaderCalibration.load(args.calib)
    period = 1.0 / args.hz
    robot = PandaInterface()

    with LeaderArm(calib, alpha=args.alpha) as leader:
        # --- safety: check the leader/follower start gap before moving ---
        arm, gripper = leader.get_arm_and_gripper()
        current = robot.get_joint_positions()
        if arm.shape != current.shape:
            raise SystemExit(
                f"DoF mismatch: leader arm {arm.shape} vs robot {current.shape}"
            )
        gap = np.abs(arm - current)
        if gap.max() > args.max_start_delta:
            worst = int(np.argmax(gap))
            raise SystemExit(
                f"Leader/follower too far apart at joint {worst} "
                f"(delta {gap[worst]:.3f} > {args.max_start_delta}). "
                "Move the GELLO to match the robot before starting."
            )

        print("Teleop running. Ctrl+C to stop.")
        try:
            while True:
                arm, gripper = leader.get_arm_and_gripper()
                current = robot.get_joint_positions()

                delta = arm - current
                biggest = np.abs(delta).max()
                if biggest > args.max_delta:
                    delta = delta / biggest * args.max_delta
                robot.command_joint_positions(current + delta)

                if gripper is not None:
                    robot.command_gripper(gripper)

                time.sleep(period)
        except KeyboardInterrupt:
            print("\nstopped.")


if __name__ == "__main__":
    main()
