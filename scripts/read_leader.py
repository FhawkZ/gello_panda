"""Continuously print the processed leader-arm joint state.

Use this to verify a calibration: in the calibration pose the arm joints
should read close to your --start-joints (e.g. 0 0 0 -1.57 0 1.57 0 for Panda),
and the gripper should move within [0, 1].

    python scripts/read_leader.py --calib configs/panda_calib.json
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from gello_leader import LeaderArm, LeaderCalibration  # noqa: E402


def main() -> None:
    p = argparse.ArgumentParser(description="Print processed leader joint state")
    p.add_argument("--calib", required=True, help="Path to calibration JSON")
    p.add_argument("--hz", type=float, default=20.0)
    p.add_argument("--alpha", type=float, default=1.0, help="Smoothing (1.0 = off)")
    args = p.parse_args()

    calib = LeaderCalibration.load(args.calib)
    period = 1.0 / args.hz

    with LeaderArm(calib, alpha=args.alpha) as leader:
        print(f"Reading {leader.num_dofs} DoF. Ctrl+C to stop.\n")
        try:
            while True:
                arm, gripper = leader.get_arm_and_gripper()
                arm_str = " ".join(f"{x:+.3f}" for x in arm)
                if gripper is None:
                    print(f"arm: [{arm_str}]", end="\r")
                else:
                    print(f"arm: [{arm_str}]  grip: {gripper:.2f}", end="\r")
                time.sleep(period)
        except KeyboardInterrupt:
            print("\nstopped.")


if __name__ == "__main__":
    main()
