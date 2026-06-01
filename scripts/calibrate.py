"""Offset calibration for a GELLO leader arm (Panda layout by default).

Port of `gello_software/scripts/gello_get_offset.py`, trimmed to the leader
side and able to write a portable JSON calibration file.

Procedure
---------
1. Set unique IDs on every Dynamixel (use Dynamixel Wizard).
2. Physically place the GELLO in the KNOWN calibration pose. For a Franka
   Panda the recommended pose is:  0 0 0 -1.57 0 1.57 0  (radians).
3. Run this script while holding that pose:

   Windows:
     python scripts/calibrate.py --port COM5 ^
       --start-joints 0 0 0 -1.57 0 1.57 0 ^
       --joint-signs 1 1 1 1 1 -1 1 ^
       --gripper --out configs/panda_calib.json

   Linux:
     python scripts/calibrate.py --port /dev/ttyUSB0 \
       --start-joints 0 0 0 -1.57 0 1.57 0 \
       --joint-signs 1 1 1 1 1 -1 1 \
       --gripper --out configs/panda_calib.json

The detected offsets (and gripper open/close angles) are printed and, if
--out is given, saved to JSON for use by `LeaderCalibration.load(...)`.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

# Allow running directly: add the project root (parent of this scripts/ dir).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from gello_leader.calibration import LeaderCalibration  # noqa: E402
from gello_leader.dynamixel_driver import DynamixelDriver  # noqa: E402


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="GELLO leader-arm offset calibration")
    p.add_argument("--port", required=True, help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    p.add_argument("--baudrate", type=int, default=57600)
    p.add_argument(
        "--start-joints",
        type=float,
        nargs="+",
        default=[0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0],
        help="Known arm pose in radians (Panda default).",
    )
    p.add_argument(
        "--joint-signs",
        type=int,
        nargs="+",
        default=[1, 1, 1, 1, 1, -1, 1],
        help="Per-joint sign (+1/-1). Panda default: 1 1 1 1 1 -1 1.",
    )
    p.add_argument("--gripper", action="store_true", help="A gripper servo is attached.")
    p.add_argument("--no-gripper", dest="gripper", action="store_false")
    p.set_defaults(gripper=True)
    p.add_argument("--out", default=None, help="Path to write calibration JSON.")
    return p.parse_args()


def detect_offsets(
    driver: DynamixelDriver,
    start_joints: np.ndarray,
    joint_signs: np.ndarray,
) -> np.ndarray:
    """Brute-force the per-joint offset (multiple of pi/2) closest to the pose."""
    for _ in range(10):  # warm up the reader thread
        driver.get_joints()

    curr = driver.get_joints()
    n = len(start_joints)
    best_offsets = np.zeros(n)
    candidates = np.linspace(-8 * np.pi, 8 * np.pi, 8 * 4 + 1)  # pi/2 steps
    for i in range(n):
        best_err = np.inf
        for off in candidates:
            q = joint_signs[i] * (curr[i] - off)
            err = abs(q - start_joints[i])
            if err < best_err:
                best_err = err
                best_offsets[i] = off
    return best_offsets


def main() -> None:
    args = parse_args()
    start_joints = np.array(args.start_joints, dtype=float)
    joint_signs = np.array(args.joint_signs, dtype=int)
    assert len(start_joints) == len(joint_signs), "start-joints / joint-signs mismatch"

    n_arm = len(start_joints)
    joint_ids = list(range(1, n_arm + 1))
    all_ids = joint_ids + ([n_arm + 1] if args.gripper else [])

    print(f"Connecting to {args.port} @ {args.baudrate} (IDs {all_ids}) ...")
    driver = DynamixelDriver(all_ids, port=args.port, baudrate=args.baudrate)

    print("Hold the GELLO in the calibration pose, detecting offsets...")
    offsets = detect_offsets(driver, start_joints, joint_signs)

    print()
    print("joint_offsets        :", [f"{x:.5f}" for x in offsets])
    print(
        "joint_offsets (pi/2) :",
        [f"{int(round(x / (np.pi / 2)))}*pi/2" for x in offsets],
    )

    gripper = None
    if args.gripper:
        grip_deg = float(np.rad2deg(driver.get_joints()[-1]))
        gripper_open = grip_deg - 0.2
        gripper_close = grip_deg - 42.0
        print(f"gripper open  (deg)  : {gripper_open:.3f}")
        print(f"gripper close (deg)  : {gripper_close:.3f}")
        gripper = (all_ids[-1], gripper_open, gripper_close)

    driver.close()

    calib = LeaderCalibration(
        port=args.port,
        joint_ids=joint_ids,
        joint_offsets=[round(float(x), 5) for x in offsets],
        joint_signs=[int(x) for x in joint_signs],
        gripper=gripper,
        baudrate=args.baudrate,
    )

    if args.out:
        out_path = Path(args.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        calib.save(out_path)
        print(f"\nSaved calibration to: {out_path.resolve()}")
    else:
        print("\n(no --out given; calibration not saved)")


if __name__ == "__main__":
    main()
