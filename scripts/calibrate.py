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

4. If --gripper is set, after arm offsets are detected you will be prompted to
   fully open the gripper (Enter), then fully close it (Enter). Those angles
   define the [0, 1] mapping used at runtime.

Re-calibrate gripper range only (keep existing arm offsets):

     python scripts/calibrate.py --port /dev/ttyUSB0 \
       --gripper-only --calib configs/panda_calib.json
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

# Allow running directly: add the project root (parent of this scripts/ dir).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from gello_leader.calibration import (  # noqa: E402
    LeaderCalibration,
    calibrate_gripper_range_interactive,
)
from gello_leader.dynamixel_driver import DynamixelDriver  # noqa: E402


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="GELLO leader-arm calibration")
    p.add_argument("--port", default=None, help="Serial port, e.g. COM5 or /dev/ttyUSB0")
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
    p.add_argument(
        "--gripper-id",
        type=int,
        default=None,
        help="Gripper servo ID (default: last arm id + 1, usually 8).",
    )
    p.add_argument("--out", default=None, help="Path to write calibration JSON.")
    p.add_argument(
        "--gripper-only",
        action="store_true",
        help="Only re-run interactive gripper range; load/update --calib JSON.",
    )
    p.add_argument(
        "--calib",
        default=None,
        help="Existing calibration JSON (required with --gripper-only).",
    )
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


def run_gripper_calibration(
    driver: DynamixelDriver,
    gripper_servo_id: int,
) -> tuple[int, float, float]:
    """Interactive open/close range; returns (servo_id, open_deg, close_deg)."""
    open_deg, close_deg = calibrate_gripper_range_interactive(driver.get_joints)
    return (gripper_servo_id, open_deg, close_deg)


def main() -> None:
    args = parse_args()

    if args.gripper_only:
        if not args.calib:
            raise SystemExit("--gripper-only 需要同时指定 --calib <已有标定.json>")
        calib = LeaderCalibration.load(args.calib)
        port = args.port or calib.port
        baudrate = calib.baudrate
        gripper_id = (
            int(args.gripper_id)
            if args.gripper_id is not None
            else (int(calib.gripper[0]) if calib.gripper else calib.num_arm_joints + 1)
        )
        all_ids = list(calib.joint_ids) + [gripper_id]
        print(f"Connecting to {port} @ {baudrate} (IDs {all_ids}) ...")
        driver = DynamixelDriver(all_ids, port=port, baudrate=baudrate)
        gripper = run_gripper_calibration(driver, gripper_id)
        driver.close()
        calib = LeaderCalibration(
            port=port,
            joint_ids=calib.joint_ids,
            joint_offsets=calib.joint_offsets,
            joint_signs=calib.joint_signs,
            gripper=gripper,
            baudrate=baudrate,
        )
        calib.validate_gripper_range()
        out_path = Path(args.out) if args.out else Path(args.calib)
        calib.save(out_path)
        print(f"\nUpdated gripper range in: {out_path.resolve()}")
        print(f"gripper: id={gripper[0]}  open={gripper[1]:.3f} deg  close={gripper[2]:.3f} deg")
        return

    if not args.port:
        raise SystemExit("请指定 --port（完整标定）或使用 --gripper-only --calib ...")

    start_joints = np.array(args.start_joints, dtype=float)
    joint_signs = np.array(args.joint_signs, dtype=int)
    assert len(start_joints) == len(joint_signs), "start-joints / joint-signs mismatch"

    n_arm = len(start_joints)
    joint_ids = list(range(1, n_arm + 1))
    gripper_id = args.gripper_id if args.gripper_id is not None else n_arm + 1
    all_ids = joint_ids + ([gripper_id] if args.gripper else [])

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
        gripper = run_gripper_calibration(driver, gripper_id)
        print(f"gripper open  (deg)  : {gripper[1]:.3f}")
        print(f"gripper close (deg)  : {gripper[2]:.3f}")

    driver.close()

    calib = LeaderCalibration(
        port=args.port,
        joint_ids=joint_ids,
        joint_offsets=[round(float(x), 5) for x in offsets],
        joint_signs=[int(x) for x in joint_signs],
        gripper=gripper,
        baudrate=args.baudrate,
    )
    calib.validate_gripper_range()

    if args.out:
        out_path = Path(args.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        calib.save(out_path)
        print(f"\nSaved calibration to: {out_path.resolve()}")
    else:
        print("\n(no --out given; calibration not saved)")


if __name__ == "__main__":
    main()
