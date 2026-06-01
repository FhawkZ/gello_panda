"""Real-time monitor of raw Dynamixel joint readings (no calibration needed).

Connects directly to the servos and prints, for every joint, the live
position in radians and degrees (plus velocity in rad/s). This is meant for
debugging wiring / IDs and for watching values while you set up calibration --
it does NOT apply joint_offsets / joint_signs / gripper mapping.

Examples
--------
    # 7 arm joints + gripper (IDs 1..8), Windows port COM5
    python scripts/monitor_joints.py --port COM5 --num-joints 8

    # explicit IDs, Linux
    python scripts/monitor_joints.py --port /dev/ttyUSB0 --ids 1 2 3 4 5 6 7

    # also show velocities
    python scripts/monitor_joints.py --port COM5 --num-joints 8 --show-vel
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from gello_leader.dynamixel_driver import DynamixelDriver  # noqa: E402


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Live raw Dynamixel joint monitor")
    p.add_argument("--port", required=True, help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    p.add_argument("--baudrate", type=int, default=57600)
    g = p.add_mutually_exclusive_group()
    g.add_argument(
        "--num-joints",
        type=int,
        default=None,
        help="Number of servos; IDs assumed to be 1..N.",
    )
    g.add_argument(
        "--ids",
        type=int,
        nargs="+",
        default=None,
        help="Explicit servo IDs in bus order, e.g. --ids 1 2 3 4 5 6 7 8.",
    )
    p.add_argument("--hz", type=float, default=20.0, help="Print rate (Hz).")
    p.add_argument("--show-vel", action="store_true", help="Also print velocities.")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    if args.ids is not None:
        ids = list(args.ids)
    elif args.num_joints is not None:
        ids = list(range(1, args.num_joints + 1))
    else:
        ids = list(range(1, 9))  # default: 7 arm + gripper

    period = 1.0 / args.hz

    print(f"Connecting to {args.port} @ {args.baudrate}, IDs {ids} ...")
    driver = DynamixelDriver(ids, port=args.port, baudrate=args.baudrate)
    print("Connected. Move the arm; Ctrl+C to stop.\n")

    header = "  ".join(f"id{i:<2d}" for i in ids)
    try:
        while True:
            if args.show_vel:
                pos, vel = driver.get_positions_and_velocities()
            else:
                pos = driver.get_joints()
                vel = None

            rad = "  ".join(f"{v:+6.3f}" for v in pos)
            deg = "  ".join(f"{np.rad2deg(v):+7.2f}" for v in pos)
            sys.stdout.write("\x1b[2J\x1b[H")  # clear screen, cursor home
            print("Live Dynamixel readings (raw, no calibration)")
            print(f"port: {args.port}   ids: {ids}\n")
            print(f"        {header}")
            print(f"rad :   {rad}")
            print(f"deg :   {deg}")
            if vel is not None:
                vstr = "  ".join(f"{v:+6.3f}" for v in vel)
                print(f"vel :   {vstr}   (rad/s)")
            print("\nCtrl+C to stop.")
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nstopping...")
    finally:
        driver.close()


if __name__ == "__main__":
    main()
