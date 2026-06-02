"""Calibration container for a GELLO leader arm.

Holds the parameters produced by the offset-detection step and knows how to
load/save them as JSON so the configuration is portable across machines and
projects.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, List, Optional, Tuple, Union

import numpy as np


@dataclass
class LeaderCalibration:
    """All parameters needed to turn raw servo angles into follower joints.

    Attributes:
        port: Serial port of the U2D2 (e.g. "COM5" or "/dev/ttyUSB0").
        joint_ids: Arm servo IDs in bus order, NOT including the gripper.
        joint_offsets: One offset (rad) per arm joint, from calibration.
        joint_signs: One sign (+1 / -1) per arm joint.
        gripper: Optional (servo_id, open_deg, close_deg). open/close 为夹爪舵机
            在「完全张开 / 完全闭合」时读到的角度（度），用于线性映射到 [0, 1]。
            由标定脚本交互记录，勿手填。
        baudrate: Bus baudrate (GELLO default 57600).
    """

    port: str
    joint_ids: List[int]
    joint_offsets: List[float]
    joint_signs: List[int]
    gripper: Optional[Tuple[int, float, float]] = None
    baudrate: int = 57600

    def __post_init__(self) -> None:
        n = len(self.joint_ids)
        assert len(self.joint_offsets) == n, "joint_offsets length mismatch"
        assert len(self.joint_signs) == n, "joint_signs length mismatch"
        for s in self.joint_signs:
            assert s in (-1, 1), f"joint_signs must be +/-1, got {s}"

    @property
    def num_arm_joints(self) -> int:
        return len(self.joint_ids)

    @property
    def has_gripper(self) -> bool:
        return self.gripper is not None

    def validate_gripper_range(self) -> None:
        """Ensure open/close endpoints differ enough for mapping."""
        if self.gripper is None:
            return
        open_deg, close_deg = float(self.gripper[1]), float(self.gripper[2])
        if abs(open_deg - close_deg) < 0.5:
            raise ValueError(
                f"gripper open/close too close ({open_deg:.3f} vs {close_deg:.3f} deg); "
                "re-run gripper calibration."
            )

    def all_ids(self) -> List[int]:
        """Servo IDs including the gripper (if any), in read order."""
        if self.gripper is None:
            return list(self.joint_ids)
        return list(self.joint_ids) + [int(self.gripper[0])]

    # ------------------------------------------------------------------ #
    # Persistence                                                        #
    # ------------------------------------------------------------------ #
    def save(self, path: Union[str, Path]) -> None:
        data = asdict(self)
        if self.gripper is not None:
            data["gripper"] = list(self.gripper)
        Path(path).write_text(json.dumps(data, indent=2), encoding="utf-8")

    @classmethod
    def load(cls, path: Union[str, Path]) -> "LeaderCalibration":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        gripper = data.get("gripper")
        if gripper is not None:
            gripper = (int(gripper[0]), float(gripper[1]), float(gripper[2]))
        calib = cls(
            port=data["port"],
            joint_ids=list(data["joint_ids"]),
            joint_offsets=[float(x) for x in data["joint_offsets"]],
            joint_signs=[int(x) for x in data["joint_signs"]],
            gripper=gripper,
            baudrate=int(data.get("baudrate", 57600)),
        )
        calib.validate_gripper_range()
        return calib


def read_gripper_servo_deg(
    get_joints: Callable[[], np.ndarray],
    gripper_index: int = -1,
    warmup_reads: int = 5,
) -> float:
    """Read the gripper servo angle (degrees) after a few warmup polls."""
    for _ in range(warmup_reads):
        get_joints()
    return float(np.rad2deg(get_joints()[gripper_index]))


def calibrate_gripper_range_interactive(
    get_joints: Callable[[], np.ndarray],
    *,
    gripper_index: int = -1,
    min_span_deg: float = 1.0,
) -> Tuple[float, float]:
    """Record gripper open/close endpoints by hand; Enter confirms each pose.

    Returns:
        (open_deg, close_deg) mapping to gripper_01 = 0 (open) and 1 (closed).
    """
    print("\n=== 夹爪行程标定 ===")
    print("将 GELLO 夹爪完全张开，按回车记录「张开」端点 …")
    input()
    open_deg = read_gripper_servo_deg(get_joints, gripper_index)
    print(f"  已记录张开: {open_deg:.3f} deg")

    print("将 GELLO 夹爪完全闭合，按回车记录「闭合」端点 …")
    input()
    close_deg = read_gripper_servo_deg(get_joints, gripper_index)
    print(f"  已记录闭合: {close_deg:.3f} deg")

    span = abs(close_deg - open_deg)
    if span < min_span_deg:
        raise ValueError(
            f"张开/闭合角度过近（相差 {span:.3f} deg < {min_span_deg} deg），请重新标定。"
        )

    print(f"  行程: {span:.3f} deg  →  映射 gripper_01: 0=张开, 1=闭合")
    return open_deg, close_deg
