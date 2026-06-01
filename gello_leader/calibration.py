"""Calibration container for a GELLO leader arm.

Holds the parameters produced by the offset-detection step and knows how to
load/save them as JSON so the configuration is portable across machines and
projects.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple, Union


@dataclass
class LeaderCalibration:
    """All parameters needed to turn raw servo angles into follower joints.

    Attributes:
        port: Serial port of the U2D2 (e.g. "COM5" or "/dev/ttyUSB0").
        joint_ids: Arm servo IDs in bus order, NOT including the gripper.
        joint_offsets: One offset (rad) per arm joint, from calibration.
        joint_signs: One sign (+1 / -1) per arm joint.
        gripper: Optional (servo_id, open_deg, close_deg). If None, no gripper.
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
        return cls(
            port=data["port"],
            joint_ids=list(data["joint_ids"]),
            joint_offsets=[float(x) for x in data["joint_offsets"]],
            joint_signs=[int(x) for x in data["joint_signs"]],
            gripper=gripper,
            baudrate=int(data.get("baudrate", 57600)),
        )
