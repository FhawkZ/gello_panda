"""High-level reader: raw servo angles -> follower (Panda) joint space.

This reproduces the exact mapping used by GELLO's `DynamixelRobot`:

    q[i] = joint_signs[i] * (raw_rad[i] - joint_offsets[i])

and, for the gripper, normalises the last servo into [0, 1]:

    g = (q_gripper - open_rad) / (close_rad - open_rad)   # clipped to [0, 1]

`g = 0` means fully open, `g = 1` means fully closed (same convention GELLO
feeds to the Panda gripper, where width = 0.09 * (1 - g)).

An optional exponential smoothing matches GELLO's default behaviour but can be
disabled by setting `alpha=1.0`.
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

from .calibration import LeaderCalibration
from .dynamixel_driver import DynamixelDriver, DynamixelDriverProtocol


class LeaderArm:
    """Owns a Dynamixel driver and turns its readings into follower joints."""

    def __init__(
        self,
        calib: LeaderCalibration,
        alpha: float = 1.0,
        driver: Optional[DynamixelDriverProtocol] = None,
    ):
        """
        Args:
            calib: Calibration parameters (see `LeaderCalibration`).
            alpha: Exponential smoothing factor in (0, 1]. 1.0 = no smoothing;
                GELLO uses 0.99. Smoothing is applied to the full output vector.
            driver: Inject a custom/fake driver (useful for testing). When None,
                a real `DynamixelDriver` is created from the calibration.
        """
        self._calib = calib
        self._alpha = float(alpha)
        self._offsets = np.array(calib.joint_offsets, dtype=float)
        self._signs = np.array(calib.joint_signs, dtype=float)
        if calib.gripper is not None:
            self._gripper_open_rad = np.deg2rad(calib.gripper[1])
            self._gripper_close_rad = np.deg2rad(calib.gripper[2])
        else:
            self._gripper_open_rad = None
            self._gripper_close_rad = None

        self._last: Optional[np.ndarray] = None

        if driver is not None:
            self._driver = driver
        else:
            self._driver = DynamixelDriver(
                ids=calib.all_ids(),
                port=calib.port,
                baudrate=calib.baudrate,
            )

    # ------------------------------------------------------------------ #
    # Public API                                                         #
    # ------------------------------------------------------------------ #
    def get_joint_state(self) -> np.ndarray:
        """Return the processed state.

        Shape is (num_arm_joints,) with no gripper, or (num_arm_joints + 1,)
        where the last element is the gripper in [0, 1].
        """
        raw = self._driver.get_joints()
        n_arm = self._calib.num_arm_joints

        # Apply calibration to ALL read joints (arm + gripper if present).
        if self._calib.has_gripper:
            # offsets/signs cover only the arm; gripper uses sign +1, offset 0
            full_offsets = np.concatenate([self._offsets, [0.0]])
            full_signs = np.concatenate([self._signs, [1.0]])
        else:
            full_offsets = self._offsets
            full_signs = self._signs

        pos = (raw - full_offsets) * full_signs

        if self._calib.has_gripper:
            g = (pos[-1] - self._gripper_open_rad) / (
                self._gripper_close_rad - self._gripper_open_rad
            )
            pos[-1] = float(np.clip(g, 0.0, 1.0))

        if self._alpha >= 1.0 or self._last is None:
            self._last = pos
        else:
            pos = self._last * (1.0 - self._alpha) + pos * self._alpha
            self._last = pos

        return pos

    def get_arm_and_gripper(self) -> Tuple[np.ndarray, Optional[float]]:
        """Convenience split: (arm_joints[rad], gripper[0,1] or None)."""
        state = self.get_joint_state()
        if self._calib.has_gripper:
            return state[:-1].copy(), float(state[-1])
        return state.copy(), None

    @property
    def num_dofs(self) -> int:
        return self._calib.num_arm_joints + (1 if self._calib.has_gripper else 0)

    def close(self) -> None:
        self._driver.close()

    def __enter__(self) -> "LeaderArm":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
