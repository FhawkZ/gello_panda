"""Read-only Dynamixel driver for a GELLO leader arm.

This is a trimmed, cross-platform version of the driver from
`gello_software/gello/dynamixel/driver.py`. It keeps ONLY what is needed to
read joint positions/velocities from the servos (the leader side). All of the
follower-side functionality (position/current/torque writing, gravity
compensation helpers) and the Linux-only port management helpers have been
removed so the module works on Windows (`COMx`), Linux (`/dev/...`) and macOS.

Conversion convention (identical to GELLO):
    position_in_radians = raw_pulse / 2048.0 * pi
    velocity_in_rad_s   = raw_unit  * 0.229 * 2 * pi / 60

A background thread continuously polls the bus so that `get_joints()` always
returns the most recent reading without blocking on serial I/O.
"""

from __future__ import annotations

import time
from threading import Event, Lock, Thread
from typing import Optional, Protocol, Sequence, Tuple

import numpy as np

# `dynamixel_sdk` is imported lazily inside `DynamixelDriver.__init__` so that
# the calibration/mapping logic (and tests using an injected driver) can be
# imported without the SDK installed.
COMM_SUCCESS = 0  # value from dynamixel_sdk.robotis_def; redefined to avoid import

# --- Control table (Dynamixel X-series, protocol 2.0) ---
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
ADDR_PRESENT_VELOCITY = 128
LEN_PRESENT_VELOCITY = 4
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0


class DynamixelDriverProtocol(Protocol):
    def get_joints(self) -> np.ndarray:
        """Return current joint angles in radians."""
        ...

    def get_positions_and_velocities(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return (positions [rad], velocities [rad/s])."""
        ...

    def set_torque_mode(self, enable: bool) -> None:
        """Enable/disable servo torque. Leader reading wants this disabled."""
        ...

    def close(self) -> None:
        """Release the serial port."""
        ...


class DynamixelDriver(DynamixelDriverProtocol):
    """Reads joint state from a chain of Dynamixel servos over a U2D2/USB port."""

    def __init__(
        self,
        ids: Sequence[int],
        port: str = "COM3",
        baudrate: int = 57600,
    ):
        """
        Args:
            ids: Servo IDs in bus order, e.g. (1, 2, 3, 4, 5, 6, 7) for a 7-DoF
                arm, optionally plus the gripper id (e.g. 8) as the last entry.
            port: Serial port. Windows: "COM5"; Linux: "/dev/ttyUSB0" or a
                "/dev/serial/by-id/..." path; macOS: "/dev/cu.usbserial-XXXX".
            baudrate: Bus baudrate (GELLO default 57600).
        """
        from dynamixel_sdk.group_sync_read import GroupSyncRead
        from dynamixel_sdk.packet_handler import PacketHandler
        from dynamixel_sdk.port_handler import PortHandler

        self._ids = list(ids)
        self._joint_angles: Optional[np.ndarray] = None
        self._velocities: Optional[np.ndarray] = None
        self._lock = Lock()
        self._port = port
        self._baudrate = baudrate
        self._torque_enabled = False
        self._stop_thread = Event()

        self._port_handler = PortHandler(self._port)
        self._packet_handler = PacketHandler(2.0)
        # One transaction reads velocity + position for every servo.
        self._group_sync_read = GroupSyncRead(
            self._port_handler,
            self._packet_handler,
            ADDR_PRESENT_VELOCITY,
            LEN_PRESENT_VELOCITY + LEN_PRESENT_POSITION,
        )

        if not self._port_handler.openPort():
            raise RuntimeError(f"Failed to open port {self._port}")
        if not self._port_handler.setBaudRate(self._baudrate):
            raise RuntimeError(f"Failed to set baudrate {self._baudrate}")

        for dxl_id in self._ids:
            if not self._group_sync_read.addParam(dxl_id):
                raise RuntimeError(f"Failed to add sync-read param for ID {dxl_id}")

        # Leader arm is read passively -> make sure torque is off.
        try:
            self.set_torque_mode(False)
        except Exception as exc:  # noqa: BLE001 - non-fatal, keep reading
            print(f"[DynamixelDriver] warning while disabling torque: {exc}")

        self._start_reading_thread()

    # ------------------------------------------------------------------ #
    # Reading                                                            #
    # ------------------------------------------------------------------ #
    def _start_reading_thread(self) -> None:
        self._reading_thread = Thread(target=self._read_joint_states, daemon=True)
        self._reading_thread.start()

    def _read_joint_states(self) -> None:
        while not self._stop_thread.is_set():
            time.sleep(0.001)
            with self._lock:
                angles = np.zeros(len(self._ids), dtype=int)
                velocities = np.zeros(len(self._ids), dtype=int)
                if self._group_sync_read.txRxPacket() != COMM_SUCCESS:
                    continue
                ok = True
                for i, dxl_id in enumerate(self._ids):
                    if self._group_sync_read.isAvailable(
                        dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY
                    ):
                        vel = self._group_sync_read.getData(
                            dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY
                        )
                        if vel > 0x7FFFFFFF:  # 32-bit two's complement
                            vel -= 0x100000000
                        velocities[i] = vel
                    else:
                        ok = False
                        break

                    if self._group_sync_read.isAvailable(
                        dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                    ):
                        ang = self._group_sync_read.getData(
                            dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                        )
                        if ang > 0x7FFFFFFF:
                            ang -= 0x100000000
                        angles[i] = ang
                    else:
                        ok = False
                        break

                if ok:
                    self._joint_angles = angles
                    self._velocities = velocities

    def get_joints(self) -> np.ndarray:
        while self._joint_angles is None:
            time.sleep(0.05)
        with self._lock:
            raw = self._joint_angles.copy()
        return raw / 2048.0 * np.pi

    def get_positions_and_velocities(self) -> Tuple[np.ndarray, np.ndarray]:
        while self._joint_angles is None or self._velocities is None:
            time.sleep(0.05)
        with self._lock:
            raw_pos = self._joint_angles.copy()
            raw_vel = self._velocities.copy()
        positions = raw_pos / 2048.0 * np.pi
        velocities = raw_vel * 0.229 * 2 * np.pi / 60.0
        return positions, velocities

    # ------------------------------------------------------------------ #
    # Torque (only used to make sure the leader stays passive)           #
    # ------------------------------------------------------------------ #
    def set_torque_mode(self, enable: bool) -> None:
        value = TORQUE_ENABLE if enable else TORQUE_DISABLE
        with self._lock:
            for dxl_id in self._ids:
                comm, err = self._packet_handler.write1ByteTxRx(
                    self._port_handler, dxl_id, ADDR_TORQUE_ENABLE, value
                )
                if comm != COMM_SUCCESS or err != 0:
                    raise RuntimeError(f"Failed to set torque mode for ID {dxl_id}")
        self._torque_enabled = enable

    def close(self) -> None:
        self._stop_thread.set()
        if hasattr(self, "_reading_thread"):
            self._reading_thread.join(timeout=1.0)
        self._port_handler.closePort()
