# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Shared-memory adapter for MuJoCo-based manipulator simulation.
this adapter reads from and writes to the same SHM buffers.
"""

from __future__ import annotations

import math
import time
from typing import Any

from dimos.hardware.manipulators.spec import (
    ControlMode,
    ManipulatorInfo,
)
from dimos.hardware.spec import JointLimits
from dimos.simulation.engines.mujoco_shm import (
    ManipShmReader,
    shm_key_from_path,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_READY_WAIT_TIMEOUT_S = 60.0
_READY_WAIT_POLL_S = 0.1
_ATTACH_RETRY_TIMEOUT_S = 30.0
_ATTACH_RETRY_POLL_S = 0.2


class ShmMujocoAdapter:
    """``ManipulatorAdapter`` that proxies to a ``MujocoSimModule`` via SHM.

    Uses ``address`` (the MJCF XML path) as the discovery key. The sim module
    must be running and have signalled ready before ``connect()`` returns.
    """

    def __init__(
        self,
        dof: int = 7,
        address: str | None = None,
        hardware_id: str | None = None,
        **_: Any,
    ) -> None:
        if address is None:
            raise ValueError("address (MJCF XML path) is required for sim_mujoco adapter")
        self._dof = dof
        self._arm_dof = dof
        self._gripper_dof = 0
        self._gripper_range: tuple[float, float] = (0.0, 1.0)  # from SHM on connect
        self._address = address
        self._hardware_id = hardware_id
        self._shm_key = shm_key_from_path(address)
        self._shm: ManipShmReader | None = None
        self._connected = False
        self._servos_enabled = False
        self._control_mode = ControlMode.POSITION
        self._error_code = 0
        self._error_message = ""
        self._effort_mode_warned = False

    def connect(self) -> bool:
        deadline = time.monotonic() + _ATTACH_RETRY_TIMEOUT_S
        while True:
            try:
                self._shm = ManipShmReader(self._shm_key)
                break
            except FileNotFoundError:
                if time.monotonic() > deadline:
                    logger.error(
                        "SHM buffers not found",
                        address=self._address,
                        shm_key=self._shm_key,
                        timeout_s=_ATTACH_RETRY_TIMEOUT_S,
                    )
                    return False
                time.sleep(_ATTACH_RETRY_POLL_S)

        # Wait for sim module to signal ready.
        deadline = time.monotonic() + _READY_WAIT_TIMEOUT_S
        while not self._shm.is_ready():
            if time.monotonic() > deadline:
                logger.error("sim module not ready", timeout_s=_READY_WAIT_TIMEOUT_S)
                self._shm.cleanup()
                self._shm = None
                return False
            time.sleep(_READY_WAIT_POLL_S)

        if self._shm.num_joints() != self._dof:
            reported_dof = self._shm.num_joints()
            self._shm.cleanup()
            self._shm = None
            raise ValueError(f"sim_mujoco reports {reported_dof} joints, expected {self._dof}")
        self._arm_dof = self._shm.arm_joints()
        self._gripper_dof = self._dof - self._arm_dof
        if self._gripper_dof not in (0, 1):
            raise ValueError(
                f"sim_mujoco supports at most one extra joint (got {self._gripper_dof})"
            )
        if self._gripper_dof:
            self._gripper_range = self._shm.read_gripper_range()
        self._connected = True
        self._servos_enabled = True
        logger.info(
            "ShmMujocoAdapter connected",
            dof=self._dof,
            gripper_dof=self._gripper_dof,
            gripper_range=self._gripper_range if self._gripper_dof else None,
        )
        return True

    def disconnect(self) -> None:
        try:
            if self._shm is not None:
                self._shm.cleanup()
        finally:
            self._shm = None
            self._connected = False

    def is_connected(self) -> bool:
        return self._connected and self._shm is not None

    def activate(self) -> bool:
        return self.write_enable(True)

    def deactivate(self) -> bool:
        self.write_stop()
        return self.write_enable(False)

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(
            vendor="Simulation",
            model="Simulation",
            dof=self._dof,
            firmware_version=None,
            serial_number=None,
        )

    def get_dof(self) -> int:
        """Total joints owned by this adapter."""
        return self._dof

    def get_limits(self) -> JointLimits:
        """Arm limits in radians, then the gripper's MJCF joint range."""
        lo, hi = self._gripper_range
        max_vel_rad = math.radians(180.0)
        return JointLimits(
            position_lower=[-math.pi] * self._arm_dof + [lo] * self._gripper_dof,
            position_upper=[math.pi] * self._arm_dof + [hi] * self._gripper_dof,
            velocity_max=[max_vel_rad] * self._arm_dof + [0.0] * self._gripper_dof,
        )

    def set_control_mode(self, mode: ControlMode) -> bool:
        self._control_mode = mode
        return True

    def get_control_mode(self) -> ControlMode:
        return self._control_mode

    def read_joint_positions(self) -> list[float]:
        """Read arm positions, then the gripper's MJCF joint position."""
        if self._shm is None:
            return [0.0] * self._dof
        positions = self._shm.read_positions(self._arm_dof)
        if self._gripper_dof:
            positions.append(self._shm.read_gripper_position())
        return positions

    def read_joint_velocities(self) -> list[float]:
        """Read arm velocities; the gripper reports 0.0."""
        if self._shm is None:
            return [0.0] * self._dof
        return self._shm.read_velocities(self._arm_dof) + [0.0] * self._gripper_dof

    def read_joint_efforts(self) -> list[float]:
        if self._shm is None:
            return [0.0] * self._dof
        return self._shm.read_efforts(self._arm_dof) + [0.0] * self._gripper_dof

    def read_state(self) -> dict[str, int]:
        velocities = self.read_joint_velocities()
        is_moving = any(abs(v) > 1e-4 for v in velocities)
        mode_int = list(ControlMode).index(self._control_mode)
        return {"state": 1 if is_moving else 0, "mode": mode_int}

    def read_error(self) -> tuple[int, str]:
        return self._error_code, self._error_message

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        """Command all joints; the gripper entry is in the MJCF joint range."""
        if not self._servos_enabled or self._shm is None:
            return False
        self._control_mode = ControlMode.POSITION
        if len(positions) != self._dof:
            return False
        self._shm.write_position_command(positions[: self._arm_dof])
        if self._gripper_dof:
            self._shm.write_gripper_command(positions[self._arm_dof])
        return True

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        if not self._servos_enabled or self._shm is None:
            return False
        self._control_mode = ControlMode.VELOCITY
        if len(velocities) != self._dof:
            return False
        if any(value != 0.0 for value in velocities[self._arm_dof :]):
            return False
        self._shm.write_velocity_command(velocities[: self._arm_dof])
        return True

    def write_joint_efforts(self, efforts: list[float]) -> bool:
        # Effort mode not exposed via SHM yet; caller can fall back to position.
        if not self._effort_mode_warned:
            logger.warning(
                "write_joint_efforts not supported by sim adapter; ignoring and returning False",
                dof=self._dof,
            )
            self._effort_mode_warned = True
        return False

    def write_stop(self) -> bool:
        # Hold current position.
        if self._shm is None:
            return False
        positions = self._shm.read_positions(self._dof)
        self._shm.write_position_command(positions)
        return True

    def write_enable(self, enable: bool) -> bool:
        self._servos_enabled = enable
        return True

    def read_enabled(self) -> bool:
        return self._servos_enabled

    def write_clear_errors(self) -> bool:
        self._error_code = 0
        self._error_message = ""
        return True

    def read_cartesian_position(self) -> dict[str, float] | None:
        return None

    def write_cartesian_position(self, pose: dict[str, float], velocity: float = 1.0) -> bool:
        return False

    def read_force_torque(self) -> list[float] | None:
        return None
