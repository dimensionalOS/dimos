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
from typing import TYPE_CHECKING, Any

from dimos.hardware.manipulators.spec import (
    ControlMode,
    JointLimits,
    ManipulatorInfo,
)
from dimos.simulation.engines.mujoco_shm import (
    ManipShmReader,
    shm_key_from_path,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.hardware.manipulators.registry import AdapterRegistry


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
        self._address = address
        self._hardware_id = hardware_id
        self._shm_key = shm_key_from_path(address)
        self._shm: ManipShmReader | None = None
        self._connected = False
        self._servos_enabled = False
        self._control_mode = ControlMode.POSITION
        self._error_code = 0
        self._error_message = ""
        self._has_gripper = False
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

        num_joints = self._shm.num_joints()
        self._has_gripper = num_joints > self._dof
        self._connected = True
        self._servos_enabled = True
        logger.info("ShmMujocoAdapter connected", dof=self._dof, gripper=self._has_gripper)
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

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(
            vendor="Simulation",
            model="Simulation",
            dof=self._dof,
            firmware_version=None,
            serial_number=None,
        )

    def get_dof(self) -> int:
        return self._dof

    def get_limits(self) -> JointLimits:
        lower = [-math.pi] * self._dof
        upper = [math.pi] * self._dof
        max_vel_rad = math.radians(180.0)
        return JointLimits(
            position_lower=lower,
            position_upper=upper,
            velocity_max=[max_vel_rad] * self._dof,
        )

    def set_control_mode(self, mode: ControlMode) -> bool:
        self._control_mode = mode
        return True

    def get_control_mode(self) -> ControlMode:
        return self._control_mode

    def read_joint_positions(self) -> list[float]:
        if self._shm is None:
            return [0.0] * self._dof
        return self._shm.read_positions(self._dof)

    def read_joint_velocities(self) -> list[float]:
        if self._shm is None:
            return [0.0] * self._dof
        return self._shm.read_velocities(self._dof)

    def read_joint_efforts(self) -> list[float]:
        if self._shm is None:
            return [0.0] * self._dof
        return self._shm.read_efforts(self._dof)

    def read_state(self) -> dict[str, int]:
        velocities = self.read_joint_velocities()
        is_moving = any(abs(v) > 1e-4 for v in velocities)
        mode_int = list(ControlMode).index(self._control_mode)
        return {"state": 1 if is_moving else 0, "mode": mode_int}

    def read_error(self) -> tuple[int, str]:
        return self._error_code, self._error_message

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        if not self._servos_enabled or self._shm is None:
            return False
        self._control_mode = ControlMode.POSITION
        self._shm.write_position_command(positions[: self._dof])
        return True

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        if not self._servos_enabled or self._shm is None:
            return False
        self._control_mode = ControlMode.VELOCITY
        self._shm.write_velocity_command(velocities[: self._dof])
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

    def read_gripper_position(self) -> float | None:
        if not self._has_gripper or self._shm is None:
            return None
        return self._shm.read_gripper_position()

    def write_gripper_position(self, position: float) -> bool:
        if not self._has_gripper or self._shm is None:
            return False
        self._shm.write_gripper_command(position)
        return True

    def read_force_torque(self) -> list[float] | None:
        return None


def register(registry: AdapterRegistry) -> None:
    """Register this adapter with the registry."""
    registry.register("sim_mujoco", ShmMujocoAdapter)


__all__ = ["ShmMujocoAdapter"]
