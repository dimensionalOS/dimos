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

"""Shared-memory whole-body adapter for MuJoCo simulation providers."""

from __future__ import annotations

from pathlib import Path
import time
from typing import Any

from dimos.hardware.simulation.shared_memory import ManipShmReader, shm_key_from_path
from dimos.hardware.whole_body.spec import POS_STOP, IMUState, MotorCommand, MotorState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_READY_WAIT_TIMEOUT_S = 180.0
_READY_WAIT_POLL_S = 0.1
_ATTACH_RETRY_TIMEOUT_S = 30.0
_ATTACH_RETRY_POLL_S = 0.2


class SimMujocoWholeBodyAdapter:
    """Exchange whole-body state and PD-plus-torque commands through SHM."""

    def __init__(
        self,
        dof: int,
        address: str | Path | None = None,
        hardware_id: str | None = None,
        **_: Any,
    ) -> None:
        if dof <= 0:
            raise ValueError("sim_mujoco whole-body adapter requires a positive dof")
        if address is None:
            raise ValueError("address is required for sim_mujoco whole-body adapter")
        self._dof = dof
        self._address = address
        self._hardware_id = hardware_id
        self._shm_key = shm_key_from_path(address)
        self._shm: ManipShmReader | None = None
        self._connected = False

    def connect(self) -> bool:
        deadline = time.monotonic() + _ATTACH_RETRY_TIMEOUT_S
        while True:
            try:
                self._shm = ManipShmReader(self._shm_key)
                break
            except FileNotFoundError:
                if time.monotonic() > deadline:
                    logger.error(
                        "sim_mujoco whole-body SHM buffers not found",
                        address=self._address,
                        shm_key=self._shm_key,
                        timeout_s=_ATTACH_RETRY_TIMEOUT_S,
                    )
                    return False
                time.sleep(_ATTACH_RETRY_POLL_S)

        deadline = time.monotonic() + _READY_WAIT_TIMEOUT_S
        while not self._shm.is_ready():
            if time.monotonic() > deadline:
                logger.error(
                    "sim_mujoco whole-body runtime not ready",
                    timeout_s=_READY_WAIT_TIMEOUT_S,
                )
                self._shm.cleanup()
                self._shm = None
                return False
            time.sleep(_READY_WAIT_POLL_S)

        if self._shm.num_joints() != self._dof:
            logger.error(
                "sim_mujoco whole-body joint count mismatch",
                expected=self._dof,
                actual=self._shm.num_joints(),
            )
            self._shm.cleanup()
            self._shm = None
            return False

        self._connected = True
        logger.info(
            "SimMujocoWholeBodyAdapter connected",
            hardware_id=self._hardware_id,
            num_motors=self._dof,
            shm_key=self._shm_key,
        )
        return True

    def disconnect(self) -> None:
        if self._shm is not None:
            self._shm.cleanup()
        self._shm = None
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected and self._shm is not None

    def read_motor_states(self) -> list[MotorState]:
        if not self.has_motor_states():
            return [MotorState()] * self._dof
        assert self._shm is not None
        positions = self._shm.read_positions(self._dof)
        velocities = self._shm.read_velocities(self._dof)
        efforts = self._shm.read_efforts(self._dof)
        return [
            MotorState(q=positions[index], dq=velocities[index], tau=efforts[index])
            for index in range(self._dof)
        ]

    def has_motor_states(self) -> bool:
        return self._connected and self._shm is not None

    def read_imu(self) -> IMUState:
        if not self.has_motor_states():
            return IMUState()
        assert self._shm is not None
        quaternion, gyroscope, accelerometer = self._shm.read_imu()
        return IMUState(
            quaternion=quaternion,
            gyroscope=gyroscope,
            accelerometer=accelerometer,
        )

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        if not self.is_connected():
            return False
        if len(commands) != self._dof:
            logger.error(
                "sim_mujoco whole-body command count mismatch",
                expected=self._dof,
                actual=len(commands),
            )
            return False
        assert self._shm is not None
        self._shm.write_pd_tau_command(
            [command.q if command.q != POS_STOP else 0.0 for command in commands],
            [command.kp for command in commands],
            [command.kd for command in commands],
            [command.tau for command in commands],
        )
        return True


__all__ = ["SimMujocoWholeBodyAdapter"]
