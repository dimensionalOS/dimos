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


from __future__ import annotations

from pathlib import Path
from typing import Any

from dimos.hardware.spec import JointLimits
from dimos.hardware.whole_body.spec import (
    POS_STOP,
    IMUState,
    MotorCommand,
    MotorState,
)
from dimos.simulation.engines.mujoco_shm import (
    ManipShmReader,
    shm_key_from_path,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_NUM_MOTORS = 29

_READY_WAIT_TIMEOUT_S = 180.0
_READY_WAIT_POLL_S = 0.1
_ATTACH_RETRY_TIMEOUT_S = 30.0
_ATTACH_RETRY_POLL_S = 0.2


class SimMujocoG1WholeBodyAdapter:
    def __init__(
        self,
        address: str | Path | None = None,
        **_: Any,
    ) -> None:
        if address is None:
            raise ValueError(
                "SimMujocoG1WholeBodyAdapter: address (MJCF XML path) is required - "
                "set HardwareComponent.address to the same MJCF path the "
                "MujocoSimModule loads."
            )
        self._address = address
        self._shm_key = shm_key_from_path(address)
        self._shm: ManipShmReader | None = None
        self._connected = False

    def connect(self) -> bool:
        self._shm = ManipShmReader.wait_for_live(
            self._shm_key,
            attach_timeout_s=_ATTACH_RETRY_TIMEOUT_S,
            ready_timeout_s=_READY_WAIT_TIMEOUT_S,
            poll_s=min(_ATTACH_RETRY_POLL_S, _READY_WAIT_POLL_S),
        )
        if self._shm is None:
            logger.error(
                "SimMujocoG1WholeBodyAdapter: no fresh joint state became ready",
                shm_key=self._shm_key,
            )
            return False

        self._connected = True
        logger.info(
            "SimMujocoG1WholeBodyAdapter connected",
            num_motors=_NUM_MOTORS,
            shm_key=self._shm_key,
        )
        return True

    def disconnect(self) -> None:
        if self._shm is not None:
            self._shm.cleanup()
        self._shm = None
        self._connected = False

    def is_connected(self) -> bool:
        return self._live_shm() is not None

    def read_motor_states(self) -> list[MotorState]:
        if not self.has_motor_states():
            return [MotorState()] * _NUM_MOTORS
        assert self._shm is not None
        positions = self._shm.read_positions(_NUM_MOTORS)
        velocities = self._shm.read_velocities(_NUM_MOTORS)
        efforts = self._shm.read_efforts(_NUM_MOTORS)
        return [
            MotorState(q=positions[i], dq=velocities[i], tau=efforts[i]) for i in range(_NUM_MOTORS)
        ]

    def has_motor_states(self) -> bool:
        return self._live_shm() is not None

    def motor_states_ts(self) -> float | None:
        return None

    def read_imu(self) -> IMUState:
        if not self.has_motor_states():
            return IMUState()
        assert self._shm is not None
        quat, gyro, accel = self._shm.read_imu()
        return IMUState(
            quaternion=quat,
            gyroscope=gyro,
            accelerometer=accel,
        )

    def get_limits(self) -> JointLimits | None:
        return None

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        shm = self._live_shm()
        if shm is None:
            return False
        if len(commands) != _NUM_MOTORS:
            logger.error(
                f"SimMujocoG1WholeBodyAdapter: expected {_NUM_MOTORS} commands, got {len(commands)}"
            )
            return False
        q = [cmd.q if cmd.q != POS_STOP else 0.0 for cmd in commands]
        kp = [cmd.kp for cmd in commands]
        kd = [cmd.kd for cmd in commands]
        tau = [cmd.tau for cmd in commands]
        shm.write_pd_tau_command(q, kp, kd, tau)
        return True

    def _live_shm(self) -> ManipShmReader | None:
        shm = self._shm
        if not self._connected or shm is None:
            return None
        if shm.is_live():
            return shm
        shm.cleanup()
        self._shm = None
        self._connected = False
        logger.error(
            "SimMujocoG1WholeBodyAdapter: joint state stopped updating",
            shm_key=self._shm_key,
        )
        return None
