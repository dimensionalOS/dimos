# Copyright 2026 Dimensional Inc.
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

"""Native-position MuJoCo WholeBodyAdapter for the 14-DOF MicroDuck."""

from __future__ import annotations

from pathlib import Path
import time
from typing import Any

from dimos.hardware.spec import JointLimits
from dimos.hardware.whole_body.spec import POS_STOP, IMUState, MotorCommand, MotorState
from dimos.simulation.engines.mujoco_shm import ManipShmReader, shm_key_from_path
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_NUM_MOTORS = 14
_READY_WAIT_TIMEOUT_S = 180.0
_READY_WAIT_POLL_S = 0.1
_ATTACH_RETRY_TIMEOUT_S = 30.0
_ATTACH_RETRY_POLL_S = 0.2


class SimMujocoMicroDuckWholeBodyAdapter:
    """Proxy a MicroDuck coordinator component to ``MujocoSimModule`` SHM.

    Unlike the G1 simulator adapter, this deliberately ignores MotorCommand
    gains and torque: the official MicroDuck MJCF supplies tuned ``<position>``
    actuators, matching upstream's functional ``--no-bam`` inference path.
    """

    def __init__(self, address: str | Path | None = None, **_: Any) -> None:
        if address is None:
            raise ValueError(
                "SimMujocoMicroDuckWholeBodyAdapter requires the same MJCF address "
                "as MujocoSimModule"
            )
        self._address = address
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
                        "MicroDuck MuJoCo SHM buffers not found",
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
                    "MicroDuck MuJoCo module did not become ready",
                    timeout_s=_READY_WAIT_TIMEOUT_S,
                )
                self._shm.cleanup()
                self._shm = None
                return False
            time.sleep(_READY_WAIT_POLL_S)

        if self._shm.num_joints() != _NUM_MOTORS:
            actual = self._shm.num_joints()
            logger.error(
                "MicroDuck MuJoCo joint count mismatch", expected=_NUM_MOTORS, actual=actual
            )
            self._shm.cleanup()
            self._shm = None
            return False

        self._connected = True
        logger.info(
            "MicroDuck MuJoCo adapter connected",
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
        return self._connected and self._shm is not None

    def has_motor_states(self) -> bool:
        return self.is_connected()

    def read_motor_states(self) -> list[MotorState]:
        if not self.has_motor_states():
            return [MotorState()] * _NUM_MOTORS
        assert self._shm is not None
        positions = self._shm.read_positions(_NUM_MOTORS)
        velocities = self._shm.read_velocities(_NUM_MOTORS)
        efforts = self._shm.read_efforts(_NUM_MOTORS)
        return [
            MotorState(q=positions[index], dq=velocities[index], tau=efforts[index])
            for index in range(_NUM_MOTORS)
        ]

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

    def get_limits(self) -> JointLimits | None:
        return None

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        if not self.is_connected():
            return False
        if len(commands) != _NUM_MOTORS:
            logger.error(
                "MicroDuck MuJoCo command count mismatch",
                expected=_NUM_MOTORS,
                actual=len(commands),
            )
            return False
        positions = [command.q if command.q != POS_STOP else 0.0 for command in commands]
        assert self._shm is not None
        self._shm.write_position_command(positions)
        return True
