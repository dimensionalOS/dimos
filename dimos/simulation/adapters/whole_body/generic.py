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

"""Robot-agnostic MuJoCo simulation ``WholeBodyAdapter``.

Generalises ``SimMujocoG1WholeBodyAdapter``: motor count comes from config,
the IMU is optional, and the command path is selectable. Pairs with
``MujocoSimModule`` over the shared-memory bridge keyed on the MJCF path.
"""

from __future__ import annotations

from pathlib import Path
import time
from typing import Any, Literal

from dimos.hardware.spec import JointLimits
from dimos.hardware.whole_body.spec import (
    POS_STOP,
    IMUState,
    MotorCommand,
    MotorState,
)
from dimos.simulation.engines.mujoco_shm import (
    MAX_JOINTS,
    ManipShmReader,
    shm_key_from_path,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_READY_WAIT_TIMEOUT_S = 180.0
_READY_WAIT_POLL_S = 0.1
_ATTACH_RETRY_TIMEOUT_S = 30.0
_ATTACH_RETRY_POLL_S = 0.2
_REATTACH_INTERVAL_S = 2.0

CommandMode = Literal["position", "pd_tau"]


class SimMujocoWholeBodyAdapter:
    """``WholeBodyAdapter`` over the ``MujocoSimModule`` SHM bridge, N motors.

    ``command_mode="pd_tau"`` forwards (q, kp, kd, tau) for the sim-side PD
    hook to turn into actuator torque, which needs direct-torque MJCF
    actuators. ``command_mode="position"`` writes joint targets straight
    through, for MJCFs whose actuators already close a position loop; kp/kd
    and feedforward torque are then the MJCF's, not the coordinator's.

    Grippers are ordinary entries in ``joints``: their MJCF joint is driven
    like any other, and ``HardwareComponent.limits`` declares the command
    coordinate. The single gripper SHM slot stays unused.
    """

    def __init__(
        self,
        address: str | Path | None = None,
        num_motors: int | None = None,
        *,
        dof: int | None = None,
        require_imu: bool = False,
        command_mode: CommandMode = "pd_tau",
        **_: Any,
    ) -> None:
        if address is None:
            raise ValueError(
                "SimMujocoWholeBodyAdapter: address (MJCF XML path) is required - "
                "set HardwareComponent.address to the same MJCF path the "
                "MujocoSimModule loads."
            )
        resolved_motors = num_motors if num_motors is not None else dof
        if resolved_motors is None or resolved_motors <= 0:
            raise ValueError(
                "SimMujocoWholeBodyAdapter: num_motors must be a positive count "
                f"(got {resolved_motors!r})"
            )
        if resolved_motors > MAX_JOINTS:
            raise ValueError(
                f"SimMujocoWholeBodyAdapter: num_motors {resolved_motors} exceeds the "
                f"SHM limit of {MAX_JOINTS}"
            )
        if command_mode not in ("position", "pd_tau"):
            raise ValueError(
                f"SimMujocoWholeBodyAdapter: unknown command_mode {command_mode!r}; "
                "expected 'position' or 'pd_tau'"
            )
        self._address = address
        self._num_motors = int(resolved_motors)
        self._require_imu = require_imu
        self._command_mode: CommandMode = command_mode
        self._shm_key = shm_key_from_path(address)
        self._shm: ManipShmReader | None = None
        self._connected = False
        self._has_imu = False

    def connect(self) -> bool:
        deadline = time.monotonic() + _ATTACH_RETRY_TIMEOUT_S
        while (shm := self._attach()) is None:
            if time.monotonic() > deadline:
                logger.error(
                    "SimMujocoWholeBodyAdapter: SHM buffers not found",
                    address=self._address,
                    shm_key=self._shm_key,
                    timeout_s=_ATTACH_RETRY_TIMEOUT_S,
                )
                return False
            time.sleep(_ATTACH_RETRY_POLL_S)
        self._shm = shm

        # The sim signals ready only after the first joint-state packet, so
        # without this wait the first read_motor_states() returns zeros.
        # Coordinators are constructed before the sim module starts, so the
        # buffers found above may be a dead predecessor's that MujocoSimModule
        # then unlinks and recreates - re-attach while waiting or we hold a
        # mapping that never goes ready.
        deadline = time.monotonic() + _READY_WAIT_TIMEOUT_S
        next_reattach = time.monotonic() + _REATTACH_INTERVAL_S
        while not self._shm.is_ready():
            if time.monotonic() > deadline:
                logger.error(
                    "SimMujocoWholeBodyAdapter: sim module not ready",
                    timeout_s=_READY_WAIT_TIMEOUT_S,
                )
                self._shm.cleanup()
                self._shm = None
                return False
            if time.monotonic() >= next_reattach:
                next_reattach = time.monotonic() + _REATTACH_INTERVAL_S
                if (fresh := self._attach()) is not None:
                    self._shm.cleanup()
                    self._shm = fresh
            time.sleep(_READY_WAIT_POLL_S)

        published = self._shm.num_joints()
        if published < self._num_motors:
            logger.error(
                "SimMujocoWholeBodyAdapter: sim publishes fewer joints than configured",
                published=published,
                num_motors=self._num_motors,
            )
            self._shm.cleanup()
            self._shm = None
            return False

        # An MJCF without IMU sensors leaves the buffer untouched; any written
        # sample carries a unit quaternion, so a zero quaternion means absent.
        self._has_imu = any(self._shm.read_imu()[0])
        if self._require_imu and not self._has_imu:
            logger.error(
                "SimMujocoWholeBodyAdapter: require_imu set but the MJCF publishes no IMU",
                address=self._address,
            )
            self._shm.cleanup()
            self._shm = None
            return False

        self._connected = True
        logger.info(
            "SimMujocoWholeBodyAdapter connected",
            num_motors=self._num_motors,
            command_mode=self._command_mode,
            has_imu=self._has_imu,
            shm_key=self._shm_key,
        )
        return True

    def _attach(self) -> ManipShmReader | None:
        try:
            return ManipShmReader(self._shm_key)
        except FileNotFoundError:
            return None

    def disconnect(self) -> None:
        if self._shm is not None:
            self._shm.cleanup()
        self._shm = None
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected and self._shm is not None

    def read_motor_states(self) -> list[MotorState]:
        if not self.has_motor_states():
            return [MotorState()] * self._num_motors
        assert self._shm is not None
        n = self._num_motors
        positions = self._shm.read_positions(n)
        velocities = self._shm.read_velocities(n)
        efforts = self._shm.read_efforts(n)
        return [MotorState(q=positions[i], dq=velocities[i], tau=efforts[i]) for i in range(n)]

    def has_motor_states(self) -> bool:
        # Sim ground truth is there as soon as SHM attaches; no ramp-up window.
        return self._connected and self._shm is not None

    def read_imu(self) -> IMUState:
        if not self._has_imu or not self.has_motor_states():
            return IMUState()
        assert self._shm is not None
        quat, gyro, accel = self._shm.read_imu()
        return IMUState(quaternion=quat, gyroscope=gyro, accelerometer=accel)

    def get_limits(self) -> JointLimits | None:
        """Limits are the MJCF's; the component declares the command coordinate."""
        return None

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        if not self.is_connected():
            return False
        assert self._shm is not None
        if len(commands) != self._num_motors:
            logger.error(
                f"SimMujocoWholeBodyAdapter: expected {self._num_motors} commands, "
                f"got {len(commands)}"
            )
            return False
        q = self._resolve_positions(commands)
        if self._command_mode == "position":
            self._shm.write_position_command(q)
            return True
        self._shm.write_pd_tau_command(
            q,
            [cmd.kp for cmd in commands],
            [cmd.kd for cmd in commands],
            [cmd.tau for cmd in commands],
        )
        return True

    def _resolve_positions(self, commands: list[MotorCommand]) -> list[float]:
        """Substitute the measured position for POS_STOP ("no command") joints."""
        if not any(cmd.q == POS_STOP for cmd in commands):
            return [cmd.q for cmd in commands]
        assert self._shm is not None
        measured = self._shm.read_positions(self._num_motors)
        return [measured[i] if cmd.q == POS_STOP else cmd.q for i, cmd in enumerate(commands)]
