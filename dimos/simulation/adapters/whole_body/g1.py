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

"""MuJoCo simulation ``WholeBodyAdapter`` for the Unitree G1.

Pairs with ``MujocoSimModule`` (in-process MuJoCo engine + SHM publisher).
The blueprint composes both modules; they share the same ``MujocoEngine``
indirectly via SHM keyed on the MJCF path.

The adapter conforms to the same ``WholeBodyAdapter`` Protocol the real-hw
DDS adapter implements, so ControlCoordinator (and the GR00T WBC task on
top of it) can't tell sim from real.
"""

from __future__ import annotations

from pathlib import Path
import time
from typing import TYPE_CHECKING, Any

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

if TYPE_CHECKING:
    from dimos.hardware.whole_body.registry import WholeBodyAdapterRegistry

logger = setup_logger()

_NUM_MOTORS = 29

_READY_WAIT_TIMEOUT_S = 180.0
_READY_WAIT_POLL_S = 0.1
_ATTACH_RETRY_TIMEOUT_S = 30.0
_ATTACH_RETRY_POLL_S = 0.2


class SimMujocoG1WholeBodyAdapter:
    """G1 ``WholeBodyAdapter`` that proxies to a ``MujocoSimModule`` via SHM.

    The sim module owns the engine and publishes joint state + IMU into
    SHM each step; this adapter reads them and forwards per-joint
    (q, kp, kd, tau) commands back into SHM for the engine's pre-step
    PD-with-feedforward hook to apply.

    ``address`` (the MJCF XML path) is the discovery key - both sides
    derive the same SHM names from it via ``shm_key_from_path``.
    """

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

    # Lifecycle

    def connect(self) -> bool:
        # Attach with retry - MujocoSimModule may still be starting up.
        deadline = time.monotonic() + _ATTACH_RETRY_TIMEOUT_S
        while True:
            try:
                self._shm = ManipShmReader(self._shm_key)
                break
            except FileNotFoundError:
                if time.monotonic() > deadline:
                    logger.error(
                        "SimMujocoG1WholeBodyAdapter: SHM buffers not found",
                        address=self._address,
                        shm_key=self._shm_key,
                        timeout_s=_ATTACH_RETRY_TIMEOUT_S,
                    )
                    return False
                time.sleep(_ATTACH_RETRY_POLL_S)

        # Wait for the sim to signal ready (engine connected, first
        # joint-state packet written).  Without this the first
        # read_motor_states() returns zeros and the WBC obs is junk.
        deadline = time.monotonic() + _READY_WAIT_TIMEOUT_S
        while not self._shm.is_ready():
            if time.monotonic() > deadline:
                logger.error(
                    "SimMujocoG1WholeBodyAdapter: sim module not ready",
                    timeout_s=_READY_WAIT_TIMEOUT_S,
                )
                self._shm.cleanup()
                self._shm = None
                return False
            time.sleep(_READY_WAIT_POLL_S)

        self._connected = True
        logger.info(
            "SimMujocoG1WholeBodyAdapter connected",
            num_motors=_NUM_MOTORS,
            shm_key=self._shm_key,
        )
        return True

    def disconnect(self) -> None:
        if self._shm is not None:
            # ManipShmReader.cleanup() is already best-effort: it closes each
            # SHM buffer and swallows FileNotFoundError/OSError internally.
            self._shm.cleanup()
        self._shm = None
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected and self._shm is not None

    # IO (WholeBodyAdapter protocol)

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
        # Sim ground truth is available the moment SHM attaches.
        # No ramp-up window like real DDS adapters need before the
        # first state msg arrives.
        return self._connected and self._shm is not None

    def read_imu(self) -> IMUState:
        if not self.has_motor_states():
            return IMUState()
        assert self._shm is not None
        quat, gyro, accel = self._shm.read_imu()
        # rpy is left at its zero default to match the real G1 adapter
        # (TransportWholeBodyAdapter._on_imu). The WBC task's observation
        # uses only quaternion + gyroscope, so euler is never read downstream.
        return IMUState(
            quaternion=quat,
            gyroscope=gyro,
            accelerometer=accel,
        )

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        if not self.is_connected():
            return False
        assert self._shm is not None
        if len(commands) != _NUM_MOTORS:
            logger.error(
                f"SimMujocoG1WholeBodyAdapter: expected {_NUM_MOTORS} commands, got {len(commands)}"
            )
            return False
        # Flatten the per-motor command into per-joint arrays.  POS_STOP
        # ("no command") is replaced with 0.0 - the engine's PD only
        # acts when kp > 0 anyway, so a zeroed q is harmless.
        q = [cmd.q if cmd.q != POS_STOP else 0.0 for cmd in commands]
        kp = [cmd.kp for cmd in commands]
        kd = [cmd.kd for cmd in commands]
        tau = [cmd.tau for cmd in commands]
        self._shm.write_pd_tau_command(q, kp, kd, tau)
        return True


def register(registry: WholeBodyAdapterRegistry) -> None:
    """Register with the whole-body adapter registry."""
    registry.register("sim_mujoco_g1", SimMujocoG1WholeBodyAdapter)
