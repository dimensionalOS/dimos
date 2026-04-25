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

Delegates to the existing ``MujocoConnection`` subprocess (the same
infrastructure ``unitree-go2 --simulation`` uses), running in
"low-level passthrough" mode: the subprocess owns the MuJoCo world +
viewer; this adapter reads per-joint state and writes per-joint
commands through shared memory.

That choice — reusing the battle-tested subprocess pattern — is what
lets ``dimos --simulation run unitree-g1-groot-wbc`` open the viewer
on macOS without the user prefixing ``mjpython``.  The subprocess is
auto-spawned under ``mjpython`` on macOS by ``MujocoConnection``
(``mujoco_connection.py:124``).
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.core.global_config import global_config as _global_config
from dimos.hardware.whole_body.spec import (
    POS_STOP,
    IMUState,
    MotorCommand,
    MotorState,
)
from dimos.robot.unitree.mujoco_connection import MujocoConnection
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.global_config import GlobalConfig
    from dimos.hardware.whole_body.registry import WholeBodyAdapterRegistry

logger = setup_logger()

_NUM_MOTORS = 29


class SimMujocoG1WholeBodyAdapter:
    """Whole-body adapter backed by a ``MujocoConnection`` in low-level mode.

    The connection spawns the standard ``mujoco_process.py`` subprocess
    (``dimos/simulation/mujoco/mujoco_process.py``), auto-selecting
    ``mjpython`` on macOS, and passes ``control_mode="low_level"`` so
    the subprocess skips its baked locomotion ONNX and instead reads
    per-joint commands from shared memory.

    ``GlobalConfig.robot_model`` must be ``"unitree_g1"`` (the blueprint
    sets this) so the subprocess loads the G1 MJCF.  ``mujoco_room``
    controls which scene wraps the robot (default ``"office1"``; the
    blueprint overrides to ``"empty"`` for a flat floor).

    Args:
        network_interface: Unused; kept for adapter-registry kwarg
            symmetry with the DDS adapter.
        domain_id: Unused; same reason.
        cfg: Global config to pass to the subprocess.  Defaults to the
            process-wide ``global_config`` (what the CLI populates).
    """

    def __init__(
        self,
        network_interface: int | str = 0,
        domain_id: int = 0,
        cfg: GlobalConfig | None = None,
        **_: Any,
    ) -> None:
        # Force the two MuJoCo-subprocess-relevant knobs on our own copy
        # of the config, regardless of what the worker's ``global_config``
        # singleton says.  The worker process starts fresh (forkserver
        # spawn), so blueprint-level ``.global_config(robot_model=...)``
        # overrides applied in the main process do NOT propagate into
        # the worker's singleton.  We hard-pin G1 + empty scene here so
        # the subprocess always loads the right model.
        base = cfg if cfg is not None else _global_config
        self._cfg = base.model_copy(update={"robot_model": "unitree_g1", "mujoco_room": "empty"})
        self._connection: MujocoConnection | None = None
        self._connected = False
        # Warn once if downstream consumers try to use the adapter
        # before the first state packet lands in shm.
        self._warned_no_state = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        try:
            self._connection = MujocoConnection(self._cfg, control_mode="low_level")
            self._connection.start()
            # Block briefly until the child has actually produced a state
            # packet, so the first read_motor_states() returns valid data
            # (otherwise the coordinator's first tick sees zeros and the
            # WBC task builds a junk obs).
            deadline = time.time() + 5.0
            while time.time() < deadline:
                if self._connection.read_motor_states(_NUM_MOTORS) is not None:
                    break
                time.sleep(0.05)
            self._connected = True
            logger.info("SimMujocoG1WholeBodyAdapter connected (subprocess ready)")
            return True
        except Exception as e:
            logger.error(f"Failed to start MuJoCo G1 sim subprocess: {e}")
            self._connected = False
            return False

    def disconnect(self) -> None:
        if self._connection is not None:
            try:
                self._connection.stop()
            except Exception as e:  # best-effort cleanup
                logger.warning(f"MuJoCo sim subprocess stop raised: {e}")
        self._connection = None
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected and self._connection is not None

    # ------------------------------------------------------------------
    # IO (WholeBodyAdapter protocol)
    # ------------------------------------------------------------------

    def read_motor_states(self) -> list[MotorState]:
        if not self._connected or self._connection is None:
            return [MotorState()] * _NUM_MOTORS
        arr = self._connection.read_motor_states(_NUM_MOTORS)
        if arr is None:
            if not self._warned_no_state:
                logger.warning("MuJoCo subprocess has not produced any state yet")
                self._warned_no_state = True
            return [MotorState()] * _NUM_MOTORS
        return [
            MotorState(q=float(arr[i, 0]), dq=float(arr[i, 1]), tau=float(arr[i, 2]))
            for i in range(_NUM_MOTORS)
        ]

    def read_imu(self) -> IMUState:
        if not self._connected or self._connection is None:
            return IMUState()
        arr = self._connection.read_imu_sensor()
        if arr is None or len(arr) < 10:
            return IMUState()
        w, x, y, z = (float(arr[0]), float(arr[1]), float(arr[2]), float(arr[3]))
        gyro = (float(arr[4]), float(arr[5]), float(arr[6]))
        accel = (float(arr[7]), float(arr[8]), float(arr[9]))
        # Derive ZYX Euler from the quaternion — matches the real G1 adapter.
        sinr = 2.0 * (w * x + y * z)
        cosr = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr, cosr)
        sinp = 2.0 * (w * y - z * x)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny, cosy)
        return IMUState(
            quaternion=(w, x, y, z),
            gyroscope=gyro,
            accelerometer=accel,
            rpy=(roll, pitch, yaw),
        )

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        if not self._connected or self._connection is None:
            return False
        if len(commands) != _NUM_MOTORS:
            logger.error(
                f"SimMujocoG1WholeBodyAdapter: expected {_NUM_MOTORS} commands, got {len(commands)}"
            )
            return False
        q = np.empty(_NUM_MOTORS, dtype=np.float32)
        kp = np.empty(_NUM_MOTORS, dtype=np.float32)
        kd = np.empty(_NUM_MOTORS, dtype=np.float32)
        for i, cmd in enumerate(commands):
            # POS_STOP ("no command") — write current state back as the
            # target so the subprocess doesn't see a stale target drift.
            q[i] = cmd.q if cmd.q != POS_STOP else 0.0
            kp[i] = cmd.kp
            kd[i] = cmd.kd
        self._connection.write_motor_commands(q, kp, kd)
        return True


def register(registry: WholeBodyAdapterRegistry) -> None:
    """Register with the whole-body adapter registry."""
    registry.register("sim_mujoco_g1", SimMujocoG1WholeBodyAdapter)


__all__ = ["SimMujocoG1WholeBodyAdapter"]
