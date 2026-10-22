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

"""Pub/sub adapter for a simulated manipulator in another process.

The ``ShmMujocoAdapter`` contract over topics, for simulators that are not
``MujocoSimModule`` (e.g. a native process in its own environment). The sim publishes
``/{hardware_id}/sim_state`` (JointState: arm joints, then the gripper in command
units) and follows ``/{hardware_id}/sim_command`` (JointState positions, same order).
"""

from __future__ import annotations

from collections.abc import Callable
import math
import threading
import time
from typing import Any

from dimos.core.transport_factory import make_transport
from dimos.hardware.manipulators.spec import ControlMode, ManipulatorInfo
from dimos.hardware.spec import JointLimits
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_CONNECT_TIMEOUT_S = 120.0
_STALE_AFTER_S = 5.0


class TransportSimAdapter:
    """``ManipulatorAdapter`` for a sim that speaks ``sim_state`` / ``sim_command`` topics."""

    def __init__(
        self,
        dof: int = 7,
        hardware_id: str = "arm",
        gripper_range: tuple[float, float] = (0.0, 1.0),
        transport_cls: Callable[[str, type], Any] = make_transport,
        **_: Any,
    ) -> None:
        self._dof = dof
        self._prefix = hardware_id
        self._gripper_range = gripper_range
        self._transport_cls = transport_cls
        self._lock = threading.Lock()
        self._state: JointState | None = None
        self._state_at = 0.0
        self._state_transport: Any = None
        self._command_transport: Any = None
        self._unsubscribe: Any = None
        self._arm_dof = dof
        self._connected = False
        self._enabled = False
        self._control_mode = ControlMode.POSITION

    def connect(self) -> bool:
        self._state_transport = self._transport_cls(f"/{self._prefix}/sim_state", JointState)
        self._command_transport = self._transport_cls(f"/{self._prefix}/sim_command", JointState)
        self._unsubscribe = self._state_transport.subscribe(self._on_state)
        # The sim may still be building its scene; wait for its first state.
        deadline = time.monotonic() + _CONNECT_TIMEOUT_S
        while time.monotonic() < deadline:
            with self._lock:
                state = self._state
            if state is not None:
                count = len(state.position)
                if count not in (self._dof, self._dof - 1):
                    raise ValueError(f"sim reports {count} joints, expected {self._dof}")
                # Hardware dof counts the gripper; a sim without one reports dof - 1.
                self._arm_dof = self._dof - 1 if count == self._dof else self._dof
                self._connected = self._enabled = True
                logger.info("TransportSimAdapter connected", prefix=self._prefix, joints=count)
                return True
            time.sleep(0.1)
        logger.error("No sim_state received", topic=f"/{self._prefix}/sim_state")
        return False

    def disconnect(self) -> None:
        if self._unsubscribe is not None:
            self._unsubscribe()
            self._unsubscribe = None
        for transport in (self._state_transport, self._command_transport):
            if transport is not None:
                transport.stop()
        self._state_transport = self._command_transport = None
        self._connected = False

    def _on_state(self, msg: JointState) -> None:
        with self._lock:
            self._state = msg
            self._state_at = time.monotonic()

    def _latest(self) -> JointState | None:
        with self._lock:
            if self._state is None or time.monotonic() - self._state_at > _STALE_AFTER_S:
                return None
            return self._state

    def is_connected(self) -> bool:
        return self._connected and self._latest() is not None

    def has_live_state(self) -> bool:
        """Gates ConnectedHardware reads and commands on fresh sim state."""
        return self.is_connected()

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
        return self._dof

    def get_limits(self) -> JointLimits:
        """Arm limits in radians, then the gripper's command range."""
        gripper = self._dof - self._arm_dof
        lo, hi = self._gripper_range
        return JointLimits(
            position_lower=[-math.pi] * self._arm_dof + [lo] * gripper,
            position_upper=[math.pi] * self._arm_dof + [hi] * gripper,
            velocity_max=[math.radians(180.0)] * self._arm_dof + [0.0] * gripper,
        )

    def set_control_mode(self, mode: ControlMode) -> bool:
        # The sim's servos take positions only.
        if mode not in (ControlMode.POSITION, ControlMode.SERVO_POSITION):
            return False
        self._control_mode = mode
        return True

    def get_control_mode(self) -> ControlMode:
        return self._control_mode

    def _read(self, field: str) -> list[float]:
        state = self._latest()
        if state is None:
            return [0.0] * self._dof
        values = list(getattr(state, field))
        return values + [0.0] * (self._dof - len(values))

    def read_joint_positions(self) -> list[float]:
        return self._read("position")

    def read_joint_velocities(self) -> list[float]:
        return self._read("velocity")

    def read_joint_efforts(self) -> list[float]:
        return self._read("effort")

    def read_state(self) -> dict[str, int]:
        moving = any(abs(v) > 1e-4 for v in self.read_joint_velocities())
        return {"state": 1 if moving else 0, "mode": list(ControlMode).index(self._control_mode)}

    def read_error(self) -> tuple[int, str]:
        if self._connected and self._latest() is None:
            return 1, "sim_state stopped updating"
        return 0, ""

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        if not self._enabled or self._command_transport is None or len(positions) != self._dof:
            return False
        names = [f"joint{i + 1}" for i in range(self._arm_dof)] + ["gripper"] * (
            self._dof - self._arm_dof
        )
        self._command_transport.publish(JointState(name=names, position=list(positions)))
        return True

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        # The sim's servos take positions only.
        return False

    def write_joint_efforts(self, efforts: list[float]) -> bool:
        return False

    def write_stop(self) -> bool:
        state = self._latest()
        if state is None:
            return False
        return self.write_joint_positions(self._read("position"))

    def write_enable(self, enable: bool) -> bool:
        self._enabled = enable
        return True

    def read_enabled(self) -> bool:
        return self._enabled

    def write_clear_errors(self) -> bool:
        return True

    def read_cartesian_position(self) -> dict[str, float] | None:
        return None

    def write_cartesian_position(self, pose: dict[str, float], velocity: float = 1.0) -> bool:
        return False

    def read_force_torque(self) -> list[float] | None:
        return None
