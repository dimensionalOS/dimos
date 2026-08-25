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

"""Manipulator adapter backed by ports on its owning coordinator module."""

from __future__ import annotations

from collections.abc import Callable
import math
import threading

from dimos.core.stream import In, Out
from dimos.hardware.manipulators.spec import ControlMode, ManipulatorInfo
from dimos.hardware.spec import JointLimits
from dimos.hardware.whole_body.spec import POS_STOP, VEL_STOP
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray


class PortManipulatorAdapter:
    """Bridge the synchronous manipulator API to typed module ports."""

    def __init__(
        self,
        *,
        dof: int,
        motor_states: In[JointState],
        motor_command: Out[MotorCommandArray],
        limits: JointLimits | None = None,
    ) -> None:
        self._dof = dof
        self._motor_states_port = motor_states
        self._motor_command_port = motor_command
        self._limits = limits or JointLimits(
            position_lower=[-math.pi] * dof,
            position_upper=[math.pi] * dof,
            velocity_max=[math.pi] * dof,
        )
        self._lock = threading.Lock()
        self._latest_state: JointState | None = None
        self._unsubscribe: Callable[[], None] | None = None
        self._connected = False
        self._enabled = False
        self._control_mode = ControlMode.POSITION

    def connect(self) -> bool:
        if self._connected:
            return True
        self._unsubscribe = self._motor_states_port.subscribe(self._on_motor_states)
        self._connected = True
        return True

    def disconnect(self) -> None:
        if self._unsubscribe is not None:
            self._unsubscribe()
            self._unsubscribe = None
        with self._lock:
            self._latest_state = None
        self._enabled = False
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def has_joint_state(self) -> bool:
        with self._lock:
            return self._latest_state is not None

    def activate(self) -> bool:
        return self.write_enable(True)

    def deactivate(self) -> bool:
        if self.has_joint_state():
            self.write_stop()
        return self.write_enable(False)

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(
            vendor="Simulation",
            model="MuJoCo",
            dof=self._dof,
            firmware_version=None,
            serial_number=None,
        )

    def get_dof(self) -> int:
        return self._dof

    def get_limits(self) -> JointLimits:
        return self._limits

    def set_control_mode(self, mode: ControlMode) -> bool:
        if mode not in (
            ControlMode.POSITION,
            ControlMode.SERVO_POSITION,
            ControlMode.VELOCITY,
            ControlMode.TORQUE,
        ):
            return False
        self._control_mode = mode
        return True

    def get_control_mode(self) -> ControlMode:
        return self._control_mode

    def read_joint_positions(self) -> list[float]:
        return list(self._state().position)

    def read_joint_velocities(self) -> list[float]:
        return list(self._state().velocity)

    def read_joint_efforts(self) -> list[float]:
        return list(self._state().effort)

    def read_state(self) -> dict[str, int]:
        moving = any(abs(value) > 1e-4 for value in self.read_joint_velocities())
        return {
            "state": 1 if moving else 0,
            "mode": list(ControlMode).index(self._control_mode),
        }

    def read_error(self) -> tuple[int, str]:
        return (0, "")

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        if not self._can_write(positions):
            return False
        return self._publish(
            q=positions,
            dq=[VEL_STOP] * self._dof,
        )

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        if not self._can_write(velocities):
            return False
        return self._publish(
            q=[POS_STOP] * self._dof,
            dq=velocities,
        )

    def write_joint_efforts(self, efforts: list[float]) -> bool:
        if not self._can_write(efforts):
            return False
        return self._publish(
            q=[POS_STOP] * self._dof,
            dq=[VEL_STOP] * self._dof,
            tau=efforts,
        )

    def write_stop(self) -> bool:
        if not self.has_joint_state():
            return False
        return self.write_joint_positions(self.read_joint_positions())

    def write_enable(self, enable: bool) -> bool:
        if not self._connected:
            return False
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

    def _state(self) -> JointState:
        with self._lock:
            if self._latest_state is None:
                raise RuntimeError("joint state is not ready")
            return JointState(self._latest_state)

    def _can_write(self, values: list[float]) -> bool:
        return (
            self._connected
            and self._enabled
            and len(values) == self._dof
            and all(math.isfinite(value) for value in values)
        )

    def _publish(
        self,
        *,
        q: list[float],
        dq: list[float],
        tau: list[float] | None = None,
    ) -> bool:
        self._motor_command_port.publish(MotorCommandArray(q=q, dq=dq, tau=tau))
        return True

    def _on_motor_states(self, message: JointState) -> None:
        if (
            len(message.position) != self._dof
            or len(message.velocity) != self._dof
            or len(message.effort) != self._dof
        ):
            return
        values = (*message.position, *message.velocity, *message.effort)
        if not all(math.isfinite(value) for value in values):
            return
        with self._lock:
            self._latest_state = JointState(message)
