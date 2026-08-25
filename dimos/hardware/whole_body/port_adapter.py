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

"""Whole-body adapter backed by ports on its owning coordinator module."""

from __future__ import annotations

from collections.abc import Callable
import math
import threading

from dimos.core.stream import In, Out
from dimos.hardware.spec import JointLimits
from dimos.hardware.whole_body.spec import IMUState, MotorCommand, MotorState
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray


class PortWholeBodyAdapter:
    """Bridge synchronous whole-body calls to typed module ports.

    The adapter deliberately has no topic, transport, or discovery knowledge.
    Its owning ``ControlCoordinator`` subclass supplies already-bound ports.
    """

    def __init__(
        self,
        *,
        dof: int,
        motor_states: In[JointState],
        imu: In[Imu],
        motor_command: Out[MotorCommandArray],
        limits: JointLimits | None = None,
    ) -> None:
        self._dof = dof
        self._motor_states_port = motor_states
        self._imu_port = imu
        self._motor_command_port = motor_command
        self._limits = limits
        self._lock = threading.Lock()
        self._latest_motor_states: list[MotorState] | None = None
        self._latest_imu: IMUState | None = None
        self._motor_states_unsubscribe: Callable[[], None] | None = None
        self._imu_unsubscribe: Callable[[], None] | None = None
        self._connected = False

    def connect(self) -> bool:
        if self._connected:
            return True
        self._motor_states_unsubscribe = self._motor_states_port.subscribe(self._on_motor_states)
        self._imu_unsubscribe = self._imu_port.subscribe(self._on_imu)
        self._connected = True
        return True

    def disconnect(self) -> None:
        for unsubscribe in (self._motor_states_unsubscribe, self._imu_unsubscribe):
            if unsubscribe is not None:
                unsubscribe()
        self._motor_states_unsubscribe = None
        self._imu_unsubscribe = None
        with self._lock:
            self._latest_motor_states = None
            self._latest_imu = None
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def read_motor_states(self) -> list[MotorState]:
        with self._lock:
            if self._latest_motor_states is None:
                raise RuntimeError("motor state is not ready")
            return list(self._latest_motor_states)

    def has_motor_states(self) -> bool:
        with self._lock:
            return self._latest_motor_states is not None

    def read_imu(self) -> IMUState:
        with self._lock:
            if self._latest_imu is None:
                raise RuntimeError("IMU state is not ready")
            return self._latest_imu

    def get_limits(self) -> JointLimits | None:
        return self._limits

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        if not self._connected or len(commands) != self._dof:
            return False
        self._motor_command_port.publish(
            MotorCommandArray(
                q=[command.q for command in commands],
                dq=[command.dq for command in commands],
                kp=[command.kp for command in commands],
                kd=[command.kd for command in commands],
                tau=[command.tau for command in commands],
            )
        )
        return True

    def _on_motor_states(self, message: JointState) -> None:
        if (
            len(message.position) != self._dof
            or len(message.velocity) != self._dof
            or len(message.effort) != self._dof
            or not all(
                math.isfinite(value)
                for values in (message.position, message.velocity, message.effort)
                for value in values
            )
        ):
            return
        states = [
            MotorState(
                q=message.position[index],
                dq=message.velocity[index],
                tau=message.effort[index],
            )
            for index in range(self._dof)
        ]
        with self._lock:
            self._latest_motor_states = states

    def _on_imu(self, message: Imu) -> None:
        state = IMUState(
            quaternion=(
                message.orientation.w,
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
            ),
            gyroscope=(
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ),
            accelerometer=(
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ),
        )
        with self._lock:
            self._latest_imu = state
