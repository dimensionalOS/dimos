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

"""Transport-based whole-body adapter: bridges coordinator ↔ Module via pub/sub.

Subscribes /{hardware_id}/motor_states + /{hardware_id}/imu, publishes
/{hardware_id}/motor_command. ``network_interface`` is accepted but ignored.
"""

from __future__ import annotations

from functools import partial
import threading
from typing import TYPE_CHECKING, Any

from dimos.core.transport import LCMTransport
from dimos.hardware.whole_body.spec import IMUState, MotorCommand, MotorState
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.hardware.whole_body.registry import WholeBodyAdapterRegistry

logger = setup_logger()


class TransportWholeBodyAdapter:
    """WholeBodyAdapter that bridges to a robot-side Module via pub/sub."""

    def __init__(
        self,
        dof: int = 29,
        hardware_id: str = "wholebody",
        transport_cls: type = LCMTransport,
        network_interface: int | str = "",  # accepted-and-ignored — see module docstring
        **_: object,
    ) -> None:
        self._dof = dof
        self._prefix = hardware_id
        self._transport_cls = transport_cls

        self._lock = threading.Lock()
        self._latest_motor_states: list[MotorState] | None = None
        self._latest_imu: IMUState | None = None

        self._motor_states_transport: Any = None
        self._imu_transport: Any = None
        self._motor_command_transport: Any = None
        self._motor_states_unsub: Any = None
        self._imu_unsub: Any = None
        self._connected = False

    def connect(self) -> bool:
        ms_topic = f"/{self._prefix}/motor_states"
        imu_topic = f"/{self._prefix}/imu"
        cmd_topic = f"/{self._prefix}/motor_command"

        self._motor_states_transport = self._transport_cls(ms_topic, JointState)
        self._imu_transport = self._transport_cls(imu_topic, Imu)
        self._motor_command_transport = self._transport_cls(cmd_topic, MotorCommandArray)

        self._motor_states_unsub = self._motor_states_transport.subscribe(self._on_motor_states)
        self._imu_unsub = self._imu_transport.subscribe(self._on_imu)

        self._connected = True
        logger.info(
            f"TransportWholeBodyAdapter connected: motor_states={ms_topic}, "
            f"imu={imu_topic}, motor_command={cmd_topic}"
        )
        return True

    def disconnect(self) -> None:
        if self._motor_states_unsub is not None:
            self._motor_states_unsub()
            self._motor_states_unsub = None
        if self._imu_unsub is not None:
            self._imu_unsub()
            self._imu_unsub = None

        for t in (
            self._motor_states_transport,
            self._imu_transport,
            self._motor_command_transport,
        ):
            if t is not None:
                t.stop()
        self._motor_states_transport = None
        self._imu_transport = None
        self._motor_command_transport = None

        with self._lock:
            self._latest_motor_states = None
            self._latest_imu = None

        self._connected = False
        logger.info("TransportWholeBodyAdapter disconnected")

    def is_connected(self) -> bool:
        return self._connected

    def read_motor_states(self) -> list[MotorState]:
        with self._lock:
            if self._latest_motor_states is None:
                return [MotorState() for _ in range(self._dof)]
            return list(self._latest_motor_states)

    def has_motor_states(self) -> bool:
        """True once the first motor_states frame has been received."""
        with self._lock:
            return self._latest_motor_states is not None

    def read_imu(self) -> IMUState:
        with self._lock:
            if self._latest_imu is None:
                return IMUState()
            return self._latest_imu

    def write_motor_commands(self, commands: list[MotorCommand]) -> bool:
        if self._motor_command_transport is None:
            logger.warning("write_motor_commands called before connect()")
            return False

        msg = MotorCommandArray(
            q=[c.q for c in commands],
            dq=[c.dq for c in commands],
            kp=[c.kp for c in commands],
            kd=[c.kd for c in commands],
            tau=[c.tau for c in commands],
        )
        self._motor_command_transport.publish(msg)
        return True

    def _on_motor_states(self, msg: JointState) -> None:
        # Drop short frames; downstream code indexes range(_dof) directly.
        if (
            len(msg.position) < self._dof
            or len(msg.velocity) < self._dof
            or len(msg.effort) < self._dof
        ):
            return
        states = [
            MotorState(q=msg.position[i], dq=msg.velocity[i], tau=msg.effort[i])
            for i in range(self._dof)
        ]
        with self._lock:
            self._latest_motor_states = states

    def _on_imu(self, msg: Imu) -> None:
        # dimos Imu Quaternion is (x,y,z,w); IMUState.quaternion is (w,x,y,z).
        with self._lock:
            self._latest_imu = IMUState(
                quaternion=(
                    msg.orientation.w,
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                ),
                gyroscope=(
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ),
                accelerometer=(
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ),
                rpy=(0.0, 0.0, 0.0),
            )


def register(registry: WholeBodyAdapterRegistry) -> None:
    """Auto-discovered by ``whole_body_adapter_registry.discover()``."""
    from dimos.core.transport import ROSTransport

    registry.register(
        "transport_lcm",
        partial(TransportWholeBodyAdapter, transport_cls=LCMTransport),
    )
    registry.register(
        "transport_ros",
        partial(TransportWholeBodyAdapter, transport_cls=ROSTransport),
    )


__all__ = ["TransportWholeBodyAdapter"]
