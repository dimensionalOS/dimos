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

from collections.abc import Callable
from typing import Any

import pytest

from dimos.core.stream import In, Out, Stream, Transport
from dimos.hardware.whole_body.port_adapter import PortWholeBodyAdapter
from dimos.hardware.whole_body.spec import MotorCommand, MotorState
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray


class _MemoryTransport(Transport[Any]):
    def __init__(self) -> None:
        self.callbacks: list[Callable[[Any], Any]] = []
        self.published: list[Any] = []

    def broadcast(self, selfstream: Stream[Any] | None, value: Any) -> None:
        self.published.append(value)
        for callback in list(self.callbacks):
            callback(value)

    def subscribe(
        self,
        callback: Callable[[Any], Any],
        selfstream: Stream[Any] | None = None,
    ) -> Callable[[], None]:
        self.callbacks.append(callback)

        def unsubscribe() -> None:
            self.callbacks.remove(callback)

        return unsubscribe

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


def _adapter() -> tuple[
    PortWholeBodyAdapter,
    _MemoryTransport,
    _MemoryTransport,
    _MemoryTransport,
]:
    states_transport = _MemoryTransport()
    imu_transport = _MemoryTransport()
    command_transport = _MemoryTransport()
    adapter = PortWholeBodyAdapter(
        dof=2,
        motor_states=In(JointState, "motor_states", transport=states_transport),
        imu=In(Imu, "imu", transport=imu_transport),
        motor_command=Out(
            MotorCommandArray,
            "motor_command",
            transport=command_transport,
        ),
    )
    return adapter, states_transport, imu_transport, command_transport


def test_connect_is_async_and_state_becomes_ready_after_complete_frame() -> None:
    adapter, states_transport, _, _ = _adapter()

    assert adapter.connect()
    assert not adapter.has_motor_states()
    with pytest.raises(RuntimeError, match="not ready"):
        adapter.read_motor_states()

    states_transport.publish(
        JointState(position=[1.0, 2.0], velocity=[3.0, 4.0], effort=[5.0, 6.0])
    )

    assert adapter.has_motor_states()
    assert adapter.read_motor_states() == [
        MotorState(q=1.0, dq=3.0, tau=5.0),
        MotorState(q=2.0, dq=4.0, tau=6.0),
    ]


@pytest.mark.parametrize(
    "invalid_state",
    [
        JointState(position=[9.0], velocity=[9.0], effort=[9.0]),
        JointState(position=[9.0, float("nan")], velocity=[9.0, 9.0], effort=[9.0, 9.0]),
    ],
)
def test_invalid_state_is_dropped_without_replacing_latest_snapshot(
    invalid_state: JointState,
) -> None:
    adapter, states_transport, _, _ = _adapter()
    adapter.connect()
    states_transport.publish(
        JointState(position=[1.0, 2.0], velocity=[3.0, 4.0], effort=[5.0, 6.0])
    )

    states_transport.publish(invalid_state)

    assert adapter.read_motor_states() == [
        MotorState(q=1.0, dq=3.0, tau=5.0),
        MotorState(q=2.0, dq=4.0, tau=6.0),
    ]


def test_adapter_converts_imu_and_publishes_complete_motor_command() -> None:
    adapter, _, imu_transport, command_transport = _adapter()
    adapter.connect()
    with pytest.raises(RuntimeError, match="not ready"):
        adapter.read_imu()
    imu_transport.publish(
        Imu(
            orientation=Quaternion(1.0, 2.0, 3.0, 4.0),
            angular_velocity=Vector3(5.0, 6.0, 7.0),
            linear_acceleration=Vector3(8.0, 9.0, 10.0),
        )
    )

    assert adapter.read_imu().quaternion == (4.0, 1.0, 2.0, 3.0)
    assert adapter.write_motor_commands(
        [
            MotorCommand(q=1.0, dq=2.0, kp=3.0, kd=4.0, tau=5.0),
            MotorCommand(q=6.0, dq=7.0, kp=8.0, kd=9.0, tau=10.0),
        ]
    )
    message = command_transport.published[-1]
    assert isinstance(message, MotorCommandArray)
    assert message.q == [1.0, 6.0]
    assert message.dq == [2.0, 7.0]
    assert message.kp == [3.0, 8.0]
    assert message.kd == [4.0, 9.0]
    assert message.tau == [5.0, 10.0]


def test_disconnect_clears_readiness_and_rejects_commands() -> None:
    adapter, states_transport, _, _ = _adapter()
    adapter.connect()
    states_transport.publish(
        JointState(position=[1.0, 2.0], velocity=[3.0, 4.0], effort=[5.0, 6.0])
    )

    adapter.disconnect()

    assert not adapter.is_connected()
    assert not adapter.has_motor_states()
    assert not adapter.write_motor_commands([MotorCommand(), MotorCommand()])
