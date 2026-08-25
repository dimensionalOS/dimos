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
from dimos.hardware.manipulators.port_adapter import PortManipulatorAdapter
from dimos.hardware.whole_body.spec import POS_STOP, VEL_STOP
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


def _adapter() -> tuple[PortManipulatorAdapter, _MemoryTransport, _MemoryTransport]:
    states_transport = _MemoryTransport()
    command_transport = _MemoryTransport()
    adapter = PortManipulatorAdapter(
        dof=2,
        motor_states=In(JointState, "motor_states", transport=states_transport),
        motor_command=Out(
            MotorCommandArray,
            "motor_command",
            transport=command_transport,
        ),
    )
    return adapter, states_transport, command_transport


def test_state_is_unavailable_until_first_complete_observation() -> None:
    adapter, states_transport, _ = _adapter()
    assert adapter.connect()

    with pytest.raises(RuntimeError, match="not ready"):
        adapter.read_joint_positions()
    states_transport.publish(
        JointState(position=[1.0, 2.0], velocity=[3.0, 4.0], effort=[5.0, 6.0])
    )

    assert adapter.has_joint_state()
    assert adapter.read_joint_positions() == [1.0, 2.0]
    assert adapter.read_joint_velocities() == [3.0, 4.0]
    assert adapter.read_joint_efforts() == [5.0, 6.0]


def test_position_velocity_and_effort_writes_use_one_motor_message_contract() -> None:
    adapter, _, command_transport = _adapter()
    adapter.connect()
    adapter.activate()

    assert adapter.write_joint_positions([1.0, 2.0])
    assert adapter.write_joint_velocities([3.0, 4.0])
    assert adapter.write_joint_efforts([5.0, 6.0])

    position, velocity, effort = command_transport.published
    assert position.q == [1.0, 2.0]
    assert position.dq == [VEL_STOP, VEL_STOP]
    assert velocity.q == [POS_STOP, POS_STOP]
    assert velocity.dq == [3.0, 4.0]
    assert effort.q == [POS_STOP, POS_STOP]
    assert effort.dq == [VEL_STOP, VEL_STOP]
    assert effort.tau == [5.0, 6.0]


def test_stop_holds_latest_observed_position() -> None:
    adapter, states_transport, command_transport = _adapter()
    adapter.connect()
    adapter.activate()
    states_transport.publish(
        JointState(position=[1.0, 2.0], velocity=[3.0, 4.0], effort=[5.0, 6.0])
    )

    assert adapter.write_stop()

    assert command_transport.published[-1].q == [1.0, 2.0]
