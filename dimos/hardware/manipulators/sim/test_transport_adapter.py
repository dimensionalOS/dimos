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

from dimos.hardware.manipulators.sim.transport_adapter import TransportSimAdapter
from dimos.hardware.manipulators.spec import ControlMode
from dimos.msgs.sensor_msgs.JointState import JointState


class FakeTransport:
    """In-memory pub/sub keyed by topic; a sim publishes state into ``topics``."""

    topics: dict[str, "FakeTransport"] = {}

    def __init__(self, topic: str, msg_type: type) -> None:
        self.topic = topic
        self.published: list[Any] = []
        self.callbacks: list[Callable[[Any], None]] = []
        FakeTransport.topics[topic] = self

    def subscribe(self, callback: Callable[[Any], None]) -> Callable[[], None]:
        self.callbacks.append(callback)
        return lambda: self.callbacks.remove(callback)

    def publish(self, msg: Any) -> None:
        self.published.append(msg)
        for callback in self.callbacks:
            callback(msg)

    def stop(self) -> None:
        pass


def _state(positions: list[float]) -> JointState:
    return JointState(
        name=[f"joint{i + 1}" for i in range(7)] + ["gripper"],
        position=positions,
        velocity=[0.0] * len(positions),
        effort=[0.0] * len(positions),
    )


@pytest.fixture
def adapter(monkeypatch: pytest.MonkeyPatch) -> TransportSimAdapter:
    FakeTransport.topics = {}
    adapter = TransportSimAdapter(
        dof=8, hardware_id="arm", gripper_range=(0.0, 0.08), transport_cls=FakeTransport
    )
    real_subscribe = FakeTransport.subscribe

    # The sim answers as soon as the adapter subscribes to its state.
    def subscribe_and_publish(self: FakeTransport, callback: Callable[[Any], None]) -> Any:
        unsubscribe = real_subscribe(self, callback)
        if self.topic == "/arm/sim_state":
            self.publish(_state([0.1] * 7 + [0.08]))
        return unsubscribe

    monkeypatch.setattr(FakeTransport, "subscribe", subscribe_and_publish)
    assert adapter.connect()
    return adapter


def test_reads_arm_then_gripper_from_sim_state(adapter: TransportSimAdapter) -> None:
    assert adapter.read_joint_positions() == pytest.approx([0.1] * 7 + [0.08])
    limits = adapter.get_limits()
    assert (limits.position_lower[-1], limits.position_upper[-1]) == (0.0, 0.08)


def test_position_commands_go_to_sim_command(adapter: TransportSimAdapter) -> None:
    assert adapter.set_control_mode(ControlMode.SERVO_POSITION)
    assert adapter.write_joint_positions([0.2] * 7 + [0.0])
    (command,) = FakeTransport.topics["/arm/sim_command"].published
    assert list(command.position) == pytest.approx([0.2] * 7 + [0.0])
    assert command.name[-1] == "gripper"


def test_rejects_wrong_length_and_non_position_modes(adapter: TransportSimAdapter) -> None:
    assert not adapter.write_joint_positions([0.0] * 7)
    assert not adapter.set_control_mode(ControlMode.VELOCITY)
    assert not adapter.write_joint_velocities([0.0] * 8)
