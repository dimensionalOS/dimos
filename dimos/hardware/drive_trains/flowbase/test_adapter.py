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

from __future__ import annotations

from collections.abc import Generator
from dataclasses import dataclass, field
import math
import sys
from types import ModuleType
from typing import Any

import numpy as np
import pytest

from dimos.hardware.drive_trains.flowbase.adapter import FlowBaseAdapter
from dimos.msgs.nav_msgs.Odometry import Odometry


class ManualClock:
    def __init__(self, initial: float = 10.0) -> None:
        self.now = initial

    def __call__(self) -> float:
        return self.now


class FakeFuture:
    def __init__(self, value: Any) -> None:
        self._value = value

    def result(self) -> Any:
        return self._value


@dataclass
class FakePortalClient:
    odometry: list[dict[str, Any] | None] = field(default_factory=list)
    velocity_commands: list[dict[str, Any]] = field(default_factory=list)
    closed: bool = False

    def get_odometry(self, request: dict[str, Any]) -> FakeFuture:
        assert request == {}
        return FakeFuture(self.odometry.pop(0))

    def set_target_velocity(self, command: dict[str, Any]) -> FakeFuture:
        self.velocity_commands.append(command)
        return FakeFuture(None)

    def close(self) -> None:
        self.closed = True


class FakeTransport:
    instances: list[FakeTransport] = []

    def __init__(self, topic: str, message_type: type) -> None:
        self.topic = topic
        self.message_type = message_type
        self.published: list[Odometry] = []
        self.stopped = False
        self.instances.append(self)

    def publish(self, message: Odometry) -> None:
        self.published.append(message)

    def stop(self) -> None:
        self.stopped = True


@dataclass
class AdapterHarness:
    adapter: FlowBaseAdapter
    client: FakePortalClient
    transport: FakeTransport
    clock: ManualClock


@pytest.fixture
def harness(monkeypatch: pytest.MonkeyPatch) -> Generator[AdapterHarness, None, None]:
    client = FakePortalClient()
    portal = ModuleType("portal")
    portal.Client = lambda address: client  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "portal", portal)
    FakeTransport.instances.clear()
    clock = ManualClock()
    adapter = FlowBaseAdapter(
        transport_cls=FakeTransport,
        clock=clock,
        monotonic_clock=clock,
    )
    assert adapter.connect()
    transport = FakeTransport.instances[0]
    try:
        yield AdapterHarness(adapter, client, transport, clock)
    finally:
        adapter.disconnect()


def test_read_odometry_publishes_ros_frame_pose(harness: AdapterHarness) -> None:
    harness.client.odometry.append({"translation": [1.25, -0.4], "rotation": -0.3})
    assert harness.adapter.write_velocities([0.2, -0.1, 0.05])

    result = harness.adapter.read_odometry()

    assert result == pytest.approx([0.2, -0.1, 0.05])
    assert harness.transport.topic == "/wheel_odometry"
    assert harness.transport.message_type is Odometry
    assert len(harness.transport.published) == 1
    message = harness.transport.published[0]
    assert message.frame_id == "wheel_odom"
    assert message.child_frame_id == "base_link"
    assert [message.x, message.y, message.yaw] == pytest.approx([1.25, 0.4, 0.3])
    assert [message.vx, message.vy, message.wz] == pytest.approx([0.0, 0.0, 0.0])


def test_read_odometry_derives_body_frame_twist(harness: AdapterHarness) -> None:
    harness.client.odometry.extend(
        [
            {"translation": [0.0, 0.0], "rotation": 0.0},
            {"translation": [2.0, 0.0], "rotation": -math.pi / 2.0},
        ]
    )
    assert harness.adapter.read_odometry() == pytest.approx([0.0, 0.0, 0.0])

    harness.clock.now += 2.0
    assert harness.adapter.read_odometry() == pytest.approx([0.0, 0.0, 0.0])

    message = harness.transport.published[-1]
    assert [message.vx, message.vy, message.wz] == pytest.approx(
        [0.0, -1.0, math.pi / 4.0], abs=1e-12
    )


def test_velocity_commands_keep_existing_virtual_joint_convention(
    harness: AdapterHarness,
) -> None:
    assert harness.adapter.write_velocities([0.1, 0.2, 0.3])

    command = harness.client.velocity_commands[-1]
    np.testing.assert_allclose(command["target_velocity"], [0.1, -0.2, -0.3])
    assert command["frame"] == "local"


def test_missing_odometry_preserves_virtual_joint_values(harness: AdapterHarness) -> None:
    assert harness.adapter.write_velocities([0.3, -0.2, 0.1])
    harness.client.odometry.append(None)

    assert harness.adapter.read_odometry() == pytest.approx([0.3, -0.2, 0.1])
    assert harness.transport.published == []


def test_disconnect_stops_odometry_transport(harness: AdapterHarness) -> None:
    assert harness.adapter.write_enable(True)
    assert harness.adapter.write_velocities([0.1, 0.2, 0.3])
    harness.adapter.disconnect()

    assert harness.transport.stopped
    assert harness.client.closed
    assert harness.adapter.read_velocities() == [0.0, 0.0, 0.0]
    assert not harness.adapter.read_enabled()
