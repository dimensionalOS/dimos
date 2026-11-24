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

"""Tests for ``FlowBaseConnection`` against a fake Portal client, with no base attached.

The fake stands in for ``portal.Client`` and records the speeds it is sent.
Every port gets an in-process transport, so nothing opens a socket.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
import math
from typing import Any

import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.flowbase import connection
from dimos.robot.flowbase.connection import (
    FLOWBASE_BASE_JOINTS,
    FlowBaseConnection,
    _body_speeds,
)
from dimos.utils.testing.waiting import wait_until

# Long enough that the deadman never halts mid-test, except where a test sets its own.
DEADMAN_S = 5.0


class Answer:
    """A Portal future that is already answered."""

    def __init__(self, value: Any) -> None:
        self.value = value

    def result(self, timeout: float | None = None) -> Any:
        return self.value


class FakePortalClient:
    """A FlowBase controller at rest, at ``odometry`` (its own frame: x m, y m, yaw rad).

    ``speeds`` keeps each ``target_velocity`` sent, as [vx, vy, wz].
    """

    def __init__(self) -> None:
        self.odometry = (0.0, 0.0, 0.0)
        self.speeds: list[list[float]] = []
        self.closed = False

    def connect(self, timeout: float | None = None) -> bool:
        return True

    def close(self, timeout: float | None = None) -> None:
        self.closed = True

    def set_target_velocity(self, command: dict[str, Any]) -> Answer:
        assert command["frame"] == "local"
        self.speeds.append([float(v) for v in command["target_velocity"]])
        return Answer(None)

    def get_odometry(self, _: dict[str, Any]) -> Answer:
        x, y, yaw = self.odometry
        return Answer({"translation": [x, y], "rotation": yaw})


class LocalTransport:
    """An in-process transport: publishing calls every subscriber at once."""

    def __init__(self) -> None:
        self.sent: list[Any] = []
        self.subscribers: list[Callable[[Any], Any]] = []

    def broadcast(self, selfstream: Any, value: Any) -> None:
        self.sent.append(value)
        for cb in list(self.subscribers):
            cb(value)

    def publish(self, value: Any) -> None:
        self.broadcast(None, value)

    def subscribe(self, cb: Callable[[Any], Any], selfstream: Any = None) -> Callable[[], None]:
        self.subscribers.append(cb)
        return lambda: self.subscribers.remove(cb)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


class Rig:
    """One started ``FlowBaseConnection`` on a fake client, with a transport on every port."""

    def __init__(self, module: FlowBaseConnection, client: FakePortalClient) -> None:
        self.module = module
        self.client = client
        self.ports = {name: LocalTransport() for name in {**module.inputs, **module.outputs}}
        for name, transport in self.ports.items():
            module.set_transport(name, transport)  # type: ignore[arg-type]

    def drive(self, forward: float, left: float, turn: float) -> None:
        """Send body speeds (m/s, m/s, rad/s) on ``velocity_command``."""
        self.ports["velocity_command"].publish(
            JointState(name=list(FLOWBASE_BASE_JOINTS), velocity=[forward, left, turn])
        )


@pytest.fixture
def rig(monkeypatch: pytest.MonkeyPatch) -> Iterator[Callable[..., Rig]]:
    """Build a ``FlowBaseConnection`` on a fake client, start it, and stop it at the end."""
    made: list[Rig] = []

    def build(**config: Any) -> Rig:
        client = FakePortalClient()
        monkeypatch.setattr(connection, "_open_client", lambda address: client)
        config.setdefault("deadman_timeout_s", DEADMAN_S)
        made.append(Rig(FlowBaseConnection(**config), client))
        made[-1].module.start()
        return made[-1]

    yield build
    for each in made:
        each.module.stop()


def test_speeds_go_out_with_lateral_and_turn_flipped(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.drive(0.5, 0.2, 1.0)
    assert r.client.speeds == [[0.5, -0.2, -1.0]]


def test_a_speed_over_the_limit_is_refused(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.drive(0.8, -0.8, 3.0)
    r.drive(0.81, 0.0, 0.0)
    r.drive(0.0, 0.0, -3.01)
    # Only the command at the limits went out.
    assert [s for s in r.client.speeds if any(s)] == [[0.8, 0.8, -3.0]]


def test_no_command_within_the_deadman_sends_zero_speed(rig: Callable[..., Rig]) -> None:
    r = rig(deadman_timeout_s=0.05)
    r.drive(0.5, 0.0, 0.0)
    wait_until(lambda: r.module.status().deadman_fired, timeout=2.0)
    assert r.client.speeds[-1] == [0.0, 0.0, 0.0]


def test_a_twist_on_cmd_vel_goes_out_flipped_and_the_deadman_halts_it(
    rig: Callable[..., Rig],
) -> None:
    r = rig(deadman_timeout_s=0.05)
    r.ports["cmd_vel"].publish(Twist(linear=Vector3(0.5, 0.2, 0.0), angular=Vector3(0.0, 0.0, 1.0)))
    wait_until(lambda: r.module.status().deadman_fired, timeout=2.0)
    assert r.client.speeds == [[0.5, -0.2, -1.0], [0.0, 0.0, 0.0]]


def test_odometry_and_body_speeds_come_out_in_ros_frame(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.client.odometry = (1.0, 2.0, 0.5)

    def odometry() -> list[Odometry]:
        return [m for m in r.ports["odometry"].sent if m.position.x == 1.0]

    wait_until(lambda: len(odometry()) >= 2, timeout=2.0)
    msg = odometry()[-1]
    assert (msg.position.y, msg.yaw) == pytest.approx((-2.0, -0.5))
    assert (msg.frame_id, msg.child_frame_id) == ("odom", "base_link")
    reading: JointState = r.ports["joint_state"].sent[-1]
    assert reading.name == list(FLOWBASE_BASE_JOINTS)
    assert reading.velocity == pytest.approx([0.0, 0.0, 0.0])


def test_stop_sends_zero_speed_and_closes_the_client(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.drive(0.3, 0.0, 0.0)
    r.module.stop()
    assert r.client.speeds[-1] == [0.0, 0.0, 0.0]
    assert r.client.closed


@pytest.mark.parametrize(
    ("start", "end", "speeds"),
    [
        # Facing left (+y), moving along +y: that is forward.
        ((0.0, 0.0, math.pi / 2), (0.0, 0.1, math.pi / 2), (1.0, 0.0, 0.0)),
        # Facing left, moving along +x: that is to the right.
        ((0.0, 0.0, math.pi / 2), (0.1, 0.0, math.pi / 2), (0.0, -1.0, 0.0)),
        # Turning left on the spot.
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.2), (0.0, 0.0, 2.0)),
        # Turning left across the +pi / -pi boundary.
        ((0.0, 0.0, math.pi - 0.1), (0.0, 0.0, -math.pi + 0.1), (0.0, 0.0, 2.0)),
    ],
)
def test_body_speeds_between_two_poses(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    speeds: tuple[float, float, float],
) -> None:
    assert _body_speeds(start, end, 0.1) == pytest.approx(speeds, abs=1e-9)
