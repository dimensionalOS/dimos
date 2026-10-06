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

"""Tests for ``ConnectionModule``, driven through ``MockConnectionModule``.

Real modules with real threads, no coordinator. Every port is given an
in-process transport, so nothing touches the network.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from dataclasses import replace
import math
import pickle
import threading
import time
from typing import Any

import pytest

from dimos.control.connection.connection_module import ConnectionDescription, ConnectionStatus
from dimos.control.connection.mock_connection import MockConnectionModule
from dimos.control.contract.description import ControlDescription, Limits
from dimos.control.contract.keys import POSITION, VELOCITY, VX, VY, WZ, Key
from dimos.control.contract.presets import (
    imu_resource,
    manipulator_description,
    pd_joint_description,
    twist_base_description,
)
from dimos.control.contract.validate import DescriptionError
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray

JOINT1 = "mock/joint1/position"
DEADMAN_S = 0.05


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
    """One module, with an in-process transport on every port."""

    def __init__(self, module: MockConnectionModule) -> None:
        self.module = module
        self.ports = {name: LocalTransport() for name in {**module.inputs, **module.outputs}}
        for name, transport in self.ports.items():
            module.set_transport(name, transport)  # type: ignore[arg-type]

    def send(self, port: str, msg: Any) -> None:
        self.ports[port].publish(msg)

    def move(self, **positions: float) -> None:
        """Send joint positions on ``position_command``, e.g. ``move(joint1=0.5)``."""
        names = [f"mock/{joint}" for joint in positions]
        self.send("position_command", JointState(name=names, position=list(positions.values())))

    def sent(self, port: str) -> list[Any]:
        return list(self.ports[port].sent)


@pytest.fixture
def rig() -> Iterator[Callable[..., Rig]]:
    """Build modules, start them, and stop every one at the end of the test."""
    made: list[Rig] = []

    def build(cls: type[MockConnectionModule] = MockConnectionModule, **config: Any) -> Rig:
        config.setdefault("deadman_timeout_s", DEADMAN_S)
        made.append(Rig(cls(**config)))
        made[-1].module.start()
        return made[-1]

    yield build
    for each in made:
        each.module.stop()


def wait_until(condition: Callable[[], bool], timeout_s: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while not condition():
        if time.monotonic() > deadline:
            return False
        time.sleep(0.005)
    return True


def writes(r: Rig) -> list[dict[str, float]]:
    return list(r.module.writes)


class BodyAndBase(MockConnectionModule):
    """Two robots: a two-joint PD body "g1" with an IMU, and a base "base"."""

    motor_command: In[MotorCommandArray]
    base_command: In[Twist]
    odom: Out[PoseStamped]
    imu: Out[Imu]

    def describe(self) -> list[ControlDescription]:
        limits = {Key.of("base", "base", axis): Limits(-1.0, 1.0) for axis in (VX, VY, WZ)}
        return [
            pd_joint_description("g1", ["j1", "j2"], sensors=[imu_resource()]),
            twist_base_description("base", limits=limits, measured_velocity=False),
        ]


class PushedBodyAndBase(BodyAndBase):
    """Never polled: readings only go out when the test publishes them."""

    def read_state(self) -> dict[str, float] | None:
        return None


class PositionAndVelocity(MockConnectionModule):
    velocity_command: In[JointState]

    def describe(self) -> ControlDescription:
        return manipulator_description(
            "mock", ["joint1"], state=(POSITION,), command=(POSITION, VELOCITY)
        )


class SlowWrite(MockConnectionModule):
    """Its first write waits until the test lets it go."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.inside = threading.Event()
        self.release = threading.Event()

    def write(self, values: dict[str, float]) -> None:
        self.inside.set()
        self.release.wait(5.0)
        super().write(values)


class FailingWrite(MockConnectionModule):
    def write(self, values: dict[str, float]) -> None:
        raise RuntimeError("motor said no")


class Watched(MockConnectionModule):
    """Remembers whether ``connect`` was called."""

    connected = False

    def connect(self) -> None:
        self.connected = True
        super().connect()


class BadDescription(Watched):
    def describe(self) -> ControlDescription:
        arm = super().describe()
        assert isinstance(arm, ControlDescription)
        return replace(arm, state_rate_hz=0.0)


class NoVelocityInput(Watched):
    def describe(self) -> ControlDescription:
        return manipulator_description("mock", ["joint1"], state=(POSITION,))


class NoImuOutput(Watched):
    def describe(self) -> ControlDescription:
        return manipulator_description(
            "mock", ["joint1"], state=(POSITION,), command=(POSITION,), sensors=[imu_resource()]
        )


class WrongPortType(Watched):
    position_command: In[Twist]  # type: ignore[assignment]


@pytest.mark.parametrize(
    ("cls", "error", "match"),
    [
        (BadDescription, DescriptionError, "state_rate_hz"),
        (NoVelocityInput, ValueError, r"no declared input port carries \['mock/joint1/velocity'\]"),
        (NoImuOutput, ValueError, "no declared output port carries"),
        (WrongPortType, TypeError, "position_command must carry JointState"),
    ],
)
def test_start_fails_before_connecting_on_a_bad_description_or_ports(
    cls: type[Watched], error: type[Exception], match: str
) -> None:
    module = cls()
    try:
        with pytest.raises(error, match=match):
            module.start()
        assert not module.connected
    finally:
        module.stop()


def test_readings_go_out_as_joint_states_at_about_the_state_rate(
    rig: Callable[..., Rig],
) -> None:
    started = time.monotonic()
    r = rig(state_rate_hz=100.0)
    assert wait_until(lambda: len(r.sent("joint_state")) >= 20)
    # 20 readings at 100 Hz take at least 19 periods.
    assert time.monotonic() - started >= 0.18
    msg = r.sent("joint_state")[0]
    assert isinstance(msg, JointState)
    assert msg.name == [f"mock/joint{i}" for i in range(1, 8)]
    assert msg.position == [0.0] * 7
    assert msg.velocity == [] and msg.effort == []


def test_a_position_command_is_written_and_the_readings_follow(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.move(joint1=0.5)
    assert wait_until(lambda: writes(r) == [{JOINT1: 0.5}])
    assert wait_until(lambda: any(m.position[0] == 0.5 for m in r.sent("joint_state")))
    assert r.module.status().last_command_time is not None


def test_motor_base_and_position_commands_become_named_values(rig: Callable[..., Rig]) -> None:
    r = rig(BodyAndBase)
    r.send(
        "motor_command",
        MotorCommandArray(q=[0.1, 0.2], dq=[0.3, 0.4], kp=[30, 31], kd=[1, 2], tau=[5, 6]),
    )
    assert wait_until(lambda: len(writes(r)) == 1)
    assert writes(r)[0] == {
        "g1/j1/position": 0.1,
        "g1/j1/velocity": 0.3,
        "g1/j1/effort": 5.0,
        "g1/j1/kp": 30.0,
        "g1/j1/kd": 1.0,
        "g1/j2/position": 0.2,
        "g1/j2/velocity": 0.4,
        "g1/j2/effort": 6.0,
        "g1/j2/kp": 31.0,
        "g1/j2/kd": 2.0,
    }
    r.send("base_command", Twist(linear=Vector3(0.5, 0.1, 0.0), angular=Vector3(0.0, 0.0, 0.2)))
    assert wait_until(lambda: len(writes(r)) == 2)
    assert writes(r)[1] == {"base/base/vx": 0.5, "base/base/vy": 0.1, "base/base/wz": 0.2}
    r.send("position_command", JointState(name=["g1/j2"], position=[0.7]))
    assert wait_until(lambda: writes(r)[-1] == {"g1/j2/position": 0.7})


def test_a_velocity_command_reads_only_the_velocity_field(rig: Callable[..., Rig]) -> None:
    r = rig(PositionAndVelocity)
    r.send("velocity_command", JointState(name=["mock/joint1"], position=[9.0], velocity=[0.5]))
    assert wait_until(lambda: writes(r) == [{"mock/joint1/velocity": 0.5}])


def test_readings_go_out_typed(rig: Callable[..., Rig]) -> None:
    r = rig(PushedBodyAndBase)
    readings = {
        **{f"g1/{j}/{i}": 0.1 for j in ("j1", "j2") for i in ("position", "velocity", "effort")},
        **{f"g1/imu/{i}": 0.0 for i in ("qx", "qy", "qz", "gx", "gy", "ax", "ay")},
        "g1/imu/qw": 1.0,
        "g1/imu/gz": 0.3,
        "g1/imu/az": 9.8,
        "base/base/x": 1.0,
        "base/base/y": 2.0,
        "base/base/yaw": 0.5,
    }
    r.module.publish_state(readings)
    [joints] = r.sent("joint_state")
    assert joints.name == ["g1/j1", "g1/j2"]
    assert joints.position == joints.velocity == joints.effort == [0.1, 0.1]
    [odom] = r.sent("odom")
    assert isinstance(odom, PoseStamped)
    assert (odom.position.x, odom.position.y) == (1.0, 2.0)
    assert odom.orientation.z == pytest.approx(math.sin(0.25))
    [imu] = r.sent("imu")
    assert isinstance(imu, Imu)
    assert (imu.orientation.w, imu.angular_velocity.z, imu.linear_acceleration.z) == (1.0, 0.3, 9.8)


def test_joints_of_other_robots_are_ignored(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.send("position_command", JointState(name=["other/joint1"], position=[9.0]))
    r.send("position_command", JointState(name=["other/joint1", "mock/joint1"], position=[9, 0.5]))
    assert wait_until(lambda: len(writes(r)) == 1)
    assert writes(r) == [{JOINT1: 0.5}]
    assert r.module.status().last_rejection is None


def test_only_the_newest_command_is_written_when_write_is_slow(
    rig: Callable[..., Rig],
) -> None:
    r = rig(SlowWrite, deadman_timeout_s=10.0)
    module = r.module
    assert isinstance(module, SlowWrite)
    r.move(joint1=0.1)
    assert module.inside.wait(2.0)
    # Sending returns at once even though write is stuck.
    for step in (2, 3, 4, 5):
        r.move(joint1=step / 10)
    module.release.set()
    assert wait_until(lambda: len(module.writes) == 2)
    time.sleep(0.05)
    assert writes(r) == [{JOINT1: 0.1}, {JOINT1: 0.5}]


def test_a_base_command_past_its_limit_is_pulled_back_to_it(rig: Callable[..., Rig]) -> None:
    r = rig(BodyAndBase)
    r.send("base_command", Twist(linear=Vector3(5.0, 0.0, 0.0)))
    assert wait_until(lambda: len(writes(r)) == 1)
    assert writes(r)[0]["base/base/vx"] == 1.0


def test_an_arm_command_past_its_limit_is_refused_and_shown(rig: Callable[..., Rig]) -> None:
    r = rig(position_limit=1.0)
    r.move(joint1=1.5)
    assert wait_until(lambda: r.module.status().last_rejection is not None)
    assert "limit" in str(r.module.status().last_rejection)
    assert writes(r) == []


@pytest.mark.parametrize(
    ("port", "msg", "reason"),
    [
        (
            "position_command",
            JointState(name=["mock/joint1", "mock/joint2"], position=[0.1]),
            "names",
        ),
        ("position_command", JointState(name=["mock/joint1"], position=[math.nan]), "non-finite"),
        ("motor_command", MotorCommandArray(q=[0.1]), "expected 2"),
    ],
)
def test_a_malformed_message_is_refused_and_shown(
    rig: Callable[..., Rig], port: str, msg: Any, reason: str
) -> None:
    r = rig(BodyAndBase if port == "motor_command" else MockConnectionModule)
    r.send(port, msg)
    assert wait_until(lambda: reason in str(r.module.status().last_rejection))
    assert writes(r) == []


def test_the_deadman_waits_for_the_first_command(rig: Callable[..., Rig]) -> None:
    r = rig()
    time.sleep(4 * DEADMAN_S)
    assert r.module.halts == 0
    assert not r.module.status().deadman_fired


def test_the_deadman_halts_once_then_starts_again_on_the_next_command(
    rig: Callable[..., Rig],
) -> None:
    r = rig()
    r.move(joint1=0.1)
    assert wait_until(lambda: r.module.halts == 1)
    assert r.module.status().deadman_fired
    time.sleep(4 * DEADMAN_S)
    assert r.module.halts == 1

    r.move(joint1=0.2)
    assert wait_until(lambda: len(writes(r)) == 2)
    assert wait_until(lambda: r.module.halts == 2)


def test_commands_for_other_robots_do_not_feed_the_deadman(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.move(joint1=0.1)
    deadline = time.monotonic() + 1.0
    while r.module.halts == 0 and time.monotonic() < deadline:
        r.send("position_command", JointState(name=["other/joint1"], position=[0.0]))
        r.send("position_command", JointState())
        time.sleep(DEADMAN_S / 5)
    assert r.module.halts == 1


def test_a_failing_write_halts_shows_the_error_and_carries_on(
    rig: Callable[..., Rig],
) -> None:
    r = rig(FailingWrite)
    r.move(joint1=0.1)
    assert wait_until(lambda: r.module.halts == 1)
    status = r.module.status()
    assert "motor said no" in str(status.last_error)
    assert status.connected
    # The failed write ended the deadman, so it does not halt a second time.
    time.sleep(4 * DEADMAN_S)
    assert r.module.halts == 1
    r.move(joint1=0.2)
    assert wait_until(lambda: r.module.halts == 2)


def test_published_readings_are_checked_against_the_description(
    rig: Callable[..., Rig],
) -> None:
    r = rig(PushedBodyAndBase)
    r.module.publish_state({"base/base/x": 0.0, "base/base/y": 0.0, "base/base/yaw": 0.0})
    assert len(r.sent("odom")) == 1

    r.module.publish_state({"base/base/x": 0.0})
    assert len(r.sent("odom")) == 1
    assert "missing_key" in str(r.module.status().last_error)

    r.module.publish_state({"other/joint1/position": 0.0})
    assert "other" in str(r.module.status().last_error)
    assert r.sent("joint_state") == [] and r.sent("imu") == []


def test_stopping_halts_once_and_a_second_stop_is_harmless(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.module.stop()
    r.module.stop()
    assert r.module.halts == 1
    assert not r.module.status().connected
    r.move(joint1=0.1)
    time.sleep(0.05)
    assert writes(r) == []


def test_describe_control_and_status_are_rpcs_that_cross_processes(
    rig: Callable[..., Rig],
) -> None:
    assert {"describe_control", "status"} <= set(MockConnectionModule.rpcs)
    r = rig()
    described = r.module.describe_control()
    assert isinstance(described, ConnectionDescription)
    assert described.descriptions == (r.module.describe(),)
    assert len(described.session_id) == 32
    assert pickle.loads(pickle.dumps(described)) == described

    assert wait_until(lambda: r.module.status().last_state_time is not None)
    status = r.module.status()
    assert isinstance(status, ConnectionStatus)
    assert pickle.loads(pickle.dumps(status)) == status
    assert status.connected
    assert status.last_command_time is None
    assert not status.deadman_fired
    assert status.last_rejection is None
    assert status.last_error is None


def test_the_session_id_is_new_each_start_and_steady_in_between(
    rig: Callable[..., Rig],
) -> None:
    first = rig().module
    session = first.describe_control().session_id
    assert first.describe_control().session_id == session
    first.stop()
    assert rig().module.describe_control().session_id != session
