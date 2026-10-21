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

"""Tests for ``ConnectionModule``, driven through ``FakeArm``.

Real modules with real threads, no coordinator. Every port is given an
in-process transport, so nothing touches the network.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
import math
import pickle
import threading
import time
from typing import Any

import pytest

from dimos.control._control_test_helpers import FakeArm, FakeArmConfig
from dimos.control.connection.connection_module import ConnectionStatus
from dimos.control.contract.description import Limits
from dimos.control.contract.keys import Interface
from dimos.control.contract.ports import JointPort, MotorPort, TwistPort
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
JOINTS = ["joint1", "joint2"]
#: Every reading a ``BodyAndBase`` reports, all zero but a level IMU.
BODY_AND_BASE_READINGS = {
    **{f"g1/{j}/{i}": 0.0 for j in JOINTS for i in ("position", "velocity", "effort")},
    **{f"g1/imu/{i}": 0.0 for i in ("qx", "qy", "qz", "gx", "gy", "gz", "ax", "ay", "az")},
    "g1/imu/qw": 1.0,
    **{f"g1/base/{i}": 0.0 for i in ("x", "y", "yaw", "vx", "vy", "wz")},
}


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

    def __init__(self, module: FakeArm) -> None:
        self.module = module
        self.ports = {name: LocalTransport() for name in {**module.inputs, **module.outputs}}
        for name, transport in self.ports.items():
            module.set_transport(name, transport)  # type: ignore[arg-type]

    def send(self, port: str, msg: Any) -> None:
        self.ports[port].publish(msg)

    def move(self, **positions: float) -> None:
        """Send joint positions on ``position_command``, e.g. ``move(joint1=0.5)``."""
        names = [f"{self.module.config.source}/{joint}" for joint in positions]
        self.send("position_command", JointState(name=names, position=list(positions.values())))

    def sent(self, port: str) -> list[Any]:
        return list(self.ports[port].sent)


@pytest.fixture
def rig() -> Iterator[Callable[..., Rig]]:
    """Build modules, start them, and stop every one at the end of the test."""
    made: list[Rig] = []

    def build(cls: type[FakeArm] = FakeArm, **config: Any) -> Rig:
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


class BodyAndBaseConfig(FakeArmConfig):
    """A two-joint body held by stiffness and damping, on a base, with an IMU."""

    source: str = "g1"
    position_command: JointPort = JointPort(joints=JOINTS)
    motor_command: MotorPort = MotorPort(joints=JOINTS)
    base_command: TwistPort = TwistPort(
        range=Twist(linear=Vector3(1.0, 1.0, 0.0), angular=Vector3(0.0, 0.0, 1.0))
    )


class BodyAndBase(FakeArm):
    config: BodyAndBaseConfig

    motor_command: In[MotorCommandArray]
    base_command: In[Twist]
    odom: Out[PoseStamped]
    base_velocity: Out[Twist]
    imu: Out[Imu]


class PushedBodyAndBase(BodyAndBase):
    """Never polled: readings only go out when the test publishes them."""

    def read_state(self) -> dict[str, float]:
        return {}


class PositionAndVelocityConfig(FakeArmConfig):
    position_command: JointPort = JointPort(joints=["joint1"])
    velocity_command: JointPort = JointPort(interface=Interface.VELOCITY, joints=["joint1"])


class PositionAndVelocity(FakeArm):
    config: PositionAndVelocityConfig

    velocity_command: In[JointState]


class SlowWrite(FakeArm):
    """Its first write waits until the test lets it go."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.inside = threading.Event()
        self.release = threading.Event()

    def write(self, values: dict[str, float]) -> None:
        self.inside.set()
        self.release.wait(5.0)
        super().write(values)


class FailingWrite(FakeArm):
    def write(self, values: dict[str, float]) -> None:
        raise RuntimeError("motor said no")


class Watched(FakeArm):
    """Remembers whether ``connect`` was called."""

    connected = False

    def connect(self) -> None:
        self.connected = True
        super().connect()


class NoVelocityPort(Watched):
    config: PositionAndVelocityConfig  # type: ignore[assignment]


class WrongPortType(Watched):
    position_command: In[Twist]  # type: ignore[assignment]


class TwoJointStateOutputs(Watched):
    joint_state_too: Out[JointState]


class TwoBasesConfig(BodyAndBaseConfig):
    cmd_vel: TwistPort = TwistPort(range=Twist())


class TwoBases(BodyAndBase):
    config: TwoBasesConfig  # type: ignore[assignment]
    cmd_vel: In[Twist]

    connected = False

    def connect(self) -> None:
        self.connected = True
        super().connect()


@pytest.mark.parametrize(
    ("cls", "config", "error", "match"),
    [
        (Watched, {"state_rate_hz": 0.0}, DescriptionError, "state_rate_hz"),
        (Watched, {"model": None}, ValueError, "'position_command' is a JointPort, which needs"),
        (NoVelocityPort, {}, ValueError, "no input port called 'velocity_command'"),
        (WrongPortType, {}, ValueError, "'position_command' must carry JointState"),
        (TwoJointStateOutputs, {}, ValueError, "both report JointState"),
        (TwoBases, {}, ValueError, "'base_command' and 'cmd_vel' both drive the base"),
    ],
)
def test_start_fails_before_connecting_on_a_bad_config_or_ports(
    cls: type[Watched | TwoBases], config: dict[str, Any], error: type[Exception], match: str
) -> None:
    module = cls(**config)
    try:
        with pytest.raises(error, match=match):
            module.start()
        assert not module.connected
    finally:
        module.stop()


def test_the_description_comes_from_the_config_and_the_ports() -> None:
    module = FakeArm()
    try:
        described = module.description
        assert "description" not in FakeArm.rpcs
        assert described.source == "mock"
        assert described.joint_names() == tuple(f"mock/joint{i}" for i in range(1, 8))
        joint = described.resource("joint1")
        assert joint is not None
        # Told its position; reports what an Out[JointState] carries.
        assert joint.command_interfaces == (Interface.POSITION,)
        assert joint.state_interfaces == (
            Interface.POSITION,
            Interface.VELOCITY,
            Interface.EFFORT,
        )
        assert described.limits[JOINT1] == Limits(-math.pi, math.pi)
        assert module.description is described
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
    assert msg.position == msg.velocity == msg.effort == [0.0] * 7


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
        "g1/joint1/position": 0.1,
        "g1/joint1/velocity": 0.3,
        "g1/joint1/effort": 5.0,
        "g1/joint1/kp": 30.0,
        "g1/joint1/kd": 1.0,
        "g1/joint2/position": 0.2,
        "g1/joint2/velocity": 0.4,
        "g1/joint2/effort": 6.0,
        "g1/joint2/kp": 31.0,
        "g1/joint2/kd": 2.0,
    }
    r.send("base_command", Twist(linear=Vector3(0.5, 0.1, 0.0), angular=Vector3(0.0, 0.0, 0.2)))
    assert wait_until(lambda: len(writes(r)) == 2)
    assert writes(r)[1] == {"g1/base/vx": 0.5, "g1/base/vy": 0.1, "g1/base/wz": 0.2}
    r.send("position_command", JointState(name=["g1/joint2"], position=[0.7]))
    assert wait_until(lambda: writes(r)[-1] == {"g1/joint2/position": 0.7})


def test_the_same_joint_on_a_position_and_a_velocity_port_takes_both(
    rig: Callable[..., Rig],
) -> None:
    r = rig(PositionAndVelocity)
    joint = r.module.description.resource("joint1")
    assert joint is not None
    assert joint.command_interfaces == (Interface.POSITION, Interface.VELOCITY)
    assert r.module.description.limits["mock/joint1/velocity"] == Limits(-2.0, 2.0)
    # The velocity port reads only the velocity field.
    r.send("velocity_command", JointState(name=["mock/joint1"], position=[9.0], velocity=[0.5]))
    assert wait_until(lambda: writes(r) == [{"mock/joint1/velocity": 0.5}])
    r.move(joint1=0.3)
    assert wait_until(lambda: writes(r)[-1] == {JOINT1: 0.3})


def test_readings_go_out_typed(rig: Callable[..., Rig]) -> None:
    r = rig(PushedBodyAndBase)
    readings = {
        **BODY_AND_BASE_READINGS,
        **{f"g1/{j}/{i}": 0.1 for j in JOINTS for i in ("position", "velocity", "effort")},
        "g1/imu/gz": 0.3,
        "g1/imu/az": 9.8,
        "g1/base/x": 1.0,
        "g1/base/y": 2.0,
        "g1/base/yaw": 0.5,
        "g1/base/vx": 0.4,
        "g1/base/vy": -0.1,
        "g1/base/wz": 0.2,
    }
    r.module.publish_state(readings)
    [joints] = r.sent("joint_state")
    assert joints.name == ["g1/joint1", "g1/joint2"]
    assert joints.position == joints.velocity == joints.effort == [0.1, 0.1]
    [odom] = r.sent("odom")
    assert isinstance(odom, PoseStamped)
    assert (odom.position.x, odom.position.y) == (1.0, 2.0)
    assert odom.orientation.z == pytest.approx(math.sin(0.25))
    [speed] = r.sent("base_velocity")
    assert isinstance(speed, Twist)
    assert (speed.linear.x, speed.linear.y, speed.angular.z) == (0.4, -0.1, 0.2)
    [imu] = r.sent("imu")
    assert isinstance(imu, Imu)
    assert (imu.orientation.w, imu.angular_velocity.z, imu.linear_acceleration.z) == (1.0, 0.3, 9.8)
    # Frames: the module's own for the IMU, the odometry frame for the pose.
    assert imu.frame_id == "PushedBodyAndBase"
    assert odom.frame_id == "odom"


def test_reading_frames_follow_the_module_config(rig: Callable[..., Rig]) -> None:
    r = rig(PushedBodyAndBase, frame_id="pelvis", frame_id_prefix="r1")
    r.module.publish_state(BODY_AND_BASE_READINGS)
    [imu] = r.sent("imu")
    [odom] = r.sent("odom")
    assert imu.frame_id == "r1/pelvis"
    assert odom.frame_id == "r1/odom"


def test_the_mock_base_reports_the_speed_it_was_last_told(rig: Callable[..., Rig]) -> None:
    r = rig(BodyAndBase)
    r.send("base_command", Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.2)))
    assert wait_until(
        lambda: any(t.linear.x == 0.5 and t.angular.z == 0.2 for t in r.sent("base_velocity"))
    )


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
    assert writes(r)[0]["g1/base/vx"] == 1.0


def test_an_arm_command_past_its_limit_is_refused_and_shown(rig: Callable[..., Rig]) -> None:
    r = rig(position_command=JointPort(limit_overrides={"joint1": Limits(-1.0, 1.0)}))
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
    r = rig(BodyAndBase if port == "motor_command" else FakeArm)
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
    assert not r.module.status().deadman_fired
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


def test_any_accepted_command_feeds_the_one_deadman(rig: Callable[..., Rig], mocker) -> None:
    """Base commands keep the module alive after the body's stop."""
    r = rig(BodyAndBase)
    loop = r.module._loop
    assert loop is not None
    # Advance the timer's clock only after each command reaches the driver.
    # A slow producer thread must not turn this into a real deadline miss.
    clock = mocker.patch.object(loop, "time", return_value=loop.time())
    body = MotorCommandArray(q=[0.1, 0.2], dq=[0, 0], kp=[1, 1], kd=[1, 1], tau=[0, 0])
    forward = Twist(Vector3(0.1, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
    r.send("motor_command", body)
    assert wait_until(lambda: len(writes(r)) == 1)
    for count in range(2, 7):
        clock.return_value += DEADMAN_S * 0.75
        r.send("base_command", forward)
        assert wait_until(lambda expected=count: len(writes(r)) == expected)
        assert r.module.halts == 0

    clock.return_value += DEADMAN_S * 1.01
    loop.call_soon_threadsafe(lambda: None)
    assert wait_until(lambda: r.module.status().deadman_fired)
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
    r.module.publish_state(BODY_AND_BASE_READINGS)
    assert len(r.sent("odom")) == 1

    r.module.publish_state({"g1/base/x": 0.0})
    assert len(r.sent("odom")) == 1
    assert "missing_key" in str(r.module.status().last_error)

    r.module.publish_state({**BODY_AND_BASE_READINGS, "other/joint1/position": 0.0})
    assert len(r.sent("odom")) == 1
    assert "other/joint1/position" in str(r.module.status().last_error)


def test_stopping_halts_once_and_a_second_stop_is_harmless(rig: Callable[..., Rig]) -> None:
    r = rig()
    r.module.stop()
    r.module.stop()
    assert r.module.halts == 1
    assert not r.module.status().connected
    r.move(joint1=0.1)
    time.sleep(0.05)
    assert writes(r) == []


def test_status_is_an_rpc_that_crosses_processes(rig: Callable[..., Rig]) -> None:
    assert "status" in FakeArm.rpcs
    assert "describe_control" not in FakeArm.rpcs
    r = rig()
    assert wait_until(lambda: r.module.status().last_state_time is not None)
    status = r.module.status()
    assert isinstance(status, ConnectionStatus)
    assert pickle.loads(pickle.dumps(status)) == status
    assert len(status.session_id) == 32
    assert status.connected
    assert status.last_command_time is None
    assert status.deadman_fired is False
    assert status.last_rejection is None
    assert status.last_error is None


def test_the_session_id_is_new_each_start_and_steady_in_between(
    rig: Callable[..., Rig],
) -> None:
    first = rig().module
    session = first.status().session_id
    assert first.status().session_id == session
    first.stop()
    assert rig().module.status().session_id != session
