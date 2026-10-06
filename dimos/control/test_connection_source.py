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

"""Tests for ``ConnectionSource``: reading messages in, complete commands out."""

from __future__ import annotations

import time
from typing import Any

from dimos.control.connection_source import ConnectionSource
from dimos.control.contract.description import ControlDescription, Limits
from dimos.control.contract.keys import EFFORT, POSITION, VELOCITY, VX, VY, WZ, Key, Unit
from dimos.control.contract.presets import (
    GripperSpec,
    imu_resource,
    manipulator_description,
    pd_joint_description,
    twist_base_description,
)
from dimos.hardware.manipulators.spec import ControlMode
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState

J1 = "arm/joint1"
J2 = "arm/joint2"
GRIPPER = "arm/gripper"


def arm(**kwargs: Any) -> ControlDescription:
    """A two-joint arm that reports position and takes position or velocity,
    limited to +-1 rad."""
    joints = ["joint1", "joint2"]
    settings: dict[str, Any] = {
        "limits": {Key.of("arm", j, POSITION): Limits(-1.0, 1.0) for j in joints},
        "state": (POSITION,),
        "command": (POSITION, VELOCITY),
    }
    settings.update(kwargs)
    return manipulator_description("arm", joints, **settings)


def arm_with_gripper() -> ControlDescription:
    """A one-joint arm that reports position and effort, and a gripper that
    reports only its position."""
    return manipulator_description(
        "arm",
        ["joint1"],
        state=(POSITION, EFFORT),
        command=(POSITION, VELOCITY),
        gripper=GripperSpec(unit=Unit.M, lo=0.0, hi=0.1),
    )


def reading(*positions: float) -> JointState:
    """The arm's joint positions, joint1 first."""
    return JointState(name=[J1, J2][: len(positions)], position=list(positions))


def source_at(*positions: float, description: ControlDescription | None = None) -> ConnectionSource:
    """A source for the two-joint arm that has had one reading."""
    source = ConnectionSource(description or arm())
    source.on_message("joint_state", reading(*positions))
    return source


def test_the_ports_it_needs_follow_its_description() -> None:
    assert ConnectionSource(arm()).command_ports == ["position_command", "velocity_command"]
    assert ConnectionSource(arm()).reading_ports == ["joint_state"]
    with_imu = ConnectionSource(arm(sensors=[imu_resource()]))
    assert with_imu.reading_ports == ["joint_state", "imu"]
    limits: dict[str, Limits] = {
        Key.of("go2", "base", axis): Limits(-1.0, 1.0) for axis in (VX, VY, WZ)
    }
    base = ConnectionSource(twist_base_description("go2", limits=limits))
    assert base.command_ports == ["base_command"]
    assert base.reading_ports == ["odom", "base_velocity"]


def test_joints_and_sensors_come_from_readings() -> None:
    source = ConnectionSource(arm(sensors=[imu_resource()]))
    source.on_message("joint_state", reading(0.3, -0.2))
    source.on_message("imu", Imu(orientation=Quaternion(0.0, 0.0, 0.0, 1.0)))

    joints = source.read_joints()
    assert joints.joint_positions == {J1: 0.3, J2: -0.2}
    # Not reported, so left out rather than read as 0.0.
    assert joints.get_velocity(J1) is None
    sensors = source.read_sensors()
    assert list(sensors) == ["arm/imu"]
    assert sensors["arm/imu"]["qw"] == 1.0
    assert len(sensors["arm/imu"]) == 10


def test_every_joint_state_message_counts() -> None:
    source = ConnectionSource(arm_with_gripper())

    source.on_message("joint_state", JointState(name=[J1], position=[0.3], effort=[1.5]))
    assert not source.ready_for_control()
    source.on_message("joint_state", JointState(name=[GRIPPER], position=[0.05]))

    assert source.ready_for_control()
    joints = source.read_joints()
    assert joints.joint_positions == {J1: 0.3, GRIPPER: 0.05}
    assert joints.joint_efforts == {J1: 1.5}


def test_other_robots_and_bad_messages_are_ignored() -> None:
    source = source_at(0.3, 0.0)

    source.on_message("joint_state", JointState(name=["other/joint1"], position=[0.9]))
    source.on_message("joint_state", JointState(name=[J1, J2], position=[0.9]))

    assert source.read_joints().joint_positions == {J1: 0.3, J2: 0.0}


def test_not_ready_until_everything_has_reported() -> None:
    source = ConnectionSource(arm())
    assert not source.ready_for_control()

    source.on_message("joint_state", reading(0.3))
    assert not source.ready_for_control()

    source.on_message("joint_state", reading(0.3, 0.0))
    assert source.ready_for_control()


def test_with_no_task_every_joint_is_held_where_it_is() -> None:
    source = source_at(0.3, -0.2)

    assert source.command({}, None) == {f"{J1}/position": 0.3, f"{J2}/position": -0.2}


def test_task_values_are_sent_and_the_other_joints_held() -> None:
    source = source_at(0.3, -0.2)

    assert source.command({J1: 0.5}, ControlMode.POSITION) == {
        f"{J1}/position": 0.5,
        f"{J2}/position": -0.2,
    }

    # Once the task stops, the joint stays at what it was last told, not
    # wherever it has got to.
    source.on_message("joint_state", reading(0.45, -0.2))
    assert source.command({}, None)[f"{J1}/position"] == 0.5


def test_a_velocity_tick_zeroes_the_other_joints_then_holds_where_they_stopped() -> None:
    source = source_at(0.3, -0.2)
    source.command({J1: 0.1}, ControlMode.POSITION)

    assert source.command({J1: 0.2}, ControlMode.VELOCITY) == {
        f"{J1}/velocity": 0.2,
        f"{J2}/velocity": 0.0,
    }

    source.on_message("joint_state", reading(0.6, -0.1))
    assert source.command({}, None) == {f"{J1}/position": 0.6, f"{J2}/position": -0.1}


def test_a_held_position_stays_inside_the_limit_but_task_values_are_sent_as_is() -> None:
    source = source_at(1.0001, 0.0)

    assert source.command({}, None)[f"{J1}/position"] == 1.0
    assert source.command({J1: 1.5}, ControlMode.POSITION)[f"{J1}/position"] == 1.5
    assert source.command({}, None)[f"{J1}/position"] == 1.0


def test_a_joint_that_cannot_take_a_position_gets_zero_velocity() -> None:
    source = source_at(0.3, 0.0, description=arm(command=(VELOCITY,), limits={}))

    assert source.command({}, None) == {f"{J1}/velocity": 0.0, f"{J2}/velocity": 0.0}


def test_a_winner_the_joint_cannot_take_is_held_instead() -> None:
    source = source_at(0.3, 0.0, description=arm(command=(POSITION,)))

    assert source.command({J1: 0.2}, ControlMode.VELOCITY) == {
        f"{J1}/position": 0.3,
        f"{J2}/position": 0.0,
    }


def test_each_kind_of_command_goes_in_its_own_message() -> None:
    source = ConnectionSource(arm_with_gripper())
    source.on_message("joint_state", JointState(name=[J1], position=[0.3], effort=[0.0]))
    source.on_message("joint_state", JointState(name=[GRIPPER], position=[0.05]))

    values = source.command({J1: 0.2}, ControlMode.VELOCITY)
    messages = dict(source.command_messages(values, ts=1.0))

    assert list(messages) == ["position_command", "velocity_command"]
    assert list(messages["position_command"].name) == [GRIPPER]
    assert list(messages["position_command"].position) == [0.05]
    assert list(messages["velocity_command"].name) == [J1]
    assert list(messages["velocity_command"].velocity) == [0.2]


def test_a_base_is_told_zero_on_every_axis() -> None:
    limits: dict[str, Limits] = {
        Key.of("go2", "base", axis): Limits(-1.0, 1.0) for axis in (VX, VY, WZ)
    }
    source = ConnectionSource(twist_base_description("go2", limits=limits, odometry=False))

    values = source.command({}, None)
    (port, twist), *rest = source.command_messages(values, ts=1.0)

    assert values == {"go2/base/vx": 0.0, "go2/base/vy": 0.0, "go2/base/wz": 0.0}
    assert port == "base_command" and rest == []
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)
    assert source.joint_names == []


def test_described_again_it_keeps_its_holds_and_waits_for_the_next_reading() -> None:
    description = arm()
    before = source_at(0.3, 0.0)
    before.command({J1: 0.5}, ControlMode.POSITION)

    after = ConnectionSource(description)
    after.adopt(before)

    # Its joints are still known, but nothing is sent until it reports again.
    assert after.read_joints().joint_positions[J1] == 0.3
    assert not after.ready_for_control()

    # A held arm sags a little below its target; the hold stays on the target.
    after.on_message("joint_state", reading(0.49, 0.0))
    assert after.ready_for_control()
    assert after.command({}, None)[f"{J1}/position"] == 0.5


def test_a_robot_that_goes_quiet_is_held_where_it_is_when_it_comes_back() -> None:
    source = source_at(0.3, 0.0, description=arm(deadman_timeout_s=0.05))
    source.command({J1: 0.5}, ControlMode.POSITION)

    time.sleep(0.08)
    assert not source.ready_for_control()
    # Its joints stay in what tasks see.
    assert source.read_joints().joint_positions[J1] == 0.3

    source.on_message("joint_state", reading(0.8, 0.0))
    assert source.ready_for_control()
    assert source.command({}, None)[f"{J1}/position"] == 0.8


def test_robots_it_cannot_hold_are_named() -> None:
    effort_only = manipulator_description("arm", ["joint1"], state=(POSITION,), command=(EFFORT,))
    body = pd_joint_description("g1", ["hip"])

    assert ConnectionSource.why_not_drivable(arm()) is None
    assert "joint1" in str(ConnectionSource.why_not_drivable(effort_only))
    assert "stiffness" in str(ConnectionSource.why_not_drivable(body))
