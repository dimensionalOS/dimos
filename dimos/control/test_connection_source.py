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

from collections.abc import Collection
from pathlib import Path
import tempfile

import pytest

from dimos.control.connection_source import ConnectionSource
from dimos.hardware.manipulators.spec import ControlMode
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.utils.testing.waiting import wait_until

J1 = "arm/joint1"
J2 = "arm/joint2"
GRIPPER = "arm/gripper"
TIMEOUT = 0.1


def model_of(*joints: tuple[str, float, float]) -> RobotModel:
    """A model of some turning joints, each (name, lower, upper) in radians, one after another."""
    links = "".join(f'<link name="link{i}"/>' for i in range(len(joints) + 1))
    xml = "".join(
        f'<joint name="{name}" type="revolute"><parent link="link{i}"/>'
        f'<child link="link{i + 1}"/><limit lower="{lo}" upper="{hi}" velocity="2.0" '
        f'effort="50.0"/></joint>'
        for i, (name, lo, hi) in enumerate(joints)
    )
    path = Path(tempfile.mkdtemp(prefix="arm_")) / "arm.urdf"
    path.write_text(f'<robot name="arm">{links}{xml}</robot>')
    return RobotModel.from_file(path)


#: Two arm joints limited to +-1 rad, and another robot's joint in the same model.
ARM_MODEL = model_of((J1, -1.0, 1.0), (J2, -1.0, 1.0), ("other/joint1", -1.0, 1.0))
#: One arm joint, and a gripper that opens 0 to 0.1.
GRIPPER_MODEL = model_of((J1, -1.0, 1.0), (GRIPPER, 0.0, 0.1))


def arm(
    commands: Collection[str] = ("position_command", "velocity_command"),
    readings: Collection[str] = ("joint_state",),
    model: RobotModel | None = ARM_MODEL,
    timeout: float = TIMEOUT,
) -> ConnectionSource:
    """The arm behind a connection module, with no reading yet."""
    return ConnectionSource("arm", commands, readings, model, timeout)


def base(readings: Collection[str] = ("odom", "base_velocity")) -> ConnectionSource:
    """A base called "go2" that takes speeds and reports where it is and how fast it goes."""
    return ConnectionSource("go2", ["cmd_vel"], readings, None, TIMEOUT)


def reading(*positions: float) -> JointState:
    """The arm's joint positions, joint1 first."""
    return JointState(name=[J1, J2][: len(positions)], position=list(positions))


def arm_at(
    *positions: float,
    commands: Collection[str] = ("position_command", "velocity_command"),
    timeout: float = TIMEOUT,
) -> ConnectionSource:
    """The arm after one reading of its two joints."""
    source = arm(commands=commands, timeout=timeout)
    source.on_message("joint_state", reading(*positions))
    return source


def test_joints_come_from_the_model_under_the_robots_name() -> None:
    source = arm()

    assert source.joint_names == [J1, J2]
    assert source.command_ports == ["position_command", "velocity_command"]
    assert source.reading_ports == ["joint_state"]
    assert base().command_ports == ["cmd_vel"]
    assert base().reading_ports == ["odom", "base_velocity"]


def test_joints_and_sensors_come_from_readings() -> None:
    source = arm(readings=("joint_state", "imu"))
    source.on_message("joint_state", reading(0.3, -0.2))
    source.on_message("imu", Imu(orientation=Quaternion(0.0, 0.0, 0.0, 1.0), frame_id="imu"))

    joints = source.read_joints()
    assert joints.joint_positions == {J1: 0.3, J2: -0.2}
    # Not reported, so left out rather than read as 0.0.
    assert joints.get_velocity(J1) is None
    sensors = source.read_sensors()
    assert list(sensors) == ["arm/imu"]
    assert sensors["arm/imu"]["qw"] == 1.0
    assert len(sensors["arm/imu"]) == 10


def test_every_joint_state_message_counts() -> None:
    source = arm(model=GRIPPER_MODEL)

    source.on_message("joint_state", JointState(name=[J1], position=[0.3], effort=[1.5]))
    assert not source.ready_for_control()
    source.on_message("joint_state", JointState(name=[GRIPPER], position=[0.05]))

    assert source.ready_for_control()
    joints = source.read_joints()
    assert joints.joint_positions == {J1: 0.3, GRIPPER: 0.05}
    assert joints.joint_efforts == {J1: 1.5}


def test_other_robots_and_bad_messages_are_ignored() -> None:
    source = arm_at(0.3, 0.0)

    source.on_message("joint_state", JointState(name=["other/joint1"], position=[0.9]))
    source.on_message("joint_state", JointState(name=[J1, J2], position=[0.9]))

    assert source.read_joints().joint_positions == {J1: 0.3, J2: 0.0}


def test_not_ready_until_everything_has_reported() -> None:
    source = arm()
    assert not source.ready_for_control()
    assert source.unreported() == ["joint_state"]

    source.on_message("joint_state", reading(0.3))
    assert not source.ready_for_control()
    assert source.unreported() == ["joint_state"]

    source.on_message("joint_state", reading(0.3, 0.0))
    assert source.ready_for_control()
    assert source.unreported() == []


def test_with_no_task_every_joint_is_held_where_it_is() -> None:
    source = arm_at(0.3, -0.2)

    assert source.command({}, None) == {f"{J1}/position": 0.3, f"{J2}/position": -0.2}


def test_task_values_are_sent_and_the_other_joints_held() -> None:
    source = arm_at(0.3, -0.2)

    assert source.command({J1: 0.5}, ControlMode.POSITION) == {
        f"{J1}/position": 0.5,
        f"{J2}/position": -0.2,
    }

    # Once the task stops, the joint stays at what it was last told, not
    # wherever it has got to.
    source.on_message("joint_state", reading(0.45, -0.2))
    assert source.command({}, None)[f"{J1}/position"] == 0.5


def test_a_position_outside_its_limit_is_refused_and_the_holds_kept() -> None:
    """A refused tick leaves the holds alone, so stopping the task afterwards
    does not move the robot to what was refused."""
    source = arm_at(0.2, -0.2)
    assert source.command({J1: 0.4, J2: 0.1}, ControlMode.POSITION) == {
        f"{J1}/position": 0.4,
        f"{J2}/position": 0.1,
    }

    with pytest.raises(ValueError, match="outside"):
        source.command({J1: 1.5, J2: 0.7}, ControlMode.POSITION)

    assert source.command({}, None) == {f"{J1}/position": 0.4, f"{J2}/position": 0.1}


def test_a_velocity_tick_zeroes_the_other_joints_then_holds_where_they_stopped() -> None:
    source = arm_at(0.3, -0.2)
    source.command({J1: 0.1}, ControlMode.POSITION)

    assert source.command({J1: 0.2}, ControlMode.VELOCITY) == {
        f"{J1}/velocity": 0.2,
        f"{J2}/velocity": 0.0,
    }

    source.on_message("joint_state", reading(0.6, -0.1))
    assert source.command({}, None) == {f"{J1}/position": 0.6, f"{J2}/position": -0.1}


def test_a_held_position_stays_inside_the_models_limit() -> None:
    source = arm_at(1.0001, 0.0)

    assert source.command({}, None)[f"{J1}/position"] == 1.0


def test_a_joint_that_cannot_take_a_position_gets_zero_velocity() -> None:
    source = arm_at(0.3, 0.0, commands=("velocity_command",))

    assert source.command({}, None) == {f"{J1}/velocity": 0.0, f"{J2}/velocity": 0.0}


def test_a_task_driving_a_joint_in_a_way_it_does_not_accept_raises() -> None:
    source = arm_at(0.3, 0.0, commands=("position_command",))

    with pytest.raises(ValueError, match="velocity"):
        source.command({J1: 0.2}, ControlMode.VELOCITY)


def test_each_kind_of_command_goes_to_its_own_port() -> None:
    source = arm_at(0.3, 0.0)

    held = source.command({J1: 0.5}, ControlMode.POSITION)
    (positions,) = source.command_messages(held, ts=1.0)
    assert positions.port == "position_command"
    assert list(positions.message.name) == [J1, J2]
    assert list(positions.message.position) == [0.5, 0.0]
    assert list(positions.message.velocity) == []

    moving = source.command({J1: 0.2}, ControlMode.VELOCITY)
    (velocities,) = source.command_messages(moving, ts=1.0)
    assert velocities.port == "velocity_command"
    assert list(velocities.message.name) == [J1, J2]
    assert list(velocities.message.velocity) == [0.2, 0.0]
    assert list(velocities.message.position) == []


def test_a_base_is_told_zero_on_every_axis_on_its_port() -> None:
    source = base()
    source.on_message("odom", PoseStamped(frame_id="odom"))
    source.on_message("base_velocity", TwistStamped(frame_id="base"))

    values = source.command({}, None)
    first, *rest = source.command_messages(values, ts=1.0)
    twist = first.message

    assert source.ready_for_control()
    assert values == {"go2/base/vx": 0.0, "go2/base/vy": 0.0, "go2/base/wz": 0.0}
    assert first.port == "cmd_vel" and rest == []
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)
    assert source.joint_names == []


def test_a_robot_that_goes_quiet_is_held_where_it_is_when_it_comes_back() -> None:
    source = arm_at(0.3, 0.0, timeout=0.05)
    source.command({J1: 0.5}, ControlMode.POSITION)

    wait_until(lambda: not source.ready_for_control(), timeout=2.0)
    # Its joints stay in what tasks see.
    assert source.read_joints().joint_positions[J1] == 0.3

    # Back with only part of a reading: it waits for the rest.
    source.on_message("joint_state", reading(0.8))
    assert not source.ready_for_control()
    source.on_message("joint_state", reading(0.8, 0.0))
    assert source.ready_for_control()
    assert source.command({}, None)[f"{J1}/position"] == 0.8


def test_dropped_holds_are_taken_again_from_the_next_whole_reading() -> None:
    source = arm_at(0.3, 0.0)
    source.command({J1: 0.5}, ControlMode.POSITION)
    # Halted short of where it was told to go.
    source.on_message("joint_state", reading(0.4, 0.0))

    source.drop_holds()

    # The last reading may predate the halt, so it waits for a whole new one.
    assert not source.ready_for_control()
    assert source.read_joints().joint_positions[J1] == 0.4
    source.on_message("joint_state", reading(0.42))
    assert not source.ready_for_control()
    source.on_message("joint_state", reading(0.42, 0.0))
    assert source.ready_for_control()
    assert source.command({}, None)[f"{J1}/position"] == 0.42


def test_a_robot_it_cannot_drive_is_refused() -> None:
    with pytest.raises(ValueError, match="reports nothing"):
        arm(readings=())
    with pytest.raises(ValueError, match="arm_motor_command needs stiffness"):
        arm(commands=("motor_command",))
    with pytest.raises(ValueError, match="cannot be held"):
        arm(commands=("position_command",), readings=("odom",))
    with pytest.raises(ValueError, match="cannot be held"):
        arm(commands=("cmd_vel",), readings=("joint_state",))
    with pytest.raises(ValueError, match="no model"):
        arm(model=None)
    with pytest.raises(ValueError, match="no movable joint named nobody/"):
        ConnectionSource("nobody", ["position_command"], ["joint_state"], ARM_MODEL, TIMEOUT)


def test_a_reading_in_another_robots_frame_is_dropped() -> None:
    """A frame with a namespace must be this robot's, so a stream two robots
    share can neither fill in this robot's readings nor keep them fresh. A
    frame with no namespace is taken as its own."""
    source = base()
    source.on_message("odom", PoseStamped(frame_id="go2a/odom"))
    source.on_message("base_velocity", TwistStamped(frame_id="go2a/base"))
    assert not source.ready_for_control()
    assert source.unreported() == ["odom", "base_velocity"]

    source.on_message("odom", PoseStamped(frame_id="go2/odom"))
    source.on_message("base_velocity", TwistStamped(frame_id="base"))
    assert source.ready_for_control()
