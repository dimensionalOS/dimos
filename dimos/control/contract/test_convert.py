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

from __future__ import annotations

import math
import time

import numpy as np
import pytest

from dimos.control._control_test_helpers import fake_arm_model
from dimos.control.contract.convert import (
    IMU_INTERFACES,
    imu_from_values,
    imu_to_values,
    joint_state_from_values,
    joint_state_to_values,
    motor_command_from_values,
    motor_command_to_values,
    motor_joints,
    pose_from_values,
    pose_to_values,
    twist_from_values,
    twist_to_values,
)
from dimos.control.contract.keys import Interface
from dimos.message_utils import geometry, time as message_time
from dimos_messages.dimos_msgs.msg import MotorCommandArray
from dimos_messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, Vector3
from dimos_messages.sensor_msgs.msg import Imu, JointState
from dimos_messages.std_msgs.msg import Header

BASE = "chassis/base"


def test_a_twist_carries_forwards_leftwards_and_turning_only() -> None:
    twist = Twist(linear=Vector3(0.5, 0.2, 9.0), angular=Vector3(9.0, 9.0, -0.3))
    values = twist_to_values(twist, BASE)
    assert values == {"chassis/base/vx": 0.5, "chassis/base/vy": 0.2, "chassis/base/wz": -0.3}
    back = twist_from_values(values, BASE)
    assert (back.linear.x, back.linear.y, back.linear.z) == (0.5, 0.2, 0.0)
    assert (back.angular.x, back.angular.y, back.angular.z) == (0.0, 0.0, -0.3)


def test_a_joint_state_gives_only_the_fields_asked_for() -> None:
    msg = JointState(
        header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
        name=["arm/j1", "arm/j2"],
        position=np.asarray([0.1, 0.2], dtype=np.float64),
        velocity=np.asarray([1.0, 2.0], dtype=np.float64),
        effort=np.asarray([], dtype=np.float64),
    )
    assert joint_state_to_values(msg, (Interface.POSITION,)) == {
        "arm/j1/position": 0.1,
        "arm/j2/position": 0.2,
    }
    assert joint_state_to_values(msg, (Interface.VELOCITY, Interface.EFFORT)) == {
        "arm/j1/velocity": 1.0,
        "arm/j2/velocity": 2.0,
    }


def test_a_malformed_joint_state_raises() -> None:
    with pytest.raises(ValueError, match="2 names but 1 position"):
        joint_state_to_values(
            JointState(
                header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
                name=["a/j1", "a/j2"],
                position=np.asarray([0.1], dtype=np.float64),
                velocity=np.asarray([], dtype=np.float64),
                effort=np.asarray([], dtype=np.float64),
            ),
            (Interface.POSITION,),
        )
    with pytest.raises(ValueError, match="no 'kp' field"):
        joint_state_to_values(
            JointState(
                header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
                name=["a/j1"],
                position=np.asarray([], dtype=np.float64),
                velocity=np.asarray([], dtype=np.float64),
                effort=np.asarray([], dtype=np.float64),
            ),
            ("kp",),
        )


def test_a_joint_state_from_values_fills_the_fields_every_joint_has() -> None:
    values = {"arm/j1/position": 0.1, "arm/j2/position": 0.2, "arm/j1/effort": 3.0}
    msg = joint_state_from_values(values, ["arm/j1"], ts=5.0)
    assert (
        message_time.to_seconds(msg.header.stamp),
        msg.name,
        list(msg.position),
        list(msg.velocity),
        list(msg.effort),
    ) == (
        5.0,
        ["arm/j1"],
        [0.1],
        [],
        [3.0],
    )
    assert joint_state_to_values(
        msg, (Interface.POSITION, Interface.VELOCITY, Interface.EFFORT)
    ) == {
        "arm/j1/position": 0.1,
        "arm/j1/effort": 3.0,
    }
    with pytest.raises(ValueError, match=r"effort is missing for \['arm/j2'\]"):
        joint_state_from_values(values, ["arm/j1", "arm/j2"])
    with pytest.raises(ValueError, match="no position"):
        joint_state_from_values(values, ["arm/j3"])


def test_motor_joints_are_the_models_moving_joints_named_in_full() -> None:
    model = fake_arm_model(("left_knee", "right_knee"))
    assert motor_joints(model, "g1") == ["g1/left_knee", "g1/right_knee"]


def test_a_motor_command_round_trips_in_joint_order() -> None:
    joints = ["g1/a", "g1/b"]
    msg = MotorCommandArray(
        header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
        q=np.asarray([0.1, 0.2], dtype=np.float64),
        dq=np.asarray([0.3, 0.4], dtype=np.float64),
        kp=np.asarray([10, 20], dtype=np.float64),
        kd=np.asarray([1, 2], dtype=np.float64),
        tau=np.asarray([5, 6], dtype=np.float64),
    )
    values = motor_command_to_values(msg, joints)
    assert values["g1/b/position"] == 0.2
    assert values["g1/a/effort"] == 5.0
    assert values["g1/b/kd"] == 2.0
    assert len(values) == 10
    back = motor_command_from_values(values, joints, ts=7.0)
    assert (list(back.q), list(back.dq), list(back.kp), list(back.kd), list(back.tau)) == (
        list(msg.q),
        list(msg.dq),
        list(msg.kp),
        list(msg.kd),
        list(msg.tau),
    )
    assert message_time.to_seconds(back.header.stamp) == 7.0


def test_a_motor_command_of_the_wrong_size_or_missing_values_raises() -> None:
    with pytest.raises(ValueError, match="1 joints, expected 2"):
        motor_command_to_values(
            MotorCommandArray(
                header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
                q=np.asarray([0.1], dtype=np.float64),
                dq=np.asarray([], dtype=np.float64),
                kp=np.asarray([], dtype=np.float64),
                kd=np.asarray([], dtype=np.float64),
                tau=np.asarray([], dtype=np.float64),
            ),
            ["g1/a", "g1/b"],
        )
    with pytest.raises(ValueError, match=r"missing values \['g1/a/kd'\]"):
        motor_command_from_values(
            {"g1/a/position": 0, "g1/a/velocity": 0, "g1/a/effort": 0, "g1/a/kp": 0}, ["g1/a"]
        )


@pytest.mark.parametrize("yaw", [0.0, 0.7, -2.5, math.pi])
def test_a_pose_round_trips_through_x_y_and_yaw(yaw: float) -> None:
    pose = pose_from_values(
        {"chassis/base/x": 1.0, "chassis/base/y": -2.0, "chassis/base/yaw": yaw},
        BASE,
        ts=3.0,
        frame_id="odom",
    )
    assert (
        message_time.to_seconds(pose.header.stamp),
        pose.header.frame_id,
        pose.pose.position.z,
    ) == (3.0, "odom", 0.0)
    values = pose_to_values(pose, BASE)
    assert values["chassis/base/x"] == 1.0
    assert values["chassis/base/y"] == -2.0
    # Facing exactly backwards may come back as +pi or -pi; both are the same.
    assert math.cos(values["chassis/base/yaw"] - yaw) == pytest.approx(1.0)


def test_pose_yaw_is_read_from_a_tilted_orientation() -> None:
    orientation = geometry.quaternion_from_euler(0.2, -0.1, 1.2)
    values = pose_to_values(
        PoseStamped(
            header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
            pose=Pose(position=Point(0, 0, 0), orientation=orientation),
        ),
        BASE,
    )
    assert values["chassis/base/yaw"] == pytest.approx(1.2)


def test_an_imu_round_trips_all_ten_readings() -> None:
    imu = Imu(
        orientation=Quaternion(0.1, 0.2, 0.3, 0.9),
        angular_velocity=Vector3(1.0, 2.0, 3.0),
        linear_acceleration=Vector3(4.0, 5.0, 9.8),
        header=Header(stamp=message_time.time_from_seconds(time.time()), frame_id=""),
        orientation_covariance=np.zeros(9, dtype=np.float64),
        angular_velocity_covariance=np.zeros(9, dtype=np.float64),
        linear_acceleration_covariance=np.zeros(9, dtype=np.float64),
    )
    values = imu_to_values(imu, "g1/imu")
    assert list(values) == [f"g1/imu/{i}" for i in IMU_INTERFACES]
    assert values["g1/imu/qw"] == 0.9
    assert values["g1/imu/gy"] == 2.0
    assert values["g1/imu/az"] == 9.8
    assert imu_to_values(imu_from_values(values, "g1/imu", ts=1.0), "g1/imu") == values


def test_from_values_names_what_is_missing() -> None:
    with pytest.raises(ValueError, match=r"missing values \['chassis/base/wz'\]"):
        twist_from_values({"chassis/base/vx": 0.0, "chassis/base/vy": 0.0}, BASE)
    with pytest.raises(ValueError, match="chassis/base/yaw"):
        pose_from_values({"chassis/base/x": 0.0, "chassis/base/y": 0.0}, BASE)
    with pytest.raises(ValueError, match="g1/imu/qx"):
        imu_from_values({}, "g1/imu")
