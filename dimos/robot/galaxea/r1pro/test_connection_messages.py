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

import math

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.dimos_msgs.msg import MotorCommandArray
from dimos_generated.geometry_msgs.msg import Twist, TwistStamped, Vector3
from dimos_generated.sensor_msgs.msg import JointState
from dimos_generated.std_msgs.msg import Header
import pytest

from dimos.hardware.whole_body.spec import VEL_STOP
from dimos.protocol.pubsub.impl.rospubsub import RawROS, RawROSTopic
from dimos.robot.galaxea.r1pro import connection


@pytest.fixture()
def robot():
    robot = connection.R1ProConnection()
    try:
        yield robot
    finally:
        robot.stop()


def speed(sec, nanosec, *, vx=1.0, vy=0.0, wz=0.0):
    return TwistStamped(
        header=Header(stamp=Time(sec=sec, nanosec=nanosec)),
        twist=Twist(linear=Vector3(x=vx, y=vy), angular=Vector3(z=wz)),
    )


def test_integrated_pose_odometry_and_tf_preserve_nanoseconds(robot, monkeypatch):
    outputs = {name: [] for name in ("odom", "odometry", "tf")}
    for name, values in outputs.items():
        monkeypatch.setattr(getattr(robot, name), "publish", values.append)
    robot._on_chassis_speed(speed(1700000000, 123456789), None)
    robot._on_chassis_speed(speed(1700000000, 223456789, vy=0.5, wz=0.2), None)

    pose, odometry, tf = [outputs[name][0] for name in outputs]
    assert pose.pose.position.x == pytest.approx(0.1)
    assert pose.pose.position.y == pytest.approx(0.05)
    assert pose.pose.orientation.z == pytest.approx(math.sin(0.01))
    assert odometry.pose.pose == pose.pose
    assert odometry.twist.twist.linear == Vector3(x=1.0, y=0.5)
    assert [edge.child_frame_id for edge in tf.transforms] == [
        robot.config.frame_id,
        robot.config.lidar_frame_id,
    ]
    assert tf.transforms[1].header.frame_id == robot.config.frame_id
    assert tf.transforms[1].transform.rotation.w == 1.0
    for message in (pose, odometry, *tf.transforms):
        assert message.header.stamp == Time(sec=1700000000, nanosec=223456789)
    for message in (pose, odometry, tf):
        assert type(message).decode(message.encode()) == message


@pytest.mark.parametrize("second_stamp", [(0, 0), (-1, 0), (2, 1)])
def test_clock_jump_does_not_move_base(robot, monkeypatch, second_stamp):
    values = []
    monkeypatch.setattr(robot.odom, "publish", values.append)
    robot._on_chassis_speed(speed(0, 0), None)
    robot._on_chassis_speed(speed(*second_stamp), None)
    assert values == []
    assert robot._odom_x == 0.0


@pytest.mark.parametrize("oldest", [Time(), Time(sec=1700000000, nanosec=123456788)])
def test_joint_snapshot_uses_oldest_exact_stamp_and_all_segments(robot, monkeypatch, oldest):
    for callback, count, stamp, value in [
        (robot._on_feedback_torso, 4, oldest, 1.0),
        (robot._on_feedback_left, 7, Time(sec=1700000000, nanosec=123456789), 2.0),
        (robot._on_feedback_right, 7, Time(sec=1700000000, nanosec=123456790), 3.0),
    ]:
        callback(
            JointState(
                header=Header(stamp=stamp),
                position=[value] * count,
                velocity=[0.5] * count,
                effort=[0.2] * count,
            ),
            None,
        )
    values = []

    def collect(message):
        values.append(message)
        robot._stop_event.set()

    monkeypatch.setattr(robot.motor_states, "publish", collect)
    robot._publish_loop()
    state = values[0]
    assert list(state.position) == [1.0] * 4 + [2.0] * 7 + [3.0] * 7
    assert list(state.velocity) == [0.5] * 18
    assert list(state.name) == connection.R1PRO_UPPER_BODY_JOINTS
    assert state.header.stamp == oldest
    assert JointState.decode(state.encode()) == state


@pytest.fixture()
def command_sink(robot, monkeypatch, mocker):
    ros = mocker.Mock(spec=RawROS)
    monkeypatch.setattr(robot, "_ros", ros)
    monkeypatch.setattr(connection, "dimos_to_ros", lambda msg, _: msg)
    monkeypatch.setattr(connection, "header_now", lambda: Header(stamp=Time(sec=9, nanosec=123)))
    for name in ("_cmd_torso_topic", "_cmd_left_topic", "_cmd_right_topic", "_speed_topic"):
        monkeypatch.setattr(robot, name, RawROSTopic(name, JointState))
    return ros.publish


def motor_command():
    return MotorCommandArray(
        q=list(range(18)), dq=[0.0, VEL_STOP, 0.2] * 6, kp=[0.0] * 18, kd=[0.0] * 18, tau=[0.0] * 18
    )


def test_motor_command_splits_generated_sequences_and_tracking_speed(robot, command_sink):
    robot._on_motor_command(motor_command())
    messages = [call.args[1] for call in command_sink.call_args_list]
    assert [list(msg.position) for msg in messages] == [
        list(range(4)),
        list(range(4, 11)),
        list(range(11, 18)),
    ]
    assert list(messages[0].velocity) == [0.5, 0.5, 0.2, 0.5]
    assert all(msg.header.stamp == Time(sec=9, nanosec=123) for msg in messages)


@pytest.mark.parametrize("field", ["q", "dq", "kp", "kd", "tau"])
def test_malformed_motor_array_never_reaches_ros(robot, command_sink, field):
    message = motor_command()
    setattr(message, field, [0.0])
    robot._on_motor_command(message)
    command_sink.assert_not_called()


def test_base_command_only_forwards_supported_axes(robot, command_sink):
    robot._on_cmd_vel(
        Twist(linear=Vector3(x=1.0, y=2.0, z=3.0), angular=Vector3(x=4.0, y=5.0, z=6.0))
    )
    message = command_sink.call_args.args[1]
    assert message == TwistStamped(
        header=Header(stamp=Time(sec=9, nanosec=123)),
        twist=Twist(linear=Vector3(x=1.0, y=2.0), angular=Vector3(z=6.0)),
    )
