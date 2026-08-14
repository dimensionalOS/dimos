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

import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.navigation.basic_path_follower.module import BasicPathFollower, lookahead_distance
from dimos.protocol.tf.tf import MultiTBuffer

MOUNT_Z = 0.163


class FakeTF(MultiTBuffer):
    def dispose(self) -> None:
        pass


def _odom() -> Odometry:
    return Odometry(
        ts=1.0,
        frame_id="odom",
        child_frame_id="mid360_link",
        pose=Pose(Vector3(1.0, 2.0, 3.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
    )


def test_on_odometry_steers_from_the_base_pose() -> None:
    tf = FakeTF()
    tf.receive_transform(
        Transform(
            translation=Vector3(0.0, 0.0, MOUNT_Z),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id="mid360_link",
            ts=1.0,
        )
    )
    module = BasicPathFollower()
    module._tf = tf
    try:
        module._on_odometry(_odom())
        assert module._current_pose is not None
        assert abs(module._current_pose.position.z - (3.0 - MOUNT_Z)) < 1e-9
    finally:
        module.stop()


def test_on_odometry_drops_frames_without_the_mount_tf() -> None:
    module = BasicPathFollower()
    module._tf = FakeTF()
    try:
        module._on_odometry(_odom())
        assert module._current_pose is None
    finally:
        module.stop()


def test_lookahead_floor_at_low_speed() -> None:
    assert lookahead_distance(0.1, 1.5, 0.4, 1.5) == 0.4


def test_lookahead_scales_in_linear_region() -> None:
    assert lookahead_distance(0.5, 1.5, 0.4, 1.5) == 0.75


def test_lookahead_clamped_at_ceiling() -> None:
    assert lookahead_distance(2.0, 1.5, 0.4, 1.5) == 1.5


def test_direct_base_pose_is_accepted_without_a_tf_lookup() -> None:
    module = BasicPathFollower()
    pose = PoseStamped(
        ts=2.0,
        frame_id="world",
        position=[1.0, 2.0, 0.74],
        orientation=[0.0, 0.0, 0.0, 1.0],
    )
    try:
        module._on_base_pose(pose)

        assert module._current_pose is not pose
        assert module._current_pose.position.as_tuple == (1.0, 2.0, 0.74)
        assert module._current_pose.frame_id == "world"
    finally:
        module.stop()


def test_rotate_before_drive_suppresses_forward_motion(mocker) -> None:
    module = BasicPathFollower(
        rotate_before_drive=True,
        drive_heading_tolerance=math.radians(20.0),
        max_angular=0.25,
        min_angular=0.12,
    )
    publish = mocker.patch.object(module.nav_cmd_vel, "publish")
    pose = PoseStamped(position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])
    try:
        module._step(pose, np.array([[0.0, 0.0], [0.0, 1.0]], dtype=np.float32), 0.0)

        command = publish.call_args.args[0]
        assert command.linear.x == 0.0
        assert command.linear.y == 0.0
        assert command.angular.z == 0.25
    finally:
        module.stop()


def test_goal_position_is_followed_by_final_yaw_alignment(mocker) -> None:
    module = BasicPathFollower(
        goal_tolerance=0.06,
        align_goal_yaw=True,
        orientation_tolerance=math.radians(5.0),
        max_angular=0.25,
        min_angular=0.12,
    )
    publish = mocker.patch.object(module.nav_cmd_vel, "publish")
    pose = PoseStamped(position=[1.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])
    try:
        module._step(pose, np.array([[0.0, 0.0], [1.0, 0.0]], dtype=np.float32), math.pi / 2)

        command = publish.call_args.args[0]
        assert command.linear.x == 0.0
        assert command.angular.z == 0.25
    finally:
        module.stop()


def test_near_goal_slowdown_keeps_the_configured_usable_speed_floor(mocker) -> None:
    module = BasicPathFollower(
        speed=0.25,
        min_linear_speed=0.15,
        slowdown_distance=0.4,
        goal_tolerance=0.06,
    )
    publish = mocker.patch.object(module.nav_cmd_vel, "publish")
    pose = PoseStamped(position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0, 1.0])
    try:
        module._step(pose, np.array([[0.0, 0.0], [0.2, 0.0]], dtype=np.float32), 0.0)

        command = publish.call_args.args[0]
        assert command.linear.x == 0.15
        assert command.angular.z == 0.0
    finally:
        module.stop()
