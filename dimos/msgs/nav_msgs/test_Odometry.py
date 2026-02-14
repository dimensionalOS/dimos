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

import time

import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseWithCovariance import PoseWithCovariance
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistWithCovariance import TwistWithCovariance
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry


def test_odometry_default_init() -> None:
    odom = Odometry()

    assert odom.ts > 0
    assert odom.frame_id == ""
    assert odom.child_frame_id == ""

    assert odom.pose.position.x == 0.0
    assert odom.pose.position.y == 0.0
    assert odom.pose.position.z == 0.0
    assert odom.pose.orientation.w == 1.0

    assert odom.twist.linear.x == 0.0
    assert odom.twist.angular.x == 0.0

    assert np.all(odom.pose.covariance == 0.0)
    assert np.all(odom.twist.covariance == 0.0)


def test_odometry_with_frames() -> None:
    ts = 1234567890.123456
    odom = Odometry(ts=ts, frame_id="odom", child_frame_id="base_link")

    assert odom.ts == ts
    assert odom.frame_id == "odom"
    assert odom.child_frame_id == "base_link"


def test_odometry_with_pose_and_twist() -> None:
    pose = Pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.9)
    twist = Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.1))

    odom = Odometry(ts=1000.0, frame_id="odom", child_frame_id="base_link", pose=pose, twist=twist)

    assert odom.pose.pose.position.x == 1.0
    assert odom.pose.pose.position.y == 2.0
    assert odom.pose.pose.position.z == 3.0
    assert odom.twist.twist.linear.x == 0.5
    assert odom.twist.twist.angular.z == 0.1


def test_odometry_with_covariances() -> None:
    pose = Pose(1.0, 2.0, 3.0)
    pose_cov = np.arange(36, dtype=float)
    pose_with_cov = PoseWithCovariance(pose, pose_cov)

    twist = Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.1))
    twist_cov = np.arange(36, 72, dtype=float)
    twist_with_cov = TwistWithCovariance(twist, twist_cov)

    odom = Odometry(
        ts=1000.0,
        frame_id="odom",
        child_frame_id="base_link",
        pose=pose_with_cov,
        twist=twist_with_cov,
    )

    assert odom.pose.position.x == 1.0
    assert np.array_equal(odom.pose.covariance, pose_cov)
    assert odom.twist.linear.x == 0.5
    assert np.array_equal(odom.twist.covariance, twist_cov)


def test_odometry_properties() -> None:
    pose = Pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.9)
    twist = Twist(Vector3(0.5, 0.6, 0.7), Vector3(0.1, 0.2, 0.3))

    odom = Odometry(ts=1000.0, frame_id="odom", child_frame_id="base_link", pose=pose, twist=twist)

    assert odom.x == 1.0
    assert odom.y == 2.0
    assert odom.z == 3.0
    assert odom.position.x == 1.0

    assert odom.orientation.x == 0.1

    assert odom.vx == 0.5
    assert odom.vy == 0.6
    assert odom.vz == 0.7
    assert odom.linear_velocity.x == 0.5

    assert odom.wx == 0.1
    assert odom.wy == 0.2
    assert odom.wz == 0.3
    assert odom.angular_velocity.x == 0.1

    assert odom.roll == pose.roll
    assert odom.pitch == pose.pitch
    assert odom.yaw == pose.yaw


def test_odometry_str_repr() -> None:
    odom = Odometry(
        ts=1234567890.123456,
        frame_id="odom",
        child_frame_id="base_link",
        pose=Pose(1.234, 2.567, 3.891),
        twist=Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.1)),
    )

    assert "Odometry" in repr(odom)
    assert "1234567890.123456" in repr(odom)

    s = str(odom)
    assert "odom -> base_link" in s
    assert "1.234" in s


def test_odometry_equality() -> None:
    kwargs = dict(
        ts=1000.0,
        frame_id="odom",
        child_frame_id="base_link",
        pose=Pose(1.0, 2.0, 3.0),
        twist=Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.1)),
    )

    assert Odometry(**kwargs) == Odometry(**kwargs)
    assert Odometry(**kwargs) != Odometry(**{**kwargs, "pose": Pose(1.1, 2.0, 3.0)})
    assert Odometry(**kwargs) != "not an odometry"


def test_odometry_lcm_roundtrip() -> None:
    pose = Pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.9)
    pose_cov = np.arange(36, dtype=float)
    twist = Twist(Vector3(0.5, 0.6, 0.7), Vector3(0.1, 0.2, 0.3))
    twist_cov = np.arange(36, 72, dtype=float)

    source = Odometry(
        ts=1234567890.123456,
        frame_id="odom",
        child_frame_id="base_link",
        pose=PoseWithCovariance(pose, pose_cov),
        twist=TwistWithCovariance(twist, twist_cov),
    )

    decoded = Odometry.lcm_decode(source.lcm_encode())

    assert abs(decoded.ts - source.ts) < 1e-6
    assert decoded.frame_id == source.frame_id
    assert decoded.child_frame_id == source.child_frame_id
    assert decoded.pose == source.pose
    assert decoded.twist == source.twist


def test_odometry_zero_timestamp() -> None:
    odom = Odometry(ts=0.0)
    assert odom.ts > 0
    assert odom.ts <= time.time()


def test_odometry_with_just_pose() -> None:
    odom = Odometry(pose=Pose(1.0, 2.0, 3.0))

    assert odom.pose.position.x == 1.0
    assert np.all(odom.pose.covariance == 0.0)
    assert np.all(odom.twist.covariance == 0.0)


def test_odometry_with_just_twist() -> None:
    odom = Odometry(twist=Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.1)))

    assert odom.twist.linear.x == 0.5
    assert odom.twist.angular.z == 0.1
    assert np.all(odom.twist.covariance == 0.0)


def test_odometry_typical_robot_scenario() -> None:
    odom = Odometry(
        ts=1000.0,
        frame_id="odom",
        child_frame_id="base_footprint",
        pose=Pose(10.0, 5.0, 0.0, 0.0, 0.0, np.sin(0.1), np.cos(0.1)),
        twist=Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.05)),
    )

    assert odom.x == 10.0
    assert odom.y == 5.0
    assert abs(odom.yaw - 0.2) < 0.01
    assert odom.vx == 0.5
    assert odom.wz == 0.05
