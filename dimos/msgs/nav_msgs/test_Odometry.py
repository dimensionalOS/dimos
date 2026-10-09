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

from dataclasses import asdict
import time

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from dimos_generated.nav_msgs.msg import Odometry
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
from rosbags.typesys import Stores, get_typestore

from dimos.msgs.geometry import quaternion_euler
from dimos.msgs.time import header_now, time_from_seconds, to_seconds


def test_odometry_default_init() -> None:
    odom = Odometry(
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        child_frame_id="",
        pose=PoseWithCovariance(
            pose=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            covariance=np.zeros(36, dtype=np.float64),
        ),
        twist=TwistWithCovariance(
            twist=Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
            covariance=np.zeros(36, dtype=np.float64),
        ),
    )
    assert to_seconds(odom.header.stamp) == 0
    assert odom.header.frame_id == ""
    assert odom.child_frame_id == ""
    assert odom.pose.pose.position.x == 0.0
    assert odom.pose.pose.position.y == 0.0
    assert odom.pose.pose.position.z == 0.0
    assert odom.pose.pose.orientation.w == 1.0
    assert odom.twist.twist.linear.x == 0.0
    assert odom.twist.twist.angular.x == 0.0
    assert np.all(np.asarray(odom.pose.covariance) == 0.0)
    assert np.all(np.asarray(odom.twist.covariance) == 0.0)


def test_odometry_with_frames() -> None:
    ts = 1234567890.123456
    odom = Odometry(
        child_frame_id="base_link",
        header=Header(stamp=time_from_seconds(ts), frame_id="odom"),
        pose=PoseWithCovariance(
            pose=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            covariance=np.zeros(36, dtype=np.float64),
        ),
        twist=TwistWithCovariance(
            twist=Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
            covariance=np.zeros(36, dtype=np.float64),
        ),
    )
    assert to_seconds(odom.header.stamp) == ts
    assert odom.header.frame_id == "odom"
    assert odom.child_frame_id == "base_link"


def test_odometry_with_pose_and_twist() -> None:
    pose = Pose(
        position=Point(x=1.0, y=2.0, z=3.0), orientation=Quaternion(x=0.1, y=0.2, z=0.3, w=0.9)
    )
    twist = Twist(linear=Vector3(x=0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.1))
    odom = Odometry(
        child_frame_id="base_link",
        pose=PoseWithCovariance(pose=pose, covariance=np.zeros(36, dtype=np.float64)),
        twist=TwistWithCovariance(twist=twist, covariance=np.zeros(36, dtype=np.float64)),
        header=Header(stamp=time_from_seconds(1000.0), frame_id="odom"),
    )
    assert odom.pose.pose.position.x == 1.0
    assert odom.pose.pose.position.y == 2.0
    assert odom.pose.pose.position.z == 3.0
    assert odom.twist.twist.linear.x == 0.5
    assert odom.twist.twist.angular.z == 0.1


def test_odometry_with_covariances() -> None:
    pose = Pose(
        position=Point(x=1.0, y=2.0, z=3.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )
    pose_cov = np.arange(36, dtype=float)
    pose_with_cov = PoseWithCovariance(pose=pose, covariance=np.asarray(pose_cov, dtype=np.float64))
    twist = Twist(linear=Vector3(x=0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.1))
    twist_cov = np.arange(36, 72, dtype=float)
    twist_with_cov = TwistWithCovariance(
        twist=twist, covariance=np.asarray(twist_cov, dtype=np.float64)
    )
    odom = Odometry(
        child_frame_id="base_link",
        pose=pose_with_cov,
        twist=twist_with_cov,
        header=Header(stamp=time_from_seconds(1000.0), frame_id="odom"),
    )
    assert odom.pose.pose.position.x == 1.0
    assert np.array_equal(odom.pose.covariance, pose_cov)
    assert odom.twist.twist.linear.x == 0.5
    assert np.array_equal(odom.twist.covariance, twist_cov)


def test_odometry_properties() -> None:
    pose = Pose(
        position=Point(x=1.0, y=2.0, z=3.0), orientation=Quaternion(x=0.1, y=0.2, z=0.3, w=0.9)
    )
    twist = Twist(linear=Vector3(x=0.5, y=0.6, z=0.7), angular=Vector3(x=0.1, y=0.2, z=0.3))
    odom = Odometry(
        child_frame_id="base_link",
        pose=PoseWithCovariance(pose=pose, covariance=np.zeros(36, dtype=np.float64)),
        twist=TwistWithCovariance(twist=twist, covariance=np.zeros(36, dtype=np.float64)),
        header=Header(stamp=time_from_seconds(1000.0), frame_id="odom"),
    )
    assert odom.pose.pose.position.x == 1.0
    assert odom.pose.pose.position.y == 2.0
    assert odom.pose.pose.position.z == 3.0
    assert odom.pose.pose.position.x == 1.0
    assert odom.pose.pose.orientation.x == 0.1
    assert odom.twist.twist.linear.x == 0.5
    assert odom.twist.twist.linear.y == 0.6
    assert odom.twist.twist.linear.z == 0.7
    assert odom.twist.twist.linear.x == 0.5
    assert odom.twist.twist.angular.x == 0.1
    assert odom.twist.twist.angular.y == 0.2
    assert odom.twist.twist.angular.z == 0.3
    assert odom.twist.twist.angular.x == 0.1
    assert quaternion_euler(odom.pose.pose.orientation)[0] == quaternion_euler(pose.orientation)[0]
    assert quaternion_euler(odom.pose.pose.orientation)[1] == quaternion_euler(pose.orientation)[1]
    assert quaternion_euler(odom.pose.pose.orientation)[2] == quaternion_euler(pose.orientation)[2]


def test_independent_ros_decoding() -> None:
    source = Odometry(
        header=Header(stamp=time_from_seconds(1000), frame_id="odom"),
        child_frame_id="base_link",
        pose=PoseWithCovariance(
            pose=Pose(
                position=Point(x=1.234, y=2.567, z=3.891),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            covariance=np.zeros(36, dtype=np.float64),
        ),
        twist=TwistWithCovariance(
            twist=Twist(linear=Vector3(x=0.5, y=0.0, z=0.0), angular=Vector3(z=0.1, x=0.0, y=0.0)),
            covariance=np.zeros(36, dtype=np.float64),
        ),
    )
    decoded = get_typestore(Stores.ROS2_JAZZY).deserialize_cdr(
        cdr_encode(source), Odometry.__msgtype__
    )
    assert decoded.header.frame_id == "odom"
    assert decoded.child_frame_id == "base_link"
    assert (
        decoded.pose.pose.position.x,
        decoded.pose.pose.position.y,
        decoded.pose.pose.position.z,
    ) == (1.234, 2.567, 3.891)
    assert decoded.twist.twist.linear.x == 0.5
    assert decoded.twist.twist.angular.z == 0.1


def test_odometry_equality() -> None:
    kwargs = dict(
        child_frame_id="base_link",
        pose=PoseWithCovariance(
            pose=Pose(
                position=Point(x=1.0, y=2.0, z=3.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            covariance=np.zeros(36, dtype=np.float64),
        ),
        twist=TwistWithCovariance(
            twist=Twist(linear=Vector3(x=0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.1)),
            covariance=np.zeros(36, dtype=np.float64),
        ),
        header=Header(stamp=time_from_seconds(1000.0), frame_id="odom"),
    )
    assert Odometry(**kwargs) == Odometry(**kwargs)
    assert Odometry(**kwargs) != Odometry(
        **{
            **kwargs,
            "pose": PoseWithCovariance(
                pose=Pose(
                    position=Point(x=1.1, y=2.0, z=3.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                covariance=np.zeros(36, dtype=np.float64),
            ),
        }
    )
    assert Odometry(**kwargs) != "not an odometry"


def test_odometry_cdr_roundtrip() -> None:
    pose = Pose(
        position=Point(x=1.0, y=2.0, z=3.0), orientation=Quaternion(x=0.1, y=0.2, z=0.3, w=0.9)
    )
    pose_cov = np.arange(36, dtype=float)
    twist = Twist(linear=Vector3(x=0.5, y=0.6, z=0.7), angular=Vector3(x=0.1, y=0.2, z=0.3))
    twist_cov = np.arange(36, 72, dtype=float)
    source = Odometry(
        child_frame_id="base_link",
        pose=PoseWithCovariance(pose=pose, covariance=np.asarray(pose_cov, dtype=np.float64)),
        twist=TwistWithCovariance(twist=twist, covariance=np.asarray(twist_cov, dtype=np.float64)),
        header=Header(stamp=time_from_seconds(1234567890.123456), frame_id="odom"),
    )
    decoded = cdr_decode(cdr_encode(source), Odometry)
    assert abs(to_seconds(decoded.header.stamp) - to_seconds(source.header.stamp)) < 1e-06
    assert decoded.header.frame_id == source.header.frame_id
    assert decoded.child_frame_id == source.child_frame_id
    np.testing.assert_equal(asdict(decoded.pose), asdict(source.pose))
    np.testing.assert_equal(asdict(decoded.twist), asdict(source.twist))


def test_odometry_zero_timestamp() -> None:
    assert (
        to_seconds(
            Odometry(
                header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
                child_frame_id="",
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(x=0.0, y=0.0, z=0.0),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                    covariance=np.zeros(36, dtype=np.float64),
                ),
                twist=TwistWithCovariance(
                    twist=Twist(
                        linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)
                    ),
                    covariance=np.zeros(36, dtype=np.float64),
                ),
            ).header.stamp
        )
        == 0
    )
    before = time.time()
    assert (
        before
        <= to_seconds(
            Odometry(
                header=header_now(),
                child_frame_id="",
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(x=0.0, y=0.0, z=0.0),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                    covariance=np.zeros(36, dtype=np.float64),
                ),
                twist=TwistWithCovariance(
                    twist=Twist(
                        linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)
                    ),
                    covariance=np.zeros(36, dtype=np.float64),
                ),
            ).header.stamp
        )
        <= time.time()
    )


def test_odometry_with_just_pose() -> None:
    odom = Odometry(
        pose=PoseWithCovariance(
            pose=Pose(
                position=Point(x=1.0, y=2.0, z=3.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            covariance=np.zeros(36, dtype=np.float64),
        ),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        child_frame_id="",
        twist=TwistWithCovariance(
            twist=Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)),
            covariance=np.zeros(36, dtype=np.float64),
        ),
    )
    assert odom.pose.pose.position.x == 1.0
    assert np.all(np.asarray(odom.pose.covariance) == 0.0)
    assert np.all(np.asarray(odom.twist.covariance) == 0.0)


def test_odometry_with_just_twist() -> None:
    odom = Odometry(
        twist=TwistWithCovariance(
            twist=Twist(linear=Vector3(x=0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.1)),
            covariance=np.zeros(36, dtype=np.float64),
        ),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        child_frame_id="",
        pose=PoseWithCovariance(
            pose=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            covariance=np.zeros(36, dtype=np.float64),
        ),
    )
    assert odom.twist.twist.linear.x == 0.5
    assert odom.twist.twist.angular.z == 0.1
    assert np.all(np.asarray(odom.twist.covariance) == 0.0)


def test_odometry_typical_robot_scenario() -> None:
    odom = Odometry(
        child_frame_id="base_footprint",
        pose=PoseWithCovariance(
            pose=Pose(
                position=Point(x=10.0, y=5.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=np.sin(0.1), w=np.cos(0.1)),
            ),
            covariance=np.zeros(36, dtype=np.float64),
        ),
        twist=TwistWithCovariance(
            twist=Twist(linear=Vector3(x=0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.05)),
            covariance=np.zeros(36, dtype=np.float64),
        ),
        header=Header(stamp=time_from_seconds(1000.0), frame_id="odom"),
    )
    assert odom.pose.pose.position.x == 10.0
    assert odom.pose.pose.position.y == 5.0
    assert abs(quaternion_euler(odom.pose.pose.orientation)[2] - 0.2) < 0.01
    assert odom.twist.twist.linear.x == 0.5
    assert odom.twist.twist.angular.z == 0.05
