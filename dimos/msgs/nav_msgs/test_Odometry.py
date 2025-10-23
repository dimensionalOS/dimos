# Copyright 2025 Dimensional Inc.
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
import pytest

try:
    from builtin_interfaces.msg import Time as ROSTime
    from geometry_msgs.msg import (
        Point as ROSPoint,
        Pose as ROSPose,
        PoseWithCovariance as ROSPoseWithCovariance,
        Quaternion as ROSQuaternion,
        Twist as ROSTwist,
        TwistWithCovariance as ROSTwistWithCovariance,
        Vector3 as ROSVector3,
    )
    from nav_msgs.msg import Odometry as ROSOdometry
    from std_msgs.msg import Header as ROSHeader
except ImportError:
    ROSTwist = None
    ROSHeader = None
    ROSPose = None
    ROSPoseWithCovariance = None
    ROSQuaternion = None
    ROSOdometry = None
    ROSPoint = None
    ROSTime = None
    ROSTwistWithCovariance = None
    ROSVector3 = None


from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseWithCovariance import PoseWithCovariance
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistWithCovariance import TwistWithCovariance
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry


def test_odometry_default_init() -> None:
    """Test default initialization."""
    if ROSVector3 is None:
        pytest.skip("ROS not available")
    if ROSTwistWithCovariance is None:
        pytest.skip("ROS not available")
    if ROSTime is None:
        pytest.skip("ROS not available")
    if ROSPoint is None:
        pytest.skip("ROS not available")
    if ROSOdometry is None:
        pytest.skip("ROS not available")
    if ROSQuaternion is None:
        pytest.skip("ROS not available")
    if ROSPoseWithCovariance is None:
        pytest.skip("ROS not available")
    if ROSPose is None:
        pytest.skip("ROS not available")
    if ROSHeader is None:
        pytest.skip("ROS not available")
    if ROSTwist is None:
        pytest.skip("ROS not available")
    odom = Odometry()

    # Should have current timestamp
    assert odom.ts > 0
    assert odom.frame_id == ""
    assert odom.child_frame_id == ""

    # Pose should be at origin with identity orientation
    assert odom.pose.position.x == 0.0
    assert odom.pose.position.y == 0.0
    assert odom.pose.position.z == 0.0
    assert odom.pose.orientation.w == 1.0

    # Twist should be zero
    assert odom.twist.linear.x == 0.0
    assert odom.twist.linear.y == 0.0
    assert odom.twist.linear.z == 0.0
    assert odom.twist.angular.x == 0.0
    assert odom.twist.angular.y == 0.0
    assert odom.twist.angular.z == 0.0

    # Covariances should be zero
    assert np.all(odom.pose.covariance == 0.0)
    assert np.all(odom.twist.covariance == 0.0)


def test_odometry_with_frames() -> None:
    """Test initialization with frame IDs."""
    ts = 1234567890.123456
    frame_id = "odom"
    child_frame_id = "base_link"

    odom = Odometry(ts=ts, frame_id=frame_id, child_frame_id=child_frame_id)

    assert odom.ts == ts
    assert odom.frame_id == frame_id
    assert odom.child_frame_id == child_frame_id


def test_odometry_with_pose_and_twist() -> None:
    """Test initialization with pose and twist."""
    pose = Pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.9)
    twist = Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.1))

    odom = Odometry(ts=1000.0, frame_id="odom", child_frame_id="base_link", pose=pose, twist=twist)

    assert odom.pose.pose.position.x == 1.0
    assert odom.pose.pose.position.y == 2.0
    assert odom.pose.pose.position.z == 3.0
    assert odom.twist.twist.linear.x == 0.5
    assert odom.twist.twist.angular.z == 0.1


def test_odometry_with_covariances() -> None:
    """Test initialization with pose and twist with covariances."""
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


def test_odometry_copy_constructor() -> None:
    """Test copy constructor."""
    original = Odometry(
        ts=1000.0,
        frame_id="odom",
        child_frame_id="base_link",
        pose=Pose(1.0, 2.0, 3.0),
        twist=Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.1)),
    )

    copy = Odometry(original)

    assert copy == original
    assert copy is not original
    assert copy.pose is not original.pose
    assert copy.twist is not original.twist


def test_odometry_dict_init() -> None:
    """Test initialization from dictionary."""
    odom_dict = {
        "ts": 1000.0,
        "frame_id": "odom",
        "child_frame_id": "base_link",
        "pose": Pose(1.0, 2.0, 3.0),
        "twist": Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.1)),
    }

    odom = Odometry(odom_dict)

    assert odom.ts == 1000.0
    assert odom.frame_id == "odom"
    assert odom.child_frame_id == "base_link"
    assert odom.pose.position.x == 1.0
    assert odom.twist.linear.x == 0.5


def test_odometry_properties() -> None:
    """Test convenience properties."""
    pose = Pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.9)
    twist = Twist(Vector3(0.5, 0.6, 0.7), Vector3(0.1, 0.2, 0.3))

    odom = Odometry(ts=1000.0, frame_id="odom", child_frame_id="base_link", pose=pose, twist=twist)

    # Position properties
    assert odom.x == 1.0
    assert odom.y == 2.0
    assert odom.z == 3.0
    assert odom.position.x == 1.0
    assert odom.position.y == 2.0
    assert odom.position.z == 3.0

    # Orientation properties
    assert odom.orientation.x == 0.1
    assert odom.orientation.y == 0.2
    assert odom.orientation.z == 0.3
    assert odom.orientation.w == 0.9

    # Velocity properties
    assert odom.vx == 0.5
    assert odom.vy == 0.6
    assert odom.vz == 0.7
    assert odom.linear_velocity.x == 0.5
    assert odom.linear_velocity.y == 0.6
    assert odom.linear_velocity.z == 0.7

    # Angular velocity properties
    assert odom.wx == 0.1
    assert odom.wy == 0.2
    assert odom.wz == 0.3
    assert odom.angular_velocity.x == 0.1
    assert odom.angular_velocity.y == 0.2
    assert odom.angular_velocity.z == 0.3

    # Euler angles
    assert odom.roll == pose.roll
    assert odom.pitch == pose.pitch
    assert odom.yaw == pose.yaw


def test_odometry_str_repr() -> None:
    """Test string representations."""
    odom = Odometry(
        ts=1234567890.123456,
        frame_id="odom",
        child_frame_id="base_link",
        pose=Pose(1.234, 2.567, 3.891),
        twist=Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.1)),
    )

    repr_str = repr(odom)
    assert "Odometry" in repr_str
    assert "1234567890.123456" in repr_str
    assert "odom" in repr_str
    assert "base_link" in repr_str

    str_repr = str(odom)
    assert "Odometry" in str_repr
    assert "odom -> base_link" in str_repr
    assert "1.234" in str_repr
    assert "0.500" in str_repr


def test_odometry_equality() -> None:
    """Test equality comparison."""
    odom1 = Odometry(
        ts=1000.0,
        frame_id="odom",
        child_frame_id="base_link",
        pose=Pose(1.0, 2.0, 3.0),
        twist=Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.1)),
    )

    odom2 = Odometry(
        ts=1000.0,
        frame_id="odom",
        child_frame_id="base_link",
        pose=Pose(1.0, 2.0, 3.0),
        twist=Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.1)),
    )

    odom3 = Odometry(
        ts=1000.0,
        frame_id="odom",
        child_frame_id="base_link",
        pose=Pose(1.1, 2.0, 3.0),  # Different position
        twist=Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.1)),
    )

    assert odom1 == odom2
    assert odom1 != odom3
    assert odom1 != "not an odometry"


def test_odometry_lcm_encode_decode() -> None:
    """Test LCM encoding and decoding."""
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

    # Encode and decode
    binary_msg = source.lcm_encode()
    decoded = Odometry.lcm_decode(binary_msg)

    # Check values (allowing for timestamp precision loss)
    assert abs(decoded.ts - source.ts) < 1e-6
    assert decoded.frame_id == source.frame_id
    assert decoded.child_frame_id == source.child_frame_id
    assert decoded.pose == source.pose
    assert decoded.twist == source.twist


@pytest.mark.ros
def test_odometry_from_ros_msg() -> None:
    """Test creating from ROS message."""
    ros_msg = ROSOdometry()

    # Set header
    ros_msg.header = ROSHeader()
    ros_msg.header.stamp = ROSTime()
    ros_msg.header.stamp.sec = 1234567890
    ros_msg.header.stamp.nanosec = 123456000
    ros_msg.header.frame_id = "odom"
    ros_msg.child_frame_id = "base_link"

    # Set pose with covariance
    ros_msg.pose = ROSPoseWithCovariance()
    ros_msg.pose.pose = ROSPose()
    ros_msg.pose.pose.position = ROSPoint(x=1.0, y=2.0, z=3.0)
    ros_msg.pose.pose.orientation = ROSQuaternion(x=0.1, y=0.2, z=0.3, w=0.9)
    ros_msg.pose.covariance = [float(i) for i in range(36)]

    # Set twist with covariance
    ros_msg.twist = ROSTwistWithCovariance()
    ros_msg.twist.twist = ROSTwist()
    ros_msg.twist.twist.linear = ROSVector3(x=0.5, y=0.6, z=0.7)
    ros_msg.twist.twist.angular = ROSVector3(x=0.1, y=0.2, z=0.3)
    ros_msg.twist.covariance = [float(i) for i in range(36, 72)]

    odom = Odometry.from_ros_msg(ros_msg)

    assert odom.ts == 1234567890.123456
    assert odom.frame_id == "odom"
    assert odom.child_frame_id == "base_link"
    assert odom.pose.position.x == 1.0
    assert odom.twist.linear.x == 0.5
    assert np.array_equal(odom.pose.covariance, np.arange(36))
    assert np.array_equal(odom.twist.covariance, np.arange(36, 72))


@pytest.mark.ros
def test_odometry_to_ros_msg() -> None:
    """Test converting to ROS message."""
    pose = Pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.9)
    pose_cov = np.arange(36, dtype=float)
    twist = Twist(Vector3(0.5, 0.6, 0.7), Vector3(0.1, 0.2, 0.3))
    twist_cov = np.arange(36, 72, dtype=float)

    odom = Odometry(
        ts=1234567890.567890,
        frame_id="odom",
        child_frame_id="base_link",
        pose=PoseWithCovariance(pose, pose_cov),
        twist=TwistWithCovariance(twist, twist_cov),
    )

    ros_msg = odom.to_ros_msg()

    assert isinstance(ros_msg, ROSOdometry)
    assert ros_msg.header.frame_id == "odom"
    assert ros_msg.header.stamp.sec == 1234567890
    assert abs(ros_msg.header.stamp.nanosec - 567890000) < 100  # Allow small rounding error
    assert ros_msg.child_frame_id == "base_link"

    # Check pose
    assert ros_msg.pose.pose.position.x == 1.0
    assert ros_msg.pose.pose.position.y == 2.0
    assert ros_msg.pose.pose.position.z == 3.0
    assert ros_msg.pose.pose.orientation.x == 0.1
    assert ros_msg.pose.pose.orientation.y == 0.2
    assert ros_msg.pose.pose.orientation.z == 0.3
    assert ros_msg.pose.pose.orientation.w == 0.9
    assert list(ros_msg.pose.covariance) == list(range(36))

    # Check twist
    assert ros_msg.twist.twist.linear.x == 0.5
    assert ros_msg.twist.twist.linear.y == 0.6
    assert ros_msg.twist.twist.linear.z == 0.7
    assert ros_msg.twist.twist.angular.x == 0.1
    assert ros_msg.twist.twist.angular.y == 0.2
    assert ros_msg.twist.twist.angular.z == 0.3
    assert list(ros_msg.twist.covariance) == list(range(36, 72))


@pytest.mark.ros
def test_odometry_ros_roundtrip() -> None:
    """Test round-trip conversion with ROS messages."""
    pose = Pose(1.5, 2.5, 3.5, 0.15, 0.25, 0.35, 0.85)
    pose_cov = np.random.rand(36)
    twist = Twist(Vector3(0.55, 0.65, 0.75), Vector3(0.15, 0.25, 0.35))
    twist_cov = np.random.rand(36)

    original = Odometry(
        ts=2147483647.987654,  # Max int32 value for ROS Time.sec
        frame_id="world",
        child_frame_id="robot",
        pose=PoseWithCovariance(pose, pose_cov),
        twist=TwistWithCovariance(twist, twist_cov),
    )

    ros_msg = original.to_ros_msg()
    restored = Odometry.from_ros_msg(ros_msg)

    # Check values (allowing for timestamp precision loss)
    assert abs(restored.ts - original.ts) < 1e-6
    assert restored.frame_id == original.frame_id
    assert restored.child_frame_id == original.child_frame_id
    assert restored.pose == original.pose
    assert restored.twist == original.twist


def test_odometry_zero_timestamp() -> None:
    """Test that zero timestamp gets replaced with current time."""
    odom = Odometry(ts=0.0)

    # Should have been replaced with current time
    assert odom.ts > 0
    assert odom.ts <= time.time()


def test_odometry_with_just_pose() -> None:
    """Test initialization with just a Pose (no covariance)."""
    pose = Pose(1.0, 2.0, 3.0)

    odom = Odometry(pose=pose)

    assert odom.pose.position.x == 1.0
    assert odom.pose.position.y == 2.0
    assert odom.pose.position.z == 3.0
    assert np.all(odom.pose.covariance == 0.0)  # Should have zero covariance
    assert np.all(odom.twist.covariance == 0.0)  # Twist should also be zero


def test_odometry_with_just_twist() -> None:
    """Test initialization with just a Twist (no covariance)."""
    twist = Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.1))

    odom = Odometry(twist=twist)

    assert odom.twist.linear.x == 0.5
    assert odom.twist.angular.z == 0.1
    assert np.all(odom.twist.covariance == 0.0)  # Should have zero covariance
    assert np.all(odom.pose.covariance == 0.0)  # Pose should also be zero


@pytest.mark.ros
@pytest.mark.parametrize(
    "frame_id,child_frame_id",
    [
        ("odom", "base_link"),
        ("map", "odom"),
        ("world", "robot"),
        ("base_link", "camera_link"),
        ("", ""),  # Empty frames
    ],
)
def test_odometry_frame_combinations(frame_id, child_frame_id) -> None:
    """Test various frame ID combinations."""
    odom = Odometry(frame_id=frame_id, child_frame_id=child_frame_id)

    assert odom.frame_id == frame_id
    assert odom.child_frame_id == child_frame_id

    # Test roundtrip through ROS
    ros_msg = odom.to_ros_msg()
    assert ros_msg.header.frame_id == frame_id
    assert ros_msg.child_frame_id == child_frame_id

    restored = Odometry.from_ros_msg(ros_msg)
    assert restored.frame_id == frame_id
    assert restored.child_frame_id == child_frame_id


def test_odometry_typical_robot_scenario() -> None:
    """Test a typical robot odometry scenario."""
    # Robot moving forward at 0.5 m/s with slight rotation
    odom = Odometry(
        ts=1000.0,
        frame_id="odom",
        child_frame_id="base_footprint",
        pose=Pose(10.0, 5.0, 0.0, 0.0, 0.0, np.sin(0.1), np.cos(0.1)),  # 0.2 rad yaw
        twist=Twist(
            Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.05)
        ),  # Moving forward, turning slightly
    )

    # Check we can access all the typical properties
    assert odom.x == 10.0
    assert odom.y == 5.0
    assert odom.z == 0.0
    assert abs(odom.yaw - 0.2) < 0.01  # Approximately 0.2 radians
    assert odom.vx == 0.5  # Forward velocity
    assert odom.wz == 0.05  # Yaw rate
