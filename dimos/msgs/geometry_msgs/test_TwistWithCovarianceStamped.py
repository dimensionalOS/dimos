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
        Twist as ROSTwist,
        TwistWithCovariance as ROSTwistWithCovariance,
        TwistWithCovarianceStamped as ROSTwistWithCovarianceStamped,
        Vector3 as ROSVector3,
    )
    from std_msgs.msg import Header as ROSHeader
except ImportError:
    ROSTwistWithCovarianceStamped = None
    ROSTwist = None
    ROSHeader = None
    ROSTime = None
    ROSTwistWithCovariance = None
    ROSVector3 = None


from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistWithCovariance import TwistWithCovariance
from dimos.msgs.geometry_msgs.TwistWithCovarianceStamped import TwistWithCovarianceStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3


def test_twist_with_covariance_stamped_default_init() -> None:
    """Test default initialization."""
    if ROSVector3 is None:
        pytest.skip("ROS not available")
    if ROSTwistWithCovariance is None:
        pytest.skip("ROS not available")
    if ROSTime is None:
        pytest.skip("ROS not available")
    if ROSHeader is None:
        pytest.skip("ROS not available")
    if ROSTwist is None:
        pytest.skip("ROS not available")
    if ROSTwistWithCovarianceStamped is None:
        pytest.skip("ROS not available")
    twist_cov_stamped = TwistWithCovarianceStamped()

    # Should have current timestamp
    assert twist_cov_stamped.ts > 0
    assert twist_cov_stamped.frame_id == ""

    # Twist should be zero
    assert twist_cov_stamped.twist.linear.x == 0.0
    assert twist_cov_stamped.twist.linear.y == 0.0
    assert twist_cov_stamped.twist.linear.z == 0.0
    assert twist_cov_stamped.twist.angular.x == 0.0
    assert twist_cov_stamped.twist.angular.y == 0.0
    assert twist_cov_stamped.twist.angular.z == 0.0

    # Covariance should be all zeros
    assert np.all(twist_cov_stamped.covariance == 0.0)


def test_twist_with_covariance_stamped_with_timestamp() -> None:
    """Test initialization with specific timestamp."""
    ts = 1234567890.123456
    frame_id = "base_link"
    twist_cov_stamped = TwistWithCovarianceStamped(ts=ts, frame_id=frame_id)

    assert twist_cov_stamped.ts == ts
    assert twist_cov_stamped.frame_id == frame_id


def test_twist_with_covariance_stamped_with_twist() -> None:
    """Test initialization with twist."""
    ts = 1234567890.123456
    frame_id = "odom"
    twist = Twist(Vector3(1.0, 2.0, 3.0), Vector3(0.1, 0.2, 0.3))
    covariance = np.arange(36, dtype=float)

    twist_cov_stamped = TwistWithCovarianceStamped(
        ts=ts, frame_id=frame_id, twist=twist, covariance=covariance
    )

    assert twist_cov_stamped.ts == ts
    assert twist_cov_stamped.frame_id == frame_id
    assert twist_cov_stamped.twist.linear.x == 1.0
    assert twist_cov_stamped.twist.linear.y == 2.0
    assert twist_cov_stamped.twist.linear.z == 3.0
    assert np.array_equal(twist_cov_stamped.covariance, covariance)


def test_twist_with_covariance_stamped_with_tuple() -> None:
    """Test initialization with tuple of velocities."""
    ts = 1234567890.123456
    frame_id = "robot_base"
    linear = [1.0, 2.0, 3.0]
    angular = [0.1, 0.2, 0.3]
    covariance = np.arange(36, dtype=float)

    twist_cov_stamped = TwistWithCovarianceStamped(
        ts=ts, frame_id=frame_id, twist=(linear, angular), covariance=covariance
    )

    assert twist_cov_stamped.ts == ts
    assert twist_cov_stamped.frame_id == frame_id
    assert twist_cov_stamped.twist.linear.x == 1.0
    assert twist_cov_stamped.twist.angular.x == 0.1
    assert np.array_equal(twist_cov_stamped.covariance, covariance)


def test_twist_with_covariance_stamped_properties() -> None:
    """Test convenience properties."""
    twist = Twist(Vector3(1.0, 2.0, 3.0), Vector3(0.1, 0.2, 0.3))
    covariance = np.eye(6).flatten()
    twist_cov_stamped = TwistWithCovarianceStamped(
        ts=1234567890.0, frame_id="cmd_vel", twist=twist, covariance=covariance
    )

    # Linear and angular properties
    assert twist_cov_stamped.linear.x == 1.0
    assert twist_cov_stamped.linear.y == 2.0
    assert twist_cov_stamped.linear.z == 3.0
    assert twist_cov_stamped.angular.x == 0.1
    assert twist_cov_stamped.angular.y == 0.2
    assert twist_cov_stamped.angular.z == 0.3

    # Covariance matrix
    cov_matrix = twist_cov_stamped.covariance_matrix
    assert cov_matrix.shape == (6, 6)
    assert np.trace(cov_matrix) == 6.0


def test_twist_with_covariance_stamped_str() -> None:
    """Test string representation."""
    twist = Twist(Vector3(1.234, 2.567, 3.891), Vector3(0.111, 0.222, 0.333))
    covariance = np.eye(6).flatten() * 2.0
    twist_cov_stamped = TwistWithCovarianceStamped(
        ts=1234567890.0, frame_id="world", twist=twist, covariance=covariance
    )

    str_repr = str(twist_cov_stamped)
    assert "TwistWithCovarianceStamped" in str_repr
    assert "1.234" in str_repr
    assert "2.567" in str_repr
    assert "3.891" in str_repr
    assert "cov_trace" in str_repr
    assert "12.000" in str_repr  # Trace of 2*identity is 12


def test_twist_with_covariance_stamped_lcm_encode_decode() -> None:
    """Test LCM encoding and decoding."""
    ts = 1234567890.123456
    frame_id = "camera_link"
    twist = Twist(Vector3(1.0, 2.0, 3.0), Vector3(0.1, 0.2, 0.3))
    covariance = np.arange(36, dtype=float)

    source = TwistWithCovarianceStamped(
        ts=ts, frame_id=frame_id, twist=twist, covariance=covariance
    )

    # Encode and decode
    binary_msg = source.lcm_encode()
    decoded = TwistWithCovarianceStamped.lcm_decode(binary_msg)

    # Check timestamp (may lose some precision)
    assert abs(decoded.ts - ts) < 1e-6
    assert decoded.frame_id == frame_id

    # Check twist
    assert decoded.twist.linear.x == 1.0
    assert decoded.twist.linear.y == 2.0
    assert decoded.twist.linear.z == 3.0
    assert decoded.twist.angular.x == 0.1
    assert decoded.twist.angular.y == 0.2
    assert decoded.twist.angular.z == 0.3

    # Check covariance
    assert np.array_equal(decoded.covariance, covariance)


@pytest.mark.ros
def test_twist_with_covariance_stamped_from_ros_msg() -> None:
    """Test creating from ROS message."""
    ros_msg = ROSTwistWithCovarianceStamped()

    # Set header
    ros_msg.header = ROSHeader()
    ros_msg.header.stamp = ROSTime()
    ros_msg.header.stamp.sec = 1234567890
    ros_msg.header.stamp.nanosec = 123456000
    ros_msg.header.frame_id = "laser"

    # Set twist with covariance
    ros_msg.twist = ROSTwistWithCovariance()
    ros_msg.twist.twist = ROSTwist()
    ros_msg.twist.twist.linear = ROSVector3(x=1.0, y=2.0, z=3.0)
    ros_msg.twist.twist.angular = ROSVector3(x=0.1, y=0.2, z=0.3)
    ros_msg.twist.covariance = [float(i) for i in range(36)]

    twist_cov_stamped = TwistWithCovarianceStamped.from_ros_msg(ros_msg)

    assert twist_cov_stamped.ts == 1234567890.123456
    assert twist_cov_stamped.frame_id == "laser"
    assert twist_cov_stamped.twist.linear.x == 1.0
    assert twist_cov_stamped.twist.linear.y == 2.0
    assert twist_cov_stamped.twist.linear.z == 3.0
    assert twist_cov_stamped.twist.angular.x == 0.1
    assert twist_cov_stamped.twist.angular.y == 0.2
    assert twist_cov_stamped.twist.angular.z == 0.3
    assert np.array_equal(twist_cov_stamped.covariance, np.arange(36))


@pytest.mark.ros
def test_twist_with_covariance_stamped_to_ros_msg() -> None:
    """Test converting to ROS message."""
    ts = 1234567890.567890
    frame_id = "imu"
    twist = Twist(Vector3(1.0, 2.0, 3.0), Vector3(0.1, 0.2, 0.3))
    covariance = np.arange(36, dtype=float)

    twist_cov_stamped = TwistWithCovarianceStamped(
        ts=ts, frame_id=frame_id, twist=twist, covariance=covariance
    )

    ros_msg = twist_cov_stamped.to_ros_msg()

    assert isinstance(ros_msg, ROSTwistWithCovarianceStamped)
    assert ros_msg.header.frame_id == frame_id
    assert ros_msg.header.stamp.sec == 1234567890
    assert abs(ros_msg.header.stamp.nanosec - 567890000) < 100  # Allow small rounding error

    assert ros_msg.twist.twist.linear.x == 1.0
    assert ros_msg.twist.twist.linear.y == 2.0
    assert ros_msg.twist.twist.linear.z == 3.0
    assert ros_msg.twist.twist.angular.x == 0.1
    assert ros_msg.twist.twist.angular.y == 0.2
    assert ros_msg.twist.twist.angular.z == 0.3
    assert list(ros_msg.twist.covariance) == list(range(36))


@pytest.mark.ros
def test_twist_with_covariance_stamped_ros_roundtrip() -> None:
    """Test round-trip conversion with ROS messages."""
    ts = 2147483647.987654  # Max int32 value for ROS Time.sec
    frame_id = "robot_base"
    twist = Twist(Vector3(1.5, 2.5, 3.5), Vector3(0.15, 0.25, 0.35))
    covariance = np.random.rand(36)

    original = TwistWithCovarianceStamped(
        ts=ts, frame_id=frame_id, twist=twist, covariance=covariance
    )

    ros_msg = original.to_ros_msg()
    restored = TwistWithCovarianceStamped.from_ros_msg(ros_msg)

    # Check timestamp (loses some precision in conversion)
    assert abs(restored.ts - ts) < 1e-6
    assert restored.frame_id == frame_id

    # Check twist
    assert restored.twist.linear.x == original.twist.linear.x
    assert restored.twist.linear.y == original.twist.linear.y
    assert restored.twist.linear.z == original.twist.linear.z
    assert restored.twist.angular.x == original.twist.angular.x
    assert restored.twist.angular.y == original.twist.angular.y
    assert restored.twist.angular.z == original.twist.angular.z

    # Check covariance
    assert np.allclose(restored.covariance, original.covariance)


def test_twist_with_covariance_stamped_zero_timestamp() -> None:
    """Test that zero timestamp gets replaced with current time."""
    twist_cov_stamped = TwistWithCovarianceStamped(ts=0.0)

    # Should have been replaced with current time
    assert twist_cov_stamped.ts > 0
    assert twist_cov_stamped.ts <= time.time()


def test_twist_with_covariance_stamped_inheritance() -> None:
    """Test that it properly inherits from TwistWithCovariance and Timestamped."""
    twist = Twist(Vector3(1.0, 2.0, 3.0), Vector3(0.1, 0.2, 0.3))
    covariance = np.eye(6).flatten()
    twist_cov_stamped = TwistWithCovarianceStamped(
        ts=1234567890.0, frame_id="test", twist=twist, covariance=covariance
    )

    # Should be instance of parent classes
    assert isinstance(twist_cov_stamped, TwistWithCovariance)

    # Should have Timestamped attributes
    assert hasattr(twist_cov_stamped, "ts")
    assert hasattr(twist_cov_stamped, "frame_id")

    # Should have TwistWithCovariance attributes
    assert hasattr(twist_cov_stamped, "twist")
    assert hasattr(twist_cov_stamped, "covariance")


def test_twist_with_covariance_stamped_is_zero() -> None:
    """Test is_zero method inheritance."""
    # Zero twist
    twist_cov_stamped1 = TwistWithCovarianceStamped()
    assert twist_cov_stamped1.is_zero()
    assert not twist_cov_stamped1  # Boolean conversion

    # Non-zero twist
    twist = Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
    twist_cov_stamped2 = TwistWithCovarianceStamped(twist=twist)
    assert not twist_cov_stamped2.is_zero()
    assert twist_cov_stamped2  # Boolean conversion


def test_twist_with_covariance_stamped_sec_nsec() -> None:
    """Test the sec_nsec helper function."""
    from dimos.msgs.geometry_msgs.TwistWithCovarianceStamped import sec_nsec

    # Test integer seconds
    s, ns = sec_nsec(1234567890.0)
    assert s == 1234567890
    assert ns == 0

    # Test fractional seconds
    s, ns = sec_nsec(1234567890.123456789)
    assert s == 1234567890
    assert abs(ns - 123456789) < 100  # Allow small rounding error

    # Test small fractional seconds
    s, ns = sec_nsec(0.000000001)
    assert s == 0
    assert ns == 1

    # Test large timestamp
    s, ns = sec_nsec(9999999999.999999999)
    # Due to floating point precision, this might round to 10000000000
    assert s in [9999999999, 10000000000]
    if s == 9999999999:
        assert abs(ns - 999999999) < 10
    else:
        assert ns == 0


@pytest.mark.ros
@pytest.mark.parametrize(
    "frame_id",
    ["", "map", "odom", "base_link", "cmd_vel", "sensor/velocity/front"],
)
def test_twist_with_covariance_stamped_frame_ids(frame_id) -> None:
    """Test various frame ID values."""
    twist_cov_stamped = TwistWithCovarianceStamped(frame_id=frame_id)
    assert twist_cov_stamped.frame_id == frame_id

    # Test roundtrip through ROS
    ros_msg = twist_cov_stamped.to_ros_msg()
    assert ros_msg.header.frame_id == frame_id

    restored = TwistWithCovarianceStamped.from_ros_msg(ros_msg)
    assert restored.frame_id == frame_id


def test_twist_with_covariance_stamped_different_covariances() -> None:
    """Test with different covariance patterns."""
    twist = Twist(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.5))

    # Zero covariance
    zero_cov = np.zeros(36)
    twist_cov1 = TwistWithCovarianceStamped(twist=twist, covariance=zero_cov)
    assert np.all(twist_cov1.covariance == 0.0)

    # Identity covariance
    identity_cov = np.eye(6).flatten()
    twist_cov2 = TwistWithCovarianceStamped(twist=twist, covariance=identity_cov)
    assert np.trace(twist_cov2.covariance_matrix) == 6.0

    # Full covariance
    full_cov = np.random.rand(36)
    twist_cov3 = TwistWithCovarianceStamped(twist=twist, covariance=full_cov)
    assert np.array_equal(twist_cov3.covariance, full_cov)
