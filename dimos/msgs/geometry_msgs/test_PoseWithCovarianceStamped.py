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
from dimos.msgs.geometry_msgs.PoseWithCovarianceStamped import PoseWithCovarianceStamped


def test_pose_with_covariance_stamped_default_init() -> None:
    """Test default initialization."""
    pose_cov_stamped = PoseWithCovarianceStamped()

    # Should have current timestamp
    assert pose_cov_stamped.ts > 0
    assert pose_cov_stamped.frame_id == ""

    # Pose should be at origin with identity orientation
    assert pose_cov_stamped.pose.position.x == 0.0
    assert pose_cov_stamped.pose.position.y == 0.0
    assert pose_cov_stamped.pose.position.z == 0.0
    assert pose_cov_stamped.pose.orientation.w == 1.0

    # Covariance should be all zeros
    assert np.all(pose_cov_stamped.covariance == 0.0)


def test_pose_with_covariance_stamped_with_timestamp() -> None:
    """Test initialization with specific timestamp."""
    ts = 1234567890.123456
    frame_id = "base_link"
    pose_cov_stamped = PoseWithCovarianceStamped(ts=ts, frame_id=frame_id)

    assert pose_cov_stamped.ts == ts
    assert pose_cov_stamped.frame_id == frame_id


def test_pose_with_covariance_stamped_with_pose() -> None:
    """Test initialization with pose."""
    ts = 1234567890.123456
    frame_id = "map"
    pose = Pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.9)
    covariance = np.arange(36, dtype=float)

    pose_cov_stamped = PoseWithCovarianceStamped(
        ts=ts, frame_id=frame_id, pose=pose, covariance=covariance
    )

    assert pose_cov_stamped.ts == ts
    assert pose_cov_stamped.frame_id == frame_id
    assert pose_cov_stamped.pose.position.x == 1.0
    assert pose_cov_stamped.pose.position.y == 2.0
    assert pose_cov_stamped.pose.position.z == 3.0
    assert np.array_equal(pose_cov_stamped.covariance, covariance)


def test_pose_with_covariance_stamped_properties() -> None:
    """Test convenience properties."""
    pose = Pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.9)
    covariance = np.eye(6).flatten()
    pose_cov_stamped = PoseWithCovarianceStamped(
        ts=1234567890.0, frame_id="odom", pose=pose, covariance=covariance
    )

    # Position properties
    assert pose_cov_stamped.x == 1.0
    assert pose_cov_stamped.y == 2.0
    assert pose_cov_stamped.z == 3.0

    # Orientation properties
    assert pose_cov_stamped.orientation.x == 0.1
    assert pose_cov_stamped.orientation.y == 0.2
    assert pose_cov_stamped.orientation.z == 0.3
    assert pose_cov_stamped.orientation.w == 0.9

    # Euler angles
    assert pose_cov_stamped.roll == pose.roll
    assert pose_cov_stamped.pitch == pose.pitch
    assert pose_cov_stamped.yaw == pose.yaw

    # Covariance matrix
    cov_matrix = pose_cov_stamped.covariance_matrix
    assert cov_matrix.shape == (6, 6)
    assert np.trace(cov_matrix) == 6.0


def test_pose_with_covariance_stamped_str() -> None:
    """Test string representation."""
    pose = Pose(1.234, 2.567, 3.891)
    covariance = np.eye(6).flatten() * 2.0
    pose_cov_stamped = PoseWithCovarianceStamped(
        ts=1234567890.0, frame_id="world", pose=pose, covariance=covariance
    )

    str_repr = str(pose_cov_stamped)
    assert "PoseWithCovarianceStamped" in str_repr
    assert "1.234" in str_repr
    assert "2.567" in str_repr
    assert "3.891" in str_repr
    assert "cov_trace" in str_repr
    assert "12.000" in str_repr  # Trace of 2*identity is 12


def test_pose_with_covariance_stamped_lcm_encode_decode() -> None:
    """Test LCM encoding and decoding."""
    ts = 1234567890.123456
    frame_id = "camera_link"
    pose = Pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.9)
    covariance = np.arange(36, dtype=float)

    source = PoseWithCovarianceStamped(ts=ts, frame_id=frame_id, pose=pose, covariance=covariance)

    # Encode and decode
    binary_msg = source.lcm_encode()
    decoded = PoseWithCovarianceStamped.lcm_decode(binary_msg)

    # Check timestamp (may lose some precision)
    assert abs(decoded.ts - ts) < 1e-6
    assert decoded.frame_id == frame_id

    # Check pose
    assert decoded.pose.position.x == 1.0
    assert decoded.pose.position.y == 2.0
    assert decoded.pose.position.z == 3.0
    assert decoded.pose.orientation.x == 0.1
    assert decoded.pose.orientation.y == 0.2
    assert decoded.pose.orientation.z == 0.3
    assert decoded.pose.orientation.w == 0.9

    # Check covariance
    assert np.array_equal(decoded.covariance, covariance)


def test_pose_with_covariance_stamped_zero_timestamp() -> None:
    """Test that zero timestamp gets replaced with current time."""
    pose_cov_stamped = PoseWithCovarianceStamped(ts=0.0)

    # Should have been replaced with current time
    assert pose_cov_stamped.ts > 0
    assert pose_cov_stamped.ts <= time.time()


def test_pose_with_covariance_stamped_inheritance() -> None:
    """Test that it properly inherits from PoseWithCovariance and Timestamped."""
    pose = Pose(1.0, 2.0, 3.0)
    covariance = np.eye(6).flatten()
    pose_cov_stamped = PoseWithCovarianceStamped(
        ts=1234567890.0, frame_id="test", pose=pose, covariance=covariance
    )

    # Should be instance of parent classes
    assert isinstance(pose_cov_stamped, PoseWithCovariance)

    # Should have Timestamped attributes
    assert hasattr(pose_cov_stamped, "ts")
    assert hasattr(pose_cov_stamped, "frame_id")

    # Should have PoseWithCovariance attributes
    assert hasattr(pose_cov_stamped, "pose")
    assert hasattr(pose_cov_stamped, "covariance")


def test_pose_with_covariance_stamped_sec_nsec() -> None:
    """Test the sec_nsec helper function."""
    from dimos.msgs.geometry_msgs.PoseWithCovarianceStamped import sec_nsec

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


def test_pose_with_covariance_stamped_different_covariances() -> None:
    """Test with different covariance patterns."""
    pose = Pose(1.0, 2.0, 3.0)

    # Zero covariance
    zero_cov = np.zeros(36)
    pose_cov1 = PoseWithCovarianceStamped(pose=pose, covariance=zero_cov)
    assert np.all(pose_cov1.covariance == 0.0)

    # Identity covariance
    identity_cov = np.eye(6).flatten()
    pose_cov2 = PoseWithCovarianceStamped(pose=pose, covariance=identity_cov)
    assert np.trace(pose_cov2.covariance_matrix) == 6.0

    # Full covariance
    full_cov = np.random.rand(36)
    pose_cov3 = PoseWithCovarianceStamped(pose=pose, covariance=full_cov)
    assert np.array_equal(pose_cov3.covariance, full_cov)
