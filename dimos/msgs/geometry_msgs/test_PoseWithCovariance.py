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

from dimos_lcm.geometry_msgs import PoseWithCovariance as LCMPoseWithCovariance
import numpy as np
import pytest

try:
    from geometry_msgs.msg import (
        Point as ROSPoint,
        Pose as ROSPose,
        PoseWithCovariance as ROSPoseWithCovariance,
        Quaternion as ROSQuaternion,
    )
except ImportError:
    ROSPoseWithCovariance = None
    ROSPose = None
    ROSPoint = None
    ROSQuaternion = None

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseWithCovariance import PoseWithCovariance


def test_pose_with_covariance_default_init() -> None:
    """Test that default initialization creates a pose at origin with zero covariance."""
    pose_cov = PoseWithCovariance()

    # Pose should be at origin with identity orientation
    assert pose_cov.pose.position.x == 0.0
    assert pose_cov.pose.position.y == 0.0
    assert pose_cov.pose.position.z == 0.0
    assert pose_cov.pose.orientation.x == 0.0
    assert pose_cov.pose.orientation.y == 0.0
    assert pose_cov.pose.orientation.z == 0.0
    assert pose_cov.pose.orientation.w == 1.0

    # Covariance should be all zeros
    assert np.all(pose_cov.covariance == 0.0)
    assert pose_cov.covariance.shape == (36,)


def test_pose_with_covariance_pose_init() -> None:
    """Test initialization with a Pose object."""
    pose = Pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.9)
    pose_cov = PoseWithCovariance(pose)

    # Pose should match
    assert pose_cov.pose.position.x == 1.0
    assert pose_cov.pose.position.y == 2.0
    assert pose_cov.pose.position.z == 3.0
    assert pose_cov.pose.orientation.x == 0.1
    assert pose_cov.pose.orientation.y == 0.2
    assert pose_cov.pose.orientation.z == 0.3
    assert pose_cov.pose.orientation.w == 0.9

    # Covariance should be zeros by default
    assert np.all(pose_cov.covariance == 0.0)


def test_pose_with_covariance_pose_and_covariance_init() -> None:
    """Test initialization with pose and covariance."""
    pose = Pose(1.0, 2.0, 3.0)
    covariance = np.arange(36, dtype=float)
    pose_cov = PoseWithCovariance(pose, covariance)

    # Pose should match
    assert pose_cov.pose.position.x == 1.0
    assert pose_cov.pose.position.y == 2.0
    assert pose_cov.pose.position.z == 3.0

    # Covariance should match
    assert np.array_equal(pose_cov.covariance, covariance)


def test_pose_with_covariance_list_covariance() -> None:
    """Test initialization with covariance as a list."""
    pose = Pose(1.0, 2.0, 3.0)
    covariance_list = list(range(36))
    pose_cov = PoseWithCovariance(pose, covariance_list)

    # Covariance should be converted to numpy array
    assert isinstance(pose_cov.covariance, np.ndarray)
    assert np.array_equal(pose_cov.covariance, np.array(covariance_list))


def test_pose_with_covariance_copy_init() -> None:
    """Test copy constructor."""
    pose = Pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.9)
    covariance = np.arange(36, dtype=float)
    original = PoseWithCovariance(pose, covariance)
    copy = PoseWithCovariance(original)

    # Should be equal but not the same object
    assert copy == original
    assert copy is not original
    assert copy.pose is not original.pose
    assert copy.covariance is not original.covariance

    # Modify original to ensure they're independent
    original.covariance[0] = 999.0
    assert copy.covariance[0] != 999.0


def test_pose_with_covariance_lcm_init() -> None:
    """Test initialization from LCM message."""
    lcm_msg = LCMPoseWithCovariance()
    lcm_msg.pose.position.x = 1.0
    lcm_msg.pose.position.y = 2.0
    lcm_msg.pose.position.z = 3.0
    lcm_msg.pose.orientation.x = 0.1
    lcm_msg.pose.orientation.y = 0.2
    lcm_msg.pose.orientation.z = 0.3
    lcm_msg.pose.orientation.w = 0.9
    lcm_msg.covariance = list(range(36))

    pose_cov = PoseWithCovariance(lcm_msg)

    # Pose should match
    assert pose_cov.pose.position.x == 1.0
    assert pose_cov.pose.position.y == 2.0
    assert pose_cov.pose.position.z == 3.0
    assert pose_cov.pose.orientation.x == 0.1
    assert pose_cov.pose.orientation.y == 0.2
    assert pose_cov.pose.orientation.z == 0.3
    assert pose_cov.pose.orientation.w == 0.9

    # Covariance should match
    assert np.array_equal(pose_cov.covariance, np.arange(36))


def test_pose_with_covariance_dict_init() -> None:
    """Test initialization from dictionary."""
    pose_dict = {"pose": Pose(1.0, 2.0, 3.0), "covariance": list(range(36))}
    pose_cov = PoseWithCovariance(pose_dict)

    assert pose_cov.pose.position.x == 1.0
    assert pose_cov.pose.position.y == 2.0
    assert pose_cov.pose.position.z == 3.0
    assert np.array_equal(pose_cov.covariance, np.arange(36))


def test_pose_with_covariance_dict_init_no_covariance() -> None:
    """Test initialization from dictionary without covariance."""
    pose_dict = {"pose": Pose(1.0, 2.0, 3.0)}
    pose_cov = PoseWithCovariance(pose_dict)

    assert pose_cov.pose.position.x == 1.0
    assert np.all(pose_cov.covariance == 0.0)


def test_pose_with_covariance_tuple_init() -> None:
    """Test initialization from tuple."""
    pose = Pose(1.0, 2.0, 3.0)
    covariance = np.arange(36, dtype=float)
    pose_tuple = (pose, covariance)
    pose_cov = PoseWithCovariance(pose_tuple)

    assert pose_cov.pose.position.x == 1.0
    assert pose_cov.pose.position.y == 2.0
    assert pose_cov.pose.position.z == 3.0
    assert np.array_equal(pose_cov.covariance, covariance)


def test_pose_with_covariance_properties() -> None:
    """Test convenience properties."""
    pose = Pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.9)
    pose_cov = PoseWithCovariance(pose)

    # Position properties
    assert pose_cov.x == 1.0
    assert pose_cov.y == 2.0
    assert pose_cov.z == 3.0
    assert pose_cov.position.x == 1.0
    assert pose_cov.position.y == 2.0
    assert pose_cov.position.z == 3.0

    # Orientation properties
    assert pose_cov.orientation.x == 0.1
    assert pose_cov.orientation.y == 0.2
    assert pose_cov.orientation.z == 0.3
    assert pose_cov.orientation.w == 0.9

    # Euler angle properties
    assert pose_cov.roll == pose.roll
    assert pose_cov.pitch == pose.pitch
    assert pose_cov.yaw == pose.yaw


def test_pose_with_covariance_matrix_property() -> None:
    """Test covariance matrix property."""
    pose = Pose()
    covariance_array = np.arange(36, dtype=float)
    pose_cov = PoseWithCovariance(pose, covariance_array)

    # Get as matrix
    cov_matrix = pose_cov.covariance_matrix
    assert cov_matrix.shape == (6, 6)
    assert cov_matrix[0, 0] == 0.0
    assert cov_matrix[5, 5] == 35.0

    # Set from matrix
    new_matrix = np.eye(6) * 2.0
    pose_cov.covariance_matrix = new_matrix
    assert np.array_equal(pose_cov.covariance[:6], [2.0, 0.0, 0.0, 0.0, 0.0, 0.0])


def test_pose_with_covariance_repr() -> None:
    """Test string representation."""
    pose = Pose(1.234, 2.567, 3.891)
    pose_cov = PoseWithCovariance(pose)

    repr_str = repr(pose_cov)
    assert "PoseWithCovariance" in repr_str
    assert "pose=" in repr_str
    assert "covariance=" in repr_str
    assert "36 elements" in repr_str


def test_pose_with_covariance_str() -> None:
    """Test string formatting."""
    pose = Pose(1.234, 2.567, 3.891)
    covariance = np.eye(6).flatten()
    pose_cov = PoseWithCovariance(pose, covariance)

    str_repr = str(pose_cov)
    assert "PoseWithCovariance" in str_repr
    assert "1.234" in str_repr
    assert "2.567" in str_repr
    assert "3.891" in str_repr
    assert "cov_trace" in str_repr
    assert "6.000" in str_repr  # Trace of identity matrix is 6


def test_pose_with_covariance_equality() -> None:
    """Test equality comparison."""
    pose1 = Pose(1.0, 2.0, 3.0)
    cov1 = np.arange(36, dtype=float)
    pose_cov1 = PoseWithCovariance(pose1, cov1)

    pose2 = Pose(1.0, 2.0, 3.0)
    cov2 = np.arange(36, dtype=float)
    pose_cov2 = PoseWithCovariance(pose2, cov2)

    # Equal
    assert pose_cov1 == pose_cov2

    # Different pose
    pose3 = Pose(1.1, 2.0, 3.0)
    pose_cov3 = PoseWithCovariance(pose3, cov1)
    assert pose_cov1 != pose_cov3

    # Different covariance
    cov3 = np.arange(36, dtype=float) + 1
    pose_cov4 = PoseWithCovariance(pose1, cov3)
    assert pose_cov1 != pose_cov4

    # Different type
    assert pose_cov1 != "not a pose"
    assert pose_cov1 is not None


def test_pose_with_covariance_lcm_encode_decode() -> None:
    """Test LCM encoding and decoding."""
    pose = Pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.9)
    covariance = np.arange(36, dtype=float)
    source = PoseWithCovariance(pose, covariance)

    # Encode and decode
    binary_msg = source.lcm_encode()
    decoded = PoseWithCovariance.lcm_decode(binary_msg)

    # Should be equal
    assert decoded == source
    assert isinstance(decoded, PoseWithCovariance)
    assert isinstance(decoded.pose, Pose)
    assert isinstance(decoded.covariance, np.ndarray)


@pytest.mark.ros
def test_pose_with_covariance_from_ros_msg() -> None:
    """Test creating from ROS message."""
    ros_msg = ROSPoseWithCovariance()
    ros_msg.pose.position = ROSPoint(x=1.0, y=2.0, z=3.0)
    ros_msg.pose.orientation = ROSQuaternion(x=0.1, y=0.2, z=0.3, w=0.9)
    ros_msg.covariance = [float(i) for i in range(36)]

    pose_cov = PoseWithCovariance.from_ros_msg(ros_msg)

    assert pose_cov.pose.position.x == 1.0
    assert pose_cov.pose.position.y == 2.0
    assert pose_cov.pose.position.z == 3.0
    assert pose_cov.pose.orientation.x == 0.1
    assert pose_cov.pose.orientation.y == 0.2
    assert pose_cov.pose.orientation.z == 0.3
    assert pose_cov.pose.orientation.w == 0.9
    assert np.array_equal(pose_cov.covariance, np.arange(36))


@pytest.mark.ros
def test_pose_with_covariance_to_ros_msg() -> None:
    """Test converting to ROS message."""
    pose = Pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.9)
    covariance = np.arange(36, dtype=float)
    pose_cov = PoseWithCovariance(pose, covariance)

    ros_msg = pose_cov.to_ros_msg()

    assert isinstance(ros_msg, ROSPoseWithCovariance)
    assert ros_msg.pose.position.x == 1.0
    assert ros_msg.pose.position.y == 2.0
    assert ros_msg.pose.position.z == 3.0
    assert ros_msg.pose.orientation.x == 0.1
    assert ros_msg.pose.orientation.y == 0.2
    assert ros_msg.pose.orientation.z == 0.3
    assert ros_msg.pose.orientation.w == 0.9
    assert list(ros_msg.covariance) == list(range(36))


@pytest.mark.ros
def test_pose_with_covariance_ros_roundtrip() -> None:
    """Test round-trip conversion with ROS messages."""
    pose = Pose(1.5, 2.5, 3.5, 0.15, 0.25, 0.35, 0.85)
    covariance = np.random.rand(36)
    original = PoseWithCovariance(pose, covariance)

    ros_msg = original.to_ros_msg()
    restored = PoseWithCovariance.from_ros_msg(ros_msg)

    assert restored == original


def test_pose_with_covariance_zero_covariance() -> None:
    """Test with zero covariance matrix."""
    pose = Pose(1.0, 2.0, 3.0)
    pose_cov = PoseWithCovariance(pose)

    assert np.all(pose_cov.covariance == 0.0)
    assert np.trace(pose_cov.covariance_matrix) == 0.0


def test_pose_with_covariance_diagonal_covariance() -> None:
    """Test with diagonal covariance matrix."""
    pose = Pose()
    covariance = np.zeros(36)
    # Set diagonal elements
    for i in range(6):
        covariance[i * 6 + i] = i + 1

    pose_cov = PoseWithCovariance(pose, covariance)

    cov_matrix = pose_cov.covariance_matrix
    assert np.trace(cov_matrix) == sum(range(1, 7))  # 1+2+3+4+5+6 = 21

    # Check diagonal elements
    for i in range(6):
        assert cov_matrix[i, i] == i + 1

    # Check off-diagonal elements are zero
    for i in range(6):
        for j in range(6):
            if i != j:
                assert cov_matrix[i, j] == 0.0


@pytest.mark.parametrize(
    "x,y,z",
    [(0.0, 0.0, 0.0), (1.0, 2.0, 3.0), (-1.0, -2.0, -3.0), (100.0, -100.0, 0.0)],
)
def test_pose_with_covariance_parametrized_positions(x, y, z) -> None:
    """Parametrized test for various position values."""
    pose = Pose(x, y, z)
    pose_cov = PoseWithCovariance(pose)

    assert pose_cov.x == x
    assert pose_cov.y == y
    assert pose_cov.z == z
