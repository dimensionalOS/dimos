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

import numpy as np
import pytest

try:
    from geometry_msgs.msg import (
        Twist as ROSTwist,
        TwistWithCovariance as ROSTwistWithCovariance,
        Vector3 as ROSVector3,
    )
except ImportError:
    ROSTwist = None
    ROSTwistWithCovariance = None
    ROSVector3 = None

from dimos_lcm.geometry_msgs import TwistWithCovariance as LCMTwistWithCovariance

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistWithCovariance import TwistWithCovariance
from dimos.msgs.geometry_msgs.Vector3 import Vector3


def test_twist_with_covariance_default_init() -> None:
    """Test that default initialization creates a zero twist with zero covariance."""
    if ROSVector3 is None:
        pytest.skip("ROS not available")
    if ROSTwistWithCovariance is None:
        pytest.skip("ROS not available")
    twist_cov = TwistWithCovariance()

    # Twist should be zero
    assert twist_cov.twist.linear.x == 0.0
    assert twist_cov.twist.linear.y == 0.0
    assert twist_cov.twist.linear.z == 0.0
    assert twist_cov.twist.angular.x == 0.0
    assert twist_cov.twist.angular.y == 0.0
    assert twist_cov.twist.angular.z == 0.0

    # Covariance should be all zeros
    assert np.all(twist_cov.covariance == 0.0)
    assert twist_cov.covariance.shape == (36,)


def test_twist_with_covariance_twist_init() -> None:
    """Test initialization with a Twist object."""
    linear = Vector3(1.0, 2.0, 3.0)
    angular = Vector3(0.1, 0.2, 0.3)
    twist = Twist(linear, angular)
    twist_cov = TwistWithCovariance(twist)

    # Twist should match
    assert twist_cov.twist.linear.x == 1.0
    assert twist_cov.twist.linear.y == 2.0
    assert twist_cov.twist.linear.z == 3.0
    assert twist_cov.twist.angular.x == 0.1
    assert twist_cov.twist.angular.y == 0.2
    assert twist_cov.twist.angular.z == 0.3

    # Covariance should be zeros by default
    assert np.all(twist_cov.covariance == 0.0)


def test_twist_with_covariance_twist_and_covariance_init() -> None:
    """Test initialization with twist and covariance."""
    twist = Twist(Vector3(1.0, 2.0, 3.0), Vector3(0.1, 0.2, 0.3))
    covariance = np.arange(36, dtype=float)
    twist_cov = TwistWithCovariance(twist, covariance)

    # Twist should match
    assert twist_cov.twist.linear.x == 1.0
    assert twist_cov.twist.linear.y == 2.0
    assert twist_cov.twist.linear.z == 3.0

    # Covariance should match
    assert np.array_equal(twist_cov.covariance, covariance)


def test_twist_with_covariance_tuple_init() -> None:
    """Test initialization with tuple of (linear, angular) velocities."""
    linear = [1.0, 2.0, 3.0]
    angular = [0.1, 0.2, 0.3]
    covariance = np.arange(36, dtype=float)
    twist_cov = TwistWithCovariance((linear, angular), covariance)

    # Twist should match
    assert twist_cov.twist.linear.x == 1.0
    assert twist_cov.twist.linear.y == 2.0
    assert twist_cov.twist.linear.z == 3.0
    assert twist_cov.twist.angular.x == 0.1
    assert twist_cov.twist.angular.y == 0.2
    assert twist_cov.twist.angular.z == 0.3

    # Covariance should match
    assert np.array_equal(twist_cov.covariance, covariance)


def test_twist_with_covariance_list_covariance() -> None:
    """Test initialization with covariance as a list."""
    twist = Twist(Vector3(1.0, 2.0, 3.0), Vector3(0.1, 0.2, 0.3))
    covariance_list = list(range(36))
    twist_cov = TwistWithCovariance(twist, covariance_list)

    # Covariance should be converted to numpy array
    assert isinstance(twist_cov.covariance, np.ndarray)
    assert np.array_equal(twist_cov.covariance, np.array(covariance_list))


def test_twist_with_covariance_copy_init() -> None:
    """Test copy constructor."""
    twist = Twist(Vector3(1.0, 2.0, 3.0), Vector3(0.1, 0.2, 0.3))
    covariance = np.arange(36, dtype=float)
    original = TwistWithCovariance(twist, covariance)
    copy = TwistWithCovariance(original)

    # Should be equal but not the same object
    assert copy == original
    assert copy is not original
    assert copy.twist is not original.twist
    assert copy.covariance is not original.covariance

    # Modify original to ensure they're independent
    original.covariance[0] = 999.0
    assert copy.covariance[0] != 999.0


def test_twist_with_covariance_lcm_init() -> None:
    """Test initialization from LCM message."""
    lcm_msg = LCMTwistWithCovariance()
    lcm_msg.twist.linear.x = 1.0
    lcm_msg.twist.linear.y = 2.0
    lcm_msg.twist.linear.z = 3.0
    lcm_msg.twist.angular.x = 0.1
    lcm_msg.twist.angular.y = 0.2
    lcm_msg.twist.angular.z = 0.3
    lcm_msg.covariance = list(range(36))

    twist_cov = TwistWithCovariance(lcm_msg)

    # Twist should match
    assert twist_cov.twist.linear.x == 1.0
    assert twist_cov.twist.linear.y == 2.0
    assert twist_cov.twist.linear.z == 3.0
    assert twist_cov.twist.angular.x == 0.1
    assert twist_cov.twist.angular.y == 0.2
    assert twist_cov.twist.angular.z == 0.3

    # Covariance should match
    assert np.array_equal(twist_cov.covariance, np.arange(36))


def test_twist_with_covariance_dict_init() -> None:
    """Test initialization from dictionary."""
    twist_dict = {
        "twist": Twist(Vector3(1.0, 2.0, 3.0), Vector3(0.1, 0.2, 0.3)),
        "covariance": list(range(36)),
    }
    twist_cov = TwistWithCovariance(twist_dict)

    assert twist_cov.twist.linear.x == 1.0
    assert twist_cov.twist.linear.y == 2.0
    assert twist_cov.twist.linear.z == 3.0
    assert np.array_equal(twist_cov.covariance, np.arange(36))


def test_twist_with_covariance_dict_init_no_covariance() -> None:
    """Test initialization from dictionary without covariance."""
    twist_dict = {"twist": Twist(Vector3(1.0, 2.0, 3.0), Vector3(0.1, 0.2, 0.3))}
    twist_cov = TwistWithCovariance(twist_dict)

    assert twist_cov.twist.linear.x == 1.0
    assert np.all(twist_cov.covariance == 0.0)


def test_twist_with_covariance_tuple_of_tuple_init() -> None:
    """Test initialization from tuple of (twist_tuple, covariance)."""
    twist_tuple = ([1.0, 2.0, 3.0], [0.1, 0.2, 0.3])
    covariance = np.arange(36, dtype=float)
    twist_cov = TwistWithCovariance((twist_tuple, covariance))

    assert twist_cov.twist.linear.x == 1.0
    assert twist_cov.twist.linear.y == 2.0
    assert twist_cov.twist.linear.z == 3.0
    assert twist_cov.twist.angular.x == 0.1
    assert twist_cov.twist.angular.y == 0.2
    assert twist_cov.twist.angular.z == 0.3
    assert np.array_equal(twist_cov.covariance, covariance)


def test_twist_with_covariance_properties() -> None:
    """Test convenience properties."""
    twist = Twist(Vector3(1.0, 2.0, 3.0), Vector3(0.1, 0.2, 0.3))
    twist_cov = TwistWithCovariance(twist)

    # Linear and angular properties
    assert twist_cov.linear.x == 1.0
    assert twist_cov.linear.y == 2.0
    assert twist_cov.linear.z == 3.0
    assert twist_cov.angular.x == 0.1
    assert twist_cov.angular.y == 0.2
    assert twist_cov.angular.z == 0.3


def test_twist_with_covariance_matrix_property() -> None:
    """Test covariance matrix property."""
    twist = Twist()
    covariance_array = np.arange(36, dtype=float)
    twist_cov = TwistWithCovariance(twist, covariance_array)

    # Get as matrix
    cov_matrix = twist_cov.covariance_matrix
    assert cov_matrix.shape == (6, 6)
    assert cov_matrix[0, 0] == 0.0
    assert cov_matrix[5, 5] == 35.0

    # Set from matrix
    new_matrix = np.eye(6) * 2.0
    twist_cov.covariance_matrix = new_matrix
    assert np.array_equal(twist_cov.covariance[:6], [2.0, 0.0, 0.0, 0.0, 0.0, 0.0])


def test_twist_with_covariance_repr() -> None:
    """Test string representation."""
    twist = Twist(Vector3(1.234, 2.567, 3.891), Vector3(0.1, 0.2, 0.3))
    twist_cov = TwistWithCovariance(twist)

    repr_str = repr(twist_cov)
    assert "TwistWithCovariance" in repr_str
    assert "twist=" in repr_str
    assert "covariance=" in repr_str
    assert "36 elements" in repr_str


def test_twist_with_covariance_str() -> None:
    """Test string formatting."""
    twist = Twist(Vector3(1.234, 2.567, 3.891), Vector3(0.1, 0.2, 0.3))
    covariance = np.eye(6).flatten()
    twist_cov = TwistWithCovariance(twist, covariance)

    str_repr = str(twist_cov)
    assert "TwistWithCovariance" in str_repr
    assert "1.234" in str_repr
    assert "2.567" in str_repr
    assert "3.891" in str_repr
    assert "cov_trace" in str_repr
    assert "6.000" in str_repr  # Trace of identity matrix is 6


def test_twist_with_covariance_equality() -> None:
    """Test equality comparison."""
    twist1 = Twist(Vector3(1.0, 2.0, 3.0), Vector3(0.1, 0.2, 0.3))
    cov1 = np.arange(36, dtype=float)
    twist_cov1 = TwistWithCovariance(twist1, cov1)

    twist2 = Twist(Vector3(1.0, 2.0, 3.0), Vector3(0.1, 0.2, 0.3))
    cov2 = np.arange(36, dtype=float)
    twist_cov2 = TwistWithCovariance(twist2, cov2)

    # Equal
    assert twist_cov1 == twist_cov2

    # Different twist
    twist3 = Twist(Vector3(1.1, 2.0, 3.0), Vector3(0.1, 0.2, 0.3))
    twist_cov3 = TwistWithCovariance(twist3, cov1)
    assert twist_cov1 != twist_cov3

    # Different covariance
    cov3 = np.arange(36, dtype=float) + 1
    twist_cov4 = TwistWithCovariance(twist1, cov3)
    assert twist_cov1 != twist_cov4

    # Different type
    assert twist_cov1 != "not a twist"
    assert twist_cov1 is not None


def test_twist_with_covariance_is_zero() -> None:
    """Test is_zero method."""
    # Zero twist
    twist_cov1 = TwistWithCovariance()
    assert twist_cov1.is_zero()
    assert not twist_cov1  # Boolean conversion

    # Non-zero twist
    twist = Twist(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
    twist_cov2 = TwistWithCovariance(twist)
    assert not twist_cov2.is_zero()
    assert twist_cov2  # Boolean conversion


def test_twist_with_covariance_lcm_encode_decode() -> None:
    """Test LCM encoding and decoding."""
    twist = Twist(Vector3(1.0, 2.0, 3.0), Vector3(0.1, 0.2, 0.3))
    covariance = np.arange(36, dtype=float)
    source = TwistWithCovariance(twist, covariance)

    # Encode and decode
    binary_msg = source.lcm_encode()
    decoded = TwistWithCovariance.lcm_decode(binary_msg)

    # Should be equal
    assert decoded == source
    assert isinstance(decoded, TwistWithCovariance)
    assert isinstance(decoded.twist, Twist)
    assert isinstance(decoded.covariance, np.ndarray)


@pytest.mark.ros
def test_twist_with_covariance_from_ros_msg() -> None:
    """Test creating from ROS message."""
    ros_msg = ROSTwistWithCovariance()
    ros_msg.twist.linear = ROSVector3(x=1.0, y=2.0, z=3.0)
    ros_msg.twist.angular = ROSVector3(x=0.1, y=0.2, z=0.3)
    ros_msg.covariance = [float(i) for i in range(36)]

    twist_cov = TwistWithCovariance.from_ros_msg(ros_msg)

    assert twist_cov.twist.linear.x == 1.0
    assert twist_cov.twist.linear.y == 2.0
    assert twist_cov.twist.linear.z == 3.0
    assert twist_cov.twist.angular.x == 0.1
    assert twist_cov.twist.angular.y == 0.2
    assert twist_cov.twist.angular.z == 0.3
    assert np.array_equal(twist_cov.covariance, np.arange(36))


@pytest.mark.ros
def test_twist_with_covariance_to_ros_msg() -> None:
    """Test converting to ROS message."""
    twist = Twist(Vector3(1.0, 2.0, 3.0), Vector3(0.1, 0.2, 0.3))
    covariance = np.arange(36, dtype=float)
    twist_cov = TwistWithCovariance(twist, covariance)

    ros_msg = twist_cov.to_ros_msg()

    assert isinstance(ros_msg, ROSTwistWithCovariance)
    assert ros_msg.twist.linear.x == 1.0
    assert ros_msg.twist.linear.y == 2.0
    assert ros_msg.twist.linear.z == 3.0
    assert ros_msg.twist.angular.x == 0.1
    assert ros_msg.twist.angular.y == 0.2
    assert ros_msg.twist.angular.z == 0.3
    assert list(ros_msg.covariance) == list(range(36))


@pytest.mark.ros
def test_twist_with_covariance_ros_roundtrip() -> None:
    """Test round-trip conversion with ROS messages."""
    twist = Twist(Vector3(1.5, 2.5, 3.5), Vector3(0.15, 0.25, 0.35))
    covariance = np.random.rand(36)
    original = TwistWithCovariance(twist, covariance)

    ros_msg = original.to_ros_msg()
    restored = TwistWithCovariance.from_ros_msg(ros_msg)

    assert restored == original


def test_twist_with_covariance_zero_covariance() -> None:
    """Test with zero covariance matrix."""
    twist = Twist(Vector3(1.0, 2.0, 3.0), Vector3(0.1, 0.2, 0.3))
    twist_cov = TwistWithCovariance(twist)

    assert np.all(twist_cov.covariance == 0.0)
    assert np.trace(twist_cov.covariance_matrix) == 0.0


def test_twist_with_covariance_diagonal_covariance() -> None:
    """Test with diagonal covariance matrix."""
    twist = Twist()
    covariance = np.zeros(36)
    # Set diagonal elements
    for i in range(6):
        covariance[i * 6 + i] = i + 1

    twist_cov = TwistWithCovariance(twist, covariance)

    cov_matrix = twist_cov.covariance_matrix
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
    "linear,angular",
    [
        ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        ([1.0, 2.0, 3.0], [0.1, 0.2, 0.3]),
        ([-1.0, -2.0, -3.0], [-0.1, -0.2, -0.3]),
        ([100.0, -100.0, 0.0], [3.14, -3.14, 0.0]),
    ],
)
def test_twist_with_covariance_parametrized_velocities(linear, angular) -> None:
    """Parametrized test for various velocity values."""
    twist = Twist(linear, angular)
    twist_cov = TwistWithCovariance(twist)

    assert twist_cov.linear.x == linear[0]
    assert twist_cov.linear.y == linear[1]
    assert twist_cov.linear.z == linear[2]
    assert twist_cov.angular.x == angular[0]
    assert twist_cov.angular.y == angular[1]
    assert twist_cov.angular.z == angular[2]
