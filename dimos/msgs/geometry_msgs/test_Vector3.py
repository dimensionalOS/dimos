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

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3


def test_vector_default_init():
    """Test that default initialization of Vector() has x,y,z components all zero."""
    v = Vector3()
    assert v.x == 0.0
    assert v.y == 0.0
    assert v.z == 0.0
    assert v.dim == 0
    assert len(v.data) == 0
    assert v.to_list() == []
    assert v.is_zero() == True  # Empty vector should be considered zero


def test_vector_specific_init():
    """Test initialization with specific values and different input types."""

    v1 = Vector3(1.0, 2.0)  # 2D vector
    assert v1.x == 1.0
    assert v1.y == 2.0
    assert v1.z == 0.0
    assert v1.dim == 2

    v2 = Vector3(3.0, 4.0, 5.0)  # 3D vector
    assert v2.x == 3.0
    assert v2.y == 4.0
    assert v2.z == 5.0
    assert v2.dim == 3

    v3 = Vector3([6.0, 7.0, 8.0])
    assert v3.x == 6.0
    assert v3.y == 7.0
    assert v3.z == 8.0
    assert v3.dim == 3

    v4 = Vector3((9.0, 10.0, 11.0))
    assert v4.x == 9.0
    assert v4.y == 10.0
    assert v4.z == 11.0
    assert v4.dim == 3

    v5 = Vector3(np.array([12.0, 13.0, 14.0]))
    assert v5.x == 12.0
    assert v5.y == 13.0
    assert v5.z == 14.0
    assert v5.dim == 3

    original = Vector3([15.0, 16.0, 17.0])
    v6 = Vector3(original)
    assert v6.x == 15.0
    assert v6.y == 16.0
    assert v6.z == 17.0
    assert v6.dim == 3

    assert v6 is not original
    assert v6 == original


def test_vector_addition():
    """Test vector addition."""
    v1 = Vector3(1.0, 2.0, 3.0)
    v2 = Vector3(4.0, 5.0, 6.0)

    v_add = v1 + v2
    assert v_add.x == 5.0
    assert v_add.y == 7.0
    assert v_add.z == 9.0


def test_vector_subtraction():
    """Test vector subtraction."""
    v1 = Vector3(1.0, 2.0, 3.0)
    v2 = Vector3(4.0, 5.0, 6.0)

    v_sub = v2 - v1
    assert v_sub.x == 3.0
    assert v_sub.y == 3.0
    assert v_sub.z == 3.0


def test_vector_scalar_multiplication():
    """Test vector multiplication by a scalar."""
    v1 = Vector3(1.0, 2.0, 3.0)

    v_mul = v1 * 2.0
    assert v_mul.x == 2.0
    assert v_mul.y == 4.0
    assert v_mul.z == 6.0

    # Test right multiplication
    v_rmul = 2.0 * v1
    assert v_rmul.x == 2.0
    assert v_rmul.y == 4.0
    assert v_rmul.z == 6.0


def test_vector_scalar_division():
    """Test vector division by a scalar."""
    v2 = Vector3(4.0, 5.0, 6.0)

    v_div = v2 / 2.0
    assert v_div.x == 2.0
    assert v_div.y == 2.5
    assert v_div.z == 3.0


def test_vector_dot_product():
    """Test vector dot product."""
    v1 = Vector3(1.0, 2.0, 3.0)
    v2 = Vector3(4.0, 5.0, 6.0)

    dot = v1.dot(v2)
    assert dot == 32.0


def test_vector_length():
    """Test vector length calculation."""
    # 2D vector with length 5
    v1 = Vector3(3.0, 4.0)
    assert v1.length() == 5.0

    # 3D vector
    v2 = Vector3(2.0, 3.0, 6.0)
    assert v2.length() == pytest.approx(7.0, 0.001)

    # Test length_squared
    assert v1.length_squared() == 25.0
    assert v2.length_squared() == 49.0


def test_vector_normalize():
    """Test vector normalization."""
    v = Vector3(2.0, 3.0, 6.0)
    assert v.is_zero() == False

    v_norm = v.normalize()
    length = v.length()
    expected_x = 2.0 / length
    expected_y = 3.0 / length
    expected_z = 6.0 / length

    assert np.isclose(v_norm.x, expected_x)
    assert np.isclose(v_norm.y, expected_y)
    assert np.isclose(v_norm.z, expected_z)
    assert np.isclose(v_norm.length(), 1.0)
    assert v_norm.is_zero() == False

    # Test normalizing a zero vector
    v_zero = Vector3(0.0, 0.0, 0.0)
    assert v_zero.is_zero() == True
    v_zero_norm = v_zero.normalize()
    assert v_zero_norm.x == 0.0
    assert v_zero_norm.y == 0.0
    assert v_zero_norm.z == 0.0
    assert v_zero_norm.is_zero() == True


def test_vector_to_2d():
    """Test conversion to 2D vector."""
    v = Vector3(2.0, 3.0, 6.0)

    v_2d = v.to_2d()
    assert v_2d.x == 2.0
    assert v_2d.y == 3.0
    assert v_2d.z == 0.0
    assert v_2d.dim == 2

    # Already 2D vector
    v2 = Vector3(4.0, 5.0)
    v2_2d = v2.to_2d()
    assert v2_2d.x == 4.0
    assert v2_2d.y == 5.0
    assert v2_2d.dim == 2


def test_vector_distance():
    """Test distance calculations between vectors."""
    v1 = Vector3(1.0, 2.0, 3.0)
    v2 = Vector3(4.0, 6.0, 8.0)

    # Distance
    dist = v1.distance(v2)
    expected_dist = np.sqrt(9.0 + 16.0 + 25.0)  # sqrt((4-1)² + (6-2)² + (8-3)²)
    assert dist == pytest.approx(expected_dist)

    # Distance squared
    dist_sq = v1.distance_squared(v2)
    assert dist_sq == 50.0  # 9 + 16 + 25


def test_vector_cross_product():
    """Test vector cross product."""
    v1 = Vector3(1.0, 0.0, 0.0)  # Unit x vector
    v2 = Vector3(0.0, 1.0, 0.0)  # Unit y vector

    # v1 × v2 should be unit z vector
    cross = v1.cross(v2)
    assert cross.x == 0.0
    assert cross.y == 0.0
    assert cross.z == 1.0

    # Test with more complex vectors
    a = Vector3(2.0, 3.0, 4.0)
    b = Vector3(5.0, 6.0, 7.0)
    c = a.cross(b)

    # Cross product manually calculated:
    # (3*7-4*6, 4*5-2*7, 2*6-3*5)
    assert c.x == -3.0
    assert c.y == 6.0
    assert c.z == -3.0

    # Test with 2D vectors (should raise error)
    v_2d = Vector3(1.0, 2.0)
    with pytest.raises(ValueError):
        v_2d.cross(v2)


def test_vector_zeros():
    """Test Vector3.zeros class method."""
    # 3D zero vector
    v_zeros = Vector3.zeros()
    assert v_zeros.x == 0.0
    assert v_zeros.y == 0.0
    assert v_zeros.z == 0.0
    assert v_zeros.is_zero() == True


def test_vector_ones():
    """Test Vector3.ones class method."""
    # 3D ones vector
    v_ones = Vector3.ones(3)
    assert v_ones.x == 1.0
    assert v_ones.y == 1.0
    assert v_ones.z == 1.0
    assert v_ones.dim == 3

    # 2D ones vector
    v_ones_2d = Vector3.ones(2)
    assert v_ones_2d.x == 1.0
    assert v_ones_2d.y == 1.0
    assert v_ones_2d.z == 0.0
    assert v_ones_2d.dim == 2


def test_vector_conversion_methods():
    """Test vector conversion methods (to_list, to_tuple, to_numpy)."""
    v = Vector3(1.0, 2.0, 3.0)

    # to_list
    assert v.to_list() == [1.0, 2.0, 3.0]

    # to_tuple
    assert v.to_tuple() == (1.0, 2.0, 3.0)

    # to_numpy
    np_array = v.to_numpy()
    assert isinstance(np_array, np.ndarray)
    assert np.array_equal(np_array, np.array([1.0, 2.0, 3.0]))


def test_vector_equality():
    """Test vector equality."""
    v1 = Vector3(1, 2, 3)
    v2 = Vector3(1, 2, 3)
    v3 = Vector3(4, 5, 6)

    assert v1 == v2
    assert v1 != v3
    assert v1 != Vector3(1, 2)  # Different dimensions
    assert v1 != Vector3(1.1, 2, 3)  # Different values
    assert v1 != [1, 2, 3]


def test_vector_is_zero():
    """Test is_zero method for vectors."""
    # Default empty vector
    v0 = Vector3()
    assert v0.is_zero() == True

    # Explicit zero vector
    v1 = Vector3(0.0, 0.0, 0.0)
    assert v1.is_zero() == True

    # Zero vector with different dimensions
    v2 = Vector3(0.0, 0.0)
    assert v2.is_zero() == True

    # Non-zero vectors
    v3 = Vector3(1.0, 0.0, 0.0)
    assert v3.is_zero() == False

    v4 = Vector3(0.0, 2.0, 0.0)
    assert v4.is_zero() == False

    v5 = Vector3(0.0, 0.0, 3.0)
    assert v5.is_zero() == False

    # Almost zero (within tolerance)
    v6 = Vector3(1e-10, 1e-10, 1e-10)
    assert v6.is_zero() == True

    # Almost zero (outside tolerance)
    v7 = Vector3(1e-6, 1e-6, 1e-6)
    assert v7.is_zero() == False


def test_vector_bool_conversion():
    """Test boolean conversion of vectors."""
    # Zero vectors should be False
    v0 = Vector3()
    assert bool(v0) == False

    v1 = Vector3(0.0, 0.0, 0.0)
    assert bool(v1) == False

    # Almost zero vectors should be False
    v2 = Vector3(1e-10, 1e-10, 1e-10)
    assert bool(v2) == False

    # Non-zero vectors should be True
    v3 = Vector3(1.0, 0.0, 0.0)
    assert bool(v3) == True

    v4 = Vector3(0.0, 2.0, 0.0)
    assert bool(v4) == True

    v5 = Vector3(0.0, 0.0, 3.0)
    assert bool(v5) == True

    # Direct use in if statements
    if v0:
        assert False, "Zero vector should be False in boolean context"
    else:
        pass  # Expected path

    if v3:
        pass  # Expected path
    else:
        assert False, "Non-zero vector should be True in boolean context"


def test_vector_add():
    """Test vector addition operator."""
    v1 = Vector3(1.0, 2.0, 3.0)
    v2 = Vector3(4.0, 5.0, 6.0)

    # Using __add__ method
    v_add = v1.__add__(v2)
    assert v_add.x == 5.0
    assert v_add.y == 7.0
    assert v_add.z == 9.0

    # Using + operator
    v_add_op = v1 + v2
    assert v_add_op.x == 5.0
    assert v_add_op.y == 7.0
    assert v_add_op.z == 9.0

    # Adding zero vector should return original vector
    v_zero = Vector3.zeros()
    assert (v1 + v_zero) == v1


def test_vector_add_dim_mismatch():
    """Test vector addition operator."""
    v1 = Vector3(1.0, 2.0)
    v2 = Vector3(4.0, 5.0, 6.0)

    # Using + operator
    v_add_op = v1 + v2


def test_yaw_pitch_roll_accessors():
    """Test yaw, pitch, and roll accessor properties."""
    # Test with a 3D vector
    v = Vector3(1.0, 2.0, 3.0)

    # According to standard convention:
    # roll = rotation around x-axis = x component
    # pitch = rotation around y-axis = y component
    # yaw = rotation around z-axis = z component
    assert v.roll == 1.0  # Should return x component
    assert v.pitch == 2.0  # Should return y component
    assert v.yaw == 3.0  # Should return z component

    # Test with a 2D vector (z should be 0.0)
    v_2d = Vector3(4.0, 5.0)
    assert v_2d.roll == 4.0  # Should return x component
    assert v_2d.pitch == 5.0  # Should return y component
    assert v_2d.yaw == 0.0  # Should return z component (defaults to 0 for 2D)

    # Test with empty vector (all should be 0.0)
    v_empty = Vector3()
    assert v_empty.roll == 0.0
    assert v_empty.pitch == 0.0
    assert v_empty.yaw == 0.0

    # Test with negative values
    v_neg = Vector3(-1.5, -2.5, -3.5)
    assert v_neg.roll == -1.5
    assert v_neg.pitch == -2.5
    assert v_neg.yaw == -3.5

    # Test with single component vector
    v_single = Vector3(7.0)
    assert v_single.roll == 7.0  # x component
    assert v_single.pitch == 0.0  # y defaults to 0
    assert v_single.yaw == 0.0  # z defaults to 0


def test_vector_to_quaternion():
    """Test conversion from Vector3 Euler angles to Quaternion."""
    # Test zero rotation (identity quaternion)
    v_zero = Vector3(0.0, 0.0, 0.0)
    q_zero = v_zero.to_quaternion()
    assert isinstance(q_zero, Quaternion)
    assert np.isclose(q_zero.x, 0.0)
    assert np.isclose(q_zero.y, 0.0)
    assert np.isclose(q_zero.z, 0.0)
    assert np.isclose(q_zero.w, 1.0)

    # Test 90 degree rotation around x-axis (roll)
    v_roll_90 = Vector3(np.pi / 2, 0.0, 0.0)
    q_roll_90 = v_roll_90.to_quaternion()
    expected_val = np.sin(np.pi / 4)  # sin(45°) for half angle
    assert np.isclose(q_roll_90.x, expected_val, atol=1e-6)
    assert np.isclose(q_roll_90.y, 0.0, atol=1e-6)
    assert np.isclose(q_roll_90.z, 0.0, atol=1e-6)
    assert np.isclose(q_roll_90.w, np.cos(np.pi / 4), atol=1e-6)

    # Test 90 degree rotation around y-axis (pitch)
    v_pitch_90 = Vector3(0.0, np.pi / 2, 0.0)
    q_pitch_90 = v_pitch_90.to_quaternion()
    assert np.isclose(q_pitch_90.x, 0.0, atol=1e-6)
    assert np.isclose(q_pitch_90.y, expected_val, atol=1e-6)
    assert np.isclose(q_pitch_90.z, 0.0, atol=1e-6)
    assert np.isclose(q_pitch_90.w, np.cos(np.pi / 4), atol=1e-6)

    # Test 90 degree rotation around z-axis (yaw)
    v_yaw_90 = Vector3(0.0, 0.0, np.pi / 2)
    q_yaw_90 = v_yaw_90.to_quaternion()
    assert np.isclose(q_yaw_90.x, 0.0, atol=1e-6)
    assert np.isclose(q_yaw_90.y, 0.0, atol=1e-6)
    assert np.isclose(q_yaw_90.z, expected_val, atol=1e-6)
    assert np.isclose(q_yaw_90.w, np.cos(np.pi / 4), atol=1e-6)

    # Test combined rotation (45 degrees around each axis)
    angle_45 = np.pi / 4
    v_combined = Vector3(angle_45, angle_45, angle_45)
    q_combined = v_combined.to_quaternion()

    # Verify quaternion is normalized (magnitude = 1)
    magnitude_sq = q_combined.x**2 + q_combined.y**2 + q_combined.z**2 + q_combined.w**2
    assert np.isclose(magnitude_sq, 1.0, atol=1e-6)

    # Test conversion round-trip: Vector3 -> Quaternion -> Vector3
    # Should get back the original Euler angles (within tolerance)
    v_original = Vector3(0.1, 0.2, 0.3)  # Small angles to avoid gimbal lock issues
    q_converted = v_original.to_quaternion()
    v_roundtrip = q_converted.to_euler()

    assert np.isclose(v_original.x, v_roundtrip.x, atol=1e-6)
    assert np.isclose(v_original.y, v_roundtrip.y, atol=1e-6)
    assert np.isclose(v_original.z, v_roundtrip.z, atol=1e-6)

    # Test negative angles
    v_negative = Vector3(-np.pi / 6, -np.pi / 4, -np.pi / 3)
    q_negative = v_negative.to_quaternion()
    assert isinstance(q_negative, Quaternion)

    # Verify quaternion is normalized for negative angles too
    magnitude_sq_neg = q_negative.x**2 + q_negative.y**2 + q_negative.z**2 + q_negative.w**2
    assert np.isclose(magnitude_sq_neg, 1.0, atol=1e-6)

    # Test with 2D vector (should treat z as 0)
    v_2d = Vector3(np.pi / 6, np.pi / 4)
    q_2d = v_2d.to_quaternion()
    # Should be equivalent to Vector3(pi/6, pi/4, 0.0)
    v_3d_equiv = Vector3(np.pi / 6, np.pi / 4, 0.0)
    q_3d_equiv = v_3d_equiv.to_quaternion()

    assert np.isclose(q_2d.x, q_3d_equiv.x, atol=1e-6)
    assert np.isclose(q_2d.y, q_3d_equiv.y, atol=1e-6)
    assert np.isclose(q_2d.z, q_3d_equiv.z, atol=1e-6)
    assert np.isclose(q_2d.w, q_3d_equiv.w, atol=1e-6)
