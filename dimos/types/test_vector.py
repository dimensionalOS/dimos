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

from dimos.types.vector import Vector


def test_vector_default_init() -> None:
    """Test that default initialization of Vector() has x,y,z components all zero."""
    v = Vector()
    assert v.x == 0.0
    assert v.y == 0.0
    assert v.z == 0.0
    assert v.dim == 0
    assert len(v.data) == 0
    assert v.to_list() == []
    assert v.is_zero()  # Empty vector should be considered zero


def test_vector_specific_init() -> None:
    """Test initialization with specific values."""
    # 2D vector
    v1 = Vector(1.0, 2.0)
    assert v1.x == 1.0
    assert v1.y == 2.0
    assert v1.z == 0.0
    assert v1.dim == 2

    # 3D vector
    v2 = Vector(3.0, 4.0, 5.0)
    assert v2.x == 3.0
    assert v2.y == 4.0
    assert v2.z == 5.0
    assert v2.dim == 3

    # From list
    v3 = Vector([6.0, 7.0, 8.0])
    assert v3.x == 6.0
    assert v3.y == 7.0
    assert v3.z == 8.0
    assert v3.dim == 3

    # From numpy array
    v4 = Vector(np.array([9.0, 10.0, 11.0]))
    assert v4.x == 9.0
    assert v4.y == 10.0
    assert v4.z == 11.0
    assert v4.dim == 3


def test_vector_addition() -> None:
    """Test vector addition."""
    v1 = Vector(1.0, 2.0, 3.0)
    v2 = Vector(4.0, 5.0, 6.0)

    v_add = v1 + v2
    assert v_add.x == 5.0
    assert v_add.y == 7.0
    assert v_add.z == 9.0


def test_vector_subtraction() -> None:
    """Test vector subtraction."""
    v1 = Vector(1.0, 2.0, 3.0)
    v2 = Vector(4.0, 5.0, 6.0)

    v_sub = v2 - v1
    assert v_sub.x == 3.0
    assert v_sub.y == 3.0
    assert v_sub.z == 3.0


def test_vector_scalar_multiplication() -> None:
    """Test vector multiplication by a scalar."""
    v1 = Vector(1.0, 2.0, 3.0)

    v_mul = v1 * 2.0
    assert v_mul.x == 2.0
    assert v_mul.y == 4.0
    assert v_mul.z == 6.0

    # Test right multiplication
    v_rmul = 2.0 * v1
    assert v_rmul.x == 2.0
    assert v_rmul.y == 4.0
    assert v_rmul.z == 6.0


def test_vector_scalar_division() -> None:
    """Test vector division by a scalar."""
    v2 = Vector(4.0, 5.0, 6.0)

    v_div = v2 / 2.0
    assert v_div.x == 2.0
    assert v_div.y == 2.5
    assert v_div.z == 3.0


def test_vector_dot_product() -> None:
    """Test vector dot product."""
    v1 = Vector(1.0, 2.0, 3.0)
    v2 = Vector(4.0, 5.0, 6.0)

    dot = v1.dot(v2)
    assert dot == 32.0


def test_vector_length() -> None:
    """Test vector length calculation."""
    # 2D vector with length 5
    v1 = Vector(3.0, 4.0)
    assert v1.length() == 5.0

    # 3D vector
    v2 = Vector(2.0, 3.0, 6.0)
    assert v2.length() == pytest.approx(7.0, 0.001)

    # Test length_squared
    assert v1.length_squared() == 25.0
    assert v2.length_squared() == 49.0


def test_vector_normalize() -> None:
    """Test vector normalization."""
    v = Vector(2.0, 3.0, 6.0)
    assert not v.is_zero()

    v_norm = v.normalize()
    length = v.length()
    expected_x = 2.0 / length
    expected_y = 3.0 / length
    expected_z = 6.0 / length

    assert np.isclose(v_norm.x, expected_x)
    assert np.isclose(v_norm.y, expected_y)
    assert np.isclose(v_norm.z, expected_z)
    assert np.isclose(v_norm.length(), 1.0)
    assert not v_norm.is_zero()

    # Test normalizing a zero vector
    v_zero = Vector(0.0, 0.0, 0.0)
    assert v_zero.is_zero()
    v_zero_norm = v_zero.normalize()
    assert v_zero_norm.x == 0.0
    assert v_zero_norm.y == 0.0
    assert v_zero_norm.z == 0.0
    assert v_zero_norm.is_zero()


def test_vector_to_2d() -> None:
    """Test conversion to 2D vector."""
    v = Vector(2.0, 3.0, 6.0)

    v_2d = v.to_2d()
    assert v_2d.x == 2.0
    assert v_2d.y == 3.0
    assert v_2d.z == 0.0
    assert v_2d.dim == 2

    # Already 2D vector
    v2 = Vector(4.0, 5.0)
    v2_2d = v2.to_2d()
    assert v2_2d.x == 4.0
    assert v2_2d.y == 5.0
    assert v2_2d.dim == 2


def test_vector_distance() -> None:
    """Test distance calculations between vectors."""
    v1 = Vector(1.0, 2.0, 3.0)
    v2 = Vector(4.0, 6.0, 8.0)

    # Distance
    dist = v1.distance(v2)
    expected_dist = np.sqrt(9.0 + 16.0 + 25.0)  # sqrt((4-1)² + (6-2)² + (8-3)²)
    assert dist == pytest.approx(expected_dist)

    # Distance squared
    dist_sq = v1.distance_squared(v2)
    assert dist_sq == 50.0  # 9 + 16 + 25


def test_vector_cross_product() -> None:
    """Test vector cross product."""
    v1 = Vector(1.0, 0.0, 0.0)  # Unit x vector
    v2 = Vector(0.0, 1.0, 0.0)  # Unit y vector

    # v1 × v2 should be unit z vector
    cross = v1.cross(v2)
    assert cross.x == 0.0
    assert cross.y == 0.0
    assert cross.z == 1.0

    # Test with more complex vectors
    a = Vector(2.0, 3.0, 4.0)
    b = Vector(5.0, 6.0, 7.0)
    c = a.cross(b)

    # Cross product manually calculated:
    # (3*7-4*6, 4*5-2*7, 2*6-3*5)
    assert c.x == -3.0
    assert c.y == 6.0
    assert c.z == -3.0

    # Test with 2D vectors (should raise error)
    v_2d = Vector(1.0, 2.0)
    with pytest.raises(ValueError):
        v_2d.cross(v2)


def test_vector_zeros() -> None:
    """Test Vector.zeros class method."""
    # 3D zero vector
    v_zeros = Vector.zeros(3)
    assert v_zeros.x == 0.0
    assert v_zeros.y == 0.0
    assert v_zeros.z == 0.0
    assert v_zeros.dim == 3
    assert v_zeros.is_zero()

    # 2D zero vector
    v_zeros_2d = Vector.zeros(2)
    assert v_zeros_2d.x == 0.0
    assert v_zeros_2d.y == 0.0
    assert v_zeros_2d.z == 0.0
    assert v_zeros_2d.dim == 2
    assert v_zeros_2d.is_zero()


def test_vector_ones() -> None:
    """Test Vector.ones class method."""
    # 3D ones vector
    v_ones = Vector.ones(3)
    assert v_ones.x == 1.0
    assert v_ones.y == 1.0
    assert v_ones.z == 1.0
    assert v_ones.dim == 3

    # 2D ones vector
    v_ones_2d = Vector.ones(2)
    assert v_ones_2d.x == 1.0
    assert v_ones_2d.y == 1.0
    assert v_ones_2d.z == 0.0
    assert v_ones_2d.dim == 2


def test_vector_conversion_methods() -> None:
    """Test vector conversion methods (to_list, to_tuple, to_numpy)."""
    v = Vector(1.0, 2.0, 3.0)

    # to_list
    assert v.to_list() == [1.0, 2.0, 3.0]

    # to_tuple
    assert v.to_tuple() == (1.0, 2.0, 3.0)

    # to_numpy
    np_array = v.to_numpy()
    assert isinstance(np_array, np.ndarray)
    assert np.array_equal(np_array, np.array([1.0, 2.0, 3.0]))


def test_vector_equality() -> None:
    """Test vector equality."""
    v1 = Vector(1, 2, 3)
    v2 = Vector(1, 2, 3)
    v3 = Vector(4, 5, 6)

    assert v1 == v2
    assert v1 != v3
    assert v1 != Vector(1, 2)  # Different dimensions
    assert v1 != Vector(1.1, 2, 3)  # Different values
    assert v1 != [1, 2, 3]


def test_vector_is_zero() -> None:
    """Test is_zero method for vectors."""
    # Default empty vector
    v0 = Vector()
    assert v0.is_zero()

    # Explicit zero vector
    v1 = Vector(0.0, 0.0, 0.0)
    assert v1.is_zero()

    # Zero vector with different dimensions
    v2 = Vector(0.0, 0.0)
    assert v2.is_zero()

    # Non-zero vectors
    v3 = Vector(1.0, 0.0, 0.0)
    assert not v3.is_zero()

    v4 = Vector(0.0, 2.0, 0.0)
    assert not v4.is_zero()

    v5 = Vector(0.0, 0.0, 3.0)
    assert not v5.is_zero()

    # Almost zero (within tolerance)
    v6 = Vector(1e-10, 1e-10, 1e-10)
    assert v6.is_zero()

    # Almost zero (outside tolerance)
    v7 = Vector(1e-6, 1e-6, 1e-6)
    assert not v7.is_zero()


def test_vector_bool_conversion():
    """Test boolean conversion of vectors."""
    # Zero vectors should be False
    v0 = Vector()
    assert not bool(v0)

    v1 = Vector(0.0, 0.0, 0.0)
    assert not bool(v1)

    # Almost zero vectors should be False
    v2 = Vector(1e-10, 1e-10, 1e-10)
    assert not bool(v2)

    # Non-zero vectors should be True
    v3 = Vector(1.0, 0.0, 0.0)
    assert bool(v3)

    v4 = Vector(0.0, 2.0, 0.0)
    assert bool(v4)

    v5 = Vector(0.0, 0.0, 3.0)
    assert bool(v5)

    # Direct use in if statements
    if v0:
        raise AssertionError("Zero vector should be False in boolean context")
    else:
        pass  # Expected path

    if v3:
        pass  # Expected path
    else:
        raise AssertionError("Non-zero vector should be True in boolean context")


def test_vector_add() -> None:
    """Test vector addition operator."""
    v1 = Vector(1.0, 2.0, 3.0)
    v2 = Vector(4.0, 5.0, 6.0)

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
    v_zero = Vector.zeros(3)
    assert (v1 + v_zero) == v1


def test_vector_add_dim_mismatch() -> None:
    """Test vector addition operator."""
    v1 = Vector(1.0, 2.0)
    v2 = Vector(4.0, 5.0, 6.0)

    # Using + operator
    v1 + v2
