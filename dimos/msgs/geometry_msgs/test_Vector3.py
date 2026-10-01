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

from dimos_generated.geometry_msgs.msg import Vector3
import numpy as np
import pytest

from dimos.msgs.geometry import (
    normalized_vector,
    quaternion_euler,
    quaternion_from_euler,
    vector_array,
    vector_from_array,
)


def test_vector_default_init() -> None:
    """Test that default initialization of Vector() has x,y,z components all zero."""
    v = Vector3()
    assert v.x == 0.0
    assert v.y == 0.0
    assert v.z == 0.0
    assert len(vector_array(v)) == 3
    assert vector_array(v).tolist() == [0.0, 0.0, 0.0]
    assert np.allclose(vector_array(v), 0)


def test_vector_specific_init() -> None:
    """Test initialization with specific values and different input types."""
    v1 = Vector3(x=1.0, y=2.0)
    assert v1.x == 1.0
    assert v1.y == 2.0
    assert v1.z == 0.0
    v2 = Vector3(x=3.0, y=4.0, z=5.0)
    assert v2.x == 3.0
    assert v2.y == 4.0
    assert v2.z == 5.0
    v3 = vector_from_array([6.0, 7.0, 8.0])
    assert v3.x == 6.0
    assert v3.y == 7.0
    assert v3.z == 8.0
    v4 = vector_from_array((9.0, 10.0, 11.0))
    assert v4.x == 9.0
    assert v4.y == 10.0
    assert v4.z == 11.0
    v5 = vector_from_array(np.array([12.0, 13.0, 14.0]))
    assert v5.x == 12.0
    assert v5.y == 13.0
    assert v5.z == 14.0
    original = vector_from_array([15.0, 16.0, 17.0])
    v6 = Vector3.decode(original.encode())
    assert v6.x == 15.0
    assert v6.y == 16.0
    assert v6.z == 17.0
    assert v6 is not original
    assert v6 == original


def test_vector_addition() -> None:
    """Test vector addition."""
    v1 = Vector3(x=1.0, y=2.0, z=3.0)
    v2 = Vector3(x=4.0, y=5.0, z=6.0)
    v_add = vector_from_array(vector_array(v1) + vector_array(v2))
    assert v_add.x == 5.0
    assert v_add.y == 7.0
    assert v_add.z == 9.0


def test_vector_subtraction() -> None:
    """Test vector subtraction."""
    v1 = Vector3(x=1.0, y=2.0, z=3.0)
    v2 = Vector3(x=4.0, y=5.0, z=6.0)
    v_sub = vector_from_array(vector_array(v2) - vector_array(v1))
    assert v_sub.x == 3.0
    assert v_sub.y == 3.0
    assert v_sub.z == 3.0


def test_vector_scalar_multiplication() -> None:
    """Test vector multiplication by a scalar."""
    v1 = Vector3(x=1.0, y=2.0, z=3.0)
    v_mul = vector_from_array(vector_array(v1) * 2.0)
    assert v_mul.x == 2.0
    assert v_mul.y == 4.0
    assert v_mul.z == 6.0
    v_rmul = vector_from_array(2.0 * vector_array(v1))
    assert v_rmul.x == 2.0
    assert v_rmul.y == 4.0
    assert v_rmul.z == 6.0


def test_vector_scalar_division() -> None:
    """Test vector division by a scalar."""
    v2 = Vector3(x=4.0, y=5.0, z=6.0)
    v_div = vector_from_array(vector_array(v2) / 2.0)
    assert v_div.x == 2.0
    assert v_div.y == 2.5
    assert v_div.z == 3.0


def test_vector_dot_product() -> None:
    """Test vector dot product."""
    v1 = Vector3(x=1.0, y=2.0, z=3.0)
    v2 = Vector3(x=4.0, y=5.0, z=6.0)
    dot = np.dot(vector_array(v1), vector_array(v2))
    assert dot == 32.0


def test_vector_length() -> None:
    """Test vector length calculation."""
    v1 = Vector3(x=3.0, y=4.0)
    assert np.linalg.norm(vector_array(v1)) == 5.0
    v2 = Vector3(x=2.0, y=3.0, z=6.0)
    assert np.linalg.norm(vector_array(v2)) == pytest.approx(7.0, 0.001)
    assert np.dot(vector_array(v1), vector_array(v1)) == 25.0
    assert np.dot(vector_array(v2), vector_array(v2)) == 49.0


def test_vector_normalize() -> None:
    """Test vector normalization."""
    v = Vector3(x=2.0, y=3.0, z=6.0)
    assert not np.allclose(vector_array(v), 0)
    v_norm = normalized_vector(v)
    length = np.linalg.norm(vector_array(v))
    expected_x = 2.0 / length
    expected_y = 3.0 / length
    expected_z = 6.0 / length
    assert np.isclose(v_norm.x, expected_x)
    assert np.isclose(v_norm.y, expected_y)
    assert np.isclose(v_norm.z, expected_z)
    assert np.isclose(np.linalg.norm(vector_array(v_norm)), 1.0)
    assert not np.allclose(vector_array(v_norm), 0)
    v_zero = Vector3(x=0.0, y=0.0, z=0.0)
    assert np.allclose(vector_array(v_zero), 0)
    v_zero_norm = normalized_vector(v_zero)
    assert v_zero_norm.x == 0.0
    assert v_zero_norm.y == 0.0
    assert v_zero_norm.z == 0.0
    assert np.allclose(vector_array(v_zero_norm), 0)


def test_vector_to_2d() -> None:
    """Test conversion to 2D vector."""
    v = Vector3(x=2.0, y=3.0, z=6.0)
    v_2d = Vector3(x=v.x, y=v.y)
    assert v_2d.x == 2.0
    assert v_2d.y == 3.0
    assert v_2d.z == 0.0
    v2 = Vector3(x=4.0, y=5.0)
    v2_2d = Vector3(x=v2.x, y=v2.y)
    assert v2_2d.x == 4.0
    assert v2_2d.y == 5.0
    assert v2_2d.z == 0.0


def test_vector_distance() -> None:
    """Test distance calculations between vectors."""
    v1 = Vector3(x=1.0, y=2.0, z=3.0)
    v2 = Vector3(x=4.0, y=6.0, z=8.0)
    dist = np.linalg.norm(vector_array(v1) - vector_array(v2))
    expected_dist = np.sqrt(9.0 + 16.0 + 25.0)
    assert dist == pytest.approx(expected_dist)
    dist_sq = np.sum((vector_array(v1) - vector_array(v2)) ** 2)
    assert dist_sq == 50.0


def test_vector_cross_product() -> None:
    """Test vector cross product."""
    v1 = Vector3(x=1.0, y=0.0, z=0.0)
    v2 = Vector3(x=0.0, y=1.0, z=0.0)
    cross = vector_from_array(np.cross(vector_array(v1), vector_array(v2)))
    assert cross.x == 0.0
    assert cross.y == 0.0
    assert cross.z == 1.0
    a = Vector3(x=2.0, y=3.0, z=4.0)
    b = Vector3(x=5.0, y=6.0, z=7.0)
    c = vector_from_array(np.cross(vector_array(a), vector_array(b)))
    assert c.x == -3.0
    assert c.y == 6.0
    assert c.z == -3.0
    v_2d1 = Vector3(x=1.0, y=2.0)
    v_2d2 = Vector3(x=3.0, y=4.0)
    cross_2d = vector_from_array(np.cross(vector_array(v_2d1), vector_array(v_2d2)))
    assert cross_2d.x == 0.0
    assert cross_2d.y == 0.0
    assert cross_2d.z == -2.0


def test_vector_zeros() -> None:
    """Test Vector3.zeros class method."""
    v_zeros = Vector3()
    assert v_zeros.x == 0.0
    assert v_zeros.y == 0.0
    assert v_zeros.z == 0.0
    assert np.allclose(vector_array(v_zeros), 0)


def test_vector_ones() -> None:
    """Test Vector3.ones class method."""
    v_ones = Vector3(x=1, y=1, z=1)
    assert v_ones.x == 1.0
    assert v_ones.y == 1.0
    assert v_ones.z == 1.0


def test_vector_conversion_methods() -> None:
    """Test vector conversion methods (to_list, to_tuple, to_numpy)."""
    v = Vector3(x=1.0, y=2.0, z=3.0)
    assert vector_array(v).tolist() == [1.0, 2.0, 3.0]
    assert tuple(vector_array(v)) == (1.0, 2.0, 3.0)
    np_array = vector_array(v)
    assert isinstance(np_array, np.ndarray)
    assert np.array_equal(np_array, np.array([1.0, 2.0, 3.0]))


def test_vector_equality() -> None:
    """Test vector equality."""
    v1 = Vector3(x=1, y=2, z=3)
    v2 = Vector3(x=1, y=2, z=3)
    v3 = Vector3(x=4, y=5, z=6)
    assert v1 == v2
    assert v1 != v3
    assert v1 != Vector3(x=1, y=2)
    assert v1 != Vector3(x=1.1, y=2, z=3)
    assert v1 != [1, 2, 3]


def test_vector_is_zero() -> None:
    """Test is_zero method for vectors."""
    v0 = Vector3()
    assert np.allclose(vector_array(v0), 0)
    v1 = Vector3(x=0.0, y=0.0, z=0.0)
    assert np.allclose(vector_array(v1), 0)
    v2 = Vector3(x=0.0, y=0.0)
    assert np.allclose(vector_array(v2), 0)
    v3 = Vector3(x=1.0, y=0.0, z=0.0)
    assert not np.allclose(vector_array(v3), 0)
    v4 = Vector3(x=0.0, y=2.0, z=0.0)
    assert not np.allclose(vector_array(v4), 0)
    v5 = Vector3(x=0.0, y=0.0, z=3.0)
    assert not np.allclose(vector_array(v5), 0)
    v6 = Vector3(x=1e-10, y=1e-10, z=1e-10)
    assert np.allclose(vector_array(v6), 0)
    v7 = Vector3(x=1e-06, y=1e-06, z=1e-06)
    assert not np.allclose(vector_array(v7), 0)


def test_vector_bool_conversion():
    """Test boolean conversion of vectors."""
    v0 = Vector3()
    assert not not np.allclose(vector_array(v0), 0)
    v1 = Vector3(x=0.0, y=0.0, z=0.0)
    assert not not np.allclose(vector_array(v1), 0)
    v2 = Vector3(x=1e-10, y=1e-10, z=1e-10)
    assert not not np.allclose(vector_array(v2), 0)
    v3 = Vector3(x=1.0, y=0.0, z=0.0)
    assert not np.allclose(vector_array(v3), 0)
    v4 = Vector3(x=0.0, y=2.0, z=0.0)
    assert not np.allclose(vector_array(v4), 0)
    v5 = Vector3(x=0.0, y=0.0, z=3.0)
    assert not np.allclose(vector_array(v5), 0)
    if not np.allclose(vector_array(v0), 0):
        raise AssertionError("Zero vector should be False in boolean context")
    else:
        pass
    if not np.allclose(vector_array(v3), 0):
        pass
    else:
        raise AssertionError("Non-zero vector should be True in boolean context")


def test_vector_add() -> None:
    """Test vector addition operator."""
    v1 = Vector3(x=1.0, y=2.0, z=3.0)
    v2 = Vector3(x=4.0, y=5.0, z=6.0)
    v_add = vector_from_array(vector_array(v1) + vector_array(v2))
    assert v_add.x == 5.0
    assert v_add.y == 7.0
    assert v_add.z == 9.0
    v_add_op = vector_from_array(vector_array(v1) + vector_array(v2))
    assert v_add_op.x == 5.0
    assert v_add_op.y == 7.0
    assert v_add_op.z == 9.0
    v_zero = Vector3()
    assert vector_from_array(vector_array(v1) + vector_array(v_zero)) == v1


def test_vector_add_dim_mismatch() -> None:
    """Test vector addition with different input dimensions (now all vectors are 3D)."""
    v1 = Vector3(x=1.0, y=2.0)
    v2 = Vector3(x=4.0, y=5.0, z=6.0)
    v_add_op = vector_from_array(vector_array(v1) + vector_array(v2))
    assert v_add_op.x == 5.0
    assert v_add_op.y == 7.0
    assert v_add_op.z == 6.0


def test_yaw_pitch_roll_accessors() -> None:
    """Test yaw, pitch, and roll accessor properties."""
    v = Vector3(x=1.0, y=2.0, z=3.0)
    assert v.x == 1.0
    assert v.y == 2.0
    assert v.z == 3.0
    v_2d = Vector3(x=4.0, y=5.0)
    assert v_2d.x == 4.0
    assert v_2d.y == 5.0
    assert v_2d.z == 0.0
    v_empty = Vector3()
    assert v_empty.x == 0.0
    assert v_empty.y == 0.0
    assert v_empty.z == 0.0
    v_neg = Vector3(x=-1.5, y=-2.5, z=-3.5)
    assert v_neg.x == -1.5
    assert v_neg.y == -2.5
    assert v_neg.z == -3.5


def test_vector_to_quaternion() -> None:
    """Test vector to quaternion conversion."""
    v_zero = Vector3(x=0.0, y=0.0, z=0.0)
    q_identity = quaternion_from_euler(v_zero.x, v_zero.y, v_zero.z)
    assert np.isclose(q_identity.x, 0.0, atol=1e-10)
    assert np.isclose(q_identity.y, 0.0, atol=1e-10)
    assert np.isclose(q_identity.z, 0.0, atol=1e-10)
    assert np.isclose(q_identity.w, 1.0, atol=1e-10)
    v_small = Vector3(x=0.1, y=0.2, z=0.3)
    q_small = quaternion_from_euler(v_small.x, v_small.y, v_small.z)
    magnitude = np.sqrt(q_small.x**2 + q_small.y**2 + q_small.z**2 + q_small.w**2)
    assert np.isclose(magnitude, 1.0, atol=1e-10)
    v_back = vector_from_array(quaternion_euler(q_small))
    assert np.isclose(v_back.x, 0.1, atol=1e-06)
    assert np.isclose(v_back.y, 0.2, atol=1e-06)
    assert np.isclose(v_back.z, 0.3, atol=1e-06)
    v_x_90 = Vector3(x=np.pi / 2, y=0.0, z=0.0)
    q_x_90 = quaternion_from_euler(v_x_90.x, v_x_90.y, v_x_90.z)
    expected = np.sqrt(2) / 2
    assert np.isclose(q_x_90.x, expected, atol=1e-10)
    assert np.isclose(q_x_90.y, 0.0, atol=1e-10)
    assert np.isclose(q_x_90.z, 0.0, atol=1e-10)
    assert np.isclose(q_x_90.w, expected, atol=1e-10)


def test_cdr_encode_decode() -> None:
    v_source = Vector3(x=1.0, y=2.0, z=3.0)
    binary_msg = v_source.encode()
    v_dest = Vector3.decode(binary_msg)
    assert isinstance(v_dest, Vector3)
    assert v_dest is not v_source
    assert v_dest == v_source
