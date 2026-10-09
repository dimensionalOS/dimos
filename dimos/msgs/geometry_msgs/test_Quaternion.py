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

from dimos_generated.geometry_msgs.msg import Quaternion, Vector3
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest

from dimos.msgs.geometry import (
    normalized_quaternion,
    quaternion_array,
    quaternion_conjugate,
    quaternion_euler,
    quaternion_from_array,
    quaternion_inverse,
    quaternion_product,
    rotate_vector,
    vector_from_array,
)


def test_quaternion_default_init() -> None:
    """Test that default initialization creates an identity quaternion (w=1, x=y=z=0)."""
    q = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    assert q.x == 0.0
    assert q.y == 0.0
    assert q.z == 0.0
    assert q.w == 1.0
    assert tuple(quaternion_array(q)) == (0.0, 0.0, 0.0, 1.0)


def test_quaternion_component_init() -> None:
    """Test initialization with four float components (x, y, z, w)."""
    q = Quaternion(x=0.5, y=0.5, z=0.5, w=0.5)
    assert q.x == 0.5
    assert q.y == 0.5
    assert q.z == 0.5
    assert q.w == 0.5
    q2 = Quaternion(x=1.0, y=2.0, z=3.0, w=4.0)
    assert q2.x == 1.0
    assert q2.y == 2.0
    assert q2.z == 3.0
    assert q2.w == 4.0
    q3 = Quaternion(x=-1.0, y=-2.0, z=-3.0, w=-4.0)
    assert q3.x == -1.0
    assert q3.y == -2.0
    assert q3.z == -3.0
    assert q3.w == -4.0
    q4 = Quaternion(x=1, y=2, z=3, w=4)
    assert q4.x == 1.0
    assert q4.y == 2.0
    assert q4.z == 3.0
    assert q4.w == 4.0
    assert isinstance(q4.x, float)


def test_quaternion_sequence_init() -> None:
    """Test initialization from sequence (list, tuple) of 4 numbers."""
    q1 = quaternion_from_array([0.1, 0.2, 0.3, 0.4])
    assert q1.x == 0.1
    assert q1.y == 0.2
    assert q1.z == 0.3
    assert q1.w == 0.4
    q2 = quaternion_from_array((0.5, 0.6, 0.7, 0.8))
    assert q2.x == 0.5
    assert q2.y == 0.6
    assert q2.z == 0.7
    assert q2.w == 0.8
    q3 = quaternion_from_array([1, 2, 3, 4])
    assert q3.x == 1.0
    assert q3.y == 2.0
    assert q3.z == 3.0
    assert q3.w == 4.0
    with pytest.raises(ValueError, match="Quaternion requires exactly 4 components"):
        quaternion_from_array([1, 2, 3])
    with pytest.raises(ValueError, match="Quaternion requires exactly 4 components"):
        quaternion_from_array([1, 2, 3, 4, 5])


def test_quaternion_numpy_init() -> None:
    """Test initialization from numpy array."""
    arr = np.array([0.1, 0.2, 0.3, 0.4])
    q1 = quaternion_from_array(arr)
    assert q1.x == 0.1
    assert q1.y == 0.2
    assert q1.z == 0.3
    assert q1.w == 0.4
    arr_int = np.array([1, 2, 3, 4], dtype=int)
    q2 = quaternion_from_array(arr_int)
    assert q2.x == 1.0
    assert q2.y == 2.0
    assert q2.z == 3.0
    assert q2.w == 4.0
    with pytest.raises(ValueError, match="Quaternion requires exactly 4 components"):
        quaternion_from_array(np.array([1, 2, 3]))
    with pytest.raises(ValueError, match="Quaternion requires exactly 4 components"):
        quaternion_from_array(np.array([1, 2, 3, 4, 5]))


def test_quaternion_copy_init() -> None:
    """Test initialization from another Quaternion (copy constructor)."""
    original = Quaternion(x=0.1, y=0.2, z=0.3, w=0.4)
    copy = cdr_decode(cdr_encode(original), Quaternion)
    assert copy.x == 0.1
    assert copy.y == 0.2
    assert copy.z == 0.3
    assert copy.w == 0.4
    assert copy is not original
    assert copy == original


def test_quaternion_mutated_value_copy() -> None:
    """Test initialization from generated Quaternion."""
    source_quaternion = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    source_quaternion.x = 0.1
    source_quaternion.y = 0.2
    source_quaternion.z = 0.3
    source_quaternion.w = 0.4
    q = cdr_decode(cdr_encode(source_quaternion), Quaternion)
    assert q.x == 0.1
    assert q.y == 0.2
    assert q.z == 0.3
    assert q.w == 0.4


def test_quaternion_properties() -> None:
    """Test quaternion component properties."""
    q = Quaternion(x=1.0, y=2.0, z=3.0, w=4.0)
    assert q.x == 1.0
    assert q.y == 2.0
    assert q.z == 3.0
    assert q.w == 4.0
    assert tuple(quaternion_array(q)) == (1.0, 2.0, 3.0, 4.0)


def test_quaternion_indexing() -> None:
    """Test quaternion indexing support."""
    q = Quaternion(x=1.0, y=2.0, z=3.0, w=4.0)
    assert quaternion_array(q)[0] == 1.0
    assert quaternion_array(q)[1] == 2.0
    assert quaternion_array(q)[2] == 3.0
    assert quaternion_array(q)[3] == 4.0


def test_quaternion_euler() -> None:
    """Test quaternion to Euler angles conversion."""
    q_identity = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    angles = vector_from_array(quaternion_euler(q_identity))
    assert np.isclose(angles.x, 0.0, atol=1e-10)
    assert np.isclose(angles.y, 0.0, atol=1e-10)
    assert np.isclose(angles.z, 0.0, atol=1e-10)
    q_z90 = Quaternion(x=0, y=0, z=np.sin(np.pi / 4), w=np.cos(np.pi / 4))
    angles_z90 = vector_from_array(quaternion_euler(q_z90))
    assert np.isclose(angles_z90.x, 0.0, atol=1e-10)
    assert np.isclose(angles_z90.y, 0.0, atol=1e-10)
    assert np.isclose(angles_z90.z, np.pi / 2, atol=1e-10)
    q_x90 = Quaternion(x=np.sin(np.pi / 4), y=0, z=0, w=np.cos(np.pi / 4))
    angles_x90 = vector_from_array(quaternion_euler(q_x90))
    assert np.isclose(angles_x90.x, np.pi / 2, atol=1e-10)
    assert np.isclose(angles_x90.y, 0.0, atol=1e-10)
    assert np.isclose(angles_x90.z, 0.0, atol=1e-10)


def test_cdr_encode_decode() -> None:
    """Test encoding and decoding of Quaternion to/from binary CDR format."""
    q_source = Quaternion(x=1.0, y=2.0, z=3.0, w=4.0)
    binary_msg = cdr_encode(q_source)
    q_dest = cdr_decode(binary_msg, Quaternion)
    assert isinstance(q_dest, Quaternion)
    assert q_dest is not q_source
    assert q_dest == q_source


def test_quaternion_multiplication() -> None:
    """Test quaternion multiplication (Hamilton product)."""
    q1 = Quaternion(x=0.5, y=0.5, z=0.5, w=0.5)
    identity = Quaternion(x=0, y=0, z=0, w=1)
    result = quaternion_product(q1, identity)
    assert np.allclose([result.x, result.y, result.z, result.w], [q1.x, q1.y, q1.z, q1.w])
    q2 = Quaternion(x=0.1, y=0.2, z=0.3, w=0.4)
    q3 = Quaternion(x=0.4, y=0.3, z=0.2, w=0.1)
    result1 = quaternion_product(q2, q3)
    result2 = quaternion_product(q3, q2)
    assert not np.allclose(
        [result1.x, result1.y, result1.z, result1.w], [result2.x, result2.y, result2.z, result2.w]
    )
    angle = np.pi / 2
    q_90z = Quaternion(x=0, y=0, z=np.sin(angle / 2), w=np.cos(angle / 2))
    result = quaternion_product(q_90z, q_90z)
    expected_angle = np.pi
    assert np.isclose(result.x, 0, atol=1e-10)
    assert np.isclose(result.y, 0, atol=1e-10)
    assert np.isclose(result.z, np.sin(expected_angle / 2), atol=1e-10)
    assert np.isclose(result.w, np.cos(expected_angle / 2), atol=1e-10)


def test_quaternion_conjugate() -> None:
    """Test quaternion conjugate."""
    q = Quaternion(x=0.1, y=0.2, z=0.3, w=0.4)
    conj = quaternion_conjugate(q)
    assert conj.x == -q.x
    assert conj.y == -q.y
    assert conj.z == -q.z
    assert conj.w == q.w
    result = quaternion_product(q, conj)
    assert np.isclose(result.x, 0, atol=1e-10)
    assert np.isclose(result.y, 0, atol=1e-10)
    assert np.isclose(result.z, 0, atol=1e-10)
    expected_w = q.x**2 + q.y**2 + q.z**2 + q.w**2
    assert np.isclose(result.w, expected_w, atol=1e-10)


def test_quaternion_inverse() -> None:
    """Test quaternion inverse."""
    q_unit = normalized_quaternion(Quaternion(x=0, y=0, z=0, w=1))
    inv = quaternion_inverse(q_unit)
    conj = quaternion_conjugate(q_unit)
    assert np.allclose([inv.x, inv.y, inv.z, inv.w], [conj.x, conj.y, conj.z, conj.w])
    q = Quaternion(x=0.5, y=0.5, z=0.5, w=0.5)
    inv = quaternion_inverse(q)
    result = quaternion_product(q, inv)
    assert np.isclose(result.x, 0, atol=1e-10)
    assert np.isclose(result.y, 0, atol=1e-10)
    assert np.isclose(result.z, 0, atol=1e-10)
    assert np.isclose(result.w, 1, atol=1e-10)
    q_non_unit = Quaternion(x=2, y=0, z=0, w=0)
    inv = quaternion_inverse(q_non_unit)
    result = quaternion_product(q_non_unit, inv)
    assert np.isclose(result.x, 0, atol=1e-10)
    assert np.isclose(result.y, 0, atol=1e-10)
    assert np.isclose(result.z, 0, atol=1e-10)
    assert np.isclose(result.w, 1, atol=1e-10)


def test_quaternion_normalize() -> None:
    """Test quaternion normalization."""
    q = Quaternion(x=1, y=2, z=3, w=4)
    q_norm = normalized_quaternion(q)
    magnitude = np.sqrt(q_norm.x**2 + q_norm.y**2 + q_norm.z**2 + q_norm.w**2)
    assert np.isclose(magnitude, 1.0, atol=1e-10)
    scale = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    assert np.isclose(q_norm.x, q.x / scale, atol=1e-10)
    assert np.isclose(q_norm.y, q.y / scale, atol=1e-10)
    assert np.isclose(q_norm.z, q.z / scale, atol=1e-10)
    assert np.isclose(q_norm.w, q.w / scale, atol=1e-10)


def test_quaternion_rotate_vector() -> None:
    """Test rotating vectors with quaternions."""
    angle = np.pi / 2
    q_rot = Quaternion(x=0, y=0, z=np.sin(angle / 2), w=np.cos(angle / 2))
    v_x = Vector3(x=1, y=0, z=0)
    v_rotated = rotate_vector(q_rot, v_x)
    assert np.isclose(v_rotated.x, 0, atol=1e-10)
    assert np.isclose(v_rotated.y, 1, atol=1e-10)
    assert np.isclose(v_rotated.z, 0, atol=1e-10)
    v_y = Vector3(x=0, y=1, z=0)
    v_rotated = rotate_vector(q_rot, v_y)
    assert np.isclose(v_rotated.x, -1, atol=1e-10)
    assert np.isclose(v_rotated.y, 0, atol=1e-10)
    assert np.isclose(v_rotated.z, 0, atol=1e-10)
    v_z = Vector3(x=0, y=0, z=1)
    v_rotated = rotate_vector(q_rot, v_z)
    assert np.isclose(v_rotated.x, 0, atol=1e-10)
    assert np.isclose(v_rotated.y, 0, atol=1e-10)
    assert np.isclose(v_rotated.z, 1, atol=1e-10)
    q_identity = Quaternion(x=0, y=0, z=0, w=1)
    v = Vector3(x=1, y=2, z=3)
    v_rotated = rotate_vector(q_identity, v)
    assert np.isclose(v_rotated.x, v.x, atol=1e-10)
    assert np.isclose(v_rotated.y, v.y, atol=1e-10)
    assert np.isclose(v_rotated.z, v.z, atol=1e-10)


def test_quaternion_inverse_zero() -> None:
    """Test that inverting zero quaternion raises error."""
    q_zero = Quaternion(x=0, y=0, z=0, w=0)
    with pytest.raises(ZeroDivisionError, match="Cannot invert zero quaternion"):
        quaternion_inverse(q_zero)


def test_quaternion_normalize_zero() -> None:
    """Test that normalizing zero quaternion raises error."""
    q_zero = Quaternion(x=0, y=0, z=0, w=0)
    with pytest.raises(ZeroDivisionError, match="Cannot normalize zero quaternion"):
        normalized_quaternion(q_zero)


def test_quaternion_multiplication_type_error() -> None:
    """Test that multiplying quaternion with non-quaternion raises error."""
    q = Quaternion(x=1, y=0, z=0, w=0)
    with pytest.raises(TypeError, match="Cannot multiply Quaternion with"):
        quaternion_product(q, 5.0)
    with pytest.raises(TypeError, match="Cannot multiply Quaternion with"):
        quaternion_product(q, [1, 2, 3, 4])
