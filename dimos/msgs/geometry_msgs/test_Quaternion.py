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
from lcm_msgs.geometry_msgs import Quaternion as LCMQuaternion

from dimos.msgs.geometry_msgs.Quaternion import Quaternion


def test_quaternion_default_init():
    """Test that default initialization creates an identity quaternion (w=1, x=y=z=0)."""
    q = Quaternion()
    assert q.x == 0.0
    assert q.y == 0.0
    assert q.z == 0.0
    assert q.w == 1.0
    assert q.to_tuple() == (0.0, 0.0, 0.0, 1.0)


def test_quaternion_component_init():
    """Test initialization with four float components (x, y, z, w)."""
    q = Quaternion(0.5, 0.5, 0.5, 0.5)
    assert q.x == 0.5
    assert q.y == 0.5
    assert q.z == 0.5
    assert q.w == 0.5

    # Test with different values
    q2 = Quaternion(1.0, 2.0, 3.0, 4.0)
    assert q2.x == 1.0
    assert q2.y == 2.0
    assert q2.z == 3.0
    assert q2.w == 4.0

    # Test with negative values
    q3 = Quaternion(-1.0, -2.0, -3.0, -4.0)
    assert q3.x == -1.0
    assert q3.y == -2.0
    assert q3.z == -3.0
    assert q3.w == -4.0

    # Test with integers (should convert to float)
    q4 = Quaternion(1, 2, 3, 4)
    assert q4.x == 1.0
    assert q4.y == 2.0
    assert q4.z == 3.0
    assert q4.w == 4.0
    assert isinstance(q4.x, float)


def test_quaternion_sequence_init():
    """Test initialization from sequence (list, tuple) of 4 numbers."""
    # From list
    q1 = Quaternion([0.1, 0.2, 0.3, 0.4])
    assert q1.x == 0.1
    assert q1.y == 0.2
    assert q1.z == 0.3
    assert q1.w == 0.4

    # From tuple
    q2 = Quaternion((0.5, 0.6, 0.7, 0.8))
    assert q2.x == 0.5
    assert q2.y == 0.6
    assert q2.z == 0.7
    assert q2.w == 0.8

    # Test with integers in sequence
    q3 = Quaternion([1, 2, 3, 4])
    assert q3.x == 1.0
    assert q3.y == 2.0
    assert q3.z == 3.0
    assert q3.w == 4.0

    # Test error with wrong length
    with pytest.raises(ValueError, match="Quaternion requires exactly 4 components"):
        Quaternion([1, 2, 3])  # Only 3 components

    with pytest.raises(ValueError, match="Quaternion requires exactly 4 components"):
        Quaternion([1, 2, 3, 4, 5])  # Too many components


def test_quaternion_numpy_init():
    """Test initialization from numpy array."""
    # From numpy array
    arr = np.array([0.1, 0.2, 0.3, 0.4])
    q1 = Quaternion(arr)
    assert q1.x == 0.1
    assert q1.y == 0.2
    assert q1.z == 0.3
    assert q1.w == 0.4

    # Test with different dtypes
    arr_int = np.array([1, 2, 3, 4], dtype=int)
    q2 = Quaternion(arr_int)
    assert q2.x == 1.0
    assert q2.y == 2.0
    assert q2.z == 3.0
    assert q2.w == 4.0

    # Test error with wrong size
    with pytest.raises(ValueError, match="Quaternion requires exactly 4 components"):
        Quaternion(np.array([1, 2, 3]))  # Only 3 elements

    with pytest.raises(ValueError, match="Quaternion requires exactly 4 components"):
        Quaternion(np.array([1, 2, 3, 4, 5]))  # Too many elements


def test_quaternion_copy_init():
    """Test initialization from another Quaternion (copy constructor)."""
    original = Quaternion(0.1, 0.2, 0.3, 0.4)
    copy = Quaternion(original)

    assert copy.x == 0.1
    assert copy.y == 0.2
    assert copy.z == 0.3
    assert copy.w == 0.4

    # Verify it's a copy, not the same object
    assert copy is not original
    assert copy == original


def test_quaternion_lcm_init():
    """Test initialization from LCM Quaternion."""
    lcm_quat = LCMQuaternion()
    lcm_quat.x = 0.1
    lcm_quat.y = 0.2
    lcm_quat.z = 0.3
    lcm_quat.w = 0.4

    q = Quaternion(lcm_quat)
    assert q.x == 0.1
    assert q.y == 0.2
    assert q.z == 0.3
    assert q.w == 0.4


def test_quaternion_properties():
    """Test quaternion component properties."""
    q = Quaternion(1.0, 2.0, 3.0, 4.0)

    # Test property access
    assert q.x == 1.0
    assert q.y == 2.0
    assert q.z == 3.0
    assert q.w == 4.0

    # Test as_tuple property
    assert q.to_tuple() == (1.0, 2.0, 3.0, 4.0)


def test_quaternion_indexing():
    """Test quaternion indexing support."""
    q = Quaternion(1.0, 2.0, 3.0, 4.0)

    # Test indexing
    assert q[0] == 1.0
    assert q[1] == 2.0
    assert q[2] == 3.0
    assert q[3] == 4.0


def test_quaternion_euler():
    """Test quaternion to Euler angles conversion."""

    # Test identity quaternion (should give zero angles)
    q_identity = Quaternion()
    angles = q_identity.to_euler()
    assert np.isclose(angles.x, 0.0, atol=1e-10)  # roll
    assert np.isclose(angles.y, 0.0, atol=1e-10)  # pitch
    assert np.isclose(angles.z, 0.0, atol=1e-10)  # yaw

    # Test 90 degree rotation around Z-axis (yaw)
    q_z90 = Quaternion(0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4))
    angles_z90 = q_z90.to_euler()
    assert np.isclose(angles_z90.roll, 0.0, atol=1e-10)  # roll should be 0
    assert np.isclose(angles_z90.pitch, 0.0, atol=1e-10)  # pitch should be 0
    assert np.isclose(angles_z90.yaw, np.pi / 2, atol=1e-10)  # yaw should be π/2 (90 degrees)

    # Test 90 degree rotation around X-axis (roll)
    q_x90 = Quaternion(np.sin(np.pi / 4), 0, 0, np.cos(np.pi / 4))
    angles_x90 = q_x90.to_euler()
    assert np.isclose(angles_x90.x, np.pi / 2, atol=1e-10)  # roll should be π/2
    assert np.isclose(angles_x90.y, 0.0, atol=1e-10)  # pitch should be 0
    assert np.isclose(angles_x90.z, 0.0, atol=1e-10)  # yaw should be 0


def test_lcm_encode_decode():
    """Test encoding and decoding of Quaternion to/from binary LCM format."""
    q_source = Quaternion(1.0, 2.0, 3.0, 4.0)

    binary_msg = q_source.lcm_encode()

    q_dest = Quaternion.lcm_decode(binary_msg)

    assert isinstance(q_dest, Quaternion)
    assert q_dest is not q_source
    assert q_dest == q_source
