# Copyright 2026 Dimensional Inc.
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

"""Wire-format compatibility tests.

Validates that Python LCM encoding matches known binary layouts from the Rust
dimos-lcm implementation, ensuring cross-language wire compatibility without
needing to build or run any non-Python code.
"""

from __future__ import annotations

import struct

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3

# Known fingerprints from dimos-lcm Rust tests (roundtrip.rs)
VECTOR3_FINGERPRINT = 0xAE7E5FBA5EECA11E
TWIST_FINGERPRINT = 0x2E7C07D7CDF7E027


def test_vector3_fingerprint() -> None:
    """Python Vector3 encoding starts with the same fingerprint as Rust."""
    v = Vector3(1.5, 2.5, 3.5)
    encoded = v.lcm_encode()
    fingerprint = struct.unpack(">Q", encoded[:8])[0]
    assert fingerprint == VECTOR3_FINGERPRINT, (
        f"Vector3 fingerprint mismatch: got 0x{fingerprint:016X}, "
        f"expected 0x{VECTOR3_FINGERPRINT:016X}"
    )


def test_twist_fingerprint() -> None:
    """Python Twist encoding starts with the same fingerprint as Rust."""
    t = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
    encoded = t.lcm_encode()
    fingerprint = struct.unpack(">Q", encoded[:8])[0]
    assert fingerprint == TWIST_FINGERPRINT, (
        f"Twist fingerprint mismatch: got 0x{fingerprint:016X}, expected 0x{TWIST_FINGERPRINT:016X}"
    )


def test_vector3_known_binary_layout() -> None:
    """Python Vector3(1.5, 2.5, 3.5) produces exact same bytes as Rust."""
    v = Vector3(1.5, 2.5, 3.5)
    encoded = v.lcm_encode()

    # 8-byte fingerprint + 3x8-byte f64 = 32 bytes
    assert len(encoded) == 32

    # Fingerprint
    assert encoded[:8] == struct.pack(">Q", VECTOR3_FINGERPRINT)

    # x=1.5 as f64 big-endian
    assert encoded[8:16] == struct.pack(">d", 1.5)

    # y=2.5 as f64 big-endian
    assert encoded[16:24] == struct.pack(">d", 2.5)

    # z=3.5 as f64 big-endian
    assert encoded[24:32] == struct.pack(">d", 3.5)


def test_twist_known_binary_layout() -> None:
    """Python Twist encoding matches Rust binary layout byte-for-byte."""
    t = Twist(
        linear=Vector3(1.0, 2.0, 3.0),
        angular=Vector3(0.1, 0.2, 0.3),
    )
    encoded = t.lcm_encode()

    # 8-byte fingerprint + 6x8-byte f64 = 56 bytes
    assert len(encoded) == 56

    # Fingerprint
    assert encoded[:8] == struct.pack(">Q", TWIST_FINGERPRINT)

    # linear.x, linear.y, linear.z
    assert encoded[8:16] == struct.pack(">d", 1.0)
    assert encoded[16:24] == struct.pack(">d", 2.0)
    assert encoded[24:32] == struct.pack(">d", 3.0)

    # angular.x, angular.y, angular.z
    assert encoded[32:40] == struct.pack(">d", 0.1)
    assert encoded[40:48] == struct.pack(">d", 0.2)
    assert encoded[48:56] == struct.pack(">d", 0.3)


def test_vector3_roundtrip_cross_language() -> None:
    """Encode in Python, verify Rust would decode the same values."""
    v = Vector3(42.0, -17.5, 0.001)
    encoded = v.lcm_encode()
    decoded = Vector3.lcm_decode(encoded)
    assert decoded.x == v.x
    assert decoded.y == v.y
    assert decoded.z == v.z


def test_twist_roundtrip_cross_language() -> None:
    """Encode in Python, verify Rust would decode the same values."""
    t = Twist(linear=Vector3(1.5, 2.5, 3.5), angular=Vector3(0.1, 0.2, 0.3))
    encoded = t.lcm_encode()
    decoded = Twist.lcm_decode(encoded)
    assert decoded.linear == t.linear
    assert decoded.angular == t.angular
