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

"""Malformed wire data is rejected before constructing generated Python values."""

import struct

from dimos_generated.geometry_msgs.msg import Polygon
from dimos_generated.shape_msgs.msg import SolidPrimitive
from dimos_generated.std_msgs.msg import (
    Bool,
    MultiArrayDimension,
    MultiArrayLayout,
    String,
    UInt32MultiArray,
)
from dimos_message_build.registry import decode, encode
import numpy as np
import pytest


@pytest.mark.parametrize("little", [True, False])
def test_every_truncation_of_nested_message_is_rejected(little):
    message = UInt32MultiArray(
        layout=MultiArrayLayout(
            dim=[MultiArrayDimension(label="café", size=2, stride=2)], data_offset=0
        ),
        data=np.array([0, 255], dtype=np.uint32),
    )
    encoded = encode(message, little_endian=little)
    for end in range(len(encoded)):
        with pytest.raises(ValueError):
            decode(encoded[:end], UInt32MultiArray)
    assert encode(decode(encoded, UInt32MultiArray), little_endian=little) == encoded


@pytest.mark.parametrize(
    "suffix",
    [
        pytest.param(
            b"\0",
            marks=pytest.mark.xfail(
                strict=True, reason="CDR-L01: docs/development/message-limitations.md#cdr-l01"
            ),
        ),
        pytest.param(
            b"\0\0",
            marks=pytest.mark.xfail(
                strict=True, reason="CDR-L01: docs/development/message-limitations.md#cdr-l01"
            ),
        ),
        pytest.param(
            b"\0\0\0",
            marks=pytest.mark.xfail(
                strict=True, reason="CDR-L01: docs/development/message-limitations.md#cdr-l01"
            ),
        ),
        b"\0\0\0\0",
    ],
)
def test_trailing_bytes_are_rejected(suffix):
    with pytest.raises(ValueError):
        decode(encode(Bool(data=True)) + suffix, Bool)


@pytest.mark.parametrize("little", [True, False])
@pytest.mark.parametrize(
    "length,payload",
    [
        pytest.param(
            0,
            b"",
            marks=pytest.mark.xfail(
                strict=True, reason="CDR-L02: docs/development/message-limitations.md#cdr-l02"
            ),
        ),
        pytest.param(
            2,
            b"ab",
            marks=pytest.mark.xfail(
                strict=True, reason="CDR-L03: docs/development/message-limitations.md#cdr-l03"
            ),
        ),
        (2, b"\xff\0"),
        pytest.param(
            0xFFFFFFFF,
            b"\0",
            marks=pytest.mark.xfail(
                strict=True, reason="CDR-L04: docs/development/message-limitations.md#cdr-l04"
            ),
        ),
    ],
)
def test_string_requires_positive_length_terminator_and_valid_utf8(little, length, payload):
    endian = "<" if little else ">"
    header = bytes([0, int(little), 0, 0])
    with pytest.raises(ValueError):
        decode(header + struct.pack(endian + "I", length) + payload, String)


@pytest.mark.parametrize("little", [True, False])
def test_standard_string_roundtrip(little):
    value = String(data="café")
    assert decode(encode(value, little_endian=little), String) == value


@pytest.mark.xfail(strict=True, reason="CDR-L05: docs/development/message-limitations.md#cdr-l05")
def test_invalid_encapsulation_is_rejected():
    with pytest.raises(ValueError):
        decode(b"\0\3\0\0\1", Bool)


@pytest.mark.xfail(strict=True, reason="CDR-L06: docs/development/message-limitations.md#cdr-l06")
def test_noncanonical_bool_is_rejected():
    with pytest.raises(ValueError):
        decode(b"\0\1\0\0\2", Bool)


def test_huge_sequence_is_rejected():
    message = UInt32MultiArray(
        layout=MultiArrayLayout(dim=[], data_offset=0), data=np.array([], dtype=np.uint32)
    )
    encoded = bytearray(encode(message))
    encoded[-4:] = struct.pack("<I", 0xFFFFFFFF)
    with pytest.raises(ValueError):
        decode(bytes(encoded), UInt32MultiArray)


@pytest.mark.xfail(strict=True, reason="CDR-L07: docs/development/message-limitations.md#cdr-l07")
def test_bounded_standard_type_supports_valid_values():
    # Required catalog gate: an unsupported-type error is not successful validation.
    value = SolidPrimitive(
        type=SolidPrimitive.BOX, dimensions=np.array([1.0, 2.0, 3.0]), polygon=Polygon(points=[])
    )
    restored = decode(encode(value), SolidPrimitive)
    np.testing.assert_array_equal(restored.dimensions, value.dimensions)


@pytest.mark.xfail(strict=True, reason="CDR-L08: docs/development/message-limitations.md#cdr-l08")
def test_bounded_standard_type_rejects_oversize_values():
    value = SolidPrimitive(
        type=SolidPrimitive.BOX, dimensions=np.ones(4), polygon=Polygon(points=[])
    )
    with pytest.raises(ValueError):
        encode(value)
