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

"""XCDR1 alignment: primitives align to their own size, structs not at all."""

from __future__ import annotations

import struct

from dimos.robot.unitree.go2.dds import cdr


class Inner:
    __cdr_fields__ = [("mode", "u8"), ("q", "f32")]

    def __init__(self, mode: int, q: float) -> None:
        self.mode, self.q = mode, q


class Outer:
    __cdr_fields__ = [("tag", "u8"), ("inner", Inner)]

    def __init__(self, tag: int, inner: Inner) -> None:
        self.tag, self.inner = tag, inner


def test_a_nested_struct_starts_immediately_not_at_its_widest_members_boundary():
    """The bug this guards: pre-aligning `inner` to 4 read every field 4 bytes late.

    On the wire after `tag` (body offset 0) comes `mode` at offset 1 -- a u8
    needs no alignment even though the struct contains an f32 -- and `q` then
    pads to offset 4. That is the layout mcap_ros2 and every DDS stack produce
    for Go2 LowState/LowCmd; the old struct-level alignment shifted all motor
    fields by 4 bytes and swallowed the trailing crc.
    """
    body = bytes([7, 9, 0, 0]) + struct.pack("<f", 1.25)
    msg, end = cdr.decode(b"\x00\x01\x00\x00" + body, Outer)
    assert end == 4 + len(body)
    assert msg.tag == 7
    assert msg.inner.mode == 9
    assert msg.inner.q == 1.25


def test_struct_array_elements_pack_back_to_back():
    # Two Inner elements: [mode pad3 f32] then the next mode again unaligned.
    body = (
        bytes([1, 0, 0, 0]) + struct.pack("<f", 2.0) + bytes([2, 0, 0, 0]) + struct.pack("<f", 3.0)
    )

    class Arr:
        __cdr_fields__ = [("items", ("array", Inner, 2))]

        def __init__(self, items: list[Inner]) -> None:
            self.items = items

    msg, end = cdr.decode(b"\x00\x01\x00\x00" + body, Arr)
    assert end == 4 + len(body)
    assert [i.mode for i in msg.items] == [1, 2]
    assert [i.q for i in msg.items] == [2.0, 3.0]
