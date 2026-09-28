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

"""Hand-written LCM wire helpers for message types that ``dimos_lcm`` does not ship.

Wire layout as in ``dimos/msgs/sensor_msgs/ImuInfo.py``: an 8-byte fingerprint, a Header,
then big-endian struct fields. Strings are LCM strings (int32 length including the NUL).
Each type's base hash is an arbitrary constant (as in ``JointCommand``), not lcm-gen's
struct hash: there is no ``.lcm`` schema, so generated bindings do not decode these types.
"""

from __future__ import annotations

from io import BytesIO
import struct

from dimos_lcm.std_msgs.Header import Header


def fingerprint(base_hash: int, cls: type) -> bytes:
    """Base hash mixed with Header's and rotated once, the way lcm-gen combines them."""
    tmphash = (base_hash + Header._get_hash_recursive([cls])) & 0xFFFFFFFFFFFFFFFF
    tmphash = (((tmphash << 1) & 0xFFFFFFFFFFFFFFFF) + (tmphash >> 63)) & 0xFFFFFFFFFFFFFFFF
    return struct.pack(">Q", tmphash)


def write_header(buf: BytesIO, ts: float, frame_id: str) -> None:
    header = Header()
    header.seq = 0
    header.frame_id = frame_id
    header.stamp.sec = int(ts)
    header.stamp.nsec = int((ts - int(ts)) * 1e9)
    header._encode_one(buf)


def read_header(buf: BytesIO) -> tuple[float, str]:
    header = Header._decode_one(buf)
    return header.stamp.sec + header.stamp.nsec / 1e9, str(header.frame_id)


def write_str(buf: BytesIO, s: str) -> None:
    raw = s.encode("utf-8")
    buf.write(struct.pack(">I", len(raw) + 1))
    buf.write(raw)
    buf.write(b"\0")


def read_str(buf: BytesIO) -> str:
    (n,) = struct.unpack(">I", buf.read(4))
    raw = buf.read(n)
    return raw[:-1].decode("utf-8")


def check_fingerprint(buf: BytesIO, expected: bytes, name: str) -> None:
    if buf.read(8) != expected:
        raise ValueError(f"{name}: fingerprint mismatch")
