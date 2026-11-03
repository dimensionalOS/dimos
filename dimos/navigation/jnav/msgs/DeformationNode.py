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

"""DeformationNode: one pose-graph keyframe, published individually (not batched).

A loop-closure backend (e.g. gsc_pgo) emits one of these per keyframe when the node
is created and again whenever the optimizer moves it. A recording captures the stream
of them so a transform lookup can, at query time, find the keyframes near a time and
read their most-recent optimized world pose — the basis for loop-closure-corrected tf.

Fields:
  * ``id``     — a stable, random uint64 identifying the keyframe (reused across the
                 node's re-publishes; random rather than monotonic so it carries no
                 ordering/index coupling and won't collide across robots/sessions).
  * ``tf_id``  — uint64 = :func:`tf_id_for` (an FNV-1a-64 hash of
                 ``frame_from + "|" + frame_to``). Identifies which transform edge
                 these poses deform, so multi-robot systems with prefixed frames (and
                 several concurrent loop closures) can filter to the right one.
  * ``pose``   — the keyframe's world pose as a ``PoseStamped`` (carries its timestamp).
"""

from __future__ import annotations

import struct
from typing import BinaryIO

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.types.timestamped import Timestamped

_FNV_OFFSET_BASIS_64 = 0xCBF29CE484222325
_FNV_PRIME_64 = 0x100000001B3
_U64_MASK = 0xFFFFFFFFFFFFFFFF


def fnv1a_64(text: str) -> int:
    """64-bit FNV-1a hash of ``text`` (utf-8). Must match the C++ producer's hash so
    ``tf_id`` filtering agrees across the wire."""
    digest = _FNV_OFFSET_BASIS_64
    for byte in text.encode("utf-8"):
        digest = ((digest ^ byte) * _FNV_PRIME_64) & _U64_MASK
    return digest


def tf_id_for(frame_from: str, frame_to: str) -> int:
    """The ``tf_id`` for a transform edge: ``fnv1a_64(frame_from + "|" + frame_to)``."""
    return fnv1a_64(f"{frame_from}|{frame_to}")


class DeformationNode(Timestamped):
    msg_name = "nav_msgs.DeformationNode"

    id: int
    tf_id: int
    pose: PoseStamped

    def __init__(self, id: int = 0, tf_id: int = 0, pose: PoseStamped | None = None) -> None:
        self.id = id
        self.tf_id = tf_id
        self.pose = pose if pose is not None else PoseStamped()
        self.ts = self.pose.ts

    def __repr__(self) -> str:
        return f"DeformationNode(id={self.id}, tf_id={self.tf_id}, ts={self.pose.ts})"

    def lcm_encode(self) -> bytes:
        frame_id_bytes = self.pose.frame_id.encode("utf-8")
        return b"".join(
            (
                struct.pack(">QQd", self.id, self.tf_id, self.pose.ts),
                struct.pack(">I", len(frame_id_bytes)),
                frame_id_bytes,
                struct.pack(
                    ">7d",
                    self.pose.position.x,
                    self.pose.position.y,
                    self.pose.position.z,
                    self.pose.orientation.x,
                    self.pose.orientation.y,
                    self.pose.orientation.z,
                    self.pose.orientation.w,
                ),
            )
        )

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> DeformationNode:
        buf = data if isinstance(data, (bytes, bytearray)) else data.read()
        node_id, tf_id, pose_ts = struct.unpack_from(">QQd", buf, 0)
        offset = 24
        (frame_id_len,) = struct.unpack_from(">I", buf, offset)
        offset += 4
        frame_id = bytes(buf[offset : offset + frame_id_len]).decode("utf-8")
        offset += frame_id_len
        px, py, pz, qx, qy, qz, qw = struct.unpack_from(">7d", buf, offset)
        pose = PoseStamped(
            ts=pose_ts,
            frame_id=frame_id,
            position=[px, py, pz],
            orientation=[qx, qy, qz, qw],
        )
        return cls(id=node_id, tf_id=tf_id, pose=pose)
