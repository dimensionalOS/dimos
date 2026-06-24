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

"""Marker: a named navigation marker — a pose in a frame, a marker name, and an
optional map name.

Drives the objective handler's ``goal_marker`` (navigate to a named marker) and
``save_marker`` (persist the current/given pose under a name) streams, so callers
can reference waypoints by name (e.g. dim_city's ``test_waypoint_*``) and scope
them to a map, instead of passing a bare Point/Pose with no identity.
"""

from __future__ import annotations

import struct
import time
from typing import BinaryIO

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.types.timestamped import Timestamped


class Marker(Timestamped):
    msg_name = "jnav.Marker"

    ts: float
    frame_id: str
    pose: Pose
    marker: str  # marker name (identity)
    map: str  # optional map name this marker belongs to ("" = unspecified)

    def __init__(
        self,
        pose: Pose | None = None,
        marker: str = "",
        map: str = "",
        ts: float = 0.0,
        frame_id: str = "map",
    ) -> None:
        self.ts = ts if ts != 0 else time.time()
        self.frame_id = frame_id
        self.pose = pose if pose is not None else Pose()
        self.marker = marker
        self.map = map

    def lcm_encode(self) -> bytes:
        parts: list[bytes] = [struct.pack(">d", self.ts)]
        for text in (self.frame_id, self.marker, self.map):
            encoded = text.encode("utf-8")
            parts.append(struct.pack(">I", len(encoded)))
            parts.append(encoded)
        p = self.pose
        parts.append(
            struct.pack(
                ">7d",
                p.position.x,
                p.position.y,
                p.position.z,
                p.orientation.x,
                p.orientation.y,
                p.orientation.z,
                p.orientation.w,
            )
        )
        return b"".join(parts)

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> Marker:
        buf = data if isinstance(data, (bytes, bytearray)) else data.read()
        offset = 0
        (ts,) = struct.unpack_from(">d", buf, offset)
        offset += 8
        texts: list[str] = []
        for _ in range(3):
            (length,) = struct.unpack_from(">I", buf, offset)
            offset += 4
            texts.append(buf[offset : offset + length].decode("utf-8"))
            offset += length
        frame_id, marker, map_name = texts
        px, py, pz, qx, qy, qz, qw = struct.unpack_from(">7d", buf, offset)
        pose = Pose()
        pose.position = Vector3(px, py, pz)
        pose.orientation = Quaternion(qx, qy, qz, qw)
        return cls(pose=pose, marker=marker, map=map_name, ts=ts, frame_id=frame_id)
