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

"""Landmark: a cross-map reference shared between maps by ``id``.

Two maps that both observe the same ``id`` are bridgeable: MultiMap composes the
relative transform between their frames through the landmark. The pose is stored
relative to the frame it was observed in (a camera frame for an apriltag, an odom
frame for a relocalization event, the nearest cloud's frame for a UI click), NOT
the map root — so the uncertainty stays honest for a future gtsam graph. MultiMap
resolves it to the map root via the map's recorded tf at ``ts``.

``id`` is URL-like and encodes the exact source so identical tag numbers from
different families/sizes don't false-bridge unrelated maps, e.g.
``apriltag://36h11/40cm/5`` or ``reloc://map0/dim_city``.

``replacement`` (milliseconds) lets a perceiver publish a *corrective* landmark:
a consumer (e.g. the PGO factor-graph manager) wipes out any earlier landmark of
the same ``id`` observed within ``replacement`` ms before this one's ``ts`` and
keeps this one instead. ``0`` (the default) means "additive" — keep all prior
landmarks, which is what loop closure needs (the first and the revisit sighting
must both survive). A small positive window only supersedes the immediately-prior
noisy estimate of the *same* sighting, never the historical closure observation.

Confidence is **per-axis**, not a single scalar: ``confidence_x/_y/_z`` (the
observed translation axes) and ``confidence_roll/_pitch/_yaw`` (the rotation
axes), each in ``[0, 1]``. A consumer maps each per-axis confidence to that
axis's measurement weight (variance ~ 1 / confidence), so a landmark can be well
observed on some DOF and weak on others — e.g. a face-on AprilTag pins in-plane
position + yaw tightly but its range and out-of-plane tilt are loose. ``confidence``
(the scalar) is retained as a coarse overall value; the per-axis fields default
to it when unspecified.
"""

from __future__ import annotations

import struct
import time
from typing import BinaryIO

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.types.timestamped import Timestamped


class Landmark(Timestamped):
    msg_name = "jnav.Landmark"

    ts: float
    id: str  # URL-like, shared across maps — the bridge key
    map_id: str  # which map's frame system this is in
    frame_id: str  # the observation frame the pose is relative to
    kind: str  # "apriltag" | "aruco" | "reloc" | "ui_click" | "shortcut"
    confidence: float  # coarse overall confidence; per-axis fields default to it
    pose: Pose
    replacement: float  # ms: wipe same-id landmarks newer than (ts - replacement); 0 = additive
    # Per-axis confidence in [0, 1]: how much this landmark constrains each DOF.
    confidence_x: float
    confidence_y: float
    confidence_z: float
    confidence_roll: float
    confidence_pitch: float
    confidence_yaw: float

    def __init__(
        self,
        id: str = "",
        map_id: str = "",
        pose: Pose | None = None,
        frame_id: str = "",
        confidence: float = 0.0,
        kind: str = "",
        ts: float = 0.0,
        replacement: float = 0.0,
        confidence_x: float | None = None,
        confidence_y: float | None = None,
        confidence_z: float | None = None,
        confidence_roll: float | None = None,
        confidence_pitch: float | None = None,
        confidence_yaw: float | None = None,
    ) -> None:
        self.ts = ts if ts != 0 else time.time()
        self.id = id
        self.map_id = map_id
        self.frame_id = frame_id
        self.kind = kind
        self.confidence = confidence
        self.pose = pose if pose is not None else Pose()
        self.replacement = replacement
        # Per-axis confidence defaults to the coarse overall confidence.
        self.confidence_x = confidence if confidence_x is None else confidence_x
        self.confidence_y = confidence if confidence_y is None else confidence_y
        self.confidence_z = confidence if confidence_z is None else confidence_z
        self.confidence_roll = confidence if confidence_roll is None else confidence_roll
        self.confidence_pitch = confidence if confidence_pitch is None else confidence_pitch
        self.confidence_yaw = confidence if confidence_yaw is None else confidence_yaw

    def axis_confidence(self) -> tuple[float, float, float, float, float, float]:
        """Per-axis confidence as ``(x, y, z, roll, pitch, yaw)``."""
        return (
            self.confidence_x,
            self.confidence_y,
            self.confidence_z,
            self.confidence_roll,
            self.confidence_pitch,
            self.confidence_yaw,
        )

    def lcm_encode(self) -> bytes:
        parts: list[bytes] = [struct.pack(">d", self.ts)]
        for text in (self.id, self.map_id, self.frame_id, self.kind):
            encoded = text.encode("utf-8")
            parts.append(struct.pack(">I", len(encoded)))
            parts.append(encoded)
        parts.append(struct.pack(">d", self.confidence))
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
        parts.append(struct.pack(">d", self.replacement))
        parts.append(
            struct.pack(
                ">6d",
                self.confidence_x,
                self.confidence_y,
                self.confidence_z,
                self.confidence_roll,
                self.confidence_pitch,
                self.confidence_yaw,
            )
        )
        return b"".join(parts)

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> Landmark:
        buf = data if isinstance(data, (bytes, bytearray)) else data.read()
        offset = 0
        (ts,) = struct.unpack_from(">d", buf, offset)
        offset += 8
        texts: list[str] = []
        for _ in range(4):
            (length,) = struct.unpack_from(">I", buf, offset)
            offset += 4
            texts.append(buf[offset : offset + length].decode("utf-8"))
            offset += length
        id_, map_id, frame_id, kind = texts
        (confidence,) = struct.unpack_from(">d", buf, offset)
        offset += 8
        px, py, pz, qx, qy, qz, qw = struct.unpack_from(">7d", buf, offset)
        offset += 56
        pose = Pose()
        pose.position = Vector3(px, py, pz)
        pose.orientation = Quaternion(qx, qy, qz, qw)
        # Backward-compatible: older encodings omit the trailing replacement field.
        replacement = 0.0
        if offset + 8 <= len(buf):
            (replacement,) = struct.unpack_from(">d", buf, offset)
            offset += 8
        # Backward-compatible: older encodings omit the per-axis confidences;
        # they then default to the coarse overall confidence (None -> confidence).
        cx = cy = cz = croll = cpitch = cyaw = None
        if offset + 48 <= len(buf):
            cx, cy, cz, croll, cpitch, cyaw = struct.unpack_from(">6d", buf, offset)
        return cls(
            id=id_,
            map_id=map_id,
            pose=pose,
            frame_id=frame_id,
            confidence=confidence,
            kind=kind,
            ts=ts,
            replacement=replacement,
            confidence_x=cx,
            confidence_y=cy,
            confidence_z=cz,
            confidence_roll=croll,
            confidence_pitch=cpitch,
            confidence_yaw=cyaw,
        )
