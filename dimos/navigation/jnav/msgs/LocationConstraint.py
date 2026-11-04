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

"""LocationConstraint: a measured pose of location `to_id` from `frame_id`, a pose graph factor (PGO) and a map bridge (MultiMap)."""

from __future__ import annotations

import struct
import time
from typing import BinaryIO

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.types.timestamped import Timestamped

# 6x6 covariance, row-major.
_COVARIANCE_LENGTH = 36
# Diagonal tangent order [rot(3), trans(3)]: per-axis confidence names -> index.
_AXIS_ORDER = ("roll", "pitch", "yaw", "x", "y", "z")
_MIN_CONFIDENCE = 1e-6


def _identity_covariance() -> list[float]:
    """A neutral, non-degenerate default: unit variance on every DOF."""
    cov = [0.0] * _COVARIANCE_LENGTH
    for axis in range(6):
        cov[axis * 6 + axis] = 1.0
    return cov


def confidence_to_variance(confidence: float) -> float:
    """The shared [0,1]-confidence -> variance convention: (1 - c) / c, so c = 1 is exact and c -> 0 worthless."""
    clamped = min(max(float(confidence), _MIN_CONFIDENCE), 1.0)
    return (1.0 - clamped) / clamped


def variance_to_confidence(variance: float) -> float:
    """Inverse of :func:`confidence_to_variance`: 1 / (1 + variance)."""
    return 1.0 / (1.0 + max(float(variance), 0.0))


class LocationConstraint(Timestamped):
    msg_name = "jnav.LocationConstraint"

    ts: float
    to_id: (
        str  # location variable id, URL-like (apriltag://36h11/40cm/5) or UUID; the bridge/loop key
    )
    map_id: str  # whose frame system (multi-map); "" for single-graph consumers
    frame_id: str  # the "from": the observation frame the pose is relative to (camera for a tag), not the map root
    kind: str  # optional category; defaults to the to_id URL scheme
    constraint_instance_id: (
        str  # same id -> replaces the earlier instance; a fresh id per event adds
    )
    pose: Pose  # relative transform frame_id -> location
    covariance: list[
        float
    ]  # 6x6 row-major, tangent order [rot(3), trans(3)]; huge variance on unobserved DOFs

    def __init__(
        self,
        to_id: str = "",
        frame_id: str = "",
        pose: Pose | None = None,
        covariance: list[float] | None = None,
        constraint_instance_id: str = "",
        map_id: str = "",
        kind: str = "",
        ts: float = 0.0,
    ) -> None:
        self.ts = ts if ts != 0 else time.time()
        self.to_id = to_id
        self.map_id = map_id
        scheme, separator, _ = to_id.partition("://")
        self.kind = kind or (scheme if separator else "")
        self.frame_id = frame_id
        self.constraint_instance_id = constraint_instance_id
        self.pose = pose if pose is not None else Pose()
        if covariance is None:
            self.covariance = _identity_covariance()
        else:
            if len(covariance) != _COVARIANCE_LENGTH:
                raise ValueError(
                    f"covariance must be {_COVARIANCE_LENGTH} values (6x6 row-major), "
                    f"got {len(covariance)}"
                )
            self.covariance = list(covariance)

    @classmethod
    def from_confidence(
        cls,
        to_id: str = "",
        frame_id: str = "",
        pose: Pose | None = None,
        confidence: float = 1.0,
        map_id: str = "",
        kind: str = "",
        constraint_instance_id: str = "",
        ts: float = 0.0,
        **per_axis: float,
    ) -> LocationConstraint:
        """Build with a diagonal covariance from [0,1] confidence; per-axis keywords (roll, pitch, yaw, x, y, z) override one DOF."""
        unknown = set(per_axis) - set(_AXIS_ORDER)
        if unknown:
            raise ValueError(f"unknown per-axis confidence(s): {sorted(unknown)}")
        cov = [0.0] * _COVARIANCE_LENGTH
        for index, axis in enumerate(_AXIS_ORDER):
            cov[index * 6 + index] = confidence_to_variance(per_axis.get(axis, confidence))
        return cls(
            to_id=to_id,
            frame_id=frame_id,
            pose=pose,
            covariance=cov,
            constraint_instance_id=constraint_instance_id,
            map_id=map_id,
            kind=kind,
            ts=ts,
        )

    @property
    def confidence(self) -> float:
        """Coarse [0,1] scalar from the covariance diagonal's mean variance; round-trips a uniform :meth:`from_confidence`."""
        diagonal = [self.covariance[axis * 6 + axis] for axis in range(6)]
        return variance_to_confidence(sum(diagonal) / len(diagonal))

    def lcm_encode(self) -> bytes:
        parts: list[bytes] = [struct.pack(">d", self.ts)]
        for text in (self.to_id, self.frame_id, self.constraint_instance_id):
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
        parts.append(struct.pack(">36d", *self.covariance))
        # map_id + kind ride at the TAIL: pre-merge payloads decode fine (missing -> ""), and a fixed-sequence decoder that stops after the covariance (the native gsc_pgo struct) tolerates the trailing bytes.
        for text in (self.map_id, self.kind):
            encoded = text.encode("utf-8")
            parts.append(struct.pack(">I", len(encoded)))
            parts.append(encoded)
        return b"".join(parts)

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> LocationConstraint:
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
        to_id, frame_id, constraint_instance_id = texts
        px, py, pz, qx, qy, qz, qw = struct.unpack_from(">7d", buf, offset)
        offset += 56
        pose = Pose()
        pose.position = Vector3(px, py, pz)
        pose.orientation = Quaternion(qx, qy, qz, qw)
        covariance = list(struct.unpack_from(">36d", buf, offset))
        offset += _COVARIANCE_LENGTH * 8
        tail: list[str] = []
        for _ in range(2):  # map_id, kind: absent on pre-merge payloads
            if offset + 4 > len(buf):
                tail.append("")
                continue
            (length,) = struct.unpack_from(">I", buf, offset)
            offset += 4
            tail.append(buf[offset : offset + length].decode("utf-8"))
            offset += length
        map_id, kind = tail
        return cls(
            to_id=to_id,
            frame_id=frame_id,
            pose=pose,
            covariance=covariance,
            constraint_instance_id=constraint_instance_id,
            map_id=map_id,
            kind=kind,
            ts=ts,
        )
