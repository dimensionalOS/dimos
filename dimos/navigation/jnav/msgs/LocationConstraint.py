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

"""LocationConstraint: THE constraint message — factor injection AND map bridging.

The consolidation of the old ``MapConstraint`` (multi-map bridging) into the
GTSAM-shaped constraint (Jeff): one message serves both consumers.

- The PGO turns one into its own pose node (placed from interpolated odometry at
  ``ts``) and a ``BetweenFactor(node, location)`` whose noise model is
  ``covariance`` directly.
- MultiMap bridges maps through them: two maps observing the same ``to_id`` are
  bridgeable (the shared location IS the bridge anchor), with ``map_id`` naming
  which map's frame system each observation lives in.

Field meanings:
- ``to_id``: the location variable's identity — URL-like or a random UUID
  (e.g. ``apriltag://36h11/40cm/5``, ``reloc://map0/dim_city``, ``gps://fix``).
  The URL form encodes the exact source so identical tag numbers from different
  families/sizes can't false-bridge. Constraints sharing a ``to_id`` observe the
  same variable — which is what closes loops and bridges maps.
- ``map_id``: which map's frame system this observation is in (multi-map
  bridging). Single-graph consumers (the PGO) ignore it; ``""`` is fine there.
- ``frame_id``: the "from" — the OBSERVATION frame the ``pose`` is relative to
  (camera frame for a tag, odom frame for a reloc fix), NOT the map root, so the
  uncertainty stays honest. MultiMap resolves it to the map root via the map's
  recorded tf at ``ts``; the PGO currently enforces ``frame_id == body_frame``.
- ``pose``: the relative transform ``frame_id -> location``.
- ``covariance``: 6x6 measurement covariance in GTSAM Pose3 tangent order
  ``[rot(3), trans(3)]`` (row-major, 36 values). Degenerate DOFs (e.g. a
  position-only fix) get a huge variance on the rotation block. For consumers
  that want a coarse scalar, ``.confidence`` derives one (see below); producers
  with only a scalar use :meth:`from_confidence`.
- ``constraint_instance_id``: identifies this specific external instance. A
  later constraint reusing the same instance id REPLACES the earlier one
  (rolling revision — e.g. a perceiver refining a tag lock); a fresh id per
  event means additive observations (e.g. successive reloc fixes, which
  averaging consumers want to keep). This subsumes the old ``replacement``
  time-window mechanism.
- ``kind``: optional coarse category ("apriltag"/"reloc"/"ui_click"/...). When
  left empty it defaults to the ``to_id`` URL scheme (``reloc://...`` ->
  ``"reloc"``), so URL-style producers get it for free.

Confidence <-> covariance convention (shared by every producer/consumer that
thinks in ``[0, 1]`` scalars): per-axis variance ``= (1 - c) / c`` — c=1 is a
perfect measurement (variance 0), c→0 is worthless (variance → inf) — and the
derived scalar is ``1 / (1 + mean(diagonal))``, which round-trips a uniform c.
"""

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
    """The shared [0,1]-confidence -> variance convention (see module docstring)."""
    clamped = min(max(float(confidence), _MIN_CONFIDENCE), 1.0)
    return (1.0 - clamped) / clamped


def variance_to_confidence(variance: float) -> float:
    """Inverse of :func:`confidence_to_variance` (for the derived scalar)."""
    return 1.0 / (1.0 + max(float(variance), 0.0))


class LocationConstraint(Timestamped):
    msg_name = "jnav.LocationConstraint"

    ts: float
    to_id: str  # location variable id — URL-like or UUID; the bridge/loop key
    map_id: str  # whose frame system (multi-map); "" for single-graph consumers
    frame_id: str  # the "from": the observation frame the pose is relative to
    kind: str  # optional category; defaults to the to_id URL scheme
    constraint_instance_id: str  # same id -> replaces the earlier instance
    pose: Pose  # relative transform frame_id -> location
    covariance: list[float]  # 6x6 row-major, tangent order [rot(3), trans(3)]

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
        """Build with a diagonal covariance from [0,1] confidence(s).

        ``confidence`` is the coarse overall scalar; per-axis keywords
        (``roll``/``pitch``/``yaw``/``x``/``y``/``z``) override individual DOFs
        (e.g. ``z=0.01`` for a fix that barely constrains height).
        """
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
        """Coarse [0,1] scalar derived from the covariance diagonal (mean variance).

        Round-trips a uniform :meth:`from_confidence`; consumers with [0,1]
        thresholds (MultiMap's ``marker_min_confidence``) gate on this.
        """
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
        # map_id + kind ride at the TAIL: pre-merge payloads decode fine
        # (missing -> ""), and a fixed-sequence decoder that stops after the
        # covariance (the native gsc_pgo struct) tolerates the trailing bytes.
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
        for _ in range(2):  # map_id, kind — absent on pre-merge payloads
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
