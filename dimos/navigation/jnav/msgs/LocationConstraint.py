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

"""LocationConstraint: the ready-made args for a GTSAM ``BetweenFactor``.

A LocationConstraint is a relative-pose measurement from a ``from`` graph node to
a ``to`` location variable, plus the 6x6 covariance that becomes the factor's
noise model directly. The PGO turns each one into its own pose node (placed from
interpolated odometry at ``ts``) and a ``BetweenFactor(node, location)``.

Field meanings (mapping onto BetweenFactor):
- ``to_id`` is the BetweenFactor "to": the location variable's identity, URL-like
  (e.g. ``apriltag://36h11/40cm/5`` or ``gps://fix`` or ``ui_click://...``). Two
  constraints sharing a ``to_id`` observe the same graph variable, which closes
  the loop.
- ``frame_id`` is the BetweenFactor "from": the frame the ``pose`` is expressed
  in. For now the PGO enforces ``frame_id == body_frame`` (no full C++ tf yet),
  and re-bases the measurement onto the node it creates.
- ``pose`` is the relative transform ``frame_id -> location``.
- ``covariance`` is the 6x6 measurement covariance in GTSAM Pose3 tangent order
  ``[rot(3), trans(3)]`` (row-major, 36 values). It is used as the factor's noise
  model directly — degenerate DOFs (e.g. a position-only fix) get a huge variance
  on the rotation block.
- ``constraint_instance_id`` identifies this specific external instance. A later
  constraint reusing the same ``constraint_instance_id`` removes the committed
  factors carrying it, letting an external estimator do rolling outlier removal /
  revision (e.g. as a tag/GPS lock improves).
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


def _identity_covariance() -> list[float]:
    """A neutral, non-degenerate default: unit variance on every DOF."""
    cov = [0.0] * _COVARIANCE_LENGTH
    for axis in range(6):
        cov[axis * 6 + axis] = 1.0
    return cov


class LocationConstraint(Timestamped):
    msg_name = "jnav.LocationConstraint"

    ts: float
    to_id: str  # the BetweenFactor "to" (location variable id), URL-like
    frame_id: str  # the BetweenFactor "from" frame (== body frame for now)
    constraint_instance_id: str  # external instance id, for revision/removal
    pose: Pose  # relative transform frame_id -> location
    covariance: list[float]  # 6x6 row-major, tangent order [rot(3), trans(3)]

    def __init__(
        self,
        to_id: str = "",
        frame_id: str = "",
        pose: Pose | None = None,
        covariance: list[float] | None = None,
        constraint_instance_id: str = "",
        ts: float = 0.0,
    ) -> None:
        self.ts = ts if ts != 0 else time.time()
        self.to_id = to_id
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
        return cls(
            to_id=to_id,
            frame_id=frame_id,
            pose=pose,
            covariance=covariance,
            constraint_instance_id=constraint_instance_id,
            ts=ts,
        )
