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

"""Roundtrip + field tests for the jnav LocationConstraint message."""

from __future__ import annotations

import struct

import pytest

from dimos.memory2.codecs.base import codec_for
from dimos.memory2.codecs.lcm import LcmCodec
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.navigation.jnav.msgs.LocationConstraint import LocationConstraint


def _pose(x: float, y: float, z: float) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    return pose


def _cov(scale: float = 1.0) -> list[float]:
    cov = [0.0] * 36
    for axis in range(6):
        cov[axis * 6 + axis] = scale * (axis + 1)
    return cov


def test_roundtrip_preserves_all_fields() -> None:
    cov = _cov(0.01)
    constraint = LocationConstraint(
        to_id="apriltag://36h11/40cm/5",
        frame_id="base_link",
        pose=_pose(1.5, -2.0, 0.3),
        covariance=cov,
        constraint_instance_id="tag5#42",
        map_id="hk_village",
        kind="apriltag",
        ts=1781565207.5,
    )
    decoded = LocationConstraint.lcm_decode(constraint.lcm_encode())

    assert decoded.to_id == "apriltag://36h11/40cm/5"
    assert decoded.frame_id == "base_link"
    assert decoded.constraint_instance_id == "tag5#42"
    assert decoded.map_id == "hk_village"
    assert decoded.kind == "apriltag"
    assert decoded.ts == constraint.ts
    assert decoded.pose.position.x == 1.5
    assert decoded.pose.position.y == -2.0
    assert decoded.pose.position.z == 0.3
    assert decoded.covariance == cov


def test_defaults() -> None:
    constraint = LocationConstraint()
    assert constraint.to_id == ""
    assert constraint.frame_id == ""
    assert constraint.constraint_instance_id == ""
    assert constraint.map_id == ""
    assert constraint.kind == ""
    assert constraint.ts > 0  # auto-stamped
    # Default covariance is a non-degenerate identity (unit variance per DOF).
    assert constraint.covariance[0] == 1.0 and constraint.covariance[35] == 1.0
    assert sum(constraint.covariance) == 6.0


def test_kind_defaults_to_to_id_scheme() -> None:
    assert LocationConstraint(to_id="reloc://map0/dim_city").kind == "reloc"
    assert LocationConstraint(to_id="apriltag://36h11/40cm/5").kind == "apriltag"
    # An explicit kind wins; a to_id without a URL scheme leaves kind empty.
    assert LocationConstraint(to_id="gps://fix", kind="override").kind == "override"
    assert LocationConstraint(to_id="bare-uuid").kind == ""


def test_pre_merge_payload_decodes_tail_as_empty() -> None:
    """A payload written before map_id/kind existed (stops after covariance)."""
    parts: list[bytes] = [struct.pack(">d", 123.0)]
    for text in ("to", "frame", "instance"):
        encoded = text.encode("utf-8")
        parts.append(struct.pack(">I", len(encoded)))
        parts.append(encoded)
    parts.append(struct.pack(">7d", 0, 0, 0, 0, 0, 0, 1))
    parts.append(struct.pack(">36d", *([0.0] * 36)))
    decoded = LocationConstraint.lcm_decode(b"".join(parts))
    assert decoded.to_id == "to"
    assert decoded.map_id == ""
    assert decoded.kind == ""


def test_full_6x6_covariance_roundtrips_offdiagonals() -> None:
    cov = [float(i) for i in range(36)]  # all entries distinct, incl. off-diagonal
    constraint = LocationConstraint(to_id="x", frame_id="base_link", covariance=cov)
    decoded = LocationConstraint.lcm_decode(constraint.lcm_encode())
    assert decoded.covariance == cov


def test_wrong_covariance_length_rejected() -> None:
    with pytest.raises(ValueError):
        LocationConstraint(to_id="x", covariance=[0.0] * 35)


def test_uses_lcm_codec_in_memory2() -> None:
    codec = codec_for(LocationConstraint)
    assert isinstance(codec, LcmCodec)
    constraint = LocationConstraint(
        to_id="gps://fix", frame_id="base_link", constraint_instance_id="gps#7"
    )
    decoded = codec.decode(codec.encode(constraint))
    assert decoded.to_id == "gps://fix"
    assert decoded.constraint_instance_id == "gps#7"
