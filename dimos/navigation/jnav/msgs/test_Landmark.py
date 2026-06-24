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

"""Roundtrip + field tests for the jnav Landmark message."""

from __future__ import annotations

from dimos.memory2.codecs.base import codec_for
from dimos.memory2.codecs.lcm import LcmCodec
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.navigation.jnav.msgs.Landmark import Landmark


def _pose(x: float, y: float, z: float) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    return pose


def test_landmark_roundtrip_preserves_all_fields() -> None:
    landmark = Landmark(
        id="apriltag://36h11/40cm/5",
        map_id="dim_city",
        pose=_pose(1.5, -2.0, 0.3),
        frame_id="camera_optical",
        confidence=0.82,
        kind="apriltag",
        ts=1781565207.5,
    )
    decoded = Landmark.lcm_decode(landmark.lcm_encode())

    assert decoded.id == "apriltag://36h11/40cm/5"
    assert decoded.map_id == "dim_city"
    assert decoded.frame_id == "camera_optical"
    assert decoded.kind == "apriltag"
    assert decoded.confidence == 0.82
    assert decoded.ts == landmark.ts
    assert decoded.pose.position.x == 1.5
    assert decoded.pose.position.y == -2.0
    assert decoded.pose.position.z == 0.3


def test_landmark_defaults() -> None:
    landmark = Landmark()
    assert landmark.id == ""
    assert landmark.map_id == ""
    assert landmark.frame_id == ""
    assert landmark.kind == ""
    assert landmark.confidence == 0.0
    assert landmark.replacement == 0.0  # additive by default
    assert landmark.ts > 0  # auto-stamped

    decoded = Landmark.lcm_decode(landmark.lcm_encode())
    assert decoded.id == "" and decoded.map_id == "" and decoded.kind == ""
    assert decoded.replacement == 0.0


def test_landmark_replacement_roundtrips() -> None:
    landmark = Landmark(
        id="apriltag://36h11/40cm/13",
        pose=_pose(0.1, 0.2, 0.5),
        kind="apriltag",
        ts=1781565207.5,
        replacement=2500.0,  # ms: corrective landmark supersedes recent same-id ones
    )
    decoded = Landmark.lcm_decode(landmark.lcm_encode())
    assert decoded.replacement == 2500.0


def test_landmark_decode_tolerates_legacy_bytes_without_replacement() -> None:
    """Bytes written before the replacement + per-axis fields decode to defaults."""
    landmark = Landmark(id="x", pose=_pose(0.0, 0.0, 0.0), confidence=0.7, replacement=999.0)
    # Strip the trailing replacement double + the 6 per-axis confidence doubles.
    legacy = landmark.lcm_encode()[: -(8 + 48)]
    decoded = Landmark.lcm_decode(legacy)
    assert decoded.id == "x"
    assert decoded.replacement == 0.0
    # Per-axis confidences fall back to the coarse overall confidence.
    assert decoded.axis_confidence() == (0.7, 0.7, 0.7, 0.7, 0.7, 0.7)


def test_per_axis_confidence_defaults_to_overall() -> None:
    lm = Landmark(id="t", confidence=0.6)
    assert lm.axis_confidence() == (0.6, 0.6, 0.6, 0.6, 0.6, 0.6)


def test_per_axis_confidence_roundtrips() -> None:
    lm = Landmark(
        id="apriltag://36h11/40cm/13",
        pose=_pose(0.1, 0.2, 0.5),
        kind="apriltag",
        confidence=0.5,
        confidence_x=0.9,
        confidence_y=0.8,
        confidence_z=0.7,
        confidence_roll=0.2,
        confidence_pitch=0.1,
        confidence_yaw=0.95,
    )
    d = Landmark.lcm_decode(lm.lcm_encode())
    assert d.axis_confidence() == (0.9, 0.8, 0.7, 0.2, 0.1, 0.95)
    assert d.confidence == 0.5  # coarse overall preserved independently


def test_per_axis_confidence_can_be_silent_on_some_dof() -> None:
    """A landmark can be confident on some DOF and zero on others."""
    lm = Landmark(id="t", pose=_pose(0.0, 0.0, 0.0), confidence_z=0.9, confidence_x=0.0)
    d = Landmark.lcm_decode(lm.lcm_encode())
    assert d.confidence_z == 0.9 and d.confidence_x == 0.0


def test_landmark_uses_lcm_codec_in_memory2() -> None:
    codec = codec_for(Landmark)
    assert isinstance(codec, LcmCodec)
    landmark = Landmark(id="reloc://map0/dim_city", map_id="map0", confidence=0.6, kind="reloc")
    decoded = codec.decode(codec.encode(landmark))
    assert decoded.id == "reloc://map0/dim_city"
    assert decoded.confidence == 0.6
