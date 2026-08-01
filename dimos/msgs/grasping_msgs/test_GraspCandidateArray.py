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

from typing import cast

import pytest
import rerun as rr

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.grasping_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.grasping_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header


def _candidate(
    candidate_id: str,
    x: float,
    score: float,
    jaw_width: float,
) -> GraspCandidate:
    return GraspCandidate(
        pose=Pose(Vector3(x, x + 1.0, x + 2.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        jaw_width=jaw_width,
        score=score,
        id=candidate_id,
    )


def test_to_pose_array_preserves_candidate_order() -> None:
    candidates = [_candidate("first", 1.0, 0.4, 0.05), _candidate("second", 2.0, 0.9, 0.06)]
    msg = GraspCandidateArray(header=Header(12.5, "world"), candidates=candidates)

    pose_array = msg.to_pose_array()

    assert pose_array.header.frame_id == "world"
    assert [pose.position.x for pose in pose_array.poses] == [1.0, 2.0]
    assert pose_array.poses[0] is candidates[0].pose
    assert pose_array.poses[1] is candidates[1].pose


def test_grasp_candidate_array_lcm_round_trip_preserves_header_and_candidates() -> None:
    msg = GraspCandidateArray(
        header=Header(123.25, "map"),
        candidates=[_candidate("grasp-a", 0.1, 0.85, 0.04), _candidate("grasp-b", 0.2, 0.35, 0.07)],
    )

    decoded = GraspCandidateArray.lcm_decode(msg.lcm_encode())

    assert decoded.header.frame_id == "map"
    assert decoded.header.timestamp == pytest.approx(123.25)
    assert decoded.ts == pytest.approx(123.25)
    assert [candidate.id for candidate in decoded] == ["grasp-a", "grasp-b"]
    assert [candidate.score for candidate in decoded] == pytest.approx([0.85, 0.35])
    assert [candidate.jaw_width for candidate in decoded] == pytest.approx([0.04, 0.07])
    assert [
        (candidate.pose.position.x, candidate.pose.position.y, candidate.pose.position.z)
        for candidate in decoded
    ] == pytest.approx([(0.1, 1.1, 2.1), (0.2, 1.2, 2.2)])


def test_grasp_candidate_array_to_rerun_empty_array_is_safe() -> None:
    msg = GraspCandidateArray(header=Header(12.5, "world"), candidates=[])

    strips = msg.to_rerun()

    assert isinstance(strips, rr.LineStrips3D)
    strips = cast("rr.LineStrips3D", strips)
    assert strips.strips.as_arrow_array().to_pylist() == []


def test_grasp_candidate_array_to_rerun_non_empty_exposes_multiple_strips() -> None:
    msg = GraspCandidateArray(
        header=Header(12.5, "world"),
        candidates=[_candidate("high", 0.0, 0.9, 0.08), _candidate("low", 1.0, 0.2, 0.06)],
    )

    strips = msg.to_rerun()

    assert isinstance(strips, rr.LineStrips3D)
    strips = cast("rr.LineStrips3D", strips)
    strip_values = strips.strips.as_arrow_array().to_pylist()
    assert len(strip_values) == 10
    assert len(strip_values[0]) == 2
    assert len(strips.colors.as_arrow_array().to_pylist()) == 10
