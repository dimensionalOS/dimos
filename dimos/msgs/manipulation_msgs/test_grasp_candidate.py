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

import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header


def test_grasp_candidate_round_trip_preserves_pose_and_score() -> None:
    candidate = GraspCandidate(Pose(0.4, -0.2, 0.3), 0.75)

    decoded = GraspCandidate.decode(candidate.encode())

    assert decoded.pose.position.x == 0.4
    assert decoded.pose.position.y == -0.2
    assert decoded.pose.position.z == 0.3
    assert decoded.score == 0.75


def test_grasp_candidate_rejects_non_finite_score() -> None:
    with pytest.raises(ValueError, match="score must be finite"):
        GraspCandidate(score=float("nan"))


def test_grasp_candidate_array_round_trip_preserves_header_and_order() -> None:
    candidates = [
        GraspCandidate(Pose(0.1, 0.0, 0.2), 0.9),
        GraspCandidate(Pose(0.2, 0.0, 0.2), 0.7),
    ]
    proposals = GraspCandidateArray(Header(123.0, "world"), candidates)

    decoded = GraspCandidateArray.decode(proposals.encode())

    assert decoded.header.timestamp == 123.0
    assert decoded.header.frame_id == "world"
    assert [candidate.score for candidate in decoded] == [0.9, 0.7]
    assert [candidate.pose.position.x for candidate in decoded] == [0.1, 0.2]
