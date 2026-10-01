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

from dimos_generated.dimos_msgs.msg import GraspCandidate, GraspCandidateArray
from dimos_generated.geometry_msgs.msg import Point, Pose
from dimos_generated.std_msgs.msg import Header

from dimos.msgs.time import time_from_seconds


def test_grasp_candidate_round_trip_preserves_pose_and_score() -> None:
    candidate = GraspCandidate(pose=Pose(position=Point(x=0.4, y=-0.2, z=0.3)), score=0.75)

    decoded = GraspCandidate.decode(candidate.encode())

    assert decoded.pose.position.x == 0.4
    assert decoded.pose.position.y == -0.2
    assert decoded.pose.position.z == 0.3
    assert decoded.score == 0.75


def test_grasp_candidate_array_round_trip_preserves_header_and_order() -> None:
    candidates = [
        GraspCandidate(pose=Pose(position=Point(x=0.1, z=0.2)), score=0.9),
        GraspCandidate(pose=Pose(position=Point(x=0.2, z=0.2)), score=0.7),
    ]
    proposals = GraspCandidateArray(
        header=Header(stamp=time_from_seconds(123.0), frame_id="world"), candidates=candidates
    )

    decoded = GraspCandidateArray.decode(proposals.encode())

    assert decoded.header.stamp == time_from_seconds(123.0)
    assert decoded.header.frame_id == "world"
    assert [candidate.score for candidate in decoded.candidates] == [0.9, 0.7]
    assert [candidate.pose.position.x for candidate in decoded.candidates] == [0.1, 0.2]
