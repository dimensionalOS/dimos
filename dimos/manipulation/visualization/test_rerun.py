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

from dimos.manipulation.visualization.rerun import (
    _graspgenx_candidates_to_rerun,
    _topic_to_entity,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header


def test_qualified_grasp_candidate_topic_uses_candidate_entity() -> None:
    assert (
        _topic_to_entity("dimos/PickNPlaceModule/graspgenx_candidates")
        == "world/graspgenx_candidates"
    )


def test_top_grasp_candidate_has_selected_rerun_marker() -> None:
    candidates = GraspCandidateArray(
        Header(0.0, "link_base"),
        [
            GraspCandidate(Pose(Vector3(0.1, 0.2, 0.3)), 0.9),
            GraspCandidate(Pose(Vector3(0.2, 0.3, 0.4)), 0.8),
        ],
        selected_index=1,
    )

    paths = [path for path, _ in _graspgenx_candidates_to_rerun(candidates)]

    assert "world/graspgenx_candidates/00/selected" not in paths
    assert "world/graspgenx_candidates/01/selected" in paths
    assert "world/graspgenx_candidates/01/gripper_base" in paths
    assert "world/graspgenx_candidates/01/gripper_base/jaws" in paths
