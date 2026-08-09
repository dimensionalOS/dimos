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

import math
from unittest.mock import MagicMock, patch

from dimos.core.module import ModuleBase
from dimos.manipulation.pick_and_place import PickAndPlaceModule, PickAndPlaceModuleConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header


def _module() -> PickAndPlaceModule:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        module = PickAndPlaceModule()
    module.config = PickAndPlaceModuleConfig()
    module._grasp_filter = MagicMock(
        inverse_kinematics_single=MagicMock(return_value=MagicMock(is_success=lambda: True))
    )
    return module


def test_select_grasp_uses_object_id_and_retains_ranked_candidates() -> None:
    module = _module()
    obj = MagicMock(object_id="cup-1", pointcloud=MagicMock())
    module._on_objects([obj])
    candidate = GraspCandidate(
        Pose(Vector3(0.1, 0.2, 0.3), Quaternion.from_euler(Vector3(-math.pi, 0.0, 0.0))),
        score=0.9,
    )
    module._heuristic_grasp_generator = MagicMock(
        propose_grasps=MagicMock(
            return_value=GraspCandidateArray(Header(1.0, "world"), [candidate])
        )
    )

    selected = module.select_grasp("cup-1")

    assert selected is not None
    assert selected.position == candidate.pose.position
    assert module.get_object("cup-1") is obj
    assert module.get_grasp_candidates().selected_index == 0
    assert module._selected_pregrasp is not None
    assert module._selected_pregrasp.position.z == 0.4


def test_place_at_uses_selected_grasp_orientation() -> None:
    module = _module()
    candidate = GraspCandidate(Pose(Vector3(), Quaternion()), score=1.0)
    module._selected_grasp = MagicMock(orientation=candidate.pose.orientation)
    module._holding_object = True
    module._pick_execution = MagicMock()
    module._pick_execution.move_to_pose.return_value = MagicMock(is_success=lambda: True)
    module._pick_execution.open_gripper.return_value = MagicMock(is_success=lambda: True)

    result = module.place_at(0.3, 0.2, 0.1)

    assert result.is_success()
    assert module._pick_execution.move_to_pose.call_count == 3


def test_candidate_policy_can_skip_filtering_and_rank_by_ik_feasibility() -> None:
    module = _module()
    module.config = PickAndPlaceModuleConfig(
        candidate_filter="off", candidate_ranking="ik_feasibility"
    )
    unsafe = GraspCandidate(Pose(Vector3(0.1, 0.2, 0.3), Quaternion()), score=0.9)
    safe = GraspCandidate(Pose(Vector3(0.2, 0.3, 0.4), Quaternion()), score=0.2)
    module._grasp_filter.inverse_kinematics_single.side_effect = [
        MagicMock(is_success=lambda: False),
        MagicMock(is_success=lambda: True),
    ]

    candidates = module._filter_candidates(
        GraspCandidateArray(Header(1.0, "world"), [unsafe, safe])
    )

    assert candidates.candidates == [safe, unsafe]
