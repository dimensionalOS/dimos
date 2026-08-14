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

from unittest.mock import MagicMock, patch

import numpy as np

from dimos.core.module import ModuleBase
from dimos.manipulation.pick_and_place import PickAndPlaceState
from dimos.manipulation.visualization.pick_and_place import PickAndPlaceVisualizationAdapter
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header


def _adapter() -> PickAndPlaceVisualizationAdapter:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        adapter = PickAndPlaceVisualizationAdapter()
    adapter._visualization = MagicMock()
    return adapter


def test_adapter_renders_detected_object_clouds() -> None:
    adapter = _adapter()
    obj = MagicMock()
    obj.pointcloud.points_f32.return_value = np.asarray([[0.1, 0.2, 0.3]])

    adapter._on_objects([obj])

    layer = adapter._visualization.set_visualization_layer.call_args.args[0]
    assert layer.id == "pick-and-place/objects"
    assert layer.elements[0].id == "object-0"


def test_adapter_renders_ranked_candidates_and_selected_targets() -> None:
    adapter = _adapter()
    pose = PoseStamped(
        ts=1.0,
        frame_id="world",
        position=Vector3(0.1, 0.2, 0.3),
        orientation=Quaternion(),
    )
    state = PickAndPlaceState(
        GraspCandidateArray(
            Header(1.0, "world"), [GraspCandidate(Pose(pose.position, pose.orientation), 0.9)]
        ),
        "cup-1",
        pose,
        pose,
    )

    adapter._on_state(state)

    candidate_layer, target_layer = [
        call.args[0] for call in adapter._visualization.set_visualization_layer.call_args_list
    ]
    assert candidate_layer.id == "pick-and-place/candidates"
    assert candidate_layer.elements[0].colors is not None
    np.testing.assert_array_equal(candidate_layer.elements[0].colors[0], [255, 220, 70])
    assert [element.id for element in target_layer.elements] == ["grasp", "pregrasp"]
