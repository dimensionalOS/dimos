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

from unittest.mock import MagicMock

import numpy as np
import pytest

from dimos.manipulation.pick_and_place import PickAndPlaceState
from dimos.manipulation.visualization.pick_and_place_rerun import (
    _objects_to_rerun,
    _state_to_rerun,
    _topic_to_entity,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header

rr = pytest.importorskip("rerun", reason="Rerun optional dependency is not installed")


def test_rerun_adapters_map_workflow_streams_and_render_objects() -> None:
    assert _topic_to_entity("/objects") == "world/pick-and-place/objects"
    assert _topic_to_entity("/pick_and_place_state") == "world/pick-and-place/selection"
    obj = MagicMock()
    obj.pointcloud.points_f32.return_value = np.asarray([[0.1, 0.2, 0.3]])

    data = _objects_to_rerun([obj])

    assert [path for path, _ in data] == [
        "world/pick-and-place/objects",
        "world/pick-and-place/objects/object-0",
    ]
    assert isinstance(data[1][1], rr.Points3D)


def test_rerun_state_adapter_renders_candidates_and_targets() -> None:
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

    data = _state_to_rerun(state)

    assert [path for path, _ in data] == [
        "world/pick-and-place/selection",
        "world/pick-and-place/selection/candidates/rank-0",
        "world/pick-and-place/selection/targets/grasp",
        "world/pick-and-place/selection/targets/pregrasp",
    ]
    assert isinstance(data[1][1], rr.Arrows3D)
