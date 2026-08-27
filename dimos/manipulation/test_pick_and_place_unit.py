# Copyright 2025-2026 Dimensional Inc.
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

from collections.abc import Iterator
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header


@pytest.fixture
def module() -> Iterator[PickAndPlaceModule]:
    instance = PickAndPlaceModule()
    instance._scene = MagicMock()
    instance._grasp_generator = MagicMock()
    instance._manipulation = MagicMock()
    instance._manipulation.list_planning_groups.return_value = [
        SimpleNamespace(id="arm/tool", has_gripper=True)
    ]
    instance._manipulation.get_state.return_value = SimpleNamespace(
        groups={
            "arm/tool": SimpleNamespace(
                gripper_position=0.5,
                end_effector_pose=PoseStamped(
                    frame_id="world", orientation=Quaternion.from_euler(Vector3(0, 0, 0.7))
                ),
            )
        }
    )
    instance._manipulation.plan_to_poses.return_value = SimpleNamespace(succeeded=True, message="")
    instance._manipulation.execute.return_value = SimpleNamespace(succeeded=True, message="")
    instance._manipulation.set_gripper_position.return_value = SimpleNamespace(
        succeeded=True, message=""
    )
    yield instance
    instance.stop()


def _candidate(x: float, score: float = 1.0) -> GraspCandidate:
    return GraspCandidate(
        Pose(Vector3(x, 0.0, 0.2), Quaternion.from_euler(Vector3(-3.141592653589793, 0.0, 0.0))),
        score,
    )


def test_scan_objects_uses_stable_ids(module: PickAndPlaceModule) -> None:
    scene: Any = module._scene
    scene.scan_scene.return_value = SimpleNamespace(detections_length=1)
    scene.get_detected_objects.return_value = [
        {"object_id": "cup-1", "name": "cup", "confidence": 0.9}
    ]

    result = module.scan_objects([" cup "])

    assert result.is_success()
    assert module.get_object("cup-1") == {"object_id": "cup-1", "name": "cup", "confidence": 0.9}
    scene.set_prompts.assert_called_once_with(["cup"])


def test_select_grasp_preserves_provider_order_and_offsets_locally(
    module: PickAndPlaceModule,
) -> None:
    scene: Any = module._scene
    grasp_generator: Any = module._grasp_generator
    module._objects = {"cup-1": {"object_id": "cup-1"}}
    scene.get_object_pointcloud_by_object_id.return_value = MagicMock()
    first, second = _candidate(0.1, 0.1), _candidate(0.2, 0.9)
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [first, second]
    )

    result = module.select_grasp("cup-1", rank=1)

    assert result.is_success()
    assert module.get_grasp_candidates().candidates == [first, second]
    assert module._selected_grasp is not None
    assert module._selected_grasp.position.x == pytest.approx(0.2)
    assert module._selected_pregrasp is not None
    assert module._selected_pregrasp.position.z == pytest.approx(0.3)


def test_select_grasp_rejects_non_planning_frame(module: PickAndPlaceModule) -> None:
    scene: Any = module._scene
    grasp_generator: Any = module._grasp_generator
    module._objects = {"cup-1": {"object_id": "cup-1"}}
    scene.get_object_pointcloud_by_object_id.return_value = MagicMock()
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "camera"), [_candidate(0.1)]
    )

    result = module.select_grasp("cup-1")

    assert not result.is_success()
    assert result.error_code == "GRASP_FRAME_MISMATCH"


def test_pick_preserves_current_yaw_when_configured(module: PickAndPlaceModule) -> None:
    module.config.yaw_policy = "preserve_current"
    module._selected_grasp = PoseStamped(
        frame_id="world",
        position=Vector3(0.1, 0.0, 0.2),
        orientation=Quaternion.from_euler(Vector3(-3.141592653589793, 0.0, 0.1)),
    )

    result = module.pick_selected()

    assert result.is_success()
    assert module._selected_grasp is not None
    assert module._selected_grasp.orientation.to_euler().z == pytest.approx(0.7)
    assert module._holding_object


def test_place_uses_local_axis_and_clears_held_state(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    module._selected_grasp = PoseStamped(
        frame_id="world",
        orientation=Quaternion.from_euler(Vector3(-3.141592653589793, 0.0, 0.0)),
    )
    module._holding_object = True

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.is_success()
    preplace = manipulation.plan_to_poses.call_args_list[0].args[0]["arm/tool"]
    assert preplace.position.z == pytest.approx(0.3)
    assert not module._holding_object
