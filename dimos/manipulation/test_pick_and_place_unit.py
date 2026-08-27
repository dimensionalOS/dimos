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

from dimos.manipulation.grasp_verification import GripperSettle
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
        SimpleNamespace(id="arm/tool", has_gripper=True, tip_frame="tool")
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


@pytest.fixture(autouse=True)
def settled_gripper(monkeypatch: pytest.MonkeyPatch) -> None:
    def settle(read: Any, target: float, config: Any) -> GripperSettle:
        position = 0.5 if target == config.closed_position else target
        return GripperSettle(True, position, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)


def _candidate(x: float, score: float = 1.0) -> GraspCandidate:
    return GraspCandidate(
        Pose(Vector3(x, 0.0, 0.2), Quaternion.from_euler(Vector3(-3.141592653589793, 0.0, 0.0))),
        score,
    )


def test_scan_objects_uses_stable_ids(module: PickAndPlaceModule) -> None:
    scene: Any = module._scene
    scene.scan_scene.return_value = SimpleNamespace(
        detections_length=1,
        detections=[
            SimpleNamespace(
                id="cup-1", results=[SimpleNamespace(hypothesis=SimpleNamespace(class_id="cup"))]
            )
        ],
    )

    result = module.scan_objects([" cup "])

    assert result.is_success()
    assert module.get_object("cup-1") == {"object_id": "cup-1", "name": "cup"}
    scene.scan_scene.assert_called_once_with(text=["cup"])


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
    assert module._selected_grasp is None


def test_scan_failure_clears_stale_selection(module: PickAndPlaceModule) -> None:
    scene: Any = module._scene
    module._selected_grasp = PoseStamped(frame_id="world")
    scene.scan_scene.side_effect = RuntimeError("No aligned RGB-D frame")

    result = module.scan_objects(["cup"])

    assert not result.is_success()
    assert result.error_code == "PERCEPTION_FAILED"
    assert module._selected_grasp is None


def test_select_and_pick_reject_when_already_holding(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    module._holding_object = True
    module._selected_grasp = PoseStamped(frame_id="world")
    module._objects = {"cup-1": {"object_id": "cup-1"}}

    selection = module.select_grasp("cup-1")
    pick = module.pick_selected()

    assert selection.error_code == "INVALID_STATE"
    assert pick.error_code == "INVALID_STATE"
    manipulation.set_gripper_position.assert_not_called()


def test_pick_retains_held_state_when_retract_fails(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    module._selected_grasp = PoseStamped(frame_id="world")
    manipulation.execute.side_effect = [
        SimpleNamespace(succeeded=True, message=""),
        SimpleNamespace(succeeded=True, message=""),
        SimpleNamespace(succeeded=False, message="retract failed"),
    ]

    result = module.pick_selected()

    assert result.error_code == "EXECUTION_FAILED"
    assert module._holding_object


def test_empty_grasp_reopens_before_failing(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    manipulation: Any = module._manipulation
    module._selected_grasp = PoseStamped(frame_id="world")

    def settle(read: Any, target: float, config: Any) -> GripperSettle:
        position = 0.0 if target == config.closed_position else target
        return GripperSettle(True, position, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)

    result = module.pick_selected()

    assert result.error_code == "GRASP_VERIFICATION_FAILED"
    assert not module._holding_object
    assert manipulation.set_gripper_position.call_args_list[-1].args[0] == 1.0


def test_pick_rejects_jaws_that_never_closed(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    module._selected_grasp = PoseStamped(frame_id="world")

    def settle(read: Any, target: float, config: Any) -> GripperSettle:
        position = config.open_position if target == config.closed_position else target
        return GripperSettle(True, position, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)

    result = module.pick_selected()

    assert result.error_code == "GRASP_VERIFICATION_FAILED"
    assert not module._holding_object


def test_empty_grasp_reports_failed_recovery(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    manipulation: Any = module._manipulation
    module._selected_grasp = PoseStamped(frame_id="world")
    manipulation.set_gripper_position.side_effect = [
        SimpleNamespace(succeeded=True, message=""),
        SimpleNamespace(succeeded=True, message=""),
        SimpleNamespace(succeeded=False, message="recovery open failed"),
    ]

    def settle(read: Any, target: float, config: Any) -> GripperSettle:
        position = 0.0 if target == config.closed_position else target
        return GripperSettle(True, position, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)

    result = module.pick_selected()

    assert result.error_code == "GRIPPER_FAILED"
    assert "recovery open failed" in result.message


def test_pick_fails_when_gripper_command_is_rejected(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    module._selected_grasp = PoseStamped(frame_id="world")
    manipulation.set_gripper_position.return_value = SimpleNamespace(
        succeeded=False, message="controller unavailable"
    )

    result = module.pick_selected()

    assert result.error_code == "GRIPPER_FAILED"


def test_pick_fails_when_gripper_feedback_is_unavailable(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    module._selected_grasp = PoseStamped(frame_id="world")

    def settle(read: Any, target: float, config: Any) -> GripperSettle:
        if target == config.closed_position:
            return GripperSettle(False, None, False, config.timeout)
        return GripperSettle(True, target, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)

    result = module.pick_selected()

    assert result.error_code == "GRASP_VERIFICATION_FAILED"
    assert not module._holding_object


def test_place_retains_held_state_when_release_fails(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    module._selected_grasp = PoseStamped(frame_id="world")
    module._holding_object = True

    def settle(read: Any, target: float, config: Any) -> GripperSettle:
        return GripperSettle(True, 0.5, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.error_code == "GRIPPER_FAILED"
    assert module._holding_object
    assert module._selected_grasp is not None
