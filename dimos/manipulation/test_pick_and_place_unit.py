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
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.pick_and_place_spec import DetectedObject, PickPlaceStatus
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header


@pytest.fixture
def module() -> Iterator[PickAndPlaceModule]:
    instance = PickAndPlaceModule(planning_frame="world")
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
    instance._objects = {"cup-1": DetectedObject("cup-1", "cup")}
    instance._scene.get_object_pointcloud_by_object_id.return_value = MagicMock()
    instance._grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [_candidate(0.1)]
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


def test_scan_objects_uses_latest_scan_ids(module: PickAndPlaceModule) -> None:
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

    assert result.succeeded
    assert module.get_object("cup-1") == DetectedObject("cup-1", "cup")
    scene.scan_scene.assert_called_once_with(text=["cup"])


def test_pick_object_uses_first_provider_candidate(
    module: PickAndPlaceModule,
) -> None:
    grasp_generator: Any = module._grasp_generator
    first, second = _candidate(0.1, 0.1), _candidate(0.2, 0.9)
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [first, second]
    )

    result = module.pick_object("cup-1")

    assert result.succeeded
    assert module.get_grasp_candidates().candidates == [first, second]
    assert module._selected_grasp is not None
    assert module._selected_grasp.position.x == pytest.approx(0.1)
    assert result.rank == 0


def test_pick_object_rejects_non_planning_frame(module: PickAndPlaceModule) -> None:
    grasp_generator: Any = module._grasp_generator
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "camera"), [_candidate(0.1)]
    )

    result = module.pick_object("cup-1")

    assert not result.succeeded
    assert result.status is PickPlaceStatus.GRASP_FRAME_MISMATCH


def test_pick_object_rejects_empty_candidates(module: PickAndPlaceModule) -> None:
    module._grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), []
    )

    result = module.pick_object("cup-1")

    assert result.status is PickPlaceStatus.GRASP_GENERATION_FAILED
    module._manipulation.set_gripper_position.assert_not_called()


def test_pick_preserves_current_yaw_when_configured(module: PickAndPlaceModule) -> None:
    module.config.yaw_policy = "preserve_current"
    module._grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"),
        [
            GraspCandidate(
                Pose(
                    Vector3(0.1, 0.0, 0.2),
                    Quaternion.from_euler(Vector3(-3.141592653589793, 0.0, 0.1)),
                ),
                1.0,
            )
        ],
    )

    result = module.pick_object("cup-1")

    assert result.succeeded
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

    assert result.succeeded
    preplace = manipulation.plan_to_poses.call_args_list[0].args[0]["arm/tool"]
    assert preplace.position.z == pytest.approx(0.3)
    assert not module._holding_object
    assert module._selected_grasp is None


def test_scan_failure_clears_stale_selection(module: PickAndPlaceModule) -> None:
    scene: Any = module._scene
    module._selected_grasp = PoseStamped(frame_id="world")
    scene.scan_scene.side_effect = RuntimeError("No aligned RGB-D frame")

    result = module.scan_objects(["cup"])

    assert not result.succeeded
    assert result.status is PickPlaceStatus.PERCEPTION_FAILED
    assert module._selected_grasp is None
    assert module.get_object("cup-1") is None


def test_pick_rejects_when_already_holding(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    module._holding_object = True
    module._selected_grasp = PoseStamped(frame_id="world")

    pick = module.pick_object("cup-1")

    assert pick.status is PickPlaceStatus.INVALID_STATE
    manipulation.set_gripper_position.assert_not_called()


def test_failed_pick_clears_previous_selection(module: PickAndPlaceModule) -> None:
    module._selected_grasp = PoseStamped(frame_id="world")

    result = module.pick_object("missing")

    assert result.status is PickPlaceStatus.OBJECT_NOT_DETECTED
    assert module._selected_grasp is None


def test_pick_retains_held_state_when_retract_fails(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    manipulation.execute.side_effect = [
        SimpleNamespace(succeeded=True, message=""),
        SimpleNamespace(succeeded=True, message=""),
        SimpleNamespace(succeeded=False, message="retract failed"),
    ]

    result = module.pick_object("cup-1")

    assert result.status is PickPlaceStatus.EXECUTION_FAILED
    assert result.holding_object
    assert module._holding_object


def test_empty_grasp_reopens_before_failing(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    manipulation: Any = module._manipulation

    def settle(read: Any, target: float, config: Any) -> GripperSettle:
        position = 0.0 if target == config.closed_position else target
        return GripperSettle(True, position, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)

    result = module.pick_object("cup-1")

    assert result.status is PickPlaceStatus.GRASP_VERIFICATION_FAILED
    assert not module._holding_object
    assert manipulation.set_gripper_position.call_args_list[-1].args[0] == 1.0
    assert result.rank == 0
    assert result.candidates == 1
    assert result.score == 1.0


def test_pick_rejects_jaws_that_never_closed(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    def settle(read: Any, target: float, config: Any) -> GripperSettle:
        position = config.open_position if target == config.closed_position else target
        return GripperSettle(True, position, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)

    result = module.pick_object("cup-1")

    assert result.status is PickPlaceStatus.GRASP_VERIFICATION_FAILED
    assert not module._holding_object


def test_empty_grasp_reports_failed_recovery(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    manipulation: Any = module._manipulation
    manipulation.set_gripper_position.side_effect = [
        SimpleNamespace(succeeded=True, message=""),
        SimpleNamespace(succeeded=True, message=""),
        SimpleNamespace(succeeded=False, message="recovery open failed"),
    ]

    def settle(read: Any, target: float, config: Any) -> GripperSettle:
        position = 0.0 if target == config.closed_position else target
        return GripperSettle(True, position, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)

    result = module.pick_object("cup-1")

    assert result.status is PickPlaceStatus.GRIPPER_FAILED
    assert "recovery open failed" in result.message


def test_pick_fails_when_gripper_command_is_rejected(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    manipulation.set_gripper_position.return_value = SimpleNamespace(
        succeeded=False, message="controller unavailable"
    )

    result = module.pick_object("cup-1")

    assert result.status is PickPlaceStatus.GRIPPER_FAILED


def test_pick_fails_when_gripper_feedback_is_unavailable(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    def settle(read: Any, target: float, config: Any) -> GripperSettle:
        if target == config.closed_position:
            return GripperSettle(False, None, False, config.timeout)
        return GripperSettle(True, target, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)

    result = module.pick_object("cup-1")

    assert result.status is PickPlaceStatus.GRASP_VERIFICATION_FAILED
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

    assert result.status is PickPlaceStatus.GRIPPER_FAILED
    assert result.holding_object
    assert module._holding_object
    assert module._selected_grasp is not None


def test_place_retract_failure_reports_object_released(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    module._selected_grasp = PoseStamped(frame_id="world")
    module._holding_object = True
    manipulation.execute.side_effect = [
        SimpleNamespace(succeeded=True, message=""),
        SimpleNamespace(succeeded=True, message=""),
        SimpleNamespace(succeeded=False, message="retract failed"),
    ]

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.status is PickPlaceStatus.EXECUTION_FAILED
    assert not result.holding_object
    assert module._selected_grasp is None


def test_motion_skills_declare_movement_capability() -> None:
    skills = [
        PickAndPlaceModule.pick_object,
        PickAndPlaceModule.place_at,
        ManipulationSkills.move_to_pose,
        ManipulationSkills.move_to_joints,
        ManipulationSkills.go_home,
        ManipulationSkills.go_init,
        ManipulationSkills.set_gripper,
        ManipulationSkills.open_gripper,
        ManipulationSkills.close_gripper,
    ]

    assert all(skill.__skill_uses__ == ["movement"] for skill in skills)
