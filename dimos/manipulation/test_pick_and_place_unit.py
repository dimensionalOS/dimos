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

import numpy as np
import pytest

from dimos.manipulation.grasp_verification import GripperSettle
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.manipulation_spec import (
    CommandResult,
    CommandStatus,
    ExecutionResult,
    ExecutionStatus,
    MoveResult,
    PlanResult,
    PlanStatus,
)
from dimos.manipulation.pick_and_place_module import TARGET_BOX, PickAndPlaceModule
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.utils.transform_utils import pose_to_matrix

PLANNED = PlanResult(PlanStatus.SUCCEEDED)
NO_PATH = PlanResult(PlanStatus.FAILED, "unreachable")
COMPLETED = ExecutionResult(ExecutionStatus.COMPLETED)
ACCEPTED = CommandResult(CommandStatus.SUCCEEDED)


def _linear_move(execution: ExecutionResult = COMPLETED) -> MoveResult:
    return MoveResult(PLANNED, execution, (0.0, 0.0, 0.0), False)


@pytest.fixture
def module() -> Iterator[PickAndPlaceModule]:
    instance = PickAndPlaceModule(planning_frame="world")
    instance._scene = MagicMock()
    instance._grasp_generator = MagicMock()
    instance._manipulation = MagicMock()
    instance._box_filter = MagicMock()
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
    instance._manipulation.plan_to_poses.return_value = PLANNED
    instance._manipulation.execute.return_value = COMPLETED
    instance._manipulation.move_linear.return_value = _linear_move()
    instance._manipulation.set_gripper_position.return_value = ACCEPTED
    instance._objects = {"cup-1": {"object_id": "cup-1", "name": "cup"}}
    # The cup's points: a 4 cm cube centred on the grasp at (0.1, 0.0, 0.2).
    instance._scene.get_object_pointcloud_by_object_id.return_value = PointCloud2.from_numpy(
        np.asarray([[0.08, -0.02, 0.18], [0.12, 0.02, 0.22]], dtype=np.float32),
        frame_id="world",
        timestamp=1.0,
    )
    instance._grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [_candidate(0.1)]
    )
    yield instance
    instance.stop()


@pytest.fixture(autouse=True)
def settled_gripper(monkeypatch: pytest.MonkeyPatch) -> None:
    def settle(read: Any, target: float, config: Any, **_: Any) -> GripperSettle:
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

    assert result.message == "Detected 1 object(s)"
    assert module.get_object("cup-1") == {"object_id": "cup-1", "name": "cup"}
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

    assert result.message == "Pick complete"
    assert module.get_grasp_candidates().candidates == [first, second]
    assert module._selected_grasp is not None
    assert module._selected_grasp.position.x == pytest.approx(0.1)
    assert result.metadata["rank"] == 0


def test_pick_object_rejects_non_planning_frame(module: PickAndPlaceModule) -> None:
    grasp_generator: Any = module._grasp_generator
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "camera"), [_candidate(0.1)]
    )

    with pytest.raises(RuntimeError, match="frame 'camera'.*planning frame is 'world'"):
        module.pick_object("cup-1")


def test_pick_falls_through_to_the_next_reachable_candidate(
    module: PickAndPlaceModule,
) -> None:
    """A learned provider's best-scoring pose is not always kinematically reachable."""
    manipulation: Any = module._manipulation
    module._grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [_candidate(0.1, score=0.9), _candidate(0.3, score=0.4)]
    )
    manipulation.plan_to_poses.side_effect = [NO_PATH, PLANNED, PLANNED, PLANNED]

    result = module.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert result.metadata["rank"] == 1
    assert result.metadata["score"] == 0.4


def test_pick_stops_walking_candidates_on_a_drive_fault(module: PickAndPlaceModule) -> None:
    """An execution fault would repeat for every candidate, so it is not a demotion."""
    manipulation: Any = module._manipulation
    module._grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [_candidate(0.1), _candidate(0.3)]
    )
    manipulation.execute.return_value = ExecutionResult(ExecutionStatus.FAULT, "drive fault")

    result = module.pick_object("cup-1")

    assert result.message == (
        "Move to grasp candidate 0 for object cup-1 did not complete; "
        "execution returned FAULT: drive fault"
    )
    assert manipulation.plan_to_poses.call_count == 1
    assert not module._holding_object


def test_pick_reports_no_reachable_candidate_when_every_attempt_fails(
    module: PickAndPlaceModule,
) -> None:
    manipulation: Any = module._manipulation
    module._grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [_candidate(0.1), _candidate(0.3)]
    )
    manipulation.plan_to_poses.return_value = NO_PATH

    result = module.pick_object("cup-1")

    assert result.message == (
        "The planner found no path to any of the 2 grasp candidate(s) tried for object cup-1; "
        "last planner result FAILED: unreachable."
    )
    assert not module._holding_object


def test_proposals_reach_the_viewer_as_they_are_generated(module: PickAndPlaceModule) -> None:
    """get_grasp_candidates only answers after the fact, which is no help live."""
    manipulation: Any = module._manipulation
    shown: list[GraspCandidateArray] = []
    manipulation.show_grasp_proposals.side_effect = lambda array: shown.append(array)

    assert module.pick_object("cup-1").message == "Pick complete"

    # The stale overlay is cleared first, then the fresh proposals go out.
    assert [[c.score for c in array.candidates] for array in shown] == [[], [1.0]]


def test_pick_object_rejects_empty_candidates(module: PickAndPlaceModule) -> None:
    module._grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), []
    )

    result = module.pick_object("cup-1")

    assert result.message == "Generated 0 grasp candidates for object cup-1."
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

    assert result.message == "Pick complete"
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

    assert result.message == "Place complete"
    preplace = manipulation.plan_to_poses.call_args_list[0].args[0]["arm/tool"]
    assert preplace.position.z == pytest.approx(0.3)
    assert not module._holding_object
    assert module._selected_grasp is None


def test_scan_failure_clears_stale_selection(module: PickAndPlaceModule) -> None:
    scene: Any = module._scene
    module._selected_grasp = PoseStamped(frame_id="world")
    scene.scan_scene.side_effect = RuntimeError("No aligned RGB-D frame")

    with pytest.raises(RuntimeError, match="No aligned RGB-D frame"):
        module.scan_objects(["cup"])

    assert module._selected_grasp is None
    assert module.get_object("cup-1") is None


def test_pick_rejects_when_already_holding(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    module._holding_object = True
    module._selected_object_id = "cup-0"
    module._selected_grasp = PoseStamped(frame_id="world")

    pick = module.pick_object("cup-1")

    assert pick.message == (
        "Still holding object cup-0; did not start a pick of object cup-1. "
        "Use place_at to put it down first."
    )
    manipulation.set_gripper_position.assert_not_called()


def test_failed_pick_clears_previous_selection(module: PickAndPlaceModule) -> None:
    module._selected_grasp = PoseStamped(frame_id="world")

    result = module.pick_object("missing")

    assert result.message == (
        "No object with id missing in the latest scan. Scanned ids: cup-1. "
        "Use scan_objects to refresh the list."
    )
    assert module._selected_grasp is None


def test_pick_retains_held_state_when_retract_fails(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    # Approach in, then the retract out; both legs are linear servos now.
    manipulation.move_linear.side_effect = [
        _linear_move(),
        _linear_move(ExecutionResult(ExecutionStatus.FAULT, "retract failed")),
    ]

    result = module.pick_object("cup-1")

    assert result.message == (
        "Retract after grasping object cup-1 did not complete; "
        "execution returned FAULT: retract failed"
    )
    assert module._holding_object


def test_final_grasp_leg_skips_collision_checking(module: PickAndPlaceModule) -> None:
    """The target is mapped geometry, so a checked plan into it always collides."""
    manipulation: Any = module._manipulation

    assert module.pick_object("cup-1").message == "Pick complete"
    assert manipulation.move_linear.call_args_list
    for call in manipulation.move_linear.call_args_list:
        assert call.kwargs["check_collision"] is False


def test_empty_grasp_reopens_before_failing(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    manipulation: Any = module._manipulation

    def settle(read: Any, target: float, config: Any, **_: Any) -> GripperSettle:
        position = 0.0 if target == config.closed_position else target
        return GripperSettle(True, position, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)

    result = module.pick_object("cup-1")

    assert result.message == (
        "Closed the gripper on object cup-1. Final gripper position 0.00. Reopened the gripper."
    )
    assert not module._holding_object
    assert manipulation.set_gripper_position.call_args_list[-1].args[0] == 1.0


def test_pick_rejects_jaws_that_never_closed(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    manipulation: Any = module._manipulation

    def settle(read: Any, target: float, config: Any, **_: Any) -> GripperSettle:
        position = config.open_position if target == config.closed_position else target
        return GripperSettle(True, position, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)

    result = module.pick_object("cup-1")

    assert result.message == "Closed the gripper on object cup-1. Final gripper position 1.00."
    assert not module._holding_object
    # Jaws that never closed are not reopened; the last command was the close.
    assert manipulation.set_gripper_position.call_args_list[-1].args[0] == 0.0


def test_empty_grasp_raises_when_the_recovery_open_is_rejected(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    manipulation: Any = module._manipulation
    manipulation.set_gripper_position.side_effect = [
        ACCEPTED,
        ACCEPTED,
        CommandResult(CommandStatus.FAILED, "recovery open failed"),
    ]

    def settle(read: Any, target: float, config: Any, **_: Any) -> GripperSettle:
        position = 0.0 if target == config.closed_position else target
        return GripperSettle(True, position, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)

    with pytest.raises(RuntimeError, match="recovery open failed"):
        module.pick_object("cup-1")


def test_pick_raises_when_gripper_command_is_rejected(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    manipulation.set_gripper_position.return_value = CommandResult(
        CommandStatus.FAILED, "controller unavailable"
    )

    with pytest.raises(RuntimeError, match="controller unavailable"):
        module.pick_object("cup-1")


def test_pick_raises_when_gripper_feedback_is_unavailable(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    def settle(read: Any, target: float, config: Any, **_: Any) -> GripperSettle:
        if target == config.closed_position:
            return GripperSettle(False, None, False, config.timeout)
        return GripperSettle(True, target, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)

    with pytest.raises(RuntimeError, match="No gripper position readback"):
        module.pick_object("cup-1")

    assert not module._holding_object


def test_place_retains_held_state_when_release_fails(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    module._selected_grasp = PoseStamped(frame_id="world")
    module._holding_object = True

    def settle(read: Any, target: float, config: Any, **_: Any) -> GripperSettle:
        return GripperSettle(True, 0.5, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.message == (
        "Commanded the gripper open to release the object. Final gripper position 0.50. "
        "The arm stayed at the place pose."
    )
    assert module._holding_object
    assert module._selected_grasp is not None


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


def _boxes(box_filter: Any) -> list[tuple[Any, ...]]:
    """Every set_box call as (pose, size, frame_id)."""
    calls = box_filter.set_box.call_args_list
    assert all(c.args[0] == TARGET_BOX for c in calls)
    return [c.args[1:] for c in calls]


def _close_on_nothing(monkeypatch: pytest.MonkeyPatch) -> None:
    def settle(read: Any, target: float, config: Any, **_: Any) -> GripperSettle:
        position = 0.0 if target == config.closed_position else target
        return GripperSettle(True, position, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)


def test_pick_leaves_the_target_out_of_the_map_then_carries_it(
    module: PickAndPlaceModule,
) -> None:
    """The target is mapped geometry; the approach goes into it and the lift takes it along."""
    box_filter: Any = module._box_filter
    grasp_generator: Any = module._grasp_generator
    # Off the cup's centre and tilted, so a box rotated the wrong way lands elsewhere.
    grasp = Pose(Vector3(0.1, 0.02, 0.21), Quaternion.from_euler(Vector3(-2.6, 0.0, 0.6)))
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [GraspCandidate(grasp, 1.0)]
    )

    assert module.pick_object("cup-1").message == "Pick complete"

    fixed, carried = _boxes(box_filter)
    # The cup's bounds plus the 3 cm clearance each way, fixed where it stood.
    assert fixed[0].position.to_tuple() == pytest.approx((0.1, 0.0, 0.2))
    assert fixed[0].orientation.angle_to(Quaternion()) == pytest.approx(0.0)
    assert fixed[1].to_tuple() == pytest.approx((0.1, 0.1, 0.1))
    assert fixed[2] == "world"
    # Then the same box rides on the tool tip: placed from the tip at the grasp
    # pose, it lands back on the cup's centre, still level.
    assert carried[2] == "tool"
    placed = pose_to_matrix(grasp) @ pose_to_matrix(carried[0])
    np.testing.assert_allclose(placed[:3, 3], [0.1, 0.0, 0.2], atol=1e-6)
    np.testing.assert_allclose(placed[:3, :3], np.eye(3), atol=1e-6)
    assert carried[1].to_tuple() == pytest.approx((0.1, 0.1, 0.1))
    box_filter.clear_box.assert_not_called()


def test_a_failed_pick_makes_the_target_an_obstacle_again(module: PickAndPlaceModule) -> None:
    box_filter: Any = module._box_filter
    module._manipulation.plan_to_poses.return_value = NO_PATH

    result = module.pick_object("cup-1")

    assert result.message.startswith("The planner found no path")
    assert len(_boxes(box_filter)) == 1
    box_filter.clear_box.assert_called_once_with(TARGET_BOX)


def test_a_pick_that_raises_makes_the_target_an_obstacle_again(
    module: PickAndPlaceModule,
) -> None:
    box_filter: Any = module._box_filter
    module._manipulation.set_gripper_position.return_value = CommandResult(
        CommandStatus.FAILED, "controller unavailable"
    )

    with pytest.raises(RuntimeError, match="controller unavailable"):
        module.pick_object("cup-1")

    box_filter.clear_box.assert_called_once_with(TARGET_BOX)


def test_release_makes_the_placed_object_an_obstacle_again(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    box_filter: Any = module._box_filter
    module._selected_grasp = PoseStamped(frame_id="world")
    module._holding_object = True
    order: list[str] = []

    def servo(*args: Any, **kwargs: Any) -> MoveResult:
        order.append("servo")
        return _linear_move()

    manipulation.move_linear.side_effect = servo
    box_filter.clear_box.side_effect = lambda name: order.append("clear")

    assert module.place_at(0.4, 0.0, 0.2).message == "Place complete"

    # Down, release and take the box off the tool, then back up.
    assert order == ["servo", "clear", "servo"]


def test_a_release_whose_retract_stops_still_clears_the_box(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    box_filter: Any = module._box_filter
    module._selected_grasp = PoseStamped(frame_id="world")
    module._holding_object = True
    manipulation.move_linear.side_effect = [
        _linear_move(),
        _linear_move(ExecutionResult(ExecutionStatus.FAULT, "retract failed")),
    ]

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.message.startswith("Retract from (0.40, 0.00, 0.20) after releasing")
    box_filter.clear_box.assert_called_once_with(TARGET_BOX)


def test_an_empty_target_cloud_sets_no_box(module: PickAndPlaceModule) -> None:
    box_filter: Any = module._box_filter
    scene: Any = module._scene
    scene.get_object_pointcloud_by_object_id.return_value = PointCloud2.from_numpy(
        np.zeros((0, 3), dtype=np.float32), frame_id="world", timestamp=1.0
    )

    assert module.pick_object("cup-1").message == "Pick complete"

    box_filter.set_box.assert_not_called()


def test_an_empty_close_backs_out_before_the_target_is_an_obstacle_again(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    manipulation: Any = module._manipulation
    box_filter: Any = module._box_filter
    _close_on_nothing(monkeypatch)
    order: list[str] = []

    def servo(*args: Any, **kwargs: Any) -> MoveResult:
        order.append("servo")
        return _linear_move()

    manipulation.move_linear.side_effect = servo
    box_filter.clear_box.side_effect = lambda name: order.append("clear")

    result = module.pick_object("cup-1")

    assert result.message == (
        "Closed the gripper on object cup-1. Final gripper position 0.00. Reopened the gripper."
    )
    # Down to the grasp, back up to the pregrasp, and only then the box goes.
    assert order == ["servo", "servo", "clear"]
    dz = [c.args[2] for c in manipulation.move_linear.call_args_list]
    assert dz[0] == pytest.approx(-dz[1])


def test_an_empty_close_reports_a_back_out_that_did_not_complete(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    manipulation: Any = module._manipulation
    box_filter: Any = module._box_filter
    _close_on_nothing(monkeypatch)
    manipulation.move_linear.side_effect = [
        _linear_move(),
        _linear_move(ExecutionResult(ExecutionStatus.FAULT, "retract failed")),
    ]

    result = module.pick_object("cup-1")

    assert result.message == (
        "Closed the gripper on object cup-1. Final gripper position 0.00. Reopened the gripper. "
        "Retract to the pre-grasp pose did not complete; execution returned FAULT: retract failed"
    )
    assert not module._holding_object
    box_filter.clear_box.assert_called_once_with(TARGET_BOX)


def test_pick_and_place_work_without_a_box_filter(module: PickAndPlaceModule) -> None:
    """A stack without a voxel map composes no box filter."""
    module._box_filter = None

    assert module.pick_object("cup-1").message == "Pick complete"
    assert module.place_at(0.4, 0.0, 0.2).message == "Place complete"
