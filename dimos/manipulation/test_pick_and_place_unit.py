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
from dimos.manipulation.manipulation_spec import (
    CommandResult,
    CommandStatus,
    ExecutionResult,
    ExecutionStatus,
    MoveResult,
    PlanResult,
    PlanStatus,
)
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header

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
    instance._scene.get_object_pointcloud_by_object_id.return_value = MagicMock()
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
                id="cup-1",
                results=[SimpleNamespace(hypothesis=SimpleNamespace(class_id="cup"))],
                bbox=SimpleNamespace(
                    center=SimpleNamespace(position=SimpleNamespace(x=0.31, y=-0.2, z=-0.01))
                ),
            )
        ],
    )

    result = module.scan_objects([" cup "])

    assert result.message == "Detected 1 object(s)"
    assert module.get_object("cup-1") == {
        "object_id": "cup-1",
        "name": "cup",
        "x": 0.31,
        "y": -0.2,
        "z": -0.01,
    }
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


def test_pregrasp_along_tool_z_backs_off_the_other_way(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    module.config.pregrasp_along_tool_z = True
    module._selected_grasp = PoseStamped(
        frame_id="world",
        orientation=Quaternion.from_euler(Vector3(-3.141592653589793, 0.0, 0.0)),
    )
    module._holding_object = True

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.message == "Place complete"
    preplace = manipulation.plan_to_poses.call_args_list[0].args[0]["arm/tool"]
    assert preplace.position.z == pytest.approx(0.1)


def test_grasp_proposals_and_the_attempt_are_published_for_the_viewer(
    module: PickAndPlaceModule,
) -> None:
    arrays: list[Any] = []
    targets: list[PoseStamped] = []
    module.grasp_candidates.subscribe(arrays.append)
    module.grasp_target.subscribe(targets.append)

    assert module.pick_object("cup-1").message == "Pick complete"

    assert [len(array.poses) for array in arrays] == [1]
    assert arrays[0].header.frame_id == "world"
    assert [target.frame_id for target in targets] == ["world"]
    assert targets[0].position.x == pytest.approx(0.1)


def _fake_plan(label: str, seconds: float = 1.0) -> Any:
    from dimos.manipulation.planning.spec.enums import PlanningStatus
    from dimos.manipulation.planning.spec.models import GeneratedPlan
    from dimos.msgs.sensor_msgs.JointState import JointState
    from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
    from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint

    names = ["j1", "j2"]
    end = JointState(name=names, position=[0.1, 0.2])
    trajectory = JointTrajectory(
        points=[
            TrajectoryPoint(time_from_start=0.0, positions=[0.0, 0.0]),
            TrajectoryPoint(time_from_start=seconds, positions=[0.1, 0.2]),
        ],
        joint_names=names,
    )
    return GeneratedPlan(
        group_ids=("arm/tool",),
        trajectory=trajectory,
        path=[JointState(name=names, position=[0.0, 0.0]), end],
        status=PlanningStatus.SUCCESS,
        message=label,
    )


def _planned(label: str) -> PlanResult:
    return PlanResult(PlanStatus.SUCCEEDED, label, _fake_plan(label))


@pytest.fixture
def staged(module: PickAndPlaceModule) -> PickAndPlaceModule:
    from dimos.msgs.sensor_msgs.JointState import JointState

    manipulation: Any = module._manipulation
    manipulation.get_current_joint_state.return_value = JointState(
        name=["j1", "j2"], position=[0.0, 0.0]
    )
    manipulation.plan_to_poses.side_effect = lambda targets, **kw: _planned("pose")
    manipulation.plan_linear.side_effect = lambda *a, **kw: _planned("linear")
    manipulation.plan_to_joints.side_effect = lambda targets, **kw: _planned("joints")
    manipulation.get_state.return_value.groups["arm/tool"].joint_presets = {
        "home": JointState(name=["j1", "j2"], position=[0.0, 0.0])
    }
    manipulation.execute_plan.return_value = COMPLETED
    manipulation.preview_plans.return_value = ACCEPTED
    return module


def test_stage_plans_the_whole_job_from_predicted_states_without_moving(
    staged: PickAndPlaceModule,
) -> None:
    manipulation: Any = staged._manipulation

    result = staged.stage_pick_and_place("cup-1", 0.35, -0.02, 0.25)

    assert result.message.startswith("Staged a pick of object cup-1")
    assert result.metadata["legs"] == [
        "open the gripper",
        "approach above the object",
        "descend to the grasp",
        "close and verify the grasp",
        "lift",
        "carry above the place",
        "lower to the place",
        "release",
        "retreat",
        "return home",
    ]
    assert result.metadata["motion_seconds"] == pytest.approx(7.0)
    # Every leg after the first planned from the predicted end of the one before.
    starts = [call.kwargs["start"] for call in manipulation.plan_linear.call_args_list]
    assert all(list(s.position) == [0.1, 0.2] for s in starts)
    manipulation.preview_plans.assert_called_once()
    assert len(manipulation.preview_plans.call_args.args[0]) == 7
    manipulation.execute.assert_not_called()
    manipulation.execute_plan.assert_not_called()
    manipulation.set_gripper_position.assert_not_called()


def test_proceed_runs_the_staged_legs_in_order_and_reports(staged: PickAndPlaceModule) -> None:
    manipulation: Any = staged._manipulation
    staged.stage_pick_and_place("cup-1", 0.35, -0.02, 0.25)

    result = staged.proceed()

    assert result.message == "Pick and place complete"
    assert [c.args[0].message for c in manipulation.execute_plan.call_args_list] == [
        "pose",
        "linear",
        "linear",
        "pose",
        "linear",
        "linear",
        "joints",
    ]
    # open before the grasp, close on it, open again to release
    positions = [c.args[0] for c in manipulation.set_gripper_position.call_args_list]
    assert positions[0] > 0.5 and positions[1] < 0.5 and positions[2] > 0.5
    assert staged._holding_object is False
    assert staged.proceed().message == "Nothing is staged. Use stage_pick_and_place first."


def test_stage_moves_to_the_next_candidate_when_a_leg_cannot_be_planned(
    staged: PickAndPlaceModule,
) -> None:
    manipulation: Any = staged._manipulation
    staged._grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [_candidate(0.1, 0.9), _candidate(0.2, 0.8)]
    )
    # the first candidate's approach fails at every clearance (10, 7, 5 cm)
    calls = iter([NO_PATH, NO_PATH, NO_PATH, _planned("pose"), _planned("pose")])
    manipulation.plan_to_poses.side_effect = lambda targets, **kw: next(calls)

    result = staged.stage_pick_and_place("cup-1", 0.35, -0.02, 0.25)

    assert result.metadata["rank"] == 1
    assert staged._staged is not None and staged._staged.rank == 1


def test_discard_drops_the_staged_job_and_clears_the_preview(staged: PickAndPlaceModule) -> None:
    manipulation: Any = staged._manipulation
    staged.stage_pick_and_place("cup-1", 0.35, -0.02, 0.25)

    assert staged.discard_staged().message.startswith("Discarded")
    manipulation.clear_planned_path.assert_called_once()
    assert staged._staged is None
    assert staged.discard_staged().message == "Nothing was staged."


def test_stage_turns_the_wrist_for_the_place_when_the_grasp_heading_cannot_reach(
    staged: PickAndPlaceModule,
) -> None:
    manipulation: Any = staged._manipulation
    poses_tried: list[Any] = []

    def plan_to_poses(targets: dict[str, PoseStamped], **kw: Any) -> PlanResult:
        pose = next(iter(targets.values()))
        poses_tried.append(pose)
        # approach plans; the first carry attempt (grasp heading) does not
        if len(poses_tried) == 2:
            return NO_PATH
        return _planned("pose")

    manipulation.plan_to_poses.side_effect = plan_to_poses

    result = staged.stage_pick_and_place("cup-1", 0.33, -0.10, 0.14)

    assert result.message.startswith("Staged a pick of object cup-1")
    assert [leg for leg in result.metadata["legs"]].count("carry above the place") == 1
    assert len(poses_tried) == 3
    first_carry, second_carry = poses_tried[1], poses_tried[2]
    assert first_carry.position.x == pytest.approx(second_carry.position.x)
    assert first_carry.orientation != second_carry.orientation


def test_stage_tilts_the_wrist_for_the_place_when_no_upright_heading_reaches(
    staged: PickAndPlaceModule,
) -> None:
    from dimos.manipulation.pick_and_place_module import _PLACE_YAW_DELTAS
    from dimos.msgs.geometry_msgs.Vector3 import Vector3

    manipulation: Any = staged._manipulation
    carries: list[Any] = []

    def plan_to_poses(targets: dict[str, PoseStamped], **kw: Any) -> PlanResult:
        pose = next(iter(targets.values()))
        carries.append(pose)
        # the approach plans; every upright carry heading fails, the first tilt plans
        if 1 < len(carries) <= 1 + len(_PLACE_YAW_DELTAS):
            return NO_PATH
        return _planned("pose")

    manipulation.plan_to_poses.side_effect = plan_to_poses

    result = staged.stage_pick_and_place("cup-1", 0.46, 0.05, 0.12)

    assert result.message.startswith("Staged a pick of object cup-1")
    assert len(carries) == 2 + len(_PLACE_YAW_DELTAS)
    upright, tilted = carries[1], carries[-1]
    up = Vector3(0.0, 0.0, 1.0)
    assert upright.orientation.rotate_vector(up).z == pytest.approx(
        carries[2].orientation.rotate_vector(up).z
    )
    assert tilted.orientation.rotate_vector(up).z != pytest.approx(
        upright.orientation.rotate_vector(up).z
    )


def _scan_with(detections: list[tuple[str, float, float, float]]) -> Any:
    return SimpleNamespace(
        detections_length=len(detections),
        detections=[
            SimpleNamespace(
                id=det_id,
                results=[SimpleNamespace(hypothesis=SimpleNamespace(class_id="cup"))],
                bbox=SimpleNamespace(
                    center=SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=z))
                ),
            )
            for det_id, x, y, z in detections
        ],
    )


def test_proceed_stops_after_the_lift_when_the_camera_still_sees_the_object(
    staged: PickAndPlaceModule,
) -> None:
    staged._objects["cup-1"] = {"object_id": "cup-1", "name": "cup", "x": 0.3, "y": 0.2, "z": 0.0}
    scene: Any = staged._scene
    scene.scan_scene.return_value = _scan_with([("cup-1", 0.31, 0.2, 0.0)])
    manipulation: Any = staged._manipulation
    manipulation.get_state.return_value.groups["arm/tool"].gripper_position = 0.05

    assert staged.stage_pick_and_place("cup-1", 0.46, 0.05, 0.12).message.startswith("Staged")
    result = staged.proceed()

    assert "STOPPED at leg 5" in result.message and "lift" in result.message
    assert "still sees the cup" in result.message
    assert staged._holding_object is False
    # the jaws closed once and were not reopened by a jaw verdict
    assert scene.scan_scene.call_args.kwargs == {"text": ["cup"]}


def test_proceed_stops_when_the_object_was_pushed_aside_on_the_table(
    staged: PickAndPlaceModule,
) -> None:
    staged._objects["cup-1"] = {"object_id": "cup-1", "name": "cup", "x": 0.3, "y": 0.2, "z": 0.0}
    scene: Any = staged._scene
    # found 8 cm away but still at table height: pushed, not lifted
    scene.scan_scene.return_value = _scan_with([("cup-1", 0.38, 0.2, 0.0)])
    manipulation: Any = staged._manipulation
    manipulation.get_state.return_value.groups["arm/tool"].gripper_position = 0.02

    assert staged.stage_pick_and_place("cup-1", 0.46, 0.05, 0.12).message.startswith("Staged")
    result = staged.proceed()

    assert "STOPPED at leg 5" in result.message and "pushed" in result.message
    assert staged._holding_object is False
    assert staged.forget_held_object().message == "No object was recorded as held."


def test_proceed_completes_when_the_object_left_its_start_position(
    staged: PickAndPlaceModule,
) -> None:
    staged._objects["cup-1"] = {"object_id": "cup-1", "name": "cup", "x": 0.3, "y": 0.2, "z": 0.0}
    scene: Any = staged._scene
    scene.scan_scene.return_value = _scan_with([])
    manipulation: Any = staged._manipulation
    manipulation.get_state.return_value.groups["arm/tool"].gripper_position = 0.05

    assert staged.stage_pick_and_place("cup-1", 0.46, 0.05, 0.12).message.startswith("Staged")
    result = staged.proceed()

    assert result.message == "Pick and place complete"


def test_proceed_replans_a_leg_the_controller_rejects_from_the_live_state(
    staged: PickAndPlaceModule,
) -> None:
    manipulation: Any = staged._manipulation
    manipulation.get_state.return_value.groups["arm/tool"].end_effector_pose = PoseStamped(
        position=Vector3(0.33, -0.10, 0.23), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)
    )
    rejected = ExecutionResult(
        ExecutionStatus.REJECTED,
        "Trajectory start for joint 'right_joint4' differs from current position by 0.0504",
    )
    executions = iter([COMPLETED, rejected] + [COMPLETED] * 20)
    manipulation.execute_plan.side_effect = lambda plan, **kw: next(executions)
    linear_calls: list[tuple[float, float, float, Any]] = []

    def plan_linear(dx: float, dy: float, dz: float, *a: Any, **kw: Any) -> PlanResult:
        linear_calls.append((round(dx, 3), round(dy, 3), round(dz, 3), kw.get("start")))
        return _planned("linear")

    manipulation.plan_linear.side_effect = plan_linear

    assert staged.stage_pick_and_place("cup-1", 0.46, 0.05, 0.12).message.startswith("Staged")
    result = staged.proceed()

    assert result.message == "Pick and place complete"
    # the descend was planned once when staged and once more, from the live
    # tool pose, after the controller rejected the staged trajectory
    live = [c for c in linear_calls if c[3] is None]
    assert len(live) == 1
    assert live[0][2] < 0  # heads down to the grasp from the live tool pose


def test_stage_lowers_the_approach_when_the_full_clearance_cannot_plan(
    staged: PickAndPlaceModule,
) -> None:
    staged.config.pregrasp_along_tool_z = True
    manipulation: Any = staged._manipulation
    heights: list[float] = []

    def plan_to_poses(targets: dict[str, PoseStamped], **kw: Any) -> PlanResult:
        pose = next(iter(targets.values()))
        if len(heights) < 2:
            # the first approach (10 cm) and the second (7 cm) are out of reach
            heights.append(pose.position.z)
            return NO_PATH
        heights.append(pose.position.z)
        return _planned("pose")

    manipulation.plan_to_poses.side_effect = plan_to_poses

    result = staged.stage_pick_and_place("cup-1", 0.46, 0.05, 0.12)

    assert result.message.startswith("Staged a pick of object cup-1")
    # each retry sits closer to the grasp along the tool axis: 10, 7, then 5 cm
    approaches = heights[:3]  # the fourth planned pose is the carry
    assert approaches == sorted(approaches) or approaches == sorted(approaches, reverse=True)
    assert abs(approaches[0] - approaches[1]) == pytest.approx(0.03)
    assert abs(approaches[0] - approaches[2]) == pytest.approx(0.05)


def test_preplace_offset_shortens_the_lift_over_the_place(module: PickAndPlaceModule) -> None:
    from dimos.msgs.sensor_msgs.JointState import JointState

    module.config.preplace_offset = 0.05
    module.config.pregrasp_along_tool_z = True
    manipulation: Any = module._manipulation
    manipulation.get_current_joint_state.return_value = JointState(
        name=["j1", "j2"], position=[0, 0]
    )
    manipulation.plan_to_poses.side_effect = lambda targets, **kw: _planned("pose")
    manipulation.plan_to_joints.side_effect = lambda targets, **kw: _planned("joints")
    manipulation.get_state.return_value.groups["arm/tool"].joint_presets = {}
    deltas: list[tuple[float, float, float]] = []

    def plan_linear(dx: float, dy: float, dz: float, *a: Any, **kw: Any) -> PlanResult:
        deltas.append((dx, dy, dz))
        return _planned("linear")

    manipulation.plan_linear.side_effect = plan_linear
    assert module.stage_pick_and_place("cup-1", 0.33, -0.10, 0.14).message.startswith("Staged")

    # descend 10 cm to the grasp, lift 10 cm, lower 5 cm to the place, retreat 5 cm
    assert [round(abs(d[2]), 3) for d in deltas] == [0.1, 0.1, 0.05, 0.05]
