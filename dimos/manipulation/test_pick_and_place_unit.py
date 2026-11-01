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
import math
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
from dimos.manipulation.pick_and_place_module import TARGET_BOX, ArmReach, PickAndPlaceModule
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.utils.transform_utils import pose_to_matrix

# Where a planned approach ends: the arm's joints and the base that drove along.
APPROACH_GOAL = JointState(name=["arm/j0", "base/x"], position=[0.7, 0.2])
PLANNED = PlanResult(
    PlanStatus.SUCCEEDED,
    plan=GeneratedPlan(
        group_ids=("arm/tool", "moving_base"),
        trajectory=JointTrajectory(),
        path=[JointState(name=APPROACH_GOAL.name, position=[0.0, 0.0]), APPROACH_GOAL],
        status=PlanningStatus.SUCCESS,
    ),
)
NO_PATH = PlanResult(PlanStatus.FAILED, "unreachable")
# The dry run of the straight leg after an approach, from where the approach ends.
CHECKED = PlanResult(PlanStatus.SUCCEEDED, "RoboPlan Cartesian path found")
NO_DESCENT = PlanResult(
    PlanStatus.FAILED,
    "Cartesian planning failed: NO_SOLUTION: position error 0.007 m exceeds the path tolerance",
)
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
        SimpleNamespace(id="arm/tool", has_gripper=True, tip_frame="tool", base_frame="arm/base"),
        SimpleNamespace(id="moving_base", has_gripper=False, tip_frame=None, base_frame="base"),
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
    instance._manipulation.plan_linear.return_value = CHECKED
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
    scene: Any = module._scene
    scene.get_object_pointcloud_by_object_id.return_value = None
    module._selected_grasp = PoseStamped(frame_id="world")

    result = module.pick_object("missing")

    assert result.message == (
        "No object with id missing in the latest scan. Scanned ids: cup-1. "
        "Use scan_objects to refresh the list."
    )
    assert module._selected_grasp is None


def test_a_pick_takes_an_id_only_a_live_scene_knows(module: PickAndPlaceModule) -> None:
    """A watching scene lists objects no scan_objects call named."""
    scene: Any = module._scene
    module._objects = {}

    result = module.pick_object("watched-1")

    scene.get_object_pointcloud_by_object_id.assert_called_once_with("watched-1")
    assert "latest scan" not in result.message


def test_a_scanned_object_the_scene_lost_says_so(module: PickAndPlaceModule) -> None:
    scene: Any = module._scene
    scene.get_object_pointcloud_by_object_id.return_value = None

    result = module.pick_object("cup-1")

    assert result.message == (
        "Object cup-1 has no point cloud in the latest scan. Use scan_objects again."
    )


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


def _auxiliary_groups_of(calls: list[Any]) -> list[tuple[str, ...]]:
    """The auxiliary groups each planner call was given."""
    return [tuple(call.kwargs["auxiliary_groups"]) for call in calls]


def test_pick_plans_arm_only_by_default(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation

    assert module.pick_object("cup-1").message == "Pick complete"

    assert _auxiliary_groups_of(manipulation.plan_to_poses.call_args_list) == [()]
    for call in manipulation.move_linear.call_args_list:
        assert "auxiliary_groups" not in call.kwargs


def test_pick_brings_the_base_along_for_the_approach_only(module: PickAndPlaceModule) -> None:
    """The planner may drive the base to reach the pre-grasp; at the object the arm moves alone."""
    manipulation: Any = module._manipulation

    result = module.pick_object("cup-1", auxiliary_groups=["moving_base"])

    assert result.message == "Pick complete"
    (approach,) = manipulation.plan_to_poses.call_args_list
    assert list(approach.args[0]) == ["arm/tool"]
    assert tuple(approach.kwargs["auxiliary_groups"]) == ("moving_base",)
    # Down onto the cup, then the lift: both straight, arm-only legs.
    assert len(manipulation.move_linear.call_args_list) == 2
    for call in manipulation.move_linear.call_args_list:
        assert call.args[3] == "arm/tool"
        assert "auxiliary_groups" not in call.kwargs


def test_place_brings_the_base_along_for_the_pre_place_only(
    module: PickAndPlaceModule,
) -> None:
    manipulation: Any = module._manipulation
    module._selected_grasp = PoseStamped(
        frame_id="world",
        orientation=Quaternion.from_euler(Vector3(-3.141592653589793, 0.0, 0.0)),
    )
    module._holding_object = True

    result = module.place_at(0.4, 0.0, 0.2, auxiliary_groups=["moving_base"])

    assert result.message == "Place complete"
    assert _auxiliary_groups_of(manipulation.plan_to_poses.call_args_list) == [("moving_base",)]
    # Down to the place pose, then the retreat: both arm-only.
    assert len(manipulation.move_linear.call_args_list) == 2
    for call in manipulation.move_linear.call_args_list:
        assert "auxiliary_groups" not in call.kwargs


def _motion_calls(manipulation: Any) -> list[str]:
    """The manipulation RPCs a pick made, in order."""
    motion = {"set_gripper_position", "plan_to_poses", "plan_linear", "execute", "move_linear"}
    return [name for name, _args, _kwargs in manipulation.method_calls if name in motion]


def test_pick_checks_the_descent_from_the_approach_goal_before_moving(
    module: PickAndPlaceModule,
) -> None:
    """The planner is asked for the descent from where the approach ends, with nothing moving;
    only then is the approach executed, and the descent driven for real after it."""
    manipulation: Any = module._manipulation

    assert module.pick_object("cup-1", auxiliary_groups=["moving_base"]).message == "Pick complete"

    assert _motion_calls(manipulation) == [
        "set_gripper_position",  # open
        "plan_to_poses",  # the approach, base allowed along
        "plan_linear",  # the descent, dry, from the approach's goal joints
        "execute",  # the approach
        "move_linear",  # the descent
        "set_gripper_position",  # close
        "move_linear",  # the lift
    ]
    (check,) = manipulation.plan_linear.call_args_list
    assert check.args[:3] == pytest.approx((0.0, 0.0, -0.1))
    assert check.args[3] == "arm/tool"
    assert check.kwargs["check_collision"] is False
    assert check.kwargs["start_joints"] is APPROACH_GOAL
    assert PLANNED.plan is not None
    manipulation.execute.assert_called_once_with(blocking=True, plan_id=PLANNED.plan.plan_id)


def test_approach_only_stops_at_the_pregrasp_with_the_gripper_open(
    module: PickAndPlaceModule,
) -> None:
    """A grasp finished by hand: the arm goes to the pre-grasp with the descent checked,
    the jaws stay open, and the report says how far and which way to advance."""
    manipulation: Any = module._manipulation
    box_filter: Any = module._box_filter

    result = module.pick_object("cup-1", approach_only=True)

    assert result.message == (
        "At the pregrasp for cup-1 with arm/tool, gripper open: "
        "advance 10 cm along the grasp axis to grasp."
    )
    assert _motion_calls(manipulation) == [
        "set_gripper_position",
        "plan_to_poses",
        "plan_linear",
        "execute",
    ]
    manipulation.move_linear.assert_not_called()
    (opened,) = manipulation.set_gripper_position.call_args_list
    assert opened.args[0] == module.config.grasp_verification.open_position
    assert not module._holding_object
    box_filter.clear_box.assert_called_once_with(TARGET_BOX)
    assert result.metadata["object_id"] == "cup-1"
    assert result.metadata["planning_group"] == "arm/tool"
    assert result.metadata["approach_m"] == pytest.approx(0.1)
    assert result.metadata["approach_dir"] == pytest.approx([0.0, 0.0, -1.0])
    grasp = result.metadata["grasp"]
    assert grasp["frame_id"] == "world"
    assert grasp["position"] == pytest.approx([0.1, 0.0, 0.2])
    assert grasp["orientation"] == pytest.approx(
        Quaternion.from_euler(Vector3(-math.pi, 0.0, 0.0)).to_list()
    )


def test_pick_skips_a_candidate_whose_descent_cannot_be_planned(
    module: PickAndPlaceModule,
) -> None:
    """Today's failure: a base-assisted pre-grasp the arm could reach but not descend from.
    Found out before the approach runs, the candidate is skipped, the arm never moves."""
    manipulation: Any = module._manipulation
    grasp_generator: Any = module._grasp_generator
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [_candidate(0.1, score=0.9), _candidate(0.3, score=0.4)]
    )
    manipulation.plan_linear.side_effect = [NO_DESCENT, CHECKED]

    result = module.pick_object("cup-1", auxiliary_groups=["moving_base"])

    assert result.message == "Pick complete"
    assert result.metadata["rank"] == 1
    assert manipulation.plan_to_poses.call_count == 2
    # One approach executed: the second, once its descent checked out.
    manipulation.execute.assert_called_once()
    assert _motion_calls(manipulation)[:5] == [
        "set_gripper_position",
        "plan_to_poses",
        "plan_linear",
        "plan_to_poses",
        "plan_linear",
    ]


def test_pick_reports_why_the_last_candidate_could_not_be_descended_on(
    module: PickAndPlaceModule,
) -> None:
    manipulation: Any = module._manipulation
    manipulation.plan_linear.return_value = NO_DESCENT

    result = module.pick_object("cup-1")

    assert result.message == (
        "The planner found no path to any of the 1 grasp candidate(s) tried for object cup-1; "
        "last planner result FAILED: the descent from the pre-grasp pose is not plannable from "
        "where the approach ends: Cartesian planning failed: NO_SOLUTION: position error "
        "0.007 m exceeds the path tolerance."
    )
    manipulation.execute.assert_not_called()
    manipulation.move_linear.assert_not_called()
    assert not module._holding_object


def test_place_checks_the_descent_before_moving_to_the_pre_place(
    module: PickAndPlaceModule,
) -> None:
    manipulation: Any = module._manipulation
    manipulation.plan_linear.return_value = NO_DESCENT
    module._selected_grasp = PoseStamped(
        frame_id="world",
        orientation=Quaternion.from_euler(Vector3(-3.141592653589793, 0.0, 0.0)),
    )
    module._holding_object = True

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.message == (
        "Move to the pre-place pose above (0.40, 0.00, 0.20) did not complete; planner returned "
        "FAILED: the descent to the place pose is not plannable from where the approach ends: "
        "Cartesian planning failed: NO_SOLUTION: position error 0.007 m exceeds the path "
        "tolerance"
    )
    manipulation.execute.assert_not_called()
    assert module._holding_object


def _odom_to_world(parent: str, child: str, *args: Any, **kwargs: Any) -> Transform | None:
    """tf with the robot standing 1 m along odom's x axis."""
    if (parent, child) == ("world", "odom"):
        return Transform(Vector3(-1.0, 0.0, 0.0), frame_id="world", child_frame_id="odom")
    return None


def _cup_in_odom(module: PickAndPlaceModule) -> None:
    """The fixture's cup, 1 m further along x: where it is in odom."""
    scene: Any = module._scene
    scene.get_object_pointcloud_by_object_id.return_value = PointCloud2.from_numpy(
        np.asarray([[1.08, -0.02, 0.18], [1.12, 0.02, 0.22]], dtype=np.float32),
        frame_id="odom",
        timestamp=1.0,
    )


def test_pick_brings_a_scene_kept_in_another_frame_into_the_planning_frame(
    module: PickAndPlaceModule,
) -> None:
    """A scene kept in odom survives a base move. The grasps and the arm want the object in
    the planning frame; the exclusion box stays on the object, in odom, where one in the
    planning frame would ride along with the base."""
    grasp_generator: Any = module._grasp_generator
    box_filter: Any = module._box_filter
    module._tf = MagicMock()
    module._tf.get.side_effect = _odom_to_world
    _cup_in_odom(module)

    assert module.pick_object("cup-1").message == "Pick complete"

    module._tf.get.assert_called_once_with("world", "odom", warn=False)
    (cloud,), _ = grasp_generator.propose_grasps.call_args
    assert cloud.frame_id == "world"
    points, _ = cloud.as_numpy()
    np.testing.assert_allclose(points, [[0.08, -0.02, 0.18], [0.12, 0.02, 0.22]], atol=1e-6)
    manipulation: Any = module._manipulation
    pregrasp = manipulation.plan_to_poses.call_args.args[0]["arm/tool"]
    assert pregrasp.position.to_tuple() == pytest.approx((0.1, 0.0, 0.3))
    fixed, carried = _boxes(box_filter)
    assert fixed[0].position.to_tuple() == pytest.approx((1.1, 0.0, 0.2))
    assert fixed[1].to_tuple() == pytest.approx((0.1, 0.1, 0.1))
    assert fixed[2] == "odom"
    # Held, the box rides with the tool, placed from the grasp in the planning frame.
    assert carried[2] == "tool"
    assert carried[0].position.to_tuple() == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)


def test_pick_fails_fast_when_the_scene_frame_is_not_on_tf(module: PickAndPlaceModule) -> None:
    module._tf = MagicMock()
    module._tf.get.return_value = None
    _cup_in_odom(module)

    result = module.pick_object("cup-1")

    assert result.message == (
        "No transform from the scene frame 'odom' to the planning frame 'world' on tf; "
        "cannot locate object cup-1 for the arm."
    )
    grasp_generator: Any = module._grasp_generator
    manipulation: Any = module._manipulation
    box_filter: Any = module._box_filter
    grasp_generator.propose_grasps.assert_not_called()
    manipulation.set_gripper_position.assert_not_called()
    box_filter.set_box.assert_not_called()


def test_pick_leaves_a_scene_in_the_planning_frame_alone(module: PickAndPlaceModule) -> None:
    module._tf = MagicMock()

    assert module.pick_object("cup-1").message == "Pick complete"

    module._tf.get.assert_not_called()
    assert _boxes(module._box_filter)[0][2] == "world"


@pytest.mark.parametrize(
    ("auxiliary_groups", "complaint"),
    [
        (["wheels"], r"\['wheels'\] are not planning groups"),
        (["arm/tool"], "must not include the group doing the pick"),
    ],
)
def test_pick_rejects_auxiliary_groups_the_planner_cannot_move(
    module: PickAndPlaceModule, auxiliary_groups: list[str], complaint: str
) -> None:
    manipulation: Any = module._manipulation

    with pytest.raises(ValueError, match=complaint):
        module.pick_object("cup-1", auxiliary_groups=auxiliary_groups)

    manipulation.set_gripper_position.assert_not_called()


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


# Alfred: two OpenArms on a lift, shoulders at (0.14, ±0.155, 1.40) in base_link,
# 0.44 m from shoulder to wrist and 0.186 m from the wrist to the gripper's TCP.
LEFT, RIGHT = "left_manipulator", "right_manipulator"
SHOULDERS = {
    "left_shoulder": Vector3(0.14, 0.155, 1.40),
    "right_shoulder": Vector3(0.14, -0.155, 1.40),
}


def _openarm(shoulder_frame: str) -> ArmReach:
    return ArmReach(radius=0.44, shoulder_frame=shoulder_frame, wrist_to_tip=0.186)


def _shoulder_on_tf(parent: str, child: str, *args: Any, **kwargs: Any) -> Transform | None:
    if child not in SHOULDERS:
        return None
    return Transform(SHOULDERS[child], frame_id=parent, child_frame_id=child)


def _arm_state(y: float) -> SimpleNamespace:
    """An arm's state with its tool tip at (0.35, y, 1.10)."""
    return SimpleNamespace(
        gripper_position=0.5,
        end_effector_pose=PoseStamped(frame_id="world", position=Vector3(0.35, y, 1.10)),
    )


def _grasp_at(x: float, y: float, z: float, score: float = 1.0) -> GraspCandidate:
    """A top-down grasp: the tool's approach axis points down onto (x, y, z)."""
    return GraspCandidate(
        Pose(Vector3(x, y, z), Quaternion.from_euler(Vector3(-math.pi, 0.0, 0.0))), score
    )


def _bench_object(module: PickAndPlaceModule, x: float, y: float, z: float = 1.10) -> None:
    """A 4 cm cube centred at (x, y, z), with one top-down grasp at its centre."""
    scene: Any = module._scene
    grasp_generator: Any = module._grasp_generator
    scene.get_object_pointcloud_by_object_id.return_value = PointCloud2.from_numpy(
        np.asarray(
            [[x - 0.02, y - 0.02, z - 0.02], [x + 0.02, y + 0.02, z + 0.02]], dtype=np.float32
        ),
        frame_id="world",
        timestamp=1.0,
    )
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [_grasp_at(x, y, z)]
    )


@pytest.fixture
def two_arms(module: PickAndPlaceModule) -> PickAndPlaceModule:
    """Alfred-like: two arms with a reach each, shoulders on tf, a base that may drive."""
    manipulation: Any = module._manipulation
    manipulation.list_planning_groups.return_value = [
        SimpleNamespace(id=LEFT, has_gripper=True, tip_frame="left_tip", base_frame="lift"),
        SimpleNamespace(id=RIGHT, has_gripper=True, tip_frame="right_tip", base_frame="lift"),
        SimpleNamespace(id="moving_base", has_gripper=False, tip_frame=None, base_frame="base"),
    ]
    manipulation.get_state.return_value = SimpleNamespace(
        groups={LEFT: _arm_state(0.30), RIGHT: _arm_state(-0.30)}
    )
    module.config.reach = {LEFT: _openarm("left_shoulder"), RIGHT: _openarm("right_shoulder")}
    module._tf = MagicMock()
    module._tf.get.side_effect = _shoulder_on_tf
    # On the bench in front of the right shoulder, well within the arm.
    _bench_object(module, 0.35, -0.15)
    return module


def _planned_groups(manipulation: Any) -> list[str]:
    return list(manipulation.plan_to_poses.call_args.args[0])


@pytest.mark.parametrize(("y", "expected"), [(-0.15, RIGHT), (0.15, LEFT)])
@pytest.mark.parametrize("known_by", ["shoulder", "tool tip"])
def test_pick_uses_the_arm_nearest_the_object(
    two_arms: PickAndPlaceModule, y: float, expected: str, known_by: str
) -> None:
    """With planning_group omitted, the arm nearest the object picks: by its shoulder
    when its reach is configured, else by where its tool tip is."""
    manipulation: Any = two_arms._manipulation
    if known_by == "tool tip":
        two_arms.config.reach = {}
    _bench_object(two_arms, 0.35, y)

    assert two_arms.pick_object("cup-1").message == "Pick complete"

    assert _planned_groups(manipulation) == [expected]
    assert manipulation.set_gripper_position.call_args.args[1] == expected


def test_pick_honours_an_explicit_arm(two_arms: PickAndPlaceModule) -> None:
    manipulation: Any = two_arms._manipulation

    assert two_arms.pick_object("cup-1", planning_group=LEFT).message == "Pick complete"

    assert _planned_groups(manipulation) == [LEFT]


BEYOND_THE_RIGHT_ARM = (
    "Object cup-1 is 0.64 m from the right_manipulator shoulder (right_shoulder) and its "
    "closest pre-grasp pose needs the wrist 0.56 m out, beyond that arm's 0.44 m reach: "
    "drive closer or pass auxiliary_groups=['moving_base']."
)


def test_pick_fails_fast_on_an_object_beyond_the_arm(two_arms: PickAndPlaceModule) -> None:
    """Known before any IK: on the bench every top-down pre-grasp was kinematically
    impossible, and IK burnt seconds per candidate finding that out."""
    manipulation: Any = two_arms._manipulation
    box_filter: Any = two_arms._box_filter
    _bench_object(two_arms, 0.70, -0.20)

    result = two_arms.pick_object("cup-1")

    assert result.message == BEYOND_THE_RIGHT_ARM
    assert result.metadata["planning_group"] == RIGHT
    assert result.metadata["reach"] == 0.44
    manipulation.plan_to_poses.assert_not_called()
    manipulation.move_linear.assert_not_called()
    manipulation.set_gripper_position.assert_not_called()
    box_filter.set_box.assert_not_called()
    assert not two_arms._holding_object


def test_pick_with_the_base_along_skips_the_reach_check(two_arms: PickAndPlaceModule) -> None:
    """A base that may drive extends the arm, so the sphere about the shoulder no longer applies."""
    manipulation: Any = two_arms._manipulation
    _bench_object(two_arms, 0.70, -0.20)

    result = two_arms.pick_object("cup-1", auxiliary_groups=["moving_base"])

    assert result.message == "Pick complete"
    assert tuple(manipulation.plan_to_poses.call_args.kwargs["auxiliary_groups"]) == (
        "moving_base",
    )


def test_pick_skips_the_candidates_beyond_the_arm(two_arms: PickAndPlaceModule) -> None:
    """Of a ranked spread, only the poses the wrist can get to are planned."""
    manipulation: Any = two_arms._manipulation
    grasp_generator: Any = two_arms._grasp_generator
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"),
        [_grasp_at(0.70, -0.20, 1.10, score=0.9), _grasp_at(0.35, -0.15, 1.10, score=0.4)],
    )

    result = two_arms.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert result.metadata["rank"] == 1
    assert manipulation.plan_to_poses.call_count == 1
    planned = manipulation.plan_to_poses.call_args.args[0][RIGHT]
    assert planned.position.x == pytest.approx(0.35)


def test_pick_proceeds_when_the_shoulder_is_not_on_tf(two_arms: PickAndPlaceModule) -> None:
    """Without the shoulder's transform there is no check; the planner decides, as before."""
    tf: Any = two_arms._tf
    tf.get.side_effect = None
    tf.get.return_value = None
    _bench_object(two_arms, 0.70, -0.20)

    assert two_arms.pick_object("cup-1").message == "Pick complete"

    # Chosen by the tool tips instead.
    assert _planned_groups(two_arms._manipulation) == [RIGHT]


def test_place_uses_the_arm_holding_the_object(two_arms: PickAndPlaceModule) -> None:
    manipulation: Any = two_arms._manipulation
    assert two_arms.pick_object("cup-1").message == "Pick complete"
    assert _planned_groups(manipulation) == [RIGHT]

    # Nearer the left shoulder, yet the right hand holds the cup.
    assert two_arms.place_at(0.35, 0.10, 1.10).message == "Place complete"

    assert _planned_groups(manipulation) == [RIGHT]
    assert two_arms._holding_group is None


def test_place_fails_fast_on_a_spot_beyond_the_arm(two_arms: PickAndPlaceModule) -> None:
    manipulation: Any = two_arms._manipulation
    assert two_arms.pick_object("cup-1").message == "Pick complete"
    manipulation.plan_to_poses.reset_mock()

    result = two_arms.place_at(0.80, -0.20, 1.10)

    assert result.message == (
        "Place target (0.80, -0.20, 1.10) is 0.73 m from the right_manipulator shoulder "
        "(right_shoulder) and the pre-place pose needs the wrist 0.66 m out, beyond that "
        "arm's 0.44 m reach: drive closer or pass auxiliary_groups=['moving_base']. "
        "Still holding object cup-1."
    )
    manipulation.plan_to_poses.assert_not_called()
    assert two_arms._holding_object
