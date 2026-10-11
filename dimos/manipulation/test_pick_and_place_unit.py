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
import time
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

from dimos_lcm.vision_msgs import BoundingBox3D, ObjectHypothesis, ObjectHypothesisWithPose
import numpy as np
import pytest

from dimos.manipulation.grasp_verification import GripperSettle
from dimos.manipulation.grasping.grasp_gen_spec import SceneGrasps
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspConfig, heuristic_grasps
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
from dimos.manipulation.pick_and_place_module import (
    VOXEL_MAP_EXCLUSION_ID,
    HeldObject,
    PickAndPlaceModule,
    ScannedObject,
    _scene_near,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray

PLANNED = PlanResult(PlanStatus.SUCCEEDED)
NO_PATH = PlanResult(PlanStatus.FAILED, "unreachable")
COMPLETED = ExecutionResult(ExecutionStatus.COMPLETED)
ACCEPTED = CommandResult(CommandStatus.SUCCEEDED)


# Where the cup was scanned; the default grasp is at its centre, from above.
CUP_AT = Vector3(0.1, 0.0, 0.2)
GROUP = SimpleNamespace(id="arm/tool", has_gripper=True, tip_frame="tool")
TOP_DOWN = Quaternion.from_euler(Vector3(-3.141592653589793, 0.0, 0.0))
# Camera frames in these tests are stamped an hour ahead, so every check takes them as new.
FRAME_TS = time.time() + 3600.0


def _linear_move(execution: ExecutionResult = COMPLETED) -> MoveResult:
    return MoveResult(PLANNED, execution, (0.0, 0.0, 0.0), False)


def _detections(*found: tuple[str, str, Vector3], ts: float = FRAME_TS) -> Detection3DArray:
    """A scan result listing (object_id, label, centre) triples, from a frame taken at ``ts``."""
    detections = [
        Detection3D(
            id=object_id,
            header=Header(ts, "world"),
            results_length=1,
            results=[
                ObjectHypothesisWithPose(hypothesis=ObjectHypothesis(class_id=name, score=1.0))
            ],
            bbox=BoundingBox3D(center=Pose(center, Quaternion()), size=Vector3(0.1, 0.1, 0.1)),
        )
        for object_id, name, center in found
    ]
    return Detection3DArray(
        detections_length=len(detections), header=Header(ts, "world"), detections=detections
    )


def _joints(*position: float) -> JointState:
    """Joint positions of the test arm, radians."""
    return JointState(name=[f"j{i}" for i in range(len(position))], position=list(position))


def _group_state(
    gripper_position: float = 0.5,
    joints: JointState | None = None,
    tip: Vector3 | None = None,
) -> SimpleNamespace:
    """Manipulation state for the test arm.

    Args:
        gripper_position: Jaw readback, 0.0 closed to 1.0 open.
        joints: Joint positions the arm reports, or None for none.
        tip: Tool tip position, metres; None puts it at (0.4, 0.0, 0.2).
    """
    return SimpleNamespace(
        groups={
            "arm/tool": SimpleNamespace(
                joints=joints,
                gripper_position=gripper_position,
                end_effector_pose=PoseStamped(
                    frame_id="world",
                    position=Vector3(0.4, 0.0, 0.2) if tip is None else tip,
                    orientation=Quaternion.from_euler(Vector3(0, 0, 0.7)),
                ),
            )
        }
    )


def _holding_cup() -> HeldObject:
    return HeldObject("cup-1", "cup", PoseStamped(frame_id="world", orientation=TOP_DOWN))


def _scanned_cup(object_id: str, at: Vector3) -> ScannedObject:
    """A cup from the latest scan, with the 10 cm box ``_detections`` gives it."""
    return ScannedObject(object_id, "cup", at, Quaternion(), Vector3(0.1, 0.1, 0.1))


def _held_id(module: PickAndPlaceModule) -> str | None:
    return None if module._held is None else module._held.object_id


@pytest.fixture
def module() -> Iterator[PickAndPlaceModule]:
    # The heuristic fallback has its own tests below.
    instance = PickAndPlaceModule(planning_frame="world", heuristic_fallback=False)
    instance._scene = MagicMock()
    instance._grasp_generator = MagicMock()
    instance._manipulation = MagicMock()
    instance._manipulation.list_planning_groups.return_value = [GROUP]
    instance._manipulation.get_state.return_value = _group_state()
    instance._manipulation.plan_to_poses.return_value = PLANNED
    instance._manipulation.plan_to_joints.return_value = PLANNED
    instance._manipulation.execute.return_value = COMPLETED
    instance._manipulation.move_linear.return_value = _linear_move()
    instance._manipulation.set_gripper_position.return_value = ACCEPTED
    instance._objects = {"cup-1": _scanned_cup("cup-1", CUP_AT)}
    # After a lift or a release the camera sees no cup.
    instance._scene.scan_scene.return_value = _detections()
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


def test_scan_objects_records_ids_and_positions(module: PickAndPlaceModule) -> None:
    scene: Any = module._scene
    scene.scan_scene.return_value = _detections(("cup-1", "cup", Vector3(0.3, 0.1, 0.2)))

    result = module.scan_objects([" cup "])

    assert result.message == "Detected 1 object(s)"
    assert result.metadata["objects"] == [{"object_id": "cup-1", "name": "cup"}]
    assert module.get_object("cup-1") == {"object_id": "cup-1", "name": "cup"}
    assert module._objects["cup-1"].position == Vector3(0.3, 0.1, 0.2)
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
    assert module._held is not None
    assert module._held.grasp.position.x == pytest.approx(0.1)
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
    assert _tries(result, "source") == ["provider"]
    assert _tries(result, "rank") == [0]
    assert result.metadata["handoff"] is False
    assert manipulation.plan_to_poses.call_count == 1
    assert _held_id(module) is None


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
    assert result.metadata["rejected_by_source"] == {"provider": 2}
    assert _held_id(module) is None


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
    assert module._held is not None
    assert module._held.grasp.orientation.to_euler().z == pytest.approx(0.7)


def test_place_uses_local_axis_and_clears_held_state(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    module._held = _holding_cup()

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.message == "Place complete"
    preplace = manipulation.plan_to_poses.call_args_list[0].args[0]["arm/tool"]
    assert preplace.position.z == pytest.approx(0.3)
    assert _held_id(module) is None


def test_scan_failure_clears_stale_proposals(module: PickAndPlaceModule) -> None:
    scene: Any = module._scene
    module._grasp_candidates = GraspCandidateArray(Header(1.0, "world"), [_candidate(0.1)])
    scene.scan_scene.side_effect = RuntimeError("No aligned RGB-D frame")

    with pytest.raises(RuntimeError, match="No aligned RGB-D frame"):
        module.scan_objects(["cup"])

    assert module.get_grasp_candidates().candidates == []
    assert module.get_object("cup-1") is None


def test_pick_rejects_when_already_holding(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    module._held = HeldObject("cup-0", "cup", PoseStamped(frame_id="world"))

    pick = module.pick_object("cup-1")

    assert pick.message == (
        "Still holding object cup-0; did not start a pick of object cup-1. "
        "Use place_at to put it down first."
    )
    manipulation.set_gripper_position.assert_not_called()


def test_failed_pick_clears_previous_proposals(module: PickAndPlaceModule) -> None:
    module._grasp_candidates = GraspCandidateArray(Header(1.0, "world"), [_candidate(0.1)])

    result = module.pick_object("missing")

    assert result.message == (
        "No object with id missing in the latest scan. Scanned ids: cup-1. "
        "Use scan_objects to refresh the list."
    )
    assert module.get_grasp_candidates().candidates == []


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
    assert _held_id(module) == "cup-1"


def test_final_grasp_leg_skips_collision_checking(module: PickAndPlaceModule) -> None:
    """The target is mapped geometry, so a checked plan into it always collides."""
    manipulation: Any = module._manipulation

    assert module.pick_object("cup-1").message == "Pick complete"
    assert manipulation.move_linear.call_args_list
    for call in manipulation.move_linear.call_args_list:
        assert call.kwargs["check_collision"] is False


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
    assert result.metadata["handoff"] is False
    assert _held_id(module) is None
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

    assert _held_id(module) is None


def test_place_retains_held_state_when_release_fails(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    module._held = _holding_cup()

    def settle(read: Any, target: float, config: Any, **_: Any) -> GripperSettle:
        return GripperSettle(True, 0.5, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.message == (
        "Commanded the gripper open to release the object. Final gripper position 0.50. "
        "The arm stayed at the place pose."
    )
    assert _held_id(module) == "cup-1"


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


def _exclusions(manipulation: Any) -> list[tuple[Any, ...]]:
    """Every set_voxel_map_exclusion call as (center, size, planning_group)."""
    return [
        (c.args[1], c.args[2], c.kwargs.get("planning_group"))
        for c in manipulation.set_voxel_map_exclusion.call_args_list
    ]


def _close_on_nothing(monkeypatch: pytest.MonkeyPatch) -> None:
    def settle(read: Any, target: float, config: Any, **_: Any) -> GripperSettle:
        position = 0.0 if target == config.closed_position else target
        return GripperSettle(True, position, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)


def test_pick_hides_the_target_from_the_planner_then_carries_it(
    module: PickAndPlaceModule,
) -> None:
    """The target is mapped geometry; the approach goes into it and the lift takes it along."""
    manipulation: Any = module._manipulation
    grasp_generator: Any = module._grasp_generator
    # Off the cup's centre and tilted, so a box rotated the wrong way lands elsewhere.
    grasp = Pose(Vector3(0.1, 0.02, 0.21), Quaternion.from_euler(Vector3(-2.6, 0.0, 0.6)))
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [GraspCandidate(grasp, 1.0)]
    )

    assert module.pick_object("cup-1").message == "Pick complete"

    fixed, carried = _exclusions(manipulation)
    # The cup's bounds plus the 3 cm clearance each way, fixed where it stood.
    assert fixed[0].to_tuple() == pytest.approx((0.1, 0.0, 0.2))
    assert fixed[1].to_tuple() == pytest.approx((0.1, 0.1, 0.1))
    assert fixed[2] is None
    # Then the same box rides with the gripper: from the tip at the grasp pose,
    # its offset in the tip's frame leads back to the cup's centre.
    at = grasp.position + grasp.orientation.rotate_vector(carried[0])
    assert at.to_tuple() == pytest.approx((0.1, 0.0, 0.2), abs=1e-6)
    assert carried[1].to_tuple() == pytest.approx((0.1, 0.1, 0.1))
    assert carried[2] == "arm/tool"
    manipulation.clear_voxel_map_exclusion.assert_not_called()


def test_a_failed_pick_makes_the_target_an_obstacle_again(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    manipulation.plan_to_poses.return_value = NO_PATH

    result = module.pick_object("cup-1")

    assert result.message.startswith("The planner found no path")
    assert len(_exclusions(manipulation)) == 1
    manipulation.clear_voxel_map_exclusion.assert_called_once_with(VOXEL_MAP_EXCLUSION_ID)


def test_a_pick_that_raises_makes_the_target_an_obstacle_again(
    module: PickAndPlaceModule,
) -> None:
    manipulation: Any = module._manipulation
    manipulation.set_gripper_position.return_value = CommandResult(
        CommandStatus.FAILED, "controller unavailable"
    )

    with pytest.raises(RuntimeError, match="controller unavailable"):
        module.pick_object("cup-1")

    manipulation.clear_voxel_map_exclusion.assert_called_once_with(VOXEL_MAP_EXCLUSION_ID)


def test_release_makes_the_placed_object_an_obstacle_again(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    module._held = _holding_cup()
    order: list[str] = []

    def servo(*args: Any, **kwargs: Any) -> MoveResult:
        order.append("servo")
        return _linear_move()

    manipulation.move_linear.side_effect = servo
    manipulation.clear_voxel_map_exclusion.side_effect = lambda name: order.append("clear")

    assert module.place_at(0.4, 0.0, 0.2).message == "Place complete"

    # Down, release, back up out of the object, and only then the box goes.
    assert order == ["servo", "servo", "clear"]


def test_a_release_whose_retract_stops_keeps_the_box(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    module._held = _holding_cup()
    manipulation.move_linear.side_effect = [
        _linear_move(),
        _linear_move(ExecutionResult(ExecutionStatus.FAULT, "retract failed")),
    ]

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.message.startswith("Retract from (0.40, 0.00, 0.20) after releasing")
    manipulation.clear_voxel_map_exclusion.assert_not_called()


def test_an_empty_target_cloud_asks_for_no_exclusion(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    scene: Any = module._scene
    scene.get_object_pointcloud_by_object_id.return_value = PointCloud2.from_numpy(
        np.zeros((0, 3), dtype=np.float32), frame_id="world", timestamp=1.0
    )

    assert module.pick_object("cup-1").message == "Pick complete"

    manipulation.set_voxel_map_exclusion.assert_not_called()


def test_an_empty_close_reports_a_back_out_that_did_not_complete(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    manipulation: Any = module._manipulation
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
    assert result.metadata["handoff"] is False
    assert _held_id(module) is None
    # The gripper is still inside the cup's cells; the next plan starts from there.
    manipulation.clear_voxel_map_exclusion.assert_not_called()


def test_pick_is_complete_when_the_object_rose_with_the_gripper(
    module: PickAndPlaceModule,
) -> None:
    scene: Any = module._scene
    scene.scan_scene.return_value = _detections(("cup-9", "cup", Vector3(0.1, 0.0, 0.29)))

    result = module.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert result.metadata["grasped"] is True
    assert result.metadata["gripper_lift_m"] == pytest.approx(0.1)
    assert result.metadata["object_lift_m"] == pytest.approx(0.09)
    assert _held_id(module) == "cup-1"
    scene.scan_scene.assert_called_once_with(text=["cup"])


@pytest.mark.parametrize(
    ("seen_at", "object_lift"),
    [
        # Jaws stopped on an edge read as a hold, but the cup never moved.
        (Vector3(0.11, 0.0, 0.2), 0.0),
        # Pushed 6 cm along the table: it left its spot but did not rise.
        (Vector3(0.16, 0.0, 0.2), 0.0),
        # Knocked over: its centre dropped 3 cm.
        (Vector3(0.14, 0.0, 0.17), -0.03),
    ],
    ids=["stayed", "pushed", "tipped"],
)
def test_pick_fails_when_the_object_did_not_rise(
    module: PickAndPlaceModule, seen_at: Vector3, object_lift: float
) -> None:
    module.config.max_grasp_retries = 1
    scene: Any = module._scene
    manipulation: Any = module._manipulation
    scene.scan_scene.return_value = _detections(("cup-1", "cup", seen_at))

    result = module.pick_object("cup-1")

    assert result.message == (
        f"Lifted the gripper 0.10 m, but the camera sees object cup-1 rose {object_lift:.2f} m "
        "with it; it stayed on the table. Reopened the gripper. "
        "Object cup-1 is still not held after 1 try."
    )
    assert result.metadata["grasped"] is False
    assert result.metadata["object_lift_m"] == pytest.approx(object_lift)
    assert result.metadata["rank"] == 0
    assert _held_id(module) is None
    assert manipulation.set_gripper_position.call_args_list[-1].args[0] == 1.0
    manipulation.clear_voxel_map_exclusion.assert_called_once_with(VOXEL_MAP_EXCLUSION_ID)


def test_pick_ignores_other_objects_with_the_same_label(module: PickAndPlaceModule) -> None:
    """A second cup 40 cm away is not the one picked; the picked cup is out of view."""
    scene: Any = module._scene
    scene.scan_scene.return_value = _detections(("cup-2", "cup", Vector3(0.5, 0.0, 0.2)))

    result = module.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert result.metadata["object_lift_m"] is None


def test_pick_counts_the_detection_that_came_along(module: PickAndPlaceModule) -> None:
    """Something 10 cm away stays on the table while the picked cup rides up."""
    scene: Any = module._scene
    scene.scan_scene.return_value = _detections(
        ("cup-5", "cup", Vector3(0.2, 0.0, 0.2)), ("cup-9", "cup", Vector3(0.1, 0.0, 0.3))
    )

    result = module.pick_object("cup-1")

    assert result.metadata["grasped"] is True
    assert result.metadata["object_lift_m"] == pytest.approx(0.1)


def test_pick_fails_when_the_jaws_empty_during_the_lift(module: PickAndPlaceModule) -> None:
    module.config.max_grasp_retries = 1
    manipulation: Any = module._manipulation
    scene: Any = module._scene
    manipulation.get_state.return_value = _group_state(gripper_position=0.02)

    result = module.pick_object("cup-1")

    assert result.message == (
        "After lifting object cup-1, the gripper reads 0.02: the jaws are closed on nothing. "
        "Reopened the gripper. Object cup-1 is still not held after 1 try."
    )
    assert result.metadata["grasped"] is False
    assert result.metadata["jaws"] == pytest.approx(0.02)
    assert _held_id(module) is None
    scene.scan_scene.assert_not_called()


def test_place_refuses_without_a_held_object(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.message == (
        "Not holding any object; nothing was placed at (0.40, 0.00, 0.20). Use pick_object first."
    )
    assert result.metadata == {"placed": False}
    manipulation.plan_to_poses.assert_not_called()


def test_place_is_complete_when_the_object_rests_inside_the_tolerance(
    module: PickAndPlaceModule,
) -> None:
    scene: Any = module._scene
    module._held = _holding_cup()
    scene.scan_scene.return_value = _detections(("cup-1", "cup", Vector3(0.43, 0.04, 0.15)))

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.message == "Place complete"
    assert result.metadata["placed"] is True
    assert result.metadata["offset_m"] == pytest.approx(0.05)
    assert result.metadata["located_by"] == "camera"
    scene.scan_scene.assert_called_once_with(text=["cup"])


def test_place_reports_an_object_outside_the_tolerance(module: PickAndPlaceModule) -> None:
    scene: Any = module._scene
    module._held = _holding_cup()
    scene.scan_scene.return_value = _detections(("cup-1", "cup", Vector3(0.5, 0.0, 0.15)))

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.message == (
        "Released object cup-1; it rests 0.10 m from the target, "
        "farther than the 0.07 m place tolerance."
    )
    assert result.metadata["placed"] is False
    assert _held_id(module) is None


def test_place_falls_back_to_the_release_pose_when_the_object_is_hidden(
    module: PickAndPlaceModule,
) -> None:
    """Right under the gripper the fingers can hide the object from the wrist camera."""
    module._held = _holding_cup()

    result = module.place_at(0.42, 0.0, 0.2)

    assert result.message == "Place complete"
    assert result.metadata["located_by"] == "gripper"
    assert result.metadata["offset_m"] == pytest.approx(0.02)


def test_place_reports_an_object_lost_on_the_way(module: PickAndPlaceModule) -> None:
    manipulation: Any = module._manipulation
    module._held = _holding_cup()
    manipulation.get_state.return_value = _group_state(gripper_position=0.02)

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.message == (
        "The gripper reads 0.02: object cup-1 is no longer in the jaws, so nothing was placed "
        "at (0.40, 0.00, 0.20). Reopened the gripper."
    )
    assert result.metadata["placed"] is False
    assert _held_id(module) is None
    manipulation.plan_to_poses.assert_not_called()
    manipulation.clear_voxel_map_exclusion.assert_called_once_with(VOXEL_MAP_EXCLUSION_ID)
    assert manipulation.set_gripper_position.call_args_list[-1].args[0] == 1.0


def test_pick_ignores_a_neighbour_from_the_latest_scan(module: PickAndPlaceModule) -> None:
    """The wrist camera rarely sees the held cup; a known neighbour must not fail the pick."""
    scene: Any = module._scene
    module._objects["cup-2"] = _scanned_cup("cup-2", Vector3(0.2, 0.0, 0.2))
    scene.scan_scene.return_value = _detections(("cup-2", "cup", Vector3(0.2, 0.0, 0.2)))

    result = module.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert result.metadata["object_lift_m"] is None


def test_pick_believes_the_target_s_own_id_over_a_new_one(module: PickAndPlaceModule) -> None:
    """The target kept its ID at its old spot, so the new cup by the gripper is another one."""
    scene: Any = module._scene
    scene.scan_scene.return_value = _detections(
        ("cup-1", "cup", Vector3(0.1, 0.0, 0.2)), ("cup-9", "cup", Vector3(0.1, 0.0, 0.3))
    )

    result = module.pick_object("cup-1")

    assert result.metadata["grasped"] is False
    assert result.metadata["object_lift_m"] == pytest.approx(0.0)


def test_a_close_still_moving_on_something_is_checked_after_the_lift(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Jaws still squeezing a soft object have not settled, but they may hold it."""
    scene: Any = module._scene

    def settle(read: Any, target: float, config: Any, **_: Any) -> GripperSettle:
        if target == config.closed_position:
            return GripperSettle(False, 0.5, True, config.timeout)
        return GripperSettle(True, target, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)
    scene.scan_scene.return_value = _detections(("cup-9", "cup", Vector3(0.1, 0.0, 0.29)))

    result = module.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert _held_id(module) == "cup-1"
    scene.scan_scene.assert_called_once_with(text=["cup"])


def test_the_lift_check_waits_for_a_frame_taken_after_the_lift(
    module: PickAndPlaceModule,
) -> None:
    """A frame from before the lift still shows the cup where it was."""
    scene: Any = module._scene
    scene.scan_scene.side_effect = [
        _detections(("cup-1", "cup", CUP_AT), ts=1.0),
        _detections(("cup-9", "cup", Vector3(0.1, 0.0, 0.3))),
    ]

    result = module.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert scene.scan_scene.call_count == 2


def test_without_a_new_frame_the_lift_check_trusts_the_jaws(module: PickAndPlaceModule) -> None:
    scene: Any = module._scene
    module.config.fresh_frame_timeout = 0.1
    scene.scan_scene.return_value = _detections(("cup-1", "cup", CUP_AT), ts=1.0)

    result = module.pick_object("cup-1")

    assert result.message == (
        "Pick complete on the jaw reading alone: the camera gave no frame taken after the arm "
        "stopped within 0.1 s."
    )
    assert result.metadata["grasped"] is True
    assert result.metadata["object_lift_m"] is None
    assert _held_id(module) == "cup-1"


def test_without_a_new_frame_the_re_scan_finds_nothing(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    scene: Any = module._scene
    module.config.fresh_frame_timeout = 0.1
    scene.scan_scene.return_value = _detections(("cup-1", "cup", CUP_AT), ts=1.0)
    _close_on_nothing(monkeypatch)

    result = module.pick_object("cup-1")

    assert result.message.endswith(
        "A new scan for cup found nothing: the camera gave no frame taken after the arm "
        "stopped within 0.1 s."
    )
    assert result.metadata["handoff"] is True
    assert _held_id(module) is None


def test_without_a_new_frame_the_place_check_uses_the_release_pose(
    module: PickAndPlaceModule,
) -> None:
    scene: Any = module._scene
    module._held = _holding_cup()
    module.config.fresh_frame_timeout = 0.1
    scene.scan_scene.return_value = _detections(ts=1.0)

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.metadata["located_by"] == "gripper"


def test_place_ignores_detections_far_from_the_release(module: PickAndPlaceModule) -> None:
    """Another cup 30 cm from where the gripper let go is not the one placed."""
    scene: Any = module._scene
    module._held = _holding_cup()
    scene.scan_scene.return_value = _detections(("cup-2", "cup", Vector3(0.7, 0.0, 0.15)))

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.message == "Place complete"
    assert result.metadata["located_by"] == "gripper"


def _cup_at(x: float, y: float, z: float, object_id: str = "cup-1") -> Detection3DArray:
    return _detections((object_id, "cup", Vector3(x, y, z)))


def _grasp_at(
    x: float, y: float, score: float, orientation: Quaternion = TOP_DOWN
) -> GraspCandidate:
    return GraspCandidate(Pose(Vector3(x, y, 0.2), orientation), score)


def _tries(result: Any, key: str) -> list[Any]:
    return [record[key] for record in result.metadata["tries"]]


def test_a_first_try_success_records_one_try(module: PickAndPlaceModule) -> None:
    result = module.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert result.metadata["tries"] == [
        {
            "try": 1,
            "object_id": "cup-1",
            "source": "provider",
            "rank": 0,
            "score": 1.0,
            "skipped": 0,
            "grasped": True,
            "candidates_by_source": {"provider": 1},
            "scene_filtered_by_source": {},
            "rejected_by_source": {"provider": 0},
        }
    ]
    assert result.metadata["handoff"] is False
    assert result.metadata["rejected_by_source"] == {"provider": 0}
    assert result.metadata["scene_filtered_by_source"] == {}


def test_a_missed_grasp_is_retried_with_the_next_candidate(module: PickAndPlaceModule) -> None:
    scene: Any = module._scene
    manipulation: Any = module._manipulation
    grasp_generator: Any = module._grasp_generator
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [_grasp_at(0.1, 0.0, 0.9), _grasp_at(0.1, 0.03, 0.5)]
    )
    # Lift check (still on the table), re-scan, lift check (rode up with the gripper).
    scene.scan_scene.side_effect = [
        _cup_at(0.1, 0.0, 0.2),
        _cup_at(0.1, 0.0, 0.2, object_id="cup-7"),
        _cup_at(0.1, 0.03, 0.3, object_id="cup-7"),
    ]

    result = module.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert _tries(result, "rank") == [0, 1]
    assert _tries(result, "skipped") == [0, 1]
    assert _tries(result, "grasped") == [False, True]
    assert _tries(result, "object_id") == ["cup-1", "cup-7"]
    assert result.metadata["handoff"] is False
    assert _held_id(module) == "cup-7"
    assert module.get_object("cup-7") == {"object_id": "cup-7", "name": "cup"}
    assert grasp_generator.propose_grasps.call_count == 2
    # Box on the table, carried, re-centred for the second approach, carried again.
    assert [group for _, _, group in _exclusions(manipulation)] == [
        None,
        "arm/tool",
        None,
        "arm/tool",
    ]
    manipulation.clear_voxel_map_exclusion.assert_not_called()


def test_empty_closes_back_out_before_the_re_scan_and_the_box_goes_last(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    scene: Any = module._scene
    manipulation: Any = module._manipulation
    grasp_generator: Any = module._grasp_generator
    module.config.max_grasp_retries = 2
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [_grasp_at(0.1, 0.0, 0.9), _grasp_at(0.1, 0.05, 0.5)]
    )
    _close_on_nothing(monkeypatch)
    order: list[str] = []

    def servo(*args: Any, **kwargs: Any) -> MoveResult:
        order.append("servo")
        return _linear_move()

    def gripper(position: float, group: str) -> CommandResult:
        order.append(f"gripper {position:.0f}")
        return ACCEPTED

    def scan(text: list[str]) -> Detection3DArray:
        order.append("scan")
        return _cup_at(0.1, 0.0, 0.2)

    manipulation.move_linear.side_effect = servo
    manipulation.set_gripper_position.side_effect = gripper
    manipulation.set_voxel_map_exclusion.side_effect = lambda *a, **k: order.append("box")
    manipulation.clear_voxel_map_exclusion.side_effect = lambda name: order.append("clear")
    scene.scan_scene.side_effect = scan

    result = module.pick_object("cup-1")

    one_try = ["gripper 1", "servo", "gripper 0", "gripper 1", "servo"]
    # Open, down, close on nothing, reopen, back out; look again, re-centre the
    # box, try again; the box goes only once the arm is out the last time.
    assert order == ["box", *one_try, "scan", "box", *one_try, "clear"]
    down, back_out = (c.args[2] for c in manipulation.move_linear.call_args_list[:2])
    assert down == pytest.approx(-back_out)
    assert result.message == (
        "Closed the gripper on object cup-1. Final gripper position 0.00. Reopened the gripper. "
        "Object cup-1 is still not held after 2 tries."
    )
    assert result.metadata["grasped"] is False
    assert result.metadata["jaws"] == 0.0
    assert result.metadata["handoff"] is True
    assert _held_id(module) is None


def test_three_missed_grasps_hand_the_pick_off(module: PickAndPlaceModule) -> None:
    scene: Any = module._scene
    manipulation: Any = module._manipulation
    grasp_generator: Any = module._grasp_generator
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"),
        [_grasp_at(0.1, 0.0, 0.9), _grasp_at(0.1, 0.05, 0.6), _grasp_at(0.1, -0.05, 0.3)],
    )
    scene.scan_scene.return_value = _cup_at(0.1, 0.0, 0.2)

    result = module.pick_object("cup-1")

    assert result.message.endswith("Object cup-1 is still not held after 3 tries.")
    assert result.metadata["handoff"] is True
    assert result.metadata["grasped"] is False
    assert _tries(result, "try") == [1, 2, 3]
    assert _tries(result, "rank") == [0, 1, 2]
    assert _tries(result, "score") == [0.9, 0.6, 0.3]
    assert _tries(result, "grasped") == [False, False, False]
    assert _held_id(module) is None
    assert manipulation.set_gripper_position.call_args_list[-1].args[0] == 1.0
    manipulation.clear_voxel_map_exclusion.assert_called_once_with(VOXEL_MAP_EXCLUSION_ID)
    # Three lift checks and the two re-scans between them.
    assert scene.scan_scene.call_count == 5


def test_a_re_scan_that_loses_the_object_hands_the_pick_off(module: PickAndPlaceModule) -> None:
    scene: Any = module._scene
    grasp_generator: Any = module._grasp_generator
    # Lift check: still on the table. Re-scan: the only cup is 40 cm away.
    scene.scan_scene.side_effect = [_cup_at(0.1, 0.0, 0.2), _cup_at(0.5, 0.0, 0.2, "cup-2")]

    result = module.pick_object("cup-1")

    assert result.message.endswith(
        "A new scan for cup found none within 0.15 m of where object cup-1 was."
    )
    assert result.metadata["handoff"] is True
    assert _tries(result, "grasped") == [False]
    assert grasp_generator.propose_grasps.call_count == 1


def test_a_re_scan_does_not_retarget_a_neighbour_from_the_latest_scan(
    module: PickAndPlaceModule,
) -> None:
    """The neighbour 5 cm away kept its own ID; picking it would report the wrong object."""
    scene: Any = module._scene
    module._objects["cup-2"] = _scanned_cup("cup-2", Vector3(0.15, 0.0, 0.2))
    # Lift check: the cup stayed. Re-scan: only the neighbour is seen.
    scene.scan_scene.side_effect = [
        _cup_at(0.1, 0.0, 0.2),
        _cup_at(0.15, 0.0, 0.2, "cup-2"),
    ]

    result = module.pick_object("cup-1")

    assert result.metadata["handoff"] is True
    assert result.message.endswith("found none within 0.15 m of where object cup-1 was.")


def test_a_retry_proposes_grasps_from_where_the_object_is_now(
    module: PickAndPlaceModule,
) -> None:
    """The scene merges the new view into the old cloud; only the new place is grasped."""
    scene: Any = module._scene
    grasp_generator: Any = module._grasp_generator
    grasp_generator.propose_grasps.side_effect = [
        GraspCandidateArray(Header(1.0, "world"), [_grasp_at(0.1, 0.0, 0.9)]),
        GraspCandidateArray(Header(1.0, "world"), [_grasp_at(0.1, 0.13, 0.9)]),
    ]
    # Points where the cup stood and where it was pushed to, 10 cm along y.
    scene.get_object_pointcloud_by_object_id.return_value = PointCloud2.from_numpy(
        np.asarray(
            [[0.1, 0.0, 0.2], [0.1, 0.01, 0.21], [0.1, 0.1, 0.2], [0.1, 0.11, 0.21]],
            dtype=np.float32,
        ),
        frame_id="world",
        timestamp=1.0,
    )
    scene.scan_scene.side_effect = [
        _cup_at(0.1, 0.1, 0.2),  # lift check: pushed aside, not lifted
        _cup_at(0.1, 0.1, 0.2),  # re-scan
        _cup_at(0.1, 0.1, 0.3),  # lift check: rode up
    ]

    result = module.pick_object("cup-1")

    assert result.message == "Pick complete"
    retry_cloud = grasp_generator.propose_grasps.call_args_list[1].args[0]
    assert all(y > 0.05 for y in retry_cloud.points_f32()[:, 1])
    assert len(retry_cloud.points_f32()) == 2


def test_a_retry_skips_candidates_that_repeat_a_tried_grasp(module: PickAndPlaceModule) -> None:
    """The cup was pushed 4 cm; grasps that match the tried one on the cup are skipped."""
    scene: Any = module._scene
    grasp_generator: Any = module._grasp_generator
    flipped = Quaternion.from_euler(Vector3(-3.141592653589793, 0.0, 3.141592653589793))

    def proposals(dx: float) -> GraspCandidateArray:
        return GraspCandidateArray(
            Header(1.0, "world"),
            [
                _grasp_at(0.1 + dx, 0.0, 0.9),  # the grasp tried first
                _grasp_at(0.1 + dx, 0.0, 0.8, flipped),  # the same, turned half a turn
                _grasp_at(0.11 + dx, 0.0, 0.7),  # 1 cm along
                _grasp_at(0.1 + dx, 0.05, 0.5),  # 5 cm along: a new grasp
            ],
        )

    grasp_generator.propose_grasps.side_effect = [proposals(0.0), proposals(0.04)]
    scene.scan_scene.side_effect = [
        _cup_at(0.14, 0.0, 0.2),  # lift check: pushed, not lifted
        _cup_at(0.14, 0.0, 0.2),  # re-scan
        _cup_at(0.14, 0.05, 0.3),  # lift check: rode up
    ]

    result = module.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert _tries(result, "rank") == [0, 3]
    assert _tries(result, "skipped") == [0, 3]


def test_a_retry_with_only_repeated_candidates_hands_the_pick_off(
    module: PickAndPlaceModule,
) -> None:
    scene: Any = module._scene
    manipulation: Any = module._manipulation
    scene.scan_scene.return_value = _cup_at(0.1, 0.0, 0.2)

    result = module.pick_object("cup-1")

    assert result.message.endswith(
        "Every new grasp candidate for object cup-1 repeats a grasp already tried."
    )
    assert result.metadata["handoff"] is True
    assert len(result.metadata["tries"]) == 1
    assert manipulation.plan_to_poses.call_count == 1


def test_a_gripper_that_never_settles_is_a_fault_not_a_handoff(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Jaws stuck at open did not grasp anything; trying again would not help."""
    manipulation: Any = module._manipulation

    def settle(read: Any, target: float, config: Any, **_: Any) -> GripperSettle:
        if target == config.closed_position:
            return GripperSettle(False, 0.98, False, config.timeout)
        return GripperSettle(True, target, True, 0.1)

    monkeypatch.setattr("dimos.manipulation.pick_and_place_module.await_gripper_settle", settle)

    result = module.pick_object("cup-1")

    assert result.message.startswith("Closed the gripper on object cup-1.")
    assert result.metadata["handoff"] is False
    assert len(result.metadata["tries"]) == 1
    assert manipulation.plan_to_poses.call_count == 1


# The scan pose's joints, and where the tool tip is there: 30 cm above the grasp.
SCAN_JOINTS = _joints(0.0, -0.2, 0.9)
SCAN_TIP = Vector3(0.1, 0.0, 0.5)


class _Arm:
    """A mock arm whose joints, tool tip and jaws follow pick_object's moves.

    It scans the cup from the scan pose. A move to a pose puts the tip there, a
    straight move shifts it, and a move to joint positions brings the arm back
    to the scan pose. ``order`` records scans, straight moves (by their height
    change, metres) and moves back to the scan pose.
    """

    def __init__(self, module: PickAndPlaceModule) -> None:
        self.tip = SCAN_TIP
        self.joints = SCAN_JOINTS
        self.jaws = 0.5
        # Jaw readback once the arm reaches the scan pose; None leaves it as it is.
        self.jaws_at_scan_pose: float | None = None
        self.order: list[str] = []
        self._manipulation: Any = module._manipulation
        self._scene: Any = module._scene
        self._manipulation.get_state.side_effect = lambda: _group_state(
            self.jaws, self.joints, self.tip
        )
        self._manipulation.plan_to_poses.side_effect = self._to_pose
        self._manipulation.move_linear.side_effect = self._straight
        self._manipulation.plan_to_joints.side_effect = self._to_scan_pose
        self._scene.scan_scene.return_value = _cup_at(0.1, 0.0, 0.2)
        self._scene.scan_scene.side_effect = self._scan
        module.scan_objects(["cup"])

    def _to_pose(self, targets: dict[str, PoseStamped]) -> PlanResult:
        (pose,) = targets.values()
        self.tip, self.joints = pose.position, _joints(0.3, 0.5, 1.2)
        return PLANNED

    def _straight(self, dx: float, dy: float, dz: float, *args: Any, **kwargs: Any) -> MoveResult:
        self.order.append(f"line {dz:+.2f}")
        self.tip = self.tip + Vector3(dx, dy, dz)
        return _linear_move()

    def _to_scan_pose(self, targets: dict[str, JointState]) -> PlanResult:
        self.order.append("back to scan pose")
        self.tip, self.joints = SCAN_TIP, targets["arm/tool"]
        if self.jaws_at_scan_pose is not None:
            self.jaws = self.jaws_at_scan_pose
        return PLANNED

    def _scan(self, text: list[str]) -> Detection3DArray:
        self.order.append("scan")
        result: Detection3DArray = self._scene.scan_scene.return_value
        return result


# Down to the grasp, back out to the pre-grasp pose, then on to the scan pose.
ONE_CARRY = ["line -0.10", "line +0.10", "back to scan pose"]


def test_a_cup_left_on_the_table_is_seen_from_the_scan_pose(module: PickAndPlaceModule) -> None:
    """At the pre-grasp pose the camera may be too close to see the cup at all."""
    manipulation: Any = module._manipulation
    arm = _Arm(module)
    module.config.max_grasp_retries = 1

    result = module.pick_object("cup-1")

    assert "it stayed on the table" in result.message
    assert result.metadata["grasped"] is False
    assert result.metadata["gripper_lift_m"] == pytest.approx(0.3)
    assert arm.order == ["scan", *ONE_CARRY, "scan"]
    manipulation.plan_to_joints.assert_called_once_with({"arm/tool": SCAN_JOINTS})


def test_a_cup_carried_to_the_scan_pose_completes_the_pick(module: PickAndPlaceModule) -> None:
    scene: Any = module._scene
    _Arm(module)
    # In the jaws the cup is closer to the wrist camera than it can see.
    scene.scan_scene.return_value = _detections()

    result = module.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert result.metadata["gripper_lift_m"] == pytest.approx(0.3)
    assert result.metadata["object_lift_m"] is None
    assert _held_id(module) == "cup-1"


def test_a_failed_move_to_the_scan_pose_falls_back_to_the_jaws(
    module: PickAndPlaceModule,
) -> None:
    manipulation: Any = module._manipulation
    scene: Any = module._scene
    _Arm(module)
    manipulation.plan_to_joints.side_effect = lambda targets: NO_PATH
    scene.scan_scene.return_value = _detections()

    result = module.pick_object("cup-1")

    assert result.message == (
        "Pick complete. Move to the scan pose to look for object cup-1 did not complete; "
        "planner returned FAILED: unreachable; the camera looked from where the arm stopped."
    )
    assert result.metadata["grasped"] is True
    # The gripper stayed at the pre-grasp pose, 10 cm above the grasp.
    assert result.metadata["gripper_lift_m"] == pytest.approx(0.1)
    assert _held_id(module) == "cup-1"


def test_a_cup_that_falls_out_on_the_way_up_is_a_miss(module: PickAndPlaceModule) -> None:
    arm = _Arm(module)
    arm.jaws_at_scan_pose = 0.0
    module.config.max_grasp_retries = 1

    result = module.pick_object("cup-1")

    assert result.message == (
        "After carrying object cup-1 up, the gripper reads 0.00: the object fell out. "
        "Reopened the gripper. Object cup-1 is still not held after 1 try."
    )
    assert result.metadata["grasped"] is False
    assert _held_id(module) is None
    assert arm.order == ["scan", *ONE_CARRY]


def test_a_cup_pushed_along_a_tilted_approach_did_not_rise(module: PickAndPlaceModule) -> None:
    """Backing out of a tilted grasp moves the gripper 8 cm sideways and 5 cm up."""
    scene: Any = module._scene
    grasp_generator: Any = module._grasp_generator
    tilted = Quaternion.from_euler(Vector3(-math.pi + 1.0, 0.0, 0.0))
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [_grasp_at(0.1, 0.0, 1.0, tilted)]
    )
    # Pushed 10 cm the way the gripper backed out, still on the table.
    scene.scan_scene.return_value = _cup_at(0.1, -0.1, 0.2)
    module.config.max_grasp_retries = 1

    result = module.pick_object("cup-1")

    assert result.metadata["grasped"] is False
    assert result.metadata["object_lift_m"] == pytest.approx(0.0)


def test_after_a_miss_seen_from_the_scan_pose_the_re_scan_looks_from_there(
    module: PickAndPlaceModule,
) -> None:
    grasp_generator: Any = module._grasp_generator
    grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [_grasp_at(0.1, 0.0, 0.9), _grasp_at(0.1, 0.05, 0.5)]
    )
    arm = _Arm(module)
    module.config.max_grasp_retries = 2

    result = module.pick_object("cup-1")

    assert _tries(result, "rank") == [0, 1]
    # Each lift check looks from the scan pose; the re-scan after the first miss
    # finds the arm already there.
    assert arm.order == ["scan", *ONE_CARRY, "scan", "scan", *ONE_CARRY, "scan"]


def test_after_jaws_closed_on_nothing_the_re_scan_looks_from_the_scan_pose(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    arm = _Arm(module)
    module.config.max_grasp_retries = 2
    _close_on_nothing(monkeypatch)

    module.pick_object("cup-1")

    assert arm.order == ["scan", "line -0.10", "line +0.10", "back to scan pose", "scan"]


def test_a_pick_that_cannot_return_to_the_scan_pose_stops(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    manipulation: Any = module._manipulation
    arm = _Arm(module)
    manipulation.plan_to_joints.side_effect = lambda targets: NO_PATH
    _close_on_nothing(monkeypatch)

    result = module.pick_object("cup-1")

    assert result.message.endswith(
        "Move back to where object cup-1 was scanned from did not complete; "
        "planner returned FAILED: unreachable"
    )
    assert result.metadata["grasped"] is False
    assert result.metadata["handoff"] is False
    assert len(result.metadata["tries"]) == 1
    # Only the first scan; no re-scan from the pre-grasp pose.
    assert arm.order.count("scan") == 1


def test_without_a_scan_pose_both_checks_look_from_where_the_arm_is(
    module: PickAndPlaceModule,
) -> None:
    scene: Any = module._scene
    manipulation: Any = module._manipulation
    scene.scan_scene.return_value = _cup_at(0.1, 0.0, 0.2)
    module.scan_objects(["cup"])

    result = module.pick_object("cup-1")

    manipulation.plan_to_joints.assert_not_called()
    assert result.metadata["gripper_lift_m"] == pytest.approx(0.1)
    # The scan, the lift check and the re-scan.
    assert scene.scan_scene.call_count == 3


def _block_cloud() -> PointCloud2:
    """A 6 cm by 4 cm block, 5 cm tall, centred on the cup: two heuristic grasps,
    the centre one and the same turned a quarter turn."""
    bounds = ((0.07, 0.13), (-0.02, 0.02), (0.175, 0.225))
    axes = [np.arange(low, high + 1e-9, 0.005) for low, high in bounds]
    points = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)
    return PointCloud2.from_numpy(points.astype(np.float32), frame_id="world", timestamp=1.0)


@pytest.fixture
def fallback(module: PickAndPlaceModule) -> PickAndPlaceModule:
    """The module with the heuristic fallback on, GraspGenX as the provider and a real
    cloud. GraspGenX's one grasp is 5 cm off the heuristic centre grasp."""
    module.config.heuristic_fallback = True
    module._grasp_generator.remote_name = "GraspGenXModule"
    module._grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [_grasp_at(0.1, 0.05, 0.9)]
    )
    module._scene.get_object_pointcloud_by_object_id.return_value = _block_cloud()
    return module


def test_provider_grasps_are_tried_before_heuristic_grasps(fallback: PickAndPlaceModule) -> None:
    result = fallback.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert result.metadata["source"] == "graspgenx"
    assert result.metadata["grasp_sources"] == ["graspgenx", "heuristic"]
    assert _tries(result, "source") == ["graspgenx"]
    assert _tries(result, "candidates_by_source") == [{"graspgenx": 1, "heuristic": 2}]
    assert [c.pose.position.y for c in fallback.get_grasp_candidates().candidates] == (
        pytest.approx([0.05, 0.0, 0.0], abs=0.003)
    )


def test_heuristic_grasps_are_used_when_the_provider_returns_none(
    fallback: PickAndPlaceModule,
) -> None:
    fallback._grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), []
    )

    result = fallback.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert _tries(result, "source") == ["heuristic"]
    assert _tries(result, "rank") == [0]
    assert _tries(result, "candidates_by_source") == [{"graspgenx": 0, "heuristic": 2}]


def test_heuristic_grasps_are_used_when_the_provider_raises(fallback: PickAndPlaceModule) -> None:
    fallback._grasp_generator.propose_grasps.side_effect = RuntimeError("model not loaded")

    result = fallback.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert _tries(result, "source") == ["heuristic"]
    assert _tries(result, "grasp_provider_error") == ["RuntimeError: model not loaded"]


def test_the_result_says_so_when_the_provider_raises_and_there_is_no_grasp(
    fallback: PickAndPlaceModule,
) -> None:
    fallback._grasp_generator.propose_grasps.side_effect = RuntimeError("model not loaded")
    # Two points: too few for a heuristic grasp.
    fallback._scene.get_object_pointcloud_by_object_id.return_value = PointCloud2.from_numpy(
        np.asarray([[0.08, -0.02, 0.18], [0.12, 0.02, 0.22]], dtype=np.float32),
        frame_id="world",
        timestamp=1.0,
    )

    result = fallback.pick_object("cup-1")

    assert result.message == (
        "Generated 0 grasp candidates for object cup-1. "
        "The grasp provider graspgenx failed: RuntimeError: model not loaded."
    )
    assert result.metadata["grasp_provider_error"] == "RuntimeError: model not loaded"


def test_heuristic_grasps_are_used_when_no_provider_grasp_is_reachable(
    fallback: PickAndPlaceModule,
) -> None:
    fallback._manipulation.plan_to_poses.side_effect = [NO_PATH, PLANNED]

    result = fallback.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert _tries(result, "source") == ["heuristic"]
    assert result.metadata["rejected_by_source"] == {"graspgenx": 1, "heuristic": 0}


def test_twenty_five_provider_grasps_are_tried_before_the_heuristic_ones(
    fallback: PickAndPlaceModule,
) -> None:
    fallback._grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), [_grasp_at(0.1 + 0.01 * i, 0.05, 0.9) for i in range(30)]
    )
    fallback._manipulation.plan_to_poses.side_effect = [NO_PATH] * 25 + [PLANNED]

    result = fallback.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert _tries(result, "source") == ["heuristic"]
    assert result.metadata["rejected_by_source"] == {"graspgenx": 25, "heuristic": 0}


def test_planner_rejections_are_counted_per_try_and_for_the_whole_pick(
    fallback: PickAndPlaceModule,
) -> None:
    scene: Any = fallback._scene
    scene.scan_scene.side_effect = [
        _cup_at(0.1, 0.0, 0.2),  # lift check: still on the table
        _cup_at(0.1, 0.0, 0.2, object_id="cup-7"),  # re-scan
        _cup_at(0.1, 0.0, 0.3, object_id="cup-7"),  # lift check: rode up
    ]
    # Each try: no path to the GraspGenX grasp, then a heuristic grasp.
    fallback._manipulation.plan_to_poses.side_effect = [NO_PATH, PLANNED, NO_PATH, PLANNED]

    result = fallback.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert _tries(result, "source") == ["heuristic", "heuristic"]
    assert _tries(result, "rejected_by_source") == [{"graspgenx": 1, "heuristic": 0}] * 2
    assert result.metadata["rejected_by_source"] == {"graspgenx": 2, "heuristic": 0}


def test_heuristic_grasps_are_tried_once_the_provider_grasps_miss(
    fallback: PickAndPlaceModule,
) -> None:
    """Each re-scan proposes both sources again; a grasp tried from either is skipped."""
    scene: Any = fallback._scene
    scene.scan_scene.side_effect = [
        _cup_at(0.1, 0.0, 0.2),  # lift check: still on the table
        _cup_at(0.1, 0.0, 0.2, object_id="cup-7"),  # re-scan
        _cup_at(0.1, 0.0, 0.2, object_id="cup-7"),  # lift check: still on the table
        _cup_at(0.1, 0.0, 0.2, object_id="cup-7"),  # re-scan
        _cup_at(0.1, 0.0, 0.3, object_id="cup-7"),  # lift check: rode up
    ]

    result = fallback.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert _tries(result, "source") == ["graspgenx", "heuristic", "heuristic"]
    assert _tries(result, "rank") == [0, 0, 1]
    assert _tries(result, "skipped") == [0, 1, 2]
    assert _tries(result, "grasped") == [False, False, True]


def test_a_provider_that_is_the_heuristic_gets_no_second_copy(
    fallback: PickAndPlaceModule,
) -> None:
    fallback._grasp_generator.remote_name = "HeuristicGraspModule"
    fallback._grasp_generator.propose_grasps.return_value = heuristic_grasps(
        _block_cloud(), HeuristicGraspConfig()
    )

    result = fallback.pick_object("cup-1")

    assert result.metadata["grasp_sources"] == ["heuristic"]
    assert _tries(result, "candidates_by_source") == [{"heuristic": 2}]


def test_heuristic_grasps_that_repeat_a_provider_grasp_are_not_tried(
    fallback: PickAndPlaceModule,
) -> None:
    """A provider under another name that returns the heuristic's own grasps."""
    fallback._grasp_generator.propose_grasps.return_value = heuristic_grasps(
        _block_cloud(), HeuristicGraspConfig()
    )
    fallback._manipulation.plan_to_poses.return_value = NO_PATH

    result = fallback.pick_object("cup-1")

    assert result.message.startswith("The planner found no path to any of the 2 grasp")
    assert fallback._manipulation.plan_to_poses.call_count == 2


def test_with_the_fallback_off_only_provider_grasps_are_tried(
    fallback: PickAndPlaceModule,
) -> None:
    fallback.config.heuristic_fallback = False
    fallback._grasp_generator.propose_grasps.return_value = GraspCandidateArray(
        Header(1.0, "world"), []
    )

    result = fallback.pick_object("cup-1")

    assert result.message == "Generated 0 grasp candidates for object cup-1."
    assert result.metadata["grasp_sources"] == ["graspgenx"]
    fallback._grasp_generator.propose_grasps.side_effect = RuntimeError("model not loaded")
    with pytest.raises(RuntimeError, match="model not loaded"):
        fallback.pick_object("cup-1")


def test_every_pick_result_names_the_grasp_sources(fallback: PickAndPlaceModule) -> None:
    result = fallback.pick_object("missing")

    assert result.metadata["grasp_sources"] == ["graspgenx", "heuristic"]


def _points(*xyz: tuple[float, float, float]) -> PointCloud2:
    return PointCloud2.from_numpy(
        np.asarray(xyz, dtype=np.float32), frame_id="world", timestamp=2.0
    )


def _table() -> PointCloud2:
    """Camera points of a table top at the block's base (z = 0.175), 2 cm apart, 1 m square."""
    xs = np.arange(-0.4, 0.6, 0.02)
    grid = np.stack(np.meshgrid(xs, xs, [0.175], indexing="ij"), axis=-1).reshape(-1, 3)
    return PointCloud2.from_numpy(grid.astype(np.float32), frame_id="world", timestamp=2.0)


def test_the_scene_near_the_target_keeps_the_table_and_drops_the_target_and_the_gripper() -> None:
    tip = Vector3(0.1, 0.0, 0.36)
    scene = _points(
        (0.15, 0.0, 0.175),  # the table 2 cm beside the block: kept
        (0.1, 0.0, 0.17),  # the table touching the block's base: dropped
        (0.07, 0.0, 0.2),  # on the block's face, missed by its mask: dropped
        (0.2, 0.0, 0.2),  # a neighbour: kept
        (0.5, 0.0, 0.2),  # beyond the margin: dropped
        (0.1, 0.05, 0.33),  # the robot's own finger, near its tool tip: dropped
        (float("nan"), 0.0, 0.2),
    )

    near = _scene_near(scene, _block_cloud(), [tip])

    np.testing.assert_allclose(near.points_f32(), [[0.15, 0.0, 0.175], [0.2, 0.0, 0.2]])
    assert near.frame_id == "world"
    assert near.ts == pytest.approx(2.0)


def test_with_no_target_points_no_scene_is_kept() -> None:
    no_points = PointCloud2.from_numpy(np.empty((0, 3), dtype=np.float32), frame_id="world")
    near = _scene_near(_points((0.1, 0.0, 0.17)), no_points, [])

    assert len(near.points_f32()) == 0


@pytest.fixture
def scene_filter(fallback: PickAndPlaceModule) -> PickAndPlaceModule:
    """The fallback module with GraspGenX also able to check its grasps against the
    camera's table; the check drops 37 grasps and keeps one."""
    provider: Any = fallback._grasp_generator
    fallback._scene_grasp_generator = provider
    provider.propose_grasps_in_scene.return_value = SceneGrasps(
        GraspCandidateArray(Header(1.0, "world"), [_grasp_at(0.1, 0.05, 0.9)]), 37
    )
    scene: Any = fallback._scene
    scene.get_full_scene_pointcloud.return_value = _table()
    return fallback


def test_grasps_are_checked_against_the_scene_around_the_target(
    scene_filter: PickAndPlaceModule,
) -> None:
    provider: Any = scene_filter._grasp_generator
    scene: Any = scene_filter._scene

    result = scene_filter.pick_object("cup-1")

    assert result.message == "Pick complete"
    provider.propose_grasps.assert_not_called()
    scene.get_full_scene_pointcloud.assert_called_once_with(exclude_object_id="cup-1")
    target, nearby, threshold = provider.propose_grasps_in_scene.call_args.args
    assert len(target.points_f32()) == len(_block_cloud().points_f32())
    assert threshold == pytest.approx(0.01)
    # The table within reach of the block, minus the part touching it.
    table = nearby.points_f32()
    assert 0 < len(table) < len(_table().points_f32())
    assert np.all(np.abs(table[:, 0] - 0.1) <= 0.18 + 1e-6)
    assert result.metadata["scene_filtered_by_source"] == {"graspgenx": 37}
    assert _tries(result, "scene_filtered_by_source") == [{"graspgenx": 37}]
    # Heuristic grasps are added unchecked.
    assert _tries(result, "candidates_by_source") == [{"graspgenx": 1, "heuristic": 2}]


def test_with_the_scene_filter_off_grasps_are_not_checked(
    scene_filter: PickAndPlaceModule,
) -> None:
    scene_filter.config.grasp_scene_filter = False
    provider: Any = scene_filter._grasp_generator

    result = scene_filter.pick_object("cup-1")

    assert result.message == "Pick complete"
    provider.propose_grasps.assert_called_once()
    provider.propose_grasps_in_scene.assert_not_called()
    scene: Any = scene_filter._scene
    scene.get_full_scene_pointcloud.assert_not_called()
    assert result.metadata["scene_filtered_by_source"] == {}
    assert "scene_filter_skipped" not in result.metadata["tries"][0]


def test_a_provider_that_cannot_check_the_scene_is_asked_as_before(
    module: PickAndPlaceModule,
) -> None:
    result = module.pick_object("cup-1")

    scene: Any = module._scene
    scene.get_full_scene_pointcloud.assert_not_called()
    assert result.metadata["scene_filtered_by_source"] == {}


def test_without_a_scene_cloud_grasps_are_not_checked_and_the_tries_say_why(
    scene_filter: PickAndPlaceModule,
) -> None:
    scene: Any = scene_filter._scene
    scene.get_full_scene_pointcloud.return_value = None
    provider: Any = scene_filter._grasp_generator

    result = scene_filter.pick_object("cup-1")

    provider.propose_grasps.assert_called_once()
    provider.propose_grasps_in_scene.assert_not_called()
    assert result.metadata["scene_filtered_by_source"] == {}
    assert _tries(result, "scene_filter_skipped") == [
        "the scene registration gave no scene point cloud"
    ]


def test_a_failing_scene_cloud_does_not_stop_the_pick(scene_filter: PickAndPlaceModule) -> None:
    scene: Any = scene_filter._scene
    scene.get_full_scene_pointcloud.side_effect = RuntimeError("no depth frame")

    result = scene_filter.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert _tries(result, "scene_filter_skipped") == [
        "the scene point cloud failed: RuntimeError: no depth frame"
    ]


def test_an_empty_scene_near_the_target_is_still_checked(
    scene_filter: PickAndPlaceModule,
) -> None:
    scene: Any = scene_filter._scene
    scene.get_full_scene_pointcloud.return_value = _points((2.0, 2.0, 2.0))
    provider: Any = scene_filter._grasp_generator
    provider.propose_grasps_in_scene.return_value = SceneGrasps(
        GraspCandidateArray(Header(1.0, "world"), [_grasp_at(0.1, 0.05, 0.9)]), 0
    )

    result = scene_filter.pick_object("cup-1")

    assert len(provider.propose_grasps_in_scene.call_args.args[1].points_f32()) == 0
    assert result.metadata["scene_filtered_by_source"] == {"graspgenx": 0}


def test_when_the_scene_check_drops_every_grasp_the_result_says_how_many(
    scene_filter: PickAndPlaceModule,
) -> None:
    scene_filter.config.heuristic_fallback = False
    provider: Any = scene_filter._grasp_generator
    provider.propose_grasps_in_scene.return_value = SceneGrasps(
        GraspCandidateArray(Header(1.0, "world"), []), 40
    )

    result = scene_filter.pick_object("cup-1")

    assert result.message == "Generated 0 grasp candidates for object cup-1."
    assert result.metadata["scene_filtered_by_source"] == {"graspgenx": 40}


def test_scene_filtered_grasps_add_up_over_retries(scene_filter: PickAndPlaceModule) -> None:
    scene_filter.config.heuristic_fallback = False
    provider: Any = scene_filter._grasp_generator
    provider.propose_grasps_in_scene.return_value = None
    provider.propose_grasps_in_scene.side_effect = [
        SceneGrasps(GraspCandidateArray(Header(1.0, "world"), [_grasp_at(0.1, 0.0, 0.9)]), 37),
        SceneGrasps(GraspCandidateArray(Header(1.0, "world"), [_grasp_at(0.1, 0.03, 0.9)]), 5),
    ]
    scene: Any = scene_filter._scene
    scene.scan_scene.side_effect = [
        _cup_at(0.1, 0.0, 0.2),  # lift check: still on the table
        _cup_at(0.1, 0.0, 0.2, object_id="cup-7"),  # re-scan
        _cup_at(0.1, 0.0, 0.3, object_id="cup-7"),  # lift check: rode up
    ]

    result = scene_filter.pick_object("cup-1")

    assert result.message == "Pick complete"
    assert _tries(result, "scene_filtered_by_source") == [{"graspgenx": 37}, {"graspgenx": 5}]
    assert result.metadata["scene_filtered_by_source"] == {"graspgenx": 42}
    assert [c.kwargs for c in scene.get_full_scene_pointcloud.call_args_list] == [
        {"exclude_object_id": "cup-1"},
        {"exclude_object_id": "cup-7"},
    ]
