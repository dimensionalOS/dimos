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
import time
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

from dimos_lcm.vision_msgs import BoundingBox3D, ObjectHypothesis, ObjectHypothesisWithPose
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
from dimos.manipulation.pick_and_place_module import (
    VOXEL_MAP_EXCLUSION_ID,
    HeldObject,
    PickAndPlaceModule,
    ScannedObject,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
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


def _group_state(gripper_position: float = 0.5) -> SimpleNamespace:
    """Manipulation state with the tool tip at (0.4, 0.0, 0.2) and the given jaw readback."""
    return SimpleNamespace(
        groups={
            "arm/tool": SimpleNamespace(
                gripper_position=gripper_position,
                end_effector_pose=PoseStamped(
                    frame_id="world",
                    position=Vector3(0.4, 0.0, 0.2),
                    orientation=Quaternion.from_euler(Vector3(0, 0, 0.7)),
                ),
            )
        }
    )


def _holding_cup() -> HeldObject:
    return HeldObject("cup-1", "cup", PoseStamped(frame_id="world", orientation=TOP_DOWN))


def _held_id(module: PickAndPlaceModule) -> str | None:
    return None if module._held is None else module._held.object_id


@pytest.fixture
def module() -> Iterator[PickAndPlaceModule]:
    instance = PickAndPlaceModule(planning_frame="world")
    instance._scene = MagicMock()
    instance._grasp_generator = MagicMock()
    instance._manipulation = MagicMock()
    instance._manipulation.list_planning_groups.return_value = [GROUP]
    instance._manipulation.get_state.return_value = _group_state()
    instance._manipulation.plan_to_poses.return_value = PLANNED
    instance._manipulation.execute.return_value = COMPLETED
    instance._manipulation.move_linear.return_value = _linear_move()
    instance._manipulation.set_gripper_position.return_value = ACCEPTED
    instance._objects = {"cup-1": ScannedObject("cup-1", "cup", CUP_AT)}
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
    assert result.metadata == {"object_id": "cup-1", "jaws": 0.0, "grasped": False}
    assert _held_id(module) is None
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


def test_an_empty_close_backs_out_before_the_target_is_an_obstacle_again(
    module: PickAndPlaceModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    manipulation: Any = module._manipulation
    _close_on_nothing(monkeypatch)
    order: list[str] = []

    def servo(*args: Any, **kwargs: Any) -> MoveResult:
        order.append("servo")
        return _linear_move()

    manipulation.move_linear.side_effect = servo
    manipulation.clear_voxel_map_exclusion.side_effect = lambda name: order.append("clear")

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
    scene: Any = module._scene
    manipulation: Any = module._manipulation
    scene.scan_scene.return_value = _detections(("cup-1", "cup", seen_at))

    result = module.pick_object("cup-1")

    assert result.message == (
        f"Lifted the gripper 0.10 m, but the camera sees object cup-1 moved {object_lift:.2f} m "
        "with it; it stayed on the table. Reopened the gripper."
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
    manipulation: Any = module._manipulation
    scene: Any = module._scene
    manipulation.get_state.return_value = _group_state(gripper_position=0.02)

    result = module.pick_object("cup-1")

    assert result.message == (
        "After lifting object cup-1, the gripper reads 0.02: the jaws are closed on nothing. "
        "Reopened the gripper."
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
    module._objects["cup-2"] = ScannedObject("cup-2", "cup", Vector3(0.2, 0.0, 0.2))
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


def test_the_lift_check_gives_up_on_a_camera_that_stopped(module: PickAndPlaceModule) -> None:
    scene: Any = module._scene
    module.config.fresh_frame_timeout = 0.05
    scene.scan_scene.return_value = _detections(ts=1.0)

    with pytest.raises(TimeoutError, match="No camera frame newer than"):
        module.pick_object("cup-1")


def test_place_ignores_detections_far_from_the_release(module: PickAndPlaceModule) -> None:
    """Another cup 30 cm from where the gripper let go is not the one placed."""
    scene: Any = module._scene
    module._held = _holding_cup()
    scene.scan_scene.return_value = _detections(("cup-2", "cup", Vector3(0.7, 0.0, 0.15)))

    result = module.place_at(0.4, 0.0, 0.2)

    assert result.message == "Place complete"
    assert result.metadata["located_by"] == "gripper"
