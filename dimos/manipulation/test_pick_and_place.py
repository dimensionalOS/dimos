# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
from unittest.mock import MagicMock, patch

import pytest

from dimos.core.module import ModuleBase
from dimos.manipulation.manipulation_spec import CommandResult, CommandStatus
from dimos.manipulation.pick_and_place import PickAndPlaceModule, PickAndPlaceModuleConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header

PREGRASP_OFFSET = 0.1


def _ok(message: str = "") -> CommandResult:
    return CommandResult(CommandStatus.SUCCEEDED, message)


def _failed(message: str) -> CommandResult:
    return CommandResult(CommandStatus.FAILED, message)


def _module(**config: object) -> PickAndPlaceModule:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        module = PickAndPlaceModule()
    module.config = PickAndPlaceModuleConfig(**config)
    module._grasp_filter = MagicMock(
        inverse_kinematics_single=MagicMock(return_value=MagicMock(is_success=lambda: True))
    )
    module._motion = MagicMock()
    module._motion.move_to_pose.return_value = MagicMock(is_success=lambda: True)
    module._execution = MagicMock()
    module._execution.get_pre_grasp_offset.return_value = PREGRASP_OFFSET
    module._execution.lift_if_low.return_value = _ok()
    module._execution.open_gripper_and_settle.return_value = _ok("gripper open at 0.947")
    module._execution.close_gripper_and_verify.return_value = _ok("object held - readback 0.730")
    return module


def _with_object(module: PickAndPlaceModule, object_id: str = "cup-1") -> MagicMock:
    # MagicMock(name=...) names the mock itself, so set the attribute afterwards.
    obj = MagicMock(object_id=object_id, confidence=0.9, pointcloud=MagicMock())
    obj.name = "cup"
    module._on_objects([obj])
    return obj


def _proposes(module: PickAndPlaceModule, *candidates: GraspCandidate) -> None:
    module._grasp_generator = MagicMock(
        propose_grasps=MagicMock(
            return_value=GraspCandidateArray(Header(1.0, "world"), list(candidates))
        )
    )


def _top_down(x: float = 0.1, y: float = 0.2, z: float = 0.3, score: float = 0.9) -> GraspCandidate:
    return GraspCandidate(
        Pose(Vector3(x, y, z), Quaternion.from_euler(Vector3(-math.pi, 0.0, 0.0))), score=score
    )


def _select(module: PickAndPlaceModule) -> None:
    _with_object(module)
    _proposes(module, _top_down())
    module.select_grasp("cup-1")


def test_select_grasp_offsets_the_pregrasp_along_the_approach_axis() -> None:
    module = _module()
    _with_object(module)
    candidate = _top_down()
    _proposes(module, candidate)

    result = module.select_grasp("cup-1")

    assert result.is_success()
    assert module._selected_grasp is not None
    assert module._selected_grasp.position == candidate.pose.position
    # A top-down grasp approaches along world -Z, so the standoff sits above it.
    assert module._selected_pregrasp is not None
    assert module._selected_pregrasp.position.z == pytest.approx(0.3 + PREGRASP_OFFSET)


def test_select_grasp_uses_the_robot_models_standoff_when_unconfigured() -> None:
    module = _module()
    module._execution.get_pre_grasp_offset.return_value = 0.05
    _with_object(module)
    _proposes(module, _top_down())

    module.select_grasp("cup-1")

    assert module._selected_pregrasp is not None
    assert module._selected_pregrasp.position.z == pytest.approx(0.35)


def test_select_grasp_reports_an_unknown_object_rather_than_selecting_nothing() -> None:
    module = _module()
    _proposes(module, _top_down())

    result = module.select_grasp("missing")

    assert not result.is_success()
    assert result.error_code == "OBJECT_NOT_DETECTED"


def test_select_grasp_fails_when_every_proposal_is_unreachable() -> None:
    module = _module()
    _with_object(module)
    _proposes(module, _top_down())
    module._grasp_filter.inverse_kinematics_single.return_value = MagicMock(
        is_success=lambda: False
    )

    result = module.select_grasp("cup-1")

    assert not result.is_success()
    assert result.error_code == "GRASP_GENERATION_FAILED"
    assert module._selected_grasp is None


def test_candidate_policy_can_skip_filtering_and_rank_by_ik_feasibility() -> None:
    module = _module(candidate_filter="off", candidate_ranking="ik_feasibility")
    unsafe = _top_down(score=0.9)
    safe = _top_down(0.2, 0.3, 0.4, score=0.2)
    module._grasp_filter.inverse_kinematics_single.side_effect = [
        MagicMock(is_success=lambda: False),
        MagicMock(is_success=lambda: True),
    ]

    candidates = module._filter_candidates(
        GraspCandidateArray(Header(1.0, "world"), [unsafe, safe])
    )

    assert candidates.candidates == [safe, unsafe]


def test_pick_opens_approaches_grasps_and_retreats() -> None:
    module = _module()
    _select(module)

    result = module.pick_selected()

    assert result.is_success()
    assert module._holding_object
    # pregrasp, grasp, pregrasp again.
    assert module._motion.move_to_pose.call_count == 3
    module._execution.close_gripper_and_verify.assert_called_once()


def test_pick_fails_loudly_on_an_empty_grasp_and_backs_off() -> None:
    module = _module()
    _select(module)
    module._execution.close_gripper_and_verify.return_value = _failed(
        "nothing in the jaws (readback 0.030, at or below 0.100)"
    )

    result = module.pick_selected()

    assert not result.is_success()
    assert result.error_code == "GRASP_VERIFICATION_FAILED"
    assert "nothing in the jaws" in result.message
    assert not module._holding_object
    # Released and retreated rather than leaving the arm shut on the object.
    assert module._execution.open_gripper_and_settle.call_count == 2


def test_pick_reports_a_refused_gripper_command_as_a_gripper_failure() -> None:
    module = _module()
    _select(module)
    module._execution.open_gripper_and_settle.return_value = _failed("gripper open command refused")

    result = module.pick_selected()

    assert not result.is_success()
    assert result.error_code == "GRIPPER_FAILED"
    module._motion.move_to_pose.assert_not_called()


def test_pick_requires_a_selected_grasp() -> None:
    result = _module().pick_selected()

    assert not result.is_success()
    assert result.error_code == "INVALID_STATE"


def test_place_lowers_releases_and_retreats_using_the_grasp_orientation() -> None:
    module = _module()
    _select(module)
    module.pick_selected()
    module._motion.move_to_pose.reset_mock()

    result = module.place_at(0.3, 0.2, 0.1)

    assert result.is_success()
    assert not module._holding_object
    heights = [call.args[2] for call in module._motion.move_to_pose.call_args_list]
    assert heights == [0.1 + PREGRASP_OFFSET, 0.1, 0.1 + PREGRASP_OFFSET]


def test_place_fails_when_the_release_never_opens() -> None:
    module = _module()
    _select(module)
    module.pick_selected()
    module._execution.open_gripper_and_settle.return_value = _failed(
        "jaws settled 0.382 short of open (readback 0.618)"
    )

    result = module.place_at(0.3, 0.2, 0.1)

    assert not result.is_success()
    assert result.error_code == "GRIPPER_FAILED"
    assert module._holding_object


def test_place_requires_a_held_object() -> None:
    result = _module().place_at(0.3, 0.2, 0.1)

    assert not result.is_success()
    assert result.error_code == "INVALID_STATE"


def test_scan_reports_the_objects_perception_publishes_after_the_prompts_are_set() -> None:
    module = _module()
    module._scene = MagicMock()
    module._scene.set_prompts.side_effect = lambda prompts: _with_object(module)

    result = module.scan_objects(["cup", "  "])

    assert result.is_success()
    module._scene.set_prompts.assert_called_once_with(["cup"])
    assert result.metadata["objects"] == [{"object_id": "cup-1", "name": "cup", "confidence": 0.9}]


def test_scan_fails_when_perception_publishes_nothing_before_the_timeout() -> None:
    module = _module(scan_timeout=0.01)
    module._scene = MagicMock()

    result = module.scan_objects(["cup"])

    assert not result.is_success()
    assert result.error_code == "OBJECT_NOT_DETECTED"


def test_scan_rejects_an_empty_prompt_list() -> None:
    module = _module()
    module._scene = MagicMock()

    result = module.scan_objects(["   "])

    assert not result.is_success()
    assert result.error_code == "INVALID_INPUT"
    module._scene.set_prompts.assert_not_called()
