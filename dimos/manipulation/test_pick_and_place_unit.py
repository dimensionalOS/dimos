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

"""Unit tests for PickAndPlaceModule pure logic (no Drake required)."""

from __future__ import annotations

from collections import Counter
import json
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import open3d as o3d
import pytest
from pytest_mock import MockerFixture

from dimos.agents.skill_result import SkillResult
from dimos.core.coordination.blueprints import BlueprintAtom, autoconnect
from dimos.core.coordination.module_coordinator import _resolve_single_ref
from dimos.core.module import ModuleBase
from dimos.manipulation.box_filling_pick_and_place_module import (
    BoxFillingPickAndPlaceModule,
    BoxFillingPickAndPlaceModuleConfig,
)
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
from dimos.manipulation.pick_and_place_module import (
    GraspVerificationConfig,
    PickAndPlaceModule,
    PickAndPlaceModuleConfig,
    _FeasibleGrasp,
    _GraspVerification,
)
from dimos.manipulation.skill_errors import ManipulationSkillError
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.perception.experimental.object import Object as DetObject
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule


def _make_det_object(
    name: str = "cup",
    object_id: str = "abc12345",
    center: tuple[float, float, float] = (0.5, 0.0, 0.3),
    size: tuple[float, float, float] = (0.05, 0.05, 0.10),
) -> DetObject:
    """Create a DetObject with the given attributes and sensible defaults."""
    return DetObject(
        name=name,
        object_id=object_id,
        center=Vector3(x=center[0], y=center[1], z=center[2]),
        size=Vector3(x=size[0], y=size[1], z=size[2]),
        pose=PoseStamped(),
        pointcloud=PointCloud2(o3d.geometry.PointCloud()),
        bbox=(0.0, 0.0, 1.0, 1.0),
        track_id=0,
        class_id=0,
        confidence=1.0,
        ts=0.0,
        frame_id="world",
        image=Image(),
    )


@pytest.fixture
def module() -> PickAndPlaceModule:
    """Create a PickAndPlaceModule with heavy base init (RPC, config) patched out."""
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        result = PickAndPlaceModule()
    result.config = PickAndPlaceModuleConfig()
    return result


class TestFindObjectInDetections:
    """Test object lookup logic in detection snapshot."""

    def test_find_by_exact_name(self, module):
        det = _make_det_object(name="cup")
        module._detection_snapshot = [det]

        result = module._find_object_in_detections("cup")
        assert result is det

    def test_find_by_partial_name(self, module):
        det = _make_det_object(name="red cup")
        module._detection_snapshot = [det]

        result = module._find_object_in_detections("cup")
        assert result is det

    def test_find_by_object_id(self, module):
        det = _make_det_object(object_id="abc12345")
        module._detection_snapshot = [det]

        # Truncated prefix match
        result = module._find_object_in_detections("anything", object_id="abc1")
        assert result is det

    def test_find_by_object_id_ambiguous_returns_none(self, module):
        det1 = _make_det_object(object_id="abc12345")
        det2 = _make_det_object(object_id="abc19999")
        module._detection_snapshot = [det1, det2]

        result = module._find_object_in_detections("anything", object_id="abc1")
        assert result is None

    def test_find_missing_returns_none(self, module):
        module._detection_snapshot = [_make_det_object(name="bottle")]

        result = module._find_object_in_detections("keyboard")
        assert result is None

    def test_find_by_name_requires_unique_match(self, module):
        module._detection_snapshot = [
            _make_det_object(name="cup", object_id="first"),
            _make_det_object(name="red cup", object_id="second"),
        ]

        assert module._find_object_in_detections("cup") is None

    def test_empty_snapshot_returns_none(self, module):
        module._detection_snapshot = []

        result = module._find_object_in_detections("cup")
        assert result is None


class TestGraspHeuristics:
    """Test grasp orientation and occlusion offset static methods."""

    def test_occlusion_offset_toward_robot(self):
        center = Vector3(x=0.5, y=0.0, z=0.3)
        size = Vector3(x=0.1, y=0.1, z=0.1)

        ox, oy = PickAndPlaceModule._occlusion_offset(center, size)
        # Offset should shift x closer to robot origin (smaller x)
        assert ox < center.x
        assert abs(oy - center.y) < 1e-6  # y should stay ~0

    def test_occlusion_offset_at_origin(self):
        center = Vector3(x=0.0, y=0.0, z=0.3)
        size = Vector3(x=0.1, y=0.1, z=0.1)

        ox, oy = PickAndPlaceModule._occlusion_offset(center, size)
        # At origin, no shift should occur (division-by-zero guard)
        assert abs(ox) < 1e-3
        assert abs(oy) < 1e-3

    def test_grasp_orientation_near_is_top_down(self):
        q = PickAndPlaceModule._grasp_orientation(gx=0.3, gy=0.0, xy_dist=0.3)
        # Near object: pitch = 180° (top-down), tilt = 0, yaw = 0
        # RPY(0, π, 0) → quaternion (x=0, y=1, z=0, w=0)
        assert abs(q.x) < 0.01
        assert abs(q.y - 1.0) < 0.01
        assert abs(q.z) < 0.01
        assert abs(q.w) < 0.01

    def test_grasp_orientation_far_differs_from_near(self):
        q_near = PickAndPlaceModule._grasp_orientation(gx=0.3, gy=0.0, xy_dist=0.3)
        q_far = PickAndPlaceModule._grasp_orientation(gx=1.0, gy=0.0, xy_dist=1.0)
        # Far object should have different orientation (tilted)
        assert not (
            abs(q_near.x - q_far.x) < 0.01
            and abs(q_near.y - q_far.y) < 0.01
            and abs(q_near.z - q_far.z) < 0.01
            and abs(q_near.w - q_far.w) < 0.01
        )


class TestPlaceBack:
    """Test place_back guard logic."""

    def test_place_back_no_pick_pose_errors(self, module):
        module._last_pick_pose = None

        result = module.place_back()
        assert not result.is_success()
        assert result.error_code == "NO_PRIOR_POSE"
        assert "pick" in result.message.lower()

    def test_place_contact_legs_use_unchecked_linear_motion(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        robot_config = SimpleNamespace(pre_grasp_offset=0.1)
        mocker.patch.object(
            module,
            "_get_robot",
            return_value=("arm", "robot-id", robot_config),
        )
        mocker.patch.object(module, "_lift_if_low", return_value=SkillResult.ok())
        approach = mocker.patch.object(module, "plan_to_pose", return_value=True)
        mocker.patch.object(module, "_preview_execute_wait", return_value=SkillResult.ok())
        linear = mocker.patch.object(
            module,
            "_execute_linear_motion",
            return_value=SkillResult.ok(),
        )
        mocker.patch.object(module, "_set_gripper_position", return_value=True)
        mocker.patch("dimos.manipulation.pick_and_place_module.time.sleep")
        place_pose = Pose(Vector3(0.5, 0.0, 0.2), Quaternion())

        result = module._place_with_orientation(
            place_pose.position.x,
            place_pose.position.y,
            place_pose.position.z,
            place_pose.orientation,
        )

        assert result.is_success()
        approach.assert_called_once()
        pre_place_pose = approach.call_args.args[0]
        assert linear.call_args_list == [
            mocker.call(place_pose, "arm", 0.03, check_collision=False),
            mocker.call(pre_place_pose, "arm", 0.03, check_collision=False),
        ]


def test_grasp_pipeline_error_agent_encoding_is_structured() -> None:
    result = SkillResult[ManipulationSkillError].fail("PICK_BUSY", "pick in progress")

    payload = json.loads(result.agent_encode()[0]["text"])

    assert payload == {
        "success": False,
        "message": "pick in progress",
        "error_code": "PICK_BUSY",
        "duration_ms": 0.0,
    }


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"planning_frame": " "}, "planning_frame"),
        ({"grasp_approach_vector": (0.0, 0.0, 2.0)}, "unit vector"),
        (
            {
                "grasp_verification": {
                    "open_position": 0.85,
                    "closed_position": 0.0,
                    "held_threshold": 0.9,
                }
            },
            "held_threshold",
        ),
    ],
)
def test_grasp_pipeline_config_rejects_invalid_values(
    kwargs: dict[str, object], message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        PickAndPlaceModuleConfig(**kwargs)


def test_pick_module_declares_optional_perception_and_grasp_specs() -> None:
    atom = BlueprintAtom.create(PickAndPlaceModule, kwargs={})

    refs = {ref.name: ref for ref in atom.module_refs}

    assert refs["_object_scene"].optional is True
    assert refs["_grasp_generator"].optional is True


@pytest.mark.parametrize(
    ("ref_name", "provider"),
    [
        ("_grasp_generator", GraspGenXModule),
        ("_object_scene", ObjectSceneRegistrationModule),
    ],
)
def test_optional_provider_resolves_when_absent_present_or_ambiguous(
    ref_name: str, provider: type[ModuleBase]
) -> None:
    consumer = BlueprintAtom.create(PickAndPlaceModule, kwargs={})
    module_ref = next(ref for ref in consumer.module_refs if ref.name == ref_name)

    absent = autoconnect(PickAndPlaceModule.blueprint())
    assert _resolve_single_ref(consumer, module_ref, module_ref.spec, absent, set()) is None

    present = autoconnect(PickAndPlaceModule.blueprint(), provider.blueprint())
    assert (
        _resolve_single_ref(consumer, module_ref, module_ref.spec, present, set()) == provider.name
    )

    ambiguous = autoconnect(
        PickAndPlaceModule.blueprint(),
        provider.blueprint(instance_name="provider-a"),
        provider.blueprint(instance_name="provider-b"),
    )
    with pytest.raises(Exception, match="Multiple modules met that spec"):
        _resolve_single_ref(consumer, module_ref, module_ref.spec, ambiguous, set())


def _pointcloud(frame_id: str = "world", timestamp: float | None = None) -> PointCloud2:
    return PointCloud2.from_numpy(
        np.asarray([[0.4, 0.0, 0.2], [0.41, 0.01, 0.2]], dtype=np.float32),
        frame_id=frame_id,
        timestamp=timestamp,
    )


def _candidate(x: float, score: float) -> GraspCandidate:
    return GraspCandidate(
        Pose(Vector3(x, 0.0, 0.2), Quaternion(0.0, 0.0, 0.0, 1.0)),
        score,
    )


class TestProposalSelection:
    def test_provider_receives_real_world_frame_cloud(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        now = 100.0
        cloud = _pointcloud(timestamp=now)
        detection = _make_det_object()
        scene = mocker.Mock()
        scene.get_object_pointcloud_by_object_id.return_value = cloud
        generator = mocker.Mock()
        generator.propose_grasps.return_value = GraspCandidateArray(
            Header(now, "world"), [_candidate(0.4, 0.8)]
        )
        module._object_scene = scene
        module._grasp_generator = generator
        mocker.patch("dimos.manipulation.pick_and_place_module.time.time", return_value=now + 0.1)

        candidates = module._provider_candidates(detection)

        generator.propose_grasps.assert_called_once_with(cloud)
        assert [(candidate.pose.position.x, candidate.score) for candidate in candidates] == [
            (0.4, 0.8)
        ]

    @pytest.mark.parametrize("cloud_available", [False, True])
    def test_provider_rejects_missing_or_stale_cloud(
        self,
        module: PickAndPlaceModule,
        mocker: MockerFixture,
        cloud_available: bool,
    ) -> None:
        scene = mocker.Mock()
        scene.get_object_pointcloud_by_object_id.return_value = (
            _pointcloud(timestamp=1.0) if cloud_available else None
        )
        module._object_scene = scene
        module._grasp_generator = mocker.Mock()
        mocker.patch("dimos.manipulation.pick_and_place_module.time.time", return_value=100.0)

        with pytest.raises(RuntimeError, match="point cloud"):
            module._provider_candidates(_make_det_object())

    @pytest.mark.parametrize(
        ("cloud_frame", "proposal_frame"),
        [("camera", "world"), ("world", "camera")],
    )
    def test_provider_rejects_frame_mismatch(
        self,
        module: PickAndPlaceModule,
        mocker: MockerFixture,
        cloud_frame: str,
        proposal_frame: str,
    ) -> None:
        now = 100.0
        scene = mocker.Mock()
        scene.get_object_pointcloud_by_object_id.return_value = _pointcloud(
            cloud_frame, timestamp=now
        )
        generator = mocker.Mock()
        generator.propose_grasps.return_value = GraspCandidateArray(
            Header(now, proposal_frame), [_candidate(0.4, 0.8)]
        )
        module._object_scene = scene
        module._grasp_generator = generator
        mocker.patch("dimos.manipulation.pick_and_place_module.time.time", return_value=now)

        with pytest.raises(RuntimeError, match="frame"):
            module._provider_candidates(_make_det_object())

    def test_provider_preserves_stable_order_for_equal_scores(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        now = 100.0
        scene = mocker.Mock()
        scene.get_object_pointcloud_by_object_id.return_value = _pointcloud(timestamp=now)
        generator = mocker.Mock()
        generator.propose_grasps.return_value = GraspCandidateArray(
            Header(now, "world"),
            [_candidate(0.1, 0.5), _candidate(0.2, 0.7), _candidate(0.3, 0.7)],
        )
        module._object_scene = scene
        module._grasp_generator = generator
        mocker.patch("dimos.manipulation.pick_and_place_module.time.time", return_value=now)

        candidates = module._provider_candidates(_make_det_object())

        assert [candidate.pose.position.x for candidate in candidates] == [0.2, 0.3, 0.1]

    def test_provider_is_required(self, module: PickAndPlaceModule) -> None:
        with pytest.raises(RuntimeError, match="No grasp proposal provider"):
            module._provider_candidates(_make_det_object())

    def test_selection_skips_higher_scored_infeasible_candidate(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        endpoint = JointState(name=["arm/joint1"], position=[0.1])
        plan_sequence = mocker.patch.object(
            module,
            "_check_connected_pose_sequence",
            side_effect=[(0, None), (None, endpoint)],
        )
        ik_sequence = mocker.patch.object(
            module,
            "_check_pose_ik_sequence",
            return_value=(None, endpoint),
        )
        plan_motion = mocker.patch.object(module, "plan_to_pose")
        command_gripper = mocker.patch.object(module, "_set_gripper_position")
        transaction = SimpleNamespace(rejections=Counter(), object_id="abc12345")

        selected = module._select_feasible_grasp(
            [_candidate(0.4, 0.9), _candidate(0.5, 0.8)],
            "arm",
            transaction,
        )

        assert selected.rank == 2
        assert selected.candidate.score == 0.8
        assert plan_sequence.call_count == 2
        ik_sequence.assert_called_once_with(
            (selected.candidate.pose, selected.retreat_pose),
            "arm",
            start=endpoint,
            check_collision=False,
        )
        assert transaction.rejections == {"pre_grasp_infeasible": 1}
        plan_motion.assert_not_called()
        command_gripper.assert_not_called()

    @pytest.mark.parametrize(
        ("failed_index", "expected_rejection"),
        [
            (0, "pre_grasp_infeasible"),
            (1, "grasp_infeasible"),
            (2, "retreat_infeasible"),
        ],
    )
    def test_selection_reports_failed_connected_segment(
        self,
        module: PickAndPlaceModule,
        mocker: MockerFixture,
        failed_index: int,
        expected_rejection: str,
    ) -> None:
        pre_grasp = mocker.patch.object(module, "_check_connected_pose_sequence")
        contact = mocker.patch.object(module, "_check_pose_ik_sequence")
        if failed_index == 0:
            pre_grasp.return_value = (0, None)
        else:
            endpoint = JointState(name=["arm/joint1"], position=[0.1])
            pre_grasp.return_value = (None, endpoint)
            contact.return_value = (failed_index - 1, None)
        transaction = SimpleNamespace(rejections=Counter())

        with pytest.raises(RuntimeError, match="No feasible grasp among 1"):
            module._select_feasible_grasp([_candidate(0.4, 0.9)], "arm", transaction)

        assert transaction.rejections == {expected_rejection: 1}
        if failed_index == 0:
            contact.assert_not_called()

    def test_selection_rejects_malformed_candidate_and_honors_limit(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        module.config.max_grasp_candidates_to_check = 1
        invalid = _candidate(0.4, 0.9)
        invalid.pose.orientation.w = 0.0
        plan_sequence = mocker.patch.object(module, "_check_connected_pose_sequence")
        ik_sequence = mocker.patch.object(module, "_check_pose_ik_sequence")
        transaction = SimpleNamespace(rejections=Counter())

        with pytest.raises(RuntimeError, match="No feasible grasp among 1"):
            module._select_feasible_grasp([invalid, _candidate(0.5, 0.8)], "arm", transaction)

        plan_sequence.assert_not_called()
        ik_sequence.assert_not_called()
        assert transaction.rejections == {"invalid": 1}

    def test_pre_grasp_uses_fixed_configured_offset(self, module: PickAndPlaceModule) -> None:
        module.config.grasp_pre_grasp_offset = 0.25
        grasp = Pose(Vector3(0.4, 0.0, 0.3), Quaternion())

        result = module._compute_pre_grasp_pose(
            grasp,
            float(module.config.grasp_pre_grasp_offset),
            Vector3(0.0, 0.0, -1.0),
        )

        assert result == Pose(Vector3(0.4, 0.0, 0.05), grasp.orientation)

    def test_retreat_moves_back_with_world_up_bias(self, module: PickAndPlaceModule) -> None:
        module.config.grasp_retreat_offset = 0.10
        module.config.grasp_retreat_lift_offset = 0.01
        half_sqrt = 2**-0.5
        grasp = Pose(
            Vector3(0.4, 0.0, 0.2),
            Quaternion(0.0, half_sqrt, 0.0, half_sqrt),
        )

        retreat = module._compute_retreat_pose(
            grasp,
            Vector3(0.0, 0.0, -1.0),
        )

        assert retreat.position.x == pytest.approx(0.3)
        assert retreat.position.y == pytest.approx(0.0)
        assert retreat.position.z == pytest.approx(0.21)
        assert retreat.orientation == grasp.orientation


class TestPickTransaction:
    def _arrange_success(self, module: PickAndPlaceModule, mocker: MockerFixture) -> GraspCandidate:
        detection = _make_det_object()
        candidate = _candidate(0.4, 0.9)
        selected = _FeasibleGrasp(candidate, 1, Pose(0.4, 0.0, 0.3), Pose(0.4, 0.0, 0.3))
        robot_config = SimpleNamespace(pre_grasp_offset=0.1)
        mocker.patch.object(module, "_get_robot", return_value=("arm", "robot-id", robot_config))
        mocker.patch.object(module, "_require_pick_object", return_value=detection)
        mocker.patch.object(module, "_provider_candidates", return_value=[candidate])
        mocker.patch.object(module, "_select_feasible_grasp", return_value=selected)
        mocker.patch.object(module, "_safety_lift_pose", return_value=None)
        mocker.patch.object(module, "_lift_if_low", return_value=SkillResult.ok())
        mocker.patch.object(module, "plan_to_pose", return_value=True)
        mocker.patch.object(module, "_preview_execute_wait", return_value=SkillResult.ok())
        mocker.patch.object(module, "_execute_linear_motion", return_value=SkillResult.ok())
        mocker.patch.object(module, "_set_gripper_position", return_value=True)
        mocker.patch.object(
            module,
            "_verify_grasp",
            return_value=_GraspVerification(True, 0.1, "verified"),
        )
        module._world_monitor = mocker.Mock()
        return candidate

    def test_success_executes_ordered_pick_and_records_metadata(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        candidate = self._arrange_success(module, mocker)

        result = module.pick("cup", object_id="abc12345")

        assert result.is_success()
        assert result.metadata["candidate_rank"] == 1
        assert result.metadata["candidate_score"] == 0.9
        assert module._last_pick_pose is candidate.pose
        assert module._set_gripper_position.call_args_list == [
            mocker.call(0.85, "arm"),
            mocker.call(0.0, "arm"),
        ]
        assert module.plan_to_pose.call_args_list == [
            mocker.call(Pose(0.4, 0.0, 0.3), "arm"),
        ]
        assert module._execute_linear_motion.call_args_list == [
            mocker.call(candidate.pose, "arm", 0.03, check_collision=False),
            mocker.call(Pose(0.4, 0.0, 0.3), "arm", 0.03, check_collision=False),
        ]
        assert module._world_monitor.method_calls == []

    def test_no_safety_lift_validates_candidates_from_current_state(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        self._arrange_success(module, mocker)
        check_sequence = mocker.patch.object(module, "_check_connected_pose_sequence")

        result = module.pick("cup", object_id="abc12345")

        assert result.is_success()
        check_sequence.assert_not_called()
        assert module._select_feasible_grasp.call_args.args[3] is None

    def test_safety_lift_endpoint_is_shared_with_candidate_validation(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        self._arrange_success(module, mocker)
        lift_pose = Pose(0.2, 0.0, 0.1)
        lift_endpoint = JointState(name=["arm/joint1"], position=[0.2])
        module._safety_lift_pose.return_value = lift_pose
        check_sequence = mocker.patch.object(
            module,
            "_check_connected_pose_sequence",
            return_value=(None, lift_endpoint),
        )

        result = module.pick("cup", object_id="abc12345")

        assert result.is_success()
        check_sequence.assert_called_once_with((lift_pose,), "arm")
        assert module._select_feasible_grasp.call_args.args[3] is lift_endpoint

    def test_safety_lift_planning_failure_aborts_prepare_without_candidate_rejection(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        self._arrange_success(module, mocker)
        lift_pose = Pose(0.2, 0.0, 0.1)
        module._safety_lift_pose.return_value = lift_pose
        check_sequence = mocker.patch.object(
            module,
            "_check_connected_pose_sequence",
            return_value=(0, None),
        )

        result = module.pick("cup", object_id="abc12345")

        assert result.error_code == "PLANNING_FAILED"
        assert result.metadata["phase"] == "PREPARE"
        assert result.metadata["rejections"] == {}
        check_sequence.assert_called_once_with((lift_pose,), "arm")
        module._select_feasible_grasp.assert_not_called()
        module._lift_if_low.assert_not_called()
        module.plan_to_pose.assert_not_called()
        module._set_gripper_position.assert_not_called()

    def test_retreat_failure_keeps_gripper_closed(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        self._arrange_success(module, mocker)
        module._execute_linear_motion.side_effect = [
            SkillResult.ok(),
            SkillResult.fail("PLANNING_FAILED", "linear planning failed"),
        ]

        result = module.pick("cup", object_id="abc12345")

        assert result.error_code == "PLANNING_FAILED"
        assert result.metadata["object_may_be_held"] is True
        assert module._set_gripper_position.call_args_list == [
            mocker.call(0.85, "arm"),
            mocker.call(0.0, "arm"),
        ]

    def test_concurrent_pick_is_rejected_without_robot_access(
        self,
        module: PickAndPlaceModule,
        mocker: MockerFixture,
    ) -> None:
        get_robot = mocker.patch.object(module, "_get_robot")
        module._pick_guard.acquire()
        try:
            result = module.pick("cup")
        finally:
            module._pick_guard.release()

        assert result.error_code == "PICK_BUSY"
        get_robot.assert_not_called()

    @pytest.mark.parametrize(
        ("setup", "expected_code", "expected_phase"),
        [
            ("prepare", "EXECUTION_FAILED", "PREPARE"),
            ("open", "GRIPPER_FAILED", "PREPARE"),
            ("approach_planning", "PLANNING_FAILED", "APPROACH"),
            ("approach_execution", "EXECUTION_FAILED", "APPROACH"),
            ("grasp_motion", "PLANNING_FAILED", "GRASP"),
            ("close", "GRIPPER_FAILED", "CLOSE"),
            ("verification", "GRASP_VERIFICATION_FAILED", "VERIFY"),
            ("retreat_motion", "EXECUTION_FAILED", "RETREAT"),
        ],
    )
    def test_phase_failures_stop_the_pipeline(
        self,
        module: PickAndPlaceModule,
        mocker: MockerFixture,
        setup: str,
        expected_code: str,
        expected_phase: str,
    ) -> None:
        self._arrange_success(module, mocker)
        if setup == "prepare":
            module._lift_if_low.return_value = SkillResult.fail("EXECUTION_FAILED", "lift failed")
        elif setup == "open":
            module._set_gripper_position.return_value = False
        elif setup == "approach_planning":
            module.plan_to_pose.side_effect = [False]
        elif setup == "approach_execution":
            module._preview_execute_wait.side_effect = [
                SkillResult.fail("EXECUTION_FAILED", "rejected")
            ]
        elif setup == "grasp_motion":
            module._execute_linear_motion.side_effect = [
                SkillResult.fail("PLANNING_FAILED", "linear planning failed")
            ]
        elif setup == "close":
            module._set_gripper_position.side_effect = [True, False]
        elif setup == "verification":
            module._verify_grasp.return_value = _GraspVerification(False, 0.0, "empty close")
        else:
            module._execute_linear_motion.side_effect = [
                SkillResult.ok(),
                SkillResult.fail("EXECUTION_FAILED", "rejected"),
            ]

        result = module.pick("cup", object_id="abc12345")

        assert result.error_code == expected_code
        assert result.metadata["phase"] == expected_phase
        module._select_feasible_grasp.assert_called_once()


def test_full_pick_pipeline_uses_real_messages_and_fake_boundary_providers(
    module: PickAndPlaceModule, mocker: MockerFixture
) -> None:
    now = 100.0
    detection = _make_det_object()
    module._detection_snapshot = [detection]
    scene = mocker.Mock()
    scene.get_object_pointcloud_by_object_id.return_value = _pointcloud(timestamp=now)
    generator = mocker.Mock()
    generator.propose_grasps.return_value = GraspCandidateArray(
        Header(now, "world"),
        [_candidate(0.4, 0.9), _candidate(0.5, 0.8)],
    )
    module._object_scene = scene
    module._grasp_generator = generator
    robot_config = SimpleNamespace(pre_grasp_offset=0.1)
    mocker.patch.object(module, "_get_robot", return_value=("arm", "robot-id", robot_config))
    plan_sequence = mocker.patch.object(
        module,
        "_check_connected_pose_sequence",
        side_effect=[(0, None), (None, JointState())],
    )
    ik_sequence = mocker.patch.object(
        module,
        "_check_pose_ik_sequence",
        return_value=(None, JointState()),
    )
    mocker.patch.object(module, "_safety_lift_pose", return_value=None)
    mocker.patch.object(module, "_lift_if_low", return_value=SkillResult.ok())
    mocker.patch.object(
        module,
        "_verify_grasp",
        return_value=_GraspVerification(True, 0.1, "verified"),
    )
    plan = mocker.patch.object(module, "plan_to_pose", return_value=True)
    execute = mocker.patch.object(module, "_preview_execute_wait", return_value=SkillResult.ok())
    linear = mocker.patch.object(module, "_execute_linear_motion", return_value=SkillResult.ok())
    gripper = mocker.patch.object(module, "_set_gripper_position", return_value=True)
    world = mocker.Mock()
    module._world_monitor = world
    mocker.patch("dimos.manipulation.pick_and_place_module.time.time", return_value=now)

    result = module.pick("cup", object_id="abc12345")

    assert result.is_success()
    assert result.metadata["proposal_source"] == "grasp_provider"
    assert result.metadata["candidate_rank"] == 2
    assert result.metadata["candidate_score"] == 0.8
    assert result.metadata["rejections"] == {"pre_grasp_infeasible": 1}
    scene.get_object_pointcloud_by_object_id.assert_called_once_with("abc12345")
    generator.propose_grasps.assert_called_once_with(scene.get_object_pointcloud_by_object_id())
    assert world.method_calls == []
    assert plan_sequence.call_count == 2
    assert ik_sequence.call_count == 1
    assert plan.call_count == 1
    assert execute.call_count == 1
    assert linear.call_count == 2
    assert gripper.call_args_list == [mocker.call(0.85, "arm"), mocker.call(0.0, "arm")]


class TestGraspVerification:
    def test_empty_close_fails_immediately(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        module.config.grasp_verification = GraspVerificationConfig(
            timeout=1.0,
            poll_interval=0.1,
            held_threshold=0.02,
        )
        mocker.patch.object(module, "get_gripper", return_value=0.0)
        mocker.patch(
            "dimos.manipulation.pick_and_place_module.time.monotonic",
            side_effect=[0.0, 0.1],
        )

        result = module._verify_grasp("arm")

        assert result == _GraspVerification(False, 0.0, "gripper reached the empty-closed region")

    def test_held_position_succeeds_after_timeout(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        module.config.grasp_verification = GraspVerificationConfig(
            timeout=1.0,
            poll_interval=0.1,
            held_threshold=0.02,
        )
        mocker.patch.object(module, "get_gripper", return_value=0.1)
        mocker.patch(
            "dimos.manipulation.pick_and_place_module.time.monotonic",
            side_effect=[0.0, 0.1, 1.1],
        )
        sleep = mocker.patch("dimos.manipulation.pick_and_place_module.time.sleep")

        result = module._verify_grasp("arm")

        assert result == _GraspVerification(True, 0.1, "grasp verified by gripper closure feedback")
        sleep.assert_called_once_with(0.1)

    def test_no_gripper_motion_is_not_misclassified_as_a_grasp(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        module.config.grasp_verification = GraspVerificationConfig(
            timeout=1.0,
            poll_interval=0.1,
            held_threshold=0.02,
        )
        mocker.patch.object(module, "get_gripper", return_value=0.85)
        mocker.patch(
            "dimos.manipulation.pick_and_place_module.time.monotonic",
            side_effect=[0.0, 0.1, 1.1],
        )
        mocker.patch("dimos.manipulation.pick_and_place_module.time.sleep")

        result = module._verify_grasp("arm")

        assert result == _GraspVerification(False, 0.85, "gripper did not leave the open position")

    def test_feedback_timeout_is_reported(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        module.config.grasp_verification = GraspVerificationConfig(
            timeout=1.0,
            poll_interval=0.1,
            held_threshold=0.02,
        )
        mocker.patch.object(module, "get_gripper", return_value=None)
        mocker.patch(
            "dimos.manipulation.pick_and_place_module.time.monotonic",
            side_effect=[0.0, 0.1, 1.1],
        )
        mocker.patch("dimos.manipulation.pick_and_place_module.time.sleep")

        result = module._verify_grasp("arm")

        assert result == _GraspVerification(False, None, "gripper feedback was unavailable")


class TestNumberedSelectionApi:
    def test_select_object_pins_snapshot_and_provider_order(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        detection = _make_det_object()
        candidates = [_candidate(0.4, 0.9), _candidate(0.5, 0.8)]
        module._replace_detection_snapshot([detection])
        provider = mocker.patch.object(module, "_provider_candidates", return_value=candidates)

        result = module.select_object(1)

        assert result.is_success()
        provider.assert_called_once_with(detection)
        assert module._prepared_pick is not None
        assert module._prepared_pick.snapshot_version == module._snapshot_version
        assert module._prepared_pick.detection is detection
        assert module._prepared_pick.candidates == tuple(candidates)

    def test_new_scan_invalidates_prepared_selection(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        module._replace_detection_snapshot([_make_det_object()])
        mocker.patch.object(module, "_provider_candidates", return_value=[_candidate(0.4, 0.9)])
        assert module.select_object(1).is_success()

        module._replace_detection_snapshot([_make_det_object(name="bottle")])

        assert module._prepared_pick is None

    def test_pick_selected_rejects_expired_preparation_without_robot_access(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        module._replace_detection_snapshot([_make_det_object()])
        mocker.patch.object(module, "_provider_candidates", return_value=[_candidate(0.4, 0.9)])
        mocker.patch("dimos.manipulation.pick_and_place_module.time.monotonic", return_value=0.0)
        assert module.select_object(1).is_success()
        module.config.preparation_timeout = 1.0
        get_robot = mocker.patch.object(module, "_get_robot")

        with patch("dimos.manipulation.pick_and_place_module.time.monotonic", return_value=2.0):
            result = module.pick_selected()

        assert result.error_code == "INVALID_STATE"
        get_robot.assert_not_called()

    def test_place_at_converts_object_reference_to_tcp_target(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        quarter_turn = Quaternion.from_euler(Vector3(0.0, 0.0, np.pi / 2.0))
        module._held_object_orientation = quarter_turn
        module._held_object_to_tcp = Pose(Vector3(0.1, 0.0, 0.0), Quaternion())
        module._held_object_size = Vector3(0.05, 0.05, 0.1)
        place = mocker.patch.object(
            module, "_place_with_orientation", return_value=SkillResult.ok()
        )

        result = module.place_at(0.5, 0.2, 0.3)

        assert result.is_success()
        target = place.call_args.args
        assert target[0] == pytest.approx(0.5)
        assert target[1] == pytest.approx(0.3)
        assert target[2] == pytest.approx(0.3)
        assert target[3] == quarter_turn
        assert module._held_object_to_tcp is None


@pytest.fixture
def box_module() -> BoxFillingPickAndPlaceModule:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        result = BoxFillingPickAndPlaceModule()
    result.config = BoxFillingPickAndPlaceModuleConfig()
    return result


class TestBoxFillingPolicy:
    def test_destination_policy_computes_fit_checked_object_target(
        self, box_module: BoxFillingPickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        box_module._replace_detection_snapshot(
            [_make_det_object(name="box", center=(0.6, 0.1, 0.1), size=(0.3, 0.2, 0.2))]
        )
        assert box_module.select_destination_container(1).is_success()
        box_module._held_object_size = Vector3(0.05, 0.04, 0.10)
        place = mocker.patch.object(box_module, "place_at", return_value=SkillResult.ok())

        result = box_module.place_in_destination("arm")

        assert result.is_success()
        place.assert_called_once_with(0.6, 0.1, pytest.approx(0.27), "arm")

    def test_destination_policy_rejects_object_that_does_not_fit(
        self, box_module: BoxFillingPickAndPlaceModule
    ) -> None:
        box_module._replace_detection_snapshot([_make_det_object(name="box", size=(0.1, 0.1, 0.2))])
        assert box_module.select_destination_container(1).is_success()
        box_module._held_object_size = Vector3(0.2, 0.04, 0.05)

        result = box_module.place_in_destination()

        assert result.error_code == "INVALID_INPUT"
