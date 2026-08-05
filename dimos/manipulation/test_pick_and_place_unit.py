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
from contextlib import nullcontext
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
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.perception.experimental.object import Object as DetObject
from dimos.perception.experimental.object_scene_registration import (
    ObjectSceneRegistrationModule,
)


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

        candidates = module._provider_candidates(
            detection, SimpleNamespace(proposal_source="grasp_provider")
        )

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
            module._provider_candidates(
                _make_det_object(), SimpleNamespace(proposal_source="grasp_provider")
            )

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
            module._provider_candidates(
                _make_det_object(), SimpleNamespace(proposal_source="grasp_provider")
            )

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

        candidates = module._provider_candidates(
            _make_det_object(), SimpleNamespace(proposal_source="grasp_provider")
        )

        assert [candidate.pose.position.x for candidate in candidates] == [0.2, 0.3, 0.1]

    def test_explicit_heuristic_fallback_identifies_source(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        module.config.heuristic_grasp_fallback = True
        transaction = SimpleNamespace(proposal_source="grasp_provider")
        pose = Pose(0.4, 0.0, 0.2)
        mocker.patch.object(module, "_generate_grasps_for_pick", return_value=[pose])

        candidates = module._provider_candidates(_make_det_object(), transaction)

        assert transaction.proposal_source == "heuristic"
        assert [(candidate.pose, candidate.score) for candidate in candidates] == [(pose, 0.0)]

    def test_provider_is_required_when_fallback_is_disabled(
        self, module: PickAndPlaceModule
    ) -> None:
        with pytest.raises(RuntimeError, match="fallback is disabled"):
            module._provider_candidates(
                _make_det_object(), SimpleNamespace(proposal_source="grasp_provider")
            )

    def test_selection_skips_higher_scored_infeasible_candidate(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        failed = SimpleNamespace(is_success=lambda: False)
        succeeded = SimpleNamespace(is_success=lambda: True)
        solve = mocker.patch.object(
            module,
            "inverse_kinematics_single",
            side_effect=[failed, succeeded, succeeded, succeeded],
        )
        transaction = SimpleNamespace(rejections=Counter())

        selected = module._select_feasible_grasp(
            [_candidate(0.4, 0.9), _candidate(0.5, 0.8)],
            "arm",
            0.1,
            transaction,
        )

        assert selected.rank == 2
        assert selected.candidate.score == 0.8
        assert solve.call_count == 4
        assert transaction.rejections == {"pre_grasp_infeasible": 1}

    def test_selection_rejects_malformed_candidate_and_honors_limit(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        module.config.max_grasp_candidates_to_check = 1
        invalid = _candidate(0.4, 0.9)
        invalid.pose.orientation.w = 0.0
        solve = mocker.patch.object(module, "inverse_kinematics_single")
        transaction = SimpleNamespace(rejections=Counter())

        with pytest.raises(RuntimeError, match="No feasible grasp among 1"):
            module._select_feasible_grasp([invalid, _candidate(0.5, 0.8)], "arm", 0.1, transaction)

        solve.assert_not_called()
        assert transaction.rejections == {"invalid": 1}


class TestPickTransaction:
    def _arrange_success(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> tuple[GraspCandidate, SimpleNamespace]:
        detection = _make_det_object()
        candidate = _candidate(0.4, 0.9)
        selected = _FeasibleGrasp(candidate, 1, Pose(0.4, 0.0, 0.3), Pose(0.4, 0.0, 0.3))
        robot_config = SimpleNamespace(pre_grasp_offset=0.1)
        mocker.patch.object(
            module, "_get_robot", return_value=("arm", "robot-id", robot_config, None)
        )
        mocker.patch.object(module, "_require_pick_object", return_value=detection)
        mocker.patch.object(module, "_provider_candidates", return_value=[candidate])
        mocker.patch.object(module, "_select_feasible_grasp", return_value=selected)
        mocker.patch.object(module, "_lift_if_low", return_value=SkillResult.ok())
        mocker.patch.object(module, "plan_to_pose", return_value=True)
        mocker.patch.object(module, "_preview_execute_wait", return_value=SkillResult.ok())
        mocker.patch.object(module, "_set_gripper_position", return_value=True)
        mocker.patch.object(
            module,
            "_verify_grasp",
            return_value=_GraspVerification(True, 0.1, "verified"),
        )
        suppression = SimpleNamespace(cleanup_error=None)
        world = mocker.Mock()
        world.suppress_object_obstacle.return_value = nullcontext(suppression)
        module._world_monitor = world
        return candidate, suppression

    def test_success_executes_ordered_pick_and_records_metadata(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        candidate, _ = self._arrange_success(module, mocker)

        result = module.pick("cup", object_id="abc12345")

        assert result.is_success()
        assert result.metadata["candidate_rank"] == 1
        assert result.metadata["candidate_score"] == 0.9
        assert module._last_pick_pose is candidate.pose
        assert module._set_gripper_position.call_args_list == [
            mocker.call(0.85, "arm"),
            mocker.call(0.0, "arm"),
        ]
        assert module.plan_to_pose.call_count == 3

    def test_retreat_failure_keeps_gripper_closed(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        self._arrange_success(module, mocker)
        module.plan_to_pose.side_effect = [True, True, False]

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
        log = mocker.patch("dimos.agents.annotation.logger.info")
        module._pick_guard.acquire()
        try:
            result = module.pick("cup")
        finally:
            module._pick_guard.release()

        assert result.error_code == "PICK_BUSY"
        get_robot.assert_not_called()
        log.assert_called_once()
        assert log.call_args.args[:3] == (
            "SKILL %s result=%s duration_ms=%.1f",
            "pick",
            "PICK_BUSY",
        )

    def test_cleanup_failure_does_not_hide_primary_failure(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        _, suppression = self._arrange_success(module, mocker)
        suppression.cleanup_error = "restore failed"
        module.plan_to_pose.side_effect = [False]

        result = module.pick("cup", object_id="abc12345")

        assert result.error_code == "PLANNING_FAILED"
        assert "cleanup: restore failed" in result.message

    def test_cleanup_failure_turns_success_into_scene_failure(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        _, suppression = self._arrange_success(module, mocker)
        suppression.cleanup_error = "restore failed"

        result = module.pick("cup", object_id="abc12345")

        assert result.error_code == "WORLD_MONITOR_UNAVAILABLE"
        assert "restore failed" in result.message

    @pytest.mark.parametrize(
        ("setup", "expected_code", "expected_phase"),
        [
            ("prepare", "EXECUTION_FAILED", "PREPARE"),
            ("open", "GRIPPER_FAILED", "PREPARE"),
            ("approach_planning", "PLANNING_FAILED", "APPROACH"),
            ("approach_execution", "EXECUTION_FAILED", "APPROACH"),
            ("grasp_planning", "PLANNING_FAILED", "GRASP"),
            ("grasp_execution", "EXECUTION_FAILED", "GRASP"),
            ("close", "GRIPPER_FAILED", "CLOSE"),
            ("verification", "GRASP_VERIFICATION_FAILED", "VERIFY"),
            ("retreat_planning", "PLANNING_FAILED", "RETREAT"),
            ("retreat_execution", "EXECUTION_FAILED", "RETREAT"),
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
        elif setup == "grasp_planning":
            module.plan_to_pose.side_effect = [True, False]
        elif setup == "grasp_execution":
            module._preview_execute_wait.side_effect = [
                SkillResult.ok(),
                SkillResult.fail("EXECUTION_FAILED", "rejected"),
            ]
        elif setup == "close":
            module._set_gripper_position.side_effect = [True, False]
        elif setup == "verification":
            module._verify_grasp.return_value = _GraspVerification(False, 0.0, "empty close")
        elif setup == "retreat_planning":
            module.plan_to_pose.side_effect = [True, True, False]
        else:
            module._preview_execute_wait.side_effect = [
                SkillResult.ok(),
                SkillResult.ok(),
                SkillResult.fail("EXECUTION_FAILED", "rejected"),
            ]

        result = module.pick("cup", object_id="abc12345")

        assert result.error_code == expected_code
        assert result.metadata["phase"] == expected_phase


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
    mocker.patch.object(module, "_get_robot", return_value=("arm", "robot-id", robot_config, None))
    failed = SimpleNamespace(is_success=lambda: False)
    feasible = SimpleNamespace(is_success=lambda: True)
    ik = mocker.patch.object(
        module,
        "inverse_kinematics_single",
        side_effect=[failed, feasible, feasible, feasible],
    )
    mocker.patch.object(module, "_lift_if_low", return_value=SkillResult.ok())
    plan = mocker.patch.object(module, "plan_to_pose", return_value=True)
    execute = mocker.patch.object(module, "_preview_execute_wait", return_value=SkillResult.ok())
    gripper = mocker.patch.object(module, "_set_gripper_position", return_value=True)
    suppression = SimpleNamespace(cleanup_error=None)
    world = mocker.Mock()
    world.suppress_object_obstacle.return_value = nullcontext(suppression)
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
    world.suppress_object_obstacle.assert_called_once_with("abc12345")
    assert ik.call_count == 4
    assert plan.call_count == 3
    assert execute.call_count == 3
    assert gripper.call_args_list == [mocker.call(0.85, "arm"), mocker.call(0.0, "arm")]


class TestGraspVerification:
    def test_empty_close_fails_immediately(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        module.config.grasp_verification = GraspVerificationConfig(
            enabled=True,
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
            enabled=True,
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
            enabled=True,
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
            enabled=True,
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
