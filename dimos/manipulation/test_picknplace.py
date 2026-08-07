# Copyright 2026 Dimensional Inc.
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

import math
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.module import ModuleBase
from dimos.manipulation.blueprints import _picknplace_xarm6_model, _xarm_graspgenx, picknplace
from dimos.manipulation.picknplace import (
    PickNPlaceConfig,
    PickNPlaceModule,
    _estimate_table_surface,
    _table_midpoint_grasp_z,
)
from dimos.manipulation.planning.spec.models import IKResult, IKStatus
from dimos.manipulation.visualization.layers import MeshElement
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header
from dimos.robot.manipulators.xarm.grasp_config import XARM_TCP_TO_GRASP_FRAME


def test_picknplace_scans_and_selects_target() -> None:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        module = PickNPlaceModule()
    module.config = PickNPlaceConfig()
    module._visualization = MagicMock()
    scene = MagicMock()
    detections = MagicMock()
    module._scene = scene
    obj = MagicMock(
        ts=1.0,
        frame_id="link_base",
        center=Vector3(0.1, 0.2, 0.04),
        confidence=0.9,
    )
    obj.name = "cup"
    obj.size = Vector3(0.4, 0.1, 0.2)
    obj.camera_transform = None
    obj.image = None
    obj.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    scene.scan_scene.side_effect = lambda: (module._on_objects([obj]), detections)[1]

    with patch("dimos.manipulation.picknplace.to_detection3d_array") as to_detection3d_array:
        result = MagicMock()
        to_detection3d_array.return_value = result
        assert module.scan_scene() is result
        to_detection3d_array.assert_called_once_with([obj], frame_id="link_base", ts=1.0)

    assert module.get_scene_info() == [{"number": 1, "name": "cup", "confidence": 0.9}]
    goal = module.get_goal_pose(1)
    assert goal is not None
    assert goal.position == Vector3(0.1, 0.2, 0.100)
    assert goal.orientation == Quaternion.from_euler(Vector3(-3.141592653589793, 0.0, 0.0))
    pre_grasp = module.get_pre_grasp_pose()
    assert pre_grasp is not None
    assert pre_grasp.position == Vector3(0.1, 0.2, 0.200)

    selected = module.select_object(1)
    assert selected.is_success()
    assert selected.metadata["goal"] == {
        "x": 0.1,
        "y": 0.2,
        "z": 0.1,
        "roll": -math.pi,
        "pitch": 0.0,
        "yaw": 0.0,
    }
    assert selected.metadata["pre_grasp"] == {
        "x": 0.1,
        "y": 0.2,
        "z": 0.2,
        "roll": -math.pi,
        "pitch": 0.0,
        "yaw": 0.0,
    }
    assert module._selected_object is obj

    module.scan_scene("water bottle")
    scene.set_prompts.assert_called_once_with(["water bottle"])

    module.config = PickNPlaceConfig(align_grasp_yaw=True)
    yaw_aligned_goal = module.get_goal_pose(1)
    assert yaw_aligned_goal is not None
    expected = Quaternion.from_euler(Vector3(-math.pi, 0.0, 0.0))
    assert yaw_aligned_goal.orientation.angle_to(expected) == pytest.approx(0.0)


def test_scan_objects_uses_independent_simple_queries() -> None:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        module = PickNPlaceModule()
    module._visualization = MagicMock()
    module.scan_scene = MagicMock(detections_length=3)
    module.get_scene_info = MagicMock(return_value=[])
    module._publish_scene_objects = MagicMock()

    result = module.scan_objects([" wooden block ", "white box", " "])

    assert result.is_success()
    assert result.metadata["queried_names"] == ["wooden block", "white box"]
    module.scan_scene.assert_called_once_with(prompts=["wooden block", "white box"])


def test_pick_selected_verifies_gripper_did_not_fully_close() -> None:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        module = PickNPlaceModule()
    module.config = PickNPlaceConfig(grasp_feedback_delay=0.0)
    module._goal_pose = PoseStamped(
        position=Vector3(0.3, 0.1, 0.12),
        orientation=Quaternion.from_euler(Vector3(-math.pi, 0.0, 0.0)),
    )
    module._pre_grasp_pose = PoseStamped(
        position=Vector3(0.3, 0.1, 0.22),
        orientation=Quaternion.from_euler(Vector3(-math.pi, 0.0, 0.0)),
    )
    module._pick_execution = MagicMock()
    module._pick_execution.open_gripper.return_value = MagicMock(is_success=lambda: True)
    module._pick_execution.move_to_pose.return_value = MagicMock(is_success=lambda: True)
    module._pick_execution.close_gripper.return_value = MagicMock(is_success=lambda: True)
    module._pick_execution.get_gripper.return_value = 0.0

    result = module.pick_selected()

    assert not result.is_success()
    assert result.error_code == "GRASP_VERIFICATION_FAILED"
    assert "empty-closed" in result.message
    assert result.metadata["gripper_position"] == 0.0
    assert result.metadata["rescan_required"] is True
    assert result.metadata["recovered_to_pre_grasp"] is True
    assert module._pick_execution.open_gripper.call_count == 2
    assert module._pick_execution.move_to_pose.call_count == 3


def test_place_selected_uses_remembered_box_and_held_object() -> None:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        module = PickNPlaceModule()
    module.config = PickNPlaceConfig()
    module._open_box = {
        "center_x": 0.4,
        "center_y": -0.1,
        "tabletop_z": 0.1,
        "rim_z": 0.18,
        "opening_width": 0.18,
        "opening_depth": 0.14,
    }
    module._held_object_size = Vector3(0.04, 0.03, 0.02)
    module._pick_execution = MagicMock()
    module._pick_execution.move_to_pose.return_value = MagicMock(is_success=lambda: True)
    module._pick_execution.open_gripper.return_value = MagicMock(is_success=lambda: True)
    module._pick_execution.get_ee_pose.return_value = Pose(
        Vector3(0.3, 0.1, 0.22), Quaternion(0.0, 0.0, 0.0, 1.0)
    )

    result = module.place_selected()

    assert result.is_success()
    assert module._pick_execution.move_to_pose.call_args_list[0].args[:3] == pytest.approx(
        (0.3, 0.1, 0.31)
    )
    assert module._pick_execution.move_to_pose.call_args_list[1].args[:3] == pytest.approx(
        (0.4, -0.1, 0.31)
    )
    assert module._pick_execution.move_to_pose.call_args_list[2].args[:3] == pytest.approx(
        (0.4, -0.1, 0.21)
    )
    assert result.metadata["object_bottom_clearance"] == pytest.approx(0.02)
    assert module._pick_execution.move_to_pose.call_count == 3
    assert module._held_object_size is None


def test_picknplace_home_matches_xarm_lifecycle_home() -> None:
    assert _picknplace_xarm6_model.home_joints == [
        0.0,
        math.radians(-40.0),
        math.radians(-50.0),
        0.0,
        math.radians(90.0),
        0.0,
    ]


def test_picknplace_graspgenx_uses_xarm_tcp_calibration() -> None:
    assert _xarm_graspgenx.grasp_frame_to_tcp[2][3] == pytest.approx(0.172)
    assert _xarm_graspgenx.grasp_frame_to_tcp[:2] == ((0.0, -1.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    assert np.allclose(
        np.asarray(_xarm_graspgenx.grasp_frame_to_tcp) @ np.asarray(XARM_TCP_TO_GRASP_FRAME),
        np.eye(4),
    )


def test_picknplace_yaw_alignment_defaults_to_disabled() -> None:
    assert not PickNPlaceConfig().align_grasp_yaw


def test_parallel_jaw_yaw_uses_the_nearest_equivalent_orientation() -> None:
    assert PickNPlaceModule._closest_parallel_jaw_yaw(-math.pi + 0.02, 0.0) == pytest.approx(0.02)


def test_picknplace_blueprint_accepts_short_backend_and_grasp_options() -> None:
    config = BlueprintConfigParser(picknplace)

    options = config.parse(
        overrides={"osr": {"det": "moondream", "seg": "edgetam"}, "pnp": {"grasp": "graspgenx"}}
    )

    assert options.module_configs["osr"]["det"] == "moondream"
    assert options.module_configs["osr"]["seg"] == "edgetam"
    assert options.module_configs["pnp"]["grasp"] == "graspgenx"


def test_table_surface_estimate_ignores_objects_above_the_table() -> None:
    x, y = np.meshgrid(np.linspace(0.2, 0.8, 20), np.linspace(-0.4, 0.4, 20))
    table = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, 0.35)))
    object_points = np.array([[0.5, 0.0, 0.55], [0.51, 0.0, 0.57], [0.5, 0.01, 0.56]])

    estimate = _estimate_table_surface(np.vstack((table, object_points)))

    assert estimate is not None
    assert estimate["tabletop_z"] == pytest.approx(0.35, abs=0.01)
    assert estimate["width"] >= 0.8
    assert estimate["depth"] >= 1.0


def test_table_midpoint_grasp_uses_observed_object_height() -> None:
    points = np.array(
        [
            [0.2, 0.1, 0.117],
            [0.2, 0.1, 0.119],
            [0.2, 0.1, 0.120],
            [0.2, 0.1, 0.121],
            [0.2, 0.1, 0.120],
            [0.2, 0.1, 0.119],
            [0.2, 0.1, 0.120],
            [0.2, 0.1, 0.119],
            [0.2, 0.1, 0.120],
            [0.2, 0.1, 0.120],
        ]
    )

    assert _table_midpoint_grasp_z(points, 0.100, 0.120) == pytest.approx(0.110275)
    assert _table_midpoint_grasp_z(points[:9], 0.100, 0.120) == pytest.approx(0.120)


def test_table_surface_estimate_displays_filled_tabletop() -> None:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        module = PickNPlaceModule()
    x, y = np.meshgrid(np.linspace(0.2, 0.8, 20), np.linspace(-0.4, 0.4, 20))
    scene_cloud = MagicMock()
    scene_cloud.points_f32.return_value = np.column_stack(
        (x.ravel(), y.ravel(), np.full(x.size, 0.35))
    )
    module._scene = MagicMock(get_full_scene_pointcloud=MagicMock(return_value=scene_cloud))
    module._visualization = MagicMock()

    assert module.estimate_table_surface() is not None

    layer = module._visualization.set_visualization_layer.call_args.args[0]
    assert isinstance(layer.elements[0], MeshElement)
    assert layer.elements[0].opacity == pytest.approx(1.0)
    np.testing.assert_array_equal(layer.elements[0].triangles, [[0, 1, 2], [0, 2, 3]])


def test_estimate_table_runs_a_fresh_scan_before_fitting() -> None:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        module = PickNPlaceModule()
    module.scan_scene = MagicMock()
    module.estimate_table_surface = MagicMock(
        return_value={
            "center_x": 0.5,
            "center_y": 0.0,
            "tabletop_z": 0.35,
            "width": 0.8,
            "depth": 1.0,
        }
    )

    result = module.estimate_table()

    assert result.is_success()
    module.scan_scene.assert_called_once_with()


def test_install_open_box_is_display_only() -> None:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        module = PickNPlaceModule()
    obj = MagicMock(
        center=Vector3(0.4, 0.1, 0.14),
        size=Vector3(0.20, 0.16, 0.08),
    )
    obj.pose.orientation = Quaternion.from_euler(Vector3(0.0, 0.0, 0.0))
    obj.pointcloud.points_f32.return_value = np.asarray([[0.3, 0.1, 0.18]] * 10, dtype=np.float32)
    module._latest_objects = (obj,)
    module._tabletop_z = 0.10
    module._obstacle_world = MagicMock()
    module._obstacle_world.update_obstacle.return_value = False
    module._obstacle_world.add_obstacle.side_effect = lambda name, *_: name
    module._visualization = MagicMock()

    result = module.install_open_box(1, wall_thickness=0.01)

    assert result.is_success()
    assert result.metadata["opening_width"] == pytest.approx(0.18)
    assert result.metadata["opening_depth"] == pytest.approx(0.14)
    module._obstacle_world.add_obstacle.assert_not_called()
    module._obstacle_world.update_obstacle.assert_not_called()
    layer = module._visualization.set_visualization_layer.call_args.args[0]
    assert layer.id == "picknplace/open-box"
    assert layer.elements[0].id == "box-envelope"


def test_picknplace_uses_top_graspgenx_candidate() -> None:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        module = PickNPlaceModule()
    module.config = PickNPlaceConfig(grasp_strategy="graspgenx")
    obj = MagicMock(
        ts=1.0,
        frame_id="link_base",
        center=Vector3(0.1, 0.2, 0.3),
        pointcloud=MagicMock(),
    )
    obj.camera_transform = None
    obj.image = None
    obj.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    obj.pointcloud.points_f32.return_value = np.asarray([[0.4, 0.5, 0.6]], dtype=np.float32)
    module._latest_objects = (obj,)
    candidate = GraspCandidate(
        Pose(
            Vector3(0.4, 0.5, 0.6),
            Quaternion.from_euler(Vector3(0.0, math.pi / 2.0, 0.0)),
        ),
        score=0.9,
    )
    second_candidate = GraspCandidate(
        Pose(Vector3(0.2, 0.3, 0.4), Quaternion()),
        score=0.8,
    )
    module._grasp_generator = MagicMock(
        propose_grasps=MagicMock(
            return_value=GraspCandidateArray(
                Header(2.0, "link_base"), [candidate, second_candidate]
            )
        )
    )
    module._grasp_filter = MagicMock(
        inverse_kinematics_single=MagicMock(return_value=IKResult(IKStatus.SUCCESS))
    )
    module.graspgenx_candidates = MagicMock()
    module._visualization = MagicMock()

    goal = module.get_goal_pose(1)

    assert goal is not None
    assert goal.ts == 2.0
    assert goal.frame_id == "link_base"
    assert goal.position == candidate.pose.position
    assert goal.orientation == candidate.pose.orientation
    assert module.get_grasp_candidates().candidates == [candidate, second_candidate]
    module.graspgenx_candidates.publish.assert_called_once_with(module.get_grasp_candidates())
    pre_grasp = module.get_pre_grasp_pose()
    assert pre_grasp is not None
    assert pre_grasp.position.x == pytest.approx(goal.position.x - 0.1)
    assert pre_grasp.position.z == pytest.approx(goal.position.z)
    layer = module._visualization.set_visualization_layer.call_args.args[0]
    assert layer.id == "picknplace/selection"
    assert layer.elements[0].points.shape[1] == 3
    assert layer.elements[1].line_width is None
    selected_goal = module.select_grasp_candidate(1)
    assert selected_goal is not None
    assert selected_goal.position == second_candidate.pose.position
    assert module.get_grasp_candidates().selected_index == 1
    pre_grasp = module.get_pre_grasp_pose()
    assert pre_grasp is not None
    assert pre_grasp.position.x == pytest.approx(selected_goal.position.x)
    assert pre_grasp.position.z == pytest.approx(selected_goal.position.z - 0.1)


def test_picknplace_excludes_collision_or_ik_infeasible_graspgenx_candidates() -> None:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        module = PickNPlaceModule()
    module.config = PickNPlaceConfig(grasp_strategy="graspgenx")
    safe = GraspCandidate(Pose(Vector3(0.4, 0.5, 0.6), Quaternion()), score=0.9)
    unsafe = GraspCandidate(Pose(Vector3(0.2, 0.3, 0.4), Quaternion()), score=0.8)
    module._grasp_filter = MagicMock(
        inverse_kinematics_single=MagicMock(
            side_effect=[IKResult(IKStatus.SUCCESS), IKResult(IKStatus.NO_SOLUTION)]
        )
    )

    filtered = module._filter_graspgenx_candidates(
        GraspCandidateArray(Header(2.0, "link_base"), [safe, unsafe])
    )

    assert filtered.candidates == [safe]
    module._grasp_filter.inverse_kinematics_single.assert_any_call(
        safe.pose, "arm", check_collision=True
    )
    module._grasp_filter.inverse_kinematics_single.assert_any_call(
        unsafe.pose, "arm", check_collision=True
    )


def test_picknplace_clears_candidates_when_no_graspgenx_proposal_is_safe() -> None:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        module = PickNPlaceModule()
    module.config = PickNPlaceConfig(grasp_strategy="graspgenx")
    obj = MagicMock(
        ts=1.0,
        frame_id="link_base",
        center=Vector3(0.1, 0.2, 0.3),
        pointcloud=MagicMock(),
    )
    obj.camera_transform = None
    obj.image = None
    obj.pose.orientation = Quaternion()
    module._latest_objects = (obj,)
    unsafe = GraspCandidate(Pose(Vector3(0.2, 0.3, 0.4), Quaternion()), score=0.8)
    module._grasp_generator = MagicMock(
        propose_grasps=MagicMock(
            return_value=GraspCandidateArray(Header(2.0, "link_base"), [unsafe])
        )
    )
    module._grasp_filter = MagicMock(
        inverse_kinematics_single=MagicMock(return_value=IKResult(IKStatus.NO_SOLUTION))
    )
    module.graspgenx_candidates = MagicMock()

    assert module.get_goal_pose(1) is None
    module.graspgenx_candidates.publish.assert_called_once()
    assert module.graspgenx_candidates.publish.call_args.args[0].candidates == []


def test_picknplace_returns_empty_candidates_for_obb_grasps() -> None:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        module = PickNPlaceModule()
    module.config = PickNPlaceConfig()

    assert module.get_grasp_candidates().candidates == []
