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
import numpy as np
import pytest

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom
from dimos.core.coordination.module_coordinator import stream_name_types
from dimos.core.module import ModuleBase
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXConfig, GraspGenXModule
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.memory.tap import check_topics, matching
from dimos.memory.utils.validation import validate_identifier
from dimos.robot.manipulators.dual_openyam.blueprints.basic import DualOpenYamCoordinator
from dimos.robot.manipulators.dual_openyam.blueprints.grasp import (
    DUAL_OPENYAM_RECORD_TOPICS,
    DUAL_OPENYAM_TCP_OFFSET,
    DUAL_OPENYAM_VIEW_TOPICS,
    dual_openyam_grasp,
    dual_openyam_grasp_blueprint,
    dual_openyam_grasp_model_config,
    dual_openyam_grasp_view,
)
from dimos.visualization.rerun.bridge import RerunBridgeModule


def _atom(blueprint: Blueprint, module: type[ModuleBase]) -> BlueprintAtom:
    return next(atom for atom in blueprint.active_blueprints if atom.module is module)


def _modules(blueprint: Blueprint) -> set[type[ModuleBase]]:
    return {atom.module for atom in blueprint.active_blueprints}


def test_each_arm_plans_to_its_fingertips_with_its_own_gripper() -> None:
    config = dual_openyam_grasp_model_config()

    assert config.base_pose.frame_id == "world"
    assert {group.name: group.tip_link for group in config.planning_groups} == {
        "left_manipulator": "left_tcp",
        "right_manipulator": "right_tcp",
    }
    assert {group.gripper_hardware_id for group in config.planning_groups} == {
        "left_arm",
        "right_arm",
    }
    assert config.model.load().get_joint("left_joint1") is not None


def test_grasp_blueprint_composes_both_gripper_tasks_and_the_yam_grasp_settings() -> None:
    tasks: list[TaskConfig] = _atom(dual_openyam_grasp, DualOpenYamCoordinator).kwargs["tasks"]
    assert {task.name for task in tasks if task.type == "gripper"} == {
        "left_arm_gripper",
        "right_arm_gripper",
    }
    assert _atom(dual_openyam_grasp, PickAndPlaceModule).kwargs["pregrasp_along_tool_z"] is True
    assert _atom(dual_openyam_grasp, HeuristicGraspModule).kwargs["yaw_candidates"] == 8
    manipulation = _atom(dual_openyam_grasp, ManipulationModule).kwargs
    assert manipulation["world_frame"] == "world"
    assert manipulation["static_transforms"][0].child_frame_id == "camera_link"


def test_graspgen_flag_swaps_the_grasp_provider() -> None:
    heuristic = _modules(dual_openyam_grasp_blueprint(graspgen=False))
    learned = _modules(dual_openyam_grasp_blueprint(graspgen=True))

    assert HeuristicGraspModule in heuristic
    assert GraspGenXModule not in heuristic
    assert GraspGenXModule in learned
    assert HeuristicGraspModule not in learned
    assert heuristic - {HeuristicGraspModule} == learned - {GraspGenXModule}


def test_graspgenx_gripper_matches_the_urdf_fingertips() -> None:
    kwargs = _atom(dual_openyam_grasp_blueprint(graspgen=True), GraspGenXModule).kwargs
    config = GraspGenXConfig(**kwargs)
    frame_to_tcp = np.asarray(config.grasp_frame_to_tcp)

    # Approach +Z in GraspGenX is -Z of the gripper link; the pad centre is
    # 3.7 cm beyond the grasp frame, 10 cm below the gripper link.
    assert np.allclose(frame_to_tcp[:3, 3], (0.0, 0.0, 0.1 - DUAL_OPENYAM_TCP_OFFSET[2]))
    assert np.isclose(np.linalg.det(frame_to_tcp[:3, :3]), 1.0)
    assert np.allclose(frame_to_tcp[:3, 2], (0.0, 0.0, -1.0))
    assert config.gripper.fingertip_depth > config.gripper.offset_open[2]
    assert config.gripper.extents_half_open[0] == pytest.approx(config.gripper.extents_open[0] / 2)


def test_three_cameras_and_only_the_overhead_one_feeds_perception() -> None:
    cameras = [a for a in dual_openyam_grasp.active_blueprints if a.module is RealSenseCamera]

    assert sorted(a.name for a in cameras) == [
        "left_wrist_camera",
        "realsensecamera",
        "right_wrist_camera",
    ]
    assert len({a.kwargs["serial_number"] for a in cameras}) == 3
    names = {n for n, _ in stream_name_types(dual_openyam_grasp)}
    # Perception subscribes color_image; the wrist copies carry a prefix.
    assert {"color_image", "left_wrist_color_image", "right_wrist_color_image"} <= names
    assert not {n for n in names if "/" in n}


def test_every_run_records_the_policy_training_streams() -> None:
    overrides = dual_openyam_grasp.global_config_overrides
    assert overrides["record"] == "sqlite"
    assert overrides["record_topics"] == DUAL_OPENYAM_RECORD_TOPICS

    names = {n for n, _ in stream_name_types(dual_openyam_grasp)}
    check_topics(DUAL_OPENYAM_RECORD_TOPICS, names)
    recorded = matching(DUAL_OPENYAM_RECORD_TOPICS, names)
    for required in (
        "coordinator_joint_state",
        "planned_joint_trajectory",
        "applied_joint_position_command",
        "color_image",
        "depth_image",
        "camera_info",
        "left_wrist_color_image",
        "left_wrist_depth_image",
        "right_wrist_color_image",
        "right_wrist_depth_image",
        "tf",
    ):
        assert required in recorded, required
    # Globs stay tight: no empty infrared or IMU streams in the recording.
    assert not {n for n in recorded if "infrared" in n or "imu" in n}
    # The recorder uses stream names as SQL identifiers.
    for name in recorded:
        validate_identifier(name)


def test_rerun_bridge_tiles_the_three_cameras_without_a_window_on_the_box() -> None:
    import rerun.blueprint as rrb

    bridge = _atom(dual_openyam_grasp, RerunBridgeModule).kwargs
    assert bridge["rerun_open"] == "none"
    assert bridge["blueprint"] is dual_openyam_grasp_view
    names = {n for n, _ in stream_name_types(dual_openyam_grasp)}
    assert set(DUAL_OPENYAM_VIEW_TOPICS) <= names
    assert isinstance(dual_openyam_grasp_view(), rrb.Blueprint)


def test_viewer_glyphs_build_for_proposals_target_and_path() -> None:
    import rerun as rr

    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.PoseArray import PoseArray
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.nav_msgs.Path import Path
    from dimos.msgs.std_msgs.Header import Header
    from dimos.robot.manipulators.dual_openyam.blueprints.grasp import (
        grasp_candidates_to_rerun,
        grasp_target_to_rerun,
        planned_tool_path_to_rerun,
    )

    poses = [Pose(position=(0.3, 0.1 * k, 0.05)) for k in range(10)]
    candidates = grasp_candidates_to_rerun(PoseArray(Header(1.0, "world"), poses))
    assert isinstance(candidates[0][1], rr.Clear)
    assert sum(path.endswith("/jaws") for path, _ in candidates) == 8

    target = grasp_target_to_rerun(PoseStamped(frame_id="world", position=(0.3, 0.0, 0.05)))
    assert [path for path, _ in target] == ["world/grasp_target", "world/grasp_target/jaws"]

    path = Path(
        frame_id="world",
        poses=[PoseStamped(frame_id="world", position=(0.3, 0, k)) for k in (0.1, 0.2)],
    )
    assert isinstance(planned_tool_path_to_rerun(path), rr.LineStrips3D)


def test_planning_model_carries_collision_geometry_cameras_and_measured_spacing() -> None:
    import xml.etree.ElementTree as ET

    from dimos.robot.manipulators.dual_openyam.blueprints.grasp import (
        DUAL_OPENYAM_BASE_SPACING,
        DUAL_OPENYAM_WRIST_CAMERA_BOXES,
    )

    config = dual_openyam_grasp_model_config()
    root = ET.fromstring(config.model.load().xml)
    links = {link.get("name"): link for link in root.findall("link")}

    # Every link with a visual mesh collides as its convex hull.
    for name, link in links.items():
        if link.find("visual/geometry/mesh") is not None:
            assert link.find("collision/geometry/mesh") is not None, name
    assert root.find(".//{http://drake.mit.edu}declare_convex") is not None

    # The wrist camera and its bracket are boxes on each gripper link.
    for side in ("left", "right"):
        boxes = {c.get("name") for c in links[f"{side}_gripper"].findall("collision")}
        assert {f"{side}_{name}" for name, _, _ in DUAL_OPENYAM_WRIST_CAMERA_BOXES} <= boxes

    # The bases stand where the tape says, not where the ABC bench had them.
    half = DUAL_OPENYAM_BASE_SPACING / 2
    for side, expected in (("left", half), ("right", -half)):
        origin = root.find(f"joint[@name='{side}_arm_fixed_joint']/origin")
        assert origin is not None
        xyz = origin.get("xyz")
        assert xyz is not None
        assert float(xyz.split()[1]) == pytest.approx(expected)

    assert config.home_joints == [0.0, 0.02, 0.0, 0.0, 0.0, 0.0] * 2
    assert ("left_tip_left", "left_tip_right") in config.collision_exclusion_pairs
    assert ("right_link4", "right_gripper") in config.collision_exclusion_pairs
