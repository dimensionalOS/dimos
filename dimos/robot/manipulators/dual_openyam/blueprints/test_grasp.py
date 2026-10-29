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

import math
import xml.etree.ElementTree as ET

import pytest

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom
from dimos.core.module import ModuleBase
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXModule
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.memory.utils.validation import validate_identifier
from dimos.robot.manipulators.dual_openyam.blueprints.agentic import (
    dual_openyam_agent_prompt,
    dual_openyam_grasp_agent,
)
from dimos.robot.manipulators.dual_openyam.blueprints.basic import DualOpenYamCoordinator
from dimos.robot.manipulators.dual_openyam.blueprints.grasp import (
    DUAL_OPENYAM_GRIPPER_TASKS,
    DUAL_OPENYAM_RECORD_TOPICS,
    DUAL_OPENYAM_VIEW_TOPICS,
    dual_openyam_grasp,
    dual_openyam_grasp_graspgenx,
    dual_openyam_grasp_model_config,
)
from dimos.visualization.rerun.bridge import RerunBridgeModule


def _atom(blueprint: Blueprint, module: type[ModuleBase]) -> BlueprintAtom:
    return next(atom for atom in blueprint.active_blueprints if atom.module is module)


def _modules(blueprint: Blueprint) -> set[type[ModuleBase]]:
    return {atom.module for atom in blueprint.active_blueprints}


def test_model_keeps_both_arms_with_a_tcp_each() -> None:
    config = dual_openyam_grasp_model_config()
    assert config.base_pose.frame_id == "world"
    assert {group.name: group.tip_link for group in config.planning_groups} == {
        "left_manipulator": "left_tcp",
        "right_manipulator": "right_tcp",
    }
    assert len(config.joint_names) == 12


@pytest.mark.self_hosted
def test_tcp_is_the_grasp_frame_turned_to_point_down_the_fingers() -> None:
    root = ET.fromstring(dual_openyam_grasp_model_config().model.load().xml)
    for side in ("left", "right"):
        joint = next(
            joint
            for joint in root.findall("joint")
            if joint.find("child").attrib["link"] == f"{side}_tcp"  # type: ignore[union-attr]
        )
        assert joint.find("parent").attrib["link"] == f"{side}_grasp_frame"  # type: ignore[union-attr]
        origin = joint.find("origin")
        assert origin is not None
        xyz = [float(v) for v in origin.attrib["xyz"].split()]
        rpy = [float(v) for v in origin.attrib["rpy"].split()]
        assert xyz[2] == pytest.approx(-0.046)
        assert rpy == pytest.approx([0.0, math.pi, 0.0])


def test_each_planning_group_is_bound_to_its_own_gripper_task() -> None:
    tasks: list[TaskConfig] = _atom(dual_openyam_grasp, DualOpenYamCoordinator).kwargs["tasks"]
    grippers = {task.name: task.joint_names for task in tasks if task.type == "gripper"}
    assert grippers == {
        "left_arm_gripper": ["left_arm/gripper"],
        "right_arm_gripper": ["right_arm/gripper"],
    }
    bound = _atom(dual_openyam_grasp, ManipulationModule).kwargs["gripper_tasks"]
    assert bound == DUAL_OPENYAM_GRIPPER_TASKS
    assert set(bound.values()) == set(grippers)
    assert not any(
        task.type == "trajectory"
        and set(task.joint_names) & {"left_arm/gripper", "right_arm/gripper"}
        for task in tasks
    )


def test_stack_is_the_xarm_grasp_stack_with_the_workcell_rig() -> None:
    manipulation = _atom(dual_openyam_grasp, ManipulationModule).kwargs
    assert manipulation["world_frame"] == "world"
    assert "floor_z" not in manipulation
    assert manipulation["static_transforms"][0].child_frame_id == "camera_link"
    pick = _atom(dual_openyam_grasp, PickAndPlaceModule).kwargs
    assert pick["planning_frame"] == "world"
    assert pick["pregrasp_offset"] == 0.05
    assert pick["grasp_verification"].empty_epsilon == pytest.approx(0.035)
    overhead = next(
        atom
        for atom in dual_openyam_grasp.active_blueprints
        if atom.module is RealSenseCamera and atom.kwargs.get("enable_pointcloud")
    )
    assert (overhead.kwargs["width"], overhead.kwargs["fps"]) == (640, 30)
    assert "height" not in overhead.kwargs
    assert dual_openyam_grasp.global_config_overrides["record"] == "sqlite"
    bridge = _atom(dual_openyam_grasp, RerunBridgeModule).kwargs
    assert bridge["topics"] == DUAL_OPENYAM_VIEW_TOPICS
    assert bridge["rerun_open"] == "none"
    # Heavy optional dependency, imported only for this check.
    import rerun.blueprint as rrb

    assert isinstance(bridge["blueprint"](), rrb.Blueprint)


def test_recorded_stream_names_are_valid_identifiers() -> None:
    assert "coordinator_joint_command" in DUAL_OPENYAM_RECORD_TOPICS.split(",")
    for name in DUAL_OPENYAM_RECORD_TOPICS.split(","):
        validate_identifier(name)


def test_agent_prompt_names_both_groups_and_their_place_squares() -> None:
    prompt = dual_openyam_agent_prompt()
    assert "left_manipulator when the object's Y is positive" in prompt
    assert "place_at 0.26 0.38 0.08 left_manipulator" in prompt
    assert "place_at 0.26 -0.38 0.08 right_manipulator" in prompt
    assert "bin" not in prompt.split("# This rig")[1]
    assert {HeuristicGraspModule, ManipulationModule} <= _modules(dual_openyam_grasp_agent)


def test_graspgenx_variant_only_swaps_the_grasp_provider() -> None:
    heuristic = _modules(dual_openyam_grasp)
    learned = _modules(dual_openyam_grasp_graspgenx)
    assert HeuristicGraspModule in heuristic and GraspGenXModule not in heuristic
    assert GraspGenXModule in learned and HeuristicGraspModule not in learned
    assert heuristic - {HeuristicGraspModule} == learned - {GraspGenXModule}
    atom = _atom(dual_openyam_grasp_graspgenx, GraspGenXModule)
    assert atom.kwargs["grasp_frame_to_tcp"][2][3] == pytest.approx(0.146)
    # Every sample goes to the pick's contact check; the attempt budget stays the default.
    assert atom.kwargs["max_candidates"] == 500
    for stack in (dual_openyam_grasp, dual_openyam_grasp_graspgenx):
        assert "max_grasp_attempts" not in _atom(stack, PickAndPlaceModule).kwargs


def test_rerun_bridge_throttles_the_image_streams() -> None:
    from dimos.visualization.rerun.bridge import RerunBridgeModule

    atom = _atom(dual_openyam_grasp, RerunBridgeModule)
    assert atom.kwargs["max_hz"]["world/color_image"] == 10.0
    assert atom.kwargs["max_hz"]["world/left_wrist_color_image"] == 3.0
    assert "world/color_image" in atom.kwargs["max_hz"]


def test_registration_is_self_cleared_for_the_overhead_camera() -> None:
    from dimos.robot.manipulators.dual_openyam.scene_registration import (
        SelfClearedSceneRegistrationModule,
    )

    for stack in (dual_openyam_grasp, dual_openyam_grasp_graspgenx):
        atom = _atom(stack, SelfClearedSceneRegistrationModule)
        assert "mask_erode_pixels" not in atom.kwargs
        assert atom.kwargs["distance_threshold"] == 0.03
        assert atom.kwargs["padding_m"] == 0.02
        xml = atom.kwargs["model"].load().xml
        assert 'name="right_base"' in xml and 'name="right_gripper"' in xml


def test_pick_checks_grasps_against_the_self_cleared_scene() -> None:
    for stack in (dual_openyam_grasp, dual_openyam_grasp_graspgenx):
        robot = _atom(stack, PickAndPlaceModule).kwargs["robot"]
        assert robot.gripper is not None
        assert robot is _atom(stack, ManipulationModule).kwargs["model"]


def test_planner_model_carries_the_gripper_config_for_the_check() -> None:
    robot = dual_openyam_grasp_model_config()
    assert robot.gripper is not None
    assert set(robot.gripper.pad_links) == {
        f"{side}_tip_{jaw}" for side in ("left", "right") for jaw in ("left", "right")
    }
    assert set(robot.gripper.finger_joints) == {
        f"{side}_finger_{jaw}_joint" for side in ("left", "right") for jaw in ("left", "right")
    }
    assert all(
        opened == -0.04695 and closed == 0.0
        for opened, closed in robot.gripper.finger_joints.values()
    )
    assert len(robot.gripper.joint_positions()) == 5
    assert {g.tip_link for g in robot.planning_groups} == {"left_tcp", "right_tcp"}


def test_planner_maps_the_overhead_cloud_like_the_xarm_stack() -> None:
    from dimos.mapping.pointclouds.box_filter import PointCloudBoxFilter
    from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
    from dimos.robot.urdf_manager_point_clearing import UrdfManagerPointClearing

    for stack in (dual_openyam_grasp, dual_openyam_grasp_graspgenx):
        voxel = _atom(stack, RayTracingVoxelMap).kwargs["voxel_size"]
        assert voxel == _atom(stack, ManipulationModule).kwargs["voxel_map_resolution"] == 0.01
        assert PointCloudBoxFilter in _modules(stack)
        clearing = _atom(stack, UrdfManagerPointClearing).kwargs
        assert clearing["padding_m"] == 0.02
        loaded = clearing["model"].load()
        xml = loaded.xml
        # The whole rig, grippers and workcell included, with the fingers fixed.
        assert 'name="left_link5"' in xml and 'name="workcell"' in xml
        assert 'name="right_gripper"' in xml and 'name="right_tip_left"' in xml
        assert all(joint.type == "fixed" for joint in loaded.joints if "finger" in joint.name)
        assert _atom(stack, RealSenseCamera).kwargs["enable_pointcloud"] is True


def test_planner_publishes_the_root_frame_the_self_filter_lifts_into() -> None:
    model = _atom(dual_openyam_grasp, ManipulationModule).kwargs["model"]
    assert model.tf_extra_links == ["dual_openyam_base"]


def test_no_model_carries_a_table() -> None:
    assert 'name="table"' not in dual_openyam_grasp_model_config().model.load().xml
