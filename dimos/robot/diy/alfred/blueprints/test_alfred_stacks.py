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

"""Alfred's stacks as commissioned: alfred-base, localize on top of it, manipulation on top of that,
and the driving stack beside them."""

from __future__ import annotations

import json
import math
from pathlib import Path
import re
from types import SimpleNamespace
from typing import Any, cast
import xml.etree.ElementTree as ElementTree

import numpy as np
from numpy.typing import NDArray
import pytest
from scipy.spatial.transform import Rotation

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.observe_skill import ObserveSkill
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.operator_handoff import OperatorHandoff
from dimos.control.tasks.operator_hold_task.operator_hold_task import OPERATOR_HOLD_TASK_NAME
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.coordination.module_coordinator import (
    resolve_module_refs,
    verify_no_name_conflicts,
)
from dimos.hardware.drive_trains.flowbase.config import FLOWBASE_MAX_LINEAR_CMD_M_S
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXConfig, GraspGenXModule
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_skills import ManipulationSkills
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.planning.spec.validation import MAX_OCTREE_POINTS
from dimos.mapping.pointclouds.box_filter import (
    PointCloudBoxFilter,
    inside_crop,
    rasterize_voxels,
)
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.mapping.relocalization.lidar.module import LocalMapRelocalization
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.navigation.embodiment.alfred import ALFRED
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.local_planner.native import LocalPlannerNative
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.perception.localize.module import PerceptionLocalize
from dimos.robot.diy.alfred import alfred_model
from dimos.robot.diy.alfred.alfred_model import (
    ALFRED_ARM_READY,
    ALFRED_CAMERA_IMAGER_JOINTS,
    ALFRED_FOLLOWER_ARTIFACT,
    ALFRED_FOOTPRINT_RADIUS_M,
    ALFRED_HEIGHT_M,
    ALFRED_LIFT_LOWER_M,
    ALFRED_LIFT_UPPER_M,
    ALFRED_PLANAR_BASE,
    ALFRED_SELF_FILTER_MODEL,
    ALFRED_TF_MODEL,
    ALFRED_V1_MODEL,
    alfred_arm_joints,
    alfred_base_command_gains,
    alfred_follower_artifact,
    alfred_model_config,
    alfred_planar_joint_names,
    alfred_rerun_urdf,
)
from dimos.robot.diy.alfred.blueprints import (
    alfred_manipulation_sim as sim,
    alfred_perception as perception,
)
from dimos.robot.diy.alfred.blueprints._nav import WALL_CLEARANCE_M
from dimos.robot.diy.alfred.blueprints.alfred_base import (
    ALFRED_BASE_ROBOT,
    ALFRED_GRIPPER_TASKS,
    BASE_FRAME,
    BASE_TRAJECTORY_TASK_NAME,
    BASE_VELOCITY_TASK_NAME,
    D455_FPS,
    D455_HEIGHT,
    D455_POINTCLOUD_DECIMATION,
    D455_POINTCLOUD_FPS,
    D455_WIDTH,
    DEPTH_TRUNC_M,
    FLOWBASE_DEADMAN_S,
    JOINT_TF_HZ,
    NAV_FOLLOWER_TASK_NAME,
    ODOM_FRAME,
    OPENARM_LEFT_CAN_ENV,
    OPENARM_RIGHT_CAN_ENV,
    AlfredNavCoordinator,
    AlfredOperatorCoordinator,
    _openarm_hardware_from_env,
    alfred_base,
    alfred_coordinator_tasks,
)
from dimos.robot.diy.alfred.blueprints.alfred_manipulation import (
    MANIP_MAP_CROP_MAX,
    MANIP_MAP_CROP_MIN,
    MANIP_MAP_SELF_FILTER,
    MANIP_VOXEL_M,
    MANIP_VOXEL_MAP,
    OPENARM_REACH_M,
    OPENARM_WRIST_TO_TIP_M,
    TF_TOLERANCE_S,
    alfred_manipulation,
    alfred_manipulation_heuristic,
    alfred_manipulation_modules,
    alfred_operator_modules,
)
from dimos.robot.diy.alfred.blueprints.alfred_manipulation_sim import (
    OWLV2_FLOAT32,
    SIM_LOCALIZE_PARAMS,
    alfred_manipulation_sim,
    alfred_sim_modules,
)
from dimos.robot.diy.alfred.blueprints.alfred_navigation import alfred_navigation
from dimos.robot.diy.alfred.blueprints.alfred_perception import (
    ALFRED_BENCH_PARAMS,
    PerceptionDevice,
    alfred_perception,
    cuda_torch,
    perception_device,
)
from dimos.robot.diy.alfred.blueprints.alfred_sim import alfred_sim
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel, AlfredHighLevelConfig
from dimos.robot.diy.alfred.gripper import (
    OPENARM_V10_FINGERTIP_DEPTH_M,
    OPENARM_V10_GRASP_FRAME_TO_TCP,
    OPENARM_V10_SWEEP_VOLUME,
)
from dimos.robot.diy.alfred.mount_tf import AlfredMountTf
from dimos.robot.diy.alfred.pillar_connection import (
    PILLAR_HARDWARE_ID,
    PILLAR_LIFT_JOINT,
    PILLAR_SIM_DEVICE,
    PillarConnection,
)
from dimos.robot.diy.alfred.sim_flowbase import SimFlowBase
from dimos.robot.diy.alfred.system_prompt import ALFRED_SYSTEM_PROMPT
from dimos.robot.diy.alfred.wheel_odometry_relay import WheelOdometryRelay
from dimos.robot.manipulators.openarm.config import OPENARM_HARDWARE_ID
from dimos.robot.roboplan_body_filter import RoboPlanBodyFilter
from dimos.robot.urdf_manager import UrdfManager
from dimos.robot.urdf_manager_point_clearing import UrdfManagerPointClearing
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule, MujocoSimModuleConfig
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer

_LAYERED_STACKS = [
    pytest.param(alfred_base, id="alfred-base"),
    pytest.param(alfred_perception, id="alfred-perception"),
    pytest.param(alfred_manipulation, id="alfred-manipulation"),
    pytest.param(alfred_manipulation_heuristic, id="alfred-manipulation-heuristic"),
    pytest.param(alfred_manipulation_sim, id="alfred-manipulation-sim"),
]
_MANIPULATION_STACKS = _LAYERED_STACKS[2:]


def _atoms(blueprint: Blueprint, module: type) -> list[Any]:
    return [
        atom
        for atom in blueprint.active_blueprints
        if isinstance(atom.module, type) and issubclass(atom.module, module)
    ]


def _coordinator_kwargs(blueprint: Blueprint) -> dict[str, Any]:
    (atom,) = _atoms(blueprint, ControlCoordinator)
    return cast("dict[str, Any]", atom.kwargs)


def _tasks(blueprint: Blueprint) -> dict[str, TaskConfig]:
    tasks = cast("list[TaskConfig]", _coordinator_kwargs(blueprint)["tasks"])
    return {task.name: task for task in tasks}


def _wired(blueprint: Blueprint, instance: str, port: str) -> str:
    """The stream an instance's port is on."""
    return cast("str", blueprint.remapping_map.get((instance, port), port))


def _writers(blueprint: Blueprint, stream: str) -> set[str]:
    """The instances whose outputs land on a stream."""
    return {
        atom.name
        for atom in blueprint.active_blueprints
        for port in atom.streams
        if port.direction != "in" and _wired(blueprint, atom.name, port.name) == stream
    }


def _self_filters(blueprint: Blueprint) -> tuple[Any, Any]:
    """The camera cloud's self filter, and the planner map's (the carve)."""
    filters = {atom.name: atom for atom in _atoms(blueprint, UrdfManagerPointClearing)}
    carve = filters.pop(MANIP_MAP_SELF_FILTER)
    (camera,) = filters.values()
    return camera, carve


# The urdf is LFS data, so these run where it is pulled (like test_alfred.py).
needs_lfs = pytest.mark.self_hosted


def _modules(blueprint: Blueprint) -> set[Any]:
    return {atom.module for atom in blueprint.active_blueprints}


def test_alfred_base_is_the_connections_the_camera_and_the_urdf() -> None:
    assert _modules(alfred_base) == {
        RealSenseCamera,
        AlfredHighLevel,
        WheelOdometryRelay,
        PillarConnection,
        AlfredOperatorCoordinator,
        UrdfManager,
    }


def test_perception_adds_localize_and_an_mcp_server_to_the_base() -> None:
    assert _modules(alfred_perception) - _modules(alfred_base) == {PerceptionLocalize, McpServer}
    assert _modules(alfred_base) <= _modules(alfred_perception)


def test_manipulation_adds_to_perception_and_swaps_only_the_urdf_manager() -> None:
    """The self filter is a UrdfManager, so one module poses the robot and cuts it out."""
    assert _modules(alfred_perception) - _modules(alfred_manipulation) == {UrdfManager}
    assert _modules(alfred_manipulation) - _modules(alfred_perception) == {
        UrdfManagerPointClearing,
        RayTracingVoxelMap,
        PointCloudBoxFilter,
        ManipulationModule,
        PickAndPlaceModule,
        GraspGenXModule,
        OperatorHandoff,
    }


@pytest.mark.parametrize("stack", _LAYERED_STACKS)
def test_a_layer_never_touches_the_coordinator_below_it(stack: Blueprint) -> None:
    assert _coordinator_kwargs(stack)["tasks"] == _coordinator_kwargs(alfred_base)["tasks"]


@pytest.mark.parametrize("stack", [*_LAYERED_STACKS, alfred_navigation])
def test_every_stack_wires_up(stack: Blueprint) -> None:
    """The coordinator's own checks, short of deploying: stream names agree on their types, and
    every module reference finds its one provider."""
    verify_no_name_conflicts(stack)
    resolve_module_refs(stack)


def test_the_agents_skills_wire_up_with_the_key(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("OPENAI_API_KEY", "sk-test")
    stack = autoconnect(alfred_perception, *alfred_manipulation_modules())
    stack = stack.disabled_modules(UrdfManager)
    verify_no_name_conflicts(stack)
    (skills,) = _atoms(stack, ManipulationSkills)
    (planner,) = _atoms(stack, ManipulationModule)
    assert resolve_module_refs(stack)[skills.name, "manipulation"] == planner.name


def test_the_agents_prompt_names_no_skill_the_stack_lacks() -> None:
    for gone in ("go_to", "stop_navigation", "lidar"):
        assert gone not in ALFRED_SYSTEM_PROMPT


@pytest.mark.parametrize("stack", _LAYERED_STACKS)
def test_what_a_layer_reads_has_a_writer(stack: Blueprint) -> None:
    assert _writers(stack, "coordinator_joint_state") == {"ControlCoordinator"}, "the robot on tf"
    if PerceptionLocalize in _modules(stack):
        cameras = _writers(stack, "color_image")
        assert len(cameras) == 1
        assert _writers(stack, "depth_image") == _writers(stack, "camera_info") == cameras


@pytest.mark.parametrize("stack", _MANIPULATION_STACKS)
def test_a_pick_scans_through_localize(stack: Blueprint) -> None:
    """Scene registration loaded a second OWLv2 + EdgeTAM beside localize's, which filled the
    Orin's 16 GB and swapped: one of them is the scene."""
    assert ObjectSceneRegistrationModule not in _modules(stack)
    (pick,) = _atoms(stack, PickAndPlaceModule)
    (localize,) = _atoms(stack, PerceptionLocalize)
    assert resolve_module_refs(stack)[pick.name, "_scene"] == localize.name


def test_localize_is_a_grasp_away_from_the_arm() -> None:
    """Instances in odom, like the map: PickAndPlace re-expresses a cloud in its planning frame."""
    (localize,) = _atoms(alfred_perception, PerceptionLocalize)
    assert localize.kwargs["world_frame"] == ODOM_FRAME, "objects stay put when the base drives"
    assert localize.kwargs["geometry"] == "depth" and localize.kwargs["mobile"] is False
    assert localize.kwargs["params"].min_views == 1, "a robot standing still: one viewpoint"
    assert localize.kwargs["params"].accept_score <= 0.3, "OWLv2 scores 0.3-0.6 on this bench"
    assert localize.kwargs["params"].fuse_voxel_m <= 0.002, "a grasp generator reads this cloud"
    assert localize.kwargs["detector"] == "owlv2" and localize.kwargs["segmenter"] == "edgetam"
    assert localize.kwargs["feed_frames"] <= 45, "the module's 300 frames hold 600 MB"
    (pick,) = _atoms(alfred_manipulation, PickAndPlaceModule)
    assert pick.kwargs["planning_frame"] == BASE_FRAME, "grasps where the arm plans"
    for side in ("left", "right"):
        reach = pick.kwargs["reach"][f"{side}_manipulator"]
        assert reach.shoulder_frame == f"openarm_{side}_link1", "a frame UrdfManager puts on tf"
        assert 0.40 <= reach.radius <= 0.45 and abs(reach.wrist_to_tip - 0.186) < 1e-9


@pytest.mark.parametrize("stack", [alfred_perception, alfred_manipulation])
def test_a_stack_with_models_refuses_a_cpu_torch(
    stack: Blueprint, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Dropping the models quietly left a stack with no Scan and no Pick, and nothing to say why."""
    assert cuda_torch in stack.requirement_checks
    monkeypatch.setattr(perception, "is_cuda_available", lambda: False)
    assert "CUDA" in (cuda_torch() or "")
    monkeypatch.setattr(perception, "is_cuda_available", lambda: True)
    assert cuda_torch() is None


def test_the_sim_needs_no_cuda() -> None:
    assert cuda_torch not in alfred_manipulation_sim.requirement_checks


@pytest.mark.parametrize("stack", [*_LAYERED_STACKS, alfred_navigation])
def test_every_command_goes_through_the_coordinators_ports(stack: Blueprint) -> None:
    """The pillar and the FlowBase are connection modules: the coordinator arbitrates its tasks
    and is the only writer of their commands, on real ports, with nothing pinned by hand. The
    FlowBase's module stays the only thing that talks to its Pi."""
    (coordinator,) = _atoms(stack, ControlCoordinator)
    base = next(iter(_atoms(stack, AlfredHighLevel) or _atoms(stack, SimFlowBase)))
    (pillar,) = _atoms(stack, PillarConnection)
    assert _wired(stack, base.name, "cmd_vel") == "flowbase_cmd_vel"
    assert _wired(stack, pillar.name, "position_command") == "pillar_position_command"
    assert _wired(stack, pillar.name, "joint_state") == "pillar_joint_state"
    assert _writers(stack, "flowbase_cmd_vel") == {coordinator.name}
    assert _writers(stack, "pillar_position_command") == {coordinator.name}
    assert _writers(stack, "pillar_joint_state") == {pillar.name}
    assert len(_writers(stack, "flowbase_odom")) == 1, "one base pose"
    assert not any(
        hw.hardware_id in ("pillar", "flowbase") for hw in _coordinator_kwargs(stack)["hardware"]
    )
    assert not stack.transport_map, "no stream goes round the blueprint"


@pytest.mark.parametrize("stack", _LAYERED_STACKS)
def test_the_arms_stay_on_their_adapter(stack: Blueprint) -> None:
    """Their CAN bus runs in the coordinator's process, on its tick: over a transport a read
    that stalls 0.3 s would outlast the reading timeout, and the arms would go uncommanded."""
    (arms,) = _coordinator_kwargs(stack)["hardware"]
    assert arms.hardware_id == OPENARM_HARDWARE_ID


def test_the_coordinator_knows_the_lifts_joint_from_the_model() -> None:
    joints = {joint.name for joint in ALFRED_V1_MODEL.load().joints}
    assert PILLAR_LIFT_JOINT in joints
    assert _coordinator_kwargs(alfred_base)["model"] is ALFRED_V1_MODEL


def test_an_e_stop_stops_the_base_on_its_deadman_within_a_tenth() -> None:
    """An e-stopped coordinator sends nothing more, by design, so this is the base's stop."""
    (base,) = _atoms(alfred_base, AlfredHighLevel)
    assert base.kwargs["deadman_timeout_s"] == FLOWBASE_DEADMAN_S <= 0.1


def test_the_stacks_without_a_coordinator_keep_a_slower_deadman() -> None:
    """A MovementManager streams far slower than a coordinator's tick."""
    assert AlfredHighLevelConfig().deadman_timeout_s >= 0.2


def test_teleop_enters_the_coordinator_not_the_base() -> None:
    """Whatever publishes tele_cmd_vel (the Controller, the web viewer) is one base claim."""
    assert alfred_base.remapping_map[("ControlCoordinator", "twist_command")] == "tele_cmd_vel"


def test_base_claims_rank_teleop_over_plan_over_navigation() -> None:
    tasks = _tasks(alfred_navigation)
    teleop = tasks[BASE_VELOCITY_TASK_NAME]
    plan = tasks[BASE_TRAJECTORY_TASK_NAME]
    nav = tasks[NAV_FOLLOWER_TASK_NAME]
    base_joints = [f"{ALFRED_BASE_ROBOT}/{axis}" for axis in ("vx", "vy", "wz")]
    for task in (teleop, plan, nav):
        assert task.joint_names == base_joints
    assert teleop.priority > plan.priority > nav.priority
    assert teleop.type == "velocity"
    assert plan.type == "planar_base_trajectory"
    assert plan.params["relative"] is True, "a plan's base segment is a motion from here"
    assert nav.type == "holonomic_pose_follower"
    claimants = [name for name, task in tasks.items() if set(task.joint_names) & set(base_joints)]
    assert sorted(claimants) == sorted(
        [BASE_VELOCITY_TASK_NAME, BASE_TRAJECTORY_TASK_NAME, NAV_FOLLOWER_TASK_NAME]
    )


def test_the_follower_runs_on_alfreds_measured_plant() -> None:
    """The artifact is config, not a build output: losing it degrades silently to Go2 gains."""
    nav = _tasks(alfred_navigation)[NAV_FOLLOWER_TASK_NAME]
    assert nav.params["artifact_path"] == ALFRED_FOLLOWER_ARTIFACT
    artifact = json.loads(Path(ALFRED_FOLLOWER_ARTIFACT).read_text())
    assert artifact["provenance"]["robot_id"] == "alfred"
    assert artifact["valid_for_tuning"]
    assert all(artifact["plant"][axis]["K"] > 0 for axis in ("vx", "vy", "wz"))
    assert alfred_follower_artifact() == ALFRED_FOLLOWER_ARTIFACT


def test_the_follower_falls_back_to_the_go2_artifact_until_alfred_has_one(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A robot that has never been characterized must still start - loudly."""
    missing = tmp_path / "alfred_posedomain.json"
    monkeypatch.setattr(alfred_model, "ALFRED_FOLLOWER_ARTIFACT", str(missing))
    assert alfred_model.alfred_follower_artifact() == alfred_model._GO2_FOLLOWER_ARTIFACT
    missing.write_text("{}")
    assert alfred_model.alfred_follower_artifact() == str(missing)


def test_one_trajectory_task_covers_the_lift_and_both_arms_with_the_hold_under_it() -> None:
    tasks = _tasks(alfred_base)
    trajectory = tasks[JOINT_TRAJECTORY_TASK_NAME]
    hold = tasks["hold_arms"]
    assert trajectory.joint_names == [*alfred_arm_joints(), PILLAR_LIFT_JOINT]
    assert set(trajectory.params["velocity_limits"]) == set(trajectory.joint_names)
    assert hold.type == "joint_hold"
    assert hold.joint_names == alfred_arm_joints(), (
        "the lift holds itself: a leadscrew with a brake"
    )
    assert hold.priority < trajectory.priority
    assert hold.params["always"] is True, "Damiao motors are torque-free until commanded"
    for task_name in ALFRED_GRIPPER_TASKS.values():
        gripper = tasks[task_name]
        assert gripper.type == "gripper"
        assert gripper.joint_names in (["left_arm/gripper"], ["right_arm/gripper"])
        assert gripper.priority > trajectory.priority, "a hand command beats a trajectory's hold"
    (planner,) = _atoms(alfred_manipulation, ManipulationModule)
    assert planner.kwargs["gripper_tasks"] == ALFRED_GRIPPER_TASKS, "one hand per arm"


def test_the_whole_body_is_planned_in_the_robots_own_frame() -> None:
    """Targets are poses on the robot (base_link), so they ride with it and never depend on
    where odometry woke up or whether it jumps; "world" is an identity edge under
    base_link for the Controller and the stock tooling. The planar base joints are the base's
    motion from here: held at zero, never fed from odometry, executed as a relative trajectory."""
    (atom,) = _atoms(alfred_manipulation, ManipulationModule)
    assert atom.kwargs["world_frame"] == BASE_FRAME
    (world,) = atom.kwargs["static_transforms"]
    assert (world.frame_id, world.child_frame_id) == (BASE_FRAME, "world")
    assert (world.translation.x, world.translation.y, world.translation.z) == (0.0, 0.0, 0.0)
    assert list(atom.kwargs["model"].joint_names) == alfred_planar_joint_names()
    assert "joint_state_aliases" not in atom.kwargs
    assert atom.kwargs["joint_state_defaults"] == dict.fromkeys(ALFRED_PLANAR_BASE.joint_names, 0.0)
    assert atom.kwargs["trajectory_tasks"] == {
        JOINT_TRAJECTORY_TASK_NAME: [*alfred_arm_joints(), PILLAR_LIFT_JOINT],
        BASE_TRAJECTORY_TASK_NAME: list(ALFRED_PLANAR_BASE.joint_names),
    }
    assert atom.kwargs["visualization"] == {"backend": "none"}, "the Controller is the arm UI"


def test_the_arms_are_mock_until_both_can_ports_are_set(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(OPENARM_LEFT_CAN_ENV, raising=False)
    monkeypatch.delenv(OPENARM_RIGHT_CAN_ENV, raising=False)
    assert _openarm_hardware_from_env().adapter_type == "mock_whole_body"
    monkeypatch.setenv(OPENARM_LEFT_CAN_ENV, "can2")
    monkeypatch.setenv(OPENARM_RIGHT_CAN_ENV, "can3")
    assert _openarm_hardware_from_env().adapter_type == "openarm_damiao"


def test_the_planner_gets_the_real_height_and_a_footprint_under_the_robot() -> None:
    """robot_height decides whether a cell is floor at all (surfaces.rs is_standable), so
    tables and sub-2 m doorways read as blocked; wall_clearance_m is knowingly below the
    footprint radius - 0.2 plans on the robot, 0.28 cannot."""
    (planner,) = _atoms(alfred_navigation, MLSPlannerNative)
    assert planner.kwargs["robot_height"] == ALFRED.height == ALFRED_HEIGHT_M
    assert planner.kwargs["wall_clearance_m"] == WALL_CLEARANCE_M
    assert WALL_CLEARANCE_M < ALFRED_FOOTPRINT_RADIUS_M


def test_the_planner_gets_a_filtered_d455_map() -> None:
    """A short-range voxel map under its own name, so a nav map's streams would stay put."""
    remaps = alfred_manipulation.remapping_map
    assert remaps[(MANIP_VOXEL_MAP, "lidar")] == "filtered_pointcloud"
    assert remaps[(MANIP_VOXEL_MAP, "global_map")] == "manip_global_map"
    nav_map_outputs = (
        "global_map",
        "local_map",
        "local_map_fine",
        "region_bounds",
        "seed_map",
        "seed_bounds",
        "map_regions",
    )
    for port in nav_map_outputs:
        assert remaps[(MANIP_VOXEL_MAP, port)] == f"manip_{port}", f"{port} is the nav map's"
    assert remaps[(PointCloudBoxFilter.name, "cloud")] == "manip_global_map"
    carve_input = remaps[(MANIP_MAP_SELF_FILTER, "pointcloud")]
    assert remaps[(PointCloudBoxFilter.name, "filtered_cloud")] == carve_input, (
        "on to the planner through the carve"
    )
    (manip_map,) = _atoms(alfred_manipulation, RayTracingVoxelMap)
    assert manip_map.kwargs["world_frame"] == ODOM_FRAME, "a drive must not move the bench"
    assert manip_map.kwargs["tf_match_tolerance_s"] >= TF_TOLERANCE_S, "a stalled joint state"
    assert manip_map.kwargs["max_range"] <= 1.5, "the bench, not the floor beyond it"
    assert manip_map.kwargs["voxel_size"] == MANIP_VOXEL_M <= 0.02, (
        "at 5 cm the bench's layer reached 3 cm above its surface, in the way of a low grasp"
    )
    assert manip_map.kwargs["grace_depth"] <= 0.05, "a picked object's voxels must clear"
    assert manip_map.kwargs["graze_cos"] <= 0.5, (
        "a removed can's top is seen as shallow as the bench"
    )
    assert manip_map.kwargs["shadow_depth"] == 0.0, "a shadow carves the layer under the bench top"
    assert manip_map.kwargs["global_emit_every"] >= 5, "the planner rebuilds its octree per map"
    (box_filter,) = _atoms(alfred_manipulation, PointCloudBoxFilter)
    assert box_filter.kwargs["target_frame"] == BASE_FRAME, "the planner drops any other frame"
    assert box_filter.kwargs["tf_tolerance_s"] >= TF_TOLERANCE_S, (
        "odom -> base_link lands at 47 Hz; the map's stamp is a 5 Hz camera's"
    )
    (planner,) = _atoms(alfred_manipulation, ManipulationModule)
    assert planner.kwargs["world_frame"] == box_filter.kwargs["target_frame"]
    assert planner.kwargs["voxel_map_resolution"] == manip_map.kwargs["voxel_size"], (
        "the octree's cells are the map's voxels"
    )
    assert box_filter.kwargs["target_voxel_m"] == planner.kwargs["voxel_map_resolution"], (
        "the map is re-rasterized onto the octree's own cells, or a turned base leaves it holes"
    )
    camera_filter, _ = _self_filters(alfred_manipulation)
    assert camera_filter.kwargs["padding_m"] >= 0.044, "an open finger is 4.4 cm outside the model"
    (camera,) = _atoms(alfred_manipulation, RealSenseCamera)
    assert camera.kwargs["enable_pointcloud"] and camera.kwargs["enable_color"]


@pytest.mark.parametrize("stack", _MANIPULATION_STACKS)
def test_the_planner_map_has_the_robot_carved_out(stack: Blueprint) -> None:
    """A second self filter between the box filter and the planner drops every cell the robot
    touches: the map remembers the bench edge, and a hand hovering just over it overlapped two
    cells of the bench's layer, so every plan failed COLLISION_AT_START (2026-10-26)."""
    camera_filter, carve = _self_filters(stack)
    assert _writers(stack, "voxel_map") == {MANIP_MAP_SELF_FILTER}, "the planner's map is carved"
    (box_filter,) = _atoms(stack, PointCloudBoxFilter)
    uncarved = _wired(stack, box_filter.name, "filtered_cloud")
    assert _wired(stack, carve.name, "pointcloud") == uncarved != "voxel_map"
    assert _writers(stack, uncarved) == {box_filter.name}
    assert _writers(stack, _wired(stack, MANIP_VOXEL_MAP, "lidar")) == {camera_filter.name}, (
        "the map is built from the camera's cloud, not from itself"
    )
    (planner,) = _atoms(stack, ManipulationModule)
    half_diagonal = planner.kwargs["voxel_map_resolution"] * math.sqrt(3) / 2
    assert half_diagonal <= carve.kwargs["padding_m"] <= 0.025, (
        "a cell the robot touches has its centre within half its diagonal; more padding opens "
        "the obstacle around the hand for its next motion"
    )
    assert carve.kwargs["model"] == camera_filter.kwargs["model"], "the planner's own model"
    assert carve.kwargs["publish_joint_tf"] is False, "one module owns the robot on tf"
    for port in ("render_file_request", "file_info"):
        assert _wired(stack, carve.name, port) != port, "one module answers for the URDF's files"
    for tolerance in ("state_tolerance_s", "tf_tolerance_s", "tf_forward_tolerance_s"):
        assert carve.kwargs[tolerance] == camera_filter.kwargs[tolerance], tolerance
    assert box_filter.kwargs["target_frame"] == BASE_FRAME, "the model's root: no tf to wait on"


@pytest.mark.parametrize("stack", _MANIPULATION_STACKS)
def test_the_planner_map_is_cropped_to_the_arms_reach(stack: Blueprint) -> None:
    """The map is kept in odom and remembers everything the camera saw as the base drove round
    the room: at 200148 points the planner refused every map and planned on one hours stale
    (2026-10-26). The box filter crops it around the robot, in the planner's frame."""
    (box_filter,) = _atoms(stack, PointCloudBoxFilter)
    assert box_filter.kwargs["target_frame"] == BASE_FRAME, "the crop moves with the robot"
    assert box_filter.kwargs["crop_min"] == MANIP_MAP_CROP_MIN
    assert box_filter.kwargs["crop_max"] == MANIP_MAP_CROP_MAX
    (planner,) = _atoms(stack, ManipulationModule)
    assert box_filter.kwargs["target_voxel_m"] == planner.kwargs["voxel_map_resolution"]
    assert _wired(stack, box_filter.name, "cloud") == _wired(stack, MANIP_VOXEL_MAP, "global_map")


# Where a whole-body pick may leave the base in odom: turned, and off the voxel grid.
_ROOM_BASE_AT = (0.3137, -0.1171, 0.0)
_ROOM_BASE_YAW_DEG = 42.0


def _room(walls: bool) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """A 2 cm odom map holding all a crop could: a 10 m square of floor, already more voxels than
    the planner takes, a bench top 0.75 m up across the crop's front, and with ``walls`` a wall
    just inside each side of the crop, floor to top (the robot in a crop-sized room). Returns
    the voxel centres in odom and base_link's pose there."""
    odom_from_base = np.eye(4)
    odom_from_base[:3, :3] = Rotation.from_euler("z", _ROOM_BASE_YAW_DEG, degrees=True).as_matrix()
    odom_from_base[:3, 3] = _ROOM_BASE_AT
    i, j = np.meshgrid(np.arange(-250, 250), np.arange(-250, 250), indexing="ij")
    floor = np.stack((i.ravel(), j.ravel(), np.zeros(i.size, dtype=np.int64)), axis=1)
    lo = np.asarray(MANIP_MAP_CROP_MIN) + MANIP_VOXEL_M / 2
    hi = np.asarray(MANIP_MAP_CROP_MAX) - MANIP_VOXEL_M / 2

    def sheet(axis: int, at: float, a: tuple[float, float], b: tuple[float, float]) -> Any:
        """Points every half voxel over a rectangle across the other two axes."""
        u, v = np.meshgrid(
            np.arange(*a, MANIP_VOXEL_M / 2), np.arange(*b, MANIP_VOXEL_M / 2), indexing="ij"
        )
        return np.insert(np.stack((u.ravel(), v.ravel()), axis=1), axis, at, axis=1)

    sheets = [sheet(2, 0.75, (0.4, hi[0]), (lo[1], hi[1]))]
    if walls:
        for x in (lo[0], hi[0]):
            sheets.append(sheet(0, x, (lo[1], hi[1]), (0.0, hi[2])))
        for y in (lo[1], hi[1]):
            sheets.append(sheet(1, y, (lo[0], hi[0]), (0.0, hi[2])))
    in_odom = np.concatenate(sheets) @ odom_from_base[:3, :3].T + odom_from_base[:3, 3]
    surfaces = np.floor(in_odom / MANIP_VOXEL_M).astype(np.int64)
    keys = np.unique(np.concatenate((floor, surfaces)), axis=0)
    return (keys + 0.5) * MANIP_VOXEL_M, odom_from_base


@pytest.mark.parametrize(
    ("walls", "share"),
    [pytest.param(False, 0.25, id="floor-and-bench"), pytest.param(True, 0.6, id="walled-in")],
)
def test_a_full_crop_reaches_the_planner_well_under_its_limit(walls: bool, share: float) -> None:
    """Whatever the room, the planner gets what the crop holds, re-rasterized: a floor and a
    bench at 2 cm are ~22k cells, and boxed in by four walls as well ~110k (the 2 x 2 x 2
    lattice puts a turned wall's voxel in 1.5 cells)."""
    centres, odom_from_base = _room(walls)
    base_from_odom = np.linalg.inv(odom_from_base)
    in_base = centres @ base_from_odom[:3, :3].T + base_from_odom[:3, 3]
    kept = centres[inside_crop(in_base, MANIP_MAP_CROP_MIN, MANIP_MAP_CROP_MAX)]
    cells = rasterize_voxels(kept, base_from_odom, MANIP_VOXEL_M)
    assert len(centres) > MAX_OCTREE_POINTS, "the room alone is over the limit"
    assert len(cells) < share * MAX_OCTREE_POINTS, f"{len(cells)} cells"


def _link_origin(root: ElementTree.Element, link: str, lift_m: float) -> NDArray[np.float64]:
    """A link's origin in the urdf's root, the lift at ``lift_m`` and the other joints at zero
    (a revolute joint turns its child about the child's own origin, so it moves no origin)."""
    joints = {
        joint.find("child").attrib["link"]: joint  # type: ignore[union-attr]
        for joint in root.findall("joint")
    }
    pose = np.eye(4)
    while link in joints:
        joint = joints[link]
        origin = joint.find("origin")
        step = np.eye(4)
        if origin is not None:
            rpy = [float(v) for v in origin.attrib.get("rpy", "0 0 0").split()]
            step[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
            step[:3, 3] = [float(v) for v in origin.attrib.get("xyz", "0 0 0").split()]
        if joint.attrib["type"] == "prismatic":
            axis = joint.find("axis").attrib["xyz"]  # type: ignore[union-attr]
            step[:3, 3] += step[:3, :3] @ np.multiply([float(v) for v in axis.split()], lift_m)
        pose = step @ pose
        link = joint.find("parent").attrib["link"]  # type: ignore[union-attr]
    assert link == BASE_FRAME
    return pose[:3, 3]


@needs_lfs
def test_the_planner_map_crop_holds_both_arms_reach_with_the_base_helping() -> None:
    """Each shoulder, over the lift's range, with its reach to the fingertips around it: all of
    it inside the crop, with 0.8 m of base drive ahead and 0.5 m to either side."""
    root = ElementTree.fromstring(ALFRED_V1_MODEL.load().xml)
    reach = OPENARM_REACH_M + OPENARM_WRIST_TO_TIP_M
    crop_min, crop_max = np.asarray(MANIP_MAP_CROP_MIN), np.asarray(MANIP_MAP_CROP_MAX)
    for side in ("left", "right"):
        for lift in (ALFRED_LIFT_LOWER_M, ALFRED_LIFT_UPPER_M):
            shoulder = _link_origin(root, f"openarm_{side}_link1", lift)
            assert np.all(crop_min <= shoulder - reach), (side, lift, shoulder)
            assert np.all(shoulder + reach <= crop_max), (side, lift, shoulder)
            assert shoulder[0] + reach + 0.8 <= crop_max[0], "the base drives to the bench"
            assert abs(shoulder[1]) + reach + 0.5 <= min(-crop_min[1], crop_max[1])


def test_the_bench_stack_runs_without_the_lidar() -> None:
    """alfred-manipulation: arms, lift, planner, camera; the base pose from wheel odometry."""
    modules = _modules(alfred_manipulation)
    assert {
        AlfredHighLevel,
        WheelOdometryRelay,
        PillarConnection,
        ManipulationModule,
        AlfredOperatorCoordinator,
        RealSenseCamera,
        UrdfManagerPointClearing,
    } <= modules
    assert not {PointLio, MLSPlannerNative, LocalPlannerNative, LocalMapRelocalization} & modules
    tasks = _tasks(alfred_manipulation)
    assert NAV_FOLLOWER_TASK_NAME not in tasks, "no path to follow here"
    assert {BASE_VELOCITY_TASK_NAME, BASE_TRAJECTORY_TASK_NAME, "hold_arms"} <= set(tasks)
    assert alfred_manipulation.remapping_map[("ControlCoordinator", "twist_command")] == (
        "tele_cmd_vel"
    )
    (relay,) = _atoms(alfred_manipulation, WheelOdometryRelay)
    assert (relay.kwargs["world_frame"], relay.kwargs["base_frame"]) == ("odom", BASE_FRAME)
    (voxel_map,) = _atoms(alfred_manipulation, RayTracingVoxelMap)
    (planner,) = _atoms(alfred_manipulation, ManipulationModule)
    assert planner.kwargs["voxel_map_resolution"] == voxel_map.kwargs["voxel_size"]


@pytest.mark.parametrize("blueprint", _LAYERED_STACKS)
def test_an_operator_can_hold_every_layer(blueprint: Blueprint) -> None:
    """The operator hold outranks every claim, so it freezes arms, grippers, lift and base."""
    tasks = _tasks(blueprint)
    hold = tasks[OPERATOR_HOLD_TASK_NAME]
    assert hold.type == "operator_hold" and not hold.joint_names, "it claims every joint"
    assert hold.params["hold_through_estop"], "teleop's velocity task ignores the e-stop"
    assert all(task.priority < hold.priority for task in tasks.values() if task is not hold)
    assert AlfredOperatorCoordinator in _modules(blueprint)
    assert AlfredNavCoordinator in AlfredOperatorCoordinator.__mro__, "the follower's ports stay"


def test_manipulation_has_the_handoff_and_every_skill_on_mcp() -> None:
    assert {OperatorHandoff, McpServer} <= _modules(alfred_manipulation)


def test_the_agent_joins_the_bench_stacks_only_with_a_key(monkeypatch: pytest.MonkeyPatch) -> None:
    def modules() -> set[Any]:
        return {atom.module for part in alfred_operator_modules() for atom in part.blueprints}

    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    assert McpClient not in modules()
    monkeypatch.setenv("OPENAI_API_KEY", "sk-test")
    assert {McpClient, ManipulationSkills, ObserveSkill} <= modules()
    (client,) = [
        atom
        for part in alfred_operator_modules()
        for atom in part.blueprints
        if atom.module is McpClient
    ]
    assert client.kwargs["system_prompt"] is ALFRED_SYSTEM_PROMPT


def test_the_driving_stack_runs_without_the_arms() -> None:
    """alfred-navigation: the lidar chain, teleop and the follower; the pillar parked, no planner."""
    modules = {atom.module for atom in alfred_navigation.blueprints}
    assert {
        PointLio,
        MLSPlannerNative,
        LocalPlannerNative,
        AlfredNavCoordinator,
        PillarConnection,
    } <= (modules)
    assert not {ManipulationModule, RealSenseCamera, WheelOdometryRelay} & modules
    assert (
        "hardware" not in _coordinator_kwargs(alfred_navigation)
        or not (_coordinator_kwargs(alfred_navigation)["hardware"])
    ), "no arms' CAN"
    tasks = _tasks(alfred_navigation)
    assert {BASE_VELOCITY_TASK_NAME, NAV_FOLLOWER_TASK_NAME} <= set(tasks)
    assert "hold_arms" not in tasks
    assert alfred_navigation.remapping_map[(MLSPlannerNative.name, "goal")] == "clicked_point"


def test_alfred_sim_still_composes() -> None:
    hardware_ids = {hw.hardware_id for hw in _coordinator_kwargs(alfred_sim)["hardware"]}
    assert {PILLAR_HARDWARE_ID, OPENARM_HARDWARE_ID} <= hardware_ids


def test_alfred_model_uses_the_pillar_joint_convention() -> None:
    """Lift is negative below the top switch, exactly as pillar_connection reports it."""
    config = alfred_model_config()
    assert config.joint_names[0] == PILLAR_LIFT_JOINT
    assert ALFRED_LIFT_LOWER_M == -0.5
    assert ALFRED_LIFT_UPPER_M == -0.002
    groups = {group.name: group for group in config.planning_groups}
    assert groups["lift"].joint_names == (PILLAR_LIFT_JOINT,)
    assert set(groups) == {"lift", "left_manipulator", "right_manipulator"}


@needs_lfs
def test_home_pose_is_inside_every_joint_limit() -> None:
    """The lift's zero is the top switch, above its reachable range; Home must not send it."""
    for wheels in (False, True):
        config = alfred_model_config(wheels=wheels)
        limits = {j.name: j for j in config.model.load().joints}
        assert config.home_joints is not None
        for name, value in zip(config.joint_names, config.home_joints, strict=True):
            joint = limits[name]
            if joint.lower is not None and joint.upper is not None:
                assert joint.lower <= value <= joint.upper, f"{name}: home {value} outside limits"


@needs_lfs
def test_rerun_urdf_is_materialized_with_resolved_meshes_and_coordinator_joints() -> None:
    xml = alfred_rerun_urdf().read_text()
    assert "package://" not in xml
    assert 'name="pillar/lift"' in xml, "renamed lift joint must reach the rerun model"
    meshes = re.findall(r'filename="([^"]+)"', xml)
    assert meshes and all(Path(m).is_file() for m in meshes), "unresolved mesh path"


@pytest.mark.parametrize("stack", _LAYERED_STACKS)
def test_one_urdf_manager_owns_the_robot_on_tf(stack: Blueprint) -> None:
    """The mounts, the lift and the arms, from the joint state; no planner has to publish them
    (alfred-base has none), and no static mount tree runs beside it."""
    assert AlfredMountTf not in _modules(stack)
    (urdf,) = [atom for atom in _atoms(stack, UrdfManager) if atom.name != MANIP_MAP_SELF_FILTER]
    assert urdf.kwargs.get("publish_joint_tf", True) is True
    assert urdf.kwargs["model"] in (ALFRED_TF_MODEL, ALFRED_SELF_FILTER_MODEL)
    assert 1.0 / urdf.kwargs["unchanged_tf_hz"] < 2 * 0.12, "localize's tf tolerance"
    assert urdf.kwargs["joint_tf_hz"] == JOINT_TF_HZ <= 10.0, "30 Hz is a third of the Orin's core"


@pytest.mark.parametrize("stack", _MANIPULATION_STACKS)
def test_the_planner_leaves_the_links_to_the_urdf_manager(stack: Blueprint) -> None:
    (planner,) = _atoms(stack, ManipulationModule)
    assert not planner.kwargs["model"].tf_extra_links


def test_the_driving_stack_keeps_its_tf_tree_under_the_lidar() -> None:
    """Point-LIO owns the lidar's parent edge, so UrdfManager only serves the XML and meshes."""
    (urdf,) = _atoms(alfred_navigation, UrdfManager)
    assert urdf.kwargs["publish_joint_tf"] is False
    (mounts,) = _atoms(alfred_navigation, AlfredMountTf)
    assert mounts.kwargs["root_frame"] == "mid360_link"


@needs_lfs
def test_the_urdf_manager_poses_the_robot_as_the_planner_does_and_cuts_it_out() -> None:
    """Off the coordinator's joint state as it arrives, grippers and base included: tf reaches
    each hand where RoboPlan's forward kinematics puts it, and the filter drops the arm from a
    cloud and keeps the bench."""
    positions = {**dict.fromkeys(alfred_arm_joints(), 0.0), **ALFRED_ARM_READY}
    positions[PILLAR_LIFT_JOINT] = -0.2
    reported = {**positions, "left_arm/gripper": 1.0, "flowbase/vx": 0.1}
    state = JointState(ts=1.0, name=list(reported), position=list(reported.values()))
    model = ALFRED_SELF_FILTER_MODEL.load()
    # Off the method, not a module: constructing one opens an RPC session.
    manager: Any = SimpleNamespace(_joints={joint.name: joint for joint in model.joints})
    parent = {edge.child_frame_id: edge for edge in UrdfManager.joint_transforms(manager, state)}

    def on_tf(link: str) -> NDArray[np.float64]:
        pose = np.eye(4)
        while link in parent:
            pose = parent[link].to_matrix() @ pose
            link = parent[link].frame_id
        assert link == BASE_FRAME
        return pose

    backend = RoboPlanBodyFilter(model, padding_m=0.05)
    q = backend.configuration(reported, {})
    for tip in ("openarm_left_grasp_frame", "openarm_right_grasp_frame", "camera_front_link"):
        kinematics = np.asarray(backend._context.forwardKinematics(q, tip, BASE_FRAME))
        assert np.allclose(on_tf(tip), kinematics, atol=1e-6), tip
    elbow = on_tf("openarm_left_link4")[:3, 3]
    bench = np.array([1.0, 0.0, 0.75])
    keep = backend.keep_mask(np.array([elbow, bench]), reported, {})
    assert keep is not None and keep.tolist() == [False, True]


@needs_lfs
def test_the_tf_model_leaves_the_imagers_to_the_camera_driver() -> None:
    """With rotate_180 the driver's optical frames are half a turn from the urdf's: an edge from
    both would flip the camera's pose on tf from one sample to the next."""
    full = {joint.name for joint in ALFRED_V1_MODEL.load().joints}
    assert set(ALFRED_CAMERA_IMAGER_JOINTS) <= full, "names off the urdf"
    loaded = ALFRED_TF_MODEL.load()
    links = {joint.child_link for joint in loaded.joints}
    assert {"camera_front_link", "camera_back_link", "mid360_link"} <= links
    assert not [link for link in links if "optical" in link or link.endswith("_imu_frame")]
    assert all(
        joint.parent_link not in ("camera_front_link", "camera_back_link")
        for joint in loaded.joints
    )
    moving = {joint.name for joint in loaded.joints if joint.type != "fixed"}
    assert {PILLAR_LIFT_JOINT, *alfred_arm_joints()} <= moving
    filtered = {j.name for j in ALFRED_SELF_FILTER_MODEL.load().joints if j.type != "fixed"}
    assert filtered == {PILLAR_LIFT_JOINT, *alfred_arm_joints()}, (
        "the joints the coordinator reports"
    )


def test_a_click_is_the_goal() -> None:
    """The Controller's "Go here" publishes clicked_point; with no MovementManager to relay it,
    the planner listens there itself."""
    assert alfred_navigation.remapping_map[(MLSPlannerNative.name, "goal")] == "clicked_point"


def test_mock_arms_start_in_the_ready_pose(monkeypatch: pytest.MonkeyPatch) -> None:
    """All-zero arms hang straight down, a singular pose the planner's IK cannot leave."""
    monkeypatch.delenv(OPENARM_LEFT_CAN_ENV, raising=False)
    monkeypatch.delenv(OPENARM_RIGHT_CAN_ENV, raising=False)
    hardware = _openarm_hardware_from_env()
    start = dict(zip(hardware.joints, hardware.adapter_kwargs["initial_positions"], strict=True))
    assert {name: start[name] for name in ALFRED_ARM_READY} == ALFRED_ARM_READY
    assert all(value == 0.0 for name, value in start.items() if name not in ALFRED_ARM_READY)
    home = dict(
        zip(alfred_model_config().joint_names, alfred_model_config().home_joints or [], strict=True)
    )
    assert {name: home[name] for name in ALFRED_ARM_READY} == ALFRED_ARM_READY, (
        "home is the ready pose"
    )


@needs_lfs
def test_an_arm_plans_on_the_planar_whole_body_model() -> None:
    """RoboPlan's native RRT rejects every start on a model with the planar base's line and
    circle joints ("Invalid start configuration"), whatever the selection; the planner must
    route such a model to the shared Python RRT, which bounds them itself."""
    from dimos.manipulation.planning.planners.roboplan_config import RoboPlanPlannerConfig
    from dimos.manipulation.planning.planners.roboplan_planner import RoboPlanPlanner
    from dimos.manipulation.planning.spec.validation import prepare_robot_model
    from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld
    from dimos.msgs.sensor_msgs.JointState import JointState
    from dimos.robot.diy.alfred.alfred_model import alfred_planar_model_config

    config = alfred_planar_model_config()
    world = RoboPlanWorld()
    world.load_model(prepare_robot_model(config))
    world.finalize()
    assert config.home_joints is not None
    world.sync_from_joint_state(
        JointState(name=list(config.joint_names), position=config.home_joints)
    )
    planner = RoboPlanPlanner(world, RoboPlanPlannerConfig())
    selection = world._planning_groups.select(("left_manipulator",))
    assert planner._selection_requires_shared_planner(selection)

    left = [name for name in config.joint_names if name.startswith("openarm_left")]
    home = dict(zip(config.joint_names, config.home_joints, strict=True))
    start = JointState(name=left, position=[home[name] for name in left])
    goal = JointState(
        name=left,
        position=[home[name] + (0.2 if name.endswith("joint1") else 0.0) for name in left],
    )
    result = planner.plan_selected_joint_path(
        world=world, selection=selection, start=start, goal=goal, timeout=5.0
    )
    assert result.is_success(), result.message


def test_graspgenx_sees_the_v10_gripper_the_urdf_carries() -> None:
    """GraspGenX's gripper frame is the wrist turned half a turn about x (approach +z, jaws
    along x); its sweep volume and the step to grasp_frame come from the URDF's numbers."""
    config = GraspGenXConfig.model_validate(
        {"gripper": OPENARM_V10_SWEEP_VOLUME, "grasp_frame_to_tcp": OPENARM_V10_GRASP_FRAME_TO_TCP}
    )
    # The URDF: ee_base_link -> grasp_frame is rpy (pi, 0, pi/2) at (0, 0, -0.186).
    wrist_to_tcp = np.eye(4)
    wrist_to_tcp[:3, :3] = Rotation.from_euler("xyz", [np.pi, 0.0, np.pi / 2]).as_matrix()
    wrist_to_tcp[:3, 3] = [0.0, 0.0, -0.186]
    wrist_to_graspgenx = np.eye(4)
    wrist_to_graspgenx[:3, :3] = Rotation.from_euler("x", np.pi).as_matrix()
    expected = np.linalg.inv(wrist_to_graspgenx) @ wrist_to_tcp
    assert np.allclose(np.asarray(config.grasp_frame_to_tcp), expected, atol=1e-9)
    # Approach +z, closing along x, the way GraspGenX wants them.
    tcp = np.asarray(config.grasp_frame_to_tcp)
    assert np.allclose(tcp[:3, 2], [0.0, 0.0, 1.0]), "grasp_frame's z is the approach"
    assert np.allclose(tcp[:3, 1], [1.0, 0.0, 0.0]), "grasp_frame's y is across the jaw"
    gripper = config.gripper
    jaw_open, pad_width, pad_length = gripper.extents_open
    assert 0.080 < jaw_open < 0.088, "88 mm of slider travel less the pads' overlap"
    assert gripper.extents_half_open[0] < jaw_open / 2 + 1e-9
    assert gripper.extents_half_open[1:] == (pad_width, pad_length), "prismatic: pads only slide"
    assert gripper.offset_open == gripper.offset_half_open
    assert gripper.fingertip_depth == OPENARM_V10_FINGERTIP_DEPTH_M
    far_face = gripper.offset_open[2] + pad_length / 2
    assert abs(far_face - gripper.fingertip_depth) < 1e-9, "the box ends at the fingertips"
    assert gripper.offset_open[2] < 0.186 < gripper.fingertip_depth, "the TCP sits in the pads"
    assert gripper.family == "parallel_2f"


def test_the_bench_stack_picks_with_graspgenx_and_can_fall_back_to_the_heuristic() -> None:
    """alfred-manipulation: GraspGenX answers propose_grasps; -heuristic is the same stack with
    the one-top-down-grasp generator and no second environment."""
    learned = _modules(alfred_manipulation)
    assert GraspGenXModule in learned and HeuristicGraspModule not in learned
    heuristic = _modules(alfred_manipulation_heuristic)
    assert HeuristicGraspModule in heuristic and GraspGenXModule not in heuristic
    assert learned - {GraspGenXModule} == heuristic - {HeuristicGraspModule}, "only the generator"
    (graspgenx,) = _atoms(alfred_manipulation, GraspGenXModule)
    assert graspgenx.kwargs["gripper"] == OPENARM_V10_SWEEP_VOLUME
    assert graspgenx.kwargs["grasp_frame_to_tcp"] == OPENARM_V10_GRASP_FRAME_TO_TCP


def test_graspgenx_has_time_to_start_beside_the_models() -> None:
    """The 30 s default covers `uv run` and the runtime's imports, which on the Orin run while ten
    workers start and OWLv2 and EdgeTAM load; the model itself loads later, in start()."""
    (graspgenx,) = _atoms(alfred_manipulation, GraspGenXModule)
    assert GraspGenXConfig.model_validate(graspgenx.kwargs).startup_timeout >= 60.0


@pytest.mark.parametrize("stack", [*_LAYERED_STACKS, alfred_navigation])
def test_no_stack_runs_a_rerun_bridge(stack: Blueprint) -> None:
    """The Desktop reads zenoh itself; a quarter of the Orin's RAM (the bridge's default) is what
    the models run in: that worker reached 6.4 GB and was OOM-killed, with the box thrashing."""
    assert not ({RerunBridgeModule, RerunWebSocketServer} & _modules(stack))


def test_the_base_task_commands_through_the_inverse_of_the_measured_plant() -> None:
    """The FlowBase yaws 2.85x and translates 0.77x what it is told; fed the plan's rates as
    they are, a turn ran away from the task's feedback and aborted. The gains are 1/K per
    axis from the artifact, and the physical limit times the gain stays under the firmware's
    cap so a fast diagonal is not clipped off its heading."""
    plan = _tasks(alfred_base)[BASE_TRAJECTORY_TASK_NAME]
    gains = plan.params["command_gains"]
    assert gains == pytest.approx((1.30, 1.30, 0.35), abs=0.05)
    assert gains == alfred_base_command_gains()
    artifact = json.loads(Path(ALFRED_FOLLOWER_ARTIFACT).read_text())
    for gain, axis in zip(gains, ("vx", "vy", "wz"), strict=True):
        assert gain == pytest.approx(1.0 / artifact["plant"][axis]["K"], rel=1e-6)
    assert plan.params["max_linear"] * max(gains[:2]) <= FLOWBASE_MAX_LINEAR_CMD_M_S + 1e-9
    assert plan.params["max_linear"] <= math.hypot(*ALFRED_PLANAR_BASE.velocity_limits[:2])
    assert plan.params["max_angular"] == ALFRED_PLANAR_BASE.velocity_limits[2]


def test_the_base_task_runs_uncompensated_until_alfred_has_a_plant_model(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    missing = tmp_path / "alfred_posedomain.json"
    monkeypatch.setattr(alfred_model, "ALFRED_FOLLOWER_ARTIFACT", str(missing))
    assert alfred_model.alfred_base_command_gains() == (1.0, 1.0, 1.0)
    plan = {task.name: task for task in alfred_coordinator_tasks(arms=False, follower=False)}[
        BASE_TRAJECTORY_TASK_NAME
    ]
    assert plan.params["command_gains"] == (1.0, 1.0, 1.0)
    assert plan.params["max_linear"] == pytest.approx(
        math.hypot(*ALFRED_PLANAR_BASE.velocity_limits[:2])
    )


# ---------------------------------------------------------------------- alfred-manipulation-sim

# What drives hardware in the bench stack, and what stands in for it in the sim.
_BENCH_HARDWARE: set[type] = {RealSenseCamera, AlfredHighLevel}
_SIM_HARDWARE: set[type] = {MujocoSimModule, SimFlowBase}


def _streams(blueprint: Blueprint, module: type) -> set[tuple[str, Any, type]]:
    """A module's ports as wired, every instance of it: (instance.port, the stream it is on,
    type)."""
    return {
        (
            f"{atom.name}.{stream.name}",
            blueprint.remapping_map.get((atom.name, stream.name), stream.name),
            stream.type,
        )
        for atom in _atoms(blueprint, module)
        for stream in atom.streams
    }


def test_the_sim_bench_stack_opens_no_hardware(monkeypatch: pytest.MonkeyPatch) -> None:
    """Mock arms whatever the robot's .env says, the lift's Nano emulated, the coordinator's mock
    base, a MuJoCo camera, and nothing that reaches the FlowBase's Pi."""
    monkeypatch.setenv(OPENARM_LEFT_CAN_ENV, "can2")
    monkeypatch.setenv(OPENARM_RIGHT_CAN_ENV, "can3")
    stack = autoconnect(*alfred_sim_modules(device="cpu"))
    modules = _modules(stack)
    assert not (_BENCH_HARDWARE & modules), "no D455, no Portal client"
    assert _SIM_HARDWARE <= modules
    (arms,) = _coordinator_kwargs(stack)["hardware"]
    assert arms.hardware_id == OPENARM_HARDWARE_ID
    assert arms.adapter_type == "mock_whole_body", "the CAN ports in the env are ignored"
    start = dict(zip(arms.joints, arms.adapter_kwargs["initial_positions"], strict=True))
    assert {name: start[name] for name in ALFRED_ARM_READY} == ALFRED_ARM_READY
    (pillar,) = _atoms(stack, PillarConnection)
    assert pillar.kwargs["device_path"] == PILLAR_SIM_DEVICE
    # The measured plant: the base task's command gains (its inverse) move it as planned.
    (base,) = _atoms(stack, SimFlowBase)
    plant = base.kwargs["plant_gains"]
    assert [k * g for k, g in zip(plant, alfred_base_command_gains(), strict=True)] == (
        pytest.approx([1.0, 1.0, 1.0])
    )


def test_the_sim_coordinator_runs_the_bench_stacks_claims() -> None:
    sim_tasks = _coordinator_kwargs(alfred_manipulation_sim)["tasks"]
    assert sim_tasks == _coordinator_kwargs(alfred_manipulation)["tasks"]
    assert {task.name for task in sim_tasks} >= set(ALFRED_GRIPPER_TASKS.values()), "Open, Close"


def test_the_sim_has_the_bench_stacks_streams_and_remappings_off_the_hardware() -> None:
    """Every module that drives no hardware is wired as on the robot; the stand-ins publish
    what the hardware's drivers do, under the same names."""
    bench = _modules(alfred_manipulation) - _BENCH_HARDWARE
    assert bench == _modules(alfred_manipulation_sim) - _SIM_HARDWARE
    for module in bench:
        assert _streams(alfred_manipulation_sim, module) == _streams(alfred_manipulation, module)
    camera = MujocoSimModule.name
    same = lambda stack, *apart: {  # noqa: E731
        key: value for key, value in stack.remapping_map.items() if key[0] not in apart
    }
    assert same(alfred_manipulation_sim, camera, SimFlowBase.name) == same(
        alfred_manipulation, AlfredHighLevel.name
    )
    published = {
        (stream, type_) for _, stream, type_ in _streams(alfred_manipulation, RealSenseCamera)
    }
    for stream in ("color_image", "depth_image", "pointcloud", "camera_info", "tf"):
        sim_streams = {
            (name, type_) for _, name, type_ in _streams(alfred_manipulation_sim, MujocoSimModule)
        }
        assert {(name, type_) for name, type_ in published if name == stream} <= sim_streams
    driver = {(name, type_) for _, name, type_ in _streams(alfred_manipulation, AlfredHighLevel)}
    reader = {(name, type_) for _, name, type_ in _streams(alfred_manipulation_sim, SimFlowBase)}
    assert {("wheel_odometry", Odometry), ("flowbase_cmd_vel", Twist)} <= driver & reader


def test_the_sim_camera_is_the_mast_d455() -> None:
    """The D455's names, frames, size and rates; mounted under UrdfManager's camera_front_link
    and carried along by the base's odometry (the pose WheelOdometryRelay publishes)."""
    (real,) = _atoms(alfred_manipulation, RealSenseCamera)
    (atom,) = _atoms(alfred_manipulation_sim, MujocoSimModule)
    config = MujocoSimModuleConfig.model_validate(atom.kwargs)
    assert real.kwargs["frame_id"] == f"{config.camera_name}_link" == config.base_frame_id
    assert (config.width, config.height, config.fps) == (D455_WIDTH, D455_HEIGHT, D455_FPS)
    assert (config.pointcloud_fps, config.pointcloud_decimation, config.depth_trunc_m) == (
        D455_POINTCLOUD_FPS,
        D455_POINTCLOUD_DECIMATION,
        DEPTH_TRUNC_M,
    )
    assert config.enable_color and config.enable_depth and config.enable_pointcloud
    assert config.headless, "no window"
    assert Path(str(config.address)).is_file()
    assert config.mocap_body == "base_link"
    remaps = alfred_manipulation_sim.remapping_map
    assert remaps[(MujocoSimModule.name, "mocap_pose")] == "flowbase_odom"


@pytest.mark.parametrize(
    ("device", "grasp", "detector", "segmenter"),
    [
        ("cuda", GraspGenXModule, "owlv2", "edgetam"),
        ("mps", HeuristicGraspModule, OWLV2_FLOAT32, "edgetam"),
        ("cpu", HeuristicGraspModule, OWLV2_FLOAT32, "none"),
    ],
)
def test_the_sim_runs_the_models_this_machine_can(
    monkeypatch: pytest.MonkeyPatch,
    device: PerceptionDevice,
    grasp: type,
    detector: str,
    segmenter: str,
) -> None:
    """GraspGenX only on CUDA, EdgeTAM on CUDA or MPS; localize and the pick pipeline everywhere,
    so a Mac or a laptop with no GPU still scans and picks."""
    monkeypatch.setattr(sim, "perception_device", lambda: device)
    stack = autoconnect(*alfred_sim_modules())
    modules = _modules(stack)
    assert {PerceptionLocalize, PickAndPlaceModule, grasp} <= modules
    assert len(modules & {GraspGenXModule, HeuristicGraspModule}) == 1, "one generator"
    (localize,) = _atoms(stack, PerceptionLocalize)
    assert localize.kwargs["detector"] == detector and localize.kwargs["segmenter"] == segmenter
    assert localize.kwargs["params"] == SIM_LOCALIZE_PARAMS
    (bench,) = _atoms(alfred_perception, PerceptionLocalize)
    machine = ("detector", "segmenter", "params")
    assert {k: v for k, v in localize.kwargs.items() if k not in machine} == {
        k: v for k, v in bench.kwargs.items() if k not in machine
    }, "only what renders and the machine need"


def test_renders_score_below_the_benchs_accept_score() -> None:
    """OWLv2 scored the rendered cucumber 0.16 and bottle 0.27 (false boxes 0.07 at most)."""
    assert 0.07 < SIM_LOCALIZE_PARAMS.accept_score < 0.16 < ALFRED_BENCH_PARAMS.accept_score
    assert SIM_LOCALIZE_PARAMS.candidate_floor <= SIM_LOCALIZE_PARAMS.accept_score


@pytest.mark.parametrize(
    ("cuda", "mps", "device"),
    [(True, True, "cuda"), (True, False, "cuda"), (False, True, "mps"), (False, False, "cpu")],
)
def test_the_device_probe_prefers_cuda_then_mps(
    monkeypatch: pytest.MonkeyPatch, cuda: bool, mps: bool, device: str
) -> None:
    monkeypatch.setattr(perception, "is_cuda_available", lambda: cuda)
    monkeypatch.setattr(perception, "is_mps_available", lambda: mps)
    assert perception_device() == device
