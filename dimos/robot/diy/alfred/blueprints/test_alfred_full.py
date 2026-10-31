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

"""alfred-full composes navigation, the pillar and the arms on one coordinator, as commissioned."""

from __future__ import annotations

import json
import math
from pathlib import Path
import re
from typing import Any, cast
import xml.etree.ElementTree as ElementTree

import numpy as np
from numpy.typing import NDArray
import pytest
from scipy.spatial.transform import Rotation

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.control.components import HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.operator_handoff import OperatorHandoff
from dimos.control.tasks.operator_hold_task.operator_hold_task import OPERATOR_HOLD_TASK_NAME
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.coordination.blueprints import Blueprint, TransportSpec, autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport, ZenohTransport
from dimos.core.transport_factory import transport_topic
from dimos.hardware.drive_trains.flowbase.config import FLOWBASE_MAX_LINEAR_CMD_M_S
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXConfig, GraspGenXModule
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.planning.spec.validation import MAX_OCTREE_POINTS
from dimos.manipulation.planning.utils.roboplan_point_cloud_self_filter import (
    RoboPlanPointCloudSelfFilter,
)
from dimos.mapping.pointclouds.box_filter import (
    PointCloudBoxFilter,
    inside_crop,
    rasterize_voxels,
)
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.mapping.relocalization.lidar.module import LocalMapRelocalization
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.navigation.embodiment.alfred import ALFRED
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.navigation.local_planner.native import LocalPlannerNative
from dimos.perception.detection.module2D import Detection2DModule
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.perception.localize.module import PerceptionLocalize
from dimos.robot.diy.alfred import alfred_model
from dimos.robot.diy.alfred.alfred_model import (
    ALFRED_ARM_READY,
    ALFRED_FOLLOWER_ARTIFACT,
    ALFRED_FOOTPRINT_RADIUS_M,
    ALFRED_HEIGHT_M,
    ALFRED_LIFT_LOWER_M,
    ALFRED_LIFT_UPPER_M,
    ALFRED_MOVING_LINKS,
    ALFRED_PLANAR_BASE,
    ALFRED_V1_MODEL,
    alfred_arm_joints,
    alfred_base_command_gains,
    alfred_follower_artifact,
    alfred_model_config,
    alfred_planar_joint_names,
    alfred_rerun_urdf,
)
from dimos.robot.diy.alfred.blueprints import (
    alfred_full_perception as perception,
    alfred_manipulation_sim as sim,
)
from dimos.robot.diy.alfred.blueprints._nav import WALL_CLEARANCE_M
from dimos.robot.diy.alfred.blueprints.alfred_full import (
    ALFRED_BASE_HARDWARE_ID,
    ALFRED_GRIPPER_TASKS,
    BASE_FRAME,
    BASE_TRAJECTORY_TASK_NAME,
    BASE_VELOCITY_TASK_NAME,
    MANIP_VOXEL_M,
    NAV_FOLLOWER_TASK_NAME,
    ODOM_FRAME,
    OPENARM_LEFT_CAN_ENV,
    OPENARM_RIGHT_CAN_ENV,
    AlfredNavCoordinator,
    AlfredOperatorCoordinator,
    _openarm_hardware_from_env,
    alfred_coordinator_tasks,
    alfred_full,
)
from dimos.robot.diy.alfred.blueprints.alfred_full_perception import (
    D455_FPS,
    D455_HEIGHT,
    D455_POINTCLOUD_DECIMATION,
    D455_POINTCLOUD_FPS,
    D455_WIDTH,
    DEPTH_TRUNC_M,
    MANIP_MAP_CROP_MAX,
    MANIP_MAP_CROP_MIN,
    MANIP_MAP_SELF_FILTER,
    MANIP_VOXEL_MAP,
    MOUNT_TF_TOLERANCE_S,
    OPENARM_REACH_M,
    OPENARM_WRIST_TO_TIP_M,
    SCENE_DETECTOR_CONFIDENCE,
    PerceptionDevice,
    alfred_full_perception,
    alfred_perception_modules,
    perception_device,
)
from dimos.robot.diy.alfred.blueprints.alfred_manipulation import (
    alfred_manipulation,
    alfred_manipulation_heuristic,
    alfred_operator_modules,
)
from dimos.robot.diy.alfred.blueprints.alfred_manipulation_sim import (
    SIM_SCENE_DETECTOR_CONFIDENCE,
    alfred_manipulation_sim,
    alfred_sim_modules,
)
from dimos.robot.diy.alfred.blueprints.alfred_navigation import alfred_navigation
from dimos.robot.diy.alfred.blueprints.alfred_sim import alfred_sim
from dimos.robot.diy.alfred.coordinator_base_odometry import CoordinatorBaseOdometry
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel
from dimos.robot.diy.alfred.gripper import (
    OPENARM_V10_FINGERTIP_DEPTH_M,
    OPENARM_V10_GRASP_FRAME_TO_TCP,
    OPENARM_V10_SWEEP_VOLUME,
)
from dimos.robot.diy.alfred.pillar_connection import (
    PILLAR_HARDWARE_ID,
    PILLAR_LIFT_JOINT,
    PILLAR_SIM_DEVICE,
    PillarConnection,
)
from dimos.robot.diy.alfred.wheel_odometry_relay import WheelOdometryRelay
from dimos.robot.manipulators.openarm.config import OPENARM_HARDWARE_ID
from dimos.robot.urdf_manager import UrdfManager
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule, MujocoSimModuleConfig
from dimos.utils.gpu_utils import is_cuda_available
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer


def _atoms(blueprint: Blueprint, module: type) -> list[Any]:
    return [
        atom
        for atom in blueprint.blueprints
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
        for atom in blueprint.blueprints
        for port in atom.streams
        if port.direction != "in" and _wired(blueprint, atom.name, port.name) == stream
    }


def _self_filters(blueprint: Blueprint) -> tuple[Any, Any]:
    """The camera cloud's self filter, and the planner map's (the carve)."""
    filters = {atom.name: atom for atom in _atoms(blueprint, RoboPlanPointCloudSelfFilter)}
    carve = filters.pop(MANIP_MAP_SELF_FILTER)
    (camera,) = filters.values()
    return camera, carve


def _channel(blueprint: Blueprint, key: tuple[str, type]) -> tuple[type, str]:
    """The backend class and topic a stream is pinned to."""
    spec = blueprint.transport_map[key]
    assert isinstance(spec, TransportSpec)
    return spec.cls, cast("str", spec.args[0])


# The urdf is LFS data, so these run where it is pulled (like test_alfred.py).
needs_lfs = pytest.mark.self_hosted


def test_alfred_full_composes_nav_pillar_arms_and_the_viewer() -> None:
    modules = {atom.module for atom in alfred_full.blueprints}
    assert {
        AlfredHighLevel,
        PointLio,
        RayTracingVoxelMap,
        MLSPlannerNative,
        LocalPlannerNative,
        LocalMapRelocalization,
        PillarConnection,
        ManipulationModule,
        AlfredNavCoordinator,
        RerunWebSocketServer,
    } <= modules


def test_the_flowbase_is_arbitrated_on_the_coordinator_but_written_by_alfred_high_level() -> None:
    """The coordinator plans for the base but must never be a second Portal writer.

    Its base is a transport adapter: the arbitrated twist goes out on /flowbase/cmd_vel,
    AlfredHighLevel's cmd_vel is pinned to that channel, and the adapter reads the base
    pose back off /flowbase/odom, which StartRelay fills from tf.
    """
    hardware = {hw.hardware_id: hw for hw in _coordinator_kwargs(alfred_full)["hardware"]}
    assert set(hardware) == {PILLAR_HARDWARE_ID, OPENARM_HARDWARE_ID, ALFRED_BASE_HARDWARE_ID}
    base = hardware[ALFRED_BASE_HARDWARE_ID]
    assert base.hardware_type is HardwareType.BASE
    assert base.adapter_type == "transport_lcm", "a direct FlowBase adapter would double-write"

    backend = ZenohTransport if global_config.transport == "zenoh" else LCMTransport
    assert _channel(alfred_full, ("cmd_vel", Twist)) == (
        backend,
        transport_topic(f"/{ALFRED_BASE_HARDWARE_ID}/cmd_vel"),
    )
    assert _channel(alfred_full, ("start_pose", PoseStamped)) == (
        backend,
        transport_topic(f"/{ALFRED_BASE_HARDWARE_ID}/odom"),
    )


def test_the_pillar_meets_its_adapter_on_the_active_backend() -> None:
    backend = ZenohTransport if global_config.transport == "zenoh" else LCMTransport
    for stream, msg_type in (("motor_command", MotorCommandArray), ("motor_states", JointState)):
        assert _channel(alfred_full, (stream, msg_type)) == (
            backend,
            transport_topic(f"/{PILLAR_HARDWARE_ID}/{stream}"),
        )


def test_teleop_enters_the_coordinator_not_the_base() -> None:
    """Whatever publishes tele_cmd_vel (the Controller, the web viewer) is one base claim."""
    assert alfred_full.remapping_map[("ControlCoordinator", "twist_command")] == "tele_cmd_vel"


def test_base_claims_rank_teleop_over_plan_over_navigation() -> None:
    tasks = _tasks(alfred_full)
    teleop = tasks[BASE_VELOCITY_TASK_NAME]
    plan = tasks[BASE_TRAJECTORY_TASK_NAME]
    nav = tasks[NAV_FOLLOWER_TASK_NAME]
    base_joints = [f"{ALFRED_BASE_HARDWARE_ID}/{axis}" for axis in ("vx", "vy", "wz")]
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
    nav = _tasks(alfred_full)[NAV_FOLLOWER_TASK_NAME]
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
    tasks = _tasks(alfred_full)
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
    (planner,) = _atoms(alfred_full, ManipulationModule)
    assert planner.kwargs["gripper_tasks"] == ALFRED_GRIPPER_TASKS, "one hand per arm"


def test_the_whole_body_is_planned_in_the_robots_own_frame() -> None:
    """Targets are poses on the robot (base_link), so they ride with it and never depend on
    where Point-LIO woke up or whether its odometry jumps; "world" is an identity edge under
    base_link for the Controller and the stock tooling. The planar base joints are the base's
    motion from here: held at zero, never fed from odometry, executed as a relative trajectory."""
    (atom,) = _atoms(alfred_full, ManipulationModule)
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
    (planner,) = _atoms(alfred_full, MLSPlannerNative)
    assert planner.kwargs["robot_height"] == ALFRED.height == ALFRED_HEIGHT_M
    assert planner.kwargs["wall_clearance_m"] == WALL_CLEARANCE_M
    assert WALL_CLEARANCE_M < ALFRED_FOOTPRINT_RADIUS_M


def test_alfred_full_perception_feeds_the_planner_a_filtered_d455_map() -> None:
    """A second, short-range voxel map under its own name, so the nav map's streams stay put."""
    modules = {atom.module for atom in alfred_full_perception.blueprints}
    assert {RealSenseCamera, RoboPlanPointCloudSelfFilter, PointCloudBoxFilter} <= modules
    assert Detection2DModule not in modules, "the perception stack's localize replaced YOLO"
    assert (PerceptionLocalize in modules) == is_cuda_available(), "localize needs the GPU"
    assert len(_atoms(alfred_full_perception, RayTracingVoxelMap)) == 2
    remaps = alfred_full_perception.remapping_map
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
    manip_map = next(
        atom
        for atom in _atoms(alfred_full_perception, RayTracingVoxelMap)
        if atom.kwargs.get("instance_name") == MANIP_VOXEL_MAP
    )
    assert manip_map.kwargs["world_frame"] == ODOM_FRAME, "a drive must not move the bench"
    assert manip_map.kwargs["tf_match_tolerance_s"] >= MOUNT_TF_TOLERANCE_S, "a 5 Hz mount edge"
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
    (box_filter,) = _atoms(alfred_full_perception, PointCloudBoxFilter)
    assert box_filter.kwargs["target_frame"] == BASE_FRAME, "the planner drops any other frame"
    assert box_filter.kwargs["tf_tolerance_s"] >= MOUNT_TF_TOLERANCE_S, (
        "odom -> base_link lands at 47 Hz; the map's stamp is a 5 Hz camera's"
    )
    (planner,) = _atoms(alfred_full_perception, ManipulationModule)
    assert planner.kwargs["world_frame"] == box_filter.kwargs["target_frame"]
    assert planner.kwargs["voxel_map_resolution"] == manip_map.kwargs["voxel_size"], (
        "the octree's cells are the map's voxels"
    )
    assert box_filter.kwargs["target_voxel_m"] == planner.kwargs["voxel_map_resolution"], (
        "the map is re-rasterized onto the octree's own cells, or a turned base leaves it holes"
    )
    camera_filter, _ = _self_filters(alfred_full_perception)
    assert camera_filter.kwargs["padding_m"] >= 0.044, "an open finger is 4.4 cm outside the model"
    (camera,) = _atoms(alfred_full_perception, RealSenseCamera)
    assert camera.kwargs["enable_pointcloud"] and camera.kwargs["enable_color"]
    localize = next(
        atom
        for fragment in alfred_perception_modules(models=True)
        for atom in fragment.blueprints
        if atom.module is PerceptionLocalize
    )
    assert localize.kwargs["world_frame"] == BASE_FRAME, "a found object is an arm target"
    assert localize.kwargs["geometry"] == "depth" and localize.kwargs["mobile"] is False
    assert localize.kwargs["params"].min_views == 1, "one fixed camera: one viewpoint"
    assert "float16" not in localize.kwargs["detector"], "the registry's fp16 build is for a GPU"
    assert localize.kwargs["segmenter"] == "none", "EdgeTAM needs a GPU"
    with_gpu = {
        atom.module for f in alfred_perception_modules(models=True) for atom in f.blueprints
    }
    assert {ObjectSceneRegistrationModule, PickAndPlaceModule, GraspGenXModule} <= with_gpu
    without = {
        atom.module for f in alfred_perception_modules(models=False) for atom in f.blueprints
    }
    assert not ({ObjectSceneRegistrationModule, PickAndPlaceModule} & without)
    scene = next(
        atom
        for f in alfred_perception_modules(models=True)
        for atom in f.blueprints
        if atom.module is ObjectSceneRegistrationModule
    )
    # The scene is kept in odom like the map, so a drive leaves the objects where they are;
    # PickAndPlaceModule re-expresses an object's cloud in its planning frame through tf.
    assert scene.kwargs["target_frame"] == ODOM_FRAME, "objects stay put when the base drives"
    assert scene.kwargs["tf_tolerance_s"] >= MOUNT_TF_TOLERANCE_S, "a 5 Hz mount edge"
    assert pick_planning_frame(alfred_perception_modules(models=True)) == BASE_FRAME, (
        "grasps where the arm plans"
    )
    assert scene.kwargs["detector_confidence"] < 0.6, "OWLv2 scores 0.3-0.6 on this bench"
    assert scene.kwargs["detect_on_request"] is True, "a scan, not a stream"
    pick = next(
        atom
        for f in alfred_perception_modules(models=True)
        for atom in f.blueprints
        if atom.module is PickAndPlaceModule
    )
    for side in ("left", "right"):
        reach = pick.kwargs["reach"][f"{side}_manipulator"]
        assert reach.shoulder_frame == f"openarm_{side}_link1", "a frame the planner puts on tf"
        assert 0.40 <= reach.radius <= 0.45 and abs(reach.wrist_to_tip - 0.186) < 1e-9
    assert not alfred_perception_modules(models=False) or all(
        atom.module is not PerceptionLocalize
        for fragment in alfred_perception_modules(models=False)
        for atom in fragment.blueprints
    ), "on a CPU-only torch localize would peg the box"


def pick_planning_frame(fragments: tuple[Any, ...]) -> str:
    (pick,) = [
        atom for f in fragments for atom in f.blueprints if atom.module is PickAndPlaceModule
    ]
    return cast("str", pick.kwargs["planning_frame"])


@pytest.mark.parametrize(
    "stack",
    [
        pytest.param(alfred_full_perception, id="alfred-full-perception"),
        pytest.param(alfred_manipulation, id="alfred-manipulation"),
        pytest.param(alfred_manipulation_heuristic, id="alfred-manipulation-heuristic"),
        pytest.param(alfred_manipulation_sim, id="alfred-manipulation-sim"),
    ],
)
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
    for tolerance in ("state_tolerance_s", "tf_tolerance_s", "tf_forward_tolerance_s"):
        assert carve.kwargs[tolerance] == camera_filter.kwargs[tolerance], tolerance
    assert box_filter.kwargs["target_frame"] == BASE_FRAME, "the model's root: no tf to wait on"


@pytest.mark.parametrize(
    "stack",
    [
        pytest.param(alfred_full_perception, id="alfred-full-perception"),
        pytest.param(alfred_manipulation, id="alfred-manipulation"),
        pytest.param(alfred_manipulation_heuristic, id="alfred-manipulation-heuristic"),
        pytest.param(alfred_manipulation_sim, id="alfred-manipulation-sim"),
    ],
)
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
    modules = {atom.module for atom in alfred_manipulation.blueprints}
    assert {
        AlfredHighLevel,
        WheelOdometryRelay,
        PillarConnection,
        ManipulationModule,
        AlfredOperatorCoordinator,
        RealSenseCamera,
        RoboPlanPointCloudSelfFilter,
    } <= modules
    assert not {PointLio, MLSPlannerNative, LocalPlannerNative, LocalMapRelocalization} & modules
    assert PerceptionLocalize not in modules, "the bench picks through the scene registration"
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


@pytest.mark.parametrize(
    "blueprint",
    [
        pytest.param(alfred_manipulation, id="alfred-manipulation"),
        pytest.param(alfred_manipulation_sim, id="alfred-manipulation-sim"),
    ],
)
def test_an_operator_can_hold_the_bench_stacks(blueprint: Blueprint) -> None:
    """The operator hold outranks every claim, so it freezes arms, grippers, lift and base."""
    tasks = _tasks(blueprint)
    hold = tasks[OPERATOR_HOLD_TASK_NAME]
    assert hold.type == "operator_hold" and not hold.joint_names, "it claims every joint"
    assert hold.params["hold_through_estop"], "teleop's velocity task ignores the e-stop"
    assert all(task.priority < hold.priority for task in tasks.values() if task is not hold)
    modules = {atom.module for atom in blueprint.blueprints}
    assert {AlfredOperatorCoordinator, OperatorHandoff, McpServer} <= modules
    assert AlfredNavCoordinator in AlfredOperatorCoordinator.__mro__, "the follower's ports stay"


def test_the_agent_joins_the_bench_stacks_only_with_a_key(monkeypatch: pytest.MonkeyPatch) -> None:
    def modules() -> set[Any]:
        return {atom.module for part in alfred_operator_modules() for atom in part.blueprints}

    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    assert McpClient not in modules()
    monkeypatch.setenv("OPENAI_API_KEY", "sk-test")
    assert McpClient in modules()


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
    hardware_ids = {hw.hardware_id for hw in _coordinator_kwargs(alfred_navigation)["hardware"]}
    assert ALFRED_BASE_HARDWARE_ID in hardware_ids
    assert not any("openarm" in hw_id or "arm" in hw_id for hw_id in hardware_ids), "no arms' CAN"
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


def test_the_controller_gets_the_urdf_without_a_second_tf_tree() -> None:
    """UrdfManager serves the XML and meshes; Point-LIO and the mount tree keep tf, and the
    planner puts the moving links on it."""
    (urdf,) = _atoms(alfred_full, UrdfManager)
    assert urdf.kwargs["publish_joint_tf"] is False
    (planner,) = _atoms(alfred_full, ManipulationModule)
    assert planner.kwargs["model"].tf_extra_links == ALFRED_MOVING_LINKS


@needs_lfs
def test_the_moving_links_are_every_link_under_a_movable_joint() -> None:
    root = ElementTree.fromstring(ALFRED_V1_MODEL.load().xml)
    under_movable = [
        joint.find("child").attrib["link"]  # type: ignore[union-attr]
        for joint in root.findall("joint")
        if joint.attrib["type"] != "fixed"
    ]
    assert ALFRED_MOVING_LINKS == under_movable


def test_a_click_is_the_goal() -> None:
    """The Controller's "Go here" and the rerun viewer publish clicked_point; with no
    MovementManager to relay it, the planner listens there itself."""
    assert alfred_full.remapping_map[(MLSPlannerNative.name, "goal")] == "clicked_point"
    assert (RerunWebSocketServer.name, "clicked_point") not in alfred_full.remapping_map


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


def test_the_agent_drives_the_same_stack_the_controller_does() -> None:
    from dimos.agents.mcp.mcp_client import McpClient
    from dimos.agents.mcp.mcp_server import McpServer
    from dimos.agents.skills.point_nav import PointNavSkillContainer
    from dimos.manipulation.manipulation_skills import ManipulationSkills
    from dimos.robot.diy.alfred.blueprints.alfred_full_agentic import alfred_full_agentic
    from dimos.robot.diy.alfred.system_prompt import ALFRED_SYSTEM_PROMPT

    modules = {atom.module for atom in alfred_full_agentic.blueprints}
    assert {McpServer, McpClient, PointNavSkillContainer, ManipulationSkills} <= modules
    (client,) = _atoms(alfred_full_agentic, McpClient)
    assert client.kwargs["system_prompt"] is ALFRED_SYSTEM_PROMPT
    # go_to publishes where "Go here" does, so one planner input serves both.
    assert (
        alfred_full_agentic.remapping_map[(PointNavSkillContainer.name, "goal")] == "clicked_point"
    )
    (nav,) = _atoms(alfred_full_agentic, PointNavSkillContainer)
    assert nav.kwargs["world_frame"] == "odom"


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
    learned = {atom.module for atom in alfred_manipulation.blueprints}
    assert GraspGenXModule in learned and HeuristicGraspModule not in learned
    heuristic = {atom.module for atom in alfred_manipulation_heuristic.blueprints}
    assert HeuristicGraspModule in heuristic and GraspGenXModule not in heuristic
    assert learned - {GraspGenXModule} == heuristic - {HeuristicGraspModule}, "only the generator"
    (graspgenx,) = _atoms(alfred_manipulation, GraspGenXModule)
    assert graspgenx.kwargs["gripper"] == OPENARM_V10_SWEEP_VOLUME
    assert graspgenx.kwargs["grasp_frame_to_tcp"] == OPENARM_V10_GRASP_FRAME_TO_TCP
    assert GraspGenXModule not in {
        atom.module for f in alfred_perception_modules(models=False) for atom in f.blueprints
    }, "GraspGenX is a GPU model"


def test_graspgenx_has_time_to_start_beside_the_models() -> None:
    """The 30 s default covers `uv run` and the runtime's imports, which on the Orin run while ten
    workers start and OWLv2 and EdgeTAM load; the model itself loads later, in start()."""
    (graspgenx,) = [
        atom
        for fragment in alfred_perception_modules(models=True)
        for atom in fragment.blueprints
        if atom.module is GraspGenXModule
    ]
    assert GraspGenXConfig.model_validate(graspgenx.kwargs).startup_timeout >= 60.0


def test_the_rerun_bridge_keeps_little_for_late_viewers() -> None:
    """A quarter of the Orin's RAM (the bridge's default) is what the models run in: that
    worker reached 6.4 GB and was OOM-killed, with the box thrashing first."""
    (bridge,) = _atoms(alfred_full, RerunBridgeModule)
    assert bridge.kwargs["memory_limit"] == "256MB"


@pytest.mark.parametrize("stack", [alfred_manipulation, alfred_navigation], ids=["bench", "nav"])
def test_the_split_stacks_run_no_rerun_bridge(stack: Blueprint) -> None:
    """The Desktop reads zenoh itself; the bridge only cost the Orin memory (the OOM above)."""
    modules = {atom.module for atom in stack.blueprints}
    assert not ({RerunBridgeModule, RerunWebSocketServer} & modules)


def test_the_base_task_commands_through_the_inverse_of_the_measured_plant() -> None:
    """The FlowBase yaws 2.85x and translates 0.77x what it is told; fed the plan's rates as
    they are, a turn ran away from the task's feedback and aborted. The gains are 1/K per
    axis from the artifact, and the physical limit times the gain stays under the firmware's
    cap so a fast diagonal is not clipped off its heading."""
    plan = _tasks(alfred_full)[BASE_TRAJECTORY_TASK_NAME]
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
_SIM_HARDWARE: set[type] = {MujocoSimModule, CoordinatorBaseOdometry}


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
    modules = {atom.module for atom in stack.blueprints}
    assert not (_BENCH_HARDWARE & modules), "no D455, no Portal client"
    assert _SIM_HARDWARE <= modules
    hardware = {hw.hardware_id: hw for hw in _coordinator_kwargs(stack)["hardware"]}
    assert set(hardware) == {PILLAR_HARDWARE_ID, OPENARM_HARDWARE_ID, ALFRED_BASE_HARDWARE_ID}
    arms = hardware[OPENARM_HARDWARE_ID]
    assert arms.adapter_type == "mock_whole_body", "the CAN ports in the env are ignored"
    start = dict(zip(arms.joints, arms.adapter_kwargs["initial_positions"], strict=True))
    assert {name: start[name] for name in ALFRED_ARM_READY} == ALFRED_ARM_READY
    assert hardware[PILLAR_HARDWARE_ID].adapter_type == "transport_lcm", "as on the robot"
    (pillar,) = _atoms(stack, PillarConnection)
    assert pillar.kwargs["device_path"] == PILLAR_SIM_DEVICE
    base = hardware[ALFRED_BASE_HARDWARE_ID]
    assert base.adapter_type == "mock_twist_base"
    assert base.adapter_kwargs["integrate_odometry"] is True
    assert base.adapter_kwargs["step_dt"] == pytest.approx(0.01), "one coordinator tick"
    # The measured plant: the base task's command gains (its inverse) move it as planned.
    plant = base.adapter_kwargs["plant_gains"]
    assert [k * g for k, g in zip(plant, alfred_base_command_gains(), strict=True)] == (
        pytest.approx([1.0, 1.0, 1.0])
    )
    (odometry,) = _atoms(stack, CoordinatorBaseOdometry)
    assert odometry.kwargs["base_joints"] == base.joints


def test_the_sim_coordinator_runs_the_bench_stacks_claims() -> None:
    sim_tasks = _coordinator_kwargs(alfred_manipulation_sim)["tasks"]
    assert sim_tasks == _coordinator_kwargs(alfred_manipulation)["tasks"]
    assert {task.name for task in sim_tasks} >= set(ALFRED_GRIPPER_TASKS.values()), "Open, Close"


def test_the_sim_has_the_bench_stacks_streams_and_remappings_off_the_hardware() -> None:
    """Every module that drives no hardware is wired as on the robot; the stand-ins publish
    what the hardware's drivers do, under the same names."""
    bench = {atom.module for atom in alfred_manipulation.blueprints} - _BENCH_HARDWARE
    assert bench == {atom.module for atom in alfred_manipulation_sim.blueprints} - _SIM_HARDWARE
    for module in bench:
        assert _streams(alfred_manipulation_sim, module) == _streams(alfred_manipulation, module)
    camera = MujocoSimModule.name
    assert {
        key: value
        for key, value in alfred_manipulation_sim.remapping_map.items()
        if key[0] != camera
    } == dict(alfred_manipulation.remapping_map)
    assert dict(alfred_manipulation_sim.transport_map) == dict(alfred_manipulation.transport_map)
    published = {
        (stream, type_) for _, stream, type_ in _streams(alfred_manipulation, RealSenseCamera)
    }
    for stream in ("color_image", "depth_image", "pointcloud", "camera_info", "tf"):
        sim_streams = {
            (name, type_) for _, name, type_ in _streams(alfred_manipulation_sim, MujocoSimModule)
        }
        assert {(name, type_) for name, type_ in published if name == stream} <= sim_streams
    driver = {(name, type_) for _, name, type_ in _streams(alfred_manipulation, AlfredHighLevel)}
    reader = {
        (name, type_)
        for _, name, type_ in _streams(alfred_manipulation_sim, CoordinatorBaseOdometry)
    }
    assert ("wheel_odometry", Odometry) in driver & reader


def test_the_sim_camera_is_the_mast_d455() -> None:
    """The D455's names, frames, size and rates; mounted under AlfredMountTf's camera_front_link
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
    assert remaps[(MujocoSimModule.name, "mocap_pose")] == "start_pose"
    assert _channel(alfred_manipulation_sim, ("start_pose", PoseStamped))[1] == transport_topic(
        f"/{ALFRED_BASE_HARDWARE_ID}/odom"
    )


@pytest.mark.parametrize(
    ("device", "grasp", "segmenter"),
    [
        ("cuda", GraspGenXModule, "edgetam"),
        ("mps", HeuristicGraspModule, "edgetam"),
        ("cpu", HeuristicGraspModule, "none"),
    ],
)
def test_the_sim_runs_the_models_this_machine_can(
    monkeypatch: pytest.MonkeyPatch, device: PerceptionDevice, grasp: type, segmenter: str
) -> None:
    """GraspGenX only on CUDA, EdgeTAM on CUDA or MPS; OWLv2 and the pick pipeline everywhere,
    so a Mac or a laptop with no GPU still scans and picks."""
    monkeypatch.setattr(sim, "perception_device", lambda: device)
    stack = autoconnect(*alfred_sim_modules())
    modules = {atom.module for atom in stack.blueprints}
    assert {ObjectSceneRegistrationModule, PickAndPlaceModule, grasp} <= modules
    assert len(modules & {GraspGenXModule, HeuristicGraspModule}) == 1, "one generator"
    assert PerceptionLocalize not in modules, "as on the bench"
    (scene,) = _atoms(stack, ObjectSceneRegistrationModule)
    assert scene.kwargs["detector_backend"] == "owlv2"
    assert scene.kwargs["segmentation_backend"] == segmenter
    assert scene.kwargs["detector_confidence"] == SIM_SCENE_DETECTOR_CONFIDENCE
    (bench_scene,) = _atoms(alfred_manipulation, ObjectSceneRegistrationModule)
    unchanged = ("detector_backend", "detector_confidence", "segmentation_backend")
    assert {k: v for k, v in scene.kwargs.items() if k not in unchanged} == {
        k: v for k, v in bench_scene.kwargs.items() if k not in unchanged
    }, "only what renders and the machine need"


def test_renders_score_below_the_benchs_detector_threshold() -> None:
    """OWLv2 scored the rendered cucumber 0.16 and bottle 0.27 (false boxes 0.07 at most)."""
    assert 0.07 < SIM_SCENE_DETECTOR_CONFIDENCE < 0.16 < SCENE_DETECTOR_CONFIDENCE


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
