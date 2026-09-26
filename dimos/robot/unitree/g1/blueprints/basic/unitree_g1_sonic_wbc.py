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

"""Unitree G1 SONIC (GEAR-SONIC) whole-body-control blueprint.

Unified 29-DOF policy: planner + encoder + decoder. Selectable GEAR locomotion
modes are reachable through the coordinator RPC surface:

    coordinator.task_invoke("sonic_wbc", "set_locomotion_mode",
                            {"mode": "HAPPY_DANCE_WALK"})

Usage:
    dimos --transport zenoh --simulation mujoco run unitree-g1-sonic-wbc
    dimos --transport zenoh run unitree-g1-sonic-wbc

Real hardware note: SONIC uses armature-derived PD gains (SONIC_KP/KD),
NOT the GR00T gain table. Never run this blueprint while the C++
g1_deploy_onnx_ref binary owns rt/lowcmd.
"""

from __future__ import annotations

from functools import cache
import os
from pathlib import Path
from typing import Any, cast

from yourdfpy import URDF

from dimos.control.components import HardwareComponent, HardwareType, make_humanoid_joints
from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.g1_sonic_wbc_task.coordinator import SonicCoordinator
from dimos.control.tasks.g1_sonic_wbc_task.models import sonic_model_directory
from dimos.control.tasks.g1_sonic_wbc_task.sonic_pipeline import (
    DEFAULT_ANGLES_DDS,
    SONIC_KD,
    SONIC_KP,
)
from dimos.control.tasks.g1_sonic_wbc_task.sonic_safety import COMMAND_TIMEOUT_SECONDS
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.hardware.whole_body.transport.adapter import zenoh_latest_transport
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.unitree.g1.config import G1
from dimos.robot.unitree.g1.g1_rerun import g1_costmap
from dimos.utils.data import LfsPath
from dimos.visualization.vis_module import vis_module

_G1_NAV_VOXEL_RESOLUTION = 0.05
_G1_REAL_NAV_VOXEL_RESOLUTION = 0.08
_G1_NAV_OVERHEAD_SAFETY_MARGIN = 0.2
_G1_NAV_MAX_STEP_HEIGHT = 0.10
_G1_NAV_ROTATION_DIAMETER = 0.8
_G1_NAV_SAFE_RADIUS_MARGIN = 0.6
assert G1.height_clearance is not None and G1.width_clearance is not None
_MUJOCO_LIDAR_CAMERAS = (
    "lidar_front_camera",
    "lidar_left_camera",
    "lidar_right_camera",
)
_MUJOCO_LIDAR_KWARGS: dict[str, Any] = {
    "camera_name": _MUJOCO_LIDAR_CAMERAS[0],
    "mujoco_lidar_camera_names": list(_MUJOCO_LIDAR_CAMERAS),
    "width": 320,
    "height": 240,
    "fps": 2,
    "enable_color": False,
    "enable_depth": False,
    "enable_pointcloud": True,
    "pointcloud_fps": 1.0,
    "enable_mujoco_lidar": True,
    "mujoco_lidar_geom_groups": [2, 3],
    "mujoco_lidar_raycast_width": 64,
    "mujoco_lidar_raycast_height": 32,
    "mujoco_lidar_robot_exclusion_radius": G1.width_clearance,
}

# Download models explicitly with dimos-sonic-models; blueprint discovery
# must not contact model hosts or materialize the old LFS archive.
_SONIC_RELEASE_DIR = sonic_model_directory()
_env_planner = os.environ.get("SONIC_PLANNER_PATH")
_SONIC_PLANNER_PATH = (
    Path(_env_planner) if _env_planner else _SONIC_RELEASE_DIR / "planner_sonic.onnx"
)

_MJCF_PATH = LfsPath("mujoco_sim/g1_gear_wbc.xml")
g1_joints = make_humanoid_joints("g1")
_G1_NUM_MOTORS = len(g1_joints)

_adapter_address: str | Path

if global_config.simulation and global_config.simulation != "mujoco":
    raise ValueError("unitree-g1-sonic-wbc only supports --simulation mujoco")

if global_config.simulation == "mujoco":
    from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
    from dimos.simulation.engines.robot_sim_binding import (
        RobotSimSpec,
        mjcf_joint_names_from_hardware,
    )

    _g1_sim_joints = tuple(g1_joints)
    _g1_sim_spec = RobotSimSpec(
        robot_id="g1",
        hardware_joints=_g1_sim_joints,
        root_body_names=("pelvis",),
        root_joint_names=("floating_base_joint",),
        require_floating_base=True,
        model_joint_names=mjcf_joint_names_from_hardware(_g1_sim_joints),
        imu_gyro_names=(
            "imu-pelvis-angular-velocity",
            "imu-torso-angular-velocity",
            "imu-angular-velocity",
            "gyro_pelvis",
            "imu_gyro",
        ),
        imu_accel_names=(
            "imu-pelvis-linear-acceleration",
            "imu-torso-linear-acceleration",
            "imu-linear-acceleration",
            "accelerometer_pelvis",
            "imu_accel",
        ),
        require_imu=True,
    )

    from dimos.mapping.voxels.module import VoxelGridMapper

    _backend = MujocoSimModule.blueprint(
        address=_MJCF_PATH,
        headless=True,
        dof=_G1_NUM_MOTORS,
        inject_legacy_assets=True,
        robot_sim_spec=_g1_sim_spec,
        reset_joint_positions=DEFAULT_ANGLES_DDS.tolist(),
        wait_for_control_command=True,
        **_MUJOCO_LIDAR_KWARGS,
    )
    _adapter_type = "sim_mujoco_g1"
    _adapter_address = _MJCF_PATH
    _tick_rate = 50.0
    _auto_arm = True
    _auto_dry_run = False
    _default_ramp_seconds = 0.0
    _decimation = 1
    _n_workers = 2
    _nav_stack = autoconnect(
        VoxelGridMapper.blueprint(emit_every=1),
        CostMapper.blueprint(
            config=HeightCostConfig(
                resolution=_G1_NAV_VOXEL_RESOLUTION,
                can_pass_under=G1.height_clearance + _G1_NAV_OVERHEAD_SAFETY_MARGIN,
                can_climb=_G1_NAV_MAX_STEP_HEIGHT,
            ),
            initial_safe_radius_meters=G1.width_clearance + _G1_NAV_SAFE_RADIUS_MARGIN,
        ),
        ReplanningAStarPlanner.blueprint(
            robot_width=G1.width_clearance,
            robot_rotation_diameter=_G1_NAV_ROTATION_DIAMETER,
        ),
        MovementManager.blueprint(),
    )
    _nav_remap = [(VoxelGridMapper, "lidar", "pointcloud")]
else:
    from dimos.robot.unitree.g1.wholebody_connection import G1WholeBodyConnection

    _backend = G1WholeBodyConnection.blueprint(command_timeout_seconds=COMMAND_TIMEOUT_SECONDS)
    _adapter_type = "transport_zenoh"
    _adapter_address = ""
    _tick_rate = 50.0
    _auto_arm = False
    _auto_dry_run = True
    _default_ramp_seconds = 3.0
    _decimation = 1
    _n_workers = 10
    from dimos.hardware.sensors.lidar.pointlio.module import PointLio
    from dimos.mapping.ray_tracing.module import RayTracingVoxelMap

    _nav_stack = autoconnect(
        PointLio.blueprint(
            lidar_ip=os.environ.get("DIMOS_POINTLIO_LIDAR_IP", "192.168.123.120"),
        ),
        RayTracingVoxelMap.blueprint(
            voxel_size=_G1_REAL_NAV_VOXEL_RESOLUTION,
            emit_every=0,
            global_emit_every=4,
            max_health=10,
            graze_cos=0.85,
        ),
        CostMapper.blueprint(
            config=HeightCostConfig(
                resolution=_G1_REAL_NAV_VOXEL_RESOLUTION,
                can_pass_under=G1.height_clearance + _G1_NAV_OVERHEAD_SAFETY_MARGIN,
                can_climb=_G1_NAV_MAX_STEP_HEIGHT,
            ),
            initial_safe_radius_meters=G1.width_clearance + _G1_NAV_SAFE_RADIUS_MARGIN,
        ),
        ReplanningAStarPlanner.blueprint(
            robot_width=G1.width_clearance,
            robot_rotation_diameter=_G1_NAV_ROTATION_DIAMETER,
        ),
        MovementManager.blueprint(),
    )
    _nav_remap = []


_coordinator = SonicCoordinator.blueprint(
    instance_name="ControlCoordinator",
    publish_robot_joint_states=True,
    tick_rate=_tick_rate,
    hardware=[
        HardwareComponent(
            hardware_id="g1",
            hardware_type=HardwareType.WHOLE_BODY,
            joints=g1_joints,
            adapter_type=_adapter_type,
            address=_adapter_address,
            wb_config=WholeBodyConfig(kp=tuple(SONIC_KP), kd=tuple(SONIC_KD)),
        ),
    ],
    tasks=[
        TaskConfig(
            name="sonic_wbc",
            type="g1_sonic_wbc",
            joint_names=g1_joints,
            priority=50,
            auto_start=True,
            params={
                "encoder_onnx": str(_SONIC_RELEASE_DIR / "sonic_v1_1" / "model_encoder.onnx"),
                "decoder_onnx": str(_SONIC_RELEASE_DIR / "sonic_v1_1" / "model_decoder.onnx"),
                "planner_onnx": str(_SONIC_PLANNER_PATH),
                "hardware_id": "g1",
                "auto_arm": _auto_arm,
                "auto_dry_run": _auto_dry_run,
                "default_ramp_seconds": _default_ramp_seconds,
                "decimation": _decimation,
            },
        ),
    ],
)

# A backlog of motor targets is not useful to a 50 Hz controller.
_coordinator = _coordinator.transports(
    {
        ("joint_command", JointState): zenoh_latest_transport("/g1/joint_command", JointState),
        ("g1_joints", JointState): zenoh_latest_transport("/g1/joints", JointState),
        ("motor_states", JointState): zenoh_latest_transport("/g1/motor_states", JointState),
        ("imu", Imu): zenoh_latest_transport("/g1/imu", Imu),
        ("motor_command", MotorCommandArray): zenoh_latest_transport(
            "/g1/motor_command", MotorCommandArray
        ),
    }
)


def _require_zenoh() -> str | None:
    if global_config.transport == "zenoh":
        return None
    return "G1 SONIC requires --transport zenoh"


@cache
def _g1_real_ground_z() -> float:
    """Use GR00T's rest-pose lidar offset and 0.74 m standing pelvis height."""
    urdf = URDF.load(str(G1.model_path), load_meshes=False)
    urdf.update_cfg([0.0] * len(urdf.actuated_joint_names))
    mount_z = float(urdf.get_transform("mid360_link", "pelvis")[2, 3])
    return -(mount_z + 0.74)


def _render_real_costmap(grid: Any) -> Any:
    return g1_costmap(grid, z_offset=_g1_real_ground_z() + 0.02)


_costmap_renderer = g1_costmap if global_config.simulation == "mujoco" else _render_real_costmap
_remappings = [*_nav_remap, (SonicCoordinator, "twist_command", "cmd_vel")]

unitree_g1_sonic_wbc = (
    autoconnect(
        _backend,
        _coordinator,
        _nav_stack,
        vis_module(
            viewer_backend=global_config.viewer,
            rerun_config={
                "tf_axes": 0.5,
                "visual_override": {
                    "world/global_costmap": _costmap_renderer,
                    "world/navigation_costmap": _costmap_renderer,
                    "world/lidar": None,
                    "world/pointcloud": None,
                    "world/local_map": None,
                    "world/local_map_fine": None,
                },
            },
        ),
    )
    .remappings(cast("Any", _remappings))
    .requirements(_require_zenoh)
    .global_config(robot_model="unitree_g1", n_workers=_n_workers)
)
