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

Unified 29-DOF policy: planner + encoder + decoder. All 27 GEAR locomotion
modes are reachable at runtime through the coordinator RPC surface:

    coordinator.task_invoke("sonic_wbc", "set_locomotion_mode",
                            {"mode": "HAPPY_DANCE_WALK"})

Usage:
    dimos --simulation mujoco run unitree-g1-sonic-wbc    # sim
    dimos run unitree-g1-sonic-wbc                        # real hardware

Real hardware note: SONIC uses armature-derived PD gains (SONIC_KP/KD),
NOT the GR00T gain table. Never run this blueprint while the C++
g1_deploy_onnx_ref binary owns rt/lowcmd.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, cast

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import g1_joints
from dimos.control.tasks.g1_sonic_wbc_task.sonic_pipeline import SONIC_KD, SONIC_KP
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.stream import Out
from dimos.core.transport import LCMTransport
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.unitree.g1.config import G1
from dimos.robot.unitree.g1.g1_rerun import (
    G1_RERUN_ROOT,
    g1_urdf_joint_state,
    g1_urdf_static_robot,
)
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

# SONIC model files ship in the LFS data archive (data/sonic: encoder,
# decoder, 774 MB planner, reference motion clips). LfsPath pulls lazily on
# first access; SONIC_MODEL_DIR / SONIC_PLANNER_PATH override for machines
# with a gear_sonic_deploy checkout.
_env_model_dir = os.environ.get("SONIC_MODEL_DIR")
_SONIC_RELEASE_DIR = Path(_env_model_dir) if _env_model_dir else LfsPath("sonic")
_env_planner = os.environ.get("SONIC_PLANNER_PATH")
_SONIC_PLANNER_PATH = Path(_env_planner) if _env_planner else LfsPath("sonic/planner_sonic.onnx")

_MJCF_PATH = LfsPath("mujoco_sim/g1_gear_wbc.xml")
_G1_NUM_MOTORS = len(g1_joints)
_cmd_vel_topic = "/cmd_vel" if global_config.simulation else "/g1/cmd_vel"

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

    _backend = G1WholeBodyConnection.blueprint(release_sport_mode=True)
    _adapter_type = "transport_lcm"
    _adapter_address = ""
    _tick_rate = 100.0
    _auto_arm = False
    _auto_dry_run = True
    _default_ramp_seconds = 10.0
    _decimation = 2  # 100 Hz tick / 2 = 50 Hz policy
    _n_workers = 10
    from dimos.hardware.sensors.lidar.pointlio.module import PointLio
    from dimos.mapping.ray_tracing.module import RayTracingVoxelMap

    _nav_stack = autoconnect(
        PointLio.blueprint(),
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


class _G1SonicCoordinator(ControlCoordinator):
    g1_joints: Out[JointState]


_coordinator = _G1SonicCoordinator.blueprint(
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
                "encoder_onnx": str(_SONIC_RELEASE_DIR / "model_encoder.onnx"),
                "decoder_onnx": str(_SONIC_RELEASE_DIR / "model_decoder.onnx"),
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

# Real hardware speaks LCM to G1WholeBodyConnection on fixed topics. In sim,
# leave transports to the runtime default (works under both lcm and zenoh);
# pinning LCMTransport here would silently break under DIMOS_TRANSPORT=zenoh.
if not global_config.simulation:
    _coordinator = _coordinator.transports(
        {
            ("joint_command", JointState): LCMTransport("/g1/joint_command", JointState),
            ("g1_joints", JointState): LCMTransport("/g1/joints", JointState),
            ("cmd_vel", Twist): LCMTransport(_cmd_vel_topic, Twist),
            ("motor_states", JointState): LCMTransport("/g1/motor_states", JointState),
            ("imu", Imu): LCMTransport("/g1/imu", Imu),
            ("motor_command", MotorCommandArray): LCMTransport(
                "/g1/motor_command", MotorCommandArray
            ),
        }
    )

_G1_JOINTS_ENTITY = "world/g1_joints"


def _g1_sonic_rerun_blueprint():
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(
            origin="world",
            name="G1 SONIC WBC",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(
                plane=rr.components.Plane3D.XY.with_distance(0.0),
            ),
        ),
        rrb.TimePanel(state="collapsed"),
    )


_rerun_config: dict[str, Any] = {
    "blueprint": _g1_sonic_rerun_blueprint,
    "visual_override": {
        "world/color_image": None,
        "world/camera_info": None,
        "world/depth_image": None,
        "world/depth_camera_info": None,
        _G1_JOINTS_ENTITY: g1_urdf_joint_state(root_path=G1_RERUN_ROOT),
    },
    "max_hz": {
        _G1_JOINTS_ENTITY: 25.0,
        "world/g1/imu": 10.0,
        "world/odometry": 15.0,
    },
    "static": {G1_RERUN_ROOT: g1_urdf_static_robot(root_path=G1_RERUN_ROOT)},
}

_remappings = [*_nav_remap, (_G1SonicCoordinator, "twist_command", "cmd_vel")]

unitree_g1_sonic_wbc = (
    autoconnect(
        _backend,
        _coordinator,
        _nav_stack,
        # rerun_config with callable factories does not survive the zenoh
        # deploy path (msgpack turns them into dicts); pass it only under LCM.
        vis_module(
            viewer_backend=global_config.viewer,
            rerun_config=None if global_config.transport == "zenoh" else _rerun_config,
        ),
    )
    .remappings(cast("Any", _remappings))
    .global_config(robot_model="unitree_g1", n_workers=_n_workers)
)
