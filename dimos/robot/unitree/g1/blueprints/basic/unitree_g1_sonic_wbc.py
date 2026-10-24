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
modes are reachable directly on the policy module:

    app.G1SonicPolicyModule.set_locomotion_mode("HAPPY_DANCE_WALK")

Usage:
    dimos --transport zenoh --simulation mujoco run unitree-g1-sonic-wbc
    dimos --transport zenoh run unitree-g1-sonic-wbc

Use DimOS Desktop for visualization and operator controls.

Real hardware note: SONIC uses armature-derived PD gains (SONIC_KP/KD),
NOT the GR00T gain table. Never run this blueprint while the C++
g1_deploy_onnx_ref binary owns rt/lowcmd.
"""

from __future__ import annotations

import os
from pathlib import Path
import platform
from typing import Any, cast

from dimos.control.components import make_humanoid_joints
from dimos.control.sonic.models import sonic_model_directory
from dimos.control.sonic.sonic_pipeline import DEFAULT_ANGLES_DDS
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.module import Module
from dimos.core.transport import ZenohTransport
from dimos.mapping.costmapper import CostMapper
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.navigation.go2.replanning_a_star.module import ReplanningAStarPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.protocol.pubsub.impl.zenohpubsub import QOS_LATEST_WINS, Topic as ZenohTopic
from dimos.robot.unitree.g1.config import G1
from dimos.robot.unitree.g1.control_safety import COMMAND_TIMEOUT_SECONDS
from dimos.robot.unitree.g1.sonic_policy import G1SonicPolicyModule
from dimos.simulation.scenes.catalog import resolve_scene_package
from dimos.utils.data import LfsPath

_G1_REAL_NAV_VOXEL_RESOLUTION = 0.08
_G1_NAV_ROTATION_DIAMETER = 0.8
_G1_NAV_SAFE_RADIUS_MARGIN = 0.6
assert G1.width_clearance is not None
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

_simulation_address: Path | None
_imu_remap: tuple[type[Module], str, str]

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

    _scene_package = resolve_scene_package(global_config.scene_package)
    _scene_kwargs: dict[str, Any] = {}
    if _scene_package is not None:
        if _scene_package.mujoco_scene_path is None:
            raise ValueError(f"Scene package has no MuJoCo scene: {_scene_package.metadata_path}")
        _scene_kwargs = {
            "scene_xml": _scene_package.mujoco_scene_path,
            "robot_mjcf": _MJCF_PATH,
            "robot_root_body": "pelvis",
            "scene_entities": _scene_package.entities,
        }

    _backend = MujocoSimModule.blueprint(
        address=_MJCF_PATH,
        headless=True,
        dof=_G1_NUM_MOTORS,
        inject_legacy_assets=True,
        robot_sim_spec=_g1_sim_spec,
        reset_joint_positions=DEFAULT_ANGLES_DDS.tolist(),
        wait_for_control_command=True,
        **_MUJOCO_LIDAR_KWARGS,
        **_scene_kwargs,
    )
    _simulation_address = _MJCF_PATH
    _imu_remap = (MujocoSimModule, "imu", "low_level_imu")
    _tick_rate = 50.0
    _auto_arm = True
    _auto_dry_run = False
    _default_ramp_seconds = 0.0
    _decimation = 1
    _n_workers = 2
    _nav_stack = autoconnect(
        VoxelGridMapper.blueprint(emit_every=1),
        CostMapper.blueprint(
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
    _simulation_address = None
    _imu_remap = (G1WholeBodyConnection, "imu", "low_level_imu")
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
            initial_safe_radius_meters=G1.width_clearance + _G1_NAV_SAFE_RADIUS_MARGIN,
        ),
        ReplanningAStarPlanner.blueprint(
            robot_width=G1.width_clearance,
            robot_rotation_diameter=_G1_NAV_ROTATION_DIAMETER,
        ),
        MovementManager.blueprint(),
    )
    _nav_remap = []


_policy = G1SonicPolicyModule.blueprint(
    encoder_onnx=_SONIC_RELEASE_DIR / "sonic_v1_1" / "model_encoder.onnx",
    decoder_onnx=_SONIC_RELEASE_DIR / "sonic_v1_1" / "model_decoder.onnx",
    planner_onnx=_SONIC_PLANNER_PATH,
    simulation_address=_simulation_address,
    cpu_simulation=_simulation_address is not None and platform.system() == "Darwin",
    joint_names=g1_joints,
    tick_rate=_tick_rate,
    auto_arm=_auto_arm,
    auto_dry_run=_auto_dry_run,
    default_ramp_seconds=_default_ramp_seconds,
    decimation=_decimation,
)


def _require_zenoh() -> str | None:
    if global_config.transport == "zenoh":
        return None
    return "G1 SONIC requires --transport zenoh"


def _zenoh_latest_transport(topic: str, msg_type: type) -> ZenohTransport[Any]:
    """Keep only the newest command or feedback frame on each G1 topic."""
    return ZenohTransport(
        ZenohTopic(
            f"dimos/{topic.lstrip('/')}",
            msg_type,
            queue_capacity=1,
            qos=QOS_LATEST_WINS,
        )
    )


_remappings = [
    *_nav_remap,
    _imu_remap,
    (G1SonicPolicyModule, "base_command", "cmd_vel"),
    (G1SonicPolicyModule, "joint_state", "g1_joints"),
]

unitree_g1_sonic_wbc = (
    autoconnect(
        _backend,
        _policy,
        _nav_stack,
    )
    .remappings(cast("Any", _remappings))
    .transports(
        {
            ("cmd_vel", Twist): _zenoh_latest_transport("cmd_vel", Twist),
            ("position_command", JointState): _zenoh_latest_transport(
                "/g1/position_command", JointState
            ),
            ("g1_joints", JointState): _zenoh_latest_transport("/g1/joints", JointState),
            ("motor_states", JointState): _zenoh_latest_transport("/g1/motor_states", JointState),
            ("low_level_imu", Imu): _zenoh_latest_transport("/g1/imu", Imu),
            ("motor_command", MotorCommandArray): _zenoh_latest_transport(
                "/g1/motor_command", MotorCommandArray
            ),
        }
    )
    .requirements(_require_zenoh)
    .global_config(robot_model="unitree_g1", n_workers=_n_workers, viewer="none")
)
