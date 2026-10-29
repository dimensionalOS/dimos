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

"""Unitree G1 GR00T whole-body control with Mid360 + Point-LIO navigation.

One blueprint for hardware and simulation. ``--simulation mujoco`` replaces the
G1 connection and the Mid360 driver with their simulated twins (see
``docs/usage/sim2.md``); the coordinator and its transport adapter, Point-LIO,
mapping, planning and the viewer are the same modules on the same streams in
both modes. What differs is the operating profile: tick rate, arming and the
worker count.

Hardware: 100 Hz tick, policy decimation 2. Comes up unarmed and dry-run; the
operator activates through the ControlCoordinator RPC, and the policy ramps
from the current pose to its bent-knee default over 10 s before taking torque
control. Simulation: 50 Hz tick, auto arm, no ramp. The 14 arm joints accept
bounded position commands through the lower-priority trajectory task in both.

Usage:
    dimos run unitree-g1-groot-wbc                                   # hardware
    dimos --simulation mujoco run unitree-g1-groot-wbc               # logistics scene
    dimos --simulation mujoco --scene-package workbench run unitree-g1-groot-wbc
    dimos run unitree-g1-groot-wbc --network-interface=enp2s0
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import (
    g1_arms,
    g1_joints,
    g1_legs_waist,
)
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.stream import Out
from dimos.core.transport import LCMTransport
from dimos.hardware.sensors.lidar.pointlio.module import PointLio
from dimos.hardware.sensors.lidar.pointlio.pointlio_blueprints import mid360_for_pointlio
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.navigation.go2.replanning_a_star.module import ReplanningAStarPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.g1.config import G1
from dimos.robot.unitree.g1.g1_rerun import g1_costmap, g1_urdf_joint_state, g1_urdf_static_robot
from dimos.robot.unitree.g1.groot_config import G1_GROOT_KD, G1_GROOT_KP
from dimos.robot.unitree.g1.manip_config import G1_TELEOP_ARM_MODEL
from dimos.robot.unitree.g1.teleop_ik import G1PinkPoseTargetSolver
from dimos.robot.unitree.g1.wholebody_connection import G1WholeBodyConnection
from dimos.utils.data import LfsPath
from dimos.visualization.vis_module import vis_module

if global_config.simulation and global_config.simulation != "mujoco":
    raise ValueError("unitree-g1-groot-wbc only supports --simulation mujoco")

# The operating profile is the only thing the simulation flag changes here.
_SIM = global_config.simulation == "mujoco"

# Lazy data handle: LfsPath pulls on first use, not at import.
_GROOT_MODEL_DIR = LfsPath("groot")
_G1_URDF_PATH = Path(__file__).resolve().parents[2] / "g1.urdf"
assert G1.height_clearance is not None and G1.width_clearance is not None
# go2 nav_3d resolution; 0.05 saturates the raytracer on the Orin.
_G1_NAV_VOXEL_RESOLUTION = 0.08
_G1_NAV_OVERHEAD_SAFETY_MARGIN = 0.2
_G1_NAV_MAX_STEP_HEIGHT = 0.10
_G1_NAV_ROTATION_DIAMETER = 0.8
_G1_NAV_SAFE_RADIUS_MARGIN = 0.6
# Nominal standing pelvis height; matches G1GrootWBCTask's height_cmd.
_G1_NOMINAL_PELVIS_Z = 0.74


class _G1GrootCoordinator(TeleopControlCoordinator):
    g1_joints: Out[JointState]


# Per-robot joint stream. Namespaced like the rest of the g1 wire topics, which
# also fixes its Rerun entity path (`world/` + topic).
_G1_JOINTS_TOPIC = "/g1/joints"
_G1_JOINTS_ENTITY = f"world{_G1_JOINTS_TOPIC}"

# The robot's wire. The coordinator's transport adapter subscribes
# /g1/motor_states and /g1/imu and publishes /g1/motor_command for hardware id
# "g1"; naming the connection's streams the same way puts both ends on the
# default transport. The retained connection uses these in both modes.
_robot = G1WholeBodyConnection.blueprint(release_sport_mode=True).remappings(
    [
        (G1WholeBodyConnection, "motor_command", "g1/motor_command"),
        (G1WholeBodyConnection, "motor_states", "g1/motor_states"),
        (G1WholeBodyConnection, "imu", "g1/imu"),
    ]
)

_G1_TELEOP_MODEL = RobotModelConfig(
    model=G1_TELEOP_ARM_MODEL,
    joint_names=list(g1_arms),
    base_link="pelvis",
)
_G1_TELEOP_PINK = PinkKinematicsConfig(
    dt=0.01,
    position_cost=8.0,
    orientation_cost=2.0,
    posture_cost=0.01,
    joint_limit_posture_margin=0.3,
    lm_damping=0.01,
    gain=0.25,
)

_coordinator = _G1GrootCoordinator.blueprint(
    instance_name="ControlCoordinator",
    publish_robot_joint_states=True,
    # The onboard Jetson cannot sustain a 500 Hz tick; the policy was trained at 50 Hz.
    tick_rate=50.0 if _SIM else 100.0,
    hardware=[
        HardwareComponent(
            hardware_id="g1",
            hardware_type=HardwareType.WHOLE_BODY,
            joints=g1_joints,
            adapter_type="transport_lcm",
            address="",
            wb_config=WholeBodyConfig(kp=tuple(G1_GROOT_KP), kd=tuple(G1_GROOT_KD)),
        ),
    ],
    tasks=[
        TaskConfig(
            name="groot_wbc",
            type="g1_groot_wbc",
            joint_names=g1_legs_waist,
            priority=50,
            auto_start=True,
            params={
                "model_path": _GROOT_MODEL_DIR,
                "hardware_id": "g1",
                # Hardware comes up unarmed and dry-run; the operator activates it.
                "auto_arm": _SIM,
                "auto_dry_run": not _SIM,
                "default_ramp_seconds": 0.0 if _SIM else 10.0,
                "decimation": 1 if _SIM else 2,
            },
        ),
        joint_trajectory_task(
            g1_arms,
            priority=10,
            velocity_limits={name: 1.0 for name in g1_arms},
        ),
        # Shared bimanual WebXR task with G1-only model and objective tuning.
        TaskConfig(
            name="teleop_g1",
            type="teleop_ik",
            joint_names=g1_arms,
            priority=20,
            params={
                "robot_model": _G1_TELEOP_MODEL,
                "bindings": [
                    {"hand": "left", "target_frame": "left_rubber_hand"},
                    {"hand": "right", "target_frame": "right_rubber_hand"},
                ],
                "solver_type": G1PinkPoseTargetSolver,
                "pink": _G1_TELEOP_PINK,
                "timeout": 0.5,
                "max_command_tracking_error_deg": 10.0,
                "max_joint_velocity_rad_s": math.radians(120.0),
                "joint_command_filter_cutoff_hz": 5.0,
            },
        ),
    ],
).transports(
    {
        # LCM topics other tools listen on: teleop, dimos topic, the Rerun bridge.
        ("joint_command", JointState): LCMTransport("/g1/joint_command", JointState),
        ("g1_joints", JointState): LCMTransport(_G1_JOINTS_TOPIC, JointState),
        ("cmd_vel", Twist): LCMTransport("/g1/cmd_vel", Twist),
    }
)

# Navigation as on the real robot: Mid360 scans and IMU into Point-LIO, a
# raytracing voxel map on its odometry, costmap, planner and movement manager.
_navigation = autoconnect(
    mid360_for_pointlio(),
    PointLio.blueprint(),
    RayTracingVoxelMap.blueprint(
        voxel_size=_G1_NAV_VOXEL_RESOLUTION,
        emit_every=0,  # no local_map consumer here
        global_emit_every=4,  # ~1 Hz global map; also paces the costmap
        # Clearing matched to go2 nav_3d.
        max_health=10,
        graze_cos=0.85,
    ),
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


def _g1_groot_rerun_blueprint() -> Any:
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(
            origin="world",
            name="G1 GR00T WBC",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(
                plane=rr.components.Plane3D.XY.with_distance(0.0),
            ),
        ),
        rrb.TimePanel(state="collapsed"),
    )


def _g1_nav_path(path: NavPath) -> Any:
    return path.to_rerun(z_offset=0.3)


# The robot mesh roots under the LIO's odometry, whose world frame is the lidar
# boot pose (ground about 1.2 m below 0), in both modes.
_G1_ROOT = "world/odometry/g1"
_g1_pelvis_mid360_cache: list[Any] = []


def _g1_pelvis_to_mid360() -> Any:
    """Rest-pose pelvis->mid360_link transform from the G1 URDF (cached)."""
    if not _g1_pelvis_mid360_cache:
        from importlib import import_module

        import numpy as np

        urdf = import_module("yourdfpy").URDF.load(str(_G1_URDF_PATH), load_meshes=False)
        urdf.update_cfg(np.zeros(len(urdf.actuated_joint_names)))
        _g1_pelvis_mid360_cache.append(urdf.get_transform("mid360_link", "pelvis"))
    return _g1_pelvis_mid360_cache[0]


def _g1_odometry_root(odom: Any) -> Any:
    """Robot-mesh root: pelvis pose from the LIO's mid360 odometry (rest offset)."""
    import numpy as np
    import rerun as rr

    from dimos.msgs.geometry_msgs.Quaternion import Quaternion

    t_world_mid360 = np.eye(4)
    # The MID-360 is mounted upside down (the URDF doesn't carry the flip):
    # un-roll by Rx(pi) == diag(1, -1, -1).
    t_world_mid360[:3, :3] = odom.orientation.to_rotation_matrix() @ np.diag([1.0, -1.0, -1.0])
    t_world_mid360[:3, 3] = (odom.x, odom.y, odom.z)
    t_world_pelvis = t_world_mid360 @ np.linalg.inv(_g1_pelvis_to_mid360())
    q = Quaternion.from_rotation_matrix(t_world_pelvis[:3, :3])
    return rr.Transform3D(
        translation=t_world_pelvis[:3, 3].tolist(),
        rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
    )


def _g1_ground_z() -> float:
    """Ground height in the LIO boot frame: -(mount z + nominal pelvis z)."""
    return -(float(_g1_pelvis_to_mid360()[2, 3]) + _G1_NOMINAL_PELVIS_Z)


def _g1_costmap_on_ground(grid: Any) -> Any:
    """Costmap rendered on the actual ground plane of the boot frame."""
    return g1_costmap(grid, z_offset=_g1_ground_z() + 0.02)


_rerun_config: dict[str, Any] = {
    "blueprint": _g1_groot_rerun_blueprint,
    "visual_override": {
        _G1_JOINTS_ENTITY: g1_urdf_joint_state(root_path=_G1_ROOT),
        "world/odometry": _g1_odometry_root,
        "world/global_costmap": _g1_costmap_on_ground,
        "world/navigation_costmap": _g1_costmap_on_ground,
        "world/path": _g1_nav_path,
        # Raw scans are sensor-frame (LIO contract); the voxel map is the live view.
        "world/lidar": None,
        "world/lidar_raw": None,
    },
    "max_hz": {
        _G1_JOINTS_ENTITY: 20.0,
        # Raw state streams arrive at hundreds of Hz; useful only as debug plots.
        "world/g1/imu": 10.0,
        "world/g1/motor_states": 10.0,
        "world/g1/motor_command": 10.0,
        "world/odometry": 15.0,
        "world/global_map": 1.0,
        "world/global_costmap": 2.0,
        "world/navigation_costmap": 2.0,
        # The planner publishes an empty Path() immediately before the new
        # planned path. Throttling this entity drops the real path.
        "world/path": 0,
    },
    "static": {_G1_ROOT: g1_urdf_static_robot(root_path=_G1_ROOT)},
}


def _viewer() -> Any:
    return vis_module(viewer_backend=global_config.viewer, rerun_config=_rerun_config)


# Robot plus coordinator, shared with the teleop and collection blueprints.
_unitree_g1_groot_wbc_core = (
    autoconnect(_robot, _coordinator)
    .remappings([(_G1GrootCoordinator, "twist_command", "cmd_vel")])
    # One process per heavy module on hardware; fewer workers starve the Rerun bridge.
    .global_config(robot_model="unitree_g1", n_workers=2 if _SIM else 11)
)

unitree_g1_groot_wbc = autoconnect(_unitree_g1_groot_wbc_core, _navigation, _viewer())
