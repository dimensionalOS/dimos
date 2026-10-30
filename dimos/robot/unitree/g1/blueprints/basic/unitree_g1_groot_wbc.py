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

One blueprint for hardware and simulation. ``--simulation mujoco`` retains the
G1 connection, supplies its robosuite motor device and simulates the Mid360 (see
``docs/usage/sim2.md``); the coordinator and its transport adapter, Point-LIO,
mapping and planning are the same modules on the same streams in both modes.
DimOS Desktop consumes those streams directly. What differs is the operating
profile: tick rate, arming and the worker count.

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

from dimos.control.components import HardwareComponent, HardwareType
from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import (
    g1_arms,
    g1_joints,
    g1_legs_waist,
)
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import Blueprint, autoconnect
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
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.navigation.go2.replanning_a_star.module import ReplanningAStarPlanner
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.g1.config import G1
from dimos.robot.unitree.g1.connection import G1Connection
from dimos.robot.unitree.g1.groot_config import G1_GROOT_HOME, G1_GROOT_KD, G1_GROOT_KP
from dimos.robot.unitree.g1.manip_config import G1_TELEOP_ARM_MODEL
from dimos.robot.unitree.g1.teleop_ik import G1PinkPoseTargetSolver
from dimos.utils.data import LfsPath

if global_config.simulation and global_config.simulation != "mujoco":
    raise ValueError("unitree-g1-groot-wbc only supports --simulation mujoco")

# The operating profile is the only thing the simulation flag changes here.
_SIM = global_config.simulation == "mujoco"

# Lazy data handle: LfsPath pulls on first use, not at import.
_GROOT_MODEL_DIR = LfsPath("groot")
assert G1.height_clearance is not None and G1.width_clearance is not None
# go2 nav_3d resolution; 0.05 saturates the raytracer on the Orin.
_G1_NAV_VOXEL_RESOLUTION = 0.08
_G1_NAV_OVERHEAD_SAFETY_MARGIN = 0.2
_G1_NAV_MAX_STEP_HEIGHT = 0.10
_G1_NAV_ROTATION_DIAMETER = 0.8
_G1_NAV_SAFE_RADIUS_MARGIN = 0.6


class _G1GrootCoordinator(TeleopControlCoordinator):
    g1_joints: Out[JointState]


# Per-robot joint stream, namespaced like the other g1 topics.
_G1_JOINTS_TOPIC = "/g1/joints"

# The robot's wire. The coordinator's transport adapter subscribes
# /g1/motor_states and /g1/imu and publishes /g1/motor_command for hardware id
# "g1"; naming the connection's streams the same way puts both ends on the
# default transport. The retained connection uses these in both modes.
# This is the sole G1 connection declaration. With --simulation mujoco,
# ModuleCoordinator.build calls attach_simulation, which resolves this class's
# describe_simulated_robot hook and injects its device after CLI settings are known.
_robot = G1Connection.blueprint(
    control_mode="motors",
    initial_qpos=tuple(G1_GROOT_HOME),
    startup_kp=tuple(G1_GROOT_KP),
    startup_kd=tuple(G1_GROOT_KD),
    wait_for_first_command=True,
).remappings(
    [
        (G1Connection, "motor_command", "g1/motor_command"),
        (G1Connection, "motor_states", "g1/motor_states"),
        (G1Connection, "imu", "g1/imu"),
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
        # Existing LCM topics used by teleop and command tools.
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


# Robot plus coordinator, shared with the teleop and collection blueprints.
_unitree_g1_groot_wbc_core = (
    autoconnect(_robot, _coordinator)
    .remappings([(_G1GrootCoordinator, "twist_command", "cmd_vel")])
    # Keep the control and perception workers separate on hardware.
    .global_config(robot_model="unitree_g1", n_workers=2 if _SIM else 11, viewer="none")
)

unitree_g1_groot_wbc = autoconnect(_unitree_g1_groot_wbc_core, _navigation)


def _require_simulation_profile() -> str | None:
    if global_config.simulation != "mujoco" or not _SIM:
        return "This G1 simulation requires --simulation mujoco before loading the blueprint."
    return None


def g1_groot_wbc_simulation_profile() -> Blueprint:
    """Return control and navigation, requiring the simulation profile.

    Set ``--simulation mujoco`` before importing this module. The requirement
    rejects a missing or subsequently changed mode before deploying modules.
    This helper does not attach physics: ModuleCoordinator.build resolves the
    connection's describe_simulated_robot binding after CLI overrides. Cameras are
    composed separately by the caller; Desktop consumes the published streams.
    """
    return autoconnect(_unitree_g1_groot_wbc_core, _navigation).requirements(
        _require_simulation_profile
    )
