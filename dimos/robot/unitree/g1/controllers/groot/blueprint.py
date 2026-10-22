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

"""G1 connection and GR00T/arm tasks, without navigation or visualization."""

import math

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import joint_trajectory_task
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.robot.unitree.g1.connection import G1Connection
from dimos.robot.unitree.g1.controllers.groot.config import (
    G1_GROOT_KD,
    G1_GROOT_KP,
)
from dimos.robot.unitree.g1.controllers.groot.coordinator import G1GrootCoordinator
from dimos.robot.unitree.g1.devices.motors import G1MotorConfig
from dimos.robot.unitree.g1.manipulation.config import G1_TELEOP_ARM_MODEL
from dimos.robot.unitree.g1.manipulation.teleop_ik import G1PinkPoseTargetSolver
from dimos.robot.unitree.g1.model.joints import (
    g1_arms,
    g1_joints,
    g1_legs_waist,
)
from dimos.utils.data import LfsPath

G1_TELEOP_MODEL = RobotModelConfig(
    model=G1_TELEOP_ARM_MODEL,
    joint_names=list(g1_arms),
    base_link="pelvis",
)


def groot_control(*, simulated: bool) -> Blueprint:
    """Keep the existing GR00T walking, arm trajectory and WebXR task settings."""
    coordinator = G1GrootCoordinator.blueprint(
        instance_name="ControlCoordinator",
        publish_robot_joint_states=True,
        # Hardware runs at 100 Hz with policy decimation 2; simulation at 50 Hz.
        tick_rate=50.0 if simulated else 100.0,
        motor_gains={"g1": WholeBodyConfig(kp=tuple(G1_GROOT_KP), kd=tuple(G1_GROOT_KD))},
        tasks=[
            TaskConfig(
                name="groot_wbc",
                type="g1_groot_wbc",
                joint_names=g1_legs_waist,
                priority=50,
                auto_start=True,
                params={
                    "model_path": LfsPath("groot"),
                    "hardware_id": "g1",
                    "all_joint_names": g1_joints,
                    "auto_arm": simulated,
                    "auto_dry_run": not simulated,
                    "default_ramp_seconds": 0.0 if simulated else 10.0,
                    "decimation": 1 if simulated else 2,
                },
            ),
            joint_trajectory_task(
                g1_arms,
                priority=10,
                velocity_limits={name: 1.0 for name in g1_arms},
            ),
            TaskConfig(
                name="teleop_g1",
                type="teleop_ik",
                joint_names=g1_arms,
                priority=20,
                params={
                    "robot_model": G1_TELEOP_MODEL,
                    "bindings": [
                        {"hand": "left", "target_frame": "left_rubber_hand"},
                        {"hand": "right", "target_frame": "right_rubber_hand"},
                    ],
                    "solver_type": G1PinkPoseTargetSolver,
                    "pink": PinkKinematicsConfig(
                        dt=0.01,
                        position_cost=8.0,
                        orientation_cost=2.0,
                        posture_cost=0.01,
                        joint_limit_posture_margin=0.3,
                        lm_damping=0.01,
                        gain=0.25,
                    ),
                    "timeout": 0.5,
                    "max_command_tracking_error_deg": 10.0,
                    "max_joint_velocity_rad_s": math.radians(120.0),
                    "joint_command_filter_cutoff_hz": 5.0,
                },
            ),
        ],
    )
    return (
        autoconnect(
            G1Connection.blueprint(
                controller="groot",
                motors=G1MotorConfig(release_sport_mode=True, command_timeout_seconds=0.25),
            ),
            coordinator,
        )
        .remappings(
            [
                (G1Connection, "motor_command", "g1_motor_command"),
                (G1Connection, "motor_states", "g1_motor_states"),
                (G1Connection, "imu", "g1_imu"),
                ("g1connection_lidar", "pointcloud", "pointcloud"),
                ("g1connection_camera", "color_image", "color_image"),
                ("g1connection_camera", "camera_info", "camera_info"),
                ("g1connection_camera", "depth_image", "depth_image"),
                ("g1connection_camera", "depth_camera_info", "depth_camera_info"),
            ]
        )
        .global_config(robot_model="unitree_g1", n_workers=2 if simulated else 11)
    )
