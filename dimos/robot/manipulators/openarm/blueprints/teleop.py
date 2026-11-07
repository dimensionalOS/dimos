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

"""OpenArm WebXR teleop blueprint."""

from __future__ import annotations

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.robot.manipulators.damiao.damiao_blueprint import (
    DamiaoTeleopCoordinator,
    arm_connection,
    arm_remaps,
)
from dimos.robot.manipulators.openarm.config import openarm_bimanual_model_config
from dimos.robot.manipulators.openarm.connection import OpenArmConnection
from dimos.robot.manipulators.openarm.teleop_ik import OpenArmPinkPoseTargetSolver
from dimos.teleop.webxr.extensions import ArmTeleopModule

OPENARM_WEBXR_TASK_NAME = "teleop_openarm"
_MODEL = openarm_bimanual_model_config(robot="arm")

_OPENARM_ARM_VELOCITY_PROFILE_RAD_S = (1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0)
_OPENARM_JOINT_VELOCITY_LIMITS_RAD_S = {
    joint_name: velocity_limit
    for joint_name, velocity_limit in zip(
        _MODEL.joint_names,
        _OPENARM_ARM_VELOCITY_PROFILE_RAD_S * 2,
        strict=True,
    )
}


def _trajectory_task(*, priority: int = 10) -> TaskConfig:
    return TaskConfig(
        name=JOINT_TRAJECTORY_TASK_NAME,
        type="trajectory",
        joint_names=[
            *_MODEL.joint_names,
            *[f"arm/{name}" for name in OpenArmConnection.robot.gripper_joint_names],
        ],
        priority=priority,
        params={"start_position_tolerance": 0.05},
    )


_openarm_webxr_pink = PinkKinematicsConfig(
    dt=0.01,
    position_cost=8.0,
    orientation_cost=2.0,
    posture_cost=0.01,
    joint_limit_posture_margin=0.3,
    lm_damping=0.01,
    gain=0.25,
)
_openarm_webxr_task = TaskConfig(
    name=OPENARM_WEBXR_TASK_NAME,
    type="teleop_ik",
    priority=20,
    joint_names=list(_MODEL.joint_names),
    params={
        "robot_model": _MODEL,
        "bindings": [
            {
                "hand": "left",
                "target_frame": "openarm_left_grasp_frame",
            },
            {
                "hand": "right",
                "target_frame": "openarm_right_grasp_frame",
            },
        ],
        "solver_type": OpenArmPinkPoseTargetSolver,
        "pink": _openarm_webxr_pink,
        "timeout": 0.5,
        "max_command_tracking_error_deg": 10.0,
        "max_joint_velocity_rad_s": 2.0,
        "joint_velocity_limits_rad_s": _OPENARM_JOINT_VELOCITY_LIMITS_RAD_S,
        "joint_command_filter_cutoff_hz": 5.0,
    },
)


def openarm_teleop_control() -> Blueprint:
    """The WebXR task set over the OpenArm connection."""
    return autoconnect(
        DamiaoTeleopCoordinator.blueprint(
            instance_name="ControlCoordinator",
            model=_MODEL.model,
            tasks=[
                _openarm_webxr_task,
                # The left jaw is open at its joint's upper end, the right at its lower end.
                TaskConfig(
                    name="left_arm_gripper",
                    type="gripper",
                    joint_names=["arm/openarm_left_finger_joint1"],
                    priority=20,
                    params={"open_at": "upper"},
                    stream_bind={"gripper_command": "left_gripper_command"},
                ),
                TaskConfig(
                    name="right_arm_gripper",
                    type="gripper",
                    joint_names=["arm/openarm_right_finger_joint1"],
                    priority=20,
                    params={"open_at": "lower"},
                    stream_bind={"gripper_command": "right_gripper_command"},
                ),
                _trajectory_task(priority=10),
            ],
            zero_stiffness_joints=[],
        ),
        arm_connection(OpenArmConnection),
    ).remappings(arm_remaps(OpenArmConnection, DamiaoTeleopCoordinator))


# Both controller grips engage one fourteen-joint task.
teleop_webxr_openarm = autoconnect(
    ArmTeleopModule.blueprint(),
    openarm_teleop_control(),
    ManipulationModule.blueprint(
        gripper_tasks={
            "left_arm": "left_arm_gripper",
            "right_arm": "right_arm_gripper",
        },
        model=_MODEL,
        kinematics=_openarm_webxr_pink,
        visualization={"backend": "viser"},
    ),
).remappings(
    [
        (ArmTeleopModule, "left_controller_output", "left_cartesian_command"),
        (ArmTeleopModule, "left_gripper_command", "left_gripper_command"),
        (ArmTeleopModule, "right_controller_output", "right_cartesian_command"),
        (ArmTeleopModule, "right_gripper_command", "right_gripper_command"),
    ]
)
