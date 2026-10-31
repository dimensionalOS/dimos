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
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.robot.manipulators.common.damiao_blueprint import (
    DamiaoTeleopCoordinator,
    connect_damiao,
)
from dimos.robot.manipulators.openarm.connection import OpenArmConnection
from dimos.robot.manipulators.openarm.teleop_ik import OpenArmPinkPoseTargetSolver
from dimos.teleop.webxr.extensions import ArmTeleopModule

OPENARM_WEBXR_TASK_NAME = "teleop_openarm"
_DEPLOYMENT = OpenArmConnection.models()

_OPENARM_ARM_VELOCITY_PROFILE_RAD_S = (1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0)
_OPENARM_JOINT_VELOCITY_LIMITS_RAD_S = {
    joint_name: velocity_limit
    for joint_name, velocity_limit in zip(
        _DEPLOYMENT.arm_joints,
        _OPENARM_ARM_VELOCITY_PROFILE_RAD_S * 2,
        strict=True,
    )
}


def _trajectory_task(*, priority: int = 10) -> TaskConfig:
    return TaskConfig(
        name=JOINT_TRAJECTORY_TASK_NAME,
        type="trajectory",
        joint_names=list(_DEPLOYMENT.joints),
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
    joint_names=list(_DEPLOYMENT.arm_joints),
    params={
        "robot_model": _DEPLOYMENT.planning,
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

# Both controller grips engage one fourteen-joint task.
teleop_webxr_openarm = autoconnect(
    ArmTeleopModule.blueprint(),
    connect_damiao(
        OpenArmConnection,
        coordinator=DamiaoTeleopCoordinator,
        tasks=[
            _openarm_webxr_task,
            TaskConfig(
                name="left_arm_gripper",
                type="gripper",
                joint_names=[_DEPLOYMENT.gripper_joints[0]],
                priority=20,
                stream_bind={"gripper_command": "left_gripper_command"},
            ),
            TaskConfig(
                name="right_arm_gripper",
                type="gripper",
                joint_names=[_DEPLOYMENT.gripper_joints[1]],
                priority=20,
                stream_bind={"gripper_command": "right_gripper_command"},
            ),
            _trajectory_task(priority=10),
        ],
    ),
    ManipulationModule.blueprint(
        model=_DEPLOYMENT.planning,
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
