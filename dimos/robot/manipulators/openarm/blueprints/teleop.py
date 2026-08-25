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

"""OpenArm Quest teleop blueprint."""

from __future__ import annotations

from dataclasses import replace

from dimos.control.coordinator import ControlCoordinatorConfig, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.robot.manipulators.openarm.config import (
    OPENARM_ARM_JOINTS,
    OPENARM_GRIPPER_JOINTS,
    OPENARM_JOINTS,
    openarm_arm_joints,
    openarm_bimanual_model_config,
    openarm_hardware,
)
from dimos.robot.manipulators.openarm.teleop_ik import OpenArmPinkPoseTargetSolver
from dimos.teleop.quest.quest_extensions import ArmTeleopModule

OPENARM_QUEST_TASK_NAME = "teleop_openarm"

_OPENARM_ARM_VELOCITY_PROFILE_RAD_S = (1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0)
_OPENARM_JOINT_VELOCITY_LIMITS_RAD_S = {
    joint_name: velocity_limit
    for side in ("left", "right")
    for joint_name, velocity_limit in zip(
        openarm_arm_joints(side),
        _OPENARM_ARM_VELOCITY_PROFILE_RAD_S,
        strict=True,
    )
}


def _trajectory_task(*, priority: int = 10) -> TaskConfig:
    return TaskConfig(
        name=JOINT_TRAJECTORY_TASK_NAME,
        type="trajectory",
        joint_names=list(OPENARM_JOINTS),
        priority=priority,
        params={"start_position_tolerance": 0.05},
    )


class OpenArmTeleopCoordinatorConfig(ControlCoordinatorConfig):
    """OpenArm teleop deployment configuration requiring a complete bus pair."""

    left_can_port: str | None = None
    right_can_port: str | None = None


class OpenArmTeleopCoordinator(TeleopControlCoordinator):
    """Install the fixed OpenArm model and resolved hardware adapter."""

    config: OpenArmTeleopCoordinatorConfig

    def _setup_from_config(self) -> None:
        self.config.tasks = [
            replace(
                task,
                params={
                    **task.params,
                    "robot_model": openarm_bimanual_model_config(),
                },
            )
            if task.name == OPENARM_QUEST_TASK_NAME
            else task
            for task in self.config.tasks
        ]
        self.config.hardware = [
            openarm_hardware(
                left_can_port=self.config.left_can_port,
                right_can_port=self.config.right_can_port,
            )
        ]
        super()._setup_from_config()


class _OpenArmManipulationModule(ManipulationModule):
    """Own the fixed OpenArm model outside blueprint CLI configuration."""

    def _initialize_planning(self) -> None:
        self.config.robots = [openarm_bimanual_model_config()]
        super()._initialize_planning()


_openarm_quest_pink = PinkKinematicsConfig(
    dt=0.01,
    position_cost=8.0,
    orientation_cost=2.0,
    posture_cost=0.01,
    joint_limit_posture_margin=0.3,
    lm_damping=0.01,
    gain=0.25,
)
_openarm_quest_task = TaskConfig(
    name=OPENARM_QUEST_TASK_NAME,
    type="teleop_ik",
    joint_names=OPENARM_ARM_JOINTS,
    params={
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
        "pink": _openarm_quest_pink,
        "timeout": 0.5,
        "max_command_tracking_error_deg": 10.0,
        "max_joint_velocity_rad_s": 2.0,
        "joint_velocity_limits_rad_s": _OPENARM_JOINT_VELOCITY_LIMITS_RAD_S,
        "joint_command_filter_cutoff_hz": 5.0,
    },
)

# Safe default: both controllers feed one bimanual task backed by in-memory
# hardware. Supplying both CAN ports selects the physical adapter.
teleop_quest_openarm = autoconnect(
    ArmTeleopModule.blueprint(),
    OpenArmTeleopCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tasks=[
            _openarm_quest_task,
            TaskConfig(
                name="left_arm_gripper",
                type="gripper",
                joint_names=[OPENARM_GRIPPER_JOINTS[0]],
                priority=20,
                stream_bind={"gripper_command": "left_gripper_command"},
            ),
            TaskConfig(
                name="right_arm_gripper",
                type="gripper",
                joint_names=[OPENARM_GRIPPER_JOINTS[1]],
                priority=20,
                stream_bind={"gripper_command": "right_gripper_command"},
            ),
            _trajectory_task(priority=20),
        ],
    ),
    _OpenArmManipulationModule.blueprint(
        kinematics=_openarm_quest_pink,
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
