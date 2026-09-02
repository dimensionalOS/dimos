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

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinatorConfig, TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.robot.manipulators.openarm.config import (
    OPENARM_ARM_JOINTS,
    OPENARM_GRIPPER_JOINTS,
    OPENARM_JOINTS,
    openarm_bimanual_model_config,
    openarm_hardware,
)
from dimos.robot.manipulators.openarm.teleop_ik import OpenArmPinkPoseTargetSolver
from dimos.teleop.quest.quest_extensions import ArmBaseTeleopModule, ArmTeleopModule

OPENARM_QUEST_TASK_NAME = "teleop_openarm"

_OPENARM_ARM_VELOCITY_PROFILE_RAD_S = (1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0)
_OPENARM_JOINT_VELOCITY_LIMITS_RAD_S = {
    joint_name: velocity_limit
    for joint_name, velocity_limit in zip(
        OPENARM_ARM_JOINTS,
        _OPENARM_ARM_VELOCITY_PROFILE_RAD_S * 2,
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
    enable_base: bool = False
    # Portal RPC "host:port"; None takes the flowbase adapter default.
    base_address: str | None = None
    # Adds the pillar lift as read-only hardware so its joint is observed.
    enable_pillar: bool = False


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
        if self.config.enable_base:
            self.config.hardware.append(
                HardwareComponent(
                    hardware_id="base",
                    hardware_type=HardwareType.BASE,
                    joints=make_twist_base_joints("base"),
                    adapter_type="flowbase",
                    address=self.config.base_address,
                )
            )
        if self.config.enable_pillar:
            from dimos.robot.diy.alfred.pillar_connection import pillar_hardware

            self.config.hardware.append(pillar_hardware())
        super()._setup_from_config()


class _OpenArmManipulationModule(ManipulationModule):
    """Own the fixed OpenArm model outside blueprint CLI configuration."""

    def _initialize_planning(self) -> None:
        self.config.model = openarm_bimanual_model_config()
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


def teleop_quest_openarm_blueprint(
    *,
    publish_joint_targets: bool = False,
    enable_base: bool = False,
    enable_pillar: bool = False,
    teleop_module_cls: type[ArmTeleopModule] | None = None,
) -> Blueprint:
    """The OpenArm Quest teleop stack, with optional coordinator extras.

    Safe default: both controllers feed one bimanual task backed by in-memory
    hardware. Supplying both CAN ports selects the physical adapter. With
    enable_base the flowbase joins the coordinator and the thumbsticks drive
    it: right stick translates, left stick yaws. With enable_pillar the lift
    joint is observed through the coordinator.
    """
    if teleop_module_cls is None:
        teleop_module_cls = ArmBaseTeleopModule if enable_base else ArmTeleopModule
    base_tasks = (
        [
            TaskConfig(
                name="vel_base",
                type="velocity",
                joint_names=make_twist_base_joints("base"),
                priority=10,
            )
        ]
        if enable_base
        else []
    )
    return autoconnect(
        teleop_module_cls.blueprint(),
        OpenArmTeleopCoordinator.blueprint(
            instance_name="ControlCoordinator",
            publish_joint_targets=publish_joint_targets,
            enable_base=enable_base,
            enable_pillar=enable_pillar,
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
                *base_tasks,
            ],
        ),
        _OpenArmManipulationModule.blueprint(
            model=openarm_bimanual_model_config(),
            kinematics=_openarm_quest_pink,
            visualization={"backend": "viser"},
        ),
    ).remappings(
        [
            (teleop_module_cls, "left_controller_output", "left_cartesian_command"),
            (teleop_module_cls, "left_gripper_command", "left_gripper_command"),
            (teleop_module_cls, "right_controller_output", "right_cartesian_command"),
            (teleop_module_cls, "right_gripper_command", "right_gripper_command"),
        ]
    )


teleop_quest_openarm = autoconnect(teleop_quest_openarm_blueprint())
