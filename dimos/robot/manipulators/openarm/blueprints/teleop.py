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

"""OpenArm keyboard and Quest teleop blueprints."""

from __future__ import annotations

from dataclasses import replace

from dimos.control.components import HardwareComponent
from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig, TaskConfig
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.robot.manipulators.common.blueprints import (
    coordinator,
    planner,
)
from dimos.robot.manipulators.common.topics import DEFAULT_TRAJECTORY_TASK_NAME
from dimos.robot.manipulators.openarm.config import (
    OPENARM_ARM_JOINTS,
    OPENARM_DOF,
    OPENARM_GRIPPER_JOINTS,
    OPENARM_LEFT_MODEL,
    OPENARM_RIGHT_MODEL,
    openarm_arm_joints,
    openarm_bimanual_model_config,
    openarm_hardware,
    openarm_mock_hardware,
)
from dimos.robot.manipulators.openarm.teleop_ik import OpenArmPinkPoseTargetSolver
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule
from dimos.teleop.quest.quest_extensions import ArmTeleopModule

# The keyboard publishes twists to one task by name; the right arm's task
# keeps holding its anchor pose.
KEYBOARD_EEF_TASK_NAME = "eef_twist_left_arm"
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

_openarm_keyboard_hw = openarm_hardware()


def _eef_twist_task(side: str, *, priority: int = 10) -> TaskConfig:
    return TaskConfig(
        name=f"eef_twist_{side}_arm",
        type="eef_twist",
        joint_names=openarm_arm_joints(side),
        priority=priority,
        params={
            "model_path": OPENARM_LEFT_MODEL if side == "left" else OPENARM_RIGHT_MODEL,
            "ee_joint_id": OPENARM_DOF,
        },
    )


def _trajectory_task(*, priority: int = 10) -> TaskConfig:
    return TaskConfig(
        name=DEFAULT_TRAJECTORY_TASK_NAME,
        type="trajectory",
        joint_names=list(OPENARM_ARM_JOINTS),
        priority=priority,
        params={"start_position_tolerance": 0.05},
    )


keyboard_teleop_openarm = autoconnect(
    KeyboardTeleopModule.blueprint(task_name=KEYBOARD_EEF_TASK_NAME),
    ControlCoordinator.blueprint(
        hardware=[_openarm_keyboard_hw],
        tasks=[
            _eef_twist_task("left"),
            _eef_twist_task("right"),
        ],
    ),
    ManipulationModule.blueprint(
        robots=[openarm_bimanual_model_config()],
        visualization={"backend": "viser"},
    ),
)

_openarm_keyboard_planner_hw = openarm_hardware()

keyboard_teleop_openarm_planner = autoconnect(
    KeyboardTeleopModule.blueprint(task_name=KEYBOARD_EEF_TASK_NAME),
    planner(robots=[openarm_bimanual_model_config()]),
    coordinator(
        hardware=[_openarm_keyboard_planner_hw],
        tasks=[
            _eef_twist_task("left", priority=10),
            _eef_twist_task("right", priority=10),
            _trajectory_task(priority=20),
        ],
    ),
)


def _openarm_quest_hardware(
    left_can_port: str | None,
    right_can_port: str | None,
) -> HardwareComponent:
    if left_can_port is None and right_can_port is None:
        return openarm_mock_hardware()
    return openarm_hardware(
        left_can_port=left_can_port,
        right_can_port=right_can_port,
    )


class OpenArmTeleopCoordinatorConfig(ControlCoordinatorConfig):
    """OpenArm teleop deployment configuration."""

    left_can_port: str | None = None
    right_can_port: str | None = None


class OpenArmTeleopCoordinator(TeleopControlCoordinator):
    """Select fake or dual-CAN OpenArm hardware before normal coordinator setup."""

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
            _openarm_quest_hardware(
                self.config.left_can_port,
                self.config.right_can_port,
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
    position_cost=1.0,
    orientation_cost=1.0,
    posture_cost=1e-3,
    joint_limit_posture_margin=0.3,
    lm_damping=1e-6,
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
                "gripper_joint": OPENARM_GRIPPER_JOINTS[0],
                "gripper_open_position": 1.0,
                "gripper_closed_position": 0.0,
            },
            {
                "hand": "right",
                "target_frame": "openarm_right_grasp_frame",
                "gripper_joint": OPENARM_GRIPPER_JOINTS[1],
                "gripper_open_position": 1.0,
                "gripper_closed_position": 0.0,
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

# Safe default: without explicit CAN ports, both controllers feed one bimanual
# task backed by in-memory hardware. Supplying both CAN ports selects hardware.
teleop_quest_openarm = autoconnect(
    ArmTeleopModule.blueprint(
        task_names={"left": OPENARM_QUEST_TASK_NAME, "right": OPENARM_QUEST_TASK_NAME}
    ),
    OpenArmTeleopCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tasks=[
            _openarm_quest_task,
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
        (ArmTeleopModule, "right_controller_output", "right_cartesian_command"),
    ]
)
