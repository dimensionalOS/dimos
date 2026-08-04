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

"""OpenArm keyboard teleop blueprints."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.common.blueprints import coordinator, planner
from dimos.robot.manipulators.common.topics import DEFAULT_TRAJECTORY_TASK_NAME
from dimos.robot.manipulators.openarm.config import (
    OPENARM_GRIPPER_JOINTS,
    openarm_arm_joints,
    openarm_hardware,
    openarm_model_config,
)
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

# The keyboard publishes twists to one task by name; the right arm's task
# keeps holding its anchor pose.
KEYBOARD_EEF_TASK_NAME = "eef_twist_left_arm"

_openarm_keyboard_hw = openarm_hardware()
_openarm_models = {
    side: openarm_model_config(side) for side in ("left", "right")
}


def _eef_twist_task(side: str, *, priority: int = 10) -> TaskConfig:
    return TaskConfig(
        name=f"eef_twist_{side}_arm",
        type="eef_twist",
        joint_names=openarm_arm_joints(side),
        priority=priority,
        params={"control_ik": {"robot_model": _openarm_models[side]}},
    )


def _trajectory_task(*, priority: int = 10) -> TaskConfig:
    return TaskConfig(
        name=DEFAULT_TRAJECTORY_TASK_NAME,
        type="trajectory",
        joint_names=[*openarm_arm_joints("left"), *openarm_arm_joints("right")],
        priority=priority,
        params={"start_position_tolerance": 0.05},
    )


def _gripper_task() -> TaskConfig:
    return TaskConfig(
        name="servo_grippers",
        type="servo",
        joint_names=list(OPENARM_GRIPPER_JOINTS),
        priority=20,
        params={"timeout": 0.0},
    )


keyboard_teleop_openarm = autoconnect(
    KeyboardTeleopModule.blueprint(
        task_name=KEYBOARD_EEF_TASK_NAME,
        gripper_joint_names=list(OPENARM_GRIPPER_JOINTS),
    ),
    ControlCoordinator.blueprint(
        hardware=[_openarm_keyboard_hw],
        tasks=[
            _eef_twist_task("left"),
            _eef_twist_task("right"),
            _gripper_task(),
        ],
    ),
    ManipulationModule.blueprint(
        robots=list(_openarm_models.values()),
        visualization={"backend": "viser"},
    ),
)

_openarm_keyboard_planner_hw = openarm_hardware()

keyboard_teleop_openarm_planner = autoconnect(
    KeyboardTeleopModule.blueprint(
        task_name=KEYBOARD_EEF_TASK_NAME,
        gripper_joint_names=list(OPENARM_GRIPPER_JOINTS),
    ),
    planner(robots=list(_openarm_models.values())),
    coordinator(
        hardware=[_openarm_keyboard_planner_hw],
        tasks=[
            _eef_twist_task("left", priority=10),
            _eef_twist_task("right", priority=10),
            _gripper_task(),
            _trajectory_task(priority=20),
        ],
    ),
)
