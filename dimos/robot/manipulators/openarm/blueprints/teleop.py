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
    OPENARM_ARM_JOINTS,
    OPENARM_DOF,
    OPENARM_LEFT_MODEL,
    OPENARM_RIGHT_MODEL,
    openarm_arm_joints,
    openarm_bimanual_model_config,
    openarm_hardware,
)
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

# The keyboard publishes twists to one task by name; the right arm's task
# keeps holding its anchor pose.
KEYBOARD_EEF_TASK_NAME = "eef_twist_left_arm"

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
