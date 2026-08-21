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
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.common.blueprints import coordinator, planner
from dimos.robot.manipulators.openarm.config import (
    OPENARM_ARM_JOINTS,
    openarm_arm_joints,
    openarm_bimanual_model_config,
    openarm_control_model_config,
    openarm_hardware,
)
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

# The keyboard publishes twists to one task by name; the right arm's task
# keeps holding its anchor pose.
KEYBOARD_EEF_TASK_NAME = "eef_twist_left_arm"

_openarm_keyboard_hw = openarm_hardware()
_openarm_control_models = {side: openarm_control_model_config(side) for side in ("left", "right")}
_openarm_planning_model = openarm_bimanual_model_config()


def _eef_twist_task(side: str, *, priority: int = 10) -> TaskConfig:
    return TaskConfig(
        name=f"eef_twist_{side}_arm",
        type="eef_twist",
        joint_names=openarm_arm_joints(side),
        priority=priority,
        params={"control_ik": {"robot_model": _openarm_control_models[side]}},
    )


def _trajectory_task(*, priority: int = 10) -> TaskConfig:
    return TaskConfig(
        name=JOINT_TRAJECTORY_TASK_NAME,
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
        robots=[_openarm_planning_model],
        visualization={"backend": "viser"},
    ),
)

_openarm_keyboard_planner_hw = openarm_hardware()

keyboard_teleop_openarm_planner = autoconnect(
    KeyboardTeleopModule.blueprint(task_name=KEYBOARD_EEF_TASK_NAME),
    planner(robots=[_openarm_planning_model]),
    coordinator(
        hardware=[_openarm_keyboard_planner_hw],
        tasks=[
            _eef_twist_task("left", priority=10),
            _eef_twist_task("right", priority=10),
            _trajectory_task(priority=20),
        ],
    ),
)
