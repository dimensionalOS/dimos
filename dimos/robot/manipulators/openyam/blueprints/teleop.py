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

"""OpenYAM keyboard teleop blueprints."""

from __future__ import annotations

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.common.blueprints import (
    coordinator,
    planner,
)
from dimos.robot.manipulators.common.coordinators import (
    ArmTwistCoordinator,
)
from dimos.robot.manipulators.common.topics import (
    EEF_TWIST_TASK_NAME,
)
from dimos.robot.manipulators.openyam.config import (
    OPENYAM_ARM_JOINTS,
    OPENYAM_GRIPPER_JOINT,
    make_openyam_model_config,
    openyam_hardware,
)
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

_openyam_keyboard_hw = openyam_hardware()
_openyam_model = make_openyam_model_config(name="arm")


def _eef_twist_task(*, priority: int = 10) -> TaskConfig:
    return TaskConfig(
        name=EEF_TWIST_TASK_NAME,
        type="eef_twist",
        joint_names=list(OPENYAM_ARM_JOINTS),
        priority=priority,
        params={"robot_model": _openyam_model},
    )


def _trajectory_task(*, priority: int = 10) -> TaskConfig:
    return TaskConfig(
        name=JOINT_TRAJECTORY_TASK_NAME,
        type="trajectory",
        joint_names=list(OPENYAM_ARM_JOINTS),
        priority=priority,
        params={"start_position_tolerance": 0.05},
    )


def _gripper_task() -> TaskConfig:
    return TaskConfig(
        name="servo_gripper",
        type="servo",
        joint_names=[OPENYAM_GRIPPER_JOINT],
        priority=20,
        params={"timeout": 0.0},
    )


keyboard_teleop_openyam = autoconnect(
    KeyboardTeleopModule.blueprint(),
    ArmTwistCoordinator.blueprint(
        instance_name="ControlCoordinator",
        hardware=[_openyam_keyboard_hw],
        tasks=[
            _eef_twist_task(),
            _gripper_task(),
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_openyam_model],
        visualization={"backend": "viser"},
    ),
)

_openyam_keyboard_planner_hw = openyam_hardware()

keyboard_teleop_openyam_planner = autoconnect(
    KeyboardTeleopModule.blueprint(),
    planner(robots=[_openyam_model]),
    coordinator(
        hardware=[_openyam_keyboard_planner_hw],
        tasks=[
            _eef_twist_task(priority=10),
            _gripper_task(),
            _trajectory_task(priority=20),
        ],
    ),
)
