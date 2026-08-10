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

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.common.blueprints import eef_twist_task
from dimos.robot.manipulators.openyam.config import (
    make_openyam_hardware,
    make_openyam_model_config,
)
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

_openyam_keyboard_hw = openyam_hardware()
_openyam_model = make_openyam_model_config()


def _eef_twist_task(*, priority: int = 10) -> TaskConfig:
    return TaskConfig(
        name=EEF_TWIST_TASK_NAME,
        type="eef_twist",
        joint_names=list(OPENYAM_ARM_JOINTS),
        priority=priority,
        params={"robot_model": _openyam_model},
    )


def _trajectory_task(*, priority: int = 10) -> TaskConfig:
    return joint_trajectory_task(OPENYAM_JOINTS, priority=priority)


def _gripper_task() -> TaskConfig:
    return TaskConfig(
        name=f"{OPENYAM_HARDWARE_ID}_gripper",
        type="gripper",
        joint_names=[OPENYAM_GRIPPER_JOINT],
        priority=20,
    )

keyboard_teleop_openyam = autoconnect(
    KeyboardTeleopModule.blueprint(),
    ControlCoordinator.blueprint(
        hardware=[_openyam_keyboard_hw],
        tasks=[
            eef_twist_task(
                _openyam_keyboard_hw,
                robot_model=_openyam_model,
                target_frame="yam_hand_tcp",
            )
        ],
    ),
    ManipulationModule.blueprint(
        model=_openyam_model,
        visualization={"backend": "viser"},
    ),
)
