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
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.common.blueprints import (
    coordinator,
    eef_twist_task,
    planner,
    trajectory_task,
)
from dimos.robot.manipulators.common.coordinators import (
    ArmTwistCoordinator,
)
from dimos.robot.manipulators.openyam.config import (
    make_openyam_model_config,
    openyam_hardware,
)
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

_openyam_keyboard_hw = openyam_hardware("arm")
_openyam_model = make_openyam_model_config(name="arm")


def _gripper_task() -> TaskConfig:
    return TaskConfig(
        name="servo_gripper",
        type="servo",
        joint_names=["arm/gripper"],
        priority=20,
        params={"timeout": 0.0, "default_positions": [0.0]},
    )


keyboard_teleop_openyam = autoconnect(
    KeyboardTeleopModule.blueprint(),
    ArmTwistCoordinator.blueprint(
        instance_name="ControlCoordinator",
        hardware=[_openyam_keyboard_hw],
        tasks=[
            eef_twist_task(
                _openyam_keyboard_hw,
                robot_model=_openyam_model,
            ),
            _gripper_task(),
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_openyam_model],
        visualization={"backend": "viser"},
    ),
)

_openyam_keyboard_planner_hw = openyam_hardware("arm")

keyboard_teleop_openyam_planner = autoconnect(
    KeyboardTeleopModule.blueprint(),
    planner(robots=[make_openyam_model_config(name="arm")]),
    coordinator(
        hardware=[_openyam_keyboard_planner_hw],
        tasks=[
            eef_twist_task(
                _openyam_keyboard_planner_hw,
                robot_model=_openyam_model,
                priority=10,
            ),
            _gripper_task(),
            trajectory_task(_openyam_keyboard_planner_hw, priority=20),
        ],
    ),
)
