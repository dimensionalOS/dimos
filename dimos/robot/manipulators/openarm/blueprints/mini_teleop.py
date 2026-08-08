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

"""OpenArm Mini leader teleop blueprints for the bimanual OpenArm follower."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.openarm.config import (
    openarm_arm_joints,
    openarm_bimanual_model_config,
    openarm_hardware,
)
from dimos.teleop.openarm_mini.calibration import OpenArmMiniSide
from dimos.teleop.openarm_mini.teleop_module import OpenArmMiniTeleopModule


def _servo_task(side: OpenArmMiniSide) -> TaskConfig:
    return TaskConfig(
        name=f"servo_{side}_arm",
        type="servo",
        joint_names=openarm_arm_joints(side),
        priority=10,
    )


mini_teleop_openarm = autoconnect(
    OpenArmMiniTeleopModule.blueprint(enabled_sides=("left", "right")),
    ControlCoordinator.blueprint(
        hardware=[openarm_hardware()],
        tasks=[_servo_task("left"), _servo_task("right")],
    ),
    ManipulationModule.blueprint(
        robots=[openarm_bimanual_model_config()],
        visualization={"backend": "viser"},
    ),
)

mini_teleop_openarm_left = autoconnect(
    OpenArmMiniTeleopModule.blueprint(enabled_sides=("left",)),
    ControlCoordinator.blueprint(
        hardware=[openarm_hardware()],
        tasks=[_servo_task("left")],
    ),
    ManipulationModule.blueprint(
        robots=[openarm_bimanual_model_config()],
        visualization={"backend": "viser"},
    ),
)

mini_teleop_openarm_right = autoconnect(
    OpenArmMiniTeleopModule.blueprint(enabled_sides=("right",)),
    ControlCoordinator.blueprint(
        hardware=[openarm_hardware()],
        tasks=[_servo_task("right")],
    ),
    ManipulationModule.blueprint(
        robots=[openarm_bimanual_model_config()],
        visualization={"backend": "viser"},
    ),
)
