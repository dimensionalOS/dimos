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

"""OpenArm teleop blueprints."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.common.blueprints import eef_twist_task
from dimos.robot.manipulators.openarm.config import (
    LEFT_CAN,
    OPENARM_V10_FK_MODEL,
    openarm_hardware,
    openarm_model_config,
    openarm_single_hardware,
    openarm_single_model_config,
)
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule
from dimos.teleop.openarm_mini.calibration import OpenArmMiniSide
from dimos.teleop.openarm_mini.teleop_module import OpenArmMiniTeleopModule


def _openarm_mini_servo_task(hw_name: str, joint_names: list[str]) -> TaskConfig:
    return TaskConfig(
        name=f"servo_{hw_name}",
        type="servo",
        joint_names=joint_names,
        priority=10,
    )


def _openarm_mini_teleop_config(*sides: OpenArmMiniSide) -> dict[str, tuple[OpenArmMiniSide, ...]]:
    return {"enabled_sides": tuple(sides)}


_teleop_hw = openarm_single_hardware()

keyboard_teleop_openarm_mock = autoconnect(
    KeyboardTeleopModule.blueprint(),
    ControlCoordinator.blueprint(
        hardware=[_teleop_hw],
        tasks=[eef_twist_task(_teleop_hw, model_path=OPENARM_V10_FK_MODEL, ee_joint_id=7)],
    ),
    ManipulationModule.blueprint(
        robots=[openarm_single_model_config()],
        visualization={"backend": "meshcat"},
    ),
)

_teleop_real_hw = openarm_single_hardware(adapter_type="openarm", address=LEFT_CAN)

keyboard_teleop_openarm = autoconnect(
    KeyboardTeleopModule.blueprint(),
    ControlCoordinator.blueprint(
        hardware=[_teleop_real_hw],
        tasks=[
            eef_twist_task(
                _teleop_real_hw,
                model_path=OPENARM_V10_FK_MODEL,
                ee_joint_id=7,
            )
        ],
    ),
    ManipulationModule.blueprint(
        robots=[openarm_single_model_config()],
        visualization={"backend": "meshcat"},
    ),
)

_openarm_mini_left_hw = openarm_hardware("left", adapter_type="mock")

openarm_mini_left_teleop_viser = autoconnect(
    OpenArmMiniTeleopModule.blueprint(**_openarm_mini_teleop_config("left")),
    ControlCoordinator.blueprint(
        hardware=[_openarm_mini_left_hw],
        tasks=[
            _openarm_mini_servo_task(
                _openarm_mini_left_hw.hardware_id,
                _openarm_mini_left_hw.joints,
            )
        ],
    ),
    ManipulationModule.blueprint(
        robots=[openarm_model_config("left")],
        visualization={"backend": "viser"},
    ),
)

_openarm_mini_right_hw = openarm_hardware("right", adapter_type="mock")

openarm_mini_right_teleop_viser = autoconnect(
    OpenArmMiniTeleopModule.blueprint(**_openarm_mini_teleop_config("right")),
    ControlCoordinator.blueprint(
        hardware=[_openarm_mini_right_hw],
        tasks=[
            _openarm_mini_servo_task(
                _openarm_mini_right_hw.hardware_id,
                _openarm_mini_right_hw.joints,
            )
        ],
    ),
    ManipulationModule.blueprint(
        robots=[openarm_model_config("right")],
        visualization={"backend": "viser"},
    ),
)

_openarm_mini_dual_left_hw = openarm_hardware("left", adapter_type="mock")
_openarm_mini_dual_right_hw = openarm_hardware("right", adapter_type="mock")

openarm_mini_dual_teleop_viser = autoconnect(
    OpenArmMiniTeleopModule.blueprint(**_openarm_mini_teleop_config("left", "right")),
    ControlCoordinator.blueprint(
        hardware=[_openarm_mini_dual_left_hw, _openarm_mini_dual_right_hw],
        tasks=[
            _openarm_mini_servo_task(
                _openarm_mini_dual_left_hw.hardware_id,
                _openarm_mini_dual_left_hw.joints,
            ),
            _openarm_mini_servo_task(
                _openarm_mini_dual_right_hw.hardware_id,
                _openarm_mini_dual_right_hw.joints,
            ),
        ],
    ),
    ManipulationModule.blueprint(
        robots=[openarm_model_config("left"), openarm_model_config("right")],
        visualization={"backend": "viser"},
    ),
)
