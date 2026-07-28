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
    OPENYAM_DOF,
    OPENYAM_MODEL_PATH,
    make_openyam_hardware,
    make_openyam_model_config,
)
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

_openyam_keyboard_hw = make_openyam_hardware("arm")

keyboard_teleop_openyam = autoconnect(
    KeyboardTeleopModule.blueprint(),
    ControlCoordinator.blueprint(
        hardware=[_openyam_keyboard_hw],
        tasks=[
            eef_twist_task(
                _openyam_keyboard_hw,
                model_path=OPENYAM_MODEL_PATH,
                ee_joint_id=OPENYAM_DOF,
            )
        ],
    ),
    ManipulationModule.blueprint(
        robots=[make_openyam_model_config(name="arm")],
        visualization={"backend": "viser"},
    ),
)
