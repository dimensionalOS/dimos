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

"""A-750 teleop blueprints."""

from __future__ import annotations

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.a750.config import (
    a750_hardware,
    make_a750_model_config,
)
from dimos.robot.manipulators.common.blueprints import eef_twist_task
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

_a750_hw = a750_hardware("arm", mock_without_address=True)
_a750_model = make_a750_model_config()

keyboard_teleop_a750 = autoconnect(
    KeyboardTeleopModule.blueprint(),
    ControlCoordinator.blueprint(
        tick_rate=100.0,
        publish_joint_state=True,
        joint_state_frame_id="coordinator",
        hardware=[_a750_hw],
        tasks=[
            eef_twist_task(
                _a750_hw,
                robot_model=_a750_model,
            )
        ],
    ),
    ManipulationModule.blueprint(
        robots=[_a750_model],
        visualization={"backend": "meshcat"},
    ),
)
