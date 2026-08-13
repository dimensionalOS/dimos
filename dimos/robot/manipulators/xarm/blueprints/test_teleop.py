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

import pytest

from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import (
    ManipulationModule,
    ManipulationModuleConfig,
)
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.robot.manipulators.xarm.blueprints.teleop import (
    keyboard_teleop_xarm6,
    keyboard_teleop_xarm7,
)


@pytest.mark.parametrize("blueprint", [keyboard_teleop_xarm6, keyboard_teleop_xarm7])
def test_keyboard_teleop_uses_roboplan_compatible_visualization(blueprint: Blueprint) -> None:
    manipulation = next(atom for atom in blueprint.blueprints if atom.module is ManipulationModule)
    config = ManipulationModuleConfig.model_validate(manipulation.kwargs)

    assert config.world_backend == "roboplan"
    assert isinstance(config.visualization, ViserVisualizationConfig)
    assert config.visualization.requires_world_visualization is False
