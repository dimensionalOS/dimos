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

from typing import Any

from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.manipulation.visualization.config import NoManipulationVisualizationConfig
from dimos.robot.manipulators.common.blueprints import planner
from dimos.robot.manipulators.xarm.blueprints.basic import (
    dual_xarm6_planner,
    xarm6_planner_only,
    xarm7_planner_coordinator,
)
from dimos.robot.manipulators.xarm.config import make_xarm7_model_config


def _manipulation_kwargs(blueprint: Blueprint) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is ManipulationModule)


def _manipulation_config(blueprint: Blueprint) -> ManipulationModuleConfig:
    return ManipulationModuleConfig(**_manipulation_kwargs(blueprint))


def test_planner_helper_defaults_to_no_visualization() -> None:
    blueprint = planner(robots=[make_xarm7_model_config(name="arm", add_gripper=True)])

    kwargs = _manipulation_kwargs(blueprint)
    config = ManipulationModuleConfig(**kwargs)

    assert "visualization" not in kwargs
    assert isinstance(config.visualization, NoManipulationVisualizationConfig)


def test_planner_helper_preserves_explicit_visualization() -> None:
    blueprint = planner(
        robots=[make_xarm7_model_config(name="arm", add_gripper=True)],
        visualization={"backend": "meshcat"},
    )

    assert _manipulation_kwargs(blueprint)["visualization"] == {"backend": "meshcat"}


def test_xarm_planner_blueprints_default_to_no_visualization() -> None:
    for blueprint in (xarm6_planner_only, dual_xarm6_planner, xarm7_planner_coordinator):
        config = _manipulation_config(blueprint)

        assert isinstance(config.visualization, NoManipulationVisualizationConfig)
