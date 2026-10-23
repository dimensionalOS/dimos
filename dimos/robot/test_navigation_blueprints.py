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

import importlib.util
import sys

import pytest
from pytest_mock import MockerFixture

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.mapping.costmapper import Config, CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.robot.all_blueprints import all_blueprints


@pytest.mark.parametrize(
    ("blueprint_name", "simulation"),
    [
        ("unitree-g1-nav-simple", ""),
        ("unitree-g1-groot-wbc", ""),
        ("unitree-g1-groot-wbc", "mujoco"),
        ("r1pro-nav", ""),
    ],
)
def test_navigation_blueprint_preserves_costmapper_config(
    blueprint_name: str,
    simulation: str,
    monkeypatch: pytest.MonkeyPatch,
    mocker: MockerFixture,
) -> None:
    monkeypatch.setattr(global_config, "simulation", simulation)
    monkeypatch.setattr(global_config, "scene_package", None)
    # The onboard hardware stack imports the optional Unitree DDS SDK. Keep this
    # test focused on the navigation config without requiring that hardware extra.
    monkeypatch.setitem(
        sys.modules,
        "dimos.robot.unitree.g1.blueprints.primitive.unitree_g1_onboard",
        mocker.Mock(spec=["_unitree_g1_onboard"], _unitree_g1_onboard=Blueprint(blueprints=())),
    )
    module_name, variable = all_blueprints[blueprint_name].split(":")
    # Re-evaluate the simulation branch and restore any cached module after the test.
    spec = importlib.util.find_spec(module_name)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, module_name, module)
    spec.loader.exec_module(module)
    blueprint = getattr(module, variable)
    assert isinstance(blueprint, Blueprint)
    costmapper = next(atom for atom in blueprint.blueprints if atom.module is CostMapper)

    parsed = BlueprintConfigParser(blueprint).parse(environ={})
    config = Config(**parsed.module_kwargs(costmapper.name))

    assert isinstance(config.config, HeightCostConfig)
    assert config.config == costmapper.kwargs["config"]
    assert config.initial_safe_radius_meters == costmapper.kwargs["initial_safe_radius_meters"]
