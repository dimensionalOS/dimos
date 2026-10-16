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

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.mapping.costmapper import Config, CostMapper
from dimos.mapping.pointclouds.occupancy import (
    GeneralOccupancyConfig,
    HeightCostConfig,
    SimpleOccupancyConfig,
)


def test_blueprint_parser_preserves_height_cost_config() -> None:
    blueprint = CostMapper.blueprint(
        config=HeightCostConfig(
            resolution=0.05,
            can_pass_under=1.7,
            can_climb=0.1,
        )
    )

    parsed = BlueprintConfigParser(blueprint).parse(environ={})
    config = Config(**parsed.module_kwargs(CostMapper.name))

    assert config.config == HeightCostConfig(
        resolution=0.05,
        can_pass_under=1.7,
        can_climb=0.1,
    )


def test_blueprint_parser_applies_general_defaults_to_sparse_config() -> None:
    blueprint = CostMapper.blueprint(algo="general", config={"min_height": 0.2})

    parsed = BlueprintConfigParser(blueprint).parse(environ={})
    config = Config(**parsed.module_kwargs(CostMapper.name))

    assert config.config == GeneralOccupancyConfig(min_height=0.2)


def test_blueprint_parser_applies_simple_defaults_to_sparse_config() -> None:
    blueprint = CostMapper.blueprint(algo="simple", config={"closing_iterations": 2})

    parsed = BlueprintConfigParser(blueprint).parse(environ={})
    config = Config(**parsed.module_kwargs(CostMapper.name))

    assert config.config == SimpleOccupancyConfig(closing_iterations=2)
