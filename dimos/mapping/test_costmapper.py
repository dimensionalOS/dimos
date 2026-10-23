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

from dataclasses import asdict, replace

import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.mapping.costmapper import Config, CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig


@pytest.mark.parametrize("serialized", [False, True])
def test_height_cost_config_survives_blueprint_parsing(serialized: bool) -> None:
    height_config = HeightCostConfig(
        resolution=0.08,
        can_pass_under=1.4,
        can_climb=0.10,
        ignore_noise=0.02,
        smoothing=0.5,
    )
    blueprint = CostMapper.blueprint(
        config=asdict(height_config) if serialized else height_config,
    )

    parsed = BlueprintConfigParser(blueprint).parse(environ={})
    config = Config(**parsed.module_kwargs("costmapper"))

    assert config.config == height_config


def test_partial_height_cost_environment_override_preserves_blueprint_settings() -> None:
    height_config = HeightCostConfig(can_pass_under=1.9, can_climb=0.03)
    blueprint = CostMapper.blueprint(config=height_config)

    parsed = BlueprintConfigParser(blueprint).parse(
        environ={
            "COSTMAPPER__CONFIG__SMOOTHING": "0.0",
            "COSTMAPPER__CONFIG__IGNORE_NOISE": "0.01",
        },
    )
    config = Config(**parsed.module_kwargs("costmapper"))

    assert config.config == replace(height_config, smoothing=0.0, ignore_noise=0.01)
