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

import numpy as np
import pytest

from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.mapping.costmapper import Config, CostMapper
from dimos.mapping.pointclouds.occupancy import (
    HeightCostConfig,
    general_occupancy,
    height_cost_occupancy,
)
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def test_general_algorithm_accepts_base_config_dict() -> None:
    config = Config(algo="general", config={"resolution": 1.0})
    cloud = PointCloud2.from_numpy(np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 0.5]]), frame_id="map")

    grid = general_occupancy(cloud, **asdict(config.config))

    assert grid.resolution == 1.0
    np.testing.assert_array_equal(grid.grid, [[-1, 0, -1], [0, 0, 0], [-1, 0, 100]])


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


def _floor(x_range, y_range, z=0.0):
    xs, ys = np.meshgrid(np.arange(*x_range, 0.02), np.arange(*y_range, 0.02))
    return np.column_stack([xs.ravel(), ys.ravel(), np.full(xs.size, z)])


def _max_cost_near(grid, x, y, radius=0.1):
    cell = grid.world_to_grid((x, y, 0.0))
    cells = int(radius / grid.resolution)
    col, row = round(cell.x), round(cell.y)
    return grid.grid[row - cells : row + cells + 1, col - cells : col + cells + 1].max()


def test_height_cost_keeps_walls_whose_cells_hold_floor_returns() -> None:
    # A thin wall at x=1.0 standing on the floor: each wall cell also holds floor points.
    wall_ys, wall_zs = np.meshgrid(np.arange(0.0, 2.0, 0.02), np.arange(0.0, 1.5, 0.05))
    wall = np.column_stack([np.full(wall_ys.size, 1.0), wall_ys.ravel(), wall_zs.ravel()])
    # A table top at x=-1.0, 0.8 m up, which the robot can pass under.
    table = _floor((-1.2, -0.8), (0.0, 2.0), z=0.8)
    points = np.vstack([_floor((-2.0, 2.0), (0.0, 2.0)), wall, table])

    grid = height_cost_occupancy(
        PointCloud2.from_numpy(points, frame_id="map"), can_pass_under=0.6, can_climb=0.15
    )

    assert _max_cost_near(grid, 1.0, 1.0) == 100
    assert _max_cost_near(grid, -1.0, 1.0) == 0
