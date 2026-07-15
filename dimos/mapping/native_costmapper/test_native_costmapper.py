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

from dataclasses import asdict

import numpy as np
import pytest

pytest.importorskip("dimos_native_costmapper")

from dimos_native_costmapper import CostMapper as NativeCostMapper  # type: ignore[import-untyped]

from dimos.mapping.costmapper import Config as PythonModuleConfig, CostMapper as PythonCostMapper
from dimos.mapping.native_costmapper.module import (
    Config as NativeModuleConfig,
    NativeCostMapper as NativeCostMapperModule,
)
from dimos.mapping.pointclouds.occupancy import (
    OCCUPANCY_ALGOS,
    GeneralOccupancyConfig,
    HeightCostConfig,
    OccupancyConfig,
    SimpleOccupancyConfig,
)
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def terrain_cloud() -> PointCloud2:
    x_coordinates, y_coordinates = np.meshgrid(
        np.arange(-1.0, 1.01, 0.25),
        np.arange(-1.0, 1.01, 0.25),
    )
    heights = 0.02 * x_coordinates + np.where(x_coordinates > 0.25, 0.25, 0.0)
    points = np.column_stack(
        (x_coordinates.ravel(), y_coordinates.ravel(), heights.ravel())
    ).astype(np.float32)
    points = np.vstack(
        (
            points,
            np.array([[0.0, 0.0, 0.5], [0.5, 0.0, 1.0]], dtype=np.float32),
        )
    )
    return PointCloud2.from_numpy(points, frame_id="map", timestamp=123.456)


def calculate_native(
    cloud: PointCloud2,
    algorithm: str,
    config: OccupancyConfig,
    *,
    initial_safe_radius_meters: float = 0.0,
) -> OccupancyGrid:
    mapper = NativeCostMapper(
        algo=algorithm,
        config=asdict(config),
        initial_safe_radius_meters=initial_safe_radius_meters,
    )
    return OccupancyGrid.lcm_decode(mapper.calculate_costmap(cloud.lcm_encode()))


@pytest.mark.parametrize(
    ("algorithm", "config"),
    [
        (
            "height_cost",
            HeightCostConfig(
                resolution=0.25,
                frame_id="costmap",
                ignore_noise=0.01,
                smoothing=1.0,
            ),
        ),
        (
            "general",
            GeneralOccupancyConfig(
                resolution=0.25,
                frame_id="costmap",
                mark_free_radius=0.5,
            ),
        ),
        (
            "simple",
            SimpleOccupancyConfig(resolution=0.25, frame_id="costmap"),
        ),
    ],
)
def test_native_costmap_matches_python(
    algorithm: str,
    config: OccupancyConfig,
) -> None:
    cloud = terrain_cloud()
    expected = OCCUPANCY_ALGOS[algorithm](cloud, **asdict(config))

    actual = calculate_native(cloud, algorithm, config)

    np.testing.assert_array_equal(actual.grid, expected.grid)
    assert actual.resolution == pytest.approx(expected.resolution)
    assert actual.origin.position.x == pytest.approx(expected.origin.position.x)
    assert actual.origin.position.y == pytest.approx(expected.origin.position.y)
    assert actual.origin.orientation.w == pytest.approx(expected.origin.orientation.w)
    assert actual.frame_id == expected.frame_id
    assert actual.ts == pytest.approx(expected.ts)


def test_native_costmap_applies_initial_safe_radius() -> None:
    cloud = terrain_cloud()
    config = SimpleOccupancyConfig(resolution=0.25)
    radius = 0.4
    expected = OCCUPANCY_ALGOS["simple"](cloud, **asdict(config))
    rows, columns = np.ogrid[: expected.height, : expected.width]
    world_x = columns * expected.resolution + expected.origin.position.x
    world_y = rows * expected.resolution + expected.origin.position.y
    effective_radius = radius + expected.resolution * 0.5
    expected.grid[world_x**2 + world_y**2 <= effective_radius**2] = 0

    actual = calculate_native(
        cloud,
        "simple",
        config,
        initial_safe_radius_meters=radius,
    )

    np.testing.assert_array_equal(actual.grid, expected.grid)


def test_native_costmap_handles_empty_cloud() -> None:
    cloud = PointCloud2.from_numpy(
        np.empty((0, 3), dtype=np.float32),
        frame_id="empty_map",
        timestamp=123.456,
    )

    result = calculate_native(cloud, "simple", SimpleOccupancyConfig(resolution=0.2))

    np.testing.assert_array_equal(result.grid, np.array([[-1]], dtype=np.int8))
    assert result.resolution == pytest.approx(0.2)
    assert result.frame_id == "empty_map"
    assert result.ts == pytest.approx(123.456)


def test_native_costmapper_rejects_unknown_algorithm() -> None:
    with pytest.raises(ValueError, match="unknown occupancy algorithm"):
        NativeCostMapper(algo="missing", config={}, initial_safe_radius_meters=0.0)


def test_native_costmapper_requires_python_owned_configuration() -> None:
    config = asdict(SimpleOccupancyConfig())

    with pytest.raises(TypeError, match="initial_safe_radius_meters"):
        NativeCostMapper(algo="simple", config=config)

    del config["frame_id"]
    with pytest.raises(ValueError, match="missing field `frame_id`"):
        NativeCostMapper(
            algo="simple",
            config=config,
            initial_safe_radius_meters=0.0,
        )


def test_native_module_matches_python_stream_contract() -> None:
    python_streams = PythonCostMapper.blueprint().blueprints[0].streams
    native_streams = NativeCostMapperModule.blueprint().blueprints[0].streams

    assert native_streams == python_streams


def test_native_module_matches_python_configuration_contract() -> None:
    python_config = PythonModuleConfig()
    native_config = NativeModuleConfig()

    assert native_config.algo == python_config.algo
    assert native_config.config == python_config.config
    assert native_config.initial_safe_radius_meters == python_config.initial_safe_radius_meters
