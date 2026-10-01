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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

from collections.abc import Callable

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.nav_msgs.msg import OccupancyGrid
from dimos_generated.sensor_msgs.msg import PointCloud2
from dimos_generated.std_msgs.msg import Header
import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.mapping.pointclouds.occupancy import (
    general_occupancy,
    height_cost_occupancy,
    simple_occupancy,
)
from dimos.msgs.occupancy import occupancy_view
from dimos.msgs.pointcloud import pointcloud_from_xyz


@pytest.mark.parametrize("algorithm", [general_occupancy, height_cost_occupancy, simple_occupancy])
def test_generated_cloud_to_grid(algorithm: Callable[..., OccupancyGrid]) -> None:
    header = Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="map")
    xy = np.arange(0, 2, 0.1)
    x, y = np.meshgrid(xy, xy)
    points = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    cloud = pointcloud_from_xyz(points, header=header)
    decoded = PointCloud2.decode(cloud.encode())
    grid = algorithm(decoded, resolution=0.1)
    restored = OccupancyGrid.decode(grid.encode())
    assert restored.header == header
    cells = occupancy_view(restored)
    assert cells.shape == (39, 39)
    assert np.any(cells == 0)
    assert np.any(cells == -1)
    assert np.all((cells >= -1) & (cells <= 100))
    override = algorithm(decoded, resolution=0.1, frame_id="world")
    assert override.header.frame_id == "world"
    assert override.header.stamp == header.stamp
    assert decoded.header == header


@pytest.mark.parametrize("algorithm", [general_occupancy, height_cost_occupancy, simple_occupancy])
@pytest.mark.parametrize("points", [np.empty((0, 3)), np.array([[np.nan, 0, 0]])])
def test_empty_or_missing_points_remain_unknown(
    algorithm: Callable[..., OccupancyGrid],
    points: NDArray[np.float64],
) -> None:
    cloud = pointcloud_from_xyz(points, header=Header(stamp=Time(nanosec=123), frame_id="map"))
    grid = algorithm(cloud)
    assert grid.header == cloud.header
    assert occupancy_view(grid).tolist() == [[-1]]
    assert grid.info.origin.orientation.w == 1


@pytest.mark.parametrize("resolution", [0, -1, float("nan"), float("inf")])
def test_invalid_resolution(resolution: float) -> None:
    cloud = pointcloud_from_xyz(np.zeros((1, 3)), header=Header())
    with pytest.raises(ValueError, match="resolution"):
        general_occupancy(cloud, resolution=resolution)


def test_inflation_preserves_metadata_and_source() -> None:
    cloud = pointcloud_from_xyz(
        np.array([[0, 0, 1.0]]), header=Header(stamp=Time(nanosec=789), frame_id="map")
    )
    grid = general_occupancy(cloud, resolution=0.1)
    original = occupancy_view(grid).copy()
    inflated = simple_inflate(grid, 0.2)
    assert inflated.header == grid.header
    assert inflated.info == grid.info
    assert np.count_nonzero(occupancy_view(inflated) == 100) == 13
    np.testing.assert_array_equal(occupancy_view(grid), original)
    inflated.header.frame_id = "changed"
    assert grid.header.frame_id == "map"
    assert simple_inflate(grid, 0) == grid
    for radius in [-1, float("nan"), float("inf")]:
        with pytest.raises(ValueError, match="radius"):
            simple_inflate(grid, radius)
