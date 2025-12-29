#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

import cv2
import numpy as np
from open3d.geometry import PointCloud  # type: ignore[import-untyped]
import pytest

from dimos.core import LCMTransport
from dimos.mapping.occupancy.visualizations import visualize_occupancy_grid
from dimos.mapping.pointclouds.occupancy import (
    height_cost_occupancy,
    simple_occupancy,
)
from dimos.mapping.pointclouds.util import read_pointcloud
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.data import get_data
from dimos.utils.testing.moment import OutputMoment
from dimos.utils.testing.test_moment import Go2Moment


@pytest.fixture
def apartment() -> PointCloud:
    return read_pointcloud(get_data("apartment") / "sum.ply")


@pytest.fixture
def big_office() -> PointCloud:
    return read_pointcloud(get_data("big_office.ply"))


@pytest.mark.parametrize(
    "occupancy_fn,output_name",
    [
        (simple_occupancy, "occupancy_simple.png"),
    ],
)
def test_occupancy(apartment: PointCloud, occupancy_fn, output_name: str) -> None:
    expected_image = cv2.imread(str(get_data(output_name)), cv2.IMREAD_GRAYSCALE)
    cloud = PointCloud2.from_numpy(np.asarray(apartment.points), frame_id="map")

    occupancy_grid = occupancy_fn(cloud)

    # Convert grid from -1..100 to 0..101 for PNG
    computed_image = (occupancy_grid.grid + 1).astype(np.uint8)

    np.testing.assert_array_equal(computed_image, expected_image)


@pytest.mark.parametrize(
    "occupancy_fn,output_name",
    [
        (height_cost_occupancy, "big_office_height_cost_occupancy.png"),
        (simple_occupancy, "big_office_simple_occupancy.png"),
    ],
)
def test_occupancy2(big_office, occupancy_fn, output_name):
    expected_image = Image.from_file(get_data(output_name))
    cloud = PointCloud2.from_numpy(np.asarray(big_office.points), frame_id="")

    occupancy_grid = occupancy_fn(cloud)

    actual = visualize_occupancy_grid(occupancy_grid, "rainbow")
    actual.ts = expected_image.ts
    np.testing.assert_array_equal(actual, expected_image)


class HeightCostMoment(Go2Moment):
    costmap: OutputMoment[OccupancyGrid] = OutputMoment(LCMTransport("/costmap", OccupancyGrid))


@pytest.fixture
def height_cost_moment():
    moment = HeightCostMoment()

    def get_moment(ts: float, publish: bool = True) -> HeightCostMoment:
        moment.seek(ts)
        if moment.lidar.value is not None:
            costmap = height_cost_occupancy(
                moment.lidar.value,
                resolution=0.05,
                can_pass_under=0.6,
                can_climb=0.15,
            )
            moment.costmap.set(costmap)
        if publish:
            moment.publish()
        return moment

    yield get_moment

    moment.stop()


def test_height_cost_occupancy_from_lidar(height_cost_moment) -> None:
    """Test height_cost_occupancy with real lidar data."""
    moment = height_cost_moment(1.0)

    costmap = moment.costmap.value
    assert costmap is not None

    # Basic sanity checks
    assert costmap.grid is not None
    assert costmap.width > 0
    assert costmap.height > 0

    # Costs should be in range -1 to 100 (-1 = unknown)
    assert costmap.grid.min() >= -1
    assert costmap.grid.max() <= 100

    # Check we have some unknown, some known
    known_mask = costmap.grid >= 0
    assert known_mask.sum() > 0, "Expected some known cells"
    assert (~known_mask).sum() > 0, "Expected some unknown cells"
