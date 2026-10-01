#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

from pathlib import Path

import cv2
from dimos_generated.nav_msgs.msg import OccupancyGrid
from dimos_generated.std_msgs.msg import Header
import numpy as np
from open3d.geometry import PointCloud
import pytest

from dimos.core.transport import LCMTransport
from dimos.e2e_tests.cdr_replay_fixture import write_go2_cdr_replay
from dimos.mapping.occupancy.visualizations import visualize_occupancy_grid
from dimos.mapping.pointclouds.occupancy import (
    height_cost_occupancy,
    simple_occupancy,
)
from dimos.mapping.pointclouds.util import read_pointcloud
from dimos.msgs.image import image_from_file, image_view
from dimos.msgs.occupancy import occupancy_view
from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.utils.data import get_data
from dimos.utils.testing.moment import OutputMoment
from dimos.utils.testing.test_moment import Go2Moment

pytestmark = pytest.mark.self_hosted


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
    cloud = pointcloud_from_xyz(np.asarray(apartment.points), header=Header(frame_id="map"))

    occupancy_grid = occupancy_fn(cloud)

    # Convert grid from -1..100 to 0..101 for PNG
    computed_image = (occupancy_view(occupancy_grid) + 1).astype(np.uint8)

    np.testing.assert_array_equal(computed_image, expected_image)


@pytest.mark.parametrize(
    "occupancy_fn,output_name",
    [
        (height_cost_occupancy, "big_office_height_cost_occupancy.png"),
        (simple_occupancy, "big_office_simple_occupancy.png"),
    ],
)
def test_occupancy2(big_office, occupancy_fn, output_name):
    expected_image = image_from_file(get_data(output_name))
    cloud = pointcloud_from_xyz(np.asarray(big_office.points), header=Header())

    occupancy_grid = occupancy_fn(cloud)

    actual = visualize_occupancy_grid(occupancy_grid, "rainbow")
    np.testing.assert_array_equal(image_view(actual), image_view(expected_image))


class HeightCostMoment(Go2Moment):
    costmap: OutputMoment[OccupancyGrid] = OutputMoment(LCMTransport("/costmap", OccupancyGrid))


@pytest.fixture
def height_cost_moment(tmp_path: Path):
    recording = tmp_path / "go2-cdr.db"
    write_go2_cdr_replay(recording, duration_s=3)
    moment = HeightCostMoment(recording)

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

    try:
        yield get_moment
    finally:
        moment.stop()


def test_height_cost_occupancy_from_lidar(height_cost_moment) -> None:
    """Test height_cost_occupancy with a deterministic CDR lidar recording."""
    moment = height_cost_moment(1.0)

    costmap = moment.costmap.value
    assert costmap is not None

    # Basic sanity checks
    assert occupancy_view(costmap) is not None
    assert costmap.info.width > 0
    assert costmap.info.height > 0

    # Costs should be in range -1 to 100 (-1 = unknown)
    assert occupancy_view(costmap).min() >= -1
    assert occupancy_view(costmap).max() <= 100

    # Check we have some unknown, some known
    known_mask = occupancy_view(costmap) >= 0
    assert known_mask.sum() > 0, "Expected some known cells"
    assert (~known_mask).sum() > 0, "Expected some unknown cells"
