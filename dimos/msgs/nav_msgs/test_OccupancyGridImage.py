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

"""Test the OccupancyGridImage class."""

import pickle

import numpy as np
import pytest

from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues
from dimos.msgs.nav_msgs.OccupancyGridImage import OccupancyGridImage
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.utils.testing import get_data


def test_grid_pixel_roundtrip_and_pixel_to_world():
    # create a simple grid 10x20
    width = 10
    height = 20
    grid = np.zeros((height, width), dtype=np.int8)

    og = OccupancyGrid(grid=grid, resolution=0.5, origin=Pose(0, 0, 0))

    size = (100, 200)
    gx, gy = 5, 10

    og_image = OccupancyGridImage.from_occupancygrid(
        occupancy_grid=og, size=size, flip_vertical=True
    )

    px, py = og_image.grid_to_pixel(gx, gy, size=size, flip_vertical=True)
    assert isinstance(px, int) and isinstance(py, int)

    # pixel -> grid should invert when flip_vertical is True and sizes match
    rgx, rgy = og_image.pixel_to_grid(px, py, size=size, flip_vertical=True)
    assert (rgx, rgy) == (gx, gy)

    # check pixel_to_world gives expected world coords (grid->world uses resolution)
    world = og_image.pixel_to_world(px, py, size=size, flip_vertical=True)
    assert pytest.approx(world.x, rel=1e-6) == gx * og.resolution
    assert pytest.approx(world.y, rel=1e-6) == gy * og.resolution


def test_is_free_space_and_get_closest_free_point():
    # 5x5 grid filled as occupied
    width = 5
    height = 5
    grid = np.full((height, width), CostValues.OCCUPIED, dtype=np.int8)

    # one free cell at (4,4)
    free_x, free_y = 4, 4
    grid[free_y, free_x] = CostValues.FREE

    og = OccupancyGrid(grid=grid, resolution=0.1, origin=Pose(0, 0, 0))

    size = (100, 100)

    og_image = OccupancyGridImage.from_occupancygrid(
        occupancy_grid=og, size=size, flip_vertical=False
    )

    # choose a pixel corresponding to an occupied cell (2,2)
    occ_px, occ_py = og_image.grid_to_pixel(2, 2, size=size, flip_vertical=False)
    assert og_image.is_free_space(occ_px, occ_py, size=size, flip_vertical=False) is False

    # closest free point should be the free cell we set
    closest = og_image.get_closest_free_point(
        occ_px, occ_py, size=size, flip_vertical=False, max_search_radius=10
    )
    assert closest is not None
    expected = og_image.grid_to_pixel(free_x, free_y, size=size, flip_vertical=False)
    assert closest == expected

    # with tiny search radius it should return None
    none_closest = og_image.get_closest_free_point(
        occ_px, occ_py, size=size, flip_vertical=False, max_search_radius=1
    )
    assert none_closest is None


def test_grid_to_image_color_mapping():
    # [ unknown, free ]
    # [ occupied, free ]
    grid = np.zeros((2, 2), dtype=np.int8)
    grid[0, 0] = CostValues.UNKNOWN
    grid[0, 1] = CostValues.FREE
    grid[1, 0] = CostValues.OCCUPIED
    grid[1, 1] = CostValues.FREE

    og = OccupancyGrid(grid=grid, resolution=0.05, origin=Pose(0, 0, 0))

    og_image = OccupancyGridImage.from_occupancygrid(
        occupancy_grid=og, size=(2, 2), flip_vertical=False
    )

    arr = og_image.image.data
    # unknown -> yellow [255,255,0]
    assert (arr[0, 0] == np.array([255, 255, 0], dtype=np.uint8)).all()

    # free -> blue-ish [0,0,200]
    assert (arr[0, 1] == np.array([0, 0, 200], dtype=np.uint8)).all()

    # occupied (100) -> red shade [100,0,0]
    assert (arr[1, 0] == np.array([100, 0, 0], dtype=np.uint8)).all()

    # another free
    assert (arr[1, 1] == np.array([0, 0, 200], dtype=np.uint8)).all()
