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

"""Test the OccupancyGrid convenience class."""

import pickle

import numpy as np
import pytest

from dimos.msgs.geometry_msgs import Pose
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.utils.testing import get_data


def test_empty_grid():
    """Test creating an empty grid."""
    grid = OccupancyGrid()
    assert grid.width == 0
    assert grid.height == 0
    assert grid.grid.shape == (0,)
    assert grid.total_cells == 0
    assert grid.header.frame_id == "world"


def test_grid_with_dimensions():
    """Test creating a grid with specified dimensions."""
    grid = OccupancyGrid(10, 10, 0.1, "map")
    assert grid.width == 10
    assert grid.height == 10
    assert grid.resolution == 0.1
    assert grid.header.frame_id == "map"
    assert grid.grid.shape == (10, 10)
    assert np.all(grid.grid == -1)  # All unknown
    assert grid.unknown_cells == 100
    assert grid.unknown_percent == 100.0


def test_grid_from_numpy_array():
    """Test creating a grid from a numpy array."""
    data = np.zeros((20, 30), dtype=np.int8)
    data[5:10, 10:20] = 100  # Add some obstacles
    data[15:18, 5:8] = -1  # Add unknown area

    origin = Pose(1.0, 2.0, 0.0)
    grid = OccupancyGrid(data, 0.05, origin, "odom")

    assert grid.width == 30
    assert grid.height == 20
    assert grid.resolution == 0.05
    assert grid.header.frame_id == "odom"
    assert grid.origin.position.x == 1.0
    assert grid.origin.position.y == 2.0
    assert grid.grid.shape == (20, 30)

    # Check cell counts
    assert grid.occupied_cells == 50  # 5x10 obstacle area
    assert grid.free_cells == 541  # Total - occupied - unknown
    assert grid.unknown_cells == 9  # 3x3 unknown area

    # Check percentages (approximately)
    assert abs(grid.occupied_percent - 8.33) < 0.1
    assert abs(grid.free_percent - 90.17) < 0.1
    assert abs(grid.unknown_percent - 1.5) < 0.1


def test_world_grid_coordinate_conversion():
    """Test converting between world and grid coordinates."""
    data = np.zeros((20, 30), dtype=np.int8)
    origin = Pose(1.0, 2.0, 0.0)
    grid = OccupancyGrid(data, 0.05, origin, "odom")

    # Test world to grid
    grid_x, grid_y = grid.world_to_grid(2.5, 3.0)
    assert grid_x == 30
    assert grid_y == 20

    # Test grid to world
    world_x, world_y = grid.grid_to_world(10, 5)
    assert world_x == 1.5
    assert world_y == 2.25


def test_get_set_values():
    """Test getting and setting values at world coordinates."""
    data = np.zeros((20, 30), dtype=np.int8)
    data[5, 10] = 100  # Place an obstacle
    origin = Pose(1.0, 2.0, 0.0)
    grid = OccupancyGrid(data, 0.05, origin, "odom")

    # Test getting a value
    value = grid.get_value(1.5, 2.25)
    assert value == 100

    # Test setting a value
    success = grid.set_value(1.5, 2.25, 50)
    assert success is True
    assert grid.get_value(1.5, 2.25) == 50

    # Test out of bounds
    assert grid.get_value(10.0, 10.0) is None
    assert grid.set_value(10.0, 10.0, 100) is False


def test_lcm_encode_decode():
    """Test LCM encoding and decoding."""
    data = np.zeros((20, 30), dtype=np.int8)
    data[5:10, 10:20] = 100  # Add some obstacles
    data[15:18, 5:8] = -1  # Add unknown area
    origin = Pose(1.0, 2.0, 0.0)
    grid = OccupancyGrid(data, 0.05, origin, "odom")

    # Set a specific value for testing
    grid.set_value(1.5, 2.25, 50)

    # Encode
    lcm_data = grid.lcm_encode()
    assert isinstance(lcm_data, bytes)
    assert len(lcm_data) > 0

    # Decode
    decoded = OccupancyGrid.lcm_decode(lcm_data)

    # Check that data matches exactly (grid arrays should be identical)
    assert np.array_equal(grid.grid, decoded.grid)
    assert grid.width == decoded.width
    assert grid.height == decoded.height
    assert abs(grid.resolution - decoded.resolution) < 1e-6  # Use approximate equality for floats
    assert abs(grid.origin.position.x - decoded.origin.position.x) < 1e-6
    assert abs(grid.origin.position.y - decoded.origin.position.y) < 1e-6
    assert grid.header.frame_id == decoded.header.frame_id

    # Check that the actual grid data was preserved (don't rely on float conversions)
    assert decoded.grid[5, 10] == 50  # Value we set should be preserved in grid


def test_string_representation():
    """Test string representations."""
    grid = OccupancyGrid(10, 10, 0.1, "map")

    # Test __str__
    str_repr = str(grid)
    assert "OccupancyGrid[map]" in str_repr
    assert "10x10" in str_repr
    assert "1.0x1.0m" in str_repr
    assert "10cm res" in str_repr

    # Test __repr__
    repr_str = repr(grid)
    assert "OccupancyGrid(" in repr_str
    assert "width=10" in repr_str
    assert "height=10" in repr_str
    assert "resolution=0.1" in repr_str


def test_grid_property_sync():
    """Test that the grid property properly syncs with the flat data."""
    grid = OccupancyGrid(5, 5, 0.1, "map")

    # Modify via numpy array
    grid.grid[2, 3] = 100
    grid._sync_data_from_array()

    # Check that flat data was updated
    assert grid.data[2 * 5 + 3] == 100

    # Modify via flat data
    grid.data[0] = 50
    grid._sync_array_from_data()

    # Check that numpy array was updated
    assert grid.grid[0, 0] == 50


def test_invalid_grid_dimensions():
    """Test handling of invalid grid dimensions."""
    # Test with non-2D array
    with pytest.raises(ValueError, match="Grid must be a 2D array"):
        OccupancyGrid(np.zeros(10), 0.1)

    # Test setting non-2D grid
    grid = OccupancyGrid(5, 5, 0.1)
    with pytest.raises(ValueError, match="Grid must be a 2D array"):
        grid.grid = np.zeros(25)


def test_from_pointcloud():
    """Test creating OccupancyGrid from PointCloud2."""
    file_path = get_data("lcm_msgs") / "sensor_msgs/PointCloud2.pickle"
    with open(file_path, "rb") as f:
        lcm_msg = pickle.loads(f.read())

    pointcloud = PointCloud2.lcm_decode(lcm_msg)

    # Convert pointcloud to occupancy grid
    occupancygrid = OccupancyGrid.from_pointcloud(
        pointcloud, resolution=0.05, min_height=0.1, max_height=2.0, inflate_radius=0.1
    )

    # Check that grid was created with reasonable properties
    assert occupancygrid.width > 0
    assert occupancygrid.height > 0
    assert occupancygrid.resolution == 0.05
    assert occupancygrid.header.frame_id == pointcloud.frame_id
    assert occupancygrid.occupied_cells > 0  # Should have some occupied cells


@pytest.mark.lcm
def test_lcm_broadcast():
    """Test creating OccupancyGrid from PointCloud2."""
    file_path = get_data("lcm_msgs") / "sensor_msgs/PointCloud2.pickle"
    with open(file_path, "rb") as f:
        lcm_msg = pickle.loads(f.read())

    pointcloud = PointCloud2.lcm_decode(lcm_msg)

    occupancygrid = OccupancyGrid.from_pointcloud(
        pointcloud, resolution=0.05, min_height=0.1, max_height=2.0, inflate_radius=0.1
    )

    lcm = LCM()
    lcm.start()
    lcm.publish(Topic("/global_map", PointCloud2), pointcloud)
    lcm.publish(Topic("/global_costmap", OccupancyGrid), occupancygrid)
