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
import open3d as o3d
from dimos_lcm.nav_msgs import OccupancyGrid as LCMOccupancyGrid

from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid, CostValues
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.utils.testing import get_data


class TestOccupancyGrid:
    """Test suite for OccupancyGrid message."""

    def test_empty_grid(self):
        """Test creating an empty grid."""
        grid = OccupancyGrid()
        assert grid.width == 0
        assert grid.height == 0
        assert grid.grid.shape == (0,)
        assert grid.total_cells == 0
        assert grid.header.frame_id == "world"

    def test_grid_with_dimensions(self):
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

    def test_grid_from_numpy_array(self):
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

    def test_init_from_lcm(self):
        """Test initialization from LCM OccupancyGrid."""
        # Create LCM grid
        lcm_grid = LCMOccupancyGrid()
        lcm_grid.data_length = 4
        lcm_grid.data = [0, 100, -1, 0]
        lcm_grid.info.width = 2
        lcm_grid.info.height = 2
        lcm_grid.info.resolution = 0.1

        grid = OccupancyGrid(lcm_grid)

        assert grid.width == 2
        assert grid.height == 2
        assert grid.resolution == 0.1
        assert list(grid.data) == [0, 100, -1, 0]

    def test_grid_property(self):
        """Test grid property returns 2D numpy array."""
        test_array = np.array([[0, 100], [-1, 0]], dtype=np.int8)

        grid = OccupancyGrid(test_array)
        grid_array = grid.grid

        assert isinstance(grid_array, np.ndarray)
        assert grid_array.shape == (2, 2)
        assert np.array_equal(grid_array, test_array)

    def test_grid_setter(self):
        """Test setting grid from numpy array."""
        grid = OccupancyGrid(10, 10)

        new_data = np.full((10, 10), CostValues.OCCUPIED, dtype=np.int8)
        grid.grid = new_data

        assert all(val == CostValues.OCCUPIED for val in grid.data)
        assert grid.data_length == 100

    def test_world_grid_coordinate_conversion(self):
        """Test converting between world and grid coordinates."""
        data = np.zeros((20, 30), dtype=np.int8)
        origin = Pose(1.0, 2.0, 0.0)
        grid = OccupancyGrid(data, 0.05, origin, "odom")

        # Test world to grid
        grid_x, grid_y = grid.world_to_grid(2.5, 3.0)
        assert grid_x == 30
        assert grid_y == 20

        # Test grid to world
        world_point = grid.grid_to_world(10, 5)
        assert world_point.x == 1.5
        assert world_point.y == 2.25

    def test_get_set_values(self):
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

    def test_lcm_encode_decode(self):
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
        assert (
            abs(grid.resolution - decoded.resolution) < 1e-6
        )  # Use approximate equality for floats
        assert abs(grid.origin.position.x - decoded.origin.position.x) < 1e-6
        assert abs(grid.origin.position.y - decoded.origin.position.y) < 1e-6
        # Note: header.frame_id might not be preserved through LCM encoding

        # Check that the actual grid data was preserved (don't rely on float conversions)
        assert decoded.grid[5, 10] == 50  # Value we set should be preserved in grid

    def test_string_representation(self):
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

    def test_inflate_binary(self):
        """Test binary obstacle inflation."""
        # Create small grid with one obstacle
        test_array = np.array(
            [
                [0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0],
                [0, 0, 100, 0, 0],
                [0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0],
            ],
            dtype=np.int8,
        )

        grid = OccupancyGrid(test_array, resolution=0.1)

        # Inflate by 0.1m (1 cell)
        inflated = grid.inflate(radius=0.1)

        # Check inflation pattern
        expected = np.array(
            [
                [0, 0, 0, 0, 0],
                [0, 100, 100, 100, 0],
                [0, 100, 100, 100, 0],
                [0, 100, 100, 100, 0],
                [0, 0, 0, 0, 0],
            ],
            dtype=np.int8,
        )

        assert np.array_equal(inflated.grid, expected)

    def test_inflate_with_decay(self):
        """Test inflation with cost decay."""
        # Create small grid with one obstacle
        test_array = np.array(
            [
                [0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0],
                [0, 0, 100, 0, 0],
                [0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0],
            ],
            dtype=np.int8,
        )

        grid = OccupancyGrid(test_array, resolution=0.1)

        # Inflate with decay
        inflated = grid.inflate(radius=0.2, cost_scaling_factor=0.5)

        # Center should still be 100
        assert inflated.grid[2, 2] == 100

        # Adjacent cells should have decayed values
        assert 0 < inflated.grid[2, 1] < 100
        assert 0 < inflated.grid[1, 2] < 100

        # Corners should have lower values due to greater distance
        assert inflated.grid[1, 1] < inflated.grid[1, 2]

    def test_from_pointcloud_empty(self):
        """Test creating grid from empty point cloud."""
        pc = o3d.geometry.PointCloud()
        pointcloud = PointCloud2(pointcloud=pc)

        grid = OccupancyGrid.from_pointcloud(pointcloud)

        assert grid.width == 1
        assert grid.height == 1

    def test_init_from_lcm(self):
        """Test initialization from LCM OccupancyGrid."""
        # Create LCM grid
        lcm_grid = LCMOccupancyGrid()
        lcm_grid.data_length = 4
        lcm_grid.data = [0, 100, -1, 0]
        lcm_grid.info.width = 2
        lcm_grid.info.height = 2
        lcm_grid.info.resolution = 0.1

        grid = OccupancyGrid(lcm_grid)

        assert grid.width == 2
        assert grid.height == 2
        assert grid.resolution == 0.1
        assert list(grid.data) == [0, 100, -1, 0]

    def test_is_occupied(self):
        """Test occupancy checking."""
        grid = OccupancyGrid(10, 10, resolution=0.1)

        # Set some occupied cells
        occupied_point = Vector3(0.5, 0.5, 0.0)
        free_point = Vector3(0.3, 0.3, 0.0)

        grid.set_value(occupied_point, CostValues.OCCUPIED)
        grid.set_value(free_point, CostValues.FREE)

        assert grid.is_occupied(occupied_point)
        assert not grid.is_occupied(free_point)

        # Out of bounds should be considered occupied
        assert grid.is_occupied(Vector3(100.0, 100.0, 0.0))

    def test_from_pointcloud_with_inflation(self):
        """Test creating inflated grid from point cloud."""
        # Single point obstacle
        points = np.array([[0.0, 0.0, 0.5]])

        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)
        pointcloud = PointCloud2(pointcloud=pc)

        # Create grid with inflation
        grid = OccupancyGrid.from_pointcloud(
            pointcloud, resolution=0.1, min_height=0.1, max_height=2.0, inflate_radius=0.2
        )

        # Check inflation
        # The grid origin will be negative due to margin
        # So we check relative to where the point actually is
        assert grid.is_occupied(Vector3(0.0, 0.0, 0.0))  # Original point
        assert grid.is_occupied(Vector3(0.1, 0.0, 0.0))  # Inflated
        assert grid.is_occupied(Vector3(0.0, 0.1, 0.0))  # Inflated

        # Check grid properties
        assert grid.resolution == 0.1
        # Origin should be negative due to margin
        assert grid.origin.position.x < 0
        assert grid.origin.position.y < 0

    def test_cell_statistics(self):
        """Test cell counting properties."""
        test_array = np.array(
            [
                [CostValues.FREE, CostValues.OCCUPIED, CostValues.UNKNOWN],
                [CostValues.FREE, CostValues.OCCUPIED, CostValues.UNKNOWN],
                [CostValues.FREE, CostValues.FREE, CostValues.OCCUPIED],
            ],
            dtype=np.int8,
        )

        grid = OccupancyGrid(test_array)

        assert grid.total_cells == 9
        assert grid.occupied_cells == 3
        assert grid.free_cells == 4
        assert grid.unknown_cells == 2

    def test_grid_property_sync(self):
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

    def test_invalid_grid_dimensions(self):
        """Test handling of invalid grid dimensions."""
        # Test with non-2D array
        with pytest.raises(ValueError, match="Grid must be a 2D array"):
            OccupancyGrid(np.zeros(10), 0.1)

        # Test setting non-2D grid
        grid = OccupancyGrid(5, 5, 0.1)
        with pytest.raises(ValueError, match="Grid must be a 2D array"):
            grid.grid = np.zeros(25)

    def test_from_pointcloud(self):
        """Test creating OccupancyGrid from PointCloud2."""
        # Create a simple test point cloud instead of loading from LFS
        points = np.array(
            [
                [0.0, 0.0, 0.5],
                [1.0, 0.0, 0.5],
                [0.0, 1.0, 0.5],
                [1.0, 1.0, 0.5],
            ]
        )

        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)
        pointcloud = PointCloud2(pointcloud=pc, frame_id="test_frame")

        # Convert pointcloud to occupancy grid
        occupancygrid = OccupancyGrid.from_pointcloud(
            pointcloud, resolution=0.05, min_height=0.1, max_height=2.0, inflate_radius=0.1
        )

        # Check that grid was created with reasonable properties
        assert occupancygrid.width > 0
        assert occupancygrid.height > 0
        assert occupancygrid.resolution == 0.05
        assert occupancygrid.header.frame_id == "test_frame"
        assert occupancygrid.occupied_cells > 0  # Should have some occupied cells

    @pytest.mark.lcm
    def test_lcm_broadcast(self):
        """Test broadcasting OccupancyGrid over LCM."""
        # Create a simple test case
        grid = OccupancyGrid(10, 10, 0.1, "test_frame")
        grid.set_value(0.5, 0.5, CostValues.OCCUPIED)

        lcm = LCM()
        lcm.start()
        lcm.publish(Topic("/global_costmap", OccupancyGrid), grid)

    def test_3d_array_error(self):
        """Test error for 3D array input."""
        test_array = np.zeros((5, 5, 5), dtype=np.int8)

        with pytest.raises(ValueError, match="2D array"):
            OccupancyGrid(test_array)

    def test_from_pointcloud_with_obstacles(self):
        """Test creating grid from point cloud with obstacles."""
        # Create test point cloud
        points = np.array(
            [
                [0.0, 0.0, 0.5],  # Within height range
                [1.0, 0.0, 0.5],
                [0.0, 1.0, 0.5],
                [1.0, 1.0, 0.5],
                [0.5, 0.5, 0.01],  # Below min_height
                [0.5, 0.5, 3.0],  # Above max_height
            ]
        )

        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)
        pointcloud = PointCloud2(pointcloud=pc)

        # Create grid with specific parameters
        grid = OccupancyGrid.from_pointcloud(
            pointcloud, resolution=0.5, min_height=0.1, max_height=2.0
        )

        # Check that correct points are marked as occupied
        # Points at (0,0), (1,0), (0,1), (1,1) at height 0.5 should be occupied
        assert grid.is_occupied(Vector3(0.0, 0.0, 0.0))
        assert grid.is_occupied(Vector3(1.0, 0.0, 0.0))
        assert grid.is_occupied(Vector3(0.0, 1.0, 0.0))
        assert grid.is_occupied(Vector3(1.0, 1.0, 0.0))

        # Points outside height range should not affect the grid
        # Check that cells have expected counts based on 4 valid points

    @pytest.mark.parametrize(
        "width,height,resolution",
        [
            (10, 10, 0.05),
            (100, 50, 0.1),
            (1, 1, 1.0),
            (1000, 1000, 0.01),
        ],
    )
    def test_various_dimensions(self, width, height, resolution):
        """Test with various grid dimensions."""
        grid = OccupancyGrid(width, height, resolution)

        assert grid.width == width
        assert grid.height == height
        assert grid.resolution == resolution
        assert grid.data_length == width * height
        assert len(grid.data) == width * height

    def test_cost_values_constants(self):
        """Test CostValues constants match ROS standards."""
        assert CostValues.FREE == 0
        assert CostValues.UNKNOWN == -1
        assert CostValues.OCCUPIED == 100
