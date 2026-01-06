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

from dimos.mapping.occupancy.gradient import gradient
from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.mapping.pointclouds.occupancy import general_occupancy
from dimos.msgs.geometry_msgs import Pose
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.utils.data import get_data


def test_empty_grid() -> None:
    """Test creating an empty grid."""
    grid = OccupancyGrid()
    assert grid.width == 0
    assert grid.height == 0
    assert grid.grid.shape == (0,)
    assert grid.total_cells == 0
    assert grid.frame_id == "world"


def test_grid_with_dimensions() -> None:
    """Test creating a grid with specified dimensions."""
    grid = OccupancyGrid(width=10, height=10, resolution=0.1, frame_id="map")
    assert grid.width == 10
    assert grid.height == 10
    assert grid.resolution == 0.1
    assert grid.frame_id == "map"
    assert grid.grid.shape == (10, 10)
    assert np.all(grid.grid == -1)  # All unknown
    assert grid.unknown_cells == 100
    assert grid.unknown_percent == 100.0


def test_grid_from_numpy_array() -> None:
    """Test creating a grid from a numpy array."""
    data = np.zeros((20, 30), dtype=np.int8)
    data[5:10, 10:20] = 100  # Add some obstacles
    data[15:18, 5:8] = -1  # Add unknown area

    origin = Pose(1.0, 2.0, 0.0)
    grid = OccupancyGrid(grid=data, resolution=0.05, origin=origin, frame_id="odom")

    assert grid.width == 30
    assert grid.height == 20
    assert grid.resolution == 0.05
    assert grid.frame_id == "odom"
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


def test_world_grid_coordinate_conversion() -> None:
    """Test converting between world and grid coordinates."""
    data = np.zeros((20, 30), dtype=np.int8)
    origin = Pose(1.0, 2.0, 0.0)
    grid = OccupancyGrid(grid=data, resolution=0.05, origin=origin, frame_id="odom")

    # Test world to grid
    grid_pos = grid.world_to_grid((2.5, 3.0))
    assert int(grid_pos.x) == 30
    assert int(grid_pos.y) == 20

    # Test grid to world
    world_pos = grid.grid_to_world((10, 5))
    assert world_pos.x == 1.5
    assert world_pos.y == 2.25


def test_lcm_encode_decode() -> None:
    """Test LCM encoding and decoding."""
    data = np.zeros((20, 30), dtype=np.int8)
    data[5:10, 10:20] = 100  # Add some obstacles
    data[15:18, 5:8] = -1  # Add unknown area
    origin = Pose(1.0, 2.0, 0.0)
    grid = OccupancyGrid(grid=data, resolution=0.05, origin=origin, frame_id="odom")

    # Set a specific value for testing
    # Convert world coordinates to grid indices
    grid_pos = grid.world_to_grid((1.5, 2.25))
    grid.grid[int(grid_pos.y), int(grid_pos.x)] = 50

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
    assert grid.frame_id == decoded.frame_id

    # Check that the actual grid data was preserved (don't rely on float conversions)
    assert decoded.grid[5, 10] == 50  # Value we set should be preserved in grid


def test_string_representation() -> None:
    """Test string representations."""
    grid = OccupancyGrid(width=10, height=10, resolution=0.1, frame_id="map")

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


def test_grid_property_sync() -> None:
    """Test that the grid property works correctly."""
    grid = OccupancyGrid(width=5, height=5, resolution=0.1, frame_id="map")

    # Modify via numpy array
    grid.grid[2, 3] = 100
    assert grid.grid[2, 3] == 100

    # Check that we can access grid values
    grid.grid[0, 0] = 50
    assert grid.grid[0, 0] == 50


def test_invalid_grid_dimensions() -> None:
    """Test handling of invalid grid dimensions."""
    # Test with non-2D array
    with pytest.raises(ValueError, match="Grid must be a 2D array"):
        OccupancyGrid(grid=np.zeros(10), resolution=0.1)


def test_from_pointcloud() -> None:
    """Test creating OccupancyGrid from PointCloud2."""
    file_path = get_data("lcm_msgs") / "sensor_msgs/PointCloud2.pickle"
    with open(file_path, "rb") as f:
        lcm_msg = pickle.loads(f.read())

    pointcloud = PointCloud2.lcm_decode(lcm_msg)

    # Convert pointcloud to occupancy grid
    occupancygrid = general_occupancy(pointcloud, resolution=0.05, min_height=0.1, max_height=2.0)
    # Apply inflation separately if needed
    occupancygrid = simple_inflate(occupancygrid, 0.1)

    # Check that grid was created with reasonable properties
    assert occupancygrid.width > 0
    assert occupancygrid.height > 0
    assert occupancygrid.resolution == 0.05
    assert occupancygrid.frame_id == pointcloud.frame_id
    assert occupancygrid.occupied_cells > 0  # Should have some occupied cells


def test_gradient() -> None:
    """Test converting occupancy grid to gradient field."""
    # Create a small test grid with an obstacle in the middle
    data = np.zeros((10, 10), dtype=np.int8)
    data[4:6, 4:6] = 100  # 2x2 obstacle in center

    grid = OccupancyGrid(grid=data, resolution=0.1)  # 0.1m per cell

    # Convert to gradient
    gradient_grid = gradient(grid, obstacle_threshold=50, max_distance=1.0)

    # Check that we get an OccupancyGrid back
    assert isinstance(gradient_grid, OccupancyGrid)
    assert gradient_grid.grid.shape == (10, 10)
    assert gradient_grid.resolution == grid.resolution
    assert gradient_grid.frame_id == grid.frame_id

    # Obstacle cells should have value 100
    assert gradient_grid.grid[4, 4] == 100
    assert gradient_grid.grid[5, 5] == 100

    # Adjacent cells should have high values (near obstacles)
    assert gradient_grid.grid[3, 4] > 85  # Very close to obstacle
    assert gradient_grid.grid[4, 3] > 85  # Very close to obstacle

    # Cells at moderate distance should have moderate values
    assert 30 < gradient_grid.grid[0, 0] < 60  # Corner is ~0.57m away

    # Check that gradient decreases with distance
    assert gradient_grid.grid[3, 4] > gradient_grid.grid[2, 4]  # Closer is higher
    assert gradient_grid.grid[2, 4] > gradient_grid.grid[0, 4]  # Further is lower

    # Test with unknown cells
    data_with_unknown = data.copy()
    data_with_unknown[0:2, 0:2] = -1  # Add unknown area (close to obstacle)
    data_with_unknown[8:10, 8:10] = -1  # Add unknown area (far from obstacle)

    grid_with_unknown = OccupancyGrid(data_with_unknown, resolution=0.1)
    gradient_with_unknown = gradient(grid_with_unknown, max_distance=1.0)  # 1m max distance

    # Unknown cells should remain unknown (new behavior - unknowns are preserved)
    assert gradient_with_unknown.grid[0, 0] == -1  # Should remain unknown
    assert gradient_with_unknown.grid[1, 1] == -1  # Should remain unknown
    assert gradient_with_unknown.grid[8, 8] == -1  # Should remain unknown
    assert gradient_with_unknown.grid[9, 9] == -1  # Should remain unknown

    # Unknown cells count should be preserved
    assert gradient_with_unknown.unknown_cells == 8  # All unknowns preserved


def test_filter_above() -> None:
    """Test filtering cells above threshold."""
    # Create test grid with various values
    data = np.array(
        [[-1, 0, 20, 50], [10, 30, 60, 80], [40, 70, 90, 100], [-1, 15, 25, -1]], dtype=np.int8
    )

    grid = OccupancyGrid(grid=data, resolution=0.1)

    # Filter to keep only values > 50
    filtered = grid.filter_above(50)

    # Check that values > 50 are preserved
    assert filtered.grid[1, 2] == 60
    assert filtered.grid[1, 3] == 80
    assert filtered.grid[2, 1] == 70
    assert filtered.grid[2, 2] == 90
    assert filtered.grid[2, 3] == 100

    # Check that values <= 50 are set to -1 (unknown)
    assert filtered.grid[0, 1] == -1  # was 0
    assert filtered.grid[0, 2] == -1  # was 20
    assert filtered.grid[0, 3] == -1  # was 50
    assert filtered.grid[1, 0] == -1  # was 10
    assert filtered.grid[1, 1] == -1  # was 30
    assert filtered.grid[2, 0] == -1  # was 40

    # Check that unknown cells are preserved
    assert filtered.grid[0, 0] == -1
    assert filtered.grid[3, 0] == -1
    assert filtered.grid[3, 3] == -1

    # Check dimensions and metadata preserved
    assert filtered.width == grid.width
    assert filtered.height == grid.height
    assert filtered.resolution == grid.resolution
    assert filtered.frame_id == grid.frame_id


def test_filter_below() -> None:
    """Test filtering cells below threshold."""
    # Create test grid with various values
    data = np.array(
        [[-1, 0, 20, 50], [10, 30, 60, 80], [40, 70, 90, 100], [-1, 15, 25, -1]], dtype=np.int8
    )

    grid = OccupancyGrid(grid=data, resolution=0.1)

    # Filter to keep only values < 50
    filtered = grid.filter_below(50)

    # Check that values < 50 are preserved
    assert filtered.grid[0, 1] == 0
    assert filtered.grid[0, 2] == 20
    assert filtered.grid[1, 0] == 10
    assert filtered.grid[1, 1] == 30
    assert filtered.grid[2, 0] == 40
    assert filtered.grid[3, 1] == 15
    assert filtered.grid[3, 2] == 25

    # Check that values >= 50 are set to -1 (unknown)
    assert filtered.grid[0, 3] == -1  # was 50
    assert filtered.grid[1, 2] == -1  # was 60
    assert filtered.grid[1, 3] == -1  # was 80
    assert filtered.grid[2, 1] == -1  # was 70
    assert filtered.grid[2, 2] == -1  # was 90
    assert filtered.grid[2, 3] == -1  # was 100

    # Check that unknown cells are preserved
    assert filtered.grid[0, 0] == -1
    assert filtered.grid[3, 0] == -1
    assert filtered.grid[3, 3] == -1

    # Check dimensions and metadata preserved
    assert filtered.width == grid.width
    assert filtered.height == grid.height
    assert filtered.resolution == grid.resolution
    assert filtered.frame_id == grid.frame_id


def test_max() -> None:
    """Test setting all non-unknown cells to maximum."""
    # Create test grid with various values
    data = np.array(
        [[-1, 0, 20, 50], [10, 30, 60, 80], [40, 70, 90, 100], [-1, 15, 25, -1]], dtype=np.int8
    )

    grid = OccupancyGrid(grid=data, resolution=0.1)

    # Apply max
    maxed = grid.max()

    # Check that all non-unknown cells are set to 100
    assert maxed.grid[0, 1] == 100  # was 0
    assert maxed.grid[0, 2] == 100  # was 20
    assert maxed.grid[0, 3] == 100  # was 50
    assert maxed.grid[1, 0] == 100  # was 10
    assert maxed.grid[1, 1] == 100  # was 30
    assert maxed.grid[1, 2] == 100  # was 60
    assert maxed.grid[1, 3] == 100  # was 80
    assert maxed.grid[2, 0] == 100  # was 40
    assert maxed.grid[2, 1] == 100  # was 70
    assert maxed.grid[2, 2] == 100  # was 90
    assert maxed.grid[2, 3] == 100  # was 100 (already max)
    assert maxed.grid[3, 1] == 100  # was 15
    assert maxed.grid[3, 2] == 100  # was 25

    # Check that unknown cells are preserved
    assert maxed.grid[0, 0] == -1
    assert maxed.grid[3, 0] == -1
    assert maxed.grid[3, 3] == -1

    # Check dimensions and metadata preserved
    assert maxed.width == grid.width
    assert maxed.height == grid.height
    assert maxed.resolution == grid.resolution
    assert maxed.frame_id == grid.frame_id

    # Verify statistics
    assert maxed.unknown_cells == 3  # Same as original
    assert maxed.occupied_cells == 13  # All non-unknown cells
    assert maxed.free_cells == 0  # No free cells


@pytest.mark.lcm
def test_lcm_broadcast() -> None:
    """Test broadcasting OccupancyGrid and gradient over LCM."""
    file_path = get_data("lcm_msgs") / "sensor_msgs/PointCloud2.pickle"
    with open(file_path, "rb") as f:
        lcm_msg = pickle.loads(f.read())

    pointcloud = PointCloud2.lcm_decode(lcm_msg)

    # Create occupancy grid from pointcloud
    occupancygrid = general_occupancy(pointcloud, resolution=0.05, min_height=0.1, max_height=2.0)
    # Apply inflation separately if needed
    occupancygrid = simple_inflate(occupancygrid, 0.1)

    # Create gradient field with larger max_distance for better visualization
    gradient_grid = gradient(occupancygrid, obstacle_threshold=70, max_distance=2.0)

    # Debug: Print actual values to see the difference
    print("\n=== DEBUG: Comparing grids ===")
    print(f"Original grid unique values: {np.unique(occupancygrid.grid)}")
    print(f"Gradient grid unique values: {np.unique(gradient_grid.grid)}")

    # Find an area with occupied cells to show the difference
    occupied_indices = np.argwhere(occupancygrid.grid == 100)
    if len(occupied_indices) > 0:
        # Pick a point near an occupied cell
        idx = len(occupied_indices) // 2  # Middle occupied cell
        sample_y, sample_x = occupied_indices[idx]
        sample_size = 15

        # Ensure we don't go out of bounds
        y_start = max(0, sample_y - sample_size // 2)
        y_end = min(occupancygrid.height, y_start + sample_size)
        x_start = max(0, sample_x - sample_size // 2)
        x_end = min(occupancygrid.width, x_start + sample_size)

        print(f"\nSample area around occupied cell ({sample_x}, {sample_y}):")
        print("Original occupancy grid:")
        print(occupancygrid.grid[y_start:y_end, x_start:x_end])
        print("\nGradient grid (same area):")
        print(gradient_grid.grid[y_start:y_end, x_start:x_end])
    else:
        print("\nNo occupied cells found for sampling")

    # Check statistics
    print("\nOriginal grid stats:")
    print(f"  Occupied (100): {np.sum(occupancygrid.grid == 100)} cells")
    print(f"  Inflated (99): {np.sum(occupancygrid.grid == 99)} cells")
    print(f"  Free (0): {np.sum(occupancygrid.grid == 0)} cells")
    print(f"  Unknown (-1): {np.sum(occupancygrid.grid == -1)} cells")

    print("\nGradient grid stats:")
    print(f"  Max gradient (100): {np.sum(gradient_grid.grid == 100)} cells")
    print(
        f"  High gradient (80-99): {np.sum((gradient_grid.grid >= 80) & (gradient_grid.grid < 100))} cells"
    )
    print(
        f"  Medium gradient (40-79): {np.sum((gradient_grid.grid >= 40) & (gradient_grid.grid < 80))} cells"
    )
    print(
        f"  Low gradient (1-39): {np.sum((gradient_grid.grid >= 1) & (gradient_grid.grid < 40))} cells"
    )
    print(f"  Zero gradient (0): {np.sum(gradient_grid.grid == 0)} cells")
    print(f"  Unknown (-1): {np.sum(gradient_grid.grid == -1)} cells")

    # # Save debug images
    # import matplotlib.pyplot as plt

    # fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # # Original
    # ax = axes[0]
    # im1 = ax.imshow(occupancygrid.grid, origin="lower", cmap="gray_r", vmin=-1, vmax=100)
    # ax.set_title(f"Original Occupancy Grid\n{occupancygrid}")
    # plt.colorbar(im1, ax=ax)

    # # Gradient
    # ax = axes[1]
    # im2 = ax.imshow(gradient_grid.grid, origin="lower", cmap="hot", vmin=-1, vmax=100)
    # ax.set_title(f"Gradient Grid\n{gradient_grid}")
    # plt.colorbar(im2, ax=ax)

    # plt.tight_layout()
    # plt.savefig("lcm_debug_grids.png", dpi=150)
    # print("\nSaved debug visualization to lcm_debug_grids.png")
    # plt.close()

    # Broadcast all the data
    lcm = LCM()
    lcm.start()
    lcm.publish(Topic("/global_map", PointCloud2), pointcloud)
    lcm.publish(Topic("/global_costmap", OccupancyGrid), occupancygrid)
    lcm.publish(Topic("/global_gradient", OccupancyGrid), gradient_grid)

    print("\nPublished to LCM:")
    print(f"  /global_map: PointCloud2 with {len(pointcloud)} points")
    print(f"  /global_costmap: {occupancygrid}")
    print(f"  /global_gradient: {gradient_grid}")
    print("\nGradient info:")
    print("  Values: 0 (free far from obstacles) -> 100 (at obstacles)")
    print(f"  Unknown cells: {gradient_grid.unknown_cells} (preserved as -1)")
    print("  Max distance for gradient: 5.0 meters")
