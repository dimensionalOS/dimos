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

"""Generated occupancy grids with external array and coordinate operations."""

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, Quaternion
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest

from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.mapping.pointclouds.occupancy import general_occupancy
from dimos.msgs.geometry import quaternion_from_euler, yaw
from dimos.msgs.occupancy import (
    block_max_reduce,
    grid_to_world,
    occupancy_extent,
    occupancy_from_array,
    occupancy_view,
    world_to_grid,
)
from dimos.msgs.pointcloud import pointcloud_from_xyz


def test_empty_grid() -> None:
    """Test creating an empty grid."""
    grid = OccupancyGrid(
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        info=MapMetaData(
            map_load_time=Time(sec=0, nanosec=0),
            resolution=0.0,
            width=0,
            height=0,
            origin=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=np.array([], dtype=np.int8),
    )
    assert grid.info.width == 0
    assert grid.info.height == 0
    assert occupancy_view(grid).shape == (0, 0)
    assert len(grid.data) == 0
    assert grid.header.frame_id == ""


def test_grid_with_dimensions() -> None:
    """Test creating a grid with specified dimensions."""
    grid = occupancy_from_array(
        np.full((10, 10), -1, dtype=np.int8),
        resolution=0.1,
        header=Header(frame_id="map", stamp=Time(sec=0, nanosec=0)),
    )
    assert grid.info.width == 10
    assert grid.info.height == 10
    assert grid.info.resolution == pytest.approx(0.1)
    assert grid.header.frame_id == "map"
    assert occupancy_view(grid).shape == (10, 10)
    assert np.all(occupancy_view(grid) == -1)
    assert np.count_nonzero(occupancy_view(grid) == -1) == 100
    assert (np.count_nonzero(occupancy_view(grid) == -1) / len(grid.data) * 100) == 100.0


def test_grid_from_numpy_array() -> None:
    """Test creating a grid from a numpy array."""
    data = np.zeros((20, 30), dtype=np.int8)
    data[5:10, 10:20] = 100
    data[15:18, 5:8] = -1
    origin = Pose(
        position=Point(x=1, y=2, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )
    grid = occupancy_from_array(
        data,
        resolution=0.05,
        origin=origin,
        header=Header(frame_id="odom", stamp=Time(sec=0, nanosec=0)),
    )
    assert grid.info.width == 30
    assert grid.info.height == 20
    assert grid.info.resolution == pytest.approx(0.05)
    assert grid.header.frame_id == "odom"
    assert grid.info.origin.position.x == 1.0
    assert grid.info.origin.position.y == 2.0
    assert occupancy_view(grid).shape == (20, 30)
    assert np.count_nonzero(occupancy_view(grid) == 100) == 50
    assert np.count_nonzero(occupancy_view(grid) == 0) == 541
    assert np.count_nonzero(occupancy_view(grid) == -1) == 9
    assert abs((np.count_nonzero(occupancy_view(grid) == 100) / len(grid.data) * 100) - 8.33) < 0.1
    assert abs((np.count_nonzero(occupancy_view(grid) == 0) / len(grid.data) * 100) - 90.17) < 0.1
    assert abs((np.count_nonzero(occupancy_view(grid) == -1) / len(grid.data) * 100) - 1.5) < 0.1


def test_world_grid_coordinate_conversion() -> None:
    """Test converting between world and grid coordinates."""
    data = np.zeros((20, 30), dtype=np.int8)
    origin = Pose(
        position=Point(x=1, y=2, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )
    grid = occupancy_from_array(
        data,
        resolution=0.05,
        origin=origin,
        header=Header(frame_id="odom", stamp=Time(sec=0, nanosec=0)),
    )
    grid_pos = world_to_grid(grid, Point(x=2.5, y=3.0, z=0.0))
    assert grid_pos[0] == pytest.approx(30)
    assert grid_pos[1] == pytest.approx(20)
    world_pos = grid_to_world(grid, (10, 5))
    assert world_pos.x == pytest.approx(1.5)
    assert world_pos.y == pytest.approx(2.25)


def test_cdr_encode_decode() -> None:
    """Test LCM encoding and decoding."""
    data = np.zeros((20, 30), dtype=np.int8)
    data[5:10, 10:20] = 100
    data[15:18, 5:8] = -1
    origin = Pose(
        position=Point(x=1, y=2, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )
    grid = occupancy_from_array(
        data,
        resolution=0.05,
        origin=origin,
        header=Header(frame_id="odom", stamp=Time(sec=0, nanosec=0)),
    )
    grid_pos = world_to_grid(grid, Point(x=1.5, y=2.25, z=0.0))
    grid.data[round(grid_pos[1]) * grid.info.width + round(grid_pos[0])] = 50
    lcm_data = cdr_encode(grid)
    assert isinstance(lcm_data, bytes)
    assert len(lcm_data) > 0
    decoded = cdr_decode(lcm_data, OccupancyGrid)
    assert np.array_equal(occupancy_view(grid), occupancy_view(decoded))
    assert grid.info.width == decoded.info.width
    assert grid.info.height == decoded.info.height
    assert abs(grid.info.resolution - decoded.info.resolution) < 1e-06
    assert abs(grid.info.origin.position.x - decoded.info.origin.position.x) < 1e-06
    assert abs(grid.info.origin.position.y - decoded.info.origin.position.y) < 1e-06
    assert grid.header.frame_id == decoded.header.frame_id
    assert occupancy_view(decoded)[5, 10] == 50


def test_cdr_decode_origin_is_dimos_pose() -> None:
    """Generated origin orientation works with the external yaw helper."""
    quat = quaternion_from_euler(0, 0, 0.5)
    origin = Pose(position=Point(x=1, y=2, z=0.0), orientation=quat)
    grid = occupancy_from_array(
        np.zeros((2, 3), dtype=np.int8),
        resolution=0.05,
        origin=origin,
        header=Header(frame_id="", stamp=Time(sec=0, nanosec=0)),
    )
    decoded = cdr_decode(cdr_encode(grid), OccupancyGrid)
    assert isinstance(decoded.info.origin, Pose)
    assert yaw(decoded.info.origin.orientation) == pytest.approx(0.5)


def test_cdr_decode_empty_grid() -> None:
    """An empty grid (mapper warming up) must survive the wire; the decoder
    used to rebuild it as a 1-D array the constructor rejects."""
    decoded = cdr_decode(
        cdr_encode(
            OccupancyGrid(
                header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
                info=MapMetaData(
                    map_load_time=Time(sec=0, nanosec=0),
                    resolution=0.0,
                    width=0,
                    height=0,
                    origin=Pose(
                        position=Point(x=0.0, y=0.0, z=0.0),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                ),
                data=np.array([], dtype=np.int8),
            )
        ),
        OccupancyGrid,
    )
    assert occupancy_view(decoded).size == 0
    assert isinstance(decoded.info.origin, Pose)


def test_physical_extent() -> None:
    grid = occupancy_from_array(
        np.full((10, 10), -1, dtype=np.int8),
        header=Header(frame_id="map", stamp=Time(sec=0, nanosec=0)),
        resolution=0.1,
    )
    assert occupancy_extent(grid) == pytest.approx((1, 1))


def test_grid_property_sync() -> None:
    """Test that the grid property works correctly."""
    grid = occupancy_from_array(
        np.full((5, 5), -1, dtype=np.int8),
        resolution=0.1,
        header=Header(frame_id="map", stamp=Time(sec=0, nanosec=0)),
    )
    grid.data[2 * grid.info.width + 3] = 100
    assert occupancy_view(grid)[2, 3] == 100
    grid.data[0] = 50
    assert occupancy_view(grid)[0, 0] == 50


def test_invalid_grid_dimensions() -> None:
    """Test handling of invalid grid dimensions."""
    with pytest.raises(ValueError, match="2D integer array"):
        occupancy_from_array(
            np.zeros(10), resolution=0.1, header=Header(frame_id="", stamp=Time(sec=0, nanosec=0))
        )


def test_from_pointcloud() -> None:
    """Test creating OccupancyGrid from PointCloud2."""
    x, y = np.meshgrid(np.arange(10) * 0.05, np.arange(10) * 0.05)
    points = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, 0.5)))
    pointcloud = pointcloud_from_xyz(
        points, header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0))
    )
    occupancygrid = general_occupancy(pointcloud, resolution=0.05, min_height=0.1, max_height=2.0)
    occupancygrid = simple_inflate(occupancygrid, 0.1)
    assert occupancygrid.info.width > 0
    assert occupancygrid.info.height > 0
    assert occupancygrid.info.resolution == pytest.approx(0.05)
    assert occupancygrid.header.frame_id == pointcloud.header.frame_id
    assert np.count_nonzero(occupancy_view(occupancygrid) == 100) > 0


def test_filter_above() -> None:
    """Test filtering cells above threshold."""
    data = np.array(
        [[-1, 0, 20, 50], [10, 30, 60, 80], [40, 70, 90, 100], [-1, 15, 25, -1]], dtype=np.int8
    )
    grid = occupancy_from_array(
        data, resolution=0.1, header=Header(frame_id="", stamp=Time(sec=0, nanosec=0))
    )
    filtered = occupancy_from_array(
        np.where(occupancy_view(grid) > 50, occupancy_view(grid), -1).astype(np.int8),
        header=grid.header,
        resolution=grid.info.resolution,
        origin=grid.info.origin,
    )
    assert occupancy_view(filtered)[1, 2] == 60
    assert occupancy_view(filtered)[1, 3] == 80
    assert occupancy_view(filtered)[2, 1] == 70
    assert occupancy_view(filtered)[2, 2] == 90
    assert occupancy_view(filtered)[2, 3] == 100
    assert occupancy_view(filtered)[0, 1] == -1
    assert occupancy_view(filtered)[0, 2] == -1
    assert occupancy_view(filtered)[0, 3] == -1
    assert occupancy_view(filtered)[1, 0] == -1
    assert occupancy_view(filtered)[1, 1] == -1
    assert occupancy_view(filtered)[2, 0] == -1
    assert occupancy_view(filtered)[0, 0] == -1
    assert occupancy_view(filtered)[3, 0] == -1
    assert occupancy_view(filtered)[3, 3] == -1
    assert filtered.info.width == grid.info.width
    assert filtered.info.height == grid.info.height
    assert filtered.info.resolution == grid.info.resolution
    assert filtered.header.frame_id == grid.header.frame_id


def test_filter_below() -> None:
    """Test filtering cells below threshold."""
    data = np.array(
        [[-1, 0, 20, 50], [10, 30, 60, 80], [40, 70, 90, 100], [-1, 15, 25, -1]], dtype=np.int8
    )
    grid = occupancy_from_array(
        data, resolution=0.1, header=Header(frame_id="", stamp=Time(sec=0, nanosec=0))
    )
    filtered = occupancy_from_array(
        np.where(occupancy_view(grid) < 50, occupancy_view(grid), -1).astype(np.int8),
        header=grid.header,
        resolution=grid.info.resolution,
        origin=grid.info.origin,
    )
    assert occupancy_view(filtered)[0, 1] == 0
    assert occupancy_view(filtered)[0, 2] == 20
    assert occupancy_view(filtered)[1, 0] == 10
    assert occupancy_view(filtered)[1, 1] == 30
    assert occupancy_view(filtered)[2, 0] == 40
    assert occupancy_view(filtered)[3, 1] == 15
    assert occupancy_view(filtered)[3, 2] == 25
    assert occupancy_view(filtered)[0, 3] == -1
    assert occupancy_view(filtered)[1, 2] == -1
    assert occupancy_view(filtered)[1, 3] == -1
    assert occupancy_view(filtered)[2, 1] == -1
    assert occupancy_view(filtered)[2, 2] == -1
    assert occupancy_view(filtered)[2, 3] == -1
    assert occupancy_view(filtered)[0, 0] == -1
    assert occupancy_view(filtered)[3, 0] == -1
    assert occupancy_view(filtered)[3, 3] == -1
    assert filtered.info.width == grid.info.width
    assert filtered.info.height == grid.info.height
    assert filtered.info.resolution == grid.info.resolution
    assert filtered.header.frame_id == grid.header.frame_id


def test_max() -> None:
    """Test setting all non-unknown cells to maximum."""
    data = np.array(
        [[-1, 0, 20, 50], [10, 30, 60, 80], [40, 70, 90, 100], [-1, 15, 25, -1]], dtype=np.int8
    )
    grid = occupancy_from_array(
        data, resolution=0.1, header=Header(frame_id="", stamp=Time(sec=0, nanosec=0))
    )
    maxed = occupancy_from_array(
        np.where(occupancy_view(grid) >= 0, 100, -1).astype(np.int8),
        header=grid.header,
        resolution=grid.info.resolution,
        origin=grid.info.origin,
    )
    assert occupancy_view(maxed)[0, 1] == 100
    assert occupancy_view(maxed)[0, 2] == 100
    assert occupancy_view(maxed)[0, 3] == 100
    assert occupancy_view(maxed)[1, 0] == 100
    assert occupancy_view(maxed)[1, 1] == 100
    assert occupancy_view(maxed)[1, 2] == 100
    assert occupancy_view(maxed)[1, 3] == 100
    assert occupancy_view(maxed)[2, 0] == 100
    assert occupancy_view(maxed)[2, 1] == 100
    assert occupancy_view(maxed)[2, 2] == 100
    assert occupancy_view(maxed)[2, 3] == 100
    assert occupancy_view(maxed)[3, 1] == 100
    assert occupancy_view(maxed)[3, 2] == 100
    assert occupancy_view(maxed)[0, 0] == -1
    assert occupancy_view(maxed)[3, 0] == -1
    assert occupancy_view(maxed)[3, 3] == -1
    assert maxed.info.width == grid.info.width
    assert maxed.info.height == grid.info.height
    assert maxed.info.resolution == grid.info.resolution
    assert maxed.header.frame_id == grid.header.frame_id
    assert np.count_nonzero(occupancy_view(maxed) == -1) == 3
    assert np.count_nonzero(occupancy_view(maxed) == 100) == 13
    assert np.count_nonzero(occupancy_view(maxed) == 0) == 0


def test_block_max_reduce_preserves_lone_obstacle() -> None:
    cells = np.zeros((10, 10), dtype=np.int8)
    cells[3, 4] = 100
    reduced = block_max_reduce(cells, 5)
    assert reduced.shape == (2, 2)
    assert reduced.dtype == np.int8
    assert reduced[0, 0] == 100
    assert reduced[0, 1] == 0


def test_block_max_reduce_unknown_only_when_whole_block_unknown() -> None:
    cells = np.array([[-1, -1, -1, 50], [-1, -1, 0, -1]], dtype=np.int8)
    reduced = block_max_reduce(cells, 2)
    assert reduced.tolist() == [[-1, 50]]
    assert reduced.dtype == np.int8


def test_block_max_reduce_trims_remainder() -> None:
    cells = np.arange(35, dtype=np.int8).reshape(7, 5)
    reduced = block_max_reduce(cells, 2)
    assert reduced.shape == (3, 2)
    assert reduced[0, 0] == 6
    assert reduced[2, 1] == 28


def test_block_max_reduce_thin_grid_passes_through() -> None:
    cells = np.zeros((1, 10), dtype=np.int8)
    assert block_max_reduce(cells, 5) is cells
