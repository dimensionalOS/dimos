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

import math

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, Quaternion
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
from PIL import Image
import pytest

from dimos.mapping.occupancy.gradient import gradient
from dimos.msgs.occupancy import occupancy_extent, occupancy_from_file, occupancy_view


def test_occupancy_cells_keep_ros_row_order_and_do_not_alias_message():
    message = OccupancyGrid(
        info=MapMetaData(
            width=3,
            height=2,
            resolution=0.1,
            map_load_time=Time(sec=0, nanosec=0),
            origin=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=np.array([-1, 0, 100, 25, 50, 75], dtype=np.int8),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    decoded = cdr_decode(cdr_encode(message), OccupancyGrid)
    values = occupancy_view(decoded)
    np.testing.assert_array_equal(values, [[-1, 0, 100], [25, 50, 75]])
    with pytest.raises(ValueError, match="read-only"):
        values[0, 0] = 0
    copied = values.copy()
    copied[0, 0] = 0
    assert decoded.data[0] == -1


def test_occupancy_rejects_dimensions_that_do_not_match_payload():
    message = OccupancyGrid(
        info=MapMetaData(
            width=2,
            height=2,
            resolution=1.0,
            map_load_time=Time(sec=0, nanosec=0),
            origin=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=np.array([0], dtype=np.int8),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    with pytest.raises(ValueError, match="data length"):
        occupancy_view(message)


@pytest.mark.parametrize("resolution", [0.0, -0.1, math.nan, math.inf])
def test_occupancy_rejects_invalid_resolution_for_nonempty_grid(resolution):
    message = OccupancyGrid(
        info=MapMetaData(
            width=1,
            height=1,
            resolution=resolution,
            map_load_time=Time(sec=0, nanosec=0),
            origin=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=np.array([0], dtype=np.int8),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    with pytest.raises(ValueError, match="resolution"):
        occupancy_extent(message)


def test_empty_default_occupancy_grid_has_no_cells():
    assert occupancy_view(
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
    ).shape == (0, 0)


@pytest.mark.parametrize("suffix", [".npy", ".png"])
def test_static_occupancy_loader_preserves_cells_header_and_identity(tmp_path, suffix):
    cells = np.array([[-1, 0, 100], [25, 50, 75]], dtype=np.int8)
    path = tmp_path / ("grid" + suffix)
    if suffix == ".npy":
        np.save(path, cells)
    else:
        Image.fromarray(cells.astype(np.uint8)).save(path)
    header = Header(frame_id="map", stamp=Time(sec=123, nanosec=456))
    output = cdr_decode(
        cdr_encode(occupancy_from_file(path, header=header, resolution=0.2)),
        OccupancyGrid,
    )
    np.testing.assert_array_equal(occupancy_view(output), cells)
    assert output.header == header
    assert output.info.map_load_time == header.stamp
    assert output.info.origin.orientation.w == 1
    assert output.info.resolution == float(np.float32(0.2))


@pytest.mark.parametrize(
    "cells",
    [np.zeros((2, 2), dtype=float), np.zeros(3, dtype=int), np.array([[101]]), np.array([[-2]])],
)
def test_static_occupancy_loader_rejects_invalid_cells(tmp_path, cells):
    path = tmp_path / "grid.npy"
    np.save(path, cells)
    with pytest.raises(ValueError):
        occupancy_from_file(path, header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""))


def test_gradient() -> None:
    """Test converting occupancy grid to gradient field."""
    # Create a small test grid with an obstacle in the middle
    data = np.zeros((10, 10), dtype=np.int8)
    data[4:6, 4:6] = 100  # 2x2 obstacle in center

    grid = OccupancyGrid(
        header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
        info=MapMetaData(
            width=10,
            height=10,
            resolution=0.1,
            map_load_time=Time(sec=0, nanosec=0),
            origin=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=np.asarray(data.ravel(), dtype=np.int8),
    )  # 0.1m per cell

    # Convert to gradient
    gradient_grid = gradient(grid, obstacle_threshold=50, max_distance=1.0)

    # Check that we get an OccupancyGrid back
    assert isinstance(gradient_grid, OccupancyGrid)
    assert occupancy_view(gradient_grid).shape == (10, 10)
    assert gradient_grid.info.resolution == grid.info.resolution
    assert gradient_grid.header == grid.header

    # Obstacle cells should have value 100
    assert occupancy_view(gradient_grid)[4, 4] == 100
    assert occupancy_view(gradient_grid)[5, 5] == 100

    # Adjacent cells should have high values (near obstacles)
    assert occupancy_view(gradient_grid)[3, 4] > 85  # Very close to obstacle
    assert occupancy_view(gradient_grid)[4, 3] > 85  # Very close to obstacle

    # Cells at moderate distance should have moderate values
    assert 30 < occupancy_view(gradient_grid)[0, 0] < 60  # Corner is ~0.57m away

    # Check that gradient decreases with distance
    assert (
        occupancy_view(gradient_grid)[3, 4] > occupancy_view(gradient_grid)[2, 4]
    )  # Closer is higher
    assert (
        occupancy_view(gradient_grid)[2, 4] > occupancy_view(gradient_grid)[0, 4]
    )  # Further is lower

    # Test with unknown cells
    data_with_unknown = data.copy()
    data_with_unknown[0:2, 0:2] = -1  # Add unknown area (close to obstacle)
    data_with_unknown[8:10, 8:10] = -1  # Add unknown area (far from obstacle)

    grid_with_unknown = OccupancyGrid(
        header=grid.header,
        info=grid.info,
        data=np.asarray(data_with_unknown.ravel(), dtype=np.int8),
    )
    gradient_with_unknown = gradient(grid_with_unknown, max_distance=1.0)  # 1m max distance

    # Unknown cells should remain unknown (new behavior - unknowns are preserved)
    assert occupancy_view(gradient_with_unknown)[0, 0] == -1  # Should remain unknown
    assert occupancy_view(gradient_with_unknown)[1, 1] == -1  # Should remain unknown
    assert occupancy_view(gradient_with_unknown)[8, 8] == -1  # Should remain unknown
    assert occupancy_view(gradient_with_unknown)[9, 9] == -1  # Should remain unknown

    # Unknown cells count should be preserved
    assert (
        np.count_nonzero(occupancy_view(gradient_with_unknown) == -1) == 8
    )  # All unknowns preserved
