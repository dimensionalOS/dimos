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

from dimos_cdrgen.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
from PIL import Image
import pytest

from dimos.message_utils.geometry import quaternion_from_euler
from dimos.message_utils.occupancy import (
    grid_to_world,
    occupancy_extent,
    occupancy_from_array,
    occupancy_from_file,
    occupancy_view,
    world_to_grid,
)
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Point, Pose, Quaternion
from dimos.messages.nav_msgs.msg import MapMetaData, OccupancyGrid
from dimos.messages.std_msgs.msg import Header


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


@pytest.mark.parametrize("ns", [0, -1, 1700000000123456789])
def test_array_constructor_copies_data_and_metadata_without_rounding_source_time(ns):
    seconds, nanoseconds = divmod(ns, 1_000_000_000)
    cells = np.array([[-1, 0], [100, 50]], dtype=np.int8)
    header = Header(frame_id="map", stamp=Time(sec=seconds, nanosec=nanoseconds))
    origin = Pose(
        position=Point(x=10, y=20, z=3), orientation=quaternion_from_euler(0, 0, math.pi / 2)
    )

    message = occupancy_from_array(cells, header=header, resolution=0.5, origin=origin)
    header.stamp.sec = 77
    cells[0, 0] = 44
    origin.position.x = 99
    actual = cdr_decode(cdr_encode(message), OccupancyGrid)

    assert actual.header.stamp.sec == seconds
    assert actual.header.stamp.nanosec == nanoseconds
    assert actual.info.map_load_time == actual.header.stamp
    assert actual.info.origin.position.x == 10
    np.testing.assert_array_equal(occupancy_view(actual), [[-1, 0], [100, 50]])


def test_coordinate_helpers_account_for_rotated_origin_and_grid_plane():
    origin = Pose(
        position=Point(x=10, y=20, z=3), orientation=quaternion_from_euler(0, 0, math.pi / 2)
    )
    message = occupancy_from_array(
        np.zeros((4, 4), dtype=np.int8),
        header=Header(frame_id="map", stamp=Time(sec=0, nanosec=0)),
        resolution=0.5,
        origin=origin,
    )

    point = grid_to_world(message, (2, 3))

    assert (point.x, point.y, point.z) == pytest.approx((8.5, 21, 3))
    assert world_to_grid(message, point) == pytest.approx((2, 3))
    assert occupancy_extent(message) == (2, 2)


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_coordinate_helpers_reject_nonfinite_coordinates(value):
    message = occupancy_from_array(
        np.zeros((1, 1), dtype=np.int8),
        header=Header(frame_id="map", stamp=Time(sec=0, nanosec=0)),
        resolution=1,
    )
    with pytest.raises(ValueError, match="finite"):
        grid_to_world(message, (value, 0))
    with pytest.raises(ValueError, match="finite"):
        world_to_grid(message, Point(x=value, y=0, z=0))


def test_numpy_loader_never_unpickles_objects(tmp_path):
    path = tmp_path / "objects.npy"
    np.save(path, np.array([[{}]], dtype=object), allow_pickle=True)
    with pytest.raises(ValueError, match="allow_pickle=False"):
        occupancy_from_file(path, header=Header(frame_id="map", stamp=Time(sec=0, nanosec=0)))
