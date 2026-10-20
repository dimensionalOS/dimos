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

from copy import deepcopy
import math

from dimos_cdrgen.registry import decode, encode
import numpy as np
import pytest

from dimos.mapping.occupancy.path_mask import make_path_mask
from dimos.mapping.occupancy.visualizations import generate_rgba_texture, visualize_occupancy_grid
from dimos.mapping.occupancy.visualize_path import visualize_path
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos.messages.nav_msgs.msg import Path
from dimos.messages.sensor_msgs.msg import Image
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped as LegacyStamped
from dimos.msgs.image import image_view
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid as LegacyGrid
from dimos.msgs.nav_msgs.Path import Path as LegacyPath
from dimos.msgs.occupancy import grid_to_world, occupancy_from_array
from dimos.msgs.time import time_from_nanoseconds


def make_values(stamp, rotated=False):
    cells = np.zeros((9, 9), dtype=np.int8)
    cells[0, 0] = -1
    cells[7:9, 7:9] = 100
    cells[5, 5] = 50
    grid = occupancy_from_array(
        cells,
        header=Header(frame_id="map", stamp=time_from_nanoseconds(stamp)),
        resolution=0.25,
        origin=Pose(
            position=Point(x=0, y=0, z=0),
            orientation=Quaternion(
                x=0,
                y=0,
                z=math.sin(math.pi / 4) if rotated else 0,
                w=math.cos(math.pi / 4) if rotated else 1,
            ),
        ),
    )
    path = Path(
        header=deepcopy(grid.header),
        poses=[
            PoseStamped(
                header=deepcopy(grid.header),
                pose=Pose(
                    position=grid_to_world(grid, coordinate),
                    orientation=Quaternion(x=0, y=0, z=0, w=1),
                ),
            )
            for coordinate in [(1, 1), (4, 1), (4, 4)]
        ],
    )
    old_grid = LegacyGrid(cells.copy(), resolution=0.25, frame_id="map", ts=0)
    old_path = LegacyPath(
        frame_id="map",
        poses=[
            LegacyStamped(
                frame_id="map", position=[x * 0.25, y * 0.25, 0], orientation=[0, 0, 0, 1]
            )
            for x, y in [(1, 1), (4, 1), (4, 4)]
        ],
    )
    return grid, path, old_grid, old_path


@pytest.mark.parametrize("palette", ["rainbow", "turbo"])
@pytest.mark.parametrize("stamp", [0, -1, 1700000000123456789])
def test_generated_map_pixels_match_existing_palette_and_path_overlay(palette, stamp):
    grid, path, old_grid, old_path = make_values(stamp)
    before = encode(grid)

    result = visualize_occupancy_grid(grid, palette, path)
    expected = visualize_occupancy_grid(old_grid, palette, old_path)

    assert type(result) is Image and result.encoding == "bgr8"
    assert result.header == grid.header
    np.testing.assert_array_equal(image_view(result), expected.data)
    decoded = decode(encode(result), Image)
    np.testing.assert_array_equal(image_view(decoded), expected.data)
    result.header.frame_id = "changed"
    result.data[0] = 255
    assert encode(grid) == before


@pytest.mark.parametrize("stamp", [0, -1, 1700000000123456789])
def test_generated_robot_footprints_match_existing_pixels_and_preserve_header(stamp):
    grid, path, old_grid, old_path = make_values(stamp)
    result = visualize_path(grid, path, 0.2, 0.4, scale=4)
    expected = visualize_path(old_grid, old_path, 0.2, 0.4, scale=4)
    assert result.header == grid.header and result.encoding == "bgr8"
    assert image_view(result).shape == (36, 36, 3)
    np.testing.assert_array_equal(image_view(result), expected.data)


@pytest.mark.parametrize(
    "pose_index,max_length", [(0, float("inf")), (0, 0.8), (1, float("inf")), (2, 0)]
)
@pytest.mark.parametrize("rotated", [False, True])
def test_generated_path_mask_keeps_existing_clipping_and_rotated_origin_coordinates(
    pose_index, max_length, rotated
):
    grid, path, old_grid, old_path = make_values(0, rotated)
    result = make_path_mask(grid, path, 0.2, pose_index, max_length)
    expected = make_path_mask(old_grid, old_path, 0.2, pose_index, max_length)
    np.testing.assert_array_equal(result, expected)


@pytest.mark.parametrize("colormap", [None, "viridis"])
@pytest.mark.parametrize("opacity", [0.0, 0.5, 1.0])
@pytest.mark.parametrize("cost_range", [None, (0, 50)])
def test_generated_rgba_texture_matches_existing_colormap_opacity_and_range(
    colormap, opacity, cost_range
):
    grid, _, old_grid, _ = make_values(-1)
    result = generate_rgba_texture(grid, colormap, opacity, cost_range, background="#102030")
    expected = generate_rgba_texture(old_grid, colormap, opacity, cost_range, background="#102030")
    np.testing.assert_array_equal(result, expected)
    np.testing.assert_array_equal(result[0, 0], [0, 0, 0, 0])


def test_generated_default_rgba_values_follow_existing_foxglove_palette():
    grid = occupancy_from_array(
        np.array([[-1, 0, 100]], dtype=np.int8),
        resolution=1,
        header=Header(frame_id="map", stamp=time_from_nanoseconds(0)),
    )
    np.testing.assert_array_equal(
        generate_rgba_texture(grid), [[[0, 0, 0, 0], [72, 73, 129, 255], [0, 0, 0, 255]]]
    )


@pytest.mark.parametrize("occupied_x", [1, 20])
def test_generated_mask_removes_small_occupied_edge_fraction(occupied_x):
    cells = np.zeros((5, 41), dtype=np.int8)
    cells[2, occupied_x] = 100
    grid = occupancy_from_array(
        cells, resolution=1, header=Header(frame_id="map", stamp=time_from_nanoseconds(0))
    )
    path = Path(
        header=deepcopy(grid.header),
        poses=[
            PoseStamped(
                header=deepcopy(grid.header),
                pose=Pose(
                    position=Point(x=x, y=2, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1)
                ),
            )
            for x in [0, 40]
        ],
    )
    expected = np.zeros((5, 41), dtype=bool)
    expected[2, :] = True
    expected[2, occupied_x] = False
    np.testing.assert_array_equal(make_path_mask(grid, path, 1), expected)


def test_generated_mask_rejects_unsafe_occupied_fraction():
    grid, path, _, _ = make_values(0)
    cells = np.array(grid.data, copy=True).reshape(9, 9)
    cells[1, 2] = 100
    grid.data = cells.reshape(-1)
    with pytest.raises(ValueError, match="More than 5%"):
        make_path_mask(grid, path, 0.2)


def test_generated_turbo_handles_unknown_int8_cells_with_exact_existing_colors():
    grid = occupancy_from_array(
        np.array([[-1, 0, 100]], dtype=np.int8),
        resolution=1,
        header=Header(frame_id="map", stamp=time_from_nanoseconds(0)),
    )
    image = visualize_occupancy_grid(grid, "turbo")
    np.testing.assert_array_equal(image_view(image), [[[28, 24, 34], [27, 23, 35], [0, 0, 0]]])
