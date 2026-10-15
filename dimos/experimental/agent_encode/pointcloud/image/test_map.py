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

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from PIL import Image as PILImage
import pytest

from dimos.experimental.agent_encode.pointcloud.grid.base import Grid
from dimos.experimental.agent_encode.pointcloud.grid.count import Count
from dimos.experimental.agent_encode.pointcloud.grid.occupancy import Occupancy
from dimos.experimental.agent_encode.pointcloud.grid.z_min import ZMin
from dimos.experimental.agent_encode.pointcloud.image.lib.canvas import Line
from dimos.experimental.agent_encode.pointcloud.image.map import MapImage, grid_image
from dimos.experimental.agent_encode.pointcloud.queries.select import Select
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def value_at(image: MapImage, uv: tuple[int, int]) -> float | None:
    xy = image.world(uv)
    assert xy is not None
    return image.grid.at(xy)


def test_occupancy_marks_walls(room: PointCloud2, tmp_path: Path) -> None:
    image = Occupancy((0.1, 2.5), 0.25).run(room).image(out_dir=tmp_path)
    assert image.grid.at((3.0, 0.0)) == 1
    assert image.grid.at((-2.0, 0.0)) != 1


def test_images_refuse_more_cells_than_pixels(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="at most 1024 a side"):
        Grid((0, 0), 0.1, np.zeros((1, 1025))).image(out_dir=tmp_path)


def test_height_cell_bounds_north_up_edges_and_selected_source(tmp_path: Path) -> None:
    cloud = PointCloud2.from_numpy(
        np.array(
            [[-0.75, -0.75, 2], [-0.75, -0.75, 4], [-0.5, -0.75, 6], [-0.75, -0.75, 9]],
            dtype=np.float32,
        ),
        frame_id="custom",
    )
    lowest = ZMin(0.5, area=((-1, -1), (-0.5, -0.5))).run(Select(z=(None, 8)).run(cloud))
    image = grid_image(lowest, out_dir=tmp_path)
    scale = image.pixels_per_cell
    assert image.grid.shape == (2, 2)
    assert image.size == (2 * scale, 2 * scale)
    south_west = (0, scale)
    assert image.world(south_west) == pytest.approx(
        (-1 + 0.5 / scale * 0.5, -1 + (2 - (scale + 0.5) / scale) * 0.5)
    )
    assert sorted(image.pick(uv=south_west).points_f32()[:, 2].tolist()) == [2, 4]
    assert value_at(image, south_west) == 2
    edge = (scale, scale)
    assert image.pick(uv=edge).points_f32()[:, 2].tolist() == [6]
    assert value_at(image, edge) == 6
    assert len(image.pick(uv=(0, 0))) == 0
    assert value_at(image, (0, 0)) is None


def test_derived_grids_read_values_but_do_not_claim_returns(tmp_path: Path) -> None:
    cloud = PointCloud2.from_numpy(np.array([[5.5, 5.5, 1]], dtype=np.float32))
    occupied = Count(1.0, area=((0, 0), (5.5, 5.5))).run(cloud) > 0
    for grid, expected in ((occupied.distance(), 5 * math.sqrt(2)), (~occupied, 1)):
        image = grid_image(grid, out_dir=tmp_path)
        south_west = (0, image.size[1] - 1)
        with pytest.raises(ValueError, match="computed from other grids"):
            image.pick(uv=south_west)
        assert value_at(image, south_west) == pytest.approx(expected)


def test_masks_draw_white_black_and_grey(tmp_path: Path) -> None:
    cloud = PointCloud2.from_numpy(
        np.array([[0.25, 0.25, 0], [1.25, 0.25, 2], [1.25, 0.25, 9]], dtype=np.float32)
    )
    image = Occupancy((1, 3), 1, ((0, 0), (2.5, 1.5))).run(cloud).image(out_dir=tmp_path)
    scale = 1024 // 3
    assert image.grid.shape == (3, 2)
    assert image.pixels_per_cell == scale
    assert image.scale is None
    occupied = (scale, scale)
    assert value_at(image, occupied) == 1
    assert image.pick(uv=occupied).points_f32().tolist() == [[1.25, 0.25, 2]]
    free = (0, scale)
    assert value_at(image, free) == 0
    unseen = (0, 0)
    assert len(image.pick(uv=unseen)) == 0
    assert value_at(image, unseen) is None
    with PILImage.open(image.path) as picture:
        assert picture.size == image.size
        assert picture.getpixel((scale // 2, scale + scale // 2)) == (255, 255, 255)
        assert picture.getpixel((scale // 2, scale // 2)) == (190, 190, 190)


def test_drawn_items_are_listed_with_their_colours(tmp_path: Path) -> None:
    cloud = PointCloud2.from_numpy(np.array([[1.5, 0.5, 0]], dtype=np.float32))
    heights = ZMin(1.0, area=((1, 0), (2.5, 1.5))).run(cloud)
    image = grid_image(heights, draw=(Line(((1, 0), (2, 1))),), out_dir=tmp_path)
    assert [(d.label, d.colour) for d in image.drawn] == [("Line", "#ff3bcc")]
