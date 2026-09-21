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

import numpy as np
import pytest

from dimos.experimental.agent_encode.pointcloud.render import raster as views
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def room() -> PointCloud2:
    """A 6 m x 4 m room: floor at z=0, a wall at x=3 spanning y in [-2, 2],
    a wall at y=2, and one box at (1, -1)."""
    rng = np.random.default_rng(0)
    floor = np.column_stack([rng.uniform(-3, 3, 4000), rng.uniform(-2, 2, 4000), np.zeros(4000)])
    east = np.column_stack(
        [np.full(2000, 3.0), rng.uniform(-2, 2, 2000), rng.uniform(0, 2.5, 2000)]
    )
    north = np.column_stack(
        [rng.uniform(-3, 3, 2000), np.full(2000, 2.0), rng.uniform(0, 2.5, 2000)]
    )
    box = np.column_stack(
        [rng.uniform(0.8, 1.2, 500), rng.uniform(-1.2, -0.8, 500), rng.uniform(0, 0.8, 500)]
    )
    pts = np.vstack([floor, east, north, box]).astype(np.float32)
    return PointCloud2.from_numpy(pts, frame_id="map", timestamp=7.0)


def depth(pts: np.ndarray, view: views.View, max_depth: float | None = None) -> np.ndarray:
    return views.depth_image(
        pts, view, fov_deg=90.0, size=(64, 40), max_depth=max_depth, point_size_m=None
    )[0]


def test_depth_image_sees_the_wall_ahead() -> None:
    pts = room().points_f32()
    view = views.View(0.0, 0.0, 1.0, yaw_deg=0.0)
    ahead = depth(pts, view, max_depth=10.0)
    centre = float(ahead[18:23, 30:35].min())  # sparse synthetic walls: nearest in a centre block
    assert centre == pytest.approx(3.0, abs=0.15), "the east wall is 3 m ahead"
    # Facing north the wall is 2 m ahead.
    depth_n = depth(pts, views.View(0.0, 0.0, 1.0, yaw_deg=90.0))
    assert float(depth_n[18:23, 30:35].min()) == pytest.approx(2.0, abs=0.15)
    # Facing west there is no wall within range: the top half sees nothing.
    depth_w = depth(pts, views.View(0.0, 0.0, 1.0, yaw_deg=180.0))
    assert np.isinf(depth_w[0, 32])


def test_depth_ascii_shape_and_digits() -> None:
    pts = room().points_f32()
    ahead = depth(pts, views.View(0.0, 0.0, 1.0), max_depth=10.0)
    text = views.depth_ascii(ahead, max_depth=None, shape=(32, 8))
    lines = text.split("\n")
    assert len(lines) == 8 and all(len(line) == 32 for line in lines)
    assert set(text) <= set("0123456789.\n")
    assert lines[4][16] in "0123456789", "the wall ahead has a depth digit"


def test_occupancy_marks_walls_and_places_the_view() -> None:
    cloud = room()
    pts = cloud.points_f32()
    view = views.View(0.0, 0.0, 1.0, yaw_deg=90.0)
    grid, _ = views.occupancy_grid(
        cloud, pts, spacing=0.25, z_range=(0.1, 2.5), max_cells=128, free_radius=0.0
    )
    wall_col, wall_row = views.cell_of(grid, 2.9, 0.0)
    assert views.grid_north_up(grid)[wall_row, wall_col] == CostValues.OCCUPIED
    open_col, open_row = views.cell_of(grid, -2.0, 0.0)
    assert views.grid_north_up(grid)[open_row, open_col] != CostValues.OCCUPIED
    text, step = views.occupancy_ascii(grid, mark=(view.x, view.y, view.yaw_deg), max_cols=64)
    assert step == 1
    lines = text.split("\n")
    col, row = views.cell_of(grid, view.x, view.y)
    assert lines[row][col] == "^", "facing north"
    assert "#" in text and "." in text


def test_max_cells_coarsens_the_grid() -> None:
    cloud = room()
    grid, _ = views.occupancy_grid(
        cloud, cloud.points_f32(), spacing=0.05, z_range=(0.1, 2.5), max_cells=20, free_radius=0.0
    )
    assert max(grid.width, grid.height) <= 21
    assert grid.resolution > 0.05
