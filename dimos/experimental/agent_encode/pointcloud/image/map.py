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

"""Top-down pictures of grids, north up."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray
from PIL import Image as PILImage, ImageDraw

from dimos.experimental.agent_encode.pointcloud.grid.lib.mask import is_mask
from dimos.experimental.agent_encode.pointcloud.image.base import Drawn, Image, draw_items
from dimos.experimental.agent_encode.pointcloud.image.lib.canvas import Drawable
from dimos.experimental.agent_encode.pointcloud.image.lib.colour import (
    colour_scale,
    colour_table,
)
from dimos.experimental.agent_encode.pointcloud.image.lib.files import (
    artifact,
    digest_stem,
    output_dir,
)
from dimos.experimental.agent_encode.pointcloud.queries.lib.points import as_cloud, finite_points

if TYPE_CHECKING:
    from dimos.experimental.agent_encode.pointcloud.grid.base import Grid
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass(frozen=True, eq=False, repr=False)
class MapImage(Image):
    """A grid drawn top-down, north up: each cell is a square of ``pixels_per_cell``."""

    grid: Grid
    """The cells the image shows; ``grid.cloud`` holds the returns that drew them."""
    pixels_per_cell: int

    def pixel(self, point: tuple[float, ...]) -> tuple[float, float]:
        """Where world point (x, y[, z]) lands; z is ignored."""
        u, v = grid_pixels(
            np.array([point[:2]], dtype=np.float64), self.grid, self.pixels_per_cell
        )[0]
        return round(float(u), 2), round(float(v), 2)

    def world(self, uv: tuple[int, int]) -> tuple[float, float] | None:
        """World (x, y) at the centre of pixel ``uv``; None outside the image."""
        u, v = uv
        if not (0 <= u < self.size[0] and 0 <= v < self.size[1]):
            return None
        cell, scale = self.grid.cell_m, self.pixels_per_cell
        rows = self.grid.shape[1]
        x = self.grid.origin[0] + (u + 0.5) / scale * cell
        y = self.grid.origin[1] + (rows - (v + 0.5) / scale) * cell
        return round(x, 6), round(y, 6)

    def returns_under(self, pixels: NDArray[np.int64]) -> PointCloud2:
        """The returns in the cells under ``pixels``."""
        if self.grid.cloud is None:
            raise ValueError(
                "this grid was computed from other grids, so no returns drew it; "
                "read it with img.grid.at(img.world(uv))"
            )
        cols, rows = self.grid.shape
        picked = (rows - 1 - pixels[:, 1] // self.pixels_per_cell) * cols + (
            pixels[:, 0] // self.pixels_per_cell
        )
        points = finite_points(self.grid.cloud)
        ij = np.floor((points[:, :2].astype(np.float64) - self.grid.origin) / self.grid.cell_m)
        ij = ij.astype(np.int64)
        inside = ((ij >= 0) & (ij < (cols, rows))).all(axis=1)
        keep = inside & np.isin(ij[:, 1] * cols + ij[:, 0], picked)
        return as_cloud(points[keep], self.grid.cloud)


def grid_pixels(
    points: NDArray[np.float64], grid: Grid, pixels_per_cell: int
) -> NDArray[np.float64]:
    """(K, 2) continuous (u, v) of (K, 2+) world points on a north-up picture of ``grid``."""
    cells = (points[:, :2] - np.asarray(grid.origin)) / grid.cell_m
    return np.column_stack(
        (cells[:, 0] * pixels_per_cell, (grid.shape[1] - cells[:, 1]) * pixels_per_cell)
    )


def draw_on_grid(
    picture: PILImage.Image,
    grid: Grid,
    pixels_per_cell: int,
    items: tuple[Drawable | PointCloud2, ...],
    z_extent: tuple[float, float],
) -> tuple[Drawn, ...]:
    """Draw ``items`` over a north-up picture of ``grid``, at pixel centres."""

    def project(points: NDArray[np.float64]) -> NDArray[np.float64]:
        return grid_pixels(points, grid, pixels_per_cell) - 0.5

    return draw_items(picture, items, project, z_extent)


def z_extent_of(cloud: PointCloud2 | None) -> tuple[float, float]:
    """The lowest and highest z of the cloud's finite returns; (0, 0) without any."""
    z = finite_points(cloud)[:, 2] if cloud is not None else np.empty(0, dtype=np.float32)
    return (float(z.min()), float(z.max())) if len(z) else (0.0, 0.0)


def grid_image(
    grid: Grid,
    value_range: tuple[float, float] | None = None,
    draw: tuple[Drawable | PointCloud2, ...] = (),
    out_dir: Path | None = None,
) -> MapImage:
    """``grid`` as a picture, each cell a square of pixels so the long side is about
    1024 px, with grey lines on every whole metre. A mask is white at 0, black at 1 and
    light grey without data; any other grid is coloured over ``value_range`` (the finite
    values' span by default), dark grey without data."""
    cols, rows = grid.shape
    if max(cols, rows) > 1024:
        raise ValueError(
            f"the grid is {cols} x {rows} cells; an image shows at most 1024 a side, "
            "so build it over a smaller area= or with a larger cell_m"
        )
    scale = 1024 // max(cols, rows)
    values = grid.values
    finite = np.isfinite(values)
    if value_range is None and is_mask(values):
        rgb = np.full((*values.shape, 3), 190, dtype=np.uint8)
        rgb[values == 0] = 255
        rgb[values == 1] = 0
        scale_read = None
    else:
        low, high = (
            value_range
            if value_range is not None
            else (
                (float(values[finite].min()), float(values[finite].max()))
                if finite.any()
                else (0.0, 1.0)
            )
        )
        if not np.isfinite([low, high]).all() or high < low:
            raise ValueError("value_range must be two finite values, lower first")
        denominator = high - low if high != low else 1.0
        fractions = np.where(finite, np.clip((values - low) / denominator, 0, 1), 0)
        _, table = colour_table()
        rgb = table[(fractions * 255).astype(int)].astype(np.uint8)
        rgb[~finite] = (96, 96, 96)
        scale_read = colour_scale(low, high)
    picture = PILImage.fromarray(np.ascontiguousarray(rgb[::-1])).resize(
        (cols * scale, rows * scale), PILImage.Resampling.NEAREST
    )
    _metre_lines(picture, grid, scale)
    drawn = draw_on_grid(picture, grid, scale, draw, z_extent_of(grid.cloud))
    stem = digest_stem(
        repr((grid.origin, grid.cell_m, value_range, draw)).encode(),
        np.ascontiguousarray(values).tobytes(),
    )
    path = output_dir(out_dir) / f"{stem}_grid.png"
    with artifact(path) as staging:
        picture.save(staging)
    return MapImage(path, picture.size, scale_read, drawn, grid, scale)


def _metre_lines(picture: PILImage.Image, grid: Grid, pixels_per_cell: int) -> None:
    """Grey lines on every whole metre of x and y, when they are at least 8 px apart."""
    if pixels_per_cell / grid.cell_m < 8:
        return
    cols, rows = grid.shape
    x0, y0 = grid.origin
    pen = ImageDraw.Draw(picture)
    for x in range(math.ceil(x0), math.floor(x0 + cols * grid.cell_m) + 1):
        u = (x - x0) / grid.cell_m * pixels_per_cell
        pen.line([(u, 0), (u, picture.height)], fill=(150, 150, 150))
    for y in range(math.ceil(y0), math.floor(y0 + rows * grid.cell_m) + 1):
        v = (rows - (y - y0) / grid.cell_m) * pixels_per_cell
        pen.line([(0, v), (picture.width, v)], fill=(150, 150, 150))
