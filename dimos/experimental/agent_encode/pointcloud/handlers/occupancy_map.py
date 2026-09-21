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

from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy import ndimage

from dimos.experimental.agent_encode.pointcloud import constants
from dimos.experimental.agent_encode.pointcloud.fields import Grid
from dimos.experimental.agent_encode.pointcloud.handlers.lib.reference import reference
from dimos.experimental.agent_encode.pointcloud.render import raster as render
from dimos.experimental.agent_encode.pointcloud.render.overlays import draw_overlays, grid_pixel
from dimos.experimental.agent_encode.pointcloud.runtime.context import EncodeContext
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid


@dataclass(frozen=True)
class OccupancyMap:
    """The top-down occupancy map of the cloud, in this build's form (image or text).

    The result gives z_range_m, spacing_m, cell_m, origin_xy (world position of the
    south-west corner), size [columns, rows], occupied_cells, free_cells,
    unseen_cells, and mark_cell [column, row] when a mark is given.
    """

    z_range: tuple[float, float]
    """Absolute z: a return below it marks its cell free, inside it occupied, above it
    is ignored."""
    max_cells: int = 256
    """Most cells on a side. Cells are spacing_m, the gap between neighbouring returns,
    or coarser to fit; a 25 m span keeps 0.1 m cells."""
    zoom: tuple[float, float, float] | None = None
    """x, y, radius of a square to see a place closely; the whole cloud otherwise."""
    mark: tuple[float, float, float] | None = None
    """x, y, yaw_deg of a pose to draw on the map."""
    free_radius: float = 0.0
    """Metres that free spreads. At 0, free cells spread no further than the floor
    returns that made them."""
    colour: str = "flat"
    """Occupied cells are black, or with "height" coloured by their highest return over
    ``z_range``, which adds height_scale to read it back."""
    source: Any = None
    overlays: tuple[Any, ...] = ()
    grid: Grid | None = None
    """An explicit XY grid fixes the origin, resolution, and extent; it cannot be
    combined with ``zoom`` and must fit ``max_cells`` without coarsening."""

    @staticmethod
    def pixels_per_cell(grid: OccupancyGrid) -> int:
        """Cells scale up until the image's long side is about 1024 px, large enough
        to read rooms."""
        return max(1, round(1024 / max(grid.width, grid.height)))

    def _fixed_grid(self, ctx: EncodeContext) -> tuple[OccupancyGrid, np.ndarray | None]:
        spec = self.grid
        assert spec is not None
        spec.describe(ctx)
        if spec.plane != "xy":
            raise ValueError("OccupancyMap requires an XY grid")
        if max(spec.shape) > self.max_cells:
            raise ValueError("supplied grid exceeds max_cells; request a smaller grid")
        if self.zoom is not None:
            raise ValueError("an explicit grid already fixes the extent; omit zoom")
        if not np.isfinite(self.free_radius) or self.free_radius < 0:
            raise ValueError("free_radius must be finite and non-negative")
        if (
            len(self.z_range) != 2
            or not np.isfinite(self.z_range).all()
            or self.z_range[0] > self.z_range[1]
        ):
            raise ValueError("z_range must contain finite, ordered endpoints")
        indices, valid = spec.indices(ctx.points[:, :2])
        points, indices = ctx.points[valid], indices[valid]
        shape = (spec.shape[1], spec.shape[0])
        cells = np.full(shape, CostValues.UNKNOWN, dtype=np.int8)
        below = points[:, 2] < self.z_range[0]
        occupied = (points[:, 2] >= self.z_range[0]) & (points[:, 2] <= self.z_range[1])
        cells[indices[below, 1], indices[below, 0]] = CostValues.FREE
        cells[indices[occupied, 1], indices[occupied, 0]] = CostValues.OCCUPIED
        free = cells == CostValues.FREE
        if self.free_radius > 0 and free.any():
            distance = ndimage.distance_transform_edt(~free, sampling=spec.cell_m)
            cells[(distance <= self.free_radius) & (cells != CostValues.OCCUPIED)] = CostValues.FREE
        heights = None
        if self.colour == "height":
            heights = np.full(shape, -np.inf)
            np.maximum.at(
                heights,
                (indices[occupied, 1], indices[occupied, 0]),
                points[occupied, 2],
            )
            heights[~np.isfinite(heights)] = np.nan
        grid = OccupancyGrid(
            grid=cells,
            resolution=spec.cell_m,
            origin=Pose(spec.origin[0], spec.origin[1], 0.0),
            frame_id=ctx.cloud.frame_id,
            ts=ctx.cloud.ts,
        )
        return grid, heights

    def measure(self, ctx: EncodeContext) -> tuple[OccupancyGrid, np.ndarray | None, np.ndarray]:
        """Build the rendered grid and retain the builder's exact contributor cell indices."""
        if self.colour not in ("flat", "height"):
            raise ValueError(f"colour must be 'flat' or 'height', not {self.colour!r}")
        if type(self.max_cells) is not int or not 1 <= self.max_cells <= 1024:
            raise ValueError("max_cells must be an integer in 1..1024")
        if (
            len(self.z_range) != 2
            or not np.isfinite(self.z_range).all()
            or self.z_range[0] > self.z_range[1]
        ):
            raise ValueError("z_range must contain two finite ordered endpoints")
        if not np.isfinite(self.free_radius) or self.free_radius < 0:
            raise ValueError("free_radius must be finite and nonnegative")
        if self.zoom is not None and (
            len(self.zoom) != 3 or not np.isfinite(self.zoom).all() or self.zoom[2] <= 0
        ):
            raise ValueError("zoom must contain finite x,y and positive radius")
        if self.grid is None:
            provenance: dict[str, np.ndarray] = {}
            grid, heights = render.occupancy_grid(
                ctx.cloud,
                ctx.points,
                z_range=self.z_range,
                spacing=ctx.spacing_m,
                max_cells=self.max_cells,
                free_radius=self.free_radius,
                zoom=self.zoom,
                heights=self.colour == "height",
                _point_cells=provenance,
            )
            indices = provenance["indices"]
        else:
            grid, heights = self._fixed_grid(ctx)
            indices, _ = self.grid.indices(ctx.points[:, :2])
        return grid, heights, indices

    def run(self, ctx: EncodeContext) -> dict[str, Any]:
        original_ctx = ctx
        ctx = ctx.select(self.source)
        grid, heights, _ = self.measure(ctx)
        out: dict[str, Any] = {
            "handler": "OccupancyMap",
            "view_ref": reference(self, original_ctx),
            "z_range_m": list(self.z_range),
            "spacing_m": round(ctx.spacing_m, 4),
            "cell_m": round(grid.resolution, 4),
            "origin_xy": [
                round(float(grid.origin.position.x), 3),
                round(float(grid.origin.position.y), 3),
            ],
            "size": [grid.width, grid.height],
            "zoom_m": list(self.zoom) if self.zoom is not None else None,
            "mark": list(self.mark) if self.mark is not None else None,
            "free_radius_m": self.free_radius,
            "colour": self.colour,
            "occupied_cells": int((grid.grid == CostValues.OCCUPIED).sum()),
            "free_cells": int((grid.grid == CostValues.FREE).sum()),
            "unseen_cells": int((grid.grid == CostValues.UNKNOWN).sum()),
        }
        if self.grid is not None:
            out["grid"] = self.grid.describe(ctx)
            out["cell_m"] = self.grid.cell_m
            out["origin_xy"] = list(self.grid.origin)
        if self.mark is not None:
            out["mark_cell"] = list(render.cell_of(grid, self.mark[0], self.mark[1]))
        if heights is not None:
            out["height_scale"] = render.height_scale(self.z_range)
        if constants.FORM == "image":
            scale = self.pixels_per_cell(grid)
            with ctx.artifact("occupancy.png") as (staging, path):
                render.occupancy_png(
                    grid,
                    staging,
                    mark=self.mark,
                    heights=heights,
                    z_range=self.z_range,
                    scale=scale,
                )
                if self.overlays:

                    def project(point: np.ndarray) -> tuple[float, float]:
                        return grid_pixel(
                            point,
                            (0, 1),
                            (grid.origin.position.x, grid.origin.position.y),
                            grid.resolution,
                            grid.height,
                            scale,
                        )

                    out["overlays"] = draw_overlays(
                        staging, self.overlays, original_ctx, project, view_ref=out["view_ref"]
                    )
            out["image"] = str(path)
        else:
            text, step = render.occupancy_ascii(
                grid, mark=self.mark, heights=heights, z_range=self.z_range
            )
            out["ascii"] = text
            out["ascii_cells_per_char"] = step
            if self.mark is not None:
                col, row = out["mark_cell"]
                out["mark_cell"] = [col // step, row // step]
        return out
