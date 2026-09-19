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

"""Portable visual queries. References contain bounded recipes, never point buffers or paths."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from dimos.experimental.agent_encode.pointcloud import fields as field_nodes
from dimos.experimental.agent_encode.pointcloud.handlers.depth_view import depth_data
from dimos.experimental.agent_encode.pointcloud.handlers.lib.reference import (
    _json,
    _spec,
    reference,
    resolve,
)
from dimos.experimental.agent_encode.pointcloud.render import raster as render
from dimos.experimental.agent_encode.pointcloud.runtime.context import EncodeContext

MAX_PIXELS = 65536
MAX_ITEMS = 64


def _pixels(pick: Pick, width: int, height: int) -> tuple[np.ndarray, bool]:
    if sum(v is not None for v in (pick.uv, pick.rect, pick.polygon)) != 1:
        raise ValueError("choose exactly one of uv, rect, polygon")
    if type(pick.radius_px) is not int or not 0 <= pick.radius_px <= 64:
        raise ValueError("radius_px must be an integer in 0..64")
    if pick.uv is None and pick.radius_px:
        raise ValueError("radius_px applies only to uv")
    polygon = None
    if pick.uv is not None:
        if len(pick.uv) != 2 or any(type(v) is not int for v in pick.uv):
            raise ValueError("uv must contain integer native pixel indices")
        u, v = pick.uv
        if not (0 <= u < width and 0 <= v < height):
            return np.empty((0, 2), dtype=np.int64), True
        r = pick.radius_px
        x0, y0, x1, y1 = max(0, u - r), max(0, v - r), min(width, u + r + 1), min(height, v + r + 1)
    elif pick.rect is not None:
        if len(pick.rect) != 4 or any(type(v) is not int for v in pick.rect):
            raise ValueError("rect must contain integer (u,v,width,height)")
        x0, y0, w, h = pick.rect
        if min(w, h) <= 0:
            raise ValueError("rectangle dimensions must be positive")
        x1, y1 = x0 + w, y0 + h
    else:
        polygon = np.asarray(pick.polygon, dtype=float)
        if (
            polygon.ndim != 2
            or polygon.shape[1] != 2
            or not 3 <= len(polygon) <= 32
            or not np.isfinite(polygon).all()
        ):
            raise ValueError("polygon needs 3..32 finite native-pixel vertices")
        if np.any(polygon < 0) or np.any(polygon > [width, height]):
            raise ValueError("polygon must lie within image edges")
        x0, y0 = np.floor(polygon.min(0)).astype(int)
        x1, y1 = np.ceil(polygon.max(0)).astype(int)
    if x0 < 0 or y0 < 0 or x1 > width or y1 > height:
        raise ValueError("region must lie inside image edges")
    if (x1 - x0) * (y1 - y0) > MAX_PIXELS:
        raise ValueError("pick region exceeds 65536 pixels; use a smaller region")
    yy, xx = np.mgrid[y0:y1, x0:x1]
    pixels = np.column_stack((xx.ravel(), yy.ravel()))
    if polygon is not None:
        # Even-odd rule at pixel centres; boundary ties use this same deterministic rule.
        x, y = pixels[:, 0] + 0.5, pixels[:, 1] + 0.5
        inside = np.zeros(len(pixels), dtype=bool)
        for a, b in zip(polygon, np.roll(polygon, -1, axis=0), strict=True):
            if a[1] != b[1]:
                inside ^= ((a[1] > y) != (b[1] > y)) & (
                    x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]
                )
        pixels = pixels[inside]
    return pixels, False


def _cell_points(source: Any, ctx: EncodeContext) -> np.ndarray | None:
    if isinstance(source, field_nodes.HeightField):
        return ctx.select(source.source).points
    if isinstance(source, field_nodes.Channel):
        return _cell_points(source.source, ctx)
    # Derived values may depend on remote cells/returns. Do not claim local contributors.
    return None


@dataclass(frozen=True)
class Pick:
    """Lazy exact visual measurement; uv is a top-left native integer pixel index.

    radius_px selects a clipped square neighbourhood. Rectangles are half-open;
    polygons use pixel centres and the even-odd rule. Limits: 65536 region pixels,
    64 reported hits/cells. The reusable selection includes all selected returns.
    """

    view_ref: Any
    uv: tuple[int, int] | None = None
    radius_px: int = 0
    rect: tuple[int, int, int, int] | None = None
    polygon: Any = None
    max_items: int = 16

    @property
    def selection(self) -> PickSelection:
        return PickSelection(self)

    def _measure(self, ctx: EncodeContext) -> tuple[dict[str, Any], np.ndarray]:
        if render.FORM != "image":
            raise ValueError("Pick requires the image build and native image pixels")
        if type(self.max_items) is not int or not 1 <= self.max_items <= MAX_ITEMS:
            raise ValueError("max_items must be an integer in 1..64")
        ctx = ctx.root
        node = resolve(self.view_ref, ctx)
        kind = type(node).__name__
        if kind == "DepthView":
            raster, selected = depth_data(node, ctx)
            width, height = node.size
            grid = None
        elif kind == "Map":
            data = ctx.evaluate(node.source)
            data.scalar()
            grid = data.grid
            if (
                type(node.max_side) is not int
                or not 1 <= node.max_side <= 2048
                or max(grid.shape) > node.max_side
            ):
                raise ValueError("invalid map max_side")
            scale = max(1, node.max_side // max(grid.shape))
            width, height = grid.shape[0] * scale, grid.shape[1] * scale
        elif kind == "OccupancyMap":
            selected = ctx.select(node.source)
            occupancy, heights, occupancy_indices = node.measure(selected)
            grid = field_nodes.Grid(
                (float(occupancy.origin.position.x), float(occupancy.origin.position.y)),
                (occupancy.width, occupancy.height),
                float(occupancy.resolution),
                frame=ctx.cloud.frame_id,
            )
            scale = max(1, round(render.OCCUPANCY_TARGET_PX / max(grid.shape)))
            width, height = grid.shape[0] * scale, grid.shape[1] * scale
        else:
            raise ValueError("view_ref must describe DepthView, Map, or OccupancyMap")
        pixels, outside = _pixels(self, width, height)
        out: dict[str, Any] = {
            "handler": "Pick",
            "status": "outside_image" if outside else "no_return",
            "view_ref": self.view_ref,
            "selection_ref": reference(self.selection, ctx, "selection"),
            "image_size": [width, height],
            "selected_pixels": len(pixels),
            "pixel_region": {
                "uv": self.uv,
                "radius_px": self.radius_px,
                "rect": self.rect,
                "polygon": self.polygon,
            },
        }
        empty = ctx.points[:0]
        if not len(pixels):
            return out, empty
        if kind == "DepthView":
            ids = raster.point_ids[pixels[:, 1], pixels[:, 0]]
            unique = np.unique(ids[ids >= 0])
            hits = []
            forward = render.as_view(node.view).axes()[0]
            origin = (
                np.array(node.view[:3])
                if isinstance(node.view, (tuple, list))
                else np.array(
                    [
                        render.as_view(node.view).x,
                        render.as_view(node.view).y,
                        render.as_view(node.view).z,
                    ]
                )
            )
            depths = (selected.points[unique] - origin) @ forward
            for point_id, depth in zip(
                unique[: self.max_items], depths[: self.max_items], strict=True
            ):
                locations = pixels[ids == point_id]
                provenance = np.unique(raster.provenance[locations[:, 1], locations[:, 0]])
                hits.append(
                    {
                        "point_id": int(point_id),
                        "point_m": selected.points[point_id].tolist(),
                        "forward_depth_m": float(depth),
                        "projected_uv": raster.projected_uv[point_id].tolist(),
                        "pixel_provenance": [
                            "no_return",
                            "projected_return",
                            "splat",
                            "filled_pixel",
                        ][int(provenance[0])]
                        if len(provenance) == 1
                        else [
                            ["no_return", "projected_return", "splat", "filled_pixel"][int(p)]
                            for p in provenance
                        ],
                        "selected_pixel_count": len(locations),
                    }
                )
            out.update(
                status="hit" if len(unique) == 1 else ("ambiguous" if len(unique) else "no_return"),
                hit_count=len(unique),
                hits=hits,
                hits_omitted=max(0, len(unique) - self.max_items),
                depth_span_m=[float(depths.min()), float(depths.max())] if len(unique) else None,
                point_id_scope="finite returns of the view source, in source order",
            )
            if len(unique) == 1:
                out.update(hits[0])
            return out, selected.points[unique]
        assert grid is not None
        cells = np.unique(
            np.column_stack((pixels[:, 0] // scale, grid.shape[1] - 1 - pixels[:, 1] // scale)),
            axis=0,
        )
        points = selected.points if kind == "OccupancyMap" else _cell_points(node.source, ctx)
        keep = None
        if points is not None:
            indices = (
                occupancy_indices
                if kind == "OccupancyMap"
                else grid.indices(points.astype(np.float64)[:, grid.axes])[0]
            )
            valid = ((indices >= 0) & (indices < grid.shape)).all(axis=1)
            if kind == "OccupancyMap":
                valid &= points[:, 2] <= node.z_range[1]
            cell_mask = np.zeros((grid.shape[1], grid.shape[0]), dtype=bool)
            cell_mask[cells[:, 1], cells[:, 0]] = True
            keep = np.zeros(len(points), dtype=bool)
            keep[valid] = cell_mask[indices[valid, 1], indices[valid, 0]]
        records = []
        for col, row in cells[: self.max_items]:
            lower = np.array(grid.origin) + np.array([col, row]) * grid.cell_m
            member = (
                valid & (indices[:, 0] == col) & (indices[:, 1] == row)
                if points is not None
                else None
            )
            if member is not None and kind == "OccupancyMap":
                member &= points[:, 2] <= node.z_range[1]
            local = points[member] if member is not None else empty
            entry: dict[str, Any] = {
                "cell": [int(col), int(row)],
                "bounds_m": [lower.tolist(), (lower + grid.cell_m).tolist()],
                "centre_m": (lower + grid.cell_m / 2).tolist(),
                "count": len(local) if points is not None else None,
                "min_m": float(local[:, grid.normal].min()) if len(local) else None,
                "max_m": float(local[:, grid.normal].max()) if len(local) else None,
                "contributor_scope": "cell_returns"
                if points is not None
                else "derived_field_no_local_contributors",
            }
            if kind == "Map":
                value = data.scalar()[row, col]
                entry.update(
                    channel=next(iter(data.values)),
                    value=float(value) if np.isfinite(value) else None,
                )
            else:
                entry.update(
                    channel="occupancy",
                    value=int(occupancy.grid[row, col]),
                    classification={
                        render.FREE: "free",
                        render.OCCUPIED: "occupied",
                        render.UNKNOWN: "unseen",
                    }[int(occupancy.grid[row, col])],
                    z_range_m=list(node.z_range),
                    height_colour_m=float(heights[row, col])
                    if heights is not None and np.isfinite(heights[row, col])
                    else None,
                )
            records.append(entry)
        selected_points = points[keep] if keep is not None else empty
        out.update(
            status="hit"
            if len(selected_points)
            else ("field_value" if points is None else "no_return"),
            grid=grid.describe(ctx),
            cell_count=len(cells),
            cells=records,
            cells_omitted=max(0, len(cells) - self.max_items),
            selected_return_count=len(selected_points),
            selection_scope="cell_returns" if points is not None else "none_for_derived_field",
            bounds_m=[selected_points.min(0).tolist(), selected_points.max(0).tolist()]
            if len(selected_points)
            else None,
        )
        return out, selected_points

    def computed(self, ctx: EncodeContext) -> tuple[dict[str, Any], np.ndarray]:
        key = ("pick_measurement", _json(_spec(self)))
        if key not in ctx.cache:
            ctx.cache[key] = self._measure(ctx)
        return ctx.cache[key]

    def run(self, ctx: EncodeContext) -> dict[str, Any]:
        return dict(self.computed(ctx)[0])


@dataclass(frozen=True)
class PickSelection:
    pick: Pick

    def run(self, ctx: EncodeContext) -> np.ndarray:
        return self.pick.computed(ctx)[1]


@dataclass(frozen=True)
class SelectionRef:
    """Reopen a Pick result's JSON selection_ref against the exact original cloud."""

    selection_ref: Any

    def run(self, ctx: EncodeContext) -> np.ndarray:
        node = resolve(self.selection_ref, ctx, "selection")
        if not isinstance(node, PickSelection):
            raise ValueError("selection_ref must describe a pick selection")
        return node.run(ctx.root)
