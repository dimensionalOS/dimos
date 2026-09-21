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

"""Project geometric query evidence onto existing renders."""

from __future__ import annotations

from collections.abc import Callable
from itertools import pairwise
import math
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw

from dimos.experimental.agent_encode.pointcloud.runtime.context import EncodeContext

Project = Callable[[np.ndarray], tuple[float, float] | None]


def grid_pixel(
    coordinates: np.ndarray,
    axes: tuple[int, int],
    origin: tuple[float, float] | np.ndarray,
    cell_m: float,
    rows: int,
    pixels_per_cell: int,
) -> tuple[float, float]:
    """Project grid coordinates onto pixel centres in a vertically flipped image."""
    u, v = (coordinates[list(axes)] - np.asarray(origin)) / cell_m
    return (
        float(u * pixels_per_cell - 0.5),
        float((rows - v) * pixels_per_cell - 0.5),
    )


def shape_lines(shape: dict[str, Any]) -> list[np.ndarray]:
    """World-coordinate wireframe paths for the supported geometric shapes."""
    kind = shape.get("shape")
    if kind == "Box":
        half = np.asarray(shape["size"], dtype=float) / 2
        corners = (
            np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], dtype=float)
            * half
        )
        angle = math.radians(shape.get("yaw_deg", 0.0))
        c, s = math.cos(angle), math.sin(angle)
        rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        corners = corners @ rotation.T + np.asarray(shape["center"])
        return [corners[[i, i ^ bit]] for i in range(8) for bit in (1, 2, 4) if i < i ^ bit]
    if kind not in ("Sphere", "Cylinder"):
        return []
    angles = np.linspace(0, 2 * math.pi, 49)
    circle = np.column_stack((np.cos(angles), np.sin(angles))) * shape["radius"]
    if kind == "Sphere":
        center = np.asarray(shape["center"])
        lines = []
        for a, b in ((0, 1), (0, 2), (1, 2)):
            points = np.zeros((len(circle), 3))
            points[:, [a, b]] = circle
            lines.append(points + center)
        return lines
    xy = circle + np.asarray(shape["center"])
    low, high = shape["z_range"]
    lower = np.column_stack((xy, np.full(len(xy), low)))
    upper = np.column_stack((xy, np.full(len(xy), high)))
    return [lower, upper, *[np.stack((lower[i], upper[i])) for i in (0, 12, 24, 36)]]


def draw_overlays(
    path: Path | str,
    overlays: tuple[Any, ...],
    ctx: EncodeContext,
    project: Project,
    *,
    view_ref: dict[str, Any] | None = None,
    palette: tuple[str, ...] = ("#ff3bcc", "#00cfef", "#f79b24", "#91d52a"),
) -> list[dict[str, Any]]:
    """Draw shapes, query contacts, and sweep paths with the render's own projection.

    Queries are evaluated against the original context, honoring each query's source.
    Overlays take ``palette`` colours in turn; the returned metadata identifies each
    colour and the query evidence it represents.
    """
    if not overlays:
        return []
    with Image.open(path) as original:
        canvas = original.convert("RGB")
    draw = ImageDraw.Draw(canvas)
    metadata: list[dict[str, Any]] = []
    for index, overlay in enumerate(overlays):
        if isinstance(overlay, dict):
            result = overlay
        elif hasattr(overlay, "run"):
            result = ctx.evaluate(overlay)
        elif hasattr(overlay, "describe"):
            result = overlay.describe()
        else:
            raise TypeError("overlays must be geometric shapes, query nodes, or query results")
        if not isinstance(result, dict):
            raise TypeError("overlay queries must return geometric result dictionaries")
        colour = palette[index % len(palette)]
        if None in result.get("z_range", ()):
            # Unbounded cylinder ends are drawn at the cloud's lowest/highest return.
            z = ctx.points[:, 2]
            low, high = result["z_range"]
            bounded = (
                float(z.min()) if low is None else low,
                float(z.max()) if high is None else high,
            )
            result = {**result, "z_range": bounded}
        paths = shape_lines(result)
        geometry = ["shape"] if paths else []
        if result.get("handler") == "Pick":
            # Native-pixel highlights are valid only for the exact referenced view recipe.
            if view_ref is not None and result.get("view_ref") == view_ref:
                region = result["pixel_region"]
                if region.get("uv") is not None:
                    u, v = region["uv"]
                    r = region["radius_px"]
                    draw.rectangle((u - r, v - r, u + r, v + r), outline=colour, width=1)
                    draw.ellipse((u - 4, v - 4, u + 4, v + 4), outline=colour, width=1)
                elif region.get("rect") is not None:
                    u, v, w, h = region["rect"]
                    draw.rectangle((u, v, u + w - 1, v + h - 1), outline=colour, width=2)
                else:
                    draw.polygon([tuple(p) for p in region["polygon"]], outline=colour, width=2)
                geometry.append("picked_pixels")
            for hit in result.get("hits", []):
                pixel = project(np.asarray(hit["point_m"], dtype=float))
                if pixel is not None:
                    x, y = pixel
                    draw.ellipse((x - 3, y - 3, x + 3, y + 3), outline=colour, width=2)
                    geometry.append("picked_return")
            grid = result.get("grid")
            if grid is not None:
                axes = ["xyz".index(axis) for axis in grid["plane"]]
                normal = next(i for i in range(3) if i not in axes)
                for cell in result["cells"]:
                    if cell["min_m"] is None:
                        continue
                    (a, b), (c, d) = cell["bounds_m"]
                    corners = np.zeros((5, 3))
                    corners[:, axes] = [[a, b], [c, b], [c, d], [a, d], [a, b]]
                    corners[:, normal] = cell["min_m"]
                    paths.append(corners)
                    geometry.append("picked_cell_at_min")
        center_values = result.get("center")
        center = None
        if center_values is not None:
            center = np.asarray(center_values, dtype=float)
            if len(center) == 2:
                center = np.append(center, np.mean(result.get("z_range", (0.0, 0.0))))
        point = result.get("point_m")
        if point is not None:
            pixel = project(np.asarray(point, dtype=float))
            if pixel is not None:
                x, y = pixel
                draw.ellipse((x - 4, y - 4, x + 4, y + 4), fill=colour, outline="white")
                geometry.append("point")
            if center is not None:
                paths.append(np.stack((center, point)))
                geometry.append("nearest_segment")
        if result.get("handler") == "Sweep" and center is not None:
            direction = np.asarray(result["direction"])
            distance = result["distance_m"] if result["hit"] else result["max_distance"]
            paths.append(np.stack((center, center + direction * distance)))
            geometry.append("sweep_segment")
        if result.get("segment_m") is not None:
            paths.append(np.asarray(result["segment_m"], dtype=float))
            geometry.append("segment")
        for points in paths:
            for start, end in pairwise(points):
                a, b = project(start), project(end)
                if a is not None and b is not None:
                    draw.line((a, b), fill=colour, width=2)
        metadata.append(
            {
                "label": result.get("handler", result.get("shape", "geometry")),
                "colour": colour,
                "geometry": geometry,
                "result": result,
            }
        )
    canvas.save(path)
    return metadata
