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

from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass
from itertools import pairwise
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from pydantic import JsonValue

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


@dataclass(frozen=True)
class Canvas:
    """One overlay's pen on a render: its colour and the render's own projection."""

    draw: ImageDraw.ImageDraw
    project: Project
    colour: str
    ctx: EncodeContext
    """The render's original context; queries evaluate against it."""
    view_ref: dict[str, JsonValue] | None
    """The render's reference, so a pick highlights pixels only on its own view."""

    @property
    def z_extent(self) -> tuple[float, float]:
        """The cloud's lowest and highest return, where unbounded shapes end."""
        z = self.ctx.points[:, 2]
        return (float(z.min()), float(z.max())) if len(z) else (0.0, 0.0)

    def path(self, points: np.ndarray) -> None:
        for start, end in pairwise(points):
            a, b = self.project(start), self.project(end)
            if a is not None and b is not None:
                self.draw.line((a, b), fill=self.colour, width=2)

    def point(self, point: np.ndarray, radius: int, *, filled: bool) -> bool:
        """Mark a world point; whether it was in view."""
        pixel = self.project(point)
        if pixel is None:
            return False
        x, y = pixel
        box = (x - radius, y - radius, x + radius, y + radius)
        if filled:
            self.draw.ellipse(box, fill=self.colour, outline="white")
        else:
            self.draw.ellipse(box, outline=self.colour, width=2)
        return True


class Overlay(ABC):
    """Something a render can draw over its image."""

    @abstractmethod
    def draw(self, canvas: Canvas) -> list[str]:
        """Draw onto ``canvas``; the kinds of geometry drawn."""


@dataclass(frozen=True)
class Segment(Overlay):
    """A polyline through world points."""

    points: tuple[tuple[float, float, float], ...]

    def draw(self, canvas: Canvas) -> list[str]:
        canvas.path(np.asarray(self.points, dtype=float))
        return ["segment"]


@dataclass(frozen=True)
class DrawnOverlay:
    label: str
    """The overlay's class name."""
    colour: str
    geometry: list[str]
    """What was drawn: shape, point, nearest_segment, sweep_segment, picked_pixels,
    picked_return, picked_cell_at_min or segment."""


def draw_overlays(
    path: Path | str,
    overlays: tuple[Overlay, ...],
    ctx: EncodeContext,
    project: Project,
    *,
    view_ref: dict[str, JsonValue] | None = None,
    palette: tuple[str, ...] = ("#ff3bcc", "#00cfef", "#f79b24", "#91d52a"),
) -> list[DrawnOverlay]:
    """Draw each overlay with the render's projection, taking ``palette`` colours in
    turn; which colour marks which overlay and what it drew."""
    if not overlays:
        return []
    with Image.open(path) as original:
        image = original.convert("RGB")
    draw = ImageDraw.Draw(image)
    drawn: list[DrawnOverlay] = []
    for index, overlay in enumerate(overlays):
        if not isinstance(overlay, Overlay):
            raise TypeError(
                f"{type(overlay).__name__} cannot be drawn; overlays are shapes, Overlap, "
                "Closest, Sweep, Pick or Segment"
            )
        colour = palette[index % len(palette)]
        geometry = overlay.draw(Canvas(draw, project, colour, ctx, view_ref))
        drawn.append(DrawnOverlay(type(overlay).__name__, colour, geometry))
    image.save(path)
    return drawn
