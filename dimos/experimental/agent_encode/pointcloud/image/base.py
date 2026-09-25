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

"""What every rendered image offers: its file, the world under its pixels, and drawing."""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from PIL import Image as PILImage, ImageDraw

from dimos.experimental.agent_encode.pointcloud.image.lib.canvas import Canvas, Drawable
from dimos.experimental.agent_encode.pointcloud.image.lib.colour import ColourScale
from dimos.experimental.agent_encode.pointcloud.queries.lib.points import finite_points
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass(frozen=True, eq=False)
class Drawn:
    """Which colour marks which ``draw=`` item."""

    label: str
    """The item's type name."""
    colour: str


@dataclass(frozen=True, eq=False)
class Image(ABC):
    """A rendered image file whose pixels map to the world and back."""

    path: Path
    size: tuple[int, int]
    """(width, height) in pixels; pixel (0, 0) is the top-left."""
    scale: ColourScale | None
    """How to read values off the colours; None when the colours are classes."""
    drawn: tuple[Drawn, ...]

    @abstractmethod
    def pixel(self, point: tuple[float, ...]) -> tuple[float, float] | None:
        """The continuous (u, v) where a world point lands; the floor of each is the pixel
        to pick. It may lie outside the image."""

    @abstractmethod
    def world(self, uv: tuple[int, int]) -> tuple[float, ...] | None:
        """The world position pixel ``uv`` shows; None when it shows nothing."""

    @abstractmethod
    def returns_under(self, pixels: NDArray[np.int64]) -> PointCloud2:
        """The returns that drew the (K, 2) (u, v) ``pixels``, each once."""

    def pick(
        self,
        uv: tuple[int, int] | None = None,
        radius_px: int = 0,
        rect: tuple[int, int, int, int] | None = None,
        polygon: tuple[tuple[float, float], ...] | None = None,
    ) -> PointCloud2:
        """The returns under one pixel region, at most 65536 pixels: ``uv`` with a square
        of ``radius_px`` (0..64) around it clipped to the image, ``rect`` (u, v, width,
        height) half-open, or a ``polygon`` of 3..32 vertices selecting pixel centres by
        the even-odd rule. A ``uv`` outside the image picks nothing."""
        pixels = selected_pixels(uv, radius_px, rect, polygon, self.size)
        return self.returns_under(pixels)

    def __repr__(self) -> str:
        return f"{type(self).__name__}({self.size[0]}x{self.size[1]} px, {self.path})"


def selected_pixels(
    uv: tuple[int, int] | None,
    radius_px: int,
    rect: tuple[int, int, int, int] | None,
    polygon: tuple[tuple[float, float], ...] | None,
    size: tuple[int, int],
) -> NDArray[np.int64]:
    """The (K, 2) (u, v) pixels a pick selects on an image of ``size``."""
    width, height = size
    if sum(v is not None for v in (uv, rect, polygon)) != 1:
        raise ValueError("choose exactly one of uv, rect, polygon")
    if type(radius_px) is not int or not 0 <= radius_px <= 64:
        raise ValueError("radius_px must be an integer in 0..64")
    if uv is None and radius_px:
        raise ValueError("radius_px applies only to uv")
    vertices = None
    if uv is not None:
        if len(uv) != 2 or any(type(v) is not int for v in uv):
            raise ValueError("uv must contain integer pixel indices")
        u, v = uv
        if not (0 <= u < width and 0 <= v < height):
            return np.empty((0, 2), dtype=np.int64)
        r = radius_px
        x0, y0, x1, y1 = max(0, u - r), max(0, v - r), min(width, u + r + 1), min(height, v + r + 1)
    elif rect is not None:
        if len(rect) != 4 or any(type(v) is not int for v in rect):
            raise ValueError("rect must contain integer (u, v, width, height)")
        x0, y0, w, h = rect
        if min(w, h) <= 0:
            raise ValueError("rect width and height must be positive")
        x1, y1 = x0 + w, y0 + h
    else:
        vertices = np.asarray(polygon, dtype=np.float64)
        if (
            vertices.ndim != 2
            or vertices.shape[1] != 2
            or not 3 <= len(vertices) <= 32
            or not np.isfinite(vertices).all()
        ):
            raise ValueError("polygon needs 3..32 finite pixel vertices")
        if np.any(vertices < 0) or np.any(vertices > [width, height]):
            raise ValueError("polygon must lie within the image edges")
        x0, y0 = np.floor(vertices.min(0)).astype(int)
        x1, y1 = np.ceil(vertices.max(0)).astype(int)
    if x0 < 0 or y0 < 0 or x1 > width or y1 > height:
        raise ValueError("region must lie inside the image edges")
    if (x1 - x0) * (y1 - y0) > 65536:
        raise ValueError("pick region exceeds 65536 pixels; use a smaller region")
    yy, xx = np.mgrid[y0:y1, x0:x1]
    pixels: NDArray[np.int64] = np.column_stack((xx.ravel(), yy.ravel())).astype(np.int64)
    if vertices is not None:
        # Even-odd rule at pixel centres; boundary ties use this same deterministic rule.
        x, y = pixels[:, 0] + 0.5, pixels[:, 1] + 0.5
        inside = np.zeros(len(pixels), dtype=np.bool_)
        for a, b in zip(vertices, np.roll(vertices, -1, axis=0), strict=True):
            if a[1] != b[1]:
                inside ^= ((a[1] > y) != (b[1] > y)) & (
                    x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]
                )
        pixels = pixels[inside]
    return pixels


def draw_items(
    picture: PILImage.Image,
    items: tuple[Drawable | PointCloud2, ...],
    project: Callable[[NDArray[np.float64]], NDArray[np.float64]],
    z_extent: tuple[float, float],
    palette: tuple[str, ...] = ("#ff3bcc", "#00cfef", "#f79b24", "#91d52a"),
) -> tuple[Drawn, ...]:
    """Draw each item onto ``picture`` through the image's projection, taking ``palette``
    colours in turn; a cloud is drawn as small dots at its finite returns."""
    if not items:
        return ()
    pen = ImageDraw.Draw(picture)

    def project_one(point: NDArray[np.float64]) -> tuple[float, float] | None:
        u, v = project(point[None].astype(np.float64))[0]
        return None if np.isnan(u) else (float(u), float(v))

    drawn = []
    for index, item in enumerate(items):
        colour = palette[index % len(palette)]
        if isinstance(item, PointCloud2):
            uv = project(finite_points(item).astype(np.float64))
            uv = uv[np.isfinite(uv).all(axis=1)]
            for du in (-1, 0, 1):
                for dv in (-1, 0, 1):
                    pen.point((uv + np.array((du, dv))).ravel().tolist(), fill=colour)
        else:
            item.draw(Canvas(pen, project_one, colour, z_extent))
        drawn.append(Drawn(type(item).__name__, colour))
    return tuple(drawn)
