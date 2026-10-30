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

"""Drawing world geometry onto an image through the image's own projection."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from itertools import pairwise
import math
from typing import Protocol

import numpy as np
from numpy.typing import NDArray
from PIL import ImageDraw

Project = Callable[[NDArray[np.float64]], tuple[float, float] | None]


@dataclass(frozen=True)
class Canvas:
    """One drawable's pen on an image."""

    draw: ImageDraw.ImageDraw
    project: Project
    """World point (x, y, z) to pixel (u, v); None when it is not in view."""
    colour: str
    z_extent: tuple[float, float]
    """The drawn cloud's lowest and highest z, where unbounded shapes end."""

    def path(self, points: NDArray[np.float64]) -> None:
        """A polyline through world points; segments with an end out of view are skipped."""
        for start, end in pairwise(points):
            a, b = self.project(start), self.project(end)
            if a is not None and b is not None:
                self.draw.line((a, b), fill=self.colour, width=2)

    def point(self, point: NDArray[np.float64], radius: int, *, filled: bool) -> None:
        """A dot at a world point, when it is in view."""
        pixel = self.project(point)
        if pixel is None:
            return
        x, y = pixel
        box = (x - radius, y - radius, x + radius, y + radius)
        if filled:
            self.draw.ellipse(box, fill=self.colour, outline="white")
        else:
            self.draw.ellipse(box, outline=self.colour, width=2)


class Drawable(Protocol):
    """Anything an image's ``draw=`` accepts."""

    def draw(self, canvas: Canvas) -> None: ...


@dataclass(frozen=True)
class Line(Drawable):
    """A polyline through world points; (x, y) points are drawn at the canvas's lowest z."""

    points: tuple[tuple[float, ...], ...]

    def draw(self, canvas: Canvas) -> None:
        xyz = np.array(
            [(*p[:2], p[2] if len(p) > 2 else canvas.z_extent[0]) for p in self.points],
            dtype=np.float64,
        )
        canvas.path(xyz)


@dataclass(frozen=True)
class Arrow(Drawable):
    """A pose: a dot at (x, y) and a line along its heading."""

    x: float
    y: float
    yaw_deg: float
    """0 faces +x, positive turns toward +y. Odometry yaw is in radians: math.degrees(pose.yaw)."""
    z: float | None = None
    """Height it is drawn at; None is the drawn cloud's lowest z."""
    length_m: float = 0.5
    """How far the heading line reaches."""

    def draw(self, canvas: Canvas) -> None:
        z = canvas.z_extent[0] if self.z is None else self.z
        yaw = math.radians(self.yaw_deg)
        tip = (self.x + self.length_m * math.cos(yaw), self.y + self.length_m * math.sin(yaw))
        canvas.path(np.array([(self.x, self.y, z), (*tip, z)], dtype=np.float64))
        canvas.point(np.array([self.x, self.y, z], dtype=np.float64), 12, filled=True)
