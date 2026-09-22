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

import numpy as np
from numpy.typing import NDArray

from dimos.experimental.agent_encode.pointcloud.shapes.base import Shape
from dimos.experimental.agent_encode.pointcloud.z_band import (
    ZBand,
    check_z_band,
    closed_z_band,
    in_z_band,
)


@dataclass(frozen=True)
class Cylinder(Shape):
    """A vertical cylinder."""

    center: tuple[float, float]
    """x, y."""
    radius: float
    """In metres. 0 is a vertical line."""
    z: ZBand = (None, None)
    """The heights it spans."""

    def __post_init__(self) -> None:
        check_z_band(self.z)

    def _horizontal(self, points: NDArray[np.float32] | NDArray[np.float64]) -> NDArray[np.float64]:
        rel = points[:, :2] - np.asarray(self.center, dtype=points.dtype)
        d: NDArray[np.float64] = np.linalg.norm(rel, axis=1).astype(np.float64)
        return d

    def _in_band(self, points: NDArray[np.float32] | NDArray[np.float64]) -> NDArray[np.bool_]:
        return in_z_band(points[:, 2], self.z)

    def contains(self, points: NDArray[np.float32] | NDArray[np.float64]) -> NDArray[np.bool_]:
        return self._in_band(points) & (self._horizontal(points) <= self.radius)

    def distance(self, points: NDArray[np.float32] | NDArray[np.float64]) -> NDArray[np.float64]:
        """Horizontal distance from the cylinder surface to each point inside the z band;
        ``inf`` for points outside the band; 0 inside."""
        d = np.maximum(self._horizontal(points) - self.radius, 0.0)
        return np.where(self._in_band(points), d, np.inf)

    def chord(self, direction: NDArray[np.float64]) -> float:
        low, high = self.z
        span = np.inf if low is None or high is None else high - low
        along = np.array([np.hypot(direction[0], direction[1]), abs(direction[2])])
        lengths = np.divide(
            [2.0 * self.radius, span], along, out=np.full(2, np.inf), where=along > 0
        )
        return float(lengths.min())

    def anchor(self, z_extent: tuple[float, float]) -> NDArray[np.float64]:
        return np.array([*self.center, sum(closed_z_band(self.z, z_extent)) / 2], dtype=np.float64)

    def wireframe(self, z_extent: tuple[float, float]) -> list[NDArray[np.float64]]:
        angles = np.linspace(0, 2 * np.pi, 49)
        xy = np.column_stack((np.cos(angles), np.sin(angles))) * self.radius + self.center
        low, high = closed_z_band(self.z, z_extent)
        lower = np.column_stack((xy, np.full(len(xy), low)))
        upper = np.column_stack((xy, np.full(len(xy), high)))
        return [lower, upper, *[np.stack((lower[i], upper[i])) for i in (0, 12, 24, 36)]]
