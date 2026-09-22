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

from dimos.experimental.agent_encode.pointcloud.shapes.base import Shape


@dataclass(frozen=True)
class Cylinder(Shape):
    """A vertical cylinder."""

    center: tuple[float, float]
    """x, y."""
    radius: float
    """In metres. 0 is a vertical line, useful for "the nearest return to this spot"."""
    z_range: tuple[float | None, float | None]
    """The absolute (low, high) it spans in the cloud's frame; None = unbounded, as in Band."""

    def __post_init__(self) -> None:
        if not isinstance(self.z_range, (tuple, list)) or len(self.z_range) != 2:
            raise ValueError("z_range must be a (low, high) pair; use (None, None) for unbounded")

    def _horizontal(self, points: np.ndarray) -> np.ndarray:
        rel = points[:, :2] - np.asarray(self.center, dtype=points.dtype)
        d: np.ndarray = np.linalg.norm(rel, axis=1)
        return d

    def _in_band(self, points: np.ndarray) -> np.ndarray:
        low, high = self.z_range
        low = -np.inf if low is None else low
        high = np.inf if high is None else high
        return (points[:, 2] >= low) & (points[:, 2] <= high)

    def contains(self, points: np.ndarray) -> np.ndarray:
        return self._in_band(points) & (self._horizontal(points) <= self.radius)

    def distance(self, points: np.ndarray) -> np.ndarray:
        """Horizontal distance from the cylinder surface to each point inside
        the z band; ``inf`` for points outside the band; 0 inside."""
        d = np.maximum(self._horizontal(points) - self.radius, 0.0)
        return np.where(self._in_band(points), d, np.inf)

    def chord(self, direction: np.ndarray) -> float:
        """Length of the longest segment along the unit ``direction`` inside the cylinder."""
        low, high = self.z_range
        span = np.inf if low is None or high is None else high - low
        along = np.array([np.hypot(direction[0], direction[1]), abs(direction[2])])
        lengths = np.divide(
            [2.0 * self.radius, span], along, out=np.full(2, np.inf), where=along > 0
        )
        return float(lengths.min())

    def _closed(self, z_extent: tuple[float, float]) -> tuple[float, float]:
        low, high = self.z_range
        return (z_extent[0] if low is None else low, z_extent[1] if high is None else high)

    def anchor(self, z_extent: tuple[float, float]) -> np.ndarray:
        return np.array([*self.center, sum(self._closed(z_extent)) / 2])

    def wireframe(self, z_extent: tuple[float, float]) -> list[np.ndarray]:
        angles = np.linspace(0, 2 * np.pi, 49)
        xy = np.column_stack((np.cos(angles), np.sin(angles))) * self.radius + self.center
        low, high = self._closed(z_extent)
        lower = np.column_stack((xy, np.full(len(xy), low)))
        upper = np.column_stack((xy, np.full(len(xy), high)))
        return [lower, upper, *[np.stack((lower[i], upper[i])) for i in (0, 12, 24, 36)]]
