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
class Sphere(Shape):
    """A sphere."""

    center: tuple[float, float, float]
    """x, y, z."""
    radius: float
    """In metres. 0 is a point, useful for "the nearest return to here in any direction"."""

    def _from_center(self, points: np.ndarray) -> np.ndarray:
        d: np.ndarray = np.linalg.norm(points - np.asarray(self.center, dtype=points.dtype), axis=1)
        return d

    def contains(self, points: np.ndarray) -> np.ndarray:
        return self._from_center(points) <= self.radius

    def distance(self, points: np.ndarray) -> np.ndarray:
        """Distance from the sphere surface to each point; 0 inside."""
        return np.maximum(self._from_center(points) - self.radius, 0.0)

    def chord(self, direction: np.ndarray) -> float:
        """Length of the longest segment along any unit ``direction`` inside the sphere."""
        return 2.0 * self.radius

    def anchor(self, z_extent: tuple[float, float]) -> np.ndarray:
        return np.asarray(self.center, dtype=float)

    def wireframe(self, z_extent: tuple[float, float]) -> list[np.ndarray]:
        angles = np.linspace(0, 2 * np.pi, 49)
        circle = np.column_stack((np.cos(angles), np.sin(angles))) * self.radius
        lines = []
        for a, b in ((0, 1), (0, 2), (1, 2)):
            points = np.zeros((len(circle), 3))
            points[:, [a, b]] = circle
            lines.append(points + self.anchor(z_extent))
        return lines
