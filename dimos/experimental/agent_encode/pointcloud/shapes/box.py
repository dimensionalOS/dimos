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
import math

import numpy as np
from numpy.typing import NDArray

from dimos.experimental.agent_encode.pointcloud.shapes.base import Shape


@dataclass(frozen=True)
class Box(Shape):
    """A box turned about z by ``yaw_deg``."""

    center: tuple[float, float, float]
    """x, y, z in the cloud's frame."""
    size: tuple[float, float, float]
    """x, y, z full extents in metres."""
    yaw_deg: float = 0.0
    """Rotation about z."""

    def _rotation(self) -> NDArray[np.float64]:
        """Box axes to world: the columns are the box's x, y, z in the cloud's frame."""
        c, s = math.cos(math.radians(self.yaw_deg)), math.sin(math.radians(self.yaw_deg))
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    def _local(
        self, points: NDArray[np.float32] | NDArray[np.float64]
    ) -> NDArray[np.float32] | NDArray[np.float64]:
        """Points in the box frame: origin at the centre, x along the yaw."""
        rel = points - np.asarray(self.center, dtype=points.dtype)
        local: NDArray[np.float32] | NDArray[np.float64] = rel @ self._rotation().astype(
            points.dtype
        )
        return local

    def contains(self, points: NDArray[np.float32] | NDArray[np.float64]) -> NDArray[np.bool_]:
        half = (np.asarray(self.size, dtype=np.float64) / 2.0).astype(points.dtype)
        return np.asarray(np.all(np.abs(self._local(points)) <= half, axis=1), dtype=np.bool_)

    def distance(self, points: NDArray[np.float32] | NDArray[np.float64]) -> NDArray[np.float64]:
        half = np.asarray(self.size, dtype=np.float64) / 2.0
        local = self._local(points.astype(np.float64))
        outside = np.maximum(np.abs(local) - half, 0.0)
        d: NDArray[np.float64] = np.linalg.norm(outside, axis=1)
        return d

    def chord(self, direction: NDArray[np.float64]) -> float:
        along = np.abs(self._rotation().T @ direction)
        lengths = np.divide(self.size, along, out=np.full(3, np.inf), where=along > 0)
        return float(lengths.min())

    def anchor(self, z_extent: tuple[float, float]) -> NDArray[np.float64]:
        return np.asarray(self.center, dtype=np.float64)

    def wireframe(self, z_extent: tuple[float, float]) -> list[NDArray[np.float64]]:
        signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
        corners = (
            signs * np.asarray(self.size, dtype=np.float64) / 2
        ) @ self._rotation().T + self.anchor(z_extent)
        return [corners[[i, i ^ bit]] for i in range(8) for bit in (1, 2, 4) if i < i ^ bit]
