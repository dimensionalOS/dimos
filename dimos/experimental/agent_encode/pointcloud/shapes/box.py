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


@dataclass(frozen=True)
class Box:
    """An axis-aligned-in-z box."""

    center: tuple[float, float, float]
    """x, y, z in the cloud's frame."""
    size: tuple[float, float, float]
    """x, y, z full extents in metres."""
    yaw_deg: float = 0.0
    """Rotation about z."""

    def _local(self, points: np.ndarray) -> np.ndarray:
        """Points in the box frame: origin at the centre, x along the yaw."""
        rel = points - np.asarray(self.center, dtype=points.dtype)
        c, s = math.cos(math.radians(self.yaw_deg)), math.sin(math.radians(self.yaw_deg))
        local: np.ndarray = np.empty_like(rel)
        local[:, 0] = rel[:, 0] * c + rel[:, 1] * s
        local[:, 1] = -rel[:, 0] * s + rel[:, 1] * c
        local[:, 2] = rel[:, 2]
        return local

    def contains(self, points: np.ndarray) -> np.ndarray:
        half = np.asarray(self.size, dtype=points.dtype) / 2.0
        inside: np.ndarray = (np.abs(self._local(points)) <= half).all(axis=1)
        return inside

    def distance(self, points: np.ndarray) -> np.ndarray:
        """Distance from the box surface to each point; 0 inside."""
        half = np.asarray(self.size, dtype=points.dtype) / 2.0
        outside = np.maximum(np.abs(self._local(points)) - half, 0.0)
        d: np.ndarray = np.linalg.norm(outside, axis=1)
        return d

    def shifted(self, dx: float, dy: float) -> Box:
        cx, cy, cz = self.center
        return Box((cx + dx, cy + dy, cz), self.size, self.yaw_deg)

    def describe(self) -> dict[str, object]:
        return {
            "shape": "Box",
            "center": list(self.center),
            "size": list(self.size),
            "yaw_deg": self.yaw_deg,
        }
