# Copyright 2025-2026 Dimensional Inc.
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

from dimos.msgs.nav_msgs.Path import Path


@dataclass(frozen=True)
class PolylineProjection:
    """Foot of a query point on a polyline and the arc length from the start to it."""

    foot_xy: tuple[float, float]
    s_along_path_m: float


def project_to_polyline(x: float, y: float, polyline_xy: NDArray[np.float64]) -> PolylineProjection:
    """Project a point onto the nearest segment of a ``(N, 2)`` polyline."""
    if len(polyline_xy) == 1:
        return PolylineProjection((float(polyline_xy[0, 0]), float(polyline_xy[0, 1])), 0.0)
    starts = polyline_xy[:-1]
    seg = polyline_xy[1:] - starts
    seg_len2 = np.maximum(np.einsum("ij,ij->i", seg, seg), 1e-18)
    t = np.clip(np.einsum("ij,ij->i", np.array([x, y]) - starts, seg) / seg_len2, 0.0, 1.0)
    feet = starts + t[:, None] * seg
    best = int(np.argmin(np.einsum("ij,ij->i", feet - [x, y], feet - [x, y])))
    seg_lens = np.sqrt(seg_len2)
    s = float(seg_lens[:best].sum() + t[best] * seg_lens[best])
    return PolylineProjection((float(feet[best, 0]), float(feet[best, 1])), s)


class PathDistancer:
    """Arc-length projection onto a fixed ``Path`` polyline."""

    def __init__(self, path: Path) -> None:
        self._path = np.array([[p.position.x, p.position.y] for p in path.poses])

    def project(self, pos: NDArray[np.float64]) -> PolylineProjection:
        return project_to_polyline(float(pos[0]), float(pos[1]), self._path)
