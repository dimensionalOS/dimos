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
from typing import TYPE_CHECKING

import numpy as np

from dimos.experimental.agent_encode.pointcloud.image.lib.canvas import Canvas, Drawable
from dimos.experimental.agent_encode.pointcloud.queries.base import Query
from dimos.experimental.agent_encode.pointcloud.queries.lib.points import finite_points

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass(frozen=True)
class Hit(Drawable):
    """The return nearest to a query point."""

    from_m: tuple[float, float, float]
    """The query point; its z is the hit's z when the query gave only x, y."""
    point_m: tuple[float, float, float]
    """The nearest return."""
    distance_m: float
    """Horizontal when the query gave x, y; 3D when it gave x, y, z."""

    def draw(self, canvas: Canvas) -> None:
        point = np.asarray(self.point_m, dtype=np.float64)
        canvas.point(point, 4, filled=True)
        canvas.path(np.stack((np.asarray(self.from_m, dtype=np.float64), point)))


@dataclass(frozen=True)
class Nearest(Query["Hit | None"]):
    """The return nearest to a point; None when the cloud has no returns."""

    to: tuple[float, float] | tuple[float, float, float]
    """(x, y) measures horizontally to every return; (x, y, z) measures in 3D."""

    def __post_init__(self) -> None:
        if len(self.to) not in (2, 3) or not np.isfinite(self.to).all():
            raise ValueError("to must be two (x, y) or three (x, y, z) finite coordinates")

    def run(self, cloud: PointCloud2) -> Hit | None:
        points = finite_points(cloud).astype(np.float64)
        if not len(points):
            return None
        dims = len(self.to)
        d = np.linalg.norm(points[:, :dims] - np.asarray(self.to, dtype=np.float64), axis=1)
        i = int(np.argmin(d))
        x, y, z = (round(float(v), 3) for v in points[i])
        origin = (*self.to, z) if dims == 2 else self.to
        fx, fy, fz = (round(float(v), 3) for v in origin)
        return Hit(from_m=(fx, fy, fz), point_m=(x, y, z), distance_m=round(float(d[i]), 3))
