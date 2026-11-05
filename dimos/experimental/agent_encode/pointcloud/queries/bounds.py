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

from dimos.experimental.agent_encode.pointcloud.queries.base import Query
from dimos.experimental.agent_encode.pointcloud.queries.lib.points import finite_points

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass(frozen=True)
class Bounds(Query["tuple[tuple[float, float, float], tuple[float, float, float]] | None"]):
    """The lowest and highest corner of the finite returns, rounded outward to the
    millimetre; None when there are none."""

    def run(
        self, cloud: PointCloud2
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]] | None:
        points = finite_points(cloud)
        if not len(points):
            return None
        low = np.floor(points.min(axis=0).astype(np.float64) * 1000) / 1000
        high = np.ceil(points.max(axis=0).astype(np.float64) * 1000) / 1000
        (x0, y0, z0), (x1, y1, z1) = low.tolist(), high.tolist()
        return (x0, y0, z0), (x1, y1, z1)
