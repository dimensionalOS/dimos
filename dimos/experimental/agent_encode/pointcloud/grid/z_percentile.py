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

"""A percentile of the returns' z per cell."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from dimos.experimental.agent_encode.pointcloud.grid.base import Grid
from dimos.experimental.agent_encode.pointcloud.grid.lib.cells import bin_returns
from dimos.experimental.agent_encode.pointcloud.queries.base import Query

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass(frozen=True)
class ZPercentile(Query[Grid]):
    """The q-th percentile of z in each cell's column, interpolating linearly between
    sorted returns; NaN where the cell has fewer than min_count returns."""

    q: float
    """In [0, 100]."""
    cell_m: float
    min_count: int = 4
    """Returns a cell needs to have a value."""
    area: tuple[tuple[float, float], tuple[float, float]] | None = None
    """((x0, y0), (x1, y1)) the cells cover; None covers every finite return."""

    def __post_init__(self) -> None:
        if not np.isfinite(self.q) or not 0 <= self.q <= 100:
            raise ValueError("q must be finite and between 0 and 100")
        if type(self.min_count) is not int or self.min_count <= 0:
            raise ValueError("min_count must be a positive integer")

    def run(self, cloud: PointCloud2) -> Grid:
        binned = bin_returns(cloud, self.cell_m, self.area)
        count = binned.count()
        ordered = binned.z[np.lexsort((binned.z, binned.index))]
        supported = count >= self.min_count
        starts = np.cumsum(count) - count
        position = (count[supported] - 1) * (self.q / 100)
        lower_index = np.floor(position).astype(np.int64)
        upper_index = np.ceil(position).astype(np.int64)
        lower = ordered[starts[supported] + lower_index]
        upper = ordered[starts[supported] + upper_index]
        values = np.full(count.shape, np.nan)
        values[supported] = lower + (upper - lower) * (position - lower_index)
        return Grid(binned.origin, self.cell_m, binned.rows(values), cloud)
