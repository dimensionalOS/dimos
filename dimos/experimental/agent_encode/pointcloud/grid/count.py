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

"""Returns per cell."""

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
class Count(Query[Grid]):
    """The number of returns in each cell's column; 0 where there are none."""

    cell_m: float
    area: tuple[tuple[float, float], tuple[float, float]] | None = None
    """((x0, y0), (x1, y1)) the cells cover; None covers every finite return."""

    def run(self, cloud: PointCloud2) -> Grid:
        binned = bin_returns(cloud, self.cell_m, self.area)
        return Grid(
            binned.origin, self.cell_m, binned.rows(binned.count().astype(np.float64)), cloud
        )
