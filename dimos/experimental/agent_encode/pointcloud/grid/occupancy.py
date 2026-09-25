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

"""Which cells hold returns in a height band."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from dimos.experimental.agent_encode.pointcloud.grid.base import Grid
from dimos.experimental.agent_encode.pointcloud.grid.lib.cells import bin_returns
from dimos.experimental.agent_encode.pointcloud.queries.base import Query
from dimos.experimental.agent_encode.pointcloud.queries.select import Select
from dimos.experimental.agent_encode.pointcloud.z_band import ZBand, check_z_band, in_z_band

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass(frozen=True)
class Occupancy(Query[Grid]):
    """1 where a cell holds a return in the band; else 0 where it holds a return below
    it; else NaN. Returns above the band are ignored, so the grid's ``cloud`` holds only
    the returns up to the band's top."""

    z: ZBand
    """The occupied band; with no low end no cell is free."""
    cell_m: float
    area: tuple[tuple[float, float], tuple[float, float]] | None = None
    """((x0, y0), (x1, y1)) the cells cover; None covers every finite return."""

    def __post_init__(self) -> None:
        check_z_band(self.z)

    def run(self, cloud: PointCloud2) -> Grid:
        binned = bin_returns(cloud, self.cell_m, self.area)
        low = self.z[0]
        occupied = binned.any(in_z_band(binned.z, self.z))
        free = binned.any(binned.z < (-np.inf if low is None else low))
        values = np.where(occupied, 1.0, np.where(free, 0.0, np.nan))
        used = Select(z=(None, self.z[1])).run(cloud) if self.z[1] is not None else cloud
        return Grid(binned.origin, self.cell_m, binned.rows(values), used, mask=True)
