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
from dimos.experimental.agent_encode.pointcloud.queries.lib.spacing import point_spacing

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass(frozen=True)
class Spacing(Query[float]):
    """The typical gap between neighbouring returns in metres: the median nearest-neighbour
    distance. A cloud without returns is spaced like a lone return."""

    def run(self, cloud: PointCloud2) -> float:
        points = finite_points(cloud)
        lone = np.zeros((1, 3), dtype=np.float32)
        return float(np.median(point_spacing(points if len(points) else lone)))
