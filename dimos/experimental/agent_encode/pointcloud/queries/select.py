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

from dimos.experimental.agent_encode.pointcloud.queries.base import Query
from dimos.experimental.agent_encode.pointcloud.queries.lib.points import as_cloud, finite_points
from dimos.experimental.agent_encode.pointcloud.shapes.base import Shape
from dimos.experimental.agent_encode.pointcloud.z_band import ZBand, check_z_band, in_z_band

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass(frozen=True)
class Select(Query["PointCloud2"]):
    """The finite returns within the z band, inside every ``inside`` shape and outside every
    ``outside`` shape."""

    z: ZBand = (None, None)
    """Keep only returns in this band."""
    inside: Shape | tuple[Shape, ...] = ()
    """Keep only returns inside each of these."""
    outside: Shape | tuple[Shape, ...] = ()
    """Drop returns inside any of these."""

    def __post_init__(self) -> None:
        check_z_band(self.z)

    def run(self, cloud: PointCloud2) -> PointCloud2:
        points = finite_points(cloud)
        keep = in_z_band(points[:, 2], self.z)
        for shape in (self.inside,) if isinstance(self.inside, Shape) else self.inside:
            keep &= shape.contains(points)
        for shape in (self.outside,) if isinstance(self.outside, Shape) else self.outside:
            keep &= ~shape.contains(points)
        return as_cloud(points[keep], cloud)
