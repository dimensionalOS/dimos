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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Render a synthetic CDR point cloud as an inflated CDR occupancy map."""

from pathlib import Path

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.nav_msgs.msg import OccupancyGrid
from dimos_generated.sensor_msgs.msg import PointCloud2
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.mapping.pointclouds.occupancy import general_occupancy
from dimos.memory.vis.space.space import Space
from dimos.msgs.occupancy import occupancy_view
from dimos.msgs.pointcloud import pointcloud_from_xyz


def main() -> None:
    x, y = np.meshgrid(np.arange(0, 4, 0.1), np.arange(0, 3, 0.1))
    points = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    points[(points[:, 0] > 1.8) & (points[:, 0] < 2.2), 2] = 1
    header = Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="map")
    cloud = PointCloud2.decode(pointcloud_from_xyz(points, header=header).encode())
    grid = general_occupancy(cloud, resolution=0.1)
    inflated = OccupancyGrid.decode(simple_inflate(grid, 0.2).encode())
    assert inflated.header == header
    before = np.count_nonzero(occupancy_view(grid) == 100)
    after = np.count_nonzero(occupancy_view(inflated) == 100)
    assert after > before
    output = Path("build/message-codegen/demo/evidence/occupancy.svg")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(Space().add(inflated).to_svg())
    print(f"Input: {len(points)} CDR points; stamp=1700000000123456789 ns")
    print(f"Occupied cells: {before} → {after} after 0.2 m inflation")
    print(f"SVG: {output}")
    print("PASS: cloud → occupancy → inflation → CDR → SVG; exact source header retained")


if __name__ == "__main__":
    main()
