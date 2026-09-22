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

import math

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import GeneralOccupancyConfig
from dimos.msgs.geometry import quaternion_from_euler
from dimos.msgs.occupancy import occupancy_view
from dimos.msgs.pointcloud import pointcloud_from_xyz


def test_cost_mapper_generates_cdr_grid_with_exact_header(request):
    mapper = CostMapper(algo="general", config=GeneralOccupancyConfig(resolution=0.25))
    request.addfinalizer(mapper.stop)
    header = Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="map")
    cloud = pointcloud_from_xyz(np.array([[0, 0, 0], [1, 1, 1]]), header=header)
    grid = mapper._calculate_costmap(cloud)
    decoded = OccupancyGrid.decode(grid.encode())
    assert decoded.header == header
    assert np.any(occupancy_view(decoded) == 100)
    assert np.any(occupancy_view(decoded) == 0)


def test_initial_safe_radius_respects_rotated_origin_and_replaces_readonly_data(request):
    mapper = CostMapper(initial_safe_radius_meters=0.1)
    request.addfinalizer(mapper.stop)
    grid = OccupancyGrid(
        info=MapMetaData(
            width=3,
            height=3,
            resolution=1,
            origin=Pose(
                position=Point(x=1, y=-1), orientation=quaternion_from_euler(0, 0, math.pi / 2)
            ),
        ),
        data=[100] * 9,
    )
    mapper._apply_initial_safe_radius(grid)
    expected = np.full((3, 3), 100, dtype=np.int8)
    expected[1, 1] = 0
    np.testing.assert_array_equal(occupancy_view(grid), expected)
