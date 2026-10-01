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

import math

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid, Path
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.mapping.occupancy.path_mask import make_path_mask
from dimos.mapping.occupancy.path_resampling import simple_resample_path, smooth_resample_path
from dimos.msgs.geometry import quaternion_euler, quaternion_from_euler
from dimos.msgs.occupancy import grid_to_world, world_to_grid
from dimos.navigation.replanning_a_star.min_cost_astar import _USE_CPP, min_cost_astar


@pytest.mark.parametrize("use_cpp", [False, True])
@pytest.mark.parametrize("yaw", [0.0, math.pi / 2])
def test_generated_grid_to_path_and_mask(use_cpp, yaw):
    if use_cpp:
        assert _USE_CPP, "native A* extension must be built for this integration test"
    cells = np.zeros((30, 30), dtype=np.int8)
    cells[15, 0:20] = 100
    header = Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="map")
    grid = OccupancyGrid(
        header=header,
        info=MapMetaData(
            width=30,
            height=30,
            resolution=0.25,
            origin=Pose(position=Point(x=2, y=3), orientation=quaternion_from_euler(0, 0, yaw)),
        ),
        data=cells.ravel(),
    )
    received = OccupancyGrid.decode(grid.encode())
    start = grid_to_world(received, (5, 5))
    goal = grid_to_world(received, (25, 25))
    path = min_cost_astar(received, goal, start, use_cpp=use_cpp)
    assert path is not None
    decoded = Path.decode(path.encode())
    assert decoded.header == header
    assert world_to_grid(received, decoded.poses[0].pose.position) == pytest.approx((5, 5))
    assert world_to_grid(received, decoded.poses[-1].pose.position) == pytest.approx((25, 25))
    for pose in decoded.poses:
        assert pose.header == header
        x, y = world_to_grid(received, pose.pose.position)
        assert cells[round(y), round(x)] == 0
    mask = make_path_mask(received, decoded, 0.1)
    assert mask.any()
    assert not np.any(mask & (cells == 100))
    assert world_to_grid(grid, start) == pytest.approx((5, 5))
    assert world_to_grid(grid, goal) == pytest.approx((25, 25))


@pytest.mark.parametrize("resample", [simple_resample_path, smooth_resample_path])
def test_resampling_preserves_headers_orientations_and_input(resample):
    header = Header(stamp=Time(sec=123, nanosec=456), frame_id="map")
    path = Path(
        header=header,
        poses=[
            PoseStamped(
                header=header,
                pose=Pose(position=Point(x=1, y=y), orientation=quaternion_from_euler(0, 0, 0)),
            )
            for y in [1, 2, 3]
        ],
    )
    before = path.encode()
    goal = Pose(orientation=quaternion_from_euler(0, 0, -math.pi / 2))
    actual = Path.decode(resample(path, goal, 0.1).encode())
    assert actual.header == header
    assert len(actual.poses) > 3
    assert actual.poses[0].pose.position == path.poses[0].pose.position
    assert actual.poses[-1].pose.position == path.poses[-1].pose.position
    assert actual.poses[-1].pose.orientation == goal.orientation
    for pose in list(actual.poses)[:-1]:
        assert pose.header == header
        assert quaternion_euler(pose.pose.orientation)[2] == pytest.approx(math.pi / 2)
    assert path.encode() == before


@pytest.mark.parametrize("use_cpp", [False, True])
def test_outside_start_and_goal_are_rejected(use_cpp):
    grid = OccupancyGrid(
        info=MapMetaData(
            width=3, height=3, resolution=1, origin=Pose(orientation=quaternion_from_euler(0, 0, 0))
        ),
        data=[0] * 9,
    )
    assert min_cost_astar(grid, Point(x=1, y=1), Point(x=-0.1), use_cpp=use_cpp) is None
    assert min_cost_astar(grid, Point(x=-0.1), Point(x=1, y=1), use_cpp=use_cpp) is None
    assert min_cost_astar(grid, Point(x=3), use_cpp=use_cpp) is None
