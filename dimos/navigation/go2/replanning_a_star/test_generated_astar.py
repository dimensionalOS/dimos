# Copyright 2025-2026 Dimensional Inc.
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

from copy import deepcopy
from itertools import pairwise
import math

from dimos_cdrgen.registry import decode, encode
import numpy as np
import pytest

from dimos.mapping.occupancy.path_map import make_navigation_map
from dimos.messages.geometry_msgs.msg import Point, Pose, Quaternion
from dimos.messages.nav_msgs.msg import Path
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid as LegacyGrid
from dimos.msgs.occupancy import grid_to_world, occupancy_from_array, occupancy_view, world_to_grid
from dimos.msgs.time import time_from_nanoseconds
from dimos.navigation.go2.replanning_a_star.min_cost_astar import _USE_CPP, min_cost_astar


@pytest.mark.parametrize("use_cpp", [False, True])
@pytest.mark.parametrize("stamp", [0, -1, 1700000000123456789])
@pytest.mark.parametrize("rotated", [False, True])
def test_generated_astar_routes_around_obstacles_and_retains_exact_metadata(
    use_cpp, stamp, rotated
):
    if use_cpp:
        assert _USE_CPP, "Acceptance requires the actual compiled C++ extension"
    cells = np.zeros((6, 7), dtype=np.int8)
    cells[:5, 3] = 100
    origin = Pose(
        position=Point(x=10, y=-20, z=0),
        orientation=Quaternion(
            x=0,
            y=0,
            z=math.sin(math.pi / 4) if rotated else 0,
            w=math.cos(math.pi / 4) if rotated else 1,
        ),
    )
    grid = occupancy_from_array(
        cells,
        header=Header(frame_id="map", stamp=time_from_nanoseconds(stamp)),
        resolution=0.5,
        origin=origin,
    )
    grid = decode(encode(grid), type(grid))
    before = deepcopy(grid)
    start = grid_to_world(grid, (1.25, 1.25))
    goal = grid_to_world(grid, (5.25, 1.25))

    result = min_cost_astar(grid, goal, start, use_cpp=use_cpp)

    assert type(result) is Path
    assert result.header == grid.header
    assert all(p.header == grid.header and p.pose.orientation.w == 1 for p in result.poses)
    coordinates = [
        tuple(round(v) for v in world_to_grid(grid, p.pose.position)) for p in result.poses
    ]
    assert coordinates[0] == (1, 1)
    assert coordinates[-1] == (5, 1)
    assert (3, 5) in coordinates
    assert all(cells[y, x] != 100 for x, y in coordinates)
    assert all(max(abs(a[0] - b[0]), abs(a[1] - b[1])) == 1 for a, b in pairwise(coordinates))
    assert decode(encode(result), Path) == result
    result.header.frame_id = "changed"
    result.poses[0].header.frame_id = "changed"
    assert result.poses[-1].header == before.header
    assert grid.header == before.header and grid.info == before.info
    np.testing.assert_array_equal(grid.data, before.data)


@pytest.mark.parametrize("use_cpp", [False, True])
def test_generated_astar_unknown_penalty_and_unreachable_outside_goal(use_cpp):
    cells = np.array([[0, -1, 0]], dtype=np.int8)
    grid = occupancy_from_array(
        cells, header=Header(frame_id="map", stamp=time_from_nanoseconds(0)), resolution=1
    )
    assert min_cost_astar(grid, (2, 0), unknown_penalty=1, use_cpp=use_cpp) is None
    path = min_cost_astar(grid, (2, 0), unknown_penalty=0.1, use_cpp=use_cpp)
    assert path is not None
    assert [p.pose.position.x for p in path.poses] == [0, 1, 2]
    assert min_cost_astar(grid, (3, 0), use_cpp=use_cpp) is None


@pytest.mark.parametrize("use_cpp", [False, True])
def test_generated_and_existing_astar_share_identical_cell_route(use_cpp):
    cells = np.zeros((7, 7), dtype=np.int8)
    cells[:5, 3] = 100
    generated = occupancy_from_array(
        cells, header=Header(frame_id="map", stamp=time_from_nanoseconds(-1)), resolution=0.5
    )
    legacy = LegacyGrid(cells, resolution=0.5, frame_id="map", ts=0)
    old = min_cost_astar(legacy, (2.5, 0.5), (0.5, 0.5), use_cpp=use_cpp)
    new = min_cost_astar(generated, (2.5, 0.5), (0.5, 0.5), use_cpp=use_cpp)
    assert old is not None and new is not None
    assert [(p.x, p.y, p.z) for p in old.poses] == [
        (p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in new.poses
    ]


@pytest.mark.parametrize("strategy", ["simple", "mixed"])
@pytest.mark.parametrize("gradient_strategy", ["gradient", "voronoi"])
def test_navigation_costmap_generation_matches_existing_math_and_preserves_metadata(
    strategy, gradient_strategy
):
    cells = np.zeros((10, 10), dtype=np.int8)
    cells[4:7, 4:7] = 100
    cells[0, :] = -1
    generated = occupancy_from_array(
        cells, header=Header(frame_id="map", stamp=time_from_nanoseconds(-1)), resolution=0.5
    )
    generated.info.map_load_time = time_from_nanoseconds(17)
    old = make_navigation_map(LegacyGrid(cells, resolution=0.5), 0.5, strategy, gradient_strategy)
    new = make_navigation_map(generated, 0.5, strategy, gradient_strategy)
    assert new.header == generated.header and new.info == generated.info
    np.testing.assert_array_equal(occupancy_view(new), old.grid)
    np.testing.assert_array_equal(occupancy_view(generated), cells)
