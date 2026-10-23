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
import math

from dimos_cdrgen.registry import decode, encode
import numpy as np
import pytest

from dimos.message_utils.occupancy import grid_to_world, occupancy_from_array, world_to_grid
from dimos.message_utils.time import time_from_nanoseconds
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos.messages.nav_msgs.msg import Path
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped as LegacyStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid as LegacyGrid
from dimos.msgs.nav_msgs.Path import Path as LegacyPath
from dimos.navigation.go2.replanning_a_star.goal_validator import find_safe_goal
from dimos.navigation.go2.replanning_a_star.path_clearance import PathClearance
from dimos.navigation.go2.replanning_a_star.path_distancer import PathDistancer


def grid_from_cells(cells, rotated=False):
    return occupancy_from_array(
        cells,
        header=Header(frame_id="map", stamp=time_from_nanoseconds(-1)),
        resolution=0.5,
        origin=Pose(
            position=Point(x=0, y=0, z=0),
            orientation=Quaternion(
                x=0,
                y=0,
                z=math.sin(math.pi / 4) if rotated else 0,
                w=math.cos(math.pi / 4) if rotated else 1,
            ),
        ),
    )


@pytest.mark.parametrize("algorithm", ["bfs", "bfs_contiguous"])
@pytest.mark.parametrize("rotated", [False, True])
@pytest.mark.parametrize("clearance", [0, 0.5])
def test_generated_safe_goal_preserves_existing_search_choices_and_rotated_coordinates(
    algorithm, rotated, clearance
):
    cells = np.zeros((12, 12), dtype=np.int8)
    cells[5:7, 5:7] = 100
    grid = grid_from_cells(cells, rotated)
    grid = decode(encode(grid), type(grid))
    before = encode(grid)
    point = grid_to_world(grid, (5.25, 5.25))
    expected = find_safe_goal(
        LegacyGrid(cells, resolution=0.5),
        (5.25 * 0.5, 5.25 * 0.5),
        algorithm=algorithm,
        min_clearance=clearance,
        connectivity_check_radius=0,
    )

    result = find_safe_goal(
        grid, point, algorithm=algorithm, min_clearance=clearance, connectivity_check_radius=0
    )

    assert type(result) is Point
    assert expected is not None
    coordinate = world_to_grid(grid, result)
    np.testing.assert_allclose(coordinate, (expected.x / 0.5, expected.y / 0.5), atol=1e-12, rtol=0)
    assert encode(grid) == before
    assert decode(encode(result), Point) == result


@pytest.mark.parametrize("algorithm", ["bfs", "bfs_contiguous"])
def test_generated_safe_goal_returns_none_when_no_safe_cell_exists(algorithm):
    grid = grid_from_cells(np.full((7, 7), 100, dtype=np.int8))
    assert (
        find_safe_goal(grid, Point(x=1, y=1, z=0), algorithm=algorithm, connectivity_check_radius=0)
        is None
    )


@pytest.mark.parametrize("algorithm", ["bfs", "bfs_contiguous"])
def test_generated_safe_goal_rejects_unknown_cells(algorithm):
    grid = grid_from_cells(np.full((7, 7), -1, dtype=np.int8))
    assert find_safe_goal(grid, (1, 1), algorithm=algorithm, connectivity_check_radius=0) is None


def make_path():
    header = Header(frame_id="map", stamp=time_from_nanoseconds(1700000000123456789))
    return Path(
        header=deepcopy(header),
        poses=[
            PoseStamped(
                header=deepcopy(header),
                pose=Pose(
                    position=Point(x=x, y=1, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1)
                ),
            )
            for x in [1.0, 2.0, 3.0, 4.0, 5.0]
        ],
    )


@pytest.mark.parametrize("index,expected", [(0, (1.5, 1)), (1, (2.5, 1)), (4, (5.0, 1))])
def test_generated_path_distances_and_lookahead_match_existing_geometry(index, expected):
    path = make_path()
    original = deepcopy(path)
    helper = PathDistancer(path)
    old = PathDistancer(
        LegacyPath(
            poses=[
                LegacyStamped(position=[p.pose.position.x, p.pose.position.y, 0])
                for p in path.poses
            ]
        )
    )
    np.testing.assert_array_equal(helper.find_lookahead_point(index), expected)
    np.testing.assert_array_equal(
        helper.find_lookahead_point(index), old.find_lookahead_point(index)
    )
    assert helper.find_closest_point_index(np.array([2.1, 1.0])) == 1
    assert helper.get_distance_to_path(np.array([2.0, 2.0])) == 1
    assert helper.distance_to_goal(np.array([2.0, 1.0])) == 3
    assert path == original


def test_generated_path_clearance_detects_new_obstacle_using_retained_path_mask():
    path = make_path()
    helper = PathClearance(0.2, path)
    assert helper.is_obstacle_ahead()
    cells = np.zeros((15, 15), dtype=np.int8)
    clear = grid_from_cells(cells)
    helper.update_costmap(clear)
    assert not helper.is_obstacle_ahead()
    cells[2, 4] = 100
    helper.update_costmap(grid_from_cells(cells))
    assert helper.is_obstacle_ahead()
    helper.update_pose_index(3)
    assert not helper.is_obstacle_ahead()
    assert clear.header.stamp == time_from_nanoseconds(-1)


def test_contiguous_generated_search_cannot_cross_an_enclosing_obstacle():
    cells = np.zeros((12, 12), dtype=np.int8)
    cells[4:8, 4:8] = 100
    grid = grid_from_cells(cells)
    goal = grid_to_world(grid, (5.25, 5.25))
    assert (
        find_safe_goal(
            grid, goal, algorithm="bfs_contiguous", min_clearance=0, connectivity_check_radius=0
        )
        is None
    )
    assert (
        find_safe_goal(grid, goal, algorithm="bfs", min_clearance=0, connectivity_check_radius=0)
        is not None
    )
