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

import time

import cv2
from dimos_generated.geometry_msgs.msg import Point, Pose, Quaternion
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid
import numpy as np
import pytest

from dimos.mapping.occupancy.gradient import gradient, voronoi_gradient
from dimos.mapping.occupancy.visualizations import visualize_occupancy_grid
from dimos.msgs.image import image_view
from dimos.msgs.occupancy import grid_to_world, world_to_grid
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.utils.data import get_data


def _grid(cells, resolution=0.05):
    return OccupancyGrid(
        info=MapMetaData(
            width=cells.shape[1],
            height=cells.shape[0],
            resolution=resolution,
            origin=Pose(orientation=Quaternion(w=1)),
        ),
        data=cells.astype(np.int8).ravel(),
    )


@pytest.fixture
def costmap() -> OccupancyGrid:
    return gradient(_grid(np.load(get_data("occupancy_simple.npy"))), max_distance=1.5)


@pytest.fixture
def costmap_three_paths() -> OccupancyGrid:
    return voronoi_gradient(_grid(np.load(get_data("three_paths.npy"))), max_distance=1.5)


def test_astar(costmap) -> None:
    start = Point(x=4, y=2)
    goal = Point(x=6.15, y=10)
    expected = cv2.imread(str(get_data("astar_min_cost.png")), cv2.IMREAD_COLOR)

    path = min_cost_astar(costmap, goal, start, use_cpp=False)
    actual = visualize_occupancy_grid(costmap, "rainbow", path)

    np.testing.assert_array_equal(image_view(actual), expected)


def test_astar_corner(costmap_three_paths) -> None:
    start = Point(x=2.8, y=3.35)
    goal = Point(x=6.35, y=4.25)
    expected = cv2.imread(str(get_data("astar_corner_min_cost.png")), cv2.IMREAD_COLOR)

    path = min_cost_astar(costmap_three_paths, goal, start, use_cpp=False)
    actual = visualize_occupancy_grid(costmap_three_paths, "rainbow", path)

    np.testing.assert_array_equal(image_view(actual), expected)


def test_astar_unknown_penalty_blocks_unknown_cells(costmap) -> None:
    """With unknown_penalty=1.0, unknown cells should be untraversable."""
    # Create a grid with a corridor of free cells and unknown cells surrounding it.
    # Place start and goal such that the shortest path would go through unknown cells
    # but with penalty=1.0 it should either avoid them or return None.
    grid = np.full((100, 100), -1, dtype=np.int8)  # All unknown
    # Carve a U-shaped free corridor: left column, bottom row, right column
    grid[10:90, 10] = 0  # left column
    grid[89, 10:90] = 0  # bottom row
    grid[10:90, 89] = 0  # right column
    og = _grid(grid, resolution=0.1)

    start = grid_to_world(og, (10, 10))
    goal = grid_to_world(og, (89, 10))

    for use_cpp in [False, True]:
        path = min_cost_astar(og, goal, start, unknown_penalty=1.0, use_cpp=use_cpp)
        if path is None:
            # No path through unknown is also acceptable
            continue
        # Verify no path cell lands on an unknown cell
        for pose in path.poses:
            gp = world_to_grid(og, pose.pose.position)
            gx, gy = round(gp[0]), round(gp[1])
            if 0 <= gx < 100 and 0 <= gy < 100:
                assert grid[gy, gx] != -1, (
                    f"Path traverses unknown cell at grid ({gx}, {gy}), use_cpp={use_cpp}"
                )


def test_astar_unknown_penalty_allows_with_low_penalty(costmap) -> None:
    """With unknown_penalty < 1.0, unknown cells should be traversable."""
    grid = np.full((50, 50), -1, dtype=np.int8)  # All unknown
    grid[5, 5] = 0  # start cell free
    grid[45, 45] = 0  # goal cell free
    og = _grid(grid, resolution=0.1)

    start = grid_to_world(og, (5, 5))
    goal = grid_to_world(og, (45, 45))

    for use_cpp in [False, True]:
        path = min_cost_astar(og, goal, start, unknown_penalty=0.5, use_cpp=use_cpp)
        assert path is not None, (
            f"Should find path through unknown with penalty=0.5, use_cpp={use_cpp}"
        )
        assert len(path.poses) > 0


def test_astar_python_and_cpp(costmap) -> None:
    start = Point(x=4, y=2)
    goal = Point(x=6.15, y=10)

    start_time = time.perf_counter()
    path_python = min_cost_astar(costmap, goal, start, use_cpp=False)
    elapsed_time_python = time.perf_counter() - start_time
    print(f"\nastar Python took {elapsed_time_python:.6f} seconds")
    assert path_python is not None
    assert len(path_python.poses) > 0

    start_time = time.perf_counter()
    path_cpp = min_cost_astar(costmap, goal, start, use_cpp=True)
    elapsed_time_cpp = time.perf_counter() - start_time
    print(f"astar C++ took {elapsed_time_cpp:.6f} seconds")
    assert path_cpp is not None
    assert len(path_cpp.poses) > 0

    times_better = elapsed_time_python / elapsed_time_cpp
    print(f"astar C++ is {times_better:.2f} times faster than Python")

    # Assert that both implementations return almost identical points.
    np.testing.assert_allclose(
        [(p.pose.position.x, p.pose.position.y) for p in path_python.poses],
        [(p.pose.position.x, p.pose.position.y) for p in path_cpp.poses],
        atol=0.05001,
    )
