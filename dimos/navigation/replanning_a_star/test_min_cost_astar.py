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

import numpy as np
from open3d.geometry import PointCloud
import pytest

from dimos.mapping.occupancy.gradient import gradient, voronoi_gradient
from dimos.mapping.occupancy.visualizations import visualize_occupancy_grid
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.utils.data import get_data


@pytest.fixture
def costmap() -> PointCloud:
    return gradient(OccupancyGrid(np.load(get_data("occupancy_simple.npy"))), max_distance=1.5)


@pytest.fixture
def costmap_three_paths() -> PointCloud:
    return voronoi_gradient(OccupancyGrid(np.load(get_data("three_paths.npy"))), max_distance=1.5)


def test_astar(costmap) -> None:
    start = Vector3(4.0, 2.0)
    goal = Vector3(6.15, 10.0)
    expected = Image.from_file(get_data("astar_min_cost.png"))

    path = min_cost_astar(costmap, goal, start, use_cpp=False)
    actual = visualize_occupancy_grid(costmap, "rainbow", path)

    np.testing.assert_array_equal(actual.data, expected.data)


def test_astar_corner(costmap_three_paths) -> None:
    start = Vector3(2.8, 3.35)
    goal = Vector3(6.35, 4.25)
    expected = Image.from_file(get_data("astar_corner_min_cost.png"))

    path = min_cost_astar(costmap_three_paths, goal, start, use_cpp=False)
    actual = visualize_occupancy_grid(costmap_three_paths, "rainbow", path)

    np.testing.assert_array_equal(actual.data, expected.data)


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
    og = OccupancyGrid(grid, resolution=0.1)

    start = og.grid_to_world((10, 10))
    goal = og.grid_to_world((89, 10))

    for use_cpp in [False, True]:
        path = min_cost_astar(og, goal, start, unknown_penalty=1.0, use_cpp=use_cpp)
        if path is None:
            # No path through unknown is also acceptable
            continue
        # Verify no path cell lands on an unknown cell
        for pose in path.poses:
            gp = og.world_to_grid((pose.position.x, pose.position.y))
            gx, gy = round(gp.x), round(gp.y)
            if 0 <= gx < 100 and 0 <= gy < 100:
                assert grid[gy, gx] != -1, (
                    f"Path traverses unknown cell at grid ({gx}, {gy}), use_cpp={use_cpp}"
                )


def test_astar_unknown_penalty_allows_with_low_penalty(costmap) -> None:
    """With unknown_penalty < 1.0, unknown cells should be traversable."""
    grid = np.full((50, 50), -1, dtype=np.int8)  # All unknown
    grid[5, 5] = 0  # start cell free
    grid[45, 45] = 0  # goal cell free
    og = OccupancyGrid(grid, resolution=0.1)

    start = og.grid_to_world((5, 5))
    goal = og.grid_to_world((45, 45))

    for use_cpp in [False, True]:
        path = min_cost_astar(og, goal, start, unknown_penalty=0.5, use_cpp=use_cpp)
        assert path is not None, (
            f"Should find path through unknown with penalty=0.5, use_cpp={use_cpp}"
        )
        assert len(path.poses) > 0


def _path_grid_cells(path, costmap: OccupancyGrid) -> list[tuple[int, int]]:
    return [
        (
            int(costmap.world_to_grid(pose.position).x),
            int(costmap.world_to_grid(pose.position).y),
        )
        for pose in path.poses
    ]


@pytest.mark.parametrize("use_cpp", [False, True])
def test_astar_path_length_weight_prevents_low_cost_terminal_hook(use_cpp: bool) -> None:
    """A short direct corridor should beat a slightly cheaper route that turns back at the goal."""
    grid = np.full((6, 4), 100, dtype=np.int8)
    grid[5, 0] = 0
    grid[4, 1] = 18
    grid[3, 1] = 22
    grid[2, 1] = 24
    grid[1, 1] = 27
    grid[0, 1] = 29
    grid[3, 2] = 19
    grid[2, 3] = 21
    grid[1, 2] = 25
    costmap = OccupancyGrid(grid, resolution=1.0)
    start = costmap.grid_to_world((0, 5))
    goal = costmap.grid_to_world((1, 0))

    legacy_path = min_cost_astar(costmap, goal, start, use_cpp=use_cpp)
    assert legacy_path is not None
    assert max(x for x, _ in _path_grid_cells(legacy_path, costmap)) > 1

    weighted_path = min_cost_astar(
        costmap,
        goal,
        start,
        distance_weight=1.0,
        cell_cost_weight=3.0,
        use_cpp=use_cpp,
    )
    assert weighted_path is not None
    assert _path_grid_cells(weighted_path, costmap) == [
        (0, 5),
        (1, 4),
        (1, 3),
        (1, 2),
        (1, 1),
        (1, 0),
    ]


def test_astar_python_and_cpp(costmap) -> None:
    start = Vector3(4.0, 2.0, 0)
    goal = Vector3(6.15, 10.0)

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
        [(p.position.x, p.position.y) for p in path_python.poses],
        [(p.position.x, p.position.y) for p in path_cpp.poses],
        atol=0.05001,
    )
