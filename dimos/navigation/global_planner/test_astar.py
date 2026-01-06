# Copyright 2025 Dimensional Inc.
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
from open3d.geometry import PointCloud  # type: ignore[import-untyped]
import pytest

from dimos.mapping.occupancy.gradient import gradient, voronoi_gradient
from dimos.mapping.occupancy.visualizations import visualize_occupancy_grid
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.global_planner.astar import astar
from dimos.utils.data import get_data


@pytest.fixture
def costmap() -> PointCloud:
    return gradient(OccupancyGrid(np.load(get_data("occupancy_simple.npy"))), max_distance=1.5)


@pytest.fixture
def costmap_three_paths() -> PointCloud:
    return voronoi_gradient(OccupancyGrid(np.load(get_data("three_paths.npy"))), max_distance=1.5)


@pytest.mark.parametrize(
    "mode,expected_image",
    [
        ("general", "astar_general.png"),
        ("min_cost", "astar_min_cost.png"),
    ],
)
def test_astar(costmap, mode, expected_image) -> None:
    start = Vector3(4.0, 2.0)
    goal = Vector3(6.15, 10.0)
    expected = Image.from_file(get_data(expected_image))

    path = astar(mode, costmap, goal, start, use_cpp=False)
    actual = visualize_occupancy_grid(costmap, "rainbow", path)

    np.testing.assert_array_equal(actual.data, expected.data)


@pytest.mark.parametrize(
    "mode,expected_image",
    [
        ("general", "astar_corner_general.png"),
        ("min_cost", "astar_corner_min_cost.png"),
    ],
)
def test_astar_corner(costmap_three_paths, mode, expected_image) -> None:
    start = Vector3(2.8, 3.35)
    goal = Vector3(6.35, 4.25)
    expected = Image.from_file(get_data(expected_image))

    path = astar(mode, costmap_three_paths, goal, start, use_cpp=False)
    actual = visualize_occupancy_grid(costmap_three_paths, "rainbow", path)

    np.testing.assert_array_equal(actual.data, expected.data)


def test_astar_python_and_cpp(costmap) -> None:
    start = Vector3(4.0, 2.0, 0)
    goal = Vector3(6.15, 10.0)

    start_time = time.perf_counter()
    path_python = astar("min_cost", costmap, goal, start, use_cpp=False)
    elapsed_time_python = time.perf_counter() - start_time
    print(f"\nastar Python took {elapsed_time_python:.6f} seconds")
    assert path_python is not None
    assert len(path_python.poses) > 0

    start_time = time.perf_counter()
    path_cpp = astar("min_cost", costmap, goal, start, use_cpp=True)
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
