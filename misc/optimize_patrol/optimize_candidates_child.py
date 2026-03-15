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

"""Child process: test a single _candidates_to_consider value.

Runs until target coverage is reached. Outputs JSON:
  {"avg_next_goal_time": float, "distance": float}
"""

import argparse
import json
import math
import time

import numpy as np

from dimos.mapping.occupancy.gradient import gradient
from dimos.mapping.occupancy.path_resampling import smooth_resample_path
from dimos.mapping.pointclouds.occupancy import height_cost_occupancy
from dimos.mapping.pointclouds.util import read_pointcloud
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.patrolling.create_patrol_router import create_patrol_router
from dimos.navigation.patrolling.routers.coverage_patrol_router import CoveragePatrolRouter
from dimos.navigation.patrolling.utilities import point_to_pose_stamped
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.utils.data import get_data

SCORING_STAMP_RADIUS_M = 0.2
CLEARANCE_RADIUS_M = 0.2
MAX_DISTANCE = 50_000.0  # Safety cap to avoid infinite loops.


def _circular_disk(radius_cells: int) -> np.ndarray:
    y, x = np.ogrid[-radius_cells : radius_cells + 1, -radius_cells : radius_cells + 1]
    return (x * x + y * y) <= radius_cells * radius_cells


def _stamp_scoring_map(
    visited: np.ndarray, x: float, y: float, occupancy_grid, radius_cells: int, disk: np.ndarray
) -> None:
    grid_pos = occupancy_grid.world_to_grid((x, y))
    col, row = int(grid_pos.x), int(grid_pos.y)
    h, w = visited.shape
    r = radius_cells
    if row + r < 0 or row - r >= h or col + r < 0 or col - r >= w:
        return
    r_min = max(0, row - r)
    r_max = min(h, row + r + 1)
    c_min = max(0, col - r)
    c_max = min(w, col + r + 1)
    d_r_min = r_min - (row - r)
    d_r_max = d_r_min + (r_max - r_min)
    d_c_min = c_min - (col - r)
    d_c_max = d_c_min + (c_max - c_min)
    visited[r_min:r_max, c_min:c_max] |= disk[d_r_min:d_r_max, d_c_min:d_c_max]


def run_iteration(
    candidates_to_consider: int,
    target_coverage: float,
    occupancy_grid,
    costmap,
    scoring_radius_cells: int,
    scoring_disk: np.ndarray,
) -> tuple[float, float]:
    """Returns (avg_next_goal_time_seconds, distance_traveled)."""
    start = (-1.03, -13.48)

    router = create_patrol_router("coverage", CLEARANCE_RADIUS_M)
    assert isinstance(router, CoveragePatrolRouter)
    router._candidates_to_consider = candidates_to_consider
    router.handle_occupancy_grid(occupancy_grid)
    router.handle_odom(point_to_pose_stamped(start))

    h, w = occupancy_grid.height, occupancy_grid.width
    scoring_visited = np.zeros((h, w), dtype=bool)
    free_mask = occupancy_grid.grid == 0
    total_free = int(np.count_nonzero(free_mask))
    if total_free == 0:
        return 0.0, 0.0

    _stamp_scoring_map(
        scoring_visited, start[0], start[1], occupancy_grid, scoring_radius_cells, scoring_disk
    )

    distance_walked = 0.0
    next_goal_times: list[float] = []

    while distance_walked < MAX_DISTANCE:
        t0 = time.perf_counter()
        goal = router.next_goal()
        next_goal_times.append(time.perf_counter() - t0)

        if goal is None:
            break
        path = min_cost_astar(costmap, goal.position, start, unknown_penalty=1.0, use_cpp=True)
        if path is None:
            continue
        path = smooth_resample_path(path, goal, 0.1)

        for pose in path.poses:
            dx = pose.position.x - start[0]
            dy = pose.position.y - start[1]
            distance_walked += math.sqrt(dx * dx + dy * dy)
            start = (pose.position.x, pose.position.y)

            router.handle_odom(pose)
            _stamp_scoring_map(
                scoring_visited,
                pose.position.x,
                pose.position.y,
                occupancy_grid,
                scoring_radius_cells,
                scoring_disk,
            )

        coverage = int(np.count_nonzero(scoring_visited & free_mask)) / total_free
        if coverage >= target_coverage:
            break

    avg_time = float(np.mean(next_goal_times)) if next_goal_times else 0.0
    return avg_time, distance_walked


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidates", type=int, required=True)
    parser.add_argument("--target_coverage", type=float, default=0.3)
    parser.add_argument("--n_iterations", type=int, default=3)
    args = parser.parse_args()

    data = read_pointcloud(get_data("big_office.ply"))
    cloud = PointCloud2.from_numpy(np.asarray(data.points), frame_id="")
    occupancy_grid = height_cost_occupancy(cloud)
    costmap = gradient(occupancy_grid, max_distance=1.5)

    scoring_radius_cells = int(np.ceil(SCORING_STAMP_RADIUS_M / occupancy_grid.resolution))
    scoring_disk = _circular_disk(scoring_radius_cells)

    avg_times = []
    distances = []
    for _ in range(args.n_iterations):
        avg_t, dist = run_iteration(
            args.candidates,
            args.target_coverage,
            occupancy_grid,
            costmap,
            scoring_radius_cells,
            scoring_disk,
        )
        avg_times.append(avg_t)
        distances.append(dist)

    result = {
        "avg_next_goal_time": float(np.median(avg_times)),
        "distance": float(np.median(distances)),
    }
    print(json.dumps(result))


if __name__ == "__main__":
    main()
