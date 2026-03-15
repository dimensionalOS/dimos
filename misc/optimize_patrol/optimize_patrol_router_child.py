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

"""Child process: test a single (saturation_threshold, clearance_radius_m) pair."""

import argparse
import math

import numpy as np

from dimos.mapping.occupancy.gradient import gradient
from dimos.mapping.occupancy.path_resampling import smooth_resample_path
from dimos.mapping.pointclouds.occupancy import height_cost_occupancy
from dimos.mapping.pointclouds.util import read_pointcloud
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.patrolling.create_patrol_router import create_patrol_router
from dimos.navigation.patrolling.routers.visitation_history import VisitationHistory
from dimos.navigation.patrolling.utilities import point_to_pose_stamped
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.utils.data import get_data

SCORING_STAMP_RADIUS_M = 0.2


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
    saturation_threshold: float,
    clearance_radius_m: float,
    total_distance: float,
    occupancy_grid,
    costmap,
    scoring_radius_cells: int,
    scoring_disk: np.ndarray,
) -> float:
    start = (-1.03, -13.48)

    VisitationHistory._saturation_threshold = saturation_threshold
    router = create_patrol_router("coverage", clearance_radius_m)
    router.handle_occupancy_grid(occupancy_grid)
    router.handle_odom(point_to_pose_stamped(start))

    h, w = occupancy_grid.height, occupancy_grid.width
    scoring_visited = np.zeros((h, w), dtype=bool)
    free_mask = occupancy_grid.grid == 0

    _stamp_scoring_map(
        scoring_visited, start[0], start[1], occupancy_grid, scoring_radius_cells, scoring_disk
    )

    distance_walked = 0.0

    while distance_walked < total_distance:
        goal = router.next_goal()
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

            if distance_walked >= total_distance:
                break

    total_free = int(np.count_nonzero(free_mask))
    if total_free == 0:
        return 0.0
    return int(np.count_nonzero(scoring_visited & free_mask)) / total_free


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--saturation_threshold", type=float, required=True)
    parser.add_argument("--clearance_radius_m", type=float, required=True)
    parser.add_argument("--n_iterations", type=int, default=5)
    parser.add_argument("--total_distance", type=float, default=100.0)
    args = parser.parse_args()

    data = read_pointcloud(get_data("big_office.ply"))
    cloud = PointCloud2.from_numpy(np.asarray(data.points), frame_id="")
    occupancy_grid = height_cost_occupancy(cloud)
    costmap = gradient(occupancy_grid, max_distance=1.5)

    scoring_radius_cells = int(np.ceil(SCORING_STAMP_RADIUS_M / occupancy_grid.resolution))
    scoring_disk = _circular_disk(scoring_radius_cells)

    scores = []
    for _ in range(args.n_iterations):
        score = run_iteration(
            args.saturation_threshold,
            args.clearance_radius_m,
            args.total_distance,
            occupancy_grid,
            costmap,
            scoring_radius_cells,
            scoring_disk,
        )
        scores.append(score)

    median = float(np.median(scores))
    print(median)


if __name__ == "__main__":
    main()
