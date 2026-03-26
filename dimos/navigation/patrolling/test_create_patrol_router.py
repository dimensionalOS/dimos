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


import os

import cv2
import numpy as np
import pytest

from dimos.mapping.occupancy.gradient import gradient
from dimos.mapping.occupancy.path_resampling import smooth_resample_path
from dimos.mapping.occupancy.visualizations import visualize_occupancy_grid
from dimos.mapping.pointclouds.occupancy import height_cost_occupancy
from dimos.mapping.pointclouds.util import read_pointcloud
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.patrolling.create_patrol_router import create_patrol_router
from dimos.navigation.patrolling.utilities import point_to_pose_stamped
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.utils.data import get_data


@pytest.fixture
def big_office() -> OccupancyGrid:
    data = read_pointcloud(get_data("big_office.ply"))
    cloud = PointCloud2.from_numpy(np.asarray(data.points), frame_id="")
    return height_cost_occupancy(cloud)


@pytest.mark.slow
@pytest.mark.parametrize(
    "router_name, saturation", [("random", 0.20), ("coverage", 0.30), ("frontier", 0.20)]
)
def test_patrolling_coverage(router_name, saturation, big_office) -> None:
    start = (-1.03, -13.48)
    robot_width = 0.4
    multiplier = 1.5
    big_office_gradient = gradient(big_office, max_distance=1.5)
    router = create_patrol_router(router_name, robot_width * multiplier)
    router.handle_occupancy_grid(big_office)
    router.handle_odom(point_to_pose_stamped(start))

    all_poses: list = []
    for _ in range(15):
        goal = router.next_goal()
        if goal is None:
            continue
        path = min_cost_astar(
            big_office_gradient, goal.position, start, unknown_penalty=1.0, use_cpp=True
        )
        if path is None:
            continue
        path = smooth_resample_path(path, goal, 0.1)
        for pose in path.poses:
            router.handle_odom(pose)
            all_poses.append(pose)
        start = (path.poses[-1].position.x, path.poses[-1].position.y)

    assert router.get_saturation() > saturation

    if os.environ.get("DEBUG"):
        _save_coverage_image(router_name, router, all_poses, big_office, big_office_gradient)


def _save_coverage_image(router_name, router, all_poses, big_office, big_office_gradient) -> None:
    image = visualize_occupancy_grid(big_office_gradient, "rainbow")
    h, w = image.data.shape[:2]
    visit_counts = np.zeros((h, w), dtype=np.float32)
    radius = int(np.ceil(router._clearance_radius_m / big_office.resolution))
    stamp = np.zeros((h, w), dtype=np.uint8)

    for pose in all_poses:
        grid = big_office.world_to_grid((pose.position.x, pose.position.y))
        gx, gy = int(grid.x), int(grid.y)
        if 0 <= gy < h and 0 <= gx < w:
            stamp[:] = 0
            cv2.circle(stamp, (gx, gy), radius, 1, -1)
            visit_counts += stamp

    alpha = 0.05
    mask = visit_counts > 0
    blend = 1.0 - (1.0 - alpha) ** visit_counts

    overlay = image.data.astype(np.float32) * 0.24
    for c in range(3):
        overlay[:, :, c][mask] = overlay[:, :, c][mask] * (1.0 - blend[mask]) + 255.0 * blend[mask]

    image.data = overlay.astype(np.uint8)
    image.save(f"patrolling_coverage_{router_name}.png")
