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

import cv2
from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, Quaternion
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.mapping.occupancy.gradient import gradient
from dimos.mapping.occupancy.path_resampling import simple_resample_path, smooth_resample_path
from dimos.mapping.occupancy.visualize_path import visualize_path
from dimos.msgs.image import image_view
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.utils.data import get_data


def _grid(cells, resolution=0.05):
    return OccupancyGrid(
        info=MapMetaData(
            width=cells.shape[1],
            height=cells.shape[0],
            resolution=float(np.float32(resolution)),
            origin=Pose(
                orientation=Quaternion(w=1, x=0.0, y=0.0, z=0.0),
                position=Point(x=0.0, y=0.0, z=0.0),
            ),
            map_load_time=Time(sec=0, nanosec=0),
        ),
        data=np.asarray(cells.astype(np.int8).ravel(), dtype=np.int8),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )


@pytest.fixture
def costmap() -> OccupancyGrid:
    return gradient(_grid(np.load(get_data("occupancy_simple.npy"))), max_distance=1.5)


@pytest.mark.parametrize("method", ["simple", "smooth"])
def test_resample_path(costmap, method) -> None:
    start = Point(x=4, y=2, z=0.0)
    goal_pose = Pose(
        position=Point(x=6.15, y=10, z=0.0), orientation=Quaternion(w=1, x=0.0, y=0.0, z=0.0)
    )
    expected = cv2.imread(str(get_data(f"resample_path_{method}.png")), cv2.IMREAD_COLOR)
    path = min_cost_astar(costmap, goal_pose.position, start, use_cpp=False)

    match method:
        case "simple":
            resampled = simple_resample_path(path, goal_pose, 0.1)
        case "smooth":
            resampled = smooth_resample_path(path, goal_pose, 0.1)
        case _:
            raise ValueError(f"Unknown resampling method: {method}")

    actual = visualize_path(costmap, resampled, 0.2, 0.4)
    np.testing.assert_array_equal(image_view(actual), expected)
