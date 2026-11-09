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
import numpy as np
import pytest

from dimos.mapping.occupancy.gradient import gradient
from dimos.mapping.occupancy.path_resampling import simple_resample_path, smooth_resample_path
from dimos.mapping.occupancy.visualize_path import visualize_path
from dimos.message_utils import (
    image,
    image as image_utils,
    occupancy as occupancy_utils,
    time as message_time,
)
from dimos.messages.geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from dimos.messages.nav_msgs.msg import OccupancyGrid
from dimos.messages.std_msgs.msg import Header
from dimos.navigation.go2.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.utils.data import get_data


@pytest.fixture
def costmap() -> OccupancyGrid:
    return gradient(
        occupancy_utils.from_array(
            np.asarray(np.load(get_data("occupancy_simple.npy")), dtype=np.int8),
            header=Header(stamp=message_time.time_from_seconds(0.0), frame_id=""),
            resolution=0.05,
            origin=None,
        ),
        max_distance=1.5,
    )


@pytest.mark.parametrize("method", ["simple", "smooth"])
def test_resample_path(costmap, method) -> None:
    start = Vector3(4.0, 2.0, 0)
    goal_pose = Pose(position=Point(6.15, 10.0, 0), orientation=Quaternion(0, 0, 0, 1))
    expected = image.from_array(
        cv2.imread(str(get_data(f"resample_path_{method}.png"))),
        encoding="bgr8",
        header=message_time.header_now(),
    )
    path = min_cost_astar(costmap, goal_pose.position, start, use_cpp=False)

    match method:
        case "simple":
            resampled = simple_resample_path(path, goal_pose, 0.1)
        case "smooth":
            resampled = smooth_resample_path(path, goal_pose, 0.1)
        case _:
            raise ValueError(f"Unknown resampling method: {method}")

    actual = visualize_path(costmap, resampled, 0.2, 0.4)
    np.testing.assert_array_equal(image_utils.view(actual), image_utils.view(expected))
