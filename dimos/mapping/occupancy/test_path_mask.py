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
from dimos_generated.geometry_msgs.msg import Point, Pose, Quaternion
import numpy as np
import pytest

from dimos.mapping.occupancy.path_mask import make_path_mask
from dimos.mapping.occupancy.path_resampling import smooth_resample_path
from dimos.mapping.occupancy.visualizations import visualize_occupancy_grid
from dimos.msgs.image import image_view
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.utils.data import get_data


@pytest.mark.parametrize(
    "pose_index,max_length,expected_image",
    [
        (0, float("inf"), "make_path_mask_full.png"),
        (50, 2, "make_path_mask_two_meters.png"),
    ],
)
def test_make_path_mask(occupancy_gradient, pose_index, max_length, expected_image) -> None:
    start = Point(x=4, y=2)
    goal_pose = Pose(position=Point(x=6.15, y=10), orientation=Quaternion(w=1))
    expected = cv2.imread(str(get_data(expected_image)), cv2.IMREAD_COLOR)
    path = min_cost_astar(occupancy_gradient, goal_pose.position, start, use_cpp=False)
    path = smooth_resample_path(path, goal_pose, 0.1)
    robot_width = 0.4
    path_mask = make_path_mask(occupancy_gradient, path, robot_width, pose_index, max_length)
    actual = visualize_occupancy_grid(occupancy_gradient, "rainbow")

    pixels = image_view(actual).copy()
    pixels[path_mask] = [0, 100, 0]

    np.testing.assert_array_equal(pixels, expected)
