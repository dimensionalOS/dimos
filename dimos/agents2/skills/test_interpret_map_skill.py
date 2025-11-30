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

import json
import pickle
from typing import TYPE_CHECKING

import cv2
import numpy as np
import pytest

from dimos.agents2.skills import interpret_map
from dimos.agents2.skills.interpret_map import InterpretMapSkill
from dimos.msgs.geometry_msgs import Quaternion, Vector3

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs import Pose
    from dimos.msgs.nav_msgs import OccupancyGrid
    from dimos.msgs.sensor_msgs import Image


def load_costmap_from_pickle(pickle_path: str):
    try:
        with open(pickle_path, "rb") as f:
            data = pickle.load(f)
        costmap: OccupancyGrid = data["costmap"]
        robot_pose: Pose = data["robot_pose"]
        costmap_image: Image = data["costmap_image"]
        return costmap, robot_pose, costmap_image
    except Exception as e:
        logger.error(f"Failed to load costmap from {pickle_path}: {e!s}")
        raise


def test_get_goal_position(interpret_map_skill):
    # Load test costmap and robot pose
    costmap, robot_pose, costmap_image = load_costmap_from_pickle(
        "tests/mockdata/local_costmap.pkl"
    )
    costmap.robot_pose = robot_pose

    interpret_map_skill._latest_local_costmap = costmap
    interpret_map_skill._robot_pose = robot_pose

    # define a range of values for testing
    # (description, ((x_min, x_max), (y_min, y_max)))
    test_cases = [
        (
            "a clear area near the center of the map",
            (
                (costmap.info.width * 0.25, costmap.info.width * 0.75),
                (costmap.info.height * 0.25, costmap.info.height * 0.75),
            ),
        ),
        (
            "an open space close to the bottom right corner",
            (
                (costmap.info.width * 0.5, costmap.info.width * 1.0),
                (costmap.info.height * 0.0, costmap.info.height * 0.5),
            ),
        ),
    ]

    for description, expected_range in test_cases:
        goal_world = interpret_map_skill.get_goal_position(description=description)
        goal_grid = costmap.world_to_grid(goal_world)
        assert goal_world is not None, (
            f"Goal position should not be None for description: {description}"
        )
        assert expected_range[0][0] <= goal_grid.x <= expected_range[0][1], (
            f"Goal x {goal_grid.x} out of expected range {expected_range[0]} for description: {description}"
        )
        assert expected_range[1][0] <= goal_grid.y <= expected_range[1][1], (
            f"Goal y {goal_grid.y} out of expected range {expected_range[1]} for description: {description}"
        )
