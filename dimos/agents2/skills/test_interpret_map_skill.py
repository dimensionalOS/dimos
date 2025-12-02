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
import re
from typing import TYPE_CHECKING

import cv2
import numpy as np
import pytest

from dimos.agents2.skills import interpret_map
from dimos.agents2.skills.interpret_map import InterpretMapSkill
from dimos.models.vl.qwen import QwenVlModel
from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import Image
from dimos.utils.data import get_data
from dimos.utils.generic import extract_json_from_llm_response


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
    mockdata_path = get_data("maps") / "mockdata_local_costmap.pkl"
    # Load test costmap and robot pose
    costmap, robot_pose, costmap_image = load_costmap_from_pickle(str(mockdata_path))
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


def build_occupancygrid_from_image(image: Image, resolution: float = 0.05) -> "OccupancyGrid":
    image_arr = image.to_rgb().data
    height, width = image_arr.shape[:2]
    grid = np.full((height, width), -1, dtype=np.int8)  # Unknown by default

    # drop alpha channel if present
    if image_arr.shape[2] == 4:
        image_arr = image_arr[:, :, :3]

    # Define colors and threshold
    RED = np.array([255, 0, 0])
    BLUE = np.array([0, 0, 200])
    color_threshold = 20

    for y in range(height):
        for x in range(width):
            pixel = image_arr[y, x]

            # calculate distances to target colors
            red_dist = np.sqrt(np.sum((pixel.astype(np.float32) - RED) ** 2))
            blue_dist = np.sqrt(np.sum((pixel.astype(np.float32) - BLUE) ** 2))

            # assign based on closest color within threshold
            if red_dist <= color_threshold:
                grid[y, x] = 100  # Obstacle
            elif blue_dist <= color_threshold:
                grid[y, x] = 0  # Free space
            # else: remains -1 (unknown)

    occupancy_grid = OccupancyGrid()
    occupancy_grid.info.width = width
    occupancy_grid.info.height = height
    occupancy_grid.info.resolution = resolution
    occupancy_grid.grid = grid
    occupancy_grid.info.origin.position = Vector3(0.0, 0.0, 0.0)
    occupancy_grid.info.origin.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    return occupancy_grid


@pytest.fixture
def occupancy_grid_from_image() -> OccupancyGrid:
    image_path = get_data("maps") / "floorplan.png"
    image = Image.from_file(str(image_path))
    occupancy_grid = build_occupancygrid_from_image(image, resolution=0.05)
    return occupancy_grid


def test_goal_placement(occupancy_grid_from_image: OccupancyGrid):
    vl_model = QwenVlModel()
    occupancy_grid = occupancy_grid_from_image

    # set robot pose for testing
    occupancy_grid.robot_pose = Pose(
        position=Vector3(51.0, 15.0, 0.0),
        orientation=Quaternion(0.0, 0.0, 1.0, 0.0),
    )

    image = occupancy_grid.grid_to_image(flip_vertical=False)
    qwen_image = image.to_bgr().to_opencv()

    questions = {
        "furthest room from the robot's current position": (
            (274, 484),
            (33, 266),
        )
    }

    for question, expected_range in questions.items():
        base_prompt = (
            "Look at this image carefully \n"
            "it represents a 2D occupancy grid map where,\n"
            " - blue area is free space, \n"
            " - yellow area is unknown space, \n"
            " - red (and its shades) areas are obstacles/walls, \n"
            " - green object represents the robot's position and points to the direction it is facing. \n"
            f"Identify a goal position ONLY IN FREE SPACE that closely matches the following description: {question}\n"
            "Prioritize selecting a goal position in free space (blue area) over exactly matching the description. \n"
            "MAKE SURE there is a clear path from the robot's current position to the goal position without crossing any obstacles. \n"
            "MAKE SURE the goal position is located in the blue area (free space) of the map and few pixels away from obstacles or objects. \n"
            "Return ONLY a JSON object with this exact format:\n"
            "{'point': [x, y]}\n"
            "where x,y are the pixel coordinates of the goal position in the image. \n"
        )

        prompt = base_prompt
        response = vl_model.query(qwen_image, prompt)
        point = extract_json_from_llm_response(response)
        x, y = point["point"]

        # debug_image_with_identified_point(
        #     qwen_image,
        #     (x, y),
        #     filepath=f"./debug_goal_placement_{question.replace(' ', '_')}.png",
        # )

        assert expected_range[0][0] <= x <= expected_range[0][1], (
            f"Goal x {x} out of expected range {expected_range[0]} for question: {question}"
        )
        assert expected_range[1][0] <= y <= expected_range[1][1], (
            f"Goal y {y} out of expected range {expected_range[1]} for question: {question}"
        )


def test_map_interpretability(occupancy_grid_from_image: OccupancyGrid):
    vl_model = QwenVlModel()
    occupancy_grid = occupancy_grid_from_image

    # set robot pose for testing
    occupancy_grid.robot_pose = Pose(
        position=Vector3(51.0, 15.0, 0.0),
        orientation=Quaternion(0.0, 0.0, 1.0, 0.0),
    )

    image = occupancy_grid.grid_to_image(flip_vertical=False)
    qwen_image = image.to_bgr().to_opencv()

    base_prompt = (
        "Look at this image carefully \n"
        "it represents a 2D occupancy grid map where,\n"
        " - blue area is free space, \n"
        " - yellow area is unknown space, \n"
        " - red (and its shades) areas are obstacles/walls, \n"
        " - green object represents the robot's position and points to the direction it is facing. \n"
        "Answer the following question based on this image: \n"
    )

    questions = {
        # basic questions about the robot
        "is there any open area for the robot to move behind itself, yes or no?": r"\b(no)\b",
        "what direction is the robot facing in the map?": r"\b(left|west)\b",
        "describe the layout of the environment shown in the map.": r".+",
        # spatial understanding questions
        "how many rooms can you identify in the map?": r"\b(3|three)\b",
        "do all rooms have an entrance or doorway?": r"\b(yes)\b",
        "how many doorways are there in the map?": r"\b(3|three)\b",
        "how many corridors can you see in the map?": r"\b(2|two)\b",
        "where is the furthest room from the robot's current position?": r"\b(top left|left)\b",
        "which is the largest room in the map?": r"\b(top right|right)\b",
        "is the robot currently in a doorway, room or a corridor?": r"\b(corridor|hallway|passage)\b",
        "what is down the corridor in front of the robot?": r"\b(unknown)\b",
        # spatial understanding + navigation
        "if the robot wants to go to the top left room, what command should it send? available commands are: 'go forward', 'turn left', 'turn right', 'stop'": r"\b(|go forward|turn right|stop)\b",
    }

    # query and score responses
    responses = {}
    score = 0
    for question in questions.keys():
        prompt = base_prompt + question
        response = vl_model.query(qwen_image, prompt)
        responses[question] = response
        if re.search(questions[question], response, re.IGNORECASE):
            score += 1
        else:
            print(f"Q: {question}\nA: {response}\n")

    print(f"Map interpretability score: {score}/{len(questions)}")
    assert score >= len(questions) * 0.7, (
        f"Map interpretability score too low: {score}/{len(questions)}. Responses: {responses}"
    )


def debug_image_with_identified_point(image_frame, point: tuple[int, int], filepath: str) -> None:
    """Utility to visualize identified points on the image for debugging."""
    debug_image = image_frame.copy()
    x, y = point
    cv2.drawMarker(debug_image, (x, y), (255, 255, 255), cv2.MARKER_CROSS, 15, 2)
    cv2.imwrite(filepath, debug_image)
