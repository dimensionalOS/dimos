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
from pathlib import Path
import pickle
import re
from typing import TYPE_CHECKING

import cv2
import numpy as np
import pytest

from dimos.agents2.skills.interpret_map import OccupancyGridImage
from dimos.models.vl.qwen import QwenVlModel
from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import Image
from dimos.utils.data import get_data
from dimos.utils.generic import extract_json_from_llm_response

TEST_DIR = Path(__file__).parent


def load_test_cases(filepath: str):
    import yaml

    print(f"Loading test cases from {filepath}")
    with open(filepath) as f:
        data = yaml.safe_load(f)
    return data


@pytest.fixture
def vl_model():
    return QwenVlModel()


def build_occupancygrid_from_image(image: Image, resolution: float = 0.05) -> "OccupancyGrid":
    image_arr = image.to_rgb().data
    height, width = image_arr.shape[:2]
    grid = np.full((height, width), 100, dtype=np.int8)  # obstacle by default

    # drop alpha channel if present
    if image_arr.shape[2] == 4:
        image_arr = image_arr[:, :, :3]

    # print(f"top left {image_arr[136:150, 215:250]}")

    # Define colors and threshold
    WHITE = np.array([255, 255, 255], dtype=np.float32)
    GRAY = np.array([127, 127, 127], dtype=np.float32)  # approx RGB for 127 gray
    white_threshold = 30
    gray_threshold = 10

    # Convert to float32 for distance calculations
    image_float = image_arr.astype(np.float32)

    # Calculate distances to target colors using broadcasting
    white_dist = np.sqrt(np.sum((image_float - WHITE) ** 2, axis=2))
    gray_dist = np.sqrt(np.sum((image_float - GRAY) ** 2, axis=2))

    # Assign based on closest color within threshold
    grid[white_dist <= white_threshold] = 0  # Free space
    grid[gray_dist <= gray_threshold] = -1  # Unknown space

    occupancy_grid = OccupancyGrid()
    occupancy_grid.info.width = width
    occupancy_grid.info.height = height
    occupancy_grid.info.resolution = resolution
    occupancy_grid.grid = grid
    occupancy_grid.info.origin.position = Vector3(0.0, 0.0, 0.0)
    occupancy_grid.info.origin.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    return occupancy_grid


def occupancy_grid_from_image(image_path) -> OccupancyGrid:
    image_path = get_data("maps") / image_path
    image = Image.from_file(str(image_path))
    occupancy_grid = build_occupancygrid_from_image(image, resolution=0.05)
    return occupancy_grid


def extract_coordinates(point: dict | None) -> list | str:
    if point is None:
        return "Failed to parse goal position: response is None."
    if "point" not in point:
        return "Failed to parse goal position: missing 'point' key."
    if not isinstance(point["point"], list):
        return "Failed to parse goal position: 'point' is not a list."

    return point["point"]


def goal_placement_prompt(description: str) -> str:
    prompt = (
        "Look at this image carefully \n"
        "it represents a noisy 2D occupancy grid map where,\n"
        " - white area is free space, \n"
        " - gray area is unexplored space, \n"
        " - red areas are obstacles, \n"
        " - green circle represents the robot's position and the attached arrow indicates its orientation. \n"
        f"Identify a location in free space based on the following description: {description}\n"
        "Return ONLY a JSON object with this exact format:\n"
        '{"point": [x, y]}\n'
        f"where x,y are the pixel coordinates of the goal position in the image. \n"
    )

    return prompt


def interpretability_prompt(question: str) -> str:
    prompt = (
        "Look at this image carefully \n"
        "it represents a noisy 2D occupancy grid map where,\n"
        " - white area is free space, \n"
        " - gray area is unknown space, \n"
        " - red areas are obstacles/walls, \n"
        " - green circle represents the robot's position and the attached arrow indicates its orientation. \n"
        f"Answer the following question based on this image: {question}\n"
    )
    return prompt


@pytest.mark.parametrize(
    "test_map",
    [
        test_map
        for test_map in load_test_cases(TEST_DIR / "test_map_interpretability.yaml")[
            "point_placement_tests"
        ]
    ],
)
def test_point_placement(test_map, vl_model):
    occupancy_grid = occupancy_grid_from_image(test_map["image_path"])

    # set robot pose for testing
    robot_pose = Pose(
        position=test_map["robot_pose"]["position"],
        orientation=test_map["robot_pose"]["orientation"],
    )

    og_image = OccupancyGridImage.from_occupancygrid(
        occupancy_grid, flip_vertical=False, robot_pose=robot_pose
    )
    image = og_image.image

    score = 0
    for qna in test_map["questions"]:
        prompt = goal_placement_prompt(qna["query"])
        response = vl_model.query(image, prompt)
        point = extract_json_from_llm_response(response)
        x, y = extract_coordinates(point)

        # keep track of score
        expected_area = qna["expected_range"]
        if (expected_area["x"][0] <= x <= expected_area["x"][1]) and (
            expected_area["y"][0] <= y <= expected_area["y"][1]
        ):
            score += 1
        else:
            print(f"query {qna['query']} response {response}")
            debug_image_with_identified_point(
                image.to_opencv(),
                (x, y),
                filepath=f"./{test_map['map_id']}_{qna['query'].replace(' ', '_')}.png",
            )

    print(f"Map {test_map['map_id']} point placement score: {score}/{len(test_map['questions'])}")
    # TODO: adjust these thresholds after adding more qnas in dataset
    assert score >= len(test_map["questions"]) * 0.25, (
        f"Goal placement score too low: {score}/{len(test_map['questions'])}"
    )


@pytest.mark.parametrize(
    "test_map",
    [
        test_map
        for test_map in load_test_cases(TEST_DIR / "test_map_interpretability.yaml")[
            "map_comprehension_tests"
        ]
    ],
)
def test_map_comprehension(test_map, vl_model):
    occupancy_grid = occupancy_grid_from_image(test_map["image_path"])

    # set robot pose for testing
    robot_pose = Pose(
        position=test_map["robot_pose"]["position"],
        orientation=test_map["robot_pose"]["orientation"],
    )

    og_image = OccupancyGridImage.from_occupancygrid(
        occupancy_grid, flip_vertical=False, robot_pose=robot_pose
    )
    image = og_image.image

    # query and score responses
    responses = {}
    score = 0
    for qna in test_map["questions"]:
        prompt = interpretability_prompt(qna["question"])
        response = vl_model.query(image, prompt)
        responses[qna["question"]] = response
        if re.search(qna["expected_pattern"], response, re.IGNORECASE):
            score += 1
        else:
            print(f"Q: {qna['question']}\nA: {response}\n")

    print(f"Map {test_map['map_id']} interpretability score: {score}/{len(test_map['questions'])}")
    assert score >= len(test_map["questions"]) * 0.7, (
        f"Map interpretability score too low: {score}/{len(test_map['questions'])}. Responses: {responses}"
    )


def debug_image_with_identified_point(image_frame, point: tuple[int, int], filepath: str) -> None:
    """Utility to visualize identified points on the image for debugging."""
    debug_image = image_frame.copy()
    x, y = point
    cv2.drawMarker(debug_image, (x, y), (0, 0, 0), cv2.MARKER_CROSS, 15, 2)
    cv2.imwrite(filepath, debug_image)
