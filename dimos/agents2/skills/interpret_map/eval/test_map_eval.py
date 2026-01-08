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

from functools import cached_property
import json
from pathlib import Path
import pickle
import re
from typing import TYPE_CHECKING

import cv2
import numpy as np
import pytest

from dimos.agents2.skills.interpret_map import OccupancyGridImage
from dimos.core import LCMTransport

# from dimos.models.vl.moondream import MoondreamVlModel
# from dimos.models.vl.openai import OpenAIVlModel
from dimos.models.vl.qwen import QwenVlModel
from dimos.msgs.geometry_msgs import Pose, PoseStamped, Quaternion, Transform, Vector3
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import Image
from dimos.protocol.tf import TF
from dimos.utils.data import get_data
from dimos.utils.generic import extract_json_from_llm_response

TEST_DIR = Path(__file__).parent


class SetupOccupancyGrid:
    """
    Helper class to generate OccupancyGrid from image, and produce corresponding OccupancyGridImage object.

    Attributes:
        image_path (str): Path to the map image file.
        robot_pose (dict): Robot's pose in the map with keys 'position' (list of 3 floats - X Y Z) and 'orientation' (Quaternion).
        occupancy_grid (OccupancyGrid): Generated occupancy grid from the image.
        image (Image | None): Generated OccupancyGridImage from the occupancy grid.
    """

    def __init__(self, image_path: str, robot_pose: dict) -> None:
        self.image_path = image_path
        self.robot_pose = robot_pose
        self.occupancy_grid = self._occupancy_grid_from_image()
        self.occupancy_grid_image: OccupancyGridImage | None = None
        self.image: Image | None = None

    @cached_property
    def transforms(self) -> Transform:
        return [
            Transform(
                frame_id="world",
                child_frame_id="base_link",
                translation=Vector3(
                    [i * self.occupancy_grid.info.resolution for i in self.robot_pose["position"]]
                ),
                rotation=Quaternion(*self.robot_pose["orientation"]),
            )
        ]

    @cached_property
    def pose_stamped(self) -> PoseStamped:
        return PoseStamped(
            frame_id="base_link",
            position=[i * self.occupancy_grid.info.resolution for i in self.robot_pose["position"]],
            orientation=self.robot_pose["orientation"],
        )

    def get_image(self):
        robot_pose = self.pose_stamped
        width, height = self._get_encoded_image_size()

        og_image = OccupancyGridImage.from_occupancygrid(
            self.occupancy_grid, flip_vertical=False, robot_pose=robot_pose
        )
        self.image = og_image.image
        self.occupancy_grid_image = og_image

        return self.image

    def get_grid_to_image_encoding_scale(self):
        # scale to convert coordinates from new image back to original occupancy grid / image
        width, height = self._get_encoded_image_size()
        width_scale = self.occupancy_grid.info.width / width
        height_scale = self.occupancy_grid.info.height / height
        return width_scale, height_scale

    def _get_encoded_image_size(self):
        # keep max dimension 1024 for encoding
        MAX_IMAGE_DIMENSION = 1024
        aspect_ratio = self.occupancy_grid.info.width / self.occupancy_grid.info.height
        if aspect_ratio >= 1.0:
            width = MAX_IMAGE_DIMENSION
            height = int(MAX_IMAGE_DIMENSION / aspect_ratio)
        else:
            height = MAX_IMAGE_DIMENSION
            width = int(MAX_IMAGE_DIMENSION * aspect_ratio)
        return width, height

    def _occupancy_grid_from_image(self) -> OccupancyGrid:
        """
        Build OccupancyGrid from map image`.
        """
        # load image
        image_path = get_data("maps") / self.image_path
        image = Image.from_file(str(image_path))

        # read image and convert to grid 1:1
        # expects rgb image with black as obstacles, white as free space and gray as unknown
        image_arr = image.to_rgb().data
        height, width = image_arr.shape[:2]
        grid = np.full((height, width), 100, dtype=np.int8)  # obstacle by default

        # drop alpha channel if present
        if image_arr.shape[2] == 4:
            image_arr = image_arr[:, :, :3]

        # define colors and threshold
        WHITE = np.array([255, 255, 255], dtype=np.float32)
        GRAY = np.array([127, 127, 127], dtype=np.float32)  # approx RGB for 127 gray
        white_threshold = 30
        gray_threshold = 10

        # convert to float32 for distance calculations
        image_float = image_arr.astype(np.float32)

        # calculate distances to target colors using broadcasting
        white_dist = np.sqrt(np.sum((image_float - WHITE) ** 2, axis=2))
        gray_dist = np.sqrt(np.sum((image_float - GRAY) ** 2, axis=2))

        # assign based on closest color within threshold
        grid[white_dist <= white_threshold] = 0  # Free space
        grid[gray_dist <= gray_threshold] = -1  # Unknown space

        # build OccupancyGrid object
        occupancy_grid = OccupancyGrid()
        occupancy_grid.info.width = width
        occupancy_grid.info.height = height
        occupancy_grid.info.resolution = 0.05
        occupancy_grid.grid = grid
        occupancy_grid.frame_id = "world"
        occupancy_grid.info.origin.position = Vector3(0.0, 0.0, 0.0)
        occupancy_grid.info.origin.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        return occupancy_grid


def load_test_cases(filepath: str):
    import yaml

    print(f"Loading test cases from {filepath}")
    with open(filepath) as f:
        data = yaml.safe_load(f)
    return data


@pytest.fixture
def vl_model():
    return QwenVlModel()
    # return OpenAIVlModel()
    # return MoondreamVlModel()


def extract_coordinates(point: dict | None) -> list | str:
    if point is None:
        return "Failed to parse goal position: response is None."
    if "point" not in point:
        return "Failed to parse goal position: missing 'point' key."
    if not isinstance(point["point"], list):
        return "Failed to parse goal position: 'point' is not a list."

    return point["point"]


def goal_placement_prompt(description: str, robot_pixel_coord: tuple[int, int]) -> str:
    prompt = (
        "Look at this image carefully \n"
        "it represents a 2D map percieved from above (like a floor plan).\n"
        " - white pixels represent free space, \n"
        " - gray pixels represent unexplored space, \n"
        " - black pixels are obstacles and walls, \n"
        " - green circle represents the robot.\n"
        " - Note: The image may contain some noise or artifacts, ignore them and focus on clear structural patterns.\n"
        "The image has been rotated so that the robot always faces straight upwards.\n"
        "- The robot's front is towards the of the image.\n"
        "- The robot's back is towards the bottom.\n"
        "- The robot's left is towards the left.\n"
        "- The robot's right is towards the right.\n"
        f"Identify a location in free space based on the following description: {description}\n"
        f"Metadata - pixel coordinates of robot (x, y): {robot_pixel_coord}\n"
        "Guildelines for identified location: \n"
        " - the point should be reacheable by the robot following a reasonable path through free space, it should not be surrounded by walls on all sides.\n"
        " - NEVER place the point on thick walls, obstacles or unexplored space.\n"
        " - maintain clearance of few pixels and find the nearest clear location that still matches the general direction and description.\n"
        "Return ONLY a JSON object with this exact format:\n"
        '{"point": [x, y]}\n'
        f"where x,y are the pixel coordinates of the identified location in the image. \n"
    )

    return prompt


def interpretability_prompt(question: str) -> str:
    prompt = (
        "Look at this image carefully \n"
        "it represents a 2D map perceived by a robot from above (like a floor plan).\n"
        " - white pixels represent free space, \n"
        " - gray pixels represent unexplored space, \n"
        " - black pixels are obstacles and walls, \n"
        " - red pixels represent the robot.\n"
        "The image has been rotated so that the robot always faces straight upwards."
        "- The robot's front is towards the of the image."
        "- The robot's back is towards the bottom."
        "- The robot's left is towards the left."
        "- The robot's right is towards the right."
        f"Answer the following question based on this image: {question}\n"
    )
    return prompt


@pytest.fixture(scope="session")
def publish_state():
    def publish(state: SetupOccupancyGrid, target: PoseStamped | None = None):
        tf = TF()
        costmap: LCMTransport[OccupancyGrid] = LCMTransport("/costmap", OccupancyGrid)

        if target:
            pose: LCMTransport[PoseStamped] = LCMTransport("/target", PoseStamped)
            pose.publish(target)
            pose.lcm.stop()

        agent_image: LCMTransport[OccupancyGrid] = LCMTransport("/agent_image", Image)

        agent_image.publish(state.get_image())
        costmap.publish(state.occupancy_grid)
        tf.publish(*state.transforms)

        agent_image.lcm.stop()
        costmap.lcm.stop()
        tf.stop()

    yield publish


def target_from_llm(vl_model, state: SetupOccupancyGrid, query: str) -> PoseStamped:
    prompt = goal_placement_prompt(query)
    print("prompt is", prompt)
    image = state.get_image()
    response = vl_model.query(image, prompt)
    print("response is", response)
    point = extract_json_from_llm_response(response)
    x, y = extract_coordinates(point)

    debug_image_with_identified_point(
        image.to_opencv(),
        (x, y),
        filepath="./debug_llm_response.png",
    )
    world_pos = state.occupancy_grid_image.pixel_to_world(x, y, size=(image.width, image.height))
    return PoseStamped(
        frame_id="world",
        position=[world_pos.x, world_pos.y, 0],
    )


def test_ivan(publish_state, vl_model):
    def find(lst, predicate):
        return next((x for x in lst if predicate(x)), None)

    # find where map_id is "office_noise_tilt"
    cases = load_test_cases(TEST_DIR / "test_map_interpretability.yaml")["point_placement_tests"]
    test_map = find(cases, lambda c: c["map_id"] == "office_noise_tilt")

    grid_generator = SetupOccupancyGrid(
        image_path=test_map["image_path"], robot_pose=test_map["robot_pose"]
    )
    # image = grid_generator.get_image()
    width_scale, height_scale = grid_generator.get_grid_to_image_encoding_scale()

    target = target_from_llm(
        vl_model,
        grid_generator,
        "a few meters in front of the robot down the hallway",
    )

    print("publish target", target)

    publish_state(grid_generator, target)


# main tests
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
    """
    Evaluate the VL model's ability to identify positions on occupancy grid images.

    Every instance of test_map has:
    - map_id: str - unique identifier for the map
    - image_path: str - path to the occupancy grid image file
    - robot_pose: dict - robot's pose in the map with keys 'position' (list of 3 floats - X Y Z) and 'orientation' (Quaternion)
    - questions: list of dict
        - query: str - description of the goal position to identify
        - expected_range: dict - expected pixel coordinate ranges with keys 'x' (list of 2 ints) and 'y' (list of 2 ints)
    """

    # setup
    grid_generator = SetupOccupancyGrid(
        image_path=test_map["image_path"], robot_pose=test_map["robot_pose"]
    )
    image = grid_generator.get_image()
    robot_pixel_coord = grid_generator.occupancy_grid_image.robot_pixel_coord
    width_scale, height_scale = grid_generator.get_grid_to_image_encoding_scale()

    # query and score responses
    score = 0
    failed = []

    for qna in test_map["questions"]:
        prompt = goal_placement_prompt(qna["query"], robot_pixel_coord)
        response = vl_model.query(image, prompt)
        point = extract_json_from_llm_response(response)
        x, y = extract_coordinates(point)

        expected_area = qna["expected_range"]

        x_px = round(x * width_scale)
        y_px = round(y * height_scale)
        debug_path = f"./eval_data/{test_map['map_id']}_{qna['query'].replace(' ', '_')}.png"

        if (
            expected_area["x"][0] <= x_px <= expected_area["x"][1]
            and expected_area["y"][0] <= y_px <= expected_area["y"][1]
        ):
            score += 1
        else:
            debug_image_with_identified_point(
                image.to_opencv(),
                (x, y),
                filepath=debug_path,
            )
            failed.append(
                f"Query:\n  {qna['query']}\n"
                f"Predicted (px): ({x_px}, {y_px})\n"
                f"Expected X range: {expected_area['x']}\n"
                f"Expected Y range: {expected_area['y']}\n"
                f"Debug image: {debug_path}\n"
            )

        debug_image_with_identified_point(
            image.to_opencv(),
            (x, y),
            filepath=debug_path,
        )

    total = len(test_map["questions"])
    pass_rate = score / total

    print(f"Pass rate for {test_map['map_id']}: {pass_rate}")

    assert pass_rate >= 0.25, (
        "\n"
        f"Goal placement score too low for map '{test_map['map_id']}'\n"
        f"Score: {score}/{total} ({pass_rate:.1%})\n"
        "\n"
        "Incorrectly identified points:\n"
        "------------------------------\n"
        f"{''.join(failed)}"
    )


@pytest.mark.parametrize(
    "test_map",
    [
        # "office"
        test_map
        for test_map in load_test_cases(TEST_DIR / "test_map_interpretability.yaml")[
            "map_comprehension_tests"
        ]
    ],
)
def test_map_comprehension(test_map, vl_model):
    """
    Evaluate the VL model's ability to answer questions about occupancy grid images.

    Every instance of test_map has:
    - map_id: str - unique identifier for the map
    - image_path: str - path to the occupancy grid image file
    - robot_pose: dict - robot's pose in the map with keys 'position' (list of 3 floats - X Y Z) and 'orientation' (Quaternion)
    - questions: list of dict
        - question: str - question about the map
        - expected_pattern: str - regex pattern that the answer should match
    """
    # setup
    grid_generator = SetupOccupancyGrid(
        image_path=test_map["image_path"], robot_pose=test_map["robot_pose"]
    )
    image = grid_generator.get_image()

    # query and score responses
    responses = {}
    failed = []
    score = 0
    for qna in test_map["questions"]:
        prompt = interpretability_prompt(qna["question"])
        response = vl_model.query(image, prompt)
        responses[qna["question"]] = response
        if re.search(qna["expected_pattern"], response, re.IGNORECASE):
            score += 1
        else:
            failed.append(f"Q: {qna['question']}\nA: {response}\n")

    total = len(test_map["questions"])
    pass_rate = score / total

    print(f"Pass rate for {test_map['map_id']}: {pass_rate}")
    print(f"{''.join(failed)}")

    assert pass_rate >= 0.7, (
        "\n"
        f"Map interpretability score too low for map '{test_map['map_id']}'\n"
        f"Score: {score}/{total} ({pass_rate:.1%})\n"
        "\n"
        "Failed responses:\n"
        "-----------------\n"
        f"{''.join(failed)}"
    )


def debug_image_with_identified_point(image_frame, point: tuple[int, int], filepath: str) -> None:
    """Utility to visualize identified points on the image for debugging."""
    debug_image = image_frame.copy()
    x, y = point
    cv2.drawMarker(debug_image, (x, y), (255, 0, 0), cv2.MARKER_CROSS, 15, 2)
    cv2.imwrite(filepath, debug_image)
