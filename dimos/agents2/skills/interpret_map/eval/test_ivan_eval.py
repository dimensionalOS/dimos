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

from collections.abc import Callable
from dataclasses import dataclass
from functools import cached_property
import json
from pathlib import Path
import pickle
import re
from typing import TYPE_CHECKING

import cv2
from dimos_lcm.foxglove_msgs.ImageAnnotations import ImageAnnotations
import numpy as np
import pytest

from dimos.agents2.skills.interpret_map import OccupancyGridImage
from dimos.core import LCMTransport
from dimos.models.vl.base import VlModel
from dimos.models.vl.moondream import MoondreamVlModel
from dimos.models.vl.qwen import QwenVlModel
from dimos.msgs.geometry_msgs import Pose, PoseStamped, Quaternion, Transform, Vector3
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.type import Detection2DBBox, Detection2DPoint, ImageDetections2D
from dimos.protocol.tf import TF
from dimos.utils.data import get_data
from dimos.utils.generic import extract_json_from_llm_response

TEST_DIR = Path(__file__).parent


def load_test_cases(filepath: str):
    import yaml

    print(f"Loading test cases from {filepath}")
    with open(filepath) as f:
        data = yaml.safe_load(f)
    return data


@dataclass
class SetupOccupancyGrid:
    """
    Helper class to generate OccupancyGrid from image, and produce corresponding OccupancyGridImage object.
    """

    image_path: str
    robot_pose: dict
    occupancy_grid_image: OccupancyGridImage | None = None
    resolution: float = 0.05
    image: Image | None = None
    detections: ImageDetections2D | None = None
    model: VlModel | Callable[[], VlModel] = QwenVlModel

    def __post_init__(self):
        if callable(self.model):
            self.model = self.model()
        self.image = self.get_image()

    @cached_property
    def transforms(self) -> Transform:
        return [
            Transform(
                frame_id="world",
                child_frame_id="base_link",
                translation=Vector3([i * self.resolution for i in self.robot_pose["position"]]),
                rotation=Quaternion(*self.robot_pose["orientation"]),
            )
        ]

    @cached_property
    def pose_stamped(self) -> PoseStamped:
        return PoseStamped(
            frame_id="base_link",
            position=[i * self.resolution for i in self.robot_pose["position"]],
            orientation=self.robot_pose["orientation"],
        )

    def query_multi(self, query: str) -> PoseStamped:
        image = self.get_image()
        self.detections = self.model.query_multi_points(image, query)
        return self.detections

    def get_image(self):
        robot_pose = self.pose_stamped

        og_image = OccupancyGridImage.from_occupancygrid(
            self.costmap, flip_vertical=False, robot_pose=robot_pose
        )
        self.occupancy_grid_image = og_image
        self.image = og_image.image
        return og_image.image

    @property
    def costmap(self) -> OccupancyGrid:
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


@dataclass
class State:
    robot_pose: PoseStamped | None = None
    target: PoseStamped | None = None
    transforms: list[Transform] | None = None
    image: Image | None = None
    resolution: float = 0.05
    model: VlModel | Callable[[], VlModel] = MoondreamVlModel
    detections: ImageDetections2D | None = None

    def __post_init__(self):
        if callable(self.model):
            self.model = self.model()

    @classmethod
    def from_image(cls, name: str, **kwargs):
        return cls(
            image=Image.from_file(get_data("agent_occupancygrid_experiments") / name), **kwargs
        )

    def query(self, query: str) -> PoseStamped:
        query = goal_placement_prompt(query)
        print(query)
        self.detections = self.model.query_points(self.image, query)
        print(self.detections)
        return self.detections

    @property
    def costmap(self) -> OccupancyGrid:
        """
        Build OccupancyGrid from map image`.
        """
        # read image and convert to grid 1:1
        # expects rgb image with black as obstacles, white as free space and gray as unknown
        image_arr = self.image.to_rgb().data
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
        occupancy_grid.info.resolution = self.resolution
        occupancy_grid.grid = grid
        occupancy_grid.frame_id = "world"
        occupancy_grid.info.origin.position = Vector3(0.0, 0.0, 0.0)
        occupancy_grid.info.origin.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        return occupancy_grid


@pytest.fixture(scope="session")
def publish_state():
    def publish(state: State):
        if state.transforms:
            tf = TF()
            tf.publish(*state.transforms)
            tf.stop()

        # if state.target:
        #     pose: LCMTransport[PoseStamped] = LCMTransport("/target", PoseStamped)
        #     pose.publish(target)
        #     pose.lcm.stop()

        if state.costmap:
            costmap: LCMTransport[OccupancyGrid] = LCMTransport("/costmap", OccupancyGrid)
            costmap.publish(state.costmap)
            costmap.lcm.stop()

        if state.image:
            agent_image: LCMTransport[OccupancyGrid] = LCMTransport("/agent_image", Image)
            agent_image.publish(state.image)
            agent_image.lcm.stop()

        if state.detections:
            annotations: LCMTransport[ImageAnnotations] = LCMTransport(
                "/annotations", ImageAnnotations
            )
            annotations.publish(state.detections.to_foxglove_annotations())
            annotations.lcm.stop()

    yield publish


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
    )

    return prompt


def test_basic_image(publish_state, vl_model):
    state = State.from_image("ivan1.png")
    state.query("open area")
    publish_state(state)


@pytest.mark.parametrize(
    "test_map",
    [
        test_map
        for test_map in load_test_cases(TEST_DIR / "test_map_interpretability.yaml")[
            "point_placement_tests"
        ]
    ],
)
def test_point_placement(test_map, publish_state):
    # setup
    state = SetupOccupancyGrid(image_path=test_map["image_path"], robot_pose=test_map["robot_pose"])
    robot_pixel_coord = state.occupancy_grid_image.robot_pixel_coord

    queries = [
        goal_placement_prompt(qna["query"], robot_pixel_coord) for qna in test_map["questions"]
    ]

    state.query_multi(queries)
    publish_state(state)
