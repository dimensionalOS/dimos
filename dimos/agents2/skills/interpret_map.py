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
import os

import cv2
import numpy as np

from dimos.core.module import Module
from dimos.core.rpc_client import RpcCall
from dimos.core.skill_module import SkillModule
from dimos.core.stream import In, Out
from dimos.models.vl.qwen import QwenVlModel
from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.nav_msgs.OccupancyGridImage import OccupancyGridImage
from dimos.protocol.skill.skill import rpc, skill
from dimos.utils.generic import extract_json_from_llm_response
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class InterpretMapSkill(SkillModule):
    _latest_local_costmap: OccupancyGrid | None = None
    _robot_pose: Pose | None = None
    # _queried_map: OccupancyGrid | None = None

    local_costmap: In[OccupancyGrid] = None  # type: ignore[assignment]

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(self.local_costmap.subscribe(self._on_local_costmap))  # type: ignore[arg-type]
        self.vl_model = QwenVlModel()

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_local_costmap(self, costmap: OccupancyGrid) -> None:
        self._latest_local_costmap = costmap
        self._robot_pose = self.tf.get("world", "base_link")

    @skill()
    def get_goal_position(self, description: str | None = None) -> Vector3 | str:
        """
        Identify goal position from map, based on description of location.
        Use the general description of location provided by user.

        Example call:
            args = {"description": "a clear area near a table on the left side of the room"}
            get_goal_position(**args)

        Args:
            description: General description of desired goal location.
        """

        if description is None:
            return "Please provide a description of the goal location."

        # grab latest costmap and robot pose
        costmap = self._latest_local_costmap
        robot_pose = None
        if self._robot_pose:
            robot_pose = Pose(
                position=self._robot_pose.translation, orientation=self._robot_pose.rotation
            )

        if costmap is None:
            return "No map available."

        if robot_pose:
            logger.info(f"Robot pose: {robot_pose}")
            costmap.robot_pose = robot_pose

        grid_encoder = OccupancyGridImage(
            occupancy_grid=costmap, size=(1024, 1024), flip_vertical=True, robot_pose=robot_pose
        )

        image = grid_encoder.encode()

        prompt = (
            "Look at this image carefully \n"
            "it represents a 2D occupancy grid map where,\n"
            " - blue area is free space, \n"
            " - yellow area is unknown space, \n"
            " - red (and its shades) areas are obstacles, \n"
            " - green object represents the robot's position and points to the direction it is facing. \n"
            f"Identify a location in free space based on the following description: {description}\n"
            "Prioritize selecting a goal position in free space (blue area) over exactly matching the description. \n"
            "MAKE SURE there is a clear path from the robot's current position to the goal position without crossing any obstacles. \n"
            "MAKE SURE the goal position is located in the blue area (free space) of the map and few pixels away from obstacles or objects. \n"
            "Return ONLY a JSON object with this exact format:\n"
            "{'point': [x, y]}\n"
            f"where x,y are the pixel coordinates of the goal position in the image. \n"
        )

        response = self.vl_model.query(image, prompt)
        point = extract_json_from_llm_response(response)
        if point is None or "point" not in point:
            return "Failed to parse goal position from model response."
        x, y = point["point"]

        # ensure point is in free space, else choose nearest free space
        if not grid_encoder.is_free_space(x, y):
            logger.warning(
                f"Identified goal position ({x}, {y}) is not in free space, choosing nearest free space instead."
            )
            closest_free_point = grid_encoder.get_closest_free_point(x, y)
            if closest_free_point is not None:
                x, y = closest_free_point

        if os.environ.get("DEBUG_INTERPRET_MAP_IMAGE"):
            debug_image_with_identified_point(
                image.to_opencv(),
                (x, y),
                filepath=os.environ["DEBUG_INTERPRET_MAP_IMAGE"],
            )

        # get world coordinates from pixel for navigation
        goal_pose = grid_encoder.pixel_to_world(x, y, size=(1024, 1024), flip_vertical=True)

        return goal_pose


def debug_image_with_identified_point(
    image_frame: np.ndarray, point: tuple[int, int], filepath: str
) -> None:
    """Utility to visualize identified points on the image for debugging."""
    debug_image = image_frame.copy()
    x, y = point
    cv2.drawMarker(debug_image, (x, y), (255, 255, 255), cv2.MARKER_CROSS, 15, 2)
    cv2.imwrite(filepath, debug_image)


interpret_map_skill = InterpretMapSkill.blueprint

__all__ = ["InterpretMapSkill", "interpret_map_skill"]
