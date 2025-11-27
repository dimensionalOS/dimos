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

import cv2

from dimos.agents2 import Output
from dimos.core.module import Module
from dimos.core.rpc_client import RpcCall
from dimos.core.skill_module import SkillModule
from dimos.core.stream import In, Out
from dimos.models.qwen.video_query import query_single_frame
from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.protocol.skill.skill import rpc, skill
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.agents2.skills.interpret_map")


class InterpretMapSkill(SkillModule):
    _latest_local_costmap: OccupancyGrid | None = None
    _robot_pose: Pose | None = None
    # _queried_map: OccupancyGrid | None = None

    local_costmap: In[OccupancyGrid] = None
    lidar: In[LidarMessage] = None

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(self.local_costmap.subscribe(self._on_local_costmap))
        self._disposables.add(self.lidar.subscribe(self._on_lidar))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_local_costmap(self, costmap: OccupancyGrid) -> None:
        self._latest_local_costmap = costmap

    def _on_lidar(self, lidar: LidarMessage) -> None:
        center = lidar.pointcloud.get_center()
        self._robot_pose = Pose(Vector3(center[0], center[1], 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))

    @skill()
    def get_goal_position(self, description: str | None = None) -> Pose | None:
        """
        Identify goal position from map, based on description of location.
        Use the general description of location provided by user.
        """

        if description is None:
            return "Please provide a description of the goal location."

        costmap = self._latest_local_costmap

        if costmap is None:
            return "No map available."

        # augment with robot position
        if self._robot_pose:
            costmap.robot_pose = self._robot_pose

        image = costmap.grid_to_image(size=(1024, 1024), flip_vertical=True)

        image = image.to_opencv()

        prompt = (
            "Look at this image carefully \n"
            "it represents a 2D occupancy grid map where,\n"
            " - blue area is free space, \n"
            " - yellow area is unknown space, \n"
            " - red (and its shades) areas are obstacles, \n"
            " - green square is the robot's current position. \n"
            f"Identify a goal position ONLY IN FREE SPACE that closely matches the following description: {description}\n"
            "Prioritize selecting a goal position in free space (blue area) over exactly matching the description. \n"
            "MAKE SURE there is a clear path from the robot's current position to the goal position without crossing any obstacles. \n"
            "MAKE SURE the goal position is located in the blue area (free space) of the map and few pixels away from obstacles or objects. \n"
            "Return ONLY a JSON object with this exact format:\n"
            "{'point': [x, y]}\n"
            f"where x,y are the pixel coordinates of the goal position in the image. \n"
        )

        response = query_single_frame(image, prompt)
        x, y = parse_qwen_points_response(response)

        debug_image_with_identified_point(image, (x, y), filepath="./debug_goal_position_1.png")
        # TODO:guardrails to ensure point is in free space, or find nearest free space
        if not costmap.is_free_space(x, y):
            logger.warning(
                f"Identified goal position ({x}, {y}) is not in free space, choosing nearest free space instead."
            )
            closest_free_point = costmap.get_closest_free_point(x, y)
            if closest_free_point is not None:
                x, y = closest_free_point

        if x is not None and y is not None:
            debug_image_with_identified_point(image, (x, y), filepath="./debug_goal_position.png")

        # get world coordinates from pixel for navigation
        goal_pose = costmap.pixel_to_world(x, y, size=(1024, 1024), flip_vertical=True)

        return goal_pose


def debug_image_with_identified_point(
    image_frame, point: tuple[int, int], filepath: str = "./debug_goal_position.png"
) -> None:
    """Utility to visualize identified points on the image for debugging."""
    debug_image = image_frame.copy()
    x, y = point
    cv2.drawMarker(debug_image, (x, y), (255, 255, 255), cv2.MARKER_CROSS, 15, 2)
    cv2.imwrite(filepath, debug_image)


def parse_qwen_points_response(response: str) -> tuple[int, int] | None:
    """Parse Qwen response to extract robot position.

    Args:
        response: Qwen response string

    Returns:
        Tuple of (x, y) pixel coordinates or None if parsing fails
    """
    try:
        start_idx = response.find("{")
        end_idx = response.rfind("}") + 1
        if start_idx >= 0 and end_idx > start_idx:
            json_str = response[start_idx:end_idx]
            result = json.loads(json_str)

            if "point" in result and len(result["point"]) == 2:
                x, y = result["point"]
                return (int(x), int(y))
    except Exception as e:
        logger.error(f"Error parsing Qwen response: {e}")
        logger.error(f"Raw response: {response}")
    return None


interpret_map_skill = InterpretMapSkill.blueprint

__all__ = ["InterpretMapSkill", "interpret_map_skill"]
