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

import time
from typing import Any, Optional
import cv2
from reactivex import Observable

from dimos.models.vl.qwen import QwenVlModel
from dimos.msgs.sensor_msgs import Image
from dimos.navigation.visual.query import get_object_bbox_from_image
from dimos.protocol.skill.skill import SkillContainer, skill
from dimos.robot.robot import UnitreeRobot
from dimos.types.robot_location import RobotLocation
from dimos.models.qwen.video_query import BBox
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import make_vector3
from dimos.utils.transform_utils import euler_to_quaternion, quaternion_to_euler
from dimos.utils.logging_config import setup_logger
from reactivex.disposable import Disposable, CompositeDisposable

logger = setup_logger(__file__)


class NavigationSkillContainer(SkillContainer):
    _robot: UnitreeRobot
    _disposables: CompositeDisposable
    _latest_image: Optional[Image]
    _video_stream: Observable[Image]
    _started: bool

    def __init__(self, robot: UnitreeRobot, video_stream: Observable[Image]):
        super().__init__()
        self._robot = robot
        self._disposables = CompositeDisposable()
        self._latest_image = None
        self._video_stream = video_stream
        self._similarity_threshold = 0.23
        self._started = False
        self._vl_model = QwenVlModel()

    def __enter__(self) -> "NavigationSkillContainer":
        unsub = self._video_stream.subscribe(self._on_video)
        self._disposables.add(Disposable(unsub) if callable(unsub) else unsub)
        self._started = True
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._disposables.dispose()
        self.stop()
        return False

    def _on_video(self, image: Image) -> None:
        self._latest_image = image

    @skill()
    def tag_location_in_spatial_memory(self, location_name: str) -> str:
        """Tag this location in the spatial memory with a name.

        This associates the current location with the given name in the spatial memory, allowing you to navigate back to it.

        Args:
            location_name (str): the name for the location

        Returns:
            str: the outcome
        """

        if not self._started:
            raise ValueError(f"{self} has not been started.")

        pose_data = self._robot.get_odom()
        position = pose_data.position
        rotation = quaternion_to_euler(pose_data.orientation)

        location = RobotLocation(
            name=location_name,
            position=(position.x, position.y, position.z),
            rotation=(rotation.x, rotation.y, rotation.z),
        )

        if not self._robot.spatial_memory.tag_location(location):
            return f"Failed to store '{location_name}' in the spatial memory"

        logger.info(f"Tagged {location}")
        return f"The current location has been tagged as '{location_name}'."

    @skill()
    def navigate_with_text(self, query: str) -> str:
        """Navigate to a location by querying the existing semantic map using natural language.

        First attempts to locate an object in the robot's camera view using vision.
        If the object is found, navigates to it. If not, falls back to querying the
        semantic map for a location matching the description.
        CALL THIS SKILL FOR ONE SUBJECT AT A TIME. For example: "Go to the person wearing a blue shirt in the living room",
        you should call this skill twice, once for the person wearing a blue shirt and once for the living room.
        Args:
            query: Text query to search for in the semantic map
        """

        if not self._started:
            raise ValueError(f"{self} has not been started.")

        success_msg = self._navigate_by_tagged_location(query)
        if success_msg:
            return success_msg

        logger.info(f"No tagged location found for {query}")

        success_msg = self._navigate_to_object(query)
        if success_msg:
            return success_msg

        logger.info(f"No object in view found for {query}")

        success_msg = self._navigate_using_semantic_map(query)
        if success_msg:
            return success_msg

        return f"No tagged location called '{query}'. No object in view matching '{query}'. No matching location found in semantic map for '{query}'."

    def _navigate_by_tagged_location(self, query: str) -> Optional[str]:
        robot_location = self._robot.spatial_memory.query_tagged_location(query)

        if not robot_location:
            return None

        goal_pose = PoseStamped(
            position=make_vector3(*robot_location.position),
            orientation=euler_to_quaternion(make_vector3(*robot_location.rotation)),
            frame_id="world",
        )

        result = self._robot.navigate_to(goal_pose, blocking=True)
        if not result:
            return None

        return (
            f"Successfuly arrived at location tagged '{robot_location.name}' from query '{query}'."
        )

    def _navigate_to_object(self, query: str) -> Optional[str]:
        try:
            bbox = self._get_bbox_for_current_frame(query)
        except Exception:
            logger.error(f"Failed to get bbox for {query}", exc_info=True)
            return None

        if bbox is None:
            return None

        logger.info(f"Found {query} at {bbox}")

        success = self._robot.navigate_to_object(bbox)

        if not success:
            logger.warning(f"Failed to navigate to '{query}' at {bbox}")
            return None

        return "Successfully navigated to object from query '{query}'."

    def _get_bbox_for_current_frame(self, query: str) -> Optional[BBox]:
        if self._latest_image is None:
            return None

        frame = cv2.cvtColor(self._latest_image.data, cv2.COLOR_RGB2BGR)
        if frame is None:
            return None

        return get_object_bbox_from_image(self._vl_model, frame, query)

    def _navigate_using_semantic_map(self, query: str) -> str:
        results = self._robot.spatial_memory.query_by_text(query)

        if not results:
            return f"No matching location found in semantic map for '{query}'"

        best_match = results[0]

        goal_pose = self._get_goal_pose_from_result(best_match)

        if not goal_pose:
            return f"Found a result for '{query}' but it didn't have a valid position."

        result = self._robot.navigate_to(goal_pose, blocking=True)

        if not result:
            return f"Failed to navigate for '{query}'"

        return f"Successfuly arrived at '{query}'"

    @skill()
    def follow_human(self, person: str) -> str:
        """Follow a specific person"""
        return "Not implemented yet."

    @skill()
    def stop_movement(self) -> str:
        """Immediatly stop moving."""

        if not self._started:
            raise ValueError(f"{self} has not been started.")

        self._robot.stop_exploration()

        return "Stopped"

    @skill()
    def start_exploration(self, timeout: float = 240.0) -> str:
        """A skill that performs autonomous frontier exploration.

        This skill continuously finds and navigates to unknown frontiers in the environment
        until no more frontiers are found or the exploration is stopped.

        Don't call any other skills except stop_movement skill when needed.

        Args:
            timeout (float, optional): Maximum time (in seconds) allowed for exploration
        """

        if not self._started:
            raise ValueError(f"{self} has not been started.")

        try:
            return self._start_exploration(timeout)
        finally:
            self._robot.stop_exploration()

    def _start_exploration(self, timeout: float) -> str:
        logger.info("Starting autonomous frontier exploration")

        start_time = time.time()

        has_started = self._robot.explore()
        if not has_started:
            return "Could not start exploration."

        while time.time() - start_time < timeout and self._robot.is_exploration_active():
            time.sleep(0.5)

        return "Exploration completed successfuly"

    def _get_goal_pose_from_result(self, result: dict[str, Any]) -> Optional[PoseStamped]:
        similarity = 1.0 - (result.get("distance") or 1)
        if similarity < self._similarity_threshold:
            logger.warning(
                f"Match found but similarity score ({similarity:.4f}) is below threshold ({self._similarity_threshold})"
            )
            return None

        metadata = result.get("metadata")
        if not metadata:
            return None

        first = metadata[0]
        pos_x = first.get("pos_x", 0)
        pos_y = first.get("pos_y", 0)
        theta = first.get("rot_z", 0)

        return PoseStamped(
            position=make_vector3(pos_x, pos_y, 0),
            orientation=euler_to_quaternion(make_vector3(0, 0, theta)),
            frame_id="world",
        )
