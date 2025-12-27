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
from typing import Any

from dimos.core.core import rpc
from dimos.core.skill_module import SkillModule
from dimos.core.stream import In
from dimos.models.qwen.video_query import BBox
from dimos.models.vl.qwen import QwenVlModel
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3
from dimos.msgs.geometry_msgs.Vector3 import make_vector3
from dimos.msgs.sensor_msgs import Image
from dimos.navigation.base import NavigationState
from dimos.navigation.visual.query import get_object_bbox_from_image
from dimos.protocol.skill.skill import skill
from dimos.types.robot_location import RobotLocation
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class NavigationSkillContainer(SkillModule):
    _latest_image: Image | None = None
    _latest_odom: PoseStamped | None = None
    _skill_started: bool = False
    _similarity_threshold: float = 0.23

    rpc_calls: list[str] = [
        "SpatialMemory.tag_location",
        "SpatialMemory.query_tagged_location",
        "SpatialMemory.query_by_text",
        "NavigationInterface.set_goal",
        "NavigationInterface.get_state",
        "NavigationInterface.is_goal_reached",
        "NavigationInterface.cancel_goal",
        "ObjectTracking.track",
        "ObjectTracking.stop_track",
        "ObjectTracking.is_tracking",
        "WavefrontFrontierExplorer.stop_exploration",
        "WavefrontFrontierExplorer.explore",
        "WavefrontFrontierExplorer.is_exploration_active",
    ]

    color_image: In[Image]
    odom: In[PoseStamped]

    def __init__(self) -> None:
        super().__init__()
        self._skill_started = False
        self._vl_model = QwenVlModel()

    @rpc
    def start(self) -> None:
        self._disposables.add(self.color_image.subscribe(self._on_color_image))  # type: ignore[arg-type]
        self._disposables.add(self.odom.subscribe(self._on_odom))  # type: ignore[arg-type]
        self._skill_started = True

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_color_image(self, image: Image) -> None:
        self._latest_image = image

    def _on_odom(self, odom: PoseStamped) -> None:
        self._latest_odom = odom

    @skill()
    def tag_location(self, location_name: str) -> str:
        """Tag this location in the spatial memory with a name.

        This associates the current location with the given name in the spatial memory, allowing you to navigate back to it.

        Args:
            location_name (str): the name for the location

        Returns:
            str: the outcome
        """

        if not self._skill_started:
            raise ValueError(f"{self} has not been started.")
        tf = self.tf.get("map", "base_link", time_tolerance=2.0)
        if not tf:
            return "Could not get the robot's current transform."

        position = tf.translation
        rotation = tf.rotation.to_euler()

        location = RobotLocation(
            name=location_name,
            position=(position.x, position.y, position.z),
            rotation=(rotation.x, rotation.y, rotation.z),
        )

        tag_location_rpc = self.get_rpc_calls("SpatialMemory.tag_location")
        if not tag_location_rpc(location):
            return f"Error: Failed to store '{location_name}' in the spatial memory"

        logger.info(f"Tagged {location}")
        return f"Tagged '{location_name}': ({position.x},{position.y})."

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

        if not self._skill_started:
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

    def _navigate_by_tagged_location(self, query: str) -> str | None:
        try:
            query_tagged_location_rpc = self.get_rpc_calls("SpatialMemory.query_tagged_location")
        except Exception:
            logger.warning("SpatialMemory module not connected, cannot query tagged locations")
            return None

        robot_location = query_tagged_location_rpc(query)

        if not robot_location:
            return None

        print("Found tagged location:", robot_location)
        goal_pose = PoseStamped(
            position=make_vector3(*robot_location.position),
            orientation=Quaternion.from_euler(Vector3(*robot_location.rotation)),
            frame_id="map",
        )

        result = self._navigate_to(goal_pose)
        if not result:
            return "Error: Faild to reach the tagged location."

        return (
            f"Successfuly arrived at location tagged '{robot_location.name}' from query '{query}'."
        )

    def _navigate_to(self, pose: PoseStamped) -> bool:
        try:
            set_goal_rpc, get_state_rpc, is_goal_reached_rpc = self.get_rpc_calls(
                "NavigationInterface.set_goal",
                "NavigationInterface.get_state",
                "NavigationInterface.is_goal_reached",
            )
        except Exception:
            logger.error("Navigation module not connected properly")
            return False

        logger.info(
            f"Navigating to pose: ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})"
        )
        set_goal_rpc(pose)
        time.sleep(1.0)

        while get_state_rpc() == NavigationState.FOLLOWING_PATH:
            time.sleep(0.25)

        time.sleep(1.0)
        if not is_goal_reached_rpc():
            logger.info("Navigation was cancelled or failed")
            return False
        else:
            logger.info("Navigation goal reached")
            return True

    def _navigate_to_object(self, query: str) -> str | None:
        try:
            bbox = self._get_bbox_for_current_frame(query)
        except Exception:
            logger.error(f"Failed to get bbox for {query}", exc_info=True)
            return None

        if bbox is None:
            return None

        try:
            track_rpc, stop_track_rpc, is_tracking_rpc = self.get_rpc_calls(
                "ObjectTracking.track", "ObjectTracking.stop_track", "ObjectTracking.is_tracking"
            )
        except Exception:
            logger.error("ObjectTracking module not connected properly")
            return None

        try:
            get_state_rpc, is_goal_reached_rpc = self.get_rpc_calls(
                "NavigationInterface.get_state", "NavigationInterface.is_goal_reached"
            )
        except Exception:
            logger.error("Navigation module not connected properly")
            return None

        logger.info(f"Found {query} at {bbox}")

        # Start tracking - BBoxNavigationModule automatically generates goals
        track_rpc(bbox)

        start_time = time.time()
        timeout = 30.0
        goal_set = False

        while time.time() - start_time < timeout:
            # Check if navigator finished
            if get_state_rpc() == NavigationState.IDLE and goal_set:
                logger.info("Waiting for goal result")
                time.sleep(1.0)
                if not is_goal_reached_rpc():
                    logger.info(f"Goal cancelled, tracking '{query}' failed")
                    stop_track_rpc()
                    return None
                else:
                    logger.info(f"Reached '{query}'")
                    stop_track_rpc()
                    return f"Successfully arrived at '{query}'"

            # If goal set and tracking lost, just continue (tracker will resume or timeout)
            if goal_set and not is_tracking_rpc():
                continue

            # BBoxNavigationModule automatically sends goals when tracker publishes
            # Just check if we have any detections to mark goal_set
            if is_tracking_rpc():
                goal_set = True

            time.sleep(0.25)

        logger.warning(f"Navigation to '{query}' timed out after {timeout}s")
        stop_track_rpc()
        return None

    def _get_bbox_for_current_frame(self, query: str) -> BBox | None:
        if self._latest_image is None:
            return None

        return get_object_bbox_from_image(self._vl_model, self._latest_image, query)

    def _navigate_using_semantic_map(self, query: str) -> str:
        try:
            query_by_text_rpc = self.get_rpc_calls("SpatialMemory.query_by_text")
        except Exception:
            return "Error: The SpatialMemory module is not connected."

        results = query_by_text_rpc(query)

        if not results:
            return f"No matching location found in semantic map for '{query}'"

        best_match = results[0]

        goal_pose = self._get_goal_pose_from_result(best_match)

        print("Goal pose for semantic nav:", goal_pose)
        if not goal_pose:
            return f"Found a result for '{query}' but it didn't have a valid position."

        result = self._navigate_to(goal_pose)

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

        if not self._skill_started:
            raise ValueError(f"{self} has not been started.")

        self._cancel_goal_and_stop()

        return "Stopped"

    def _cancel_goal_and_stop(self) -> None:
        try:
            cancel_goal_rpc = self.get_rpc_calls("NavigationInterface.cancel_goal")
        except Exception:
            logger.warning("Navigation module not connected, cannot cancel goal")
            return

        try:
            stop_exploration_rpc = self.get_rpc_calls("WavefrontFrontierExplorer.stop_exploration")
        except Exception:
            logger.warning("FrontierExplorer module not connected, cannot stop exploration")
            return

        cancel_goal_rpc()
        return stop_exploration_rpc()  # type: ignore[no-any-return]

    @skill()
    def start_exploration(self, timeout: float = 240.0) -> str:
        """A skill that performs autonomous frontier exploration.

        This skill continuously finds and navigates to unknown frontiers in the environment
        until no more frontiers are found or the exploration is stopped.

        Don't call any other skills except stop_movement skill when needed.

        Args:
            timeout (float, optional): Maximum time (in seconds) allowed for exploration
        """

        if not self._skill_started:
            raise ValueError(f"{self} has not been started.")

        try:
            return self._start_exploration(timeout)
        finally:
            self._cancel_goal_and_stop()

    def _start_exploration(self, timeout: float) -> str:
        try:
            explore_rpc, is_exploration_active_rpc = self.get_rpc_calls(
                "WavefrontFrontierExplorer.explore",
                "WavefrontFrontierExplorer.is_exploration_active",
            )
        except Exception:
            return "Error: The WavefrontFrontierExplorer module is not connected."

        logger.info("Starting autonomous frontier exploration")

        start_time = time.time()

        has_started = explore_rpc()
        if not has_started:
            return "Error: Could not start exploration."

        while time.time() - start_time < timeout and is_exploration_active_rpc():
            time.sleep(0.5)

        return "Exploration completed successfuly"

    def _get_goal_pose_from_result(self, result: dict[str, Any]) -> PoseStamped | None:
        similarity = 1.0 - (result.get("distance") or 1)
        if similarity < self._similarity_threshold:
            logger.warning(
                f"Match found but similarity score ({similarity:.4f}) is below threshold ({self._similarity_threshold})"
            )
            return None

        metadata = result.get("metadata")
        if not metadata:
            return None
        print(metadata)
        first = metadata[0]
        print(first)
        pos_x = first.get("pos_x", 0)
        pos_y = first.get("pos_y", 0)
        theta = first.get("rot_z", 0)

        return PoseStamped(
            position=make_vector3(pos_x, pos_y, 0),
            orientation=Quaternion.from_euler(make_vector3(0, 0, theta)),
            frame_id="map",
        )


navigation_skill = NavigationSkillContainer.blueprint

__all__ = ["NavigationSkillContainer", "navigation_skill"]
