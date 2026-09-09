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

import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.agents.skill_result import SkillResult
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.models.qwen.bbox import BBox
from dimos.models.vl.create import create
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3, make_vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.base import NavigationState
from dimos.navigation.navigation_spec import NavigationInterfaceSpec
from dimos.navigation.skill_navigation import wait_for_navigation
from dimos.navigation.visual.query import get_object_bbox_from_image
from dimos.perception.experimental.object_tracking_spec import ObjectTrackingSpec
from dimos.perception.experimental.spatial_memory_spec import SpatialMemorySpec
from dimos.types.robot_location import RobotLocation
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class NavigationSkillContainer(Module):
    _latest_image: Image | None = None
    _latest_odom: PoseStamped | None = None
    _skill_started: bool = False
    _similarity_threshold: float = 0.23

    _spatial_memory: SpatialMemorySpec
    _navigation: NavigationInterfaceSpec
    _object_tracking: ObjectTrackingSpec | None = None

    color_image: In[Image]
    odom: In[PoseStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._skill_started = False

        self._vl_model = create(self.config.g.vl_model)

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_color_image)))
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self._skill_started = True

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_color_image(self, image: Image) -> None:
        self._latest_image = image

    def _on_odom(self, odom: PoseStamped) -> None:
        self._latest_odom = odom

    @skill
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

        if not self._latest_odom:
            return "No odometry data received yet, cannot tag location."

        position = self._latest_odom.position
        rotation = self._latest_odom.orientation

        location = RobotLocation(
            name=location_name,
            position=(position.x, position.y, position.z),
            rotation=(rotation.x, rotation.y, rotation.z),
        )

        if not self._spatial_memory.tag_location(location):
            return f"Error: Failed to store '{location_name}' in the spatial memory"

        logger.info(f"Tagged {location}")
        return f"Tagged '{location_name}': ({position.x},{position.y})."

    @skill(uses=[CAP_MOVEMENT])
    def navigate_with_text(self, query: str) -> SkillResult:
        """Navigate to a described location and wait for arrival or failure. Success means the navigator reached its goal; inspect the camera for precise visual placement.

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

        return SkillResult.fail(
            "NOT_FOUND", f"No visible, tagged, or mapped location matching {query!r}"
        )

    def _navigate_by_tagged_location(self, query: str) -> SkillResult | None:
        robot_location = self._spatial_memory.query_tagged_location(query)

        if not robot_location:
            return None

        logger.info("Found tagged location", location=robot_location)
        goal_pose = PoseStamped(
            position=make_vector3(*robot_location.position),
            orientation=Quaternion.from_euler(Vector3(*robot_location.rotation)),
            frame_id="map",
        )

        return self._navigate_to(goal_pose, f"Found a tagged location called '{query}'.")

    def _navigate_to(self, pose: PoseStamped, message: str) -> SkillResult:
        logger.info(
            f"Navigating to pose: ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})"
        )
        if not self._navigation.set_goal(pose):
            return SkillResult.fail("EXECUTION_FAILED", "Navigator rejected the goal")
        result = wait_for_navigation(self._navigation)
        result.message = f"{message} {result.message}"
        return result

    def _navigate_to_object(self, query: str) -> SkillResult | None:
        if self._object_tracking is None:
            return None

        try:
            bbox = self._get_bbox_for_current_frame(query)
        except Exception:
            logger.error(f"Failed to get bbox for {query}", exc_info=True)
            return None

        if bbox is None:
            return None

        logger.info(f"Found {query} at {bbox}")

        # Start tracking - BBoxNavigationModule automatically generates goals
        self._object_tracking.track(bbox)  # type: ignore[arg-type]

        start_time = time.time()
        timeout = 30.0
        goal_set = False

        while time.time() - start_time < timeout:
            # Check if navigator finished
            if self._navigation.get_state() == NavigationState.IDLE and goal_set:
                logger.info("Waiting for goal result")
                time.sleep(1.0)
                if not self._navigation.is_goal_reached():
                    logger.info(f"Goal cancelled, tracking '{query}' failed")
                    self._object_tracking.stop_track()
                    self._navigation.cancel_goal()
                    return SkillResult.fail(
                        "NAVIGATION_STOPPED", "Visual navigation was cancelled or failed"
                    )
                else:
                    logger.info(f"Reached '{query}'")
                    self._object_tracking.stop_track()
                    return SkillResult.ok(f"Successfully arrived at {query!r}", status="succeeded")

            # If goal set and tracking lost, just continue (tracker will resume or timeout)
            if goal_set and not self._object_tracking.is_tracking():
                time.sleep(0.25)
                continue

            # BBoxNavigationModule automatically sends goals when tracker publishes
            # Just check if we have any detections to mark goal_set
            if self._object_tracking.is_tracking():
                goal_set = True

            time.sleep(0.25)

        logger.warning(f"Navigation to '{query}' timed out after {timeout}s")
        self._object_tracking.stop_track()
        self._navigation.cancel_goal()
        return SkillResult.fail("EXECUTION_TIMEOUT", "Visual navigation timed out; goal cancelled")

    def _get_bbox_for_current_frame(self, query: str) -> BBox | None:
        if self._latest_image is None:
            return None

        return get_object_bbox_from_image(self._vl_model, self._latest_image, query)

    def _navigate_using_semantic_map(self, query: str) -> SkillResult | None:
        results = self._spatial_memory.query_by_text(query)

        if not results:
            return None

        best_match = results[0]

        goal_pose = self._get_goal_pose_from_result(best_match)

        logger.info("Goal pose for semantic nav", pose=goal_pose)
        if not goal_pose:
            return SkillResult.fail("NOT_FOUND", f"No valid position for {query!r}")

        message = f"Found a location in the semantic map matching '{query}'."
        return self._navigate_to(goal_pose, message)

    @skill
    def stop_navigation(self) -> str:
        """Immediatly stop moving."""

        if not self._skill_started:
            raise ValueError(f"{self} has not been started.")

        self._cancel_goal_and_stop()

        return "Stopped"

    def _cancel_goal_and_stop(self) -> None:
        if self._object_tracking is not None:
            self._object_tracking.stop_track()
        self._navigation.cancel_goal()

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
        first = metadata[0]
        pos_x = first.get("pos_x", 0)
        pos_y = first.get("pos_y", 0)
        theta = first.get("rot_z", 0)

        return PoseStamped(
            position=make_vector3(pos_x, pos_y, 0),
            orientation=Quaternion.from_euler(make_vector3(0, 0, theta)),
            frame_id="map",
        )
