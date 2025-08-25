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

"""
Semantic map skills for building and navigating spatial memory maps.

This module provides two skills:
1. BuildSemanticMap - Builds a semantic map by recording video frames at different locations
2. Navigate - Queries an existing semantic map using natural language
"""

import os
import time
from typing import Optional, Tuple
import cv2
from pydantic import Field

from dimos.skills.skills import AbstractRobotSkill
from dimos.types.robot_location import RobotLocation
from dimos.utils.logging_config import setup_logger
from dimos.models.qwen.video_query import get_bbox_from_qwen_frame
from dimos.msgs.geometry_msgs import PoseStamped, Vector3
from dimos.utils.transform_utils import euler_to_quaternion, quaternion_to_euler

logger = setup_logger("dimos.skills.semantic_map_skills")


def get_dimos_base_path():
    """
    Get the DiMOS base path from DIMOS_PATH environment variable or default to user's home directory.

    Returns:
        Base path to use for DiMOS assets
    """
    dimos_path = os.environ.get("DIMOS_PATH")
    if dimos_path:
        return dimos_path
    # Get the current user's username
    user = os.environ.get("USER", os.path.basename(os.path.expanduser("~")))
    return f"/home/{user}/dimos"


class NavigateWithText(AbstractRobotSkill):
    """
    A skill that queries an existing semantic map using natural language or tries to navigate to an object in view.

    This skill first attempts to locate an object in the robot's camera view using vision.
    If the object is found, it navigates to it. If not, it falls back to querying the
    semantic map for a location matching the description. For example, "Find the Teddy Bear"
    will first look for a Teddy Bear in view, then check the semantic map coordinates where
    a Teddy Bear was previously observed.

    CALL THIS SKILL FOR ONE SUBJECT AT A TIME. For example: "Go to the person wearing a blue shirt in the living room",
    you should call this skill twice, once for the person wearing a blue shirt and once for the living room.

    If skip_visual_search is True, this skill will skip the visual search for the object in view.
    This is useful if you want to navigate to a general location such as a kitchen or office.
    For example, "Go to the kitchen" will not look for a kitchen in view, but will check the semantic map coordinates where
    a kitchen was previously observed.
    """

    query: str = Field("", description="Text query to search for in the semantic map")

    limit: int = Field(1, description="Maximum number of results to return")
    distance: float = Field(0.3, description="Desired distance to maintain from object in meters")
    skip_visual_search: bool = Field(False, description="Skip visual search for object in view")
    timeout: float = Field(40.0, description="Maximum time to spend navigating in seconds")

    def __init__(self, robot=None, **data):
        """
        Initialize the Navigate skill.

        Args:
            robot: The robot instance
            **data: Additional data for configuration
        """
        super().__init__(robot=robot, **data)
        self._spatial_memory = None
        self._similarity_threshold = 0.23

    def _navigate_to_object(self):
        """
        Helper method that attempts to navigate to an object visible in the camera view.

        Returns:
            dict: Result dictionary with success status and details
        """
        logger.info(
            f"Attempting to navigate to visible object: {self.query} with desired distance {self.distance}m, timeout {self.timeout} seconds..."
        )

        # Try to get a bounding box from Qwen
        bbox = None
        try:
            # Get a single frame from the robot's camera
            frame = self._robot.get_single_rgb_frame().data
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            if frame is None:
                logger.error("Failed to get camera frame")
                return {
                    "success": False,
                    "failure_reason": "Perception",
                    "error": "Could not get camera frame",
                }
            bbox = get_bbox_from_qwen_frame(frame, object_name=self.query)
        except Exception as e:
            logger.error(f"Error getting frame or bbox: {e}")
            return {
                "success": False,
                "failure_reason": "Perception",
                "error": f"Error getting frame or bbox: {e}",
            }
        if bbox is None:
            logger.error(f"Failed to get bounding box for {self.query}")
            return {
                "success": False,
                "failure_reason": "Perception",
                "error": f"Could not find {self.query} in view",
            }

        logger.info(f"Found {self.query} at {bbox}")

        # Use the robot's navigate_to_object method
        success = self._robot.navigate_to_object(bbox, self.distance, self.timeout)

        if success:
            logger.info(f"Successfully navigated to {self.query}")
            return {
                "success": True,
                "failure_reason": None,
                "query": self.query,
                "message": f"Successfully navigated to {self.query} in view",
            }
        else:
            logger.warning(f"Failed to reach {self.query} within timeout")
            return {
                "success": False,
                "failure_reason": "Navigation",
                "error": f"Failed to reach {self.query} within timeout",
            }

    def _navigate_using_semantic_map(self):
        """
        Helper method that attempts to navigate using the semantic map query.

        Returns:
            dict: Result dictionary with success status and details
        """
        logger.info(f"Querying semantic map for: '{self.query}'")

        try:
            self._spatial_memory = self._robot.spatial_memory

            # Run the query
            results = self._spatial_memory.query_by_text(self.query, self.limit)

            if not results:
                logger.warning(f"No results found for query: '{self.query}'")
                return {
                    "success": False,
                    "query": self.query,
                    "error": "No matching location found in semantic map",
                }

            # Get the best match
            best_match = results[0]
            metadata = best_match.get("metadata", {})

            if isinstance(metadata, list) and metadata:
                metadata = metadata[0]

            # Extract coordinates from metadata
            if (
                isinstance(metadata, dict)
                and "pos_x" in metadata
                and "pos_y" in metadata
                and "rot_z" in metadata
            ):
                pos_x = metadata.get("pos_x", 0)
                pos_y = metadata.get("pos_y", 0)
                theta = metadata.get("rot_z", 0)

                # Calculate similarity score (distance is inverse of similarity)
                similarity = 1.0 - (
                    best_match.get("distance", 0) if best_match.get("distance") is not None else 0
                )

                logger.info(
                    f"Found match for '{self.query}' at ({pos_x:.2f}, {pos_y:.2f}, rotation {theta:.2f}) with similarity: {similarity:.4f}"
                )

                # Check if similarity is below the threshold
                if similarity < self._similarity_threshold:
                    logger.warning(
                        f"Match found but similarity score ({similarity:.4f}) is below threshold ({self._similarity_threshold})"
                    )
                    return {
                        "success": False,
                        "query": self.query,
                        "position": (pos_x, pos_y),
                        "rotation": theta,
                        "similarity": similarity,
                        "error": f"Match found but similarity score ({similarity:.4f}) is below threshold ({self._similarity_threshold})",
                    }

                # Create a PoseStamped for navigation
                goal_pose = PoseStamped(
                    position=Vector3(pos_x, pos_y, 0),
                    orientation=euler_to_quaternion(Vector3(0, 0, theta)),
                    frame_id="world",
                )

                logger.info(
                    f"Starting navigation to ({pos_x:.2f}, {pos_y:.2f}) with rotation {theta:.2f}"
                )

                # Use the robot's navigate_to method
                result = self._robot.navigate_to(goal_pose, blocking=True)

                if result:
                    logger.info("Navigation completed successfully")
                    return {
                        "success": True,
                        "query": self.query,
                        "position": (pos_x, pos_y),
                        "rotation": theta,
                        "similarity": similarity,
                        "metadata": metadata,
                    }
                else:
                    logger.error("Navigation did not complete successfully")
                    return {
                        "success": False,
                        "query": self.query,
                        "position": (pos_x, pos_y),
                        "rotation": theta,
                        "similarity": similarity,
                        "error": "Navigation did not complete successfully",
                    }
            else:
                logger.warning(f"No valid position data found for query: '{self.query}'")
                return {
                    "success": False,
                    "query": self.query,
                    "error": "No valid position data found in semantic map",
                }

        except Exception as e:
            logger.error(f"Error in semantic map navigation: {e}")
            return {"success": False, "error": f"Semantic map error: {e}"}

    def __call__(self):
        """
        First attempts to navigate to an object in view, then falls back to querying the semantic map.

        Returns:
            A dictionary with the result of the navigation attempt
        """
        super().__call__()

        if not self.query:
            error_msg = "No query provided to Navigate skill"
            logger.error(error_msg)
            return {"success": False, "error": error_msg}

        # First, try to find and navigate to the object in camera view
        logger.info(f"First attempting to find and navigate to visible object: '{self.query}'")

        if not self.skip_visual_search:
            object_result = self._navigate_to_object()

            if object_result and object_result["success"]:
                logger.info(f"Successfully navigated to {self.query} in view")
                return object_result

            elif object_result and object_result["failure_reason"] == "Navigation":
                logger.info(
                    f"Failed to navigate to {self.query} in view: {object_result.get('error', 'Unknown error')}"
                )
                return object_result

            # If object navigation failed, fall back to semantic map
            logger.info(
                f"Object not found in view. Falling back to semantic map query for: '{self.query}'"
            )

        return self._navigate_using_semantic_map()

    def stop(self):
        """
        Stop the navigation skill and clean up resources.

        Returns:
            A message indicating whether the navigation was stopped successfully
        """
        logger.info("Stopping Navigate skill")

        # Cancel navigation
        self._robot.cancel_navigation()

        skill_library = self._robot.get_skills()
        self.unregister_as_running("Navigate", skill_library)

        return "Navigate skill stopped successfully."


class GetPose(AbstractRobotSkill):
    """
    A skill that returns the current position and orientation of the robot.

    This skill is useful for getting the current pose of the robot in the map frame. You call this skill
    if you want to remember a location, for example, "remember this is where my favorite chair is" and then
    call this skill to get the position and rotation of approximately where the chair is. You can then use
    the position to navigate to the chair.

    When location_name is provided, this skill will also remember the current location with that name,
    allowing you to navigate back to it later using the Navigate skill.
    """

    location_name: str = Field(
        "", description="Optional name to assign to this location (e.g., 'kitchen', 'office')"
    )

    def __init__(self, robot=None, **data):
        """
        Initialize the GetPose skill.

        Args:
            robot: The robot instance
            **data: Additional data for configuration
        """
        super().__init__(robot=robot, **data)

    def __call__(self):
        """
        Get the current pose of the robot.

        Returns:
            A dictionary containing the position and rotation of the robot
        """
        super().__call__()

        if self._robot is None:
            error_msg = "No robot instance provided to GetPose skill"
            logger.error(error_msg)
            return {"success": False, "error": error_msg}

        try:
            # Get the current pose using the robot's get_pose method
            pose_data = self._robot.get_odom()

            # Extract position and rotation from the new dictionary format
            position = pose_data.position
            rotation = quaternion_to_euler(pose_data.orientation)

            # Format the response
            result = {
                "success": True,
                "position": {
                    "x": position.x,
                    "y": position.y,
                    "z": position.z,
                },
                "rotation": {"roll": rotation.x, "pitch": rotation.y, "yaw": rotation.z},
            }

            # If location_name is provided, remember this location
            if self.location_name:
                # Get the spatial memory instance
                spatial_memory = self._robot.spatial_memory

                # Create a RobotLocation object
                location = RobotLocation(
                    name=self.location_name,
                    position=(position.x, position.y, position.z),
                    rotation=(rotation.x, rotation.y, rotation.z),
                )

                # Add to spatial memory
                if spatial_memory.add_robot_location(location):
                    result["location_saved"] = True
                    result["location_name"] = self.location_name
                    logger.info(f"Location '{self.location_name}' saved at {position}")
                else:
                    result["location_saved"] = False
                    logger.error(f"Failed to save location '{self.location_name}'")

            return result
        except Exception as e:
            error_msg = f"Error getting robot pose: {e}"
            logger.error(error_msg)
            return {"success": False, "error": error_msg}


class NavigateToGoal(AbstractRobotSkill):
    """
    A skill that navigates the robot to a specified position and orientation.

    This skill uses the global planner to generate a path to the target position
    and then uses navigate_path_local to follow that path, achieving the desired
    orientation at the goal position.
    """

    position: Tuple[float, float] = Field(
        (0.0, 0.0), description="Target position (x, y) in map frame"
    )
    rotation: Optional[float] = Field(None, description="Target orientation (yaw) in radians")
    frame: str = Field("map", description="Reference frame for the position and rotation")
    timeout: float = Field(120.0, description="Maximum time (in seconds) allowed for navigation")

    def __init__(self, robot=None, **data):
        """
        Initialize the NavigateToGoal skill.

        Args:
            robot: The robot instance
            **data: Additional data for configuration
        """
        super().__init__(robot=robot, **data)

    def __call__(self):
        """
        Navigate to the specified goal position and orientation.

        Returns:
            A dictionary containing the result of the navigation attempt
        """
        super().__call__()

        if self._robot is None:
            error_msg = "No robot instance provided to NavigateToGoal skill"
            logger.error(error_msg)
            return {"success": False, "error": error_msg}

        skill_library = self._robot.get_skills()
        self.register_as_running("NavigateToGoal", skill_library)

        logger.info(
            f"Starting navigation to position=({self.position[0]:.2f}, {self.position[1]:.2f}) "
            f"with rotation={self.rotation if self.rotation is not None else 'None'} "
            f"in frame={self.frame}"
        )

        try:
            # Create a PoseStamped for navigation
            goal_pose = PoseStamped(
                position=Vector3(self.position[0], self.position[1], 0),
                orientation=euler_to_quaternion(Vector3(0, 0, self.rotation or 0)),
            )

            # Use the robot's navigate_to method
            result = self._robot.navigate_to(goal_pose, blocking=True)

            if result:
                logger.info("Navigation completed successfully")
                return {
                    "success": True,
                    "position": self.position,
                    "rotation": self.rotation,
                    "message": "Goal reached successfully",
                }
            else:
                logger.warning("Navigation did not complete successfully")
                return {
                    "success": False,
                    "position": self.position,
                    "rotation": self.rotation,
                    "message": "Goal could not be reached",
                }

        except Exception as e:
            error_msg = f"Error during navigation: {e}"
            logger.error(error_msg)
            return {
                "success": False,
                "position": self.position,
                "rotation": self.rotation,
                "error": error_msg,
            }
        finally:
            self.stop()

    def stop(self):
        """
        Stop the navigation.

        Returns:
            A message indicating that the navigation was stopped
        """
        logger.info("Stopping NavigateToGoal")
        skill_library = self._robot.get_skills()
        self.unregister_as_running("NavigateToGoal", skill_library)
        self._robot.cancel_navigation()
        return "Navigation stopped"


class Explore(AbstractRobotSkill):
    """
    A skill that performs autonomous frontier exploration.

    This skill continuously finds and navigates to unknown frontiers in the environment
    until no more frontiers are found or the exploration is stopped.

    Don't save GetPose locations when frontier exploring. Don't call any other skills except stop skill when needed.
    """

    timeout: float = Field(240.0, description="Maximum time (in seconds) allowed for exploration")

    def __init__(self, robot=None, **data):
        """
        Initialize the Explore skill.

        Args:
            robot: The robot instance
            **data: Additional data for configuration
        """
        super().__init__(robot=robot, **data)

    def __call__(self):
        """
        Start autonomous frontier exploration.

        Returns:
            A dictionary containing the result of the exploration
        """
        super().__call__()

        if self._robot is None:
            error_msg = "No robot instance provided to Explore skill"
            logger.error(error_msg)
            return {"success": False, "error": error_msg}

        skill_library = self._robot.get_skills()
        self.register_as_running("Explore", skill_library)

        logger.info("Starting autonomous frontier exploration")

        try:
            # Start exploration using the robot's explore method
            result = self._robot.explore()

            if result:
                logger.info("Exploration started successfully")

                # Wait for exploration to complete or timeout
                start_time = time.time()
                while time.time() - start_time < self.timeout:
                    time.sleep(0.5)

                # Timeout reached, stop exploration
                logger.info(f"Exploration timeout reached after {self.timeout} seconds")
                self._robot.stop_exploration()
                return {
                    "success": True,
                    "message": f"Exploration ran for {self.timeout} seconds",
                }
            else:
                logger.warning("Failed to start exploration")
                return {
                    "success": False,
                    "message": "Failed to start exploration",
                }

        except Exception as e:
            error_msg = f"Error during exploration: {e}"
            logger.error(error_msg)
            return {
                "success": False,
                "error": error_msg,
            }
        finally:
            self.stop()

    def stop(self):
        """
        Stop the exploration.

        Returns:
            A message indicating that the exploration was stopped
        """
        logger.info("Stopping Explore")
        skill_library = self._robot.get_skills()
        self.unregister_as_running("Explore", skill_library)

        # Stop the robot's exploration if it's running
        try:
            self._robot.stop_exploration()
        except Exception as e:
            logger.error(f"Error stopping exploration: {e}")

        return "Exploration stopped"
