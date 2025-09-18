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
Navigation skills for agents2 framework.
These provide navigation capabilities using the new @skill decorator.
"""

import os
import time
from typing import Optional, Tuple
import cv2

from dimos.protocol.skill.skill import SkillContainer, skill
from dimos.protocol.skill.type import Return, Stream
from dimos.types.robot_location import RobotLocation
from dimos.models.qwen.video_query import get_bbox_from_qwen_frame
from dimos.msgs.geometry_msgs import PoseStamped, Vector3
from dimos.utils.transform_utils import euler_to_quaternion, quaternion_to_euler
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.agents2.skills.navigation")


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


class NavigationSkillContainer(SkillContainer):
    """Container for navigation skills using the agents2 framework."""

    def __init__(self, robot=None):
        """Initialize the navigation skill container.

        Args:
            robot: The robot instance to use for navigation
        """
        super().__init__()
        self._robot = robot
        self._spatial_memory = None
        self._similarity_threshold = 0.23

    def _navigate_to_object_sync(self, query: str, distance: float, timeout: float):
        """
        Helper method that attempts to navigate to an object visible in the camera view.

        Returns:
            dict: Result dictionary with success status and details
        """
        logger.info(
            f"Attempting to navigate to visible object: {query} with desired distance {distance}m, timeout {timeout} seconds..."
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
            bbox = get_bbox_from_qwen_frame(frame, object_name=query)
        except Exception as e:
            logger.error(f"Error getting frame or bbox: {e}")
            return {
                "success": False,
                "failure_reason": "Perception",
                "error": f"Error getting frame or bbox: {e}",
            }
        if bbox is None:
            logger.error(f"Failed to get bounding box for {query}")
            return {
                "success": False,
                "failure_reason": "Perception",
                "error": f"Could not find {query} in view",
            }

        logger.info(f"Found {query} at {bbox}")

        # Use the robot's navigate_to_object method
        success = self._robot.navigate_to_object(bbox, distance, timeout)

        if success:
            logger.info(f"Successfully navigated to {query}")
            return {
                "success": True,
                "failure_reason": None,
                "query": query,
                "message": f"Successfully navigated to {query} in view",
            }
        else:
            logger.warning(f"Failed to reach {query} within timeout")
            return {
                "success": False,
                "failure_reason": "Navigation",
                "error": f"Failed to reach {query} within timeout",
            }

    def _navigate_using_semantic_map_sync(self, query: str, limit: int):
        """
        Helper method that attempts to navigate using the semantic map query.

        Returns:
            dict: Result dictionary with success status and details
        """
        logger.info(f"Querying semantic map for: '{query}'")

        try:
            self._spatial_memory = self._robot.spatial_memory

            # Run the query
            results = self._spatial_memory.query_by_text(query, limit)

            if not results:
                logger.warning(f"No results found for query: '{query}'")
                return {
                    "success": False,
                    "query": query,
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
                    f"Found match for '{query}' at ({pos_x:.2f}, {pos_y:.2f}, rotation {theta:.2f}) with similarity: {similarity:.4f}"
                )

                # Check if similarity is below the threshold
                if similarity < self._similarity_threshold:
                    logger.warning(
                        f"Match found but similarity score ({similarity:.4f}) is below threshold ({self._similarity_threshold})"
                    )
                    return {
                        "success": False,
                        "query": query,
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

                # Get current robot position for debugging
                try:
                    current_pose = self._robot.get_odom()
                    logger.info(
                        f"Current robot position: ({current_pose.position.x:.2f}, {current_pose.position.y:.2f})"
                    )
                    distance_to_goal = (
                        (pos_x - current_pose.position.x) ** 2
                        + (pos_y - current_pose.position.y) ** 2
                    ) ** 0.5
                    logger.info(f"Distance to goal: {distance_to_goal:.2f}m")
                except Exception as e:
                    logger.warning(f"Could not get current position: {e}")

                # Use the robot's navigate_to method
                logger.info(f"Calling navigate_to with goal_pose: {goal_pose}")

                # Check navigator state before starting
                try:
                    initial_state = self._robot.navigator.get_state()
                    logger.info(f"Navigator initial state: {initial_state}")
                except Exception as e:
                    logger.warning(f"Could not get initial navigator state: {e}")

                # Try navigation
                result = self._robot.navigate_to(goal_pose, blocking=True)
                logger.info(f"Navigation result: {result}")

                # Check final navigator state
                try:
                    final_state = self._robot.navigator.get_state()
                    logger.info(f"Navigator final state: {final_state}")
                except Exception as e:
                    logger.warning(f"Could not get final navigator state: {e}")

                if result:
                    logger.info("Navigation completed successfully")
                    return {
                        "success": True,
                        "query": query,
                        "position": (pos_x, pos_y),
                        "rotation": theta,
                        "similarity": similarity,
                        "metadata": metadata,
                    }
                else:
                    logger.error(
                        "Navigation did not complete successfully - checking navigator state"
                    )
                    # Try to get more information about why navigation failed
                    try:
                        nav_state = self._robot.navigator.get_state()
                        logger.error(f"Navigator state: {nav_state}")
                        goal_reached = self._robot.navigator.is_goal_reached()
                        logger.error(f"Goal reached: {goal_reached}")
                    except Exception as e:
                        logger.error(f"Could not get navigator state: {e}")

                    return {
                        "success": False,
                        "query": query,
                        "position": (pos_x, pos_y),
                        "rotation": theta,
                        "similarity": similarity,
                        "error": "Navigation did not complete successfully - goal may be unreachable or navigator failed",
                    }
            else:
                logger.warning(f"No valid position data found for query: '{query}'")
                return {
                    "success": False,
                    "query": query,
                    "error": "No valid position data found in semantic map",
                }

        except Exception as e:
            logger.error(f"Error in semantic map navigation: {e}")
            return {"success": False, "error": f"Semantic map error: {e}"}

    @skill(ret=Return.call_agent, stream=Stream.none)
    def navigate_with_text(
        self,
        query: str = "",
        limit: int = 1,
        distance: float = 0.3,
        skip_visual_search: bool = False,
        timeout: float = 40.0,
    ) -> str:
        """Query an existing semantic map using natural language or navigate to an object in view.

        First attempts to locate an object in the robot's camera view using vision.
        If the object is found, navigates to it. If not, falls back to querying the
        semantic map for a location matching the description.

        CALL THIS SKILL FOR ONE SUBJECT AT A TIME. For example: "Go to the person wearing a blue shirt in the living room",
        you should call this skill twice, once for the person wearing a blue shirt and once for the living room.

        If skip_visual_search is True, this skill will skip the visual search for the object in view.
        This is useful if you want to navigate to a general location such as a kitchen or office.

        Args:
            query: Text query to search for in the semantic map
            limit: Maximum number of results to return
            distance: Desired distance to maintain from object in meters
            skip_visual_search: Skip visual search for object in view
            timeout: Maximum time to spend navigating in seconds
        """
        try:
            if not query:
                error_msg = "No query provided to navigate_with_text skill"
                logger.error(error_msg)
                return str({"success": False, "error": error_msg})

            # First, try to find and navigate to the object in camera view
            logger.info(f"First attempting to find and navigate to visible object: '{query}'")

            if not skip_visual_search:
                object_result = self._navigate_to_object_sync(query, distance, timeout)

                if object_result and object_result["success"]:
                    logger.info(f"Successfully navigated to {query} in view")
                    return str(object_result)

                elif object_result and object_result["failure_reason"] == "Navigation":
                    logger.info(
                        f"Failed to navigate to {query} in view: {object_result.get('error', 'Unknown error')}"
                    )
                    return str(object_result)

                # If object navigation failed, fall back to semantic map
                logger.info(
                    f"Object not found in view. Falling back to semantic map query for: '{query}'"
                )

            result = self._navigate_using_semantic_map_sync(query, limit)
            return str(result)

        except Exception as e:
            logger.error(f"Error in navigate_with_text: {e}")
            return f"Error navigating with text: {str(e)}"

    @skill(ret=Return.call_agent)
    def get_pose(self, location_name: str = "") -> str:
        """Get the robot's current pose (position and orientation).

        Returns the robot's current location including x, y, z coordinates
        and orientation as roll, pitch, yaw.

        When location_name is provided, this skill will also remember the current location with that name,
        allowing you to navigate back to it later using the navigate_with_text skill.

        Args:
            location_name: Optional name to assign to this location (e.g., 'kitchen', 'office')
        """
        try:
            if self._robot is None:
                error_msg = "No robot instance provided to get_pose skill"
                logger.error(error_msg)
                return str({"success": False, "error": error_msg})

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
            if location_name:
                # Get the spatial memory instance
                spatial_memory = self._robot.spatial_memory

                # Create a RobotLocation object
                location = RobotLocation(
                    name=location_name,
                    position=(position.x, position.y, position.z),
                    rotation=(rotation.x, rotation.y, rotation.z),
                )

                # Add to spatial memory
                if spatial_memory.add_robot_location(location):
                    result["location_saved"] = True
                    result["location_name"] = location_name
                    logger.info(f"Location '{location_name}' saved at {position}")
                else:
                    result["location_saved"] = False
                    logger.error(f"Failed to save location '{location_name}'")

            return str(result)
        except Exception as e:
            error_msg = f"Error getting robot pose: {e}"
            logger.error(error_msg)
            return str({"success": False, "error": error_msg})

    @skill(ret=Return.call_agent, stream=Stream.none)
    def navigate_to_goal(
        self, x: float, y: float, theta: Optional[float] = None, timeout: float = 40.0
    ) -> str:
        """Navigate to a specific goal position.

        Args:
            x: X coordinate of the goal in meters
            y: Y coordinate of the goal in meters
            theta: Orientation at goal in radians (optional)
            timeout: Maximum time to spend navigating in seconds
        """
        try:
            logger.info(
                f"Starting navigation to position=({x:.2f}, {y:.2f}) "
                f"with rotation={theta if theta is not None else 'None'}"
            )

            # Create a PoseStamped for navigation
            goal_pose = PoseStamped(
                position=Vector3(x, y, 0),
                orientation=euler_to_quaternion(Vector3(0, 0, theta or 0)),
            )

            # Use the robot's navigate_to method
            result = self._robot.navigate_to(goal_pose, blocking=True)

            if result:
                logger.info("Navigation completed successfully")
                return str(
                    {
                        "success": True,
                        "position": (x, y),
                        "rotation": theta,
                        "message": "Goal reached successfully",
                    }
                )
            else:
                logger.warning("Navigation did not complete successfully")
                return str(
                    {
                        "success": False,
                        "position": (x, y),
                        "rotation": theta,
                        "message": "Goal could not be reached",
                    }
                )

        except Exception as e:
            error_msg = f"Error during navigation: {e}"
            logger.error(error_msg)
            return str(
                {
                    "success": False,
                    "position": (x, y),
                    "rotation": theta,
                    "error": error_msg,
                }
            )

    @skill(ret=Return.call_agent, stream=Stream.none)
    def explore(self, strategy: str = "random", duration: float = 30.0) -> str:
        """Explore the environment using a specified strategy.

        Don't save get_pose locations when frontier exploring. Don't call any other skills except stop skill when needed.

        Args:
            strategy: Exploration strategy ('random', 'frontier', 'systematic')
            duration: How long to explore in seconds
        """
        try:
            logger.info("Starting autonomous frontier exploration")

            # Start exploration using the robot's explore method
            result = self._robot.explore()

            if result:
                logger.info("Exploration started successfully")

                # Wait for exploration to complete or timeout
                start_time = time.time()
                while time.time() - start_time < duration:
                    time.sleep(0.5)

                # Timeout reached, stop exploration
                logger.info(f"Exploration timeout reached after {duration} seconds")
                self._robot.stop_exploration()
                return str(
                    {
                        "success": True,
                        "message": f"Exploration ran for {duration} seconds",
                    }
                )
            else:
                logger.warning("Failed to start exploration")
                return str(
                    {
                        "success": False,
                        "message": "Failed to start exploration",
                    }
                )

        except Exception as e:
            error_msg = f"Error during exploration: {e}"
            logger.error(error_msg)
            return str(
                {
                    "success": False,
                    "error": error_msg,
                }
            )
