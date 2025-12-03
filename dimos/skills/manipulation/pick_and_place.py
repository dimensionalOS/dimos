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
Pick and place skill for Piper Arm robot.

This module provides a skill that uses Qwen VLM to identify pick and place
locations based on natural language queries, then executes the manipulation.
"""

import json
import os
from typing import Any

import cv2
import numpy as np
from pydantic import Field

from dimos.models.qwen.video_query import query_single_frame
from dimos.skills.skills import AbstractRobotSkill
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def parse_qwen_points_response(response: str) -> tuple[tuple[int, int], tuple[int, int]] | None:
    """
    Parse Qwen's response containing two points.

    Args:
        response: Qwen's response containing JSON with two points

    Returns:
        Tuple of (pick_point, place_point) where each point is (x, y), or None if parsing fails
    """
    try:
        # Try to extract JSON from the response
        start_idx = response.find("{")
        end_idx = response.rfind("}") + 1

        if start_idx >= 0 and end_idx > start_idx:
            json_str = response[start_idx:end_idx]
            result = json.loads(json_str)

            # Extract pick and place points
            if "pick_point" in result and "place_point" in result:
                pick = result["pick_point"]
                place = result["place_point"]

                # Validate points have x,y coordinates
                if (
                    isinstance(pick, list | tuple)
                    and len(pick) >= 2
                    and isinstance(place, list | tuple)
                    and len(place) >= 2
                ):
                    return (int(pick[0]), int(pick[1])), (int(place[0]), int(place[1]))

    except Exception as e:
        logger.error(f"Error parsing Qwen points response: {e}")
        logger.debug(f"Raw response: {response}")

    return None


def save_debug_image_with_points(
    image: np.ndarray,  # type: ignore[type-arg]
    pick_point: tuple[int, int] | None = None,
    place_point: tuple[int, int] | None = None,
    filename_prefix: str = "qwen_debug",
) -> str:
    """
    Save debug image with crosshairs marking pick and/or place points.

    Args:
        image: RGB image array
        pick_point: (x, y) coordinates for pick location
        place_point: (x, y) coordinates for place location
        filename_prefix: Prefix for the saved filename

    Returns:
        Path to the saved image
    """
    # Create a copy to avoid modifying original
    debug_image = image.copy()

    # Draw pick point crosshair (green)
    if pick_point:
        x, y = pick_point
        # Draw crosshair
        cv2.drawMarker(debug_image, (x, y), (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
        # Draw circle
        cv2.circle(debug_image, (x, y), 5, (0, 255, 0), -1)
        # Add label
        cv2.putText(
            debug_image, "PICK", (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )

    # Draw place point crosshair (cyan)
    if place_point:
        x, y = place_point
        # Draw crosshair
        cv2.drawMarker(debug_image, (x, y), (255, 255, 0), cv2.MARKER_CROSS, 30, 2)
        # Draw circle
        cv2.circle(debug_image, (x, y), 5, (255, 255, 0), -1)
        # Add label
        cv2.putText(
            debug_image, "PLACE", (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2
        )

    # Draw arrow from pick to place if both exist
    if pick_point and place_point:
        cv2.arrowedLine(debug_image, pick_point, place_point, (255, 0, 255), 2, tipLength=0.03)

    # Generate filename with timestamp
    filename = f"{filename_prefix}.png"
    filepath = os.path.join(os.getcwd(), filename)

    # Save image
    cv2.imwrite(filepath, debug_image)
    logger.info(f"Debug image saved to: {filepath}")

    return filepath


def parse_qwen_single_point_response(response: str) -> tuple[int, int] | None:
    """
    Parse Qwen's response containing a single point.

    Args:
        response: Qwen's response containing JSON with a point

    Returns:
        Tuple of (x, y) or None if parsing fails
    """
    try:
        # Try to extract JSON from the response
        start_idx = response.find("{")
        end_idx = response.rfind("}") + 1

        if start_idx >= 0 and end_idx > start_idx:
            json_str = response[start_idx:end_idx]
            result = json.loads(json_str)

            # Try different possible keys
            point = None
            for key in ["point", "location", "position", "coordinates"]:
                if key in result:
                    point = result[key]
                    break

            # Validate point has x,y coordinates
            if point and isinstance(point, list | tuple) and len(point) >= 2:
                return int(point[0]), int(point[1])

    except Exception as e:
        logger.error(f"Error parsing Qwen single point response: {e}")
        logger.debug(f"Raw response: {response}")

    return None


class PickAndPlace(AbstractRobotSkill):
    """
    A skill that performs pick and place operations using vision-language guidance.

    This skill uses Qwen VLM to identify objects and locations based on natural
    language queries, then executes pick and place operations using the robot's
    manipulation interface.

    Example usage:
        # Just pick the object
        skill = PickAndPlace(robot=robot, object_query="red mug")

        # Pick and place the object
        skill = PickAndPlace(robot=robot, object_query="red mug", target_query="on the coaster")

    The skill uses the robot's stereo camera to capture RGB images and its manipulation
    interface to execute the pick and place operation. It automatically handles coordinate
    transformation from 2D pixel coordinates to 3D world coordinates.
    """

    object_query: str = Field(
        "mug",
        description="Natural language description of the object to pick (e.g., 'red mug', 'small box')",
    )

    target_query: str | None = Field(
        None,
        description="Natural language description of where to place the object (e.g., 'on the table', 'in the basket'). If not provided, only pick operation will be performed.",
    )

    model_name: str = Field(
        "qwen2.5-vl-72b-instruct", description="Qwen model to use for visual queries"
    )

    def __init__(self, robot=None, **data) -> None:  # type: ignore[no-untyped-def]
        """
        Initialize the PickAndPlace skill.

        Args:
            robot: The PiperArmRobot instance
            **data: Additional configuration data
        """
        super().__init__(robot=robot, **data)

    def _get_camera_frame(self) -> np.ndarray | None:  # type: ignore[type-arg]
        """
        Get a single RGB frame from the robot's camera.

        Returns:
            RGB image as numpy array or None if capture fails
        """
        if not self._robot or not self._robot.manipulation_interface:  # type: ignore[attr-defined]
            logger.error("Robot or stereo camera not available")
            return None

        try:
            # Use the RPC call to get a single RGB frame
            rgb_frame = self._robot.manipulation_interface.get_single_rgb_frame()  # type: ignore[attr-defined]
            if rgb_frame is None:
                logger.error("Failed to capture RGB frame from camera")
            return rgb_frame  # type: ignore[no-any-return]
        except Exception as e:
            logger.error(f"Error getting camera frame: {e}")
            return None

    def _query_pick_and_place_points(
        self,
        frame: np.ndarray,  # type: ignore[type-arg]
    ) -> tuple[tuple[int, int], tuple[int, int]] | None:
        """
        Query Qwen to get both pick and place points in a single query.

        Args:
            frame: RGB image array

        Returns:
            Tuple of (pick_point, place_point) or None if query fails
        """
        # This method is only called when both object and target are specified
        prompt = (
            f"Look at this image carefully. I need you to identify two specific locations:\n"
            f"1. Find the {self.object_query} - this is the object I want to pick up\n"
            f"2. Identify where to place it {self.target_query}\n\n"
            "Instructions:\n"
            "- The pick_point should be at the center or graspable part of the object\n"
            "- The place_point should be a stable, flat surface at the target location\n"
            "- Consider the object's size when choosing the placement point\n\n"
            "Return ONLY a JSON object with this exact format:\n"
            "{'pick_point': [x, y], 'place_point': [x, y]}\n"
            "where [x, y] are pixel coordinates in the image."
        )

        try:
            response = query_single_frame(frame, prompt, model_name=self.model_name)
            return parse_qwen_points_response(response)
        except Exception as e:
            logger.error(f"Error querying Qwen for pick and place points: {e}")
            return None

    def _query_single_point(
        self,
        frame: np.ndarray,  # type: ignore[type-arg]
        query: str,
        point_type: str,
    ) -> tuple[int, int] | None:
        """
        Query Qwen to get a single point location.

        Args:
            frame: RGB image array
            query: Natural language description of what to find
            point_type: Type of point ('pick' or 'place') for context

        Returns:
            Tuple of (x, y) pixel coordinates or None if query fails
        """
        if point_type == "pick":
            prompt = (
                f"Look at this image carefully and find the {query}.\n\n"
                "Instructions:\n"
                "- Identify the exact object matching the description\n"
                "- Choose the center point or the most graspable location on the object\n"
                "- If multiple matching objects exist, choose the most prominent or accessible one\n"
                "- Consider the object's shape and material when selecting the grasp point\n\n"
                "Return ONLY a JSON object with this exact format:\n"
                "{'point': [x, y]}\n"
                "where [x, y] are the pixel coordinates of the optimal grasping point on the object."
            )
        else:  # place
            prompt = (
                f"Look at this image and identify where to place an object {query}.\n\n"
                "Instructions:\n"
                "- Find a stable, flat surface at the specified location\n"
                "- Ensure the placement spot is clear of obstacles\n"
                "- Consider the size of the object being placed\n"
                "- If the query specifies a container or specific spot, center the placement there\n"
                "- Otherwise, find the most appropriate nearby surface\n\n"
                "Return ONLY a JSON object with this exact format:\n"
                "{'point': [x, y]}\n"
                "where [x, y] are the pixel coordinates of the optimal placement location."
            )

        try:
            response = query_single_frame(frame, prompt, model_name=self.model_name)
            return parse_qwen_single_point_response(response)
        except Exception as e:
            logger.error(f"Error querying Qwen for {point_type} point: {e}")
            return None

    def __call__(self) -> dict[str, Any]:
        """
        Execute the pick and place operation.

        Returns:
            Dictionary with operation results
        """
        super().__call__()  # type: ignore[no-untyped-call]

        if not self._robot:
            error_msg = "No robot instance provided to PickAndPlace skill"
            logger.error(error_msg)
            return {"success": False, "error": error_msg}

        # Register skill as running
        skill_library = self._robot.get_skills()  # type: ignore[no-untyped-call]
        self.register_as_running("PickAndPlace", skill_library)

        # Get camera frame
        frame = self._get_camera_frame()
        if frame is None:
            return {"success": False, "error": "Failed to capture camera frame"}

        # Convert RGB to BGR for OpenCV if needed
        if len(frame.shape) == 3 and frame.shape[2] == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Get pick and place points from Qwen
        pick_point = None
        place_point = None

        # Determine mode based on whether target_query is provided
        if self.target_query is None:
            # Pick only mode
            logger.info("Pick-only mode (no target specified)")

            # Query for pick point
            pick_point = self._query_single_point(frame, self.object_query, "pick")
            if not pick_point:
                return {"success": False, "error": f"Failed to find {self.object_query}"}

            # No place point needed for pick-only
            place_point = None
        else:
            # Pick and place mode - can use either single or dual query
            logger.info("Pick and place mode (target specified)")

            # Try single query first for efficiency
            points = self._query_pick_and_place_points(frame)
            pick_point, place_point = points  # type: ignore[misc]

        logger.info(f"Pick point: {pick_point}, Place point: {place_point}")

        # Save debug image with marked points
        if pick_point or place_point:
            save_debug_image_with_points(frame, pick_point, place_point)

        # Execute pick (and optionally place) using the robot's interface
        try:
            if place_point:
                # Pick and place
                result = self._robot.pick_and_place(  # type: ignore[attr-defined]
                    pick_x=pick_point[0],
                    pick_y=pick_point[1],
                    place_x=place_point[0],
                    place_y=place_point[1],
                )
            else:
                # Pick only
                result = self._robot.pick_and_place(  # type: ignore[attr-defined]
                    pick_x=pick_point[0], pick_y=pick_point[1], place_x=None, place_y=None
                )

            if result:
                if self.target_query:
                    message = (
                        f"Successfully picked {self.object_query} and placed it {self.target_query}"
                    )
                else:
                    message = f"Successfully picked {self.object_query}"

                return {
                    "success": True,
                    "pick_point": pick_point,
                    "place_point": place_point,
                    "object": self.object_query,
                    "target": self.target_query,
                    "message": message,
                }
            else:
                operation = "Pick and place" if self.target_query else "Pick"
                return {
                    "success": False,
                    "pick_point": pick_point,
                    "place_point": place_point,
                    "error": f"{operation} operation failed",
                }

        except Exception as e:
            logger.error(f"Error executing pick and place: {e}")
            return {
                "success": False,
                "error": f"Execution error: {e!s}",
                "pick_point": pick_point,
                "place_point": place_point,
            }
        finally:
            # Always unregister skill when done
            self.stop()

    def stop(self) -> None:
        """
        Stop the pick and place operation and perform cleanup.
        """
        logger.info("Stopping PickAndPlace skill")

        # Unregister skill from skill library
        if self._robot:
            skill_library = self._robot.get_skills()  # type: ignore[no-untyped-call]
            self.unregister_as_running("PickAndPlace", skill_library)

        logger.info("PickAndPlace skill stopped successfully")
