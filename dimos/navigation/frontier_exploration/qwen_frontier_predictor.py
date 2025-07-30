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
Qwen-based frontier exploration goal predictor using vision language model.

This module provides a frontier goal detector that uses Qwen's vision capabilities
to analyze costmap images and predict optimal exploration goals.
"""

import os
import glob
import json
import re
from typing import Optional, List, Tuple

import numpy as np
from PIL import Image, ImageDraw

from dimos.types.costmap import Costmap
from dimos.types.vector import Vector
from dimos.models.qwen.video_query import query_single_frame
from dimos.robot.frontier_exploration.utils import (
    costmap_to_pil_image,
    smooth_costmap_for_frontiers,
)


class QwenFrontierPredictor:
    """
    Qwen-based frontier exploration goal predictor.

    Uses Qwen's vision capabilities to analyze costmap images and predict
    optimal exploration goals based on visual understanding of the map structure.
    """

    def __init__(
        self,
        api_key: Optional[str] = None,
        model_name: str = "qwen2.5-vl-72b-instruct",
        use_smoothed_costmap: bool = True,
        image_scale_factor: int = 4,
    ):
        """
        Initialize the Qwen frontier predictor.

        Args:
            api_key: Alibaba API key for Qwen access
            model_name: Qwen model to use for predictions
            image_scale_factor: Scale factor for image processing
        """
        self.api_key = api_key or os.getenv("ALIBABA_API_KEY")
        if not self.api_key:
            raise ValueError(
                "Alibaba API key must be provided or set in ALIBABA_API_KEY environment variable"
            )

        self.model_name = model_name
        self.image_scale_factor = image_scale_factor
        self.use_smoothed_costmap = use_smoothed_costmap

        # Storage for previously explored goals
        self.explored_goals: List[Vector] = []

    def _world_to_image_coords(self, world_pos: Vector, costmap: Costmap) -> Tuple[int, int]:
        """Convert world coordinates to image pixel coordinates."""
        grid_pos = costmap.world_to_grid(world_pos)
        img_x = int(grid_pos.x * self.image_scale_factor)
        img_y = int((costmap.height - grid_pos.y) * self.image_scale_factor)  # Flip Y
        return img_x, img_y

    def _image_to_world_coords(self, img_x: int, img_y: int, costmap: Costmap) -> Vector:
        """Convert image pixel coordinates to world coordinates."""
        # Unscale and flip Y coordinate
        grid_x = img_x / self.image_scale_factor
        grid_y = costmap.height - (img_y / self.image_scale_factor)

        # Convert grid to world coordinates
        world_pos = costmap.grid_to_world(Vector([grid_x, grid_y]))
        return world_pos

    def _draw_goals_on_image(
        self,
        image: Image.Image,
        robot_pose: Vector,
        costmap: Costmap,
        latest_goal: Optional[Vector] = None,
    ) -> Image.Image:
        """
        Draw explored goals and robot position on the costmap image.

        Args:
            image: PIL Image to draw on
            robot_pose: Current robot position
            costmap: Costmap for coordinate conversion
            latest_goal: Latest predicted goal to highlight in red

        Returns:
            PIL Image with goals drawn
        """
        img_copy = image.copy()
        draw = ImageDraw.Draw(img_copy)

        # Draw previously explored goals as green dots
        for explored_goal in self.explored_goals:
            x, y = self._world_to_image_coords(explored_goal, costmap)
            radius = 8
            draw.ellipse(
                [x - radius, y - radius, x + radius, y + radius],
                fill=(0, 255, 0),
                outline=(0, 128, 0),
                width=2,
            )

        # Draw robot position as blue dot
        robot_x, robot_y = self._world_to_image_coords(robot_pose, costmap)
        robot_radius = 10
        draw.ellipse(
            [
                robot_x - robot_radius,
                robot_y - robot_radius,
                robot_x + robot_radius,
                robot_y + robot_radius,
            ],
            fill=(0, 0, 255),
            outline=(0, 0, 128),
            width=3,
        )

        # Draw latest predicted goal as red dot
        if latest_goal:
            goal_x, goal_y = self._world_to_image_coords(latest_goal, costmap)
            goal_radius = 12
            draw.ellipse(
                [
                    goal_x - goal_radius,
                    goal_y - goal_radius,
                    goal_x + goal_radius,
                    goal_y + goal_radius,
                ],
                fill=(255, 0, 0),
                outline=(128, 0, 0),
                width=3,
            )

        return img_copy

    def _create_vision_prompt(self) -> str:
        """Create the vision prompt for Qwen model."""
        prompt = """You are an expert robot navigation system analyzing a costmap for frontier exploration.

COSTMAP LEGEND:
- Light gray pixels (205,205,205): FREE SPACE - areas the robot can navigate
- Dark gray pixels (128,128,128): UNKNOWN SPACE - unexplored areas that need exploration  
- Black pixels (0,0,0): OBSTACLES - walls, furniture, blocked areas
- Blue dot: CURRENT ROBOT POSITION
- Green dots: PREVIOUSLY EXPLORED GOALS - avoid these areas

TASK: Find the best frontier exploration goal by identifying the optimal point where:
1. It's at the boundary between FREE SPACE (light gray) and UNKNOWN SPACE (dark gray) (HIGHEST Priority)
2. It's reasonably far from the robot position (blue dot) (MEDIUM Priority)
3. It's reasonably far from previously explored goals (green dots) (MEDIUM Priority)
4. It leads to a large area of unknown space to explore (HIGH Priority)
5. It's accessible from the robot's current position through free space (MEDIUM Priority)
6. It's not near or on obstacles (HIGHEST Priority)

RESPONSE FORMAT: Return ONLY the pixel coordinates as a JSON object:
{"x": pixel_x_coordinate, "y": pixel_y_coordinate, "reasoning": "brief explanation"}

Example: {"x": 245, "y": 187, "reasoning": "Large unknown area to the north, good distance from robot and previous goals"}

Analyze the image and identify the single best frontier exploration goal."""

        return prompt

    def _parse_prediction_response(self, response: str) -> Optional[Tuple[int, int, str]]:
        """
        Parse the model's response to extract coordinates and reasoning.

        Args:
            response: Raw response from Qwen model

        Returns:
            Tuple of (x, y, reasoning) or None if parsing failed
        """
        try:
            # Try to find JSON object in response
            json_match = re.search(r"\{[^}]*\}", response)
            if json_match:
                json_str = json_match.group()
                data = json.loads(json_str)

                if "x" in data and "y" in data:
                    x = int(data["x"])
                    y = int(data["y"])
                    reasoning = data.get("reasoning", "No reasoning provided")
                    return (x, y, reasoning)

            # Fallback: try to extract coordinates with regex
            coord_match = re.search(r"[^\d]*(\d+)[^\d]+(\d+)", response)
            if coord_match:
                x = int(coord_match.group(1))
                y = int(coord_match.group(2))
                return (x, y, "Coordinates extracted from response")

        except (json.JSONDecodeError, ValueError, KeyError) as e:
            print(f"DEBUG: Failed to parse prediction response: {e}")

        return None

    def get_exploration_goal(self, robot_pose: Vector, costmap: Costmap) -> Optional[Vector]:
        """
        Get the best exploration goal using Qwen vision analysis.

        Args:
            robot_pose: Current robot position in world coordinates
            costmap: Current costmap for analysis

        Returns:
            Single best frontier goal in world coordinates, or None if no suitable goal found
        """
        print(
            f"DEBUG: Qwen frontier prediction starting with {len(self.explored_goals)} explored goals"
        )

        # Create costmap image
        if self.use_smoothed_costmap:
            costmap = smooth_costmap_for_frontiers(costmap, alpha=4.0)

        base_image = costmap_to_pil_image(costmap, self.image_scale_factor)

        # Draw goals on image (without latest goal initially)
        annotated_image = self._draw_goals_on_image(base_image, robot_pose, costmap)

        # Query Qwen model for frontier prediction
        try:
            prompt = self._create_vision_prompt()

            # Convert PIL image to numpy array for query_single_frame
            annotated_array = np.array(annotated_image)

            response = query_single_frame(
                annotated_array, prompt, api_key=self.api_key, model_name=self.model_name
            )

            print(f"DEBUG: Qwen response: {response}")

            # Parse response to get coordinates
            parsed_result = self._parse_prediction_response(response)
            if not parsed_result:
                print("DEBUG: Failed to parse Qwen response")
                return None

            img_x, img_y, reasoning = parsed_result
            print(f"DEBUG: Parsed coordinates: ({img_x}, {img_y}), Reasoning: {reasoning}")

            # Convert image coordinates to world coordinates
            predicted_goal = self._image_to_world_coords(img_x, img_y, costmap)
            print(
                f"DEBUG: Predicted goal in world coordinates: ({predicted_goal.x:.2f}, {predicted_goal.y:.2f})"
            )

            # Store the goal in explored_goals for future reference
            self.explored_goals.append(predicted_goal)

            print(f"DEBUG: Successfully predicted frontier goal: {predicted_goal}")
            return predicted_goal

        except Exception as e:
            print(f"DEBUG: Error during Qwen prediction: {e}")
            return None


def test_qwen_frontier_detection():
    """
    Visual test for Qwen frontier detection using saved costmaps.
    Shows frontier detection results with Qwen predictions.
    """

    # Path to saved costmaps
    saved_maps_dir = os.path.join(os.getcwd(), "assets", "saved_maps")

    if not os.path.exists(saved_maps_dir):
        print(f"Error: Saved maps directory not found: {saved_maps_dir}")
        return

    # Get all pickle files
    pickle_files = sorted(glob.glob(os.path.join(saved_maps_dir, "*.pickle")))

    if not pickle_files:
        print(f"No pickle files found in {saved_maps_dir}")
        return

    print(f"Found {len(pickle_files)} costmap files for Qwen testing")

    # Initialize Qwen frontier predictor
    predictor = QwenFrontierPredictor(image_scale_factor=4, use_smoothed_costmap=False)

    # Track the robot pose across iterations
    robot_pose = None

    # Process each costmap
    for i, pickle_file in enumerate(pickle_files):
        print(
            f"\n--- Processing costmap {i + 1}/{len(pickle_files)}: {os.path.basename(pickle_file)} ---"
        )

        try:
            # Load the costmap
            costmap = Costmap.from_pickle(pickle_file)
            print(
                f"Loaded costmap: {costmap.width}x{costmap.height}, resolution: {costmap.resolution}"
            )

            # Set robot pose: first iteration uses center, subsequent use last predicted goal
            if robot_pose is None:
                # First iteration: use center of costmap as robot position
                center_world = costmap.grid_to_world(
                    Vector([costmap.width / 2, costmap.height / 2])
                )
                robot_pose = Vector([center_world.x, center_world.y])
            # else: robot_pose remains the last predicted goal

            print(f"Using robot position: {robot_pose}")

            # Get frontier prediction from Qwen
            print("Getting Qwen frontier prediction...")
            predicted_goal = predictor.get_exploration_goal(robot_pose, costmap)

            if predicted_goal:
                distance = np.sqrt(
                    (predicted_goal.x - robot_pose.x) ** 2 + (predicted_goal.y - robot_pose.y) ** 2
                )
                print(f"Predicted goal: {predicted_goal}, Distance: {distance:.2f}m")

                # Show the final visualization
                base_image = costmap_to_pil_image(costmap, predictor.image_scale_factor)
                final_image = predictor._draw_goals_on_image(
                    base_image, robot_pose, costmap, predicted_goal
                )

                # Display image
                title = f"Qwen Frontier Prediction {i + 1:04d}"
                final_image.show(title=title)

                # Update robot pose for next iteration
                robot_pose = predicted_goal

            else:
                print("No suitable frontier goal predicted by Qwen")

        except Exception as e:
            print(f"Error processing {pickle_file}: {e}")
            continue

    print(f"\n=== Qwen Frontier Detection Test Complete ===")
    print(f"Final explored goals count: {len(predictor.explored_goals)}")


if __name__ == "__main__":
    test_qwen_frontier_detection()
