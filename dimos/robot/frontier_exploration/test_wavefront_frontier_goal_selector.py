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

from typing import List, Optional

import numpy as np
import pytest
from PIL import Image, ImageDraw
from reactivex import operators as ops

from dimos.robot.frontier_exploration.utils import costmap_to_pil_image
from dimos.robot.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.msgs.geometry_msgs import Vector3 as Vector
from dimos.utils.testing import SensorReplay


def get_office_lidar_costmap(take_frames: int = 1, voxel_size: float = 0.5) -> tuple:
    """
    Get a costmap from office_lidar data using SensorReplay.

    Args:
        take_frames: Number of lidar frames to take (default 1)
        voxel_size: Voxel size for map construction

    Returns:
        Tuple of (costmap, first_lidar_message) for testing
    """
    # Load office lidar data using SensorReplay as documented
    lidar_stream = SensorReplay("office_lidar", autocast=LidarMessage.from_msg)

    # Create map with specified voxel size
    map_obj = Map(voxel_size=voxel_size)

    # Take only the specified number of frames and build map
    limited_stream = lidar_stream.stream().pipe(ops.take(take_frames))

    # Store the first lidar message for reference
    first_lidar = None

    def capture_first_and_add(lidar_msg):
        nonlocal first_lidar
        if first_lidar is None:
            first_lidar = lidar_msg
        return map_obj.add_frame(lidar_msg)

    # Process the stream
    limited_stream.pipe(ops.map(capture_first_and_add)).run()

    # Get the resulting costmap
    costmap = map_obj.costmap()

    return costmap, first_lidar


def test_frontier_detection_with_office_lidar():
    """Test frontier detection using a single frame from office_lidar data."""
    # Get costmap from office lidar data
    costmap, first_lidar = get_office_lidar_costmap(take_frames=1, voxel_size=0.3)

    # Verify we have a valid costmap
    assert costmap is not None, "Costmap should not be None"
    assert costmap.width > 0 and costmap.height > 0, "Costmap should have valid dimensions"

    print(f"Costmap dimensions: {costmap.width}x{costmap.height}")
    print(f"Costmap resolution: {costmap.resolution}")
    print(f"Unknown percent: {costmap.unknown_percent:.1f}%")
    print(f"Free percent: {costmap.free_percent:.1f}%")
    print(f"Occupied percent: {costmap.occupied_percent:.1f}%")

    # Initialize frontier explorer with default parameters
    explorer = WavefrontFrontierExplorer()

    # Set robot pose near the center of free space in the costmap
    # We'll use the lidar origin as a reasonable robot position
    robot_pose = first_lidar.origin
    print(f"Robot pose: {robot_pose}")

    # Detect frontiers
    frontiers = explorer.detect_frontiers(robot_pose, costmap)

    # Verify frontier detection results
    assert isinstance(frontiers, list), "Frontiers should be returned as a list"
    print(f"Detected {len(frontiers)} frontiers")

    # Test that we get some frontiers (office environment should have unexplored areas)
    if len(frontiers) > 0:
        print("Frontier detection successful - found unexplored areas")

        # Verify frontiers are Vector objects with valid coordinates
        for i, frontier in enumerate(frontiers[:5]):  # Check first 5
            assert isinstance(frontier, Vector), f"Frontier {i} should be a Vector"
            assert hasattr(frontier, "x") and hasattr(frontier, "y"), (
                f"Frontier {i} should have x,y coordinates"
            )
            print(f"  Frontier {i}: ({frontier.x:.2f}, {frontier.y:.2f})")
    else:
        print("No frontiers detected - map may be fully explored or parameters too restrictive")


def test_exploration_goal_selection():
    """Test the complete exploration goal selection pipeline."""
    # Get costmap from office lidar data
    costmap, first_lidar = get_office_lidar_costmap(take_frames=1, voxel_size=0.3)

    # Initialize frontier explorer with default parameters
    explorer = WavefrontFrontierExplorer()

    # Use lidar origin as robot position
    robot_pose = first_lidar.origin

    # Get exploration goal
    goal = explorer.get_exploration_goal(robot_pose, costmap)

    if goal is not None:
        assert isinstance(goal, Vector), "Goal should be a Vector"
        print(f"Selected exploration goal: ({goal.x:.2f}, {goal.y:.2f})")

        # Test that goal gets marked as explored
        assert len(explorer.explored_goals) == 1, "Goal should be marked as explored"
        assert explorer.explored_goals[0] == goal, "Explored goal should match selected goal"

    else:
        print("No exploration goal selected - map may be fully explored")


def test_exploration_session_reset():
    """Test exploration session reset functionality."""
    # Get costmap
    costmap, first_lidar = get_office_lidar_costmap(take_frames=1, voxel_size=0.3)

    # Initialize explorer and select a goal
    explorer = WavefrontFrontierExplorer()
    robot_pose = first_lidar.origin

    # Select a goal to populate exploration state
    goal = explorer.get_exploration_goal(robot_pose, costmap)

    # Verify state is populated
    initial_explored_count = len(explorer.explored_goals)
    initial_direction = explorer.exploration_direction

    # Reset exploration session
    explorer.reset_exploration_session()

    # Verify state is cleared
    assert len(explorer.explored_goals) == 0, "Explored goals should be cleared after reset"
    assert explorer.exploration_direction.x == 0.0 and explorer.exploration_direction.y == 0.0, (
        "Exploration direction should be reset"
    )
    assert explorer.last_costmap is None, "Last costmap should be cleared"
    assert explorer.num_no_gain_attempts == 0, "No-gain attempts should be reset"

    print("Exploration session reset successfully")


@pytest.mark.vis
def test_frontier_detection_visualization():
    """Test frontier detection with visualization (marked with @pytest.mark.vis)."""
    # Get costmap from office lidar data
    costmap, first_lidar = get_office_lidar_costmap(take_frames=1, voxel_size=0.2)

    # Initialize frontier explorer with default parameters
    explorer = WavefrontFrontierExplorer()

    # Use lidar origin as robot position
    robot_pose = first_lidar.origin

    # Detect all frontiers for visualization
    all_frontiers = explorer.detect_frontiers(robot_pose, costmap)

    # Get selected goal
    selected_goal = explorer.get_exploration_goal(robot_pose, costmap)

    print(f"Visualizing {len(all_frontiers)} frontier candidates")
    if selected_goal:
        print(f"Selected goal: ({selected_goal.x:.2f}, {selected_goal.y:.2f})")

    # Create visualization
    image_scale_factor = 4
    base_image = costmap_to_pil_image(costmap, image_scale_factor)

    # Helper function to convert world coordinates to image coordinates
    def world_to_image_coords(world_pos: Vector) -> tuple[int, int]:
        grid_pos = costmap.world_to_grid(world_pos)
        img_x = int(grid_pos.x * image_scale_factor)
        img_y = int((costmap.height - grid_pos.y) * image_scale_factor)  # Flip Y
        return img_x, img_y

    # Draw visualization
    draw = ImageDraw.Draw(base_image)

    # Draw frontier candidates as gray dots
    for frontier in all_frontiers[:20]:  # Limit to top 20
        x, y = world_to_image_coords(frontier)
        radius = 6
        draw.ellipse(
            [x - radius, y - radius, x + radius, y + radius],
            fill=(128, 128, 128),  # Gray
            outline=(64, 64, 64),
            width=1,
        )

    # Draw robot position as blue dot
    robot_x, robot_y = world_to_image_coords(robot_pose)
    robot_radius = 10
    draw.ellipse(
        [
            robot_x - robot_radius,
            robot_y - robot_radius,
            robot_x + robot_radius,
            robot_y + robot_radius,
        ],
        fill=(0, 0, 255),  # Blue
        outline=(0, 0, 128),
        width=3,
    )

    # Draw selected goal as red dot
    if selected_goal:
        goal_x, goal_y = world_to_image_coords(selected_goal)
        goal_radius = 12
        draw.ellipse(
            [
                goal_x - goal_radius,
                goal_y - goal_radius,
                goal_x + goal_radius,
                goal_y + goal_radius,
            ],
            fill=(255, 0, 0),  # Red
            outline=(128, 0, 0),
            width=3,
        )

    # Display the image
    base_image.show(title="Frontier Detection - Office Lidar")

    print("Visualization displayed. Close the image window to continue.")


def test_multi_frame_exploration():
    """Tool test for multi-frame exploration analysis."""
    print("=== Multi-Frame Exploration Analysis ===")

    # Test with different numbers of frames
    frame_counts = [1, 3, 5]

    for frame_count in frame_counts:
        print(f"\n--- Testing with {frame_count} lidar frame(s) ---")

        # Get costmap with multiple frames
        costmap, first_lidar = get_office_lidar_costmap(take_frames=frame_count, voxel_size=0.3)

        print(
            f"Costmap: {costmap.width}x{costmap.height}, "
            f"unknown: {costmap.unknown_percent:.1f}%, "
            f"free: {costmap.free_percent:.1f}%, "
            f"occupied: {costmap.occupied_percent:.1f}%"
        )

        # Initialize explorer with default parameters
        explorer = WavefrontFrontierExplorer()

        # Detect frontiers
        robot_pose = first_lidar.origin
        frontiers = explorer.detect_frontiers(robot_pose, costmap)

        print(f"Detected {len(frontiers)} frontiers")

        # Get exploration goal
        goal = explorer.get_exploration_goal(robot_pose, costmap)
        if goal:
            distance = np.sqrt((goal.x - robot_pose.x) ** 2 + (goal.y - robot_pose.y) ** 2)
            print(f"Selected goal at distance {distance:.2f}m")
        else:
            print("No exploration goal selected")
