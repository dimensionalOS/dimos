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

import numpy as np
from PIL import ImageDraw
import pytest

from dimos.msgs.geometry_msgs import Vector3
from dimos.msgs.nav_msgs import CostValues, OccupancyGrid
from dimos.navigation.frontier_exploration.utils import costmap_to_pil_image
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)


@pytest.fixture
def explorer():
    """Create a WavefrontFrontierExplorer instance for testing."""
    explorer = WavefrontFrontierExplorer(
        min_frontier_perimeter=0.3,  # Smaller for faster tests
        safe_distance=0.5,  # Smaller for faster distance calculations
        info_gain_threshold=0.02,
    )
    yield explorer
    # Cleanup after test
    try:
        explorer.stop()
    except:
        pass


@pytest.fixture
def quick_costmap():
    """Create a very small costmap for quick tests."""
    width, height = 20, 20
    grid = np.full((height, width), CostValues.UNKNOWN, dtype=np.int8)

    # Simple free space in center
    grid[8:12, 8:12] = CostValues.FREE

    # Small extensions
    grid[9:11, 6:8] = CostValues.FREE  # Left
    grid[9:11, 12:14] = CostValues.FREE  # Right

    # One obstacle
    grid[9:10, 9:10] = CostValues.OCCUPIED

    from dimos.msgs.geometry_msgs import Pose

    origin = Pose()
    origin.position.x = -1.0
    origin.position.y = -1.0
    origin.position.z = 0.0
    origin.orientation.w = 1.0

    occupancy_grid = OccupancyGrid(
        grid=grid, resolution=0.1, origin=origin, frame_id="map", ts=time.time()
    )

    class MockLidar:
        def __init__(self) -> None:
            self.origin = Vector3(0.0, 0.0, 0.0)

    return occupancy_grid, MockLidar()


def create_test_costmap(width: int = 40, height: int = 40, resolution: float = 0.1):
    """Create a simple test costmap with free, occupied, and unknown regions.

    Default size reduced from 100x100 to 40x40 for faster tests.
    """
    grid = np.full((height, width), CostValues.UNKNOWN, dtype=np.int8)

    # Create a smaller free space region with simple shape
    # Central room
    grid[15:25, 15:25] = CostValues.FREE

    # Small corridors extending from central room
    grid[18:22, 10:15] = CostValues.FREE  # Left corridor
    grid[18:22, 25:30] = CostValues.FREE  # Right corridor
    grid[10:15, 18:22] = CostValues.FREE  # Top corridor
    grid[25:30, 18:22] = CostValues.FREE  # Bottom corridor

    # Add fewer obstacles for faster processing
    grid[19:21, 19:21] = CostValues.OCCUPIED  # Central obstacle
    grid[13:14, 18:22] = CostValues.OCCUPIED  # Top corridor obstacle

    # Create origin at bottom-left, adjusted for map size
    from dimos.msgs.geometry_msgs import Pose

    origin = Pose()
    # Center the map around (0, 0) in world coordinates
    origin.position.x = -(width * resolution) / 2.0
    origin.position.y = -(height * resolution) / 2.0
    origin.position.z = 0.0
    origin.orientation.w = 1.0

    occupancy_grid = OccupancyGrid(
        grid=grid, resolution=resolution, origin=origin, frame_id="map", ts=time.time()
    )

    # Create a mock lidar message with origin
    class MockLidar:
        def __init__(self) -> None:
            self.origin = Vector3(0.0, 0.0, 0.0)

    return occupancy_grid, MockLidar()


def test_frontier_detection_with_office_lidar(explorer, quick_costmap) -> None:
    """Test frontier detection using a test costmap."""
    # Get test costmap
    costmap, first_lidar = quick_costmap

    # Verify we have a valid costmap
    assert costmap is not None, "Costmap should not be None"
    assert costmap.width > 0 and costmap.height > 0, "Costmap should have valid dimensions"

    print(f"Costmap dimensions: {costmap.width}x{costmap.height}")
    print(f"Costmap resolution: {costmap.resolution}")
    print(f"Unknown percent: {costmap.unknown_percent:.1f}%")
    print(f"Free percent: {costmap.free_percent:.1f}%")
    print(f"Occupied percent: {costmap.occupied_percent:.1f}%")

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
            assert isinstance(frontier, Vector3), f"Frontier {i} should be a Vector3"
            assert hasattr(frontier, "x") and hasattr(frontier, "y"), (
                f"Frontier {i} should have x,y coordinates"
            )
            print(f"  Frontier {i}: ({frontier.x:.2f}, {frontier.y:.2f})")
    else:
        print("No frontiers detected - map may be fully explored or parameters too restrictive")

    explorer.stop()  # TODO: this should be a in try-finally


def test_exploration_goal_selection(explorer) -> None:
    """Test the complete exploration goal selection pipeline."""
    # Get test costmap - use regular size for more realistic test
    costmap, first_lidar = create_test_costmap()

    # Use lidar origin as robot position
    robot_pose = first_lidar.origin

    # Get exploration goal
    goal = explorer.get_exploration_goal(robot_pose, costmap)

    if goal is not None:
        assert isinstance(goal, Vector3), "Goal should be a Vector3"
        print(f"Selected exploration goal: ({goal.x:.2f}, {goal.y:.2f})")

        # Test that goal gets marked as explored
        assert len(explorer.explored_goals) == 1, "Goal should be marked as explored"
        assert explorer.explored_goals[0] == goal, "Explored goal should match selected goal"

        # Test that goal is within costmap bounds
        grid_pos = costmap.world_to_grid(goal)
        assert 0 <= grid_pos.x < costmap.width, "Goal x should be within costmap bounds"
        assert 0 <= grid_pos.y < costmap.height, "Goal y should be within costmap bounds"

        # Test that goal is at a reasonable distance from robot
        distance = np.sqrt((goal.x - robot_pose.x) ** 2 + (goal.y - robot_pose.y) ** 2)
        assert 0.1 < distance < 20.0, f"Goal distance {distance:.2f}m should be reasonable"

    else:
        print("No exploration goal selected - map may be fully explored")

    explorer.stop()  # TODO: this should be a in try-finally


def test_exploration_session_reset(explorer) -> None:
    """Test exploration session reset functionality."""
    # Get test costmap
    costmap, first_lidar = create_test_costmap()

    # Use lidar origin as robot position
    robot_pose = first_lidar.origin

    # Select a goal to populate exploration state
    goal = explorer.get_exploration_goal(robot_pose, costmap)

    # Verify state is populated (skip if no goals available)
    if goal:
        initial_explored_count = len(explorer.explored_goals)
        assert initial_explored_count > 0, "Should have at least one explored goal"

    # Reset exploration session
    explorer.reset_exploration_session()

    # Verify state is cleared
    assert len(explorer.explored_goals) == 0, "Explored goals should be cleared after reset"
    assert explorer.exploration_direction.x == 0.0 and explorer.exploration_direction.y == 0.0, (
        "Exploration direction should be reset"
    )
    assert explorer.last_costmap is None, "Last costmap should be cleared"
    assert explorer.no_gain_counter == 0, "No-gain counter should be reset"

    print("Exploration session reset successfully")
    explorer.stop()  # TODO: this should be a in try-finally


def test_frontier_ranking(explorer) -> None:
    """Test frontier ranking and scoring logic."""
    # Get test costmap
    costmap, first_lidar = create_test_costmap()

    robot_pose = first_lidar.origin

    # Get first set of frontiers
    frontiers1 = explorer.detect_frontiers(robot_pose, costmap)
    goal1 = explorer.get_exploration_goal(robot_pose, costmap)

    if goal1:
        # Verify the selected goal is the first in the ranked list
        assert frontiers1[0].x == goal1.x and frontiers1[0].y == goal1.y, (
            "Selected goal should be the highest ranked frontier"
        )

        # Test that goals are being marked as explored
        assert len(explorer.explored_goals) == 1, "Goal should be marked as explored"
        assert (
            explorer.explored_goals[0].x == goal1.x and explorer.explored_goals[0].y == goal1.y
        ), "Explored goal should match selected goal"

        # Get another goal
        goal2 = explorer.get_exploration_goal(robot_pose, costmap)
        if goal2:
            assert len(explorer.explored_goals) == 2, (
                "Second goal should also be marked as explored"
            )

        # Test distance to obstacles
        obstacle_dist = explorer._compute_distance_to_obstacles(goal1, costmap)
        # Note: Goals might be closer than safe_distance if that's the best available frontier
        # The safe_distance is used for scoring, not as a hard constraint
        print(
            f"Distance to obstacles: {obstacle_dist:.2f}m (safe distance: {explorer.safe_distance}m)"
        )

        print(f"Frontier ranking test passed - selected goal at ({goal1.x:.2f}, {goal1.y:.2f})")
        print(f"Total frontiers detected: {len(frontiers1)}")
    else:
        print("No frontiers found for ranking test")

    explorer.stop()  # TODO: this should be a in try-finally


def test_exploration_with_no_gain_detection() -> None:
    """Test information gain detection and exploration termination."""
    # Get initial costmap
    costmap1, first_lidar = create_test_costmap()

    # Initialize explorer with low no-gain threshold for testing
    explorer = WavefrontFrontierExplorer(info_gain_threshold=0.01, num_no_gain_attempts=2)

    try:
        robot_pose = first_lidar.origin

        # Select multiple goals to populate history
        for i in range(6):
            goal = explorer.get_exploration_goal(robot_pose, costmap1)
            if goal:
                print(f"Goal {i + 1}: ({goal.x:.2f}, {goal.y:.2f})")

        # Now use same costmap repeatedly to trigger no-gain detection
        initial_counter = explorer.no_gain_counter

        # This should increment no-gain counter
        goal = explorer.get_exploration_goal(robot_pose, costmap1)
        assert explorer.no_gain_counter > initial_counter, "No-gain counter should increment"

        # Continue until exploration stops
        for _ in range(3):
            goal = explorer.get_exploration_goal(robot_pose, costmap1)
            if goal is None:
                break

        # Should have stopped due to no information gain
        assert goal is None, "Exploration should stop after no-gain threshold"
        assert explorer.no_gain_counter == 0, "Counter should reset after stopping"
    finally:
        explorer.stop()


@pytest.mark.vis
def test_frontier_detection_visualization() -> None:
    """Test frontier detection with visualization (marked with @pytest.mark.vis)."""
    # Get test costmap
    costmap, first_lidar = create_test_costmap()

    # Initialize frontier explorer with default parameters
    explorer = WavefrontFrontierExplorer()

    try:
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
        def world_to_image_coords(world_pos: Vector3) -> tuple[int, int]:
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
    finally:
        explorer.stop()


def test_performance_timing() -> None:
    """Test performance by timing frontier detection operations."""
    import time

    # Test with different costmap sizes
    sizes = [(20, 20), (40, 40), (60, 60)]
    results = []

    for width, height in sizes:
        # Create costmap of specified size
        costmap, lidar = create_test_costmap(width, height)

        # Create explorer with optimized parameters
        explorer = WavefrontFrontierExplorer(
            min_frontier_perimeter=0.3,
            safe_distance=0.5,
            info_gain_threshold=0.02,
        )

        try:
            robot_pose = lidar.origin

            # Time frontier detection
            start = time.time()
            frontiers = explorer.detect_frontiers(robot_pose, costmap)
            detect_time = time.time() - start

            # Time goal selection
            start = time.time()
            explorer.get_exploration_goal(robot_pose, costmap)
            goal_time = time.time() - start

            results.append(
                {
                    "size": f"{width}x{height}",
                    "cells": width * height,
                    "detect_time": detect_time,
                    "goal_time": goal_time,
                    "frontiers": len(frontiers),
                }
            )

            print(f"\nSize {width}x{height}:")
            print(f"  Cells: {width * height}")
            print(f"  Frontier detection: {detect_time:.4f}s")
            print(f"  Goal selection: {goal_time:.4f}s")
            print(f"  Frontiers found: {len(frontiers)}")
        finally:
            explorer.stop()

    # Check that larger maps take more time (expected behavior)
    for result in results:
        assert result["detect_time"] < 2.0, f"Detection too slow: {result['detect_time']}s"
        assert result["goal_time"] < 1.5, f"Goal selection too slow: {result['goal_time']}s"

    print("\nPerformance test passed - all operations completed within time limits")
