#!/usr/bin/env python3

import math
import numpy as np
from typing import Dict, Tuple, Optional, Callable
import cv2
import logging
import time

from dimos.utils.logging_config import setup_logger
from dimos.utils.ros_utils import normalize_angle

from dimos.robot.local_planner.local_planner import BaseLocalPlanner, visualize_local_planner_state
from dimos.types.costmap import Costmap
from nav_msgs.msg import OccupancyGrid

logger = setup_logger("dimos.robot.unitree.vfh_local_planner", level=logging.DEBUG)

class VFHPurePursuitPlanner(BaseLocalPlanner):
    """
    A local planner that combines Vector Field Histogram (VFH) for obstacle avoidance
    with Pure Pursuit for goal tracking.
    """
    
    def __init__(self, 
                 get_costmap: Callable[[], Optional[OccupancyGrid]],
                 transform: object,
                 move_vel_control: Callable[[float, float, float], None],
                 safety_threshold: float = 0.8,
                 histogram_bins: int = 144,
                 max_linear_vel: float = 0.8,
                 max_angular_vel: float = 1.0,
                 lookahead_distance: float = 1.0,
                 goal_tolerance: float = 0.2,
                 angle_tolerance: float = 0.1,  # ~5.7 degrees
                 robot_width: float = 0.5,
                 robot_length: float = 0.7,
                 visualization_size: int = 400,
                 control_frequency: float = 10.0,
                 safe_goal_distance: float = 1.0):
        """
        Initialize the VFH + Pure Pursuit planner.
        
        Args:
            get_costmap: Function to get the latest local costmap
            transform: Object with transform methods (transform_point, transform_rot, etc.)
            move_vel_control: Function to send velocity commands
            safety_threshold: Distance to maintain from obstacles (meters)
            histogram_bins: Number of directional bins in the polar histogram
            max_linear_vel: Maximum linear velocity (m/s)
            max_angular_vel: Maximum angular velocity (rad/s)
            lookahead_distance: Lookahead distance for pure pursuit (meters)
            goal_tolerance: Distance at which the goal is considered reached (meters)
            angle_tolerance: Angle at which the goal orientation is considered reached (radians)
            robot_width: Width of the robot for visualization (meters)
            robot_length: Length of the robot for visualization (meters)
            visualization_size: Size of the visualization image in pixels
            control_frequency: Frequency at which the planner is called (Hz)
            safe_goal_distance: Distance at which to adjust the goal and ignore obstacles (meters)
        """
        # Initialize base class
        super().__init__(
            get_costmap=get_costmap,
            transform=transform,
            move_vel_control=move_vel_control,
            safety_threshold=safety_threshold,
            max_linear_vel=max_linear_vel,
            max_angular_vel=max_angular_vel,
            lookahead_distance=lookahead_distance,
            goal_tolerance=goal_tolerance,
            angle_tolerance=angle_tolerance,
            robot_width=robot_width,
            robot_length=robot_length,
            visualization_size=visualization_size,
            control_frequency=control_frequency,
            safe_goal_distance=safe_goal_distance
        )
        
        # VFH specific parameters
        self.histogram_bins = histogram_bins
        self.histogram = None
        self.selected_direction = None
        
        # VFH tuning parameters
        self.alpha = 0.2  # Histogram smoothing factor
        self.obstacle_weight = 10.0
        self.goal_weight = 1.0
        self.prev_direction_weight = 0.5
        self.prev_selected_angle = 0.0
        self.prev_linear_vel = 0.0
        self.linear_vel_filter_factor = 0.4
        self.low_speed_nudge = 0.1

        # Add after other initialization
        self.angle_mapping = np.linspace(-np.pi, np.pi, self.histogram_bins, endpoint=False)
        self.smoothing_kernel = np.array([self.alpha, (1-2*self.alpha), self.alpha])

    def _compute_velocity_commands(self) -> Dict[str, float]:
        """
        VFH + Pure Pursuit specific implementation of velocity command computation.
        
        Returns:
            Dict[str, float]: Velocity commands with 'x_vel' and 'angular_vel' keys
        """
        # Get necessary data for planning
        costmap = self.get_costmap()
        if costmap is None:
            logger.warning("No costmap available for planning")
            return {'x_vel': 0.0, 'angular_vel': 0.0}
            
        [pos, rot] = self.transform.transform_euler("base_link", "odom")
        robot_x, robot_y, robot_theta = pos[0], pos[1], rot[2]
        robot_pose = (robot_x, robot_y, robot_theta)
        
        # Calculate goal-related parameters
        goal_x, goal_y = self.goal_xy
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        goal_distance = np.linalg.norm([dx, dy])
        goal_direction = np.arctan2(dy, dx) - robot_theta
        goal_direction = normalize_angle(goal_direction)
        
        self.histogram = self.build_polar_histogram(costmap, robot_pose)
        
        # If we're ignoring obstacles near the goal, zero out the histogram
        if self.ignore_obstacles:
            self.histogram = np.zeros_like(self.histogram)
        
        self.selected_direction = self.select_direction(
            self.goal_weight,
            self.obstacle_weight,
            self.prev_direction_weight,
            self.histogram, 
            goal_direction,
        )

        # Calculate Pure Pursuit Velocities
        linear_vel, angular_vel = self.compute_pure_pursuit(goal_distance, self.selected_direction)

        # Slow down when turning sharply
        if abs(self.selected_direction) > 0.25:  # ~15 degrees
            # Scale from 1.0 (small turn) to 0.5 (sharp turn at 90 degrees or more)
            turn_factor = max(0.25, 1.0 - (abs(self.selected_direction) / (np.pi/2)))
            linear_vel *= turn_factor

        # Apply Collision Avoidance Stop - skip if ignoring obstacles
        if not self.ignore_obstacles and self.check_collision(self.selected_direction, safety_threshold=0.5):
            # Re-select direction prioritizing obstacle avoidance if colliding
            self.selected_direction = self.select_direction(
                self.goal_weight * 0.2,
                self.obstacle_weight,
                self.prev_direction_weight * 0.2,
                self.histogram,
                goal_direction
            )
            linear_vel, angular_vel = self.compute_pure_pursuit(goal_distance, self.selected_direction)

        if self.check_collision(0.0, safety_threshold=self.safety_threshold):
            logger.warning("Collision detected ahead. Stopping.")
            linear_vel = 0.0

        self.prev_linear_vel = linear_vel
        filtered_linear_vel = self.prev_linear_vel * self.linear_vel_filter_factor + linear_vel * (1 - self.linear_vel_filter_factor)

        return {'x_vel': filtered_linear_vel, 'angular_vel': angular_vel}
        
    def _smooth_histogram(self, histogram: np.ndarray) -> np.ndarray:
        """
        Apply advanced smoothing to the polar histogram to better identify valleys
        and reduce noise.
        
        Args:
            histogram: Raw histogram to smooth
            
        Returns:
            np.ndarray: Smoothed histogram
        """
        # Apply a windowed average with variable width based on obstacle density
        smoothed = np.zeros_like(histogram)
        bins = len(histogram)
        
        # First pass: basic smoothing with a 5-point kernel
        # This uses a wider window than the original 3-point smoother
        for i in range(bins):
            # Compute indices with wrap-around
            indices = [(i + j) % bins for j in range(-2, 3)]
            # Apply weighted average (more weight to the center)
            weights = [0.1, 0.2, 0.4, 0.2, 0.1]  # Sum = 1.0
            smoothed[i] = sum(histogram[idx] * weight for idx, weight in zip(indices, weights))
        
        # Second pass: peak and valley enhancement
        enhanced = np.zeros_like(smoothed)
        for i in range(bins):
            # Check neighboring values
            prev_idx = (i - 1) % bins
            next_idx = (i + 1) % bins
            
            # Enhance valleys (low values)
            if smoothed[i] < smoothed[prev_idx] and smoothed[i] < smoothed[next_idx]:
                # It's a local minimum - make it even lower
                enhanced[i] = smoothed[i] * 0.8
            # Enhance peaks (high values)
            elif smoothed[i] > smoothed[prev_idx] and smoothed[i] > smoothed[next_idx]:
                # It's a local maximum - make it even higher
                enhanced[i] = min(1.0, smoothed[i] * 1.2)
            else:
                enhanced[i] = smoothed[i]
                
        return enhanced

    def build_polar_histogram(self, costmap: Costmap, robot_pose: Tuple[float, float, float]):
        """
        Build a polar histogram of obstacle densities around the robot.
        
        Args:
            costmap: Costmap object with grid and metadata
            robot_pose: Tuple (x, y, theta) of the robot pose in the odom frame
            
        Returns:
            np.ndarray: Polar histogram of obstacle densities
        """
            
        # Get grid and find all obstacle cells
        occupancy_grid = costmap.grid
        y_indices, x_indices = np.where(occupancy_grid > 0)
        if len(y_indices) == 0:  # No obstacles
            return np.zeros(self.histogram_bins)
        
        # Get robot position in grid coordinates
        robot_x, robot_y, robot_theta = robot_pose
        robot_point = costmap.world_to_grid((robot_x, robot_y))
        robot_cell_x, robot_cell_y = robot_point.x, robot_point.y
        
        # Vectorized distance and angle calculation
        dx_cells = x_indices - robot_cell_x
        dy_cells = y_indices - robot_cell_y
        distances = np.sqrt(dx_cells**2 + dy_cells**2) * costmap.resolution
        angles_grid = np.arctan2(dy_cells, dx_cells)
        angles_robot = normalize_angle(angles_grid - robot_theta)
        
        # Convert to bin indices
        bin_indices = ((angles_robot + np.pi) / (2 * np.pi) * self.histogram_bins).astype(int) % self.histogram_bins
        
        # Get obstacle values
        obstacle_values = occupancy_grid[y_indices, x_indices] / 100.0
        
        # Build histogram
        histogram = np.zeros(self.histogram_bins)
        mask = distances > 0
        # Weight obstacles by inverse square of distance and cell value
        np.add.at(histogram, bin_indices[mask], obstacle_values[mask] / (distances[mask] ** 2))
        
        # Apply the enhanced smoothing
        return self._smooth_histogram(histogram)
    
    def select_direction(self, goal_weight, obstacle_weight, prev_direction_weight, histogram, goal_direction):
        """
        Select best direction based on a simple weighted cost function.
        
        Args:
            goal_weight: Weight for the goal direction component
            obstacle_weight: Weight for the obstacle avoidance component
            prev_direction_weight: Weight for previous direction consistency
            histogram: Polar histogram of obstacle density
            goal_direction: Desired direction to goal
            
        Returns:
            float: Selected direction in radians
        """
        # Normalize histogram if needed
        if np.max(histogram) > 0:
            histogram = histogram / np.max(histogram)
            
        # Calculate costs for each possible direction
        angle_diffs = np.abs(normalize_angle(self.angle_mapping - goal_direction))
        prev_diffs = np.abs(normalize_angle(self.angle_mapping - self.prev_selected_angle))
        
        # Combine costs with weights
        obstacle_costs = obstacle_weight * histogram
        goal_costs = goal_weight * angle_diffs
        prev_costs = prev_direction_weight * prev_diffs
        
        total_costs = obstacle_costs + goal_costs + prev_costs
        
        # Select direction with lowest cost
        min_cost_idx = np.argmin(total_costs)
        selected_angle = self.angle_mapping[min_cost_idx]
        
        # Update history for next iteration
        self.prev_selected_angle = selected_angle
        
        return selected_angle
    
    def compute_pure_pursuit(self, goal_distance: float, goal_direction: float) -> Tuple[float, float]:
        """ Compute pure pursuit velocities."""
        if goal_distance < self.goal_tolerance:
            return 0.0, 0.0
        
        lookahead = min(self.lookahead_distance, goal_distance)
        linear_vel = min(self.max_linear_vel, goal_distance)
        angular_vel = 2.0 * np.sin(goal_direction) / lookahead
        angular_vel = max(-self.max_angular_vel, min(angular_vel, self.max_angular_vel))
        
        return linear_vel, angular_vel
    
    def check_collision(self, selected_direction: float, safety_threshold: float = 1.0) -> bool:
        """Check if there's an obstacle in the selected direction within safety threshold."""
        # Skip collision check if ignoring obstacles
        if self.ignore_obstacles:
            return False
            
        # Get the latest costmap and robot pose
        costmap = self.get_costmap()
        if costmap is None:
            return False  # No costmap available
            
        [pos, rot] = self.transform.transform_euler("base_link", "odom")
        robot_x, robot_y, robot_theta = pos[0], pos[1], rot[2]
        
        # Direction in world frame
        direction_world = robot_theta + selected_direction
        
        # Safety distance in cells
        safety_cells = int(safety_threshold / costmap.resolution)
        
        # Get robot position in grid coordinates
        robot_point = costmap.world_to_grid((robot_x, robot_y))
        robot_cell_x, robot_cell_y = robot_point.x, robot_point.y
        
        # Check for obstacles along the selected direction
        for dist in range(1, safety_cells + 1):
            # Calculate cell position
            cell_x = robot_cell_x + int(dist * np.cos(direction_world))
            cell_y = robot_cell_y + int(dist * np.sin(direction_world))
            
            # Check if cell is within grid bounds
            if not (0 <= cell_x < costmap.width and 0 <= cell_y < costmap.height):
                continue
            
            # Check if cell contains an obstacle (threshold at 50)
            if costmap.grid[int(cell_y), int(cell_x)] > 50:
                return True
                
        return False  # No collision detected

    def update_visualization(self) -> np.ndarray:
        """Generate visualization of the planning state."""
        try:
            costmap = self.get_costmap()
            if costmap is None:
                raise ValueError("Costmap is None")
                
            [pos, rot] = self.transform.transform_euler("base_link", "odom")
            robot_x, robot_y, robot_theta = pos[0], pos[1], rot[2]
            robot_pose = (robot_x, robot_y, robot_theta)
            
            goal_xy = self.goal_xy # This could be a lookahead point or final goal
            
            # Get the latest histogram and selected direction, if available
            histogram = getattr(self, 'histogram', None)
            selected_direction = getattr(self, 'selected_direction', None)
            
            # Get waypoint data if in waypoint mode
            waypoints_to_draw = self.waypoints_in_odom
            current_wp_index_to_draw = self.current_waypoint_index if self.waypoints_in_odom is not None else None
            # Ensure index is valid before passing
            if waypoints_to_draw is not None and current_wp_index_to_draw is not None: 
                if not (0 <= current_wp_index_to_draw < len(waypoints_to_draw)):
                    current_wp_index_to_draw = None # Invalidate index if out of bounds

            return visualize_local_planner_state(
                occupancy_grid=costmap.grid,
                grid_resolution=costmap.resolution,
                grid_origin=(costmap.origin.x, costmap.origin.y, costmap.origin_theta),
                robot_pose=robot_pose,
                goal_xy=goal_xy, # Current target (lookahead or final)
                goal_theta=self.goal_theta, # Pass goal orientation if available
                visualization_size=self.visualization_size,
                robot_width=self.robot_width,
                robot_length=self.robot_length,
                histogram=histogram, 
                selected_direction=selected_direction,
                waypoints=waypoints_to_draw, # Pass the full path
                current_waypoint_index=current_wp_index_to_draw # Pass the target index
            )
        except Exception as e:
            logger.error(f"Error during visualization update: {e}")
            # Return a blank image with error text
            blank = np.ones((self.visualization_size, self.visualization_size, 3), dtype=np.uint8) * 255
            cv2.putText(blank, "Viz Error",
                        (self.visualization_size // 4, self.visualization_size // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
            return blank
