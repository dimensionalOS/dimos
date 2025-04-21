#!/usr/bin/env python3

import math
import numpy as np
from typing import Dict, Tuple
from dimos.robot.robot import Robot
import cv2
from reactivex import Observable
from reactivex.subject import Subject
import threading
import time
import logging
from dimos.utils.logging_config import setup_logger
from dimos.utils.ros_utils import (
    ros_msg_to_pose_tuple, 
    ros_msg_to_numpy_grid, 
    normalize_angle,
    visualize_local_planner_state
)

logger = setup_logger("dimos.robot.unitree.local_planner", level=logging.DEBUG)

class VFHPurePursuitPlanner:
    """
    A local planner that combines Vector Field Histogram (VFH) for obstacle avoidance
    with Pure Pursuit for goal tracking.
    """
    
    def __init__(self, 
                 robot: Robot,
                 safety_threshold: float = 0.5,
                 histogram_bins: int = 72,
                 max_linear_vel: float = 0.8,
                 max_angular_vel: float = 1.0,
                 lookahead_distance: float = 1.0,
                 goal_tolerance: float = 0.3,
                 robot_width: float = 0.5,
                 robot_length: float = 0.7,
                 visualization_size: int = 400):
        """
        Initialize the VFH + Pure Pursuit planner.
        
        Args:
            robot: Robot instance to get data from and send commands to
            safety_threshold: Distance to maintain from obstacles (meters)
            histogram_bins: Number of directional bins in the polar histogram
            max_linear_vel: Maximum linear velocity (m/s)
            max_angular_vel: Maximum angular velocity (rad/s)
            lookahead_distance: Lookahead distance for pure pursuit (meters)
            goal_tolerance: Distance at which the goal is considered reached (meters)
            robot_width: Width of the robot for visualization (meters)
            robot_length: Length of the robot for visualization (meters)
            visualization_size: Size of the visualization image in pixels
        """
        self.robot = robot
        self.safety_threshold = safety_threshold
        self.histogram_bins = histogram_bins
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.lookahead_distance = lookahead_distance
        self.goal_tolerance = goal_tolerance
        self.robot_width = robot_width
        self.robot_length = robot_length
        self.visualization_size = visualization_size

        # VFH variables
        self.histogram = None
        self.selected_direction = None
        
        # VFH parameters
        self.alpha = 0.2  # Histogram smoothing factor
        self.obstacle_weight = 2.0
        self.goal_weight = 1.0
        self.prev_direction_weight = 0.5
        self.prev_selected_angle = 0.0
        self.goal_distance_scale_factor = 3.0
        self.goal_xy = None  # Default goal position (odom frame)
    
    def set_goal(self, goal_xy: Tuple[float, float], is_robot_frame: bool = True):
        """Set the goal position, converting to odom frame if necessary."""
        if is_robot_frame:
            odom = self.robot.ros_control.get_odometry()
            robot_pose = ros_msg_to_pose_tuple(odom)  # Use imported function
            robot_x, robot_y, robot_theta = robot_pose
            goal_x_robot, goal_y_robot = goal_xy
            
            # Transform to odom frame using numpy functions
            goal_x_odom = robot_x + goal_x_robot * np.cos(robot_theta) - goal_y_robot * np.sin(robot_theta)
            goal_y_odom = robot_y + goal_x_robot * np.sin(robot_theta) + goal_y_robot * np.cos(robot_theta)
            
            self.goal_xy = (goal_x_odom, goal_y_odom)
            logger.info(f"Goal set in robot frame ({goal_x_robot:.2f}, {goal_y_robot:.2f}), "
                        f"transformed to odom frame: ({self.goal_xy[0]:.2f}, {self.goal_xy[1]:.2f})")
        else:
            self.goal_xy = goal_xy
            logger.info(f"Goal set directly in odom frame: ({self.goal_xy[0]:.2f}, {self.goal_xy[1]:.2f})")
    
        if self.check_goal_collision():
            logger.warning("Goal is in collision. Adjusted goal to safe position.")
            self.goal_xy = self.adjust_goal_to_valid_position(self.goal_xy, 0.5)

    def plan(self) -> Dict[str, float]:
        """
        Compute velocity commands using VFH + Pure Pursuit.
        """
        costmap = self.robot.ros_control.get_costmap()
        occupancy_grid, grid_info, _ = ros_msg_to_numpy_grid(costmap)  # Use imported function
        
        odom = self.robot.ros_control.get_odometry()
        robot_pose = ros_msg_to_pose_tuple(odom)  # Use imported function
        
        robot_x, robot_y, robot_theta = robot_pose
        if self.goal_xy is None:
            return {'x_vel': 0.0, 'angular_vel': 0.0}
        goal_x, goal_y = self.goal_xy
        
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        goal_distance = np.linalg.norm([dx, dy])
        goal_direction = np.arctan2(dy, dx) - robot_theta
        goal_direction = normalize_angle(goal_direction)  # Use imported function
        
        if goal_distance < self.goal_tolerance:
            return {'x_vel': 0.0, 'angular_vel': 0.0}

        goal_distance_scale = 1.0
        if goal_distance < self.goal_tolerance * self.goal_distance_scale_factor:
            goal_distance_scale = 1.0 / (goal_distance / (self.goal_tolerance * self.goal_distance_scale_factor))

        self.histogram = self.build_polar_histogram(occupancy_grid, grid_info, robot_pose)
        self.selected_direction = self.select_direction(
            self.goal_weight * goal_distance_scale,
            self.obstacle_weight,
            self.prev_direction_weight,
            self.histogram, 
            goal_direction,
        )

        linear_vel, angular_vel = self.compute_pure_pursuit(goal_distance, self.selected_direction)

        if self.check_collision(self.selected_direction):
            self.selected_direction = self.select_direction(
                0.0,
                self.obstacle_weight,
                0.0,
                self.histogram,
                goal_direction
            )
        
            _, angular_vel = self.compute_pure_pursuit(goal_distance, self.selected_direction)
            linear_vel = 0.0

        return {'x_vel': linear_vel, 'angular_vel': angular_vel}
    
    def update_visualization(self) -> np.ndarray:
        """
        Generate visualization by calling the utility function.
        """
        try:
            costmap = self.robot.ros_control.get_costmap()
            odom = self.robot.ros_control.get_odometry()
            
            occupancy_grid, grid_info, grid_origin = ros_msg_to_numpy_grid(costmap)
            _, _, grid_resolution = grid_info
            robot_pose = ros_msg_to_pose_tuple(odom)
            goal_xy = self.goal_xy
            
            # Get the latest histogram and selected direction, if available
            histogram = getattr(self, 'histogram', None)
            selected_direction = getattr(self, 'selected_direction', None)

            return visualize_local_planner_state(
                occupancy_grid=occupancy_grid,
                grid_resolution=grid_resolution,
                grid_origin=grid_origin,
                robot_pose=robot_pose,
                goal_xy=goal_xy,
                visualization_size=self.visualization_size,
                robot_width=self.robot_width,
                robot_length=self.robot_length,
                histogram=histogram, # Pass histogram
                selected_direction=selected_direction # Pass selected direction
            )
        except Exception as e:
            logger.error(f"Error during visualization update: {e}")
            # Return a blank image with error text
            blank = np.ones((self.visualization_size, self.visualization_size, 3), dtype=np.uint8) * 255
            cv2.putText(blank, "Viz Error",
                        (self.visualization_size // 4, self.visualization_size // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
            return blank
    
    def create_stream(self, frequency_hz: float = 10.0) -> Observable:
        """
        Create an Observable stream that emits the visualization image at a fixed frequency.
        """
        subject = Subject()
        sleep_time = 1.0 / frequency_hz
        
        def frame_emitter():
            frame_count = 0
            while True:
                try:
                    # Generate the frame using the updated method
                    frame = self.update_visualization() 
                    subject.on_next(frame)
                    frame_count += 1
                except Exception as e:
                    logger.error(f"Error in frame emitter thread: {e}")
                    # Optionally, emit an error frame or simply skip
                    # subject.on_error(e) # This would terminate the stream
                time.sleep(sleep_time)
        
        emitter_thread = threading.Thread(target=frame_emitter, daemon=True)
        emitter_thread.start()
        logger.info("Started visualization frame emitter thread")
        return subject
    
    def build_polar_histogram(self,
                              occupancy_grid: np.ndarray, 
                              grid_info: Tuple[int, int, float],
                              robot_pose: Tuple[float, float, float]) -> np.ndarray:
        """ Build polar histogram (remains unchanged)."""
        # Initialize histogram
        histogram = np.zeros(self.histogram_bins)
        grid_width, grid_height, grid_resolution = grid_info
        
        # Extract robot position in grid coordinates
        robot_x, robot_y, robot_theta = robot_pose
        
        # Need grid origin to calculate robot position relative to grid
        costmap = self.robot.ros_control.get_costmap()
        _, _, grid_origin = ros_msg_to_numpy_grid(costmap) 
        grid_origin_x, grid_origin_y, _ = grid_origin

        robot_rel_x = robot_x - grid_origin_x
        robot_rel_y = robot_y - grid_origin_y
        robot_cell_x = int(robot_rel_x / grid_resolution)
        robot_cell_y = int(robot_rel_y / grid_resolution)
        
        # Get grid dimensions
        height, width = occupancy_grid.shape
        
        # Maximum detection range (in cells)
        max_range_cells = int(max(grid_width, grid_height) / grid_resolution)  # 5 meters detection range
        
        # Scan the occupancy grid and update the histogram
        for y in range(max(0, robot_cell_y - max_range_cells), 
                       min(height, robot_cell_y + max_range_cells + 1)):
            for x in range(max(0, robot_cell_x - max_range_cells), 
                           min(width, robot_cell_x + max_range_cells + 1)):
                if occupancy_grid[y, x] <= 0: # Skip free/unknown
                    continue
                
                # Calculate distance and angle relative to robot in grid frame
                dx_cell = x - robot_cell_x
                dy_cell = y - robot_cell_y
                distance = np.linalg.norm([dx_cell, dy_cell]) * grid_resolution
                
                # Angle relative to grid origin
                angle_grid = np.arctan2(dy_cell, dx_cell) 
                # Angle relative to robot's orientation
                angle_robot = normalize_angle(angle_grid - robot_theta) 
                
                # Convert angle to bin index
                bin_index = int(((angle_robot + np.pi) / (2 * np.pi)) * self.histogram_bins) % self.histogram_bins
                
                # Update histogram with modified scaling based on distance
                obstacle_value = occupancy_grid[y, x] / 100.0  # Normalize to 0-1 range
                
                if distance > 0:
                    # Use inverse square law for obstacles beyond safety threshold
                    histogram[bin_index] += obstacle_value / (distance ** 2)
        
        # Smooth histogram
        smoothed_histogram = np.zeros_like(histogram)
        for i in range(self.histogram_bins):
            smoothed_histogram[i] = (
                histogram[(i-1) % self.histogram_bins] * self.alpha +
                histogram[i] * (1 - 2*self.alpha) +
                histogram[(i+1) % self.histogram_bins] * self.alpha
            )
        
        return smoothed_histogram
    
    def select_direction(self, goal_weight: float, 
                               obstacle_weight: float, 
                               prev_direction_weight: float, 
                               histogram: np.ndarray, 
                               goal_direction: float) -> float:
        """ Select best direction (remains unchanged)."""
        if np.max(histogram) > 0:
            histogram = histogram / np.max(histogram)
        cost = np.zeros(self.histogram_bins)
        for i in range(self.histogram_bins):
            angle = (i / self.histogram_bins) * 2 * np.pi - np.pi
            obstacle_cost = obstacle_weight * histogram[i]
            angle_diff = abs(normalize_angle(angle - goal_direction))
            goal_cost = goal_weight * angle_diff
            prev_diff = abs(normalize_angle(angle - self.prev_selected_angle))
            prev_direction_cost = prev_direction_weight * prev_diff
            cost[i] = obstacle_cost + goal_cost + prev_direction_cost
        min_cost_idx = np.argmin(cost)
        selected_angle = (min_cost_idx / self.histogram_bins) * 2 * np.pi - np.pi
        self.prev_selected_angle = selected_angle
        return selected_angle

    def compute_pure_pursuit(self, goal_distance: float, goal_direction: float) -> Tuple[float, float]:
        """ Compute pure pursuit velocities with collision check."""
        if goal_distance < self.goal_tolerance:
            return 0.0, 0.0
        
        lookahead = min(self.lookahead_distance, goal_distance)
        linear_vel = min(self.max_linear_vel, goal_distance)
        angular_vel = 2.0 * np.sin(goal_direction) / lookahead
        angular_vel = max(-self.max_angular_vel, min(angular_vel, self.max_angular_vel))
        
        return linear_vel, angular_vel

    def check_collision(self, selected_direction: float) -> bool:
        """Check if there's an obstacle in the selected direction within safety threshold.
        
        Args:
            selected_direction: The selected direction of travel in radians
            
        Returns:
            bool: True if collision detected, False otherwise
        """
        # Get the latest costmap and robot pose
        costmap = self.robot.ros_control.get_costmap()
        if costmap is None:
            return False  # No costmap available
            
        occupancy_grid, grid_info, grid_origin = ros_msg_to_numpy_grid(costmap)
        _, _, grid_resolution = grid_info
        grid_origin_x, grid_origin_y, _ = grid_origin
        
        odom = self.robot.ros_control.get_odometry()
        if odom is None:
            return False  # No odometry available
            
        robot_pose = ros_msg_to_pose_tuple(odom)
        robot_x, robot_y, robot_theta = robot_pose
        
        # Convert robot position to grid coordinates
        robot_rel_x = robot_x - grid_origin_x
        robot_rel_y = robot_y - grid_origin_y
        robot_cell_x = int(robot_rel_x / grid_resolution)
        robot_cell_y = int(robot_rel_y / grid_resolution)
        
        # Direction in world frame
        direction_world = robot_theta + selected_direction
        
        # Safety distance in cells
        safety_cells = int(self.safety_threshold / grid_resolution)
        
        # Get grid dimensions
        height, width = occupancy_grid.shape
        
        # Check for obstacles along the selected direction
        for dist in range(1, safety_cells + 1):
            # Calculate cell position
            cell_x = robot_cell_x + int(dist * np.cos(direction_world))
            cell_y = robot_cell_y + int(dist * np.sin(direction_world))
            
            # Check if cell is within grid bounds
            if not (0 <= cell_x < width and 0 <= cell_y < height):
                continue
            
            # Check if cell contains an obstacle (threshold at 50)
            if occupancy_grid[cell_y, cell_x] > 50:
                logger.debug(f"Collision detected at distance {dist * grid_resolution:.2f}m")
                return True
                
        return False  # No collision detected

    def is_goal_reached(self) -> bool:
        """Check if the robot is within the goal tolerance distance."""
        # Return False immediately if goal is not set
        if self.goal_xy is None:
            return False
            
        odom = self.robot.ros_control.get_odometry()
        if odom is None: return False # Cannot check if no odometry
        robot_pose = ros_msg_to_pose_tuple(odom)
        robot_x, robot_y, _ = robot_pose
        goal_x, goal_y = self.goal_xy
        distance_to_goal = np.linalg.norm([goal_x - robot_x, goal_y - robot_y])
        return distance_to_goal < self.goal_tolerance

    def check_goal_collision(self) -> bool:
        """Check if the current goal is in collision with obstacles in the costmap.
        
        Returns:
            bool: True if goal is in collision, False if goal is safe or cannot be checked
        """
        if self.goal_xy is None:
            logger.warning("Cannot check collision: No goal set")
            return False
            
        costmap_msg = self.robot.ros_control.get_costmap()
        if costmap_msg is None:
            logger.warning("Cannot check collision: No costmap available")
            return False
            
        # Get costmap data
        occupancy_grid, grid_info, grid_origin = ros_msg_to_numpy_grid(costmap_msg)
        _, _, grid_resolution = grid_info
        grid_origin_x, grid_origin_y, _ = grid_origin
        height, width = occupancy_grid.shape
            
        # Convert goal from odom coordinates to grid cells
        goal_x, goal_y = self.goal_xy
        goal_rel_x = goal_x - grid_origin_x
        goal_rel_y = goal_y - grid_origin_y
        goal_cell_x = int(goal_rel_x / grid_resolution)
        goal_cell_y = int(goal_rel_y / grid_resolution)
            
        # Check if goal is within the costmap bounds
        if 0 <= goal_cell_x < width and 0 <= goal_cell_y < height:
            # Check the occupancy value at the goal
            occupancy_value = occupancy_grid[goal_cell_y, goal_cell_x]
            collision_threshold = 80  # Consider values above 80 as obstacles
            
            is_collision = occupancy_value >= collision_threshold
            if is_collision:
                logger.warning(f"Goal is in collision: occupancy value = {occupancy_value}")
            return is_collision
        else:
            logger.warning(f"Goal ({goal_cell_x}, {goal_cell_y}) is outside costmap bounds")
            return False  # Can't determine collision if outside bounds

    def adjust_goal_to_valid_position(self, goal_xy: Tuple[float, float], minimum_distance: float = None) -> Tuple[float, float]:
        """Find a valid (non-colliding) goal position by moving it towards the robot.
        
        Args:
            goal_xy: Original goal position (x, y) in odom frame
            minimum_distance: Minimum distance in meters to keep from obstacles.
                             If None, uses the planner's safety_threshold.
        
        Returns:
            Tuple[float, float]: A valid goal position, or the original goal if already valid
        """
            
        # Get robot position
        odom = self.robot.ros_control.get_odometry()
        if odom is None:
            return goal_xy
            
        robot_pose = ros_msg_to_pose_tuple(odom)
        robot_x, robot_y = robot_pose[0], robot_pose[1]
        
        # Original goal
        goal_x, goal_y = goal_xy
        
        # Calculate vector from goal to robot
        dx = robot_x - goal_x
        dy = robot_y - goal_y
        distance = np.sqrt(dx*dx + dy*dy)
        
        if distance < 0.001:  # Goal is at robot position
            return goal_xy
            
        # Normalize direction vector
        dx /= distance
        dy /= distance
        
        # Step size
        step_size = 0.25  # meters
        
        # Move goal towards robot step by step
        current_x, current_y = goal_x, goal_y
        steps = 0
        max_steps = 50  # Safety limit
        
        while steps < max_steps:
            # Move towards robot
            current_x += dx * step_size
            current_y += dy * step_size
            steps += 1
            
            # Check if we've reached or passed the robot
            new_distance = np.sqrt((current_x - robot_x)**2 + (current_y - robot_y)**2)
            if new_distance < step_size:
                # We've reached the robot without finding a valid point
                # Move back one step from robot to avoid self-collision
                current_x = robot_x - dx * step_size
                current_y = robot_y - dy * step_size
                break
                
            # Check if this position is valid
            self.goal_xy = (current_x, current_y)
            if not self.check_goal_collision():
                logger.info(f"Found valid goal at ({current_x:.2f}, {current_y:.2f})")
                return (current_x, current_y)
                
        logger.warning(f"Could not find valid goal after {steps} steps, using closest point to robot")
        return (current_x, current_y)
