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
                 max_linear_vel: float = 0.5,
                 max_angular_vel: float = 1.0,
                 lookahead_distance: float = 1.0,
                 goal_tolerance: float = 1.0,
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
        self.alpha = 0.1  # Histogram smoothing factor
        self.obstacle_weight = 1.5
        self.goal_weight = 1.0
        self.prev_direction_weight = 0.5
        self.prev_selected_angle = 0.0
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
            goal_safe = self.adjust_goal_for_collision()
            logger.info(f"Goal set in robot frame ({goal_x_robot:.2f}, {goal_y_robot:.2f}), "
                        f"transformed to odom frame: ({self.goal_xy[0]:.2f}, {self.goal_xy[1]:.2f})")
            if not goal_safe:
                logger.warning("Goal is in collision. Adjusted goal to safe position.")
        else:
            self.goal_xy = goal_xy
            goal_safe = self.adjust_goal_for_collision()
            logger.info(f"Goal set directly in odom frame: ({self.goal_xy[0]:.2f}, {self.goal_xy[1]:.2f})")
            if not goal_safe:
                logger.warning("Goal is in collision. Adjusted goal to safe position.")

    def plan(self) -> Dict[str, float]:
        """
        Compute velocity commands using VFH + Pure Pursuit.
        """
        costmap = self.robot.ros_control.get_costmap()
        occupancy_grid, grid_resolution, _ = ros_msg_to_numpy_grid(costmap)  # Use imported function
        
        odom = self.robot.ros_control.get_odometry()
        robot_pose = ros_msg_to_pose_tuple(odom)  # Use imported function
        
        goal_xy = self.goal_xy
        self.adjust_goal_for_collision()
        robot_x, robot_y, robot_theta = robot_pose
        goal_x, goal_y = goal_xy
        
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        goal_distance = np.linalg.norm([dx, dy])
        goal_direction = np.arctan2(dy, dx) - robot_theta
        goal_direction = normalize_angle(goal_direction)  # Use imported function
        
        if goal_distance < self.goal_tolerance:
            return {'x_vel': 0.0, 'angular_vel': 0.0}
        
        self.histogram = self.build_polar_histogram(occupancy_grid, grid_resolution, robot_pose)
        self.selected_direction = self.select_direction(self.histogram, goal_direction)
        linear_vel, angular_vel = self.compute_pure_pursuit(goal_distance, self.selected_direction)
        
        return {'x_vel': linear_vel, 'angular_vel': angular_vel}
    
    def update_visualization(self) -> np.ndarray:
        """
        Generate visualization by calling the utility function.
        """
        try:
            costmap = self.robot.ros_control.get_costmap()
            odom = self.robot.ros_control.get_odometry()
            
            occupancy_grid, grid_resolution, grid_origin = ros_msg_to_numpy_grid(costmap)
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
                              grid_resolution: float,
                              robot_pose: Tuple[float, float, float]) -> np.ndarray:
        """ Build polar histogram (remains unchanged)."""
        # Initialize histogram
        histogram = np.zeros(self.histogram_bins)
        
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
        max_range_cells = int(5.0 / grid_resolution)  # 5 meters detection range
        
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
                
                if distance > 5.0: continue # Skip if beyond sensor range
                
                # Angle relative to grid origin
                angle_grid = np.arctan2(dy_cell, dx_cell)
                # Angle relative to robot's orientation
                angle_robot = normalize_angle(angle_grid - robot_theta)
                
                # Convert angle to bin index
                bin_index = int(((angle_robot + np.pi) / (2 * np.pi)) * self.histogram_bins) % self.histogram_bins
                
                # Update histogram
                obstacle_value = occupancy_grid[y, x] / 100.0  # Normalize
                if distance > 0:
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
    
    def select_direction(self, histogram: np.ndarray, goal_direction: float) -> float:
        """ Select best direction (remains unchanged)."""
        if np.max(histogram) > 0:
            histogram = histogram / np.max(histogram)
        cost = np.zeros(self.histogram_bins)
        for i in range(self.histogram_bins):
            angle = (i / self.histogram_bins) * 2 * np.pi - np.pi
            obstacle_cost = self.obstacle_weight * histogram[i]
            angle_diff = abs(normalize_angle(angle - goal_direction))
            goal_cost = self.goal_weight * angle_diff
            prev_diff = abs(normalize_angle(angle - self.prev_selected_angle))
            prev_direction_cost = self.prev_direction_weight * prev_diff
            cost[i] = obstacle_cost + goal_cost + prev_direction_cost
        min_cost_idx = np.argmin(cost)
        selected_angle = (min_cost_idx / self.histogram_bins) * 2 * np.pi - np.pi
        self.prev_selected_angle = selected_angle
        return selected_angle

    def compute_pure_pursuit(self, goal_distance: float, goal_direction: float) -> Tuple[float, float]:
        """ Compute pure pursuit velocities (remains unchanged)."""
        if goal_distance < self.goal_tolerance:
            return 0.0, 0.0
        lookahead = min(self.lookahead_distance, goal_distance)
        linear_vel = min(self.max_linear_vel, goal_distance)
        angular_vel = 2.0 * np.sin(goal_direction) / lookahead
        angular_vel = max(-self.max_angular_vel, min(angular_vel, self.max_angular_vel))
        return linear_vel, angular_vel

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

    def adjust_goal_for_collision(self,
                                       collision_threshold: int = 99,
                                       search_steps: int = 50,
                                       buffer_distance_meters: float = 0.5,
                                       neighbor_search_radius_meters: float = 1.0) -> bool:
        """Checks if the current goal is in collision. If so, searches backwards
           along the line from the goal towards the robot to find the closest cell
           that is both collision-free and at least `buffer_distance_meters` away
           from any colliding cell. Updates self.goal_xy if found.

        Args:
            collision_threshold: Occupancy value (0-100) considered an obstacle.
            search_steps: Max steps to check along the line segment.
            buffer_distance_meters: Required minimum distance from the candidate cell
                                    center to the nearest colliding cell center.
            neighbor_search_radius_meters: How far around a candidate cell to search
                                           for colliding neighbors.

        Returns:
            bool: True if the original goal was safe or check failed/no suitable adjustment found,
                  False if the original goal was in collision and a suitable buffered goal was found and set.
        """
        if self.goal_xy is None:
            logger.warning("Goal not set, cannot adjust.")
            return True

        costmap_msg = self.robot.ros_control.get_costmap()
        odom_msg = self.robot.ros_control.get_odometry()
        if costmap_msg is None or odom_msg is None:
            logger.warning("Cannot adjust goal: Costmap or Odometry data unavailable.")
            return True

        occupancy_grid, grid_resolution, grid_origin = ros_msg_to_numpy_grid(costmap_msg)
        if grid_resolution <= 1e-6: # Use a small epsilon for float comparison
             logger.error("Invalid grid resolution (<= 0) for collision check.")
             return True
        grid_origin_x, grid_origin_y, _ = grid_origin
        height, width = occupancy_grid.shape
        robot_pose = ros_msg_to_pose_tuple(odom_msg)
        robot_x, robot_y, _ = robot_pose

        original_goal_x_odom, original_goal_y_odom = self.goal_xy

        # Transform original goal to grid cell coordinates
        goal_rel_x = original_goal_x_odom - grid_origin_x
        goal_rel_y = original_goal_y_odom - grid_origin_y
        goal_cell_x = int(goal_rel_x / grid_resolution)
        goal_cell_y = int(goal_rel_y / grid_resolution)

        if not (0 <= goal_cell_x < width and 0 <= goal_cell_y < height):
            logger.warning("Original goal is outside costmap. No adjustment needed.")
            return True

        # Check if original goal needs adjustment
        original_cell_value = occupancy_grid[goal_cell_y, goal_cell_x]
        if original_cell_value < collision_threshold:
            logger.debug("Original goal cell is safe. No adjustment needed.")
            return True

        logger.warning(f"Original goal ({goal_cell_x}, {goal_cell_y}) colliding (val: {original_cell_value}). Searching for buffered safe cell...")

        # Transform robot pose to grid cell coordinates
        robot_rel_x = robot_x - grid_origin_x
        robot_rel_y = robot_y - grid_origin_y
        robot_cell_x = int(robot_rel_x / grid_resolution)
        robot_cell_y = int(robot_rel_y / grid_resolution)

        found_buffered_goal = False
        search_radius_cells = max(1, int(neighbor_search_radius_meters / grid_resolution))

        # Iterate backwards from goal towards robot
        for i in range(search_steps - 1, -1, -1):
            t = i / max(1, search_steps - 1)
            check_cell_x = int(robot_cell_x + t * (goal_cell_x - robot_cell_x))
            check_cell_y = int(robot_cell_y + t * (goal_cell_y - robot_cell_y))

            if not (0 <= check_cell_x < width and 0 <= check_cell_y < height):
                continue # Skip cells outside map

            # 1. Check if the candidate cell itself is safe
            cell_value = occupancy_grid[check_cell_y, check_cell_x]
            if cell_value < collision_threshold:
                # 2. Find min distance to nearest colliding cell within radius
                min_dist_sq_to_collision = float('inf')
                collision_found_nearby = False
                for dx in range(-search_radius_cells, search_radius_cells + 1):
                    for dy in range(-search_radius_cells, search_radius_cells + 1):
                        if dx == 0 and dy == 0: continue # Skip self check

                        nx, ny = check_cell_x + dx, check_cell_y + dy
                        if 0 <= nx < width and 0 <= ny < height:
                            neighbor_value = occupancy_grid[ny, nx]
                            if neighbor_value >= collision_threshold:
                                collision_found_nearby = True
                                dist_sq = float(dx*dx + dy*dy) # Ensure float for sqrt
                                min_dist_sq_to_collision = min(min_dist_sq_to_collision, dist_sq)

                # Calculate distance only if collision was found nearby
                min_dist_to_collision = float('inf')
                if collision_found_nearby:
                     # Avoid sqrt of infinity if no collision found, though logic prevents this case here
                     if min_dist_sq_to_collision != float('inf'):
                           min_dist_to_collision = np.sqrt(min_dist_sq_to_collision) * grid_resolution

                # 3. Check if buffer distance is met (either no collision nearby or closest is far enough)
                if min_dist_to_collision >= buffer_distance_meters:
                    # Found a suitable cell
                    new_goal_cell_x = check_cell_x
                    new_goal_cell_y = check_cell_y

                    # Convert safe cell center back to odom coordinates
                    new_goal_x_odom = grid_origin_x + (new_goal_cell_x + 0.5) * grid_resolution
                    new_goal_y_odom = grid_origin_y + (new_goal_cell_y + 0.5) * grid_resolution

                    self.goal_xy = (new_goal_x_odom, new_goal_y_odom)
                    logger.warning(f"Adjusted goal to buffered safe cell ({new_goal_cell_x}, {new_goal_cell_y}) "
                                   f"at odom ({new_goal_x_odom:.2f}, {new_goal_y_odom:.2f}). "
                                   f"Min dist to collision: {min_dist_to_collision:.2f}m >= {buffer_distance_meters:.2f}m.")
                    found_buffered_goal = True
                    break # Stop searching

        if not found_buffered_goal:
            logger.error(f"Could not find a safe cell with {buffer_distance_meters}m buffer. Goal remains unchanged at colliding position.")
            return True # No suitable adjustment found

        # Return False because original goal was colliding and we made an adjustment
        return False
