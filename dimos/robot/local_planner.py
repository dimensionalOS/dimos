#!/usr/bin/env python3

import math
import numpy as np
from typing import Dict, Tuple, Optional, List, Any
from abc import ABC, abstractmethod
import cv2
from reactivex import Observable
from reactivex.subject import Subject
import threading
import time
import logging
from dimos.utils.logging_config import setup_logger
from dimos.utils.ros_utils import (
    ros_msg_to_numpy_grid, 
    normalize_angle,
    visualize_local_planner_state,
    distance_angle_to_goal_xy
)

from dimos.robot.robot import Robot
from dimos.types.vector import VectorLike, Vector, to_tuple
from dimos.types.path import Path
from nav_msgs.msg import OccupancyGrid

logger = setup_logger("dimos.robot.unitree.local_planner", level=logging.DEBUG)

class BaseLocalPlanner(ABC):
    """
    Abstract base class for local planners that handle obstacle avoidance and path following.
    
    This class defines the common interface and shared functionality that all local planners 
    must implement, regardless of the specific algorithm used.
    """
    
    def __init__(self, 
                 robot: Robot,
                 safety_threshold: float = 0.8,
                 max_linear_vel: float = 0.8,
                 max_angular_vel: float = 1.0,
                 lookahead_distance: float = 1.0,
                 goal_tolerance: float = 0.2,
                 angle_tolerance: float = 0.1,  # ~5.7 degrees
                 robot_width: float = 0.5,
                 robot_length: float = 0.7,
                 visualization_size: int = 400):
        """
        Initialize the base local planner.
        
        Args:
            robot: Robot instance to get data from and send commands to
            safety_threshold: Distance to maintain from obstacles (meters)
            max_linear_vel: Maximum linear velocity (m/s)
            max_angular_vel: Maximum angular velocity (rad/s)
            lookahead_distance: Lookahead distance for path following (meters)
            goal_tolerance: Distance at which the goal is considered reached (meters)
            angle_tolerance: Angle at which the goal orientation is considered reached (radians)
            robot_width: Width of the robot for visualization (meters)
            robot_length: Length of the robot for visualization (meters)
            visualization_size: Size of the visualization image in pixels
        """
        self.robot = robot
        self.safety_threshold = safety_threshold
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.lookahead_distance = lookahead_distance
        self.goal_tolerance = goal_tolerance
        self.angle_tolerance = angle_tolerance
        self.robot_width = robot_width
        self.robot_length = robot_length
        self.visualization_size = visualization_size

        # Goal and Waypoint Tracking
        self.goal_xy: Optional[Tuple[float, float]] = None  # Current target for planning
        self.goal_theta: Optional[float] = None  # Goal orientation in odom frame
        self.position_reached: bool = False  # Flag indicating if position goal is reached
        self.waypoints: Optional[Path] = None             # Full path if following waypoints
        self.waypoints_in_odom: Optional[Path] = None     # Full path in odom frame
        self.waypoint_frame: Optional[str] = None         # Frame of the waypoints
        self.current_waypoint_index: int = 0              # Index of the next waypoint to reach
        self.final_goal_reached: bool = False             # Flag indicating if the final waypoint is reached

        # topics
        self.local_costmap = self.robot.ros_control.topic_latest("/local_costmap/costmap", OccupancyGrid)

    def set_goal(self, goal_xy: VectorLike, frame: str = "odom", goal_theta: Optional[float] = None):
        """Set a single goal position, converting to odom frame if necessary.
           This clears any existing waypoints being followed.

        Args:
            goal_xy: The goal position to set.
            frame: The frame of the goal position.
            goal_theta: Optional goal orientation in radians (in the specified frame)
        """
        # Clear waypoint following state
        self.waypoints = None
        self.current_waypoint_index = 0
        self.final_goal_reached = False
        self.position_reached = False
        self.goal_xy = None # Clear previous goal
        self.goal_theta = None # Clear previous goal orientation

        target_goal_xy: Optional[Tuple[float, float]] = None

        target_goal_xy = self.robot.ros_control.transform_point(goal_xy, source_frame=frame, target_frame="odom").to_tuple()

        logger.info(f"Goal set directly in odom frame: ({target_goal_xy[0]:.2f}, {target_goal_xy[1]:.2f})")
        
        # Check if goal is valid (in bounds and not colliding)
        if not self.is_goal_in_costmap_bounds(target_goal_xy) or self.check_goal_collision(target_goal_xy):
            logger.warning("Goal is in collision or out of bounds. Adjusting goal to valid position.")
            self.goal_xy = self.adjust_goal_to_valid_position(target_goal_xy)
        else:
            self.goal_xy = target_goal_xy # Set the adjusted or original valid goal
            
        # Set goal orientation if provided
        if goal_theta is not None:
            transformed_rot = self.robot.ros_control.transform_rot(Vector(0.0, 0.0, goal_theta), source_frame=frame, target_frame="odom")
            self.goal_theta = transformed_rot[2]

    def set_goal_waypoints(self, waypoints: Path, frame: str = "map", goal_theta: Optional[float] = None):
        """Sets a path of waypoints for the robot to follow. 

        Args:
            waypoints: A list of waypoints to follow. Each waypoint is a tuple of (x, y) coordinates in odom frame.
            frame: The frame of the waypoints.
            goal_theta: Optional final orientation in radians (in the specified frame)
        """

        if not isinstance(waypoints, Path) or len(waypoints) == 0:
            logger.warning("Invalid or empty path provided to set_goal_waypoints. Ignoring.")
            self.waypoints = None
            self.waypoint_frame = None
            self.goal_xy = None
            self.goal_theta = None
            self.current_waypoint_index = 0
            self.final_goal_reached = False
            self.position_reached = False
            return

        logger.info(f"Setting goal waypoints with {len(waypoints)} points.")
        self.waypoints = waypoints
        self.waypoint_frame = frame
        self.current_waypoint_index = 0
        self.final_goal_reached = False
        self.position_reached = False

        # Transform waypoints to odom frame
        self.waypoints_in_odom = self.robot.ros_control.transform_path(self.waypoints, source_frame=frame, target_frame="odom")
        
        # Set the initial target to the first waypoint, adjusting if necessary
        first_waypoint = self.waypoints_in_odom[0]
        if not self.is_goal_in_costmap_bounds(first_waypoint) or self.check_goal_collision(first_waypoint):
            logger.warning("First waypoint is invalid. Adjusting...")
            self.goal_xy = self.adjust_goal_to_valid_position(first_waypoint)
        else:
            self.goal_xy = to_tuple(first_waypoint) # Initial target
            
        # Set goal orientation if provided
        if goal_theta is not None:
            transformed_rot = self.robot.ros_control.transform_rot(Vector(0.0, 0.0, goal_theta), source_frame=frame, target_frame="odom")
            self.goal_theta = transformed_rot[2]

    def _update_waypoint_target(self, robot_pos_np: np.ndarray) -> bool:
        """Helper function to manage waypoint progression and update the target goal.
        
        Args:
            robot_pos_np: Current robot position as a numpy array [x, y].
            
        Returns:
            bool: True if the final waypoint has just been reached, False otherwise.
        """
        if self.waypoints is None or len(self.waypoints) == 0:
            return False  # Not in waypoint mode or empty path
        
        self.waypoints_in_odom = self.robot.ros_control.transform_path(self.waypoints, source_frame=self.waypoint_frame, target_frame="odom")

        # Check if final goal is reached
        final_waypoint = self.waypoints_in_odom[-1]
        dist_to_final = np.linalg.norm(robot_pos_np - final_waypoint)
        
        if dist_to_final < self.goal_tolerance:
            self.position_reached = True
            self.goal_xy = to_tuple(final_waypoint)
            
            # If goal orientation is not specified or achieved, consider fully reached
            if self.goal_theta is None or self._is_goal_orientation_reached():
                self.final_goal_reached = True
                logger.info("Reached final waypoint with correct orientation.")
                return True
            else:
                logger.info("Reached final waypoint position, rotating to target orientation.")
                return False
            
        # Always find the lookahead point
        lookahead_point = None
        for i in range(self.current_waypoint_index, len(self.waypoints_in_odom)):
            wp = self.waypoints_in_odom[i]
            dist_to_wp = np.linalg.norm(robot_pos_np - wp)
            if dist_to_wp >= self.lookahead_distance:
                lookahead_point = wp
                # Update current waypoint index to this point
                self.current_waypoint_index = i
                break
        
        # If no point is far enough, target the final waypoint
        if lookahead_point is None:
            lookahead_point = self.waypoints_in_odom[-1]
            self.current_waypoint_index = len(self.waypoints_in_odom) - 1
        
        # Set the lookahead point as the immediate target, adjusting if needed
        if not self.is_goal_in_costmap_bounds(lookahead_point) or self.check_goal_collision(lookahead_point):
            logger.debug("Lookahead point is invalid. Adjusting...")
            adjusted_lookahead = self.adjust_goal_to_valid_position(lookahead_point)
            # Only update if adjustment didn't fail completely
            if adjusted_lookahead is not None:
                self.goal_xy = adjusted_lookahead
        else:
            self.goal_xy = to_tuple(lookahead_point)
                
        return False  # Final goal not reached in this update cycle

    def _is_goal_orientation_reached(self) -> bool:
        """Check if the current robot orientation matches the goal orientation.
        
        Returns:
            bool: True if orientation is reached or no orientation goal is set
        """
        if self.goal_theta is None:
            return True  # No orientation goal set
            
        # Get current robot orientation in odom frame
        [_, rot] = self.robot.ros_control.transform_euler("base_link", "odom")
        _, _, robot_theta = rot  # yaw component
        
        # Calculate the angle difference and normalize
        angle_diff = abs(normalize_angle(self.goal_theta - robot_theta))
        
        logger.debug(f"Orientation error: {angle_diff:.4f} rad, tolerance: {self.angle_tolerance:.4f} rad")
        return angle_diff <= self.angle_tolerance

    def plan(self) -> Dict[str, float]:
        """
        Main planning method that computes velocity commands.
        This includes common planning logic like waypoint following,
        with algorithm-specific calculations delegated to subclasses.
        
        Returns:
            Dict[str, float]: Velocity commands with 'x_vel' and 'angular_vel' keys
        """
        # --- Waypoint Following Mode --- 
        if self.waypoints is not None:
            if self.final_goal_reached:
                logger.info("Final waypoint reached. Stopping.")
                return {'x_vel': 0.0, 'angular_vel': 0.0}
            
            # Get current robot pose
            [pos, rot] = self.robot.ros_control.transform_euler("base_link", "odom")
            robot_x, robot_y, robot_theta = pos[0], pos[1], rot[2]
            robot_pos_np = np.array([robot_x, robot_y])
            
            # Update the target goal based on waypoint progression
            just_reached_final = self._update_waypoint_target(robot_pos_np)
            
            # If the helper indicates the final goal was just reached, stop immediately
            if just_reached_final:
                 return {'x_vel': 0.0, 'angular_vel': 0.0}
                 
            # Check if position is reached but orientation isn't
            if self.position_reached and self.goal_theta is not None and not self._is_goal_orientation_reached():
                # We need to rotate in place to match the goal orientation
                return self._rotate_to_goal_orientation()

        # --- Single Goal or Current Waypoint Target Set --- 
        if self.goal_xy is None:
            # If no goal is set (e.g., empty path or rejected goal), stop.
            return {'x_vel': 0.0, 'angular_vel': 0.0}

        # Get necessary data for planning
        costmap = self.local_costmap()
        if costmap is None:
            logger.warning("Local costmap is None. Cannot plan.")
            return {'x_vel': 0.0, 'angular_vel': 0.0}
        
        # Check if we are in position goal reached state for single goal
        if self.waypoints is None:
            # Get current robot pose
            [pos, _] = self.robot.ros_control.transform_euler("base_link", "odom")
            robot_x, robot_y = pos[0], pos[1]
            goal_x, goal_y = self.goal_xy
            goal_distance = np.linalg.norm([goal_x - robot_x, goal_y - robot_y])
            
            # First check position
            if goal_distance < self.goal_tolerance:
                self.position_reached = True
                
                # If goal orientation is specified, rotate to match it
                if self.goal_theta is not None and not self._is_goal_orientation_reached():
                    logger.info("Position goal reached. Rotating to target orientation.")
                    return self._rotate_to_goal_orientation()
                else:
                    logger.info("Single goal reached.")
                    return {'x_vel': 0.0, 'angular_vel': 0.0}
            else:
                self.position_reached = False
        
        # Call the algorithm-specific planning implementation
        return self._compute_velocity_commands()

    @abstractmethod
    def _compute_velocity_commands(self) -> Dict[str, float]:
        """
        Algorithm-specific method to compute velocity commands.
        Must be implemented by derived classes.
        
        Returns:
            Dict[str, float]: Velocity commands with 'x_vel' and 'angular_vel' keys
        """
        pass

    def _rotate_to_goal_orientation(self) -> Dict[str, float]:
        """Compute velocity commands to rotate to the goal orientation.
        
        Returns:
            Dict[str, float]: Velocity commands with zero linear velocity
        """
        # Get current robot orientation
        [_, rot] = self.robot.ros_control.transform_euler("base_link", "odom")
        _, _, robot_theta = rot
        
        # Calculate the angle difference
        angle_diff = normalize_angle(self.goal_theta - robot_theta)
        
        # Determine rotation direction and speed
        if abs(angle_diff) < self.angle_tolerance:
            # Already at correct orientation
            return {'x_vel': 0.0, 'angular_vel': 0.0}
            
        # Calculate rotation speed - proportional to the angle difference
        # but capped at max_angular_vel
        direction = 1.0 if angle_diff > 0 else -1.0
        angular_vel = direction * min(abs(angle_diff) * 2.0, self.max_angular_vel)
        
        # logger.debug(f"Rotating to goal orientation: angle_diff={angle_diff:.4f}, angular_vel={angular_vel:.4f}")
        return {'x_vel': 0.0, 'angular_vel': angular_vel}

    @abstractmethod
    def update_visualization(self) -> np.ndarray:
        """
        Generate visualization of the planning state.
        Must be implemented by derived classes.
        
        Returns:
            np.ndarray: Visualization image as numpy array
        """
        pass

    def create_stream(self, frequency_hz: float = 10.0) -> Observable:
        """
        Create an Observable stream that emits the visualization image at a fixed frequency.
        """
        subject = Subject()
        sleep_time = 1.0 / frequency_hz
        
        def frame_emitter():
            while True:
                try:
                    # Generate the frame using the updated method
                    frame = self.update_visualization() 
                    subject.on_next(frame)
                except Exception as e:
                    logger.error(f"Error in frame emitter thread: {e}")
                    # Optionally, emit an error frame or simply skip
                    # subject.on_error(e) # This would terminate the stream
                time.sleep(sleep_time)
        
        emitter_thread = threading.Thread(target=frame_emitter, daemon=True)
        emitter_thread.start()
        logger.info("Started visualization frame emitter thread")
        return subject
    
    @abstractmethod
    def check_collision(self, direction: float) -> bool:
        """
        Check if there's a collision in the given direction.
        Must be implemented by derived classes.
        
        Args:
            direction: Direction to check for collision in radians
            
        Returns:
            bool: True if collision detected, False otherwise
        """
        pass

    def is_goal_reached(self) -> bool:
        """Check if the final goal (single or last waypoint) is reached, including orientation."""
        if self.waypoints is not None:
            # Waypoint mode: check if the final waypoint and orientation have been reached
            return self.final_goal_reached
        else:
            # Single goal mode: check distance to the single goal and orientation
            if self.goal_xy is None:
                return False # No goal set
            
            [pos, rot] = self.robot.ros_control.transform_euler("base_link", "odom")
            robot_x, robot_y = pos[0], pos[1]
            
            goal_x, goal_y = self.goal_xy
            distance_to_goal = np.linalg.norm([goal_x - robot_x, goal_y - robot_y])
            
            # First check position
            position_reached = distance_to_goal < self.goal_tolerance
            
            # Then check orientation if a goal orientation was specified
            if position_reached and self.goal_theta is not None:
                return self._is_goal_orientation_reached()
                
            return position_reached

    def check_goal_collision(self, goal_xy: VectorLike) -> bool:
        """Check if the current goal is in collision with obstacles in the costmap.
        
        Returns:
            bool: True if goal is in collision, False if goal is safe or cannot be checked
        """
            
        costmap_msg = self.local_costmap()
        if costmap_msg is None:
            logger.warning("Cannot check collision: No costmap available")
            return False
            
        # Get costmap data
        occupancy_grid, grid_info, grid_origin = ros_msg_to_numpy_grid(costmap_msg)
        _, _, grid_resolution = grid_info
        grid_origin_x, grid_origin_y, _ = grid_origin
        height, width = occupancy_grid.shape
            
        # Convert goal from odom coordinates to grid cells
        goal_x, goal_y = goal_xy
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

    def is_goal_in_costmap_bounds(self, goal_xy: VectorLike) -> bool:
        """Check if the goal position is within the bounds of the costmap.
        
        Args:
            goal_xy: Goal position (x, y) in odom frame
            
        Returns:
            bool: True if the goal is within the costmap bounds, False otherwise
        """
        costmap_msg = self.local_costmap()
        if costmap_msg is None:
            logger.warning("Cannot check bounds: No costmap available")
            return False
            
        # Get costmap data
        occupancy_grid, grid_info, grid_origin = ros_msg_to_numpy_grid(costmap_msg)
        _, _, grid_resolution = grid_info
        grid_origin_x, grid_origin_y, _ = grid_origin
        grid_width, grid_height = occupancy_grid.shape
        # Convert goal from odom coordinates to grid cells
        goal_x, goal_y = to_tuple(goal_xy)
        goal_rel_x = goal_x - grid_origin_x
        goal_rel_y = goal_y - grid_origin_y
        goal_cell_x = int(goal_rel_x / grid_resolution)
        goal_cell_y = int(goal_rel_y / grid_resolution)
        
        # Check if goal is within the costmap bounds
        is_in_bounds = 0 <= goal_cell_x < grid_width and 0 <= goal_cell_y < grid_height
        
        if not is_in_bounds:
            logger.warning(f"Goal ({goal_x:.2f}, {goal_y:.2f}) is outside costmap bounds")
            
        return is_in_bounds

    def adjust_goal_to_valid_position(self, goal_xy: VectorLike) -> Tuple[float, float]:
        """Find a valid (non-colliding) goal position by moving it towards the robot.
        
        Args:
            goal_xy: Original goal position (x, y) in odom frame
        
        Returns:
            Tuple[float, float]: A valid goal position, or the original goal if already valid
        """    
        [pos, rot] = self.robot.ros_control.transform_euler("base_link", "odom")
        
        robot_x, robot_y = pos[0], pos[1]
        
        # Original goal
        goal_x, goal_y = to_tuple(goal_xy)
        
        # Calculate vector from goal to robot
        dx = robot_x - goal_x
        dy = robot_y - goal_y
        distance = np.sqrt(dx*dx + dy*dy)
        
        if distance < 0.001:  # Goal is at robot position
            return to_tuple(goal_xy)
            
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
            if not self.check_goal_collision((current_x, current_y)) and self.is_goal_in_costmap_bounds((current_x, current_y)):
                logger.info(f"Found valid goal at ({current_x:.2f}, {current_y:.2f})")
                return (current_x, current_y)
                
        logger.warning(f"Could not find valid goal after {steps} steps, using closest point to robot")
        return (current_x, current_y)


class VFHPurePursuitPlanner(BaseLocalPlanner):
    """
    A local planner that combines Vector Field Histogram (VFH) for obstacle avoidance
    with Pure Pursuit for goal tracking.
    """
    
    def __init__(self, 
                 robot: Robot,
                 safety_threshold: float = 0.8,
                 histogram_bins: int = 144,
                 max_linear_vel: float = 0.8,
                 max_angular_vel: float = 1.0,
                 lookahead_distance: float = 1.0,
                 goal_tolerance: float = 0.2,
                 angle_tolerance: float = 0.1,  # ~5.7 degrees
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
            angle_tolerance: Angle at which the goal orientation is considered reached (radians)
            robot_width: Width of the robot for visualization (meters)
            robot_length: Length of the robot for visualization (meters)
            visualization_size: Size of the visualization image in pixels
        """
        super().__init__(
            robot=robot,
            safety_threshold=safety_threshold,
            max_linear_vel=max_linear_vel,
            max_angular_vel=max_angular_vel,
            lookahead_distance=lookahead_distance,
            goal_tolerance=goal_tolerance,
            angle_tolerance=angle_tolerance,
            robot_width=robot_width,
            robot_length=robot_length,
            visualization_size=visualization_size
        )
        
        # VFH specific parameters
        self.histogram_bins = histogram_bins
        self.histogram = None
        self.selected_direction = None
        
        # VFH tuning parameters
        self.alpha = 0.15  # Histogram smoothing factor
        self.obstacle_weight = 2.0
        self.goal_weight = 1.0
        self.prev_direction_weight = 0.7
        self.prev_selected_angle = 0.0
        self.goal_distance_scale_factor = 3.0
        self.prev_linear_vel = 0.0
        self.linear_vel_filter_factor = 0.4
        self.low_speed_nudge = 0.1

    def _compute_velocity_commands(self) -> Dict[str, float]:
        """
        VFH + Pure Pursuit specific implementation of velocity command computation.
        
        Returns:
            Dict[str, float]: Velocity commands with 'x_vel' and 'angular_vel' keys
        """
        # Get necessary data for planning
        costmap = self.local_costmap()
        occupancy_grid, grid_info, _ = ros_msg_to_numpy_grid(costmap)
        
        [pos, rot] = self.robot.ros_control.transform_euler("base_link", "odom")
        robot_x, robot_y, robot_theta = pos[0], pos[1], rot[2]
        robot_pose = (robot_x, robot_y, robot_theta)
        
        # Calculate goal-related parameters
        goal_x, goal_y = self.goal_xy
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        goal_distance = np.linalg.norm([dx, dy])
        goal_direction = np.arctan2(dy, dx) - robot_theta
        goal_direction = normalize_angle(goal_direction)
        
        # Calculate VFH/Direction Selection
        goal_distance_scale = 1.0
        if goal_distance < self.goal_tolerance * self.goal_distance_scale_factor:
            goal_distance_scale = 1.0 / (goal_distance / (self.goal_tolerance * self.goal_distance_scale_factor))

        self.histogram = self.build_polar_histogram(occupancy_grid, grid_info, robot_pose)
        self.selected_direction = self.select_direction(
            self.goal_weight * goal_distance_scale,
            self.obstacle_weight / goal_distance_scale,
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
            logger.debug(f"Slowing for turn: factor={turn_factor:.2f}")
            linear_vel *= turn_factor
        
        # Apply Collision Avoidance Stop
        if self.check_collision(0.0):
            logger.debug("Collision detected ahead. Slowing down.")
            # Re-select direction prioritizing obstacle avoidance if colliding
            self.selected_direction = self.select_direction(
                self.goal_weight * 0.5,
                self.obstacle_weight,
                0.0, # Zero prev direction weight
                self.histogram,
                goal_direction
            )
            _, angular_vel = self.compute_pure_pursuit(goal_distance, self.selected_direction)
            linear_vel = 0.0

        self.prev_linear_vel = linear_vel
        filtered_linear_vel = self.prev_linear_vel * self.linear_vel_filter_factor + linear_vel * (1 - self.linear_vel_filter_factor)

        return {'x_vel': filtered_linear_vel, 'angular_vel': angular_vel}
        
    def build_polar_histogram(self, 
                              occupancy_grid: np.ndarray, 
                              grid_info: Tuple[int, int, float],
                              robot_pose: Tuple[float, float, float]) -> np.ndarray:
        """ Build polar histogram from occupancy grid."""
        # Initialize histogram
        histogram = np.zeros(self.histogram_bins)
        grid_width, grid_height, grid_resolution = grid_info
        
        # Extract robot position in grid coordinates
        robot_x, robot_y, robot_theta = robot_pose
        
        # Need grid origin to calculate robot position relative to grid
        costmap = self.local_costmap()
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
        """ Select best direction based on weighted cost function."""
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
        """ Compute pure pursuit velocities."""
        if goal_distance < self.goal_tolerance:
            return 0.0, 0.0
        
        lookahead = min(self.lookahead_distance, goal_distance)
        linear_vel = min(self.max_linear_vel, goal_distance)
        angular_vel = 2.0 * np.sin(goal_direction) / lookahead
        angular_vel = max(-self.max_angular_vel, min(angular_vel, self.max_angular_vel))
        
        return linear_vel, angular_vel
    
    def check_collision(self, selected_direction: float) -> bool:
        """Check if there's an obstacle in the selected direction within safety threshold."""
        # Get the latest costmap and robot pose
        costmap = self.local_costmap()
        if costmap is None:
            return False  # No costmap available
            
        occupancy_grid, grid_info, grid_origin = ros_msg_to_numpy_grid(costmap)
        _, _, grid_resolution = grid_info
        grid_origin_x, grid_origin_y, _ = grid_origin
        
        [pos, rot] = self.robot.ros_control.transform_euler("base_link", "odom")
        robot_x, robot_y, robot_theta = pos[0], pos[1], rot[2]
        
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

    def update_visualization(self) -> np.ndarray:
        """Generate visualization of the planning state."""
        try:
            costmap = self.local_costmap()
            if costmap is None:
                raise ValueError("Costmap is None")
                
            [pos, rot] = self.robot.ros_control.transform_euler("base_link", "odom")
            robot_x, robot_y, robot_theta = pos[0], pos[1], rot[2]
            robot_pose = (robot_x, robot_y, robot_theta)
            
            occupancy_grid, grid_info, grid_origin = ros_msg_to_numpy_grid(costmap)
            if grid_info is None:
                 raise ValueError("Could not get grid info from costmap")
            _, _, grid_resolution = grid_info
            
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
                occupancy_grid=occupancy_grid,
                grid_resolution=grid_resolution,
                grid_origin=grid_origin,
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

def navigate_to_goal_local(
    robot, goal_xy_robot: Tuple[float, float], distance: float = 0.0, timeout: float = 60.0,
    stop_event: Optional[threading.Event] = None
) -> bool:
    """
    Navigates the robot to a goal specified in the robot's local frame
    using the local planner.

    Args:
        robot: Robot instance to control
        goal_xy_robot: Tuple (x, y) representing the goal position relative
                       to the robot's current position and orientation.
        distance: Desired distance to maintain from the goal in meters.
                 If non-zero, the robot will stop this far away from the goal.
        timeout: Maximum time (in seconds) allowed to reach the goal.
        stop_event: Optional threading.Event to signal when navigation should stop

    Returns:
        bool: True if the goal was reached within the timeout, False otherwise.
    """
    logger.info(f"Starting navigation to local goal {goal_xy_robot} with distance {distance}m and timeout {timeout}s.")

    goal_x, goal_y = goal_xy_robot
    
    # Calculate goal orientation to face the target
    goal_theta = np.arctan2(goal_y, goal_x)
    
    # If distance is non-zero, adjust the goal to stop at the desired distance
    if distance > 0:
        # Calculate magnitude of the goal vector
        goal_distance = np.sqrt(goal_x**2 + goal_y**2)
        
        # Only adjust if goal is further than the desired distance
        if goal_distance > distance:
            goal_x, goal_y = distance_angle_to_goal_xy(goal_distance - distance, goal_theta)
    
    # Set the goal in the robot's frame with orientation to face the original target
    robot.local_planner.set_goal((goal_x, goal_y), frame="base_link", goal_theta=-goal_theta)

    start_time = time.time()
    goal_reached = False

    try:
        while time.time() - start_time < timeout and not (stop_event and stop_event.is_set()):
            # Check if goal has been reached
            if robot.local_planner.is_goal_reached():
                logger.info("Goal reached successfully.")
                goal_reached = True
                break

            # Get planned velocity towards the goal
            vel_command = robot.local_planner.plan()
            x_vel = vel_command.get("x_vel", 0.0)
            angular_vel = vel_command.get("angular_vel", 0.0)

            # Send velocity command
            robot.ros_control.move_vel_control(x=x_vel, y=0, yaw=angular_vel)

            # Control loop frequency
            time.sleep(0.1)

        if not goal_reached:
            logger.warning(f"Navigation timed out after {timeout} seconds before reaching goal.")

    except KeyboardInterrupt:
        logger.info("Navigation to local goal interrupted by user.")
        goal_reached = False  # Consider interruption as failure
    except Exception as e:
        logger.error(f"Error during navigation to local goal: {e}")
        goal_reached = False  # Consider error as failure
    finally:
        logger.info("Stopping robot after navigation attempt.")
        robot.ros_control.stop()

    return goal_reached

def navigate_path_local(
    robot, path: Path, timeout: float = 120.0, goal_theta: Optional[float] = None, 
    stop_event: Optional[threading.Event] = None
) -> bool:
    """
    Navigates the robot along a path of waypoints using the waypoint following capability
    of the VFHPurePursuitPlanner.

    Args:
        robot: Robot instance to control
        path: Path object containing waypoints in odom/map frame
        timeout: Maximum time (in seconds) allowed to follow the complete path
        goal_theta: Optional final orientation in radians
        stop_event: Optional threading.Event to signal when navigation should stop

    Returns:
        bool: True if the entire path was successfully followed, False otherwise
    """
    logger.info(f"Starting navigation along path with {len(path)} waypoints and timeout {timeout}s.")

    # Set the path in the local planner
    robot.local_planner.set_goal_waypoints(path, goal_theta=goal_theta)

    start_time = time.time()
    path_completed = False

    try:
        while time.time() - start_time < timeout and not (stop_event and stop_event.is_set()):
            # Check if the entire path has been traversed
            if robot.local_planner.is_goal_reached():
                logger.info("Path traversed successfully.")
                path_completed = True
                break

            # Get planned velocity towards the current waypoint target
            vel_command = robot.local_planner.plan()
            x_vel = vel_command.get("x_vel", 0.0)
            angular_vel = vel_command.get("angular_vel", 0.0)

            # Send velocity command
            robot.ros_control.move_vel_control(x=x_vel, y=0, yaw=angular_vel)

            # Control loop frequency
            time.sleep(0.1)

        if not path_completed:
            logger.warning(f"Path following timed out after {timeout} seconds before completing the path.")

    except KeyboardInterrupt:
        logger.info("Path navigation interrupted by user.")
        path_completed = False
    except Exception as e:
        logger.error(f"Error during path navigation: {e}")
        path_completed = False
    finally:
        logger.info("Stopping robot after path navigation attempt.")
        robot.ros_control.stop()

    return path_completed
