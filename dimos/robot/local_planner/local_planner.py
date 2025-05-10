#!/usr/bin/env python3

import math
import numpy as np
from typing import Dict, Tuple, Optional, List, Any, Callable, Protocol
from abc import ABC, abstractmethod
import cv2
from reactivex import Observable
from reactivex.subject import Subject
import threading
import time
import logging
from collections import deque
from dimos.utils.logging_config import setup_logger
from dimos.utils.ros_utils import (
    normalize_angle,
    distance_angle_to_goal_xy
)

from dimos.robot.robot import Robot
from dimos.types.vector import VectorLike, Vector, to_tuple
from dimos.types.path import Path
from dimos.types.costmap import Costmap
from dimos.robot.global_planner.algo import astar
from nav_msgs.msg import OccupancyGrid

logger = setup_logger("dimos.robot.unitree.local_planner", level=logging.DEBUG)

class BaseLocalPlanner(ABC):
    """
    Abstract base class for local planners that handle obstacle avoidance and path following.
    
    This class defines the common interface and shared functionality that all local planners 
    must implement, regardless of the specific algorithm used.
    """
    
    def __init__(self, 
                 get_costmap: Callable[[], Optional[OccupancyGrid]],
                 transform: object,
                 move_vel_control: Callable[[float, float, float], None],
                 safety_threshold: float = 0.5,
                 max_linear_vel: float = 0.8,
                 max_angular_vel: float = 1.0,
                 lookahead_distance: float = 1.0,
                 goal_tolerance: float = 0.75,
                 angle_tolerance: float = 0.15,
                 robot_width: float = 0.5,
                 robot_length: float = 0.7,
                 visualization_size: int = 400,
                 control_frequency: float = 10.0,
                 safe_goal_distance: float = 1.5):  # Control frequency in Hz
        """
        Initialize the base local planner.
        
        Args:
            get_costmap: Function to get the latest local costmap
            transform: Object with transform methods (transform_point, transform_rot, etc.)
            move_vel_control: Function to send velocity commands
            safety_threshold: Distance to maintain from obstacles (meters)
            max_linear_vel: Maximum linear velocity (m/s)
            max_angular_vel: Maximum angular velocity (rad/s)
            lookahead_distance: Lookahead distance for path following (meters)
            goal_tolerance: Distance at which the goal is considered reached (meters)
            angle_tolerance: Angle at which the goal orientation is considered reached (radians)
            robot_width: Width of the robot for visualization (meters)
            robot_length: Length of the robot for visualization (meters)
            visualization_size: Size of the visualization image in pixels
            control_frequency: Frequency at which the planner is called (Hz)
            safe_goal_distance: Distance at which to adjust the goal and ignore obstacles (meters)
        """
        # Store callables for robot interactions
        self.get_costmap = get_costmap
        self.transform = transform
        self.move_vel_control = move_vel_control
        
        # Store parameters
        self.safety_threshold = safety_threshold
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.lookahead_distance = lookahead_distance
        self.goal_tolerance = goal_tolerance
        self.angle_tolerance = angle_tolerance
        self.robot_width = robot_width
        self.robot_length = robot_length
        self.visualization_size = visualization_size
        self.control_frequency = control_frequency
        self.control_period = 1.0 / control_frequency  # Period in seconds
        self.safe_goal_distance = safe_goal_distance  # Distance to ignore obstacles at goal
        self.ignore_obstacles = False  # Flag for derived classes to check

        # Goal and Waypoint Tracking
        self.goal_xy: Optional[Tuple[float, float]] = None  # Current target for planning
        self.goal_theta: Optional[float] = None  # Goal orientation in odom frame
        self.position_reached: bool = False  # Flag indicating if position goal is reached
        self.waypoints: Optional[Path] = None             # Full path if following waypoints
        self.waypoints_in_odom: Optional[Path] = None     # Full path in odom frame
        self.waypoint_frame: Optional[str] = None         # Frame of the waypoints
        self.current_waypoint_index: int = 0              # Index of the next waypoint to reach
        self.final_goal_reached: bool = False             # Flag indicating if the final waypoint is reached

        # Stuck detection
        self.stuck_detection_window_seconds = 8.0  # Time window for stuck detection (seconds)
        self.position_history_size = int(self.stuck_detection_window_seconds * control_frequency)
        self.position_history = deque(maxlen=self.position_history_size)  # History of recent positions
        self.stuck_distance_threshold = 0.1  # Distance threshold for stuck detection (meters)
        self.unstuck_distance_threshold = 0.5  # Distance threshold for unstuck detection (meters)
        self.stuck_time_threshold = 4.0  # Time threshold for stuck detection (seconds)
        self.is_recovery_active = False  # Whether recovery behavior is active
        self.recovery_start_time = 0.0  # When recovery behavior started
        self.recovery_duration = 8.0  # How long to run recovery before giving up (seconds)
        self.last_update_time = time.time()  # Last time position was updated
        self.navigation_failed = False  # Flag indicating if navigation should be terminated

    def reset(self):
        """
        Reset all navigation and state tracking variables.
        Should be called whenever a new goal is set.
        """
        # Reset stuck detection state
        self.position_history.clear()
        self.is_recovery_active = False
        self.recovery_start_time = 0.0
        self.last_update_time = time.time()
        
        # Reset navigation state flags
        self.navigation_failed = False
        self.position_reached = False
        self.final_goal_reached = False
        self.ignore_obstacles = False
        
        logger.info("Local planner state has been reset")

    def set_goal(self, goal_xy: VectorLike, frame: str = "odom", goal_theta: Optional[float] = None):
        """Set a single goal position, converting to odom frame if necessary.
           This clears any existing waypoints being followed.

        Args:
            goal_xy: The goal position to set.
            frame: The frame of the goal position.
            goal_theta: Optional goal orientation in radians (in the specified frame)
        """
        # Reset all state variables
        self.reset()
        
        # Clear waypoint following state
        self.waypoints = None
        self.current_waypoint_index = 0
        self.goal_xy = None # Clear previous goal
        self.goal_theta = None # Clear previous goal orientation

        target_goal_xy: Optional[Tuple[float, float]] = None

        target_goal_xy = self.transform.transform_point(goal_xy, source_frame=frame, target_frame="odom").to_tuple()

        logger.info(f"Goal set directly in odom frame: ({target_goal_xy[0]:.2f}, {target_goal_xy[1]:.2f})")
        
        # Check if goal is valid (in bounds and not colliding)
        if not self.is_goal_in_costmap_bounds(target_goal_xy) or self.check_goal_collision(target_goal_xy):
            logger.warning("Goal is in collision or out of bounds. Adjusting goal to valid position.")
            self.goal_xy = self.adjust_goal_to_valid_position(target_goal_xy)
        else:
            self.goal_xy = target_goal_xy # Set the adjusted or original valid goal
            
        # Set goal orientation if provided
        if goal_theta is not None:
            transformed_rot = self.transform.transform_rot(Vector(0.0, 0.0, goal_theta), source_frame=frame, target_frame="odom")
            self.goal_theta = transformed_rot[2]

    def set_goal_waypoints(self, waypoints: Path, frame: str = "map", goal_theta: Optional[float] = None):
        """Sets a path of waypoints for the robot to follow. 

        Args:
            waypoints: A list of waypoints to follow. Each waypoint is a tuple of (x, y) coordinates in odom frame.
            frame: The frame of the waypoints.
            goal_theta: Optional final orientation in radians (in the specified frame)
        """
        # Reset all state variables
        self.reset()

        if not isinstance(waypoints, Path) or len(waypoints) == 0:
            logger.warning("Invalid or empty path provided to set_goal_waypoints. Ignoring.")
            self.waypoints = None
            self.waypoint_frame = None
            self.goal_xy = None
            self.goal_theta = None
            self.current_waypoint_index = 0
            return

        logger.info(f"Setting goal waypoints with {len(waypoints)} points.")
        self.waypoints = waypoints
        self.waypoint_frame = frame
        self.current_waypoint_index = 0

        # Transform waypoints to odom frame
        self.waypoints_in_odom = self.transform.transform_path(self.waypoints, source_frame=frame, target_frame="odom")
        
        # Set the initial target to the first waypoint, adjusting if necessary
        first_waypoint = self.waypoints_in_odom[0]
        if not self.is_goal_in_costmap_bounds(first_waypoint) or self.check_goal_collision(first_waypoint):
            logger.warning("First waypoint is invalid. Adjusting...")
            self.goal_xy = self.adjust_goal_to_valid_position(first_waypoint)
        else:
            self.goal_xy = to_tuple(first_waypoint) # Initial target
            
        # Set goal orientation if provided
        if goal_theta is not None:
            transformed_rot = self.transform.transform_rot(Vector(0.0, 0.0, goal_theta), source_frame=frame, target_frame="odom")
            self.goal_theta = transformed_rot[2]

    def _get_robot_pose(self) -> Tuple[Tuple[float, float], float]:
        """
        Get the current robot position and orientation.
        
        Returns:
            Tuple containing:
            - position as (x, y) tuple
            - orientation (theta) in radians
        """
        [pos, rot] = self.transform.transform_euler("base_link", "odom")
        return (pos[0], pos[1]), rot[2]
        
    def _get_final_goal_position(self) -> Optional[Tuple[float, float]]:
        """
        Get the final goal position (either last waypoint or direct goal).
        
        Returns:
            Tuple (x, y) of the final goal, or None if no goal is set
        """
        if self.waypoints_in_odom is not None and len(self.waypoints_in_odom) > 0:
            return to_tuple(self.waypoints_in_odom[-1])
        elif self.goal_xy is not None:
            return self.goal_xy
        return None
        
    def _distance_to_position(self, target_position: Tuple[float, float]) -> float:
        """
        Calculate distance from the robot to a target position.
        
        Args:
            target_position: Target (x, y) position
            
        Returns:
            Distance in meters
        """
        robot_pos, _ = self._get_robot_pose()
        return np.linalg.norm([target_position[0] - robot_pos[0], 
                              target_position[1] - robot_pos[1]])
        
    def plan(self) -> Dict[str, float]:
        """
        Main planning method that computes velocity commands.
        This includes common planning logic like waypoint following,
        with algorithm-specific calculations delegated to subclasses.
        
        Returns:
            Dict[str, float]: Velocity commands with 'x_vel' and 'angular_vel' keys
        """
        # If goal orientation is specified, rotate to match it
        if self.position_reached and self.goal_theta is not None and not self._is_goal_orientation_reached():
            logger.info("Position goal reached. Rotating to target orientation.")
            return self._rotate_to_goal_orientation()

        # Check if the robot is stuck and handle accordingly
        if self.check_if_stuck() and not self.position_reached:
            # Check if we're stuck but close to our goal
            final_goal_pos = self._get_final_goal_position()
                
            # If we have a goal position, check distance to it
            if final_goal_pos is not None:
                distance_to_goal = self._distance_to_position(final_goal_pos)
                
                # If we're stuck but within 2x safe_goal_distance of the goal, consider it a success
                if distance_to_goal < 2.0 * self.safe_goal_distance:
                    logger.info(f"Robot is stuck but within {distance_to_goal:.2f}m of goal (< {2.0 * self.safe_goal_distance:.2f}m). Considering navigation successful.")
                    self.position_reached = True
                    return {'x_vel': 0.0, 'angular_vel': 0.0}
            
            # Otherwise, execute normal recovery behavior            
            logger.warning("Robot is stuck - executing recovery behavior")
            return self.execute_recovery_behavior()
        
        # Reset obstacle ignore flag
        self.ignore_obstacles = False
        
        # --- Waypoint Following Mode --- 
        if self.waypoints is not None:
            if self.final_goal_reached:
                logger.info("Final waypoint reached. Stopping.")
                return {'x_vel': 0.0, 'angular_vel': 0.0}
            
            # Get current robot pose
            robot_pos, robot_theta = self._get_robot_pose()
            robot_pos_np = np.array(robot_pos)
            
            # Check if close to final waypoint
            if self.waypoints_in_odom is not None and len(self.waypoints_in_odom) > 0:
                final_waypoint = self.waypoints_in_odom[-1]
                dist_to_final = np.linalg.norm(robot_pos_np - final_waypoint)
                
                # If we're close to the final waypoint, adjust it and ignore obstacles
                if dist_to_final < self.safe_goal_distance:
                    final_wp_tuple = to_tuple(final_waypoint)
                    adjusted_goal = self.adjust_goal_to_valid_position(final_wp_tuple)
                    # Create a new Path with the adjusted final waypoint
                    new_waypoints = self.waypoints_in_odom[:-1]  # Get all but the last waypoint
                    new_waypoints.append(adjusted_goal)  # Append the adjusted goal
                    self.waypoints_in_odom = new_waypoints
                    self.ignore_obstacles = True
                    logger.debug(f"Within safe distance of final waypoint. Ignoring obstacles.")
            
            # Update the target goal based on waypoint progression
            just_reached_final = self._update_waypoint_target(robot_pos_np)
            
            # If the helper indicates the final goal was just reached, stop immediately
            if just_reached_final:
                 return {'x_vel': 0.0, 'angular_vel': 0.0}

        # --- Single Goal or Current Waypoint Target Set --- 
        if self.goal_xy is None:
            # If no goal is set (e.g., empty path or rejected goal), stop.
            return {'x_vel': 0.0, 'angular_vel': 0.0}

        # Get necessary data for planning
        costmap = self.get_costmap()
        if costmap is None:
            logger.warning("Local costmap is None. Cannot plan.")
            return {'x_vel': 0.0, 'angular_vel': 0.0}
        
        # Check if close to single goal mode goal
        if self.waypoints is None:
            # Get distance to goal
            goal_distance = self._distance_to_position(self.goal_xy)
            
            # If within safe distance of goal, adjust it and ignore obstacles
            if goal_distance < self.safe_goal_distance:
                self.goal_xy = self.adjust_goal_to_valid_position(self.goal_xy)
                self.ignore_obstacles = True
                logger.debug(f"Within safe distance of goal. Ignoring obstacles.")
            
            # First check position
            if goal_distance < self.goal_tolerance or self.position_reached:
                self.position_reached = True

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
        _, robot_theta = self._get_robot_pose()
        
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

    def _is_goal_orientation_reached(self) -> bool:
        """Check if the current robot orientation matches the goal orientation.
        
        Returns:
            bool: True if orientation is reached or no orientation goal is set
        """
        if self.goal_theta is None:
            return True  # No orientation goal set
            
        # Get current robot orientation
        _, robot_theta = self._get_robot_pose()
        
        # Calculate the angle difference and normalize
        angle_diff = abs(normalize_angle(self.goal_theta - robot_theta))
        
        logger.debug(f"Orientation error: {angle_diff:.4f} rad, tolerance: {self.angle_tolerance:.4f} rad")
        return angle_diff <= self.angle_tolerance

    def _update_waypoint_target(self, robot_pos_np: np.ndarray) -> bool:
        """Helper function to manage waypoint progression and update the target goal.
        
        Args:
            robot_pos_np: Current robot position as a numpy array [x, y].
            
        Returns:
            bool: True if the final waypoint has just been reached, False otherwise.
        """
        if self.waypoints is None or len(self.waypoints) == 0:
            return False  # Not in waypoint mode or empty path
        
        self.waypoints_in_odom = self.transform.transform_path(self.waypoints, source_frame=self.waypoint_frame, target_frame="odom")

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

    @abstractmethod
    def update_visualization(self) -> np.ndarray:
        """
        Generate visualization of the planning state.
        Must be implemented by derived classes.
        
        Returns:
            np.ndarray: Visualization image as numpy array
        """
        pass

    def create_stream(self, frequency_hz: float = None) -> Observable:
        """
        Create an Observable stream that emits the visualization image at a fixed frequency.
        
        Args:
            frequency_hz: Optional frequency override (defaults to 1/4 of control_frequency if None)
            
        Returns:
            Observable: Stream of visualization frames
        """
        # Default to 1/4 of control frequency if not specified (to reduce CPU usage)
        if frequency_hz is None:
            frequency_hz = self.control_frequency / 4.0
            
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
        logger.info(f"Started visualization frame emitter thread at {frequency_hz:.1f} Hz")
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
            
            return self.position_reached and self._is_goal_orientation_reached()

    def check_goal_collision(self, goal_xy: VectorLike) -> bool:
        """Check if the current goal is in collision with obstacles in the costmap.
        
        Returns:
            bool: True if goal is in collision, False if goal is safe or cannot be checked
        """
            
        costmap = self.get_costmap()
        if costmap is None:
            logger.warning("Cannot check collision: No costmap available")
            return False
            
        # Check if the position is occupied
        collision_threshold = 80  # Consider values above 80 as obstacles
        
        # Use Costmap's is_occupied method
        return costmap.is_occupied(goal_xy, threshold=collision_threshold)

    def is_goal_in_costmap_bounds(self, goal_xy: VectorLike) -> bool:
        """Check if the goal position is within the bounds of the costmap.
        
        Args:
            goal_xy: Goal position (x, y) in odom frame
            
        Returns:
            bool: True if the goal is within the costmap bounds, False otherwise
        """
        costmap = self.get_costmap()
        if costmap is None:
            logger.warning("Cannot check bounds: No costmap available")
            return False
            
        # Get goal position in grid coordinates
        goal_point = costmap.world_to_grid(goal_xy)
        goal_cell_x, goal_cell_y = goal_point.x, goal_point.y
        
        # Check if goal is within the costmap bounds
        is_in_bounds = 0 <= goal_cell_x < costmap.width and 0 <= goal_cell_y < costmap.height
        
        if not is_in_bounds:
            logger.warning(f"Goal ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}) is outside costmap bounds")
            
        return is_in_bounds

    def adjust_goal_to_valid_position(self, goal_xy: VectorLike, clearance: float = 0.5) -> Tuple[float, float]:
        """Find a valid (non-colliding) goal position by moving it towards the robot.
        
        Args:
            goal_xy: Original goal position (x, y) in odom frame
            clearance: Additional distance to move back from obstacles for better clearance (meters)
        
        Returns:
            Tuple[float, float]: A valid goal position, or the original goal if already valid
        """    
        [pos, rot] = self.transform.transform_euler("base_link", "odom")
        
        robot_x, robot_y = pos[0], pos[1]
        
        # Original goal
        goal_x, goal_y = to_tuple(goal_xy)

        if not self.check_goal_collision((goal_x, goal_y)):
            return (goal_x, goal_y)

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
        
        # Variables to store the first valid position found
        valid_found = False
        valid_x, valid_y = None, None
        
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
                # Store the first valid position
                if not valid_found:
                    valid_found = True
                    valid_x, valid_y = current_x, current_y
                    
                    # If clearance is requested, continue searching for a better position
                    if clearance > 0:
                        continue
                
                # Calculate position with additional clearance
                if clearance > 0:
                    
                    # Calculate clearance position
                    clearance_x = current_x + dx * clearance
                    clearance_y = current_y + dy * clearance

                    logger.info(f"Checking clearance position at ({clearance_x:.2f}, {clearance_y:.2f})")
                    
                    # Check if the clearance position is also valid
                    if (not self.check_goal_collision((clearance_x, clearance_y)) and 
                        self.is_goal_in_costmap_bounds((clearance_x, clearance_y))):
                        logger.info(f"Found valid goal with clearance at ({clearance_x:.2f}, {clearance_y:.2f})")
                        return (clearance_x, clearance_y)
                
                # Return the valid position without clearance
                logger.info(f"Found valid goal at ({current_x:.2f}, {current_y:.2f})")
                return (current_x, current_y)
        
        # If we found a valid position earlier but couldn't add clearance
        if valid_found:
            logger.info(f"Using valid goal found at ({valid_x:.2f}, {valid_y:.2f})")
            return (valid_x, valid_y)
                
        logger.warning(f"Could not find valid goal after {steps} steps, using closest point to robot")
        return (current_x, current_y)

    def check_if_stuck(self) -> bool:
        """
        Check if the robot is stuck by analyzing movement history.
        
        Returns:
            bool: True if the robot is determined to be stuck, False otherwise
        """
        # Get current position and time
        current_time = time.time()
        
        # Get current robot position
        [pos, _] = self.transform.transform_euler("base_link", "odom")
        current_position = (pos[0], pos[1], current_time)
        
        # Add current position to history (newest is appended at the end)
        self.position_history.append(current_position)
        
        # Need enough history to make a determination
        min_history_size = self.stuck_detection_window_seconds * self.control_frequency
        if len(self.position_history) < min_history_size:
            return False
            
        # Find positions within our detection window (positions are already in order from oldest to newest)
        window_start_time = current_time - self.stuck_detection_window_seconds
        window_positions = []
        
        # Collect positions within the window (newest entries will be at the end)
        for pos_x, pos_y, timestamp in self.position_history:
            if timestamp >= window_start_time:
                window_positions.append((pos_x, pos_y, timestamp))
                
        # Need at least a few positions in the window
        if len(window_positions) < 3:
            return False
            
        # Ensure correct order: oldest to newest
        window_positions.sort(key=lambda p: p[2])
            
        # Get the oldest and newest positions in the window
        oldest_x, oldest_y, oldest_time = window_positions[0]
        newest_x, newest_y, newest_time = window_positions[-1]
        
        # Calculate time range in the window (should always be positive)
        time_range = newest_time - oldest_time
        
        # Calculate displacement from oldest to newest position
        displacement = np.sqrt((newest_x - oldest_x)**2 + (newest_y - oldest_y)**2)
        
        # Check if we're stuck - moved less than threshold over minimum time
        # Only consider it if the time range makes sense (positive and sufficient)
        is_currently_stuck = (time_range >= self.stuck_time_threshold and 
                             time_range <= self.stuck_detection_window_seconds and 
                             displacement < self.stuck_distance_threshold)
        
        if is_currently_stuck:
            logger.warning(f"Robot appears to be stuck! Displacement {displacement:.3f}m over {time_range:.1f}s")
            
            # Don't trigger recovery if it's already active
            if not self.is_recovery_active:
                self.is_recovery_active = True
                self.recovery_start_time = current_time
                return True
                
            # Check if we've been trying to recover for too long
            elif current_time - self.recovery_start_time > self.recovery_duration:
                logger.error(f"Recovery behavior has been active for {self.recovery_duration}s without success")
                # Reset recovery state - maybe a different behavior will work
                self.is_recovery_active = False
                self.recovery_start_time = current_time
                
        # If we've moved enough, we're not stuck anymore
        elif self.is_recovery_active and displacement > self.unstuck_distance_threshold:
            logger.info(f"Robot has escaped from stuck state (moved {displacement:.3f}m)")
            self.is_recovery_active = False
            
        return self.is_recovery_active
        
    def execute_recovery_behavior(self) -> Dict[str, float]:
        """
        Execute a recovery behavior when the robot is stuck.
        
        Returns:
            Dict[str, float]: Velocity commands for the recovery behavior
        """
        # Calculate how long we've been in recovery
        recovery_time = time.time() - self.recovery_start_time
        
        # Calculate recovery phases based on control frequency
        backup_phase_time = 3.0  # seconds
        rotate_phase_time = 2.0  # seconds
        
        # Simple recovery behavior state machine
        if recovery_time < backup_phase_time:
            # First try backing up
            logger.info("Recovery: backing up")
            return {'x_vel': -0.2, 'angular_vel': 0.0}
        elif recovery_time < backup_phase_time + rotate_phase_time:
            # Then try rotating
            logger.info("Recovery: rotating to find new path")
            rotation_direction = 1.0 if np.random.random() > 0.5 else -1.0
            return {'x_vel': 0.0, 'angular_vel': rotation_direction * self.max_angular_vel * 0.7}
        else:
            # If we're still stuck after backup and rotation, terminate navigation
            logger.error("Recovery failed after backup and rotation. Navigation terminated.")
            # Set a flag to indicate navigation should terminate
            self.navigation_failed = True
            # Stop the robot
            return {'x_vel': 0.0, 'angular_vel': 0.0}

def navigate_to_goal_local(
    robot, goal_xy_robot: Tuple[float, float], goal_theta: Optional[float] = None, distance: float = 0.0, timeout: float = 60.0,
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
    if goal_theta is None:
        goal_theta = np.arctan2(goal_y, goal_x)
    
    # If distance is non-zero, adjust the goal to stop at the desired distance
    if distance > 0:
        # Calculate magnitude of the goal vector
        goal_distance = np.sqrt(goal_x**2 + goal_y**2)
        
        # Only adjust if goal is further than the desired distance
        if goal_distance > distance:
            goal_x, goal_y = distance_angle_to_goal_xy(goal_distance - distance, goal_theta)
    
    # Set the goal in the robot's frame with orientation to face the original target
    robot.local_planner.set_goal((goal_x, goal_y), frame="base_link", goal_theta=goal_theta)
    
    # Get control period from robot's local planner for consistent timing
    control_period = 1.0 / robot.local_planner.control_frequency
    
    start_time = time.time()
    goal_reached = False

    try:
        while time.time() - start_time < timeout and not (stop_event and stop_event.is_set()):
            # Check if goal has been reached
            if robot.local_planner.is_goal_reached():
                logger.info("Goal reached successfully.")
                goal_reached = True
                break
                
            # Check if navigation failed flag is set
            if robot.local_planner.navigation_failed:
                logger.error("Navigation aborted due to repeated recovery failures.")
                goal_reached = False
                break

            # Get planned velocity towards the goal
            vel_command = robot.local_planner.plan()
            x_vel = vel_command.get("x_vel", 0.0)
            angular_vel = vel_command.get("angular_vel", 0.0)

            # Send velocity command
            robot.local_planner.move_vel_control(x=x_vel, y=0, yaw=angular_vel)

            # Control loop frequency - use robot's control frequency
            time.sleep(control_period)

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
        robot.local_planner.move_vel_control(0, 0, 0)  # Stop the robot

    return goal_reached

def navigate_path_local(
    robot, path: Path, timeout: float = 120.0, goal_theta: Optional[float] = None, 
    stop_event: Optional[threading.Event] = None
) -> bool:
    """
    Navigates the robot along a path of waypoints using the waypoint following capability
    of the local planner.

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
    
    # Get control period from robot's local planner for consistent timing
    control_period = 1.0 / robot.local_planner.control_frequency
    
    start_time = time.time()
    path_completed = False

    try:
        while time.time() - start_time < timeout and not (stop_event and stop_event.is_set()):
            # Check if the entire path has been traversed
            if robot.local_planner.is_goal_reached():
                logger.info("Path traversed successfully.")
                path_completed = True
                break
                
            # Check if navigation failed flag is set
            if robot.local_planner.navigation_failed:
                logger.error("Navigation aborted due to repeated recovery failures.")
                path_completed = False
                break

            # Get planned velocity towards the current waypoint target
            vel_command = robot.local_planner.plan()
            x_vel = vel_command.get("x_vel", 0.0)
            angular_vel = vel_command.get("angular_vel", 0.0)

            # Send velocity command
            robot.local_planner.move_vel_control(x=x_vel, y=0, yaw=angular_vel)

            # Control loop frequency - use robot's control frequency
            time.sleep(control_period)

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
        robot.local_planner.move_vel_control(0, 0, 0)  # Stop the robot

    return path_completed

def visualize_local_planner_state(
    occupancy_grid: np.ndarray, 
    grid_resolution: float, 
    grid_origin: Tuple[float, float, float], 
    robot_pose: Tuple[float, float, float], 
    visualization_size: int = 400, 
    robot_width: float = 0.5, 
    robot_length: float = 0.7,
    map_size_meters: float = 10.0,
    goal_xy: Optional[Tuple[float, float]] = None, 
    goal_theta: Optional[float] = None,
    histogram: Optional[np.ndarray] = None,
    selected_direction: Optional[float] = None,
    waypoints: Optional['Path'] = None,
    current_waypoint_index: Optional[int] = None
) -> np.ndarray:
    """Generate a bird's eye view visualization of the local costmap.
    Optionally includes VFH histogram, selected direction, and waypoints path.
    
    Args:
        occupancy_grid: 2D numpy array of the occupancy grid
        grid_resolution: Resolution of the grid in meters/cell
        grid_origin: Tuple (x, y, theta) of the grid origin in the odom frame
        robot_pose: Tuple (x, y, theta) of the robot pose in the odom frame
        visualization_size: Size of the visualization image in pixels
        robot_width: Width of the robot in meters
        robot_length: Length of the robot in meters
        map_size_meters: Size of the map to visualize in meters
        goal_xy: Optional tuple (x, y) of the goal position in the odom frame
        goal_theta: Optional goal orientation in radians (in odom frame)
        histogram: Optional numpy array of the VFH histogram
        selected_direction: Optional selected direction angle in radians
        waypoints: Optional Path object containing waypoints to visualize
        current_waypoint_index: Optional index of the current target waypoint
    """
    
    robot_x, robot_y, robot_theta = robot_pose
    grid_origin_x, grid_origin_y, _ = grid_origin
    vis_size = visualization_size
    scale = vis_size / map_size_meters
    
    vis_img = np.ones((vis_size, vis_size, 3), dtype=np.uint8) * 255
    center_x = vis_size // 2
    center_y = vis_size // 2
    
    grid_height, grid_width = occupancy_grid.shape
    
    # Calculate robot position relative to grid origin
    robot_rel_x = robot_x - grid_origin_x
    robot_rel_y = robot_y - grid_origin_y
    robot_cell_x = int(robot_rel_x / grid_resolution)
    robot_cell_y = int(robot_rel_y / grid_resolution)
    
    half_size_cells = int(map_size_meters / grid_resolution / 2)

    # Draw grid cells (using standard occupancy coloring)
    for y in range(max(0, robot_cell_y - half_size_cells),
                   min(grid_height, robot_cell_y + half_size_cells)):
        for x in range(max(0, robot_cell_x - half_size_cells),
                       min(grid_width, robot_cell_x + half_size_cells)):
            cell_rel_x_meters = (x - robot_cell_x) * grid_resolution
            cell_rel_y_meters = (y - robot_cell_y) * grid_resolution
            
            img_x = int(center_x + cell_rel_x_meters * scale)
            img_y = int(center_y - cell_rel_y_meters * scale)  # Flip y-axis

            if 0 <= img_x < vis_size and 0 <= img_y < vis_size:
                cell_value = occupancy_grid[y, x]
                if cell_value == -1:
                    color = (200, 200, 200)  # Unknown (Light gray)
                elif cell_value == 0:
                    color = (255, 255, 255)  # Free (White)
                else:  # Occupied
                    # Scale darkness based on occupancy value (0-100)
                    darkness = 255 - int(155 * (cell_value / 100)) - 100
                    color = (darkness, darkness, darkness)  # Shades of gray/black
                
                cell_size_px = max(1, int(grid_resolution * scale))
                cv2.rectangle(vis_img, 
                              (img_x - cell_size_px//2, img_y - cell_size_px//2),
                              (img_x + cell_size_px//2, img_y + cell_size_px//2),
                              color, -1)

    # Draw waypoints path if provided
    if waypoints is not None and len(waypoints) > 0:
        try:
            path_points = []
            for i, waypoint in enumerate(waypoints):
                # Convert waypoint from odom frame to visualization frame
                wp_x, wp_y = waypoint[0], waypoint[1]
                wp_rel_x = wp_x - robot_x
                wp_rel_y = wp_y - robot_y
                
                wp_img_x = int(center_x + wp_rel_x * scale)
                wp_img_y = int(center_y - wp_rel_y * scale)  # Flip y-axis
                
                if 0 <= wp_img_x < vis_size and 0 <= wp_img_y < vis_size:
                    path_points.append((wp_img_x, wp_img_y))
                    
                    # Draw each waypoint as a small circle
                    cv2.circle(vis_img, (wp_img_x, wp_img_y), 3, (0, 128, 0), -1)  # Dark green dots
                    
                    # Highlight current target waypoint
                    if current_waypoint_index is not None and i == current_waypoint_index:
                        cv2.circle(vis_img, (wp_img_x, wp_img_y), 6, (0, 0, 255), 2)  # Red circle
            
            # Connect waypoints with lines to show the path
            if len(path_points) > 1:
                for i in range(len(path_points) - 1):
                    cv2.line(vis_img, path_points[i], path_points[i + 1], (0, 200, 0), 1)  # Green line
        except Exception as e:
            logger.error(f"Error drawing waypoints: {e}")

    # Draw histogram
    if histogram is not None:
        num_bins = len(histogram)
        # Find absolute maximum value (ignoring any negative debug values)
        abs_histogram = np.abs(histogram)
        max_hist_value = np.max(abs_histogram) if np.max(abs_histogram) > 0 else 1.0
        hist_scale = (vis_size / 2) * 0.8 # Scale histogram lines to 80% of half the viz size
        
        for i in range(num_bins):
            # Angle relative to robot's forward direction
            angle_relative_to_robot = (i / num_bins) * 2 * math.pi - math.pi
            # Angle in the visualization frame (relative to image +X axis)
            vis_angle = angle_relative_to_robot + robot_theta 
            
            # Get the value and check if it's a special debug value (negative)
            hist_val = histogram[i]
            is_debug_value = hist_val < 0
            
            # Use absolute value for line length
            normalized_val = min(1.0, abs(hist_val) / max_hist_value)
            line_length = normalized_val * hist_scale
            
            # Calculate endpoint using the visualization angle
            end_x = int(center_x + line_length * math.cos(vis_angle))
            end_y = int(center_y - line_length * math.sin(vis_angle)) # Flipped Y
            
            # Color based on value and whether it's a debug value
            if is_debug_value:
                # Use green for debug values (minimum cost bin)
                color = (0, 255, 0)  # Green
                line_width = 2  # Thicker line for emphasis
            else:
                # Regular coloring for normal values (blue to red gradient based on obstacle density)
                blue = max(0, 255 - int(normalized_val * 255))
                red = min(255, int(normalized_val * 255))
                color = (blue, 0, red)  # BGR format: obstacles are redder, clear areas are bluer
                line_width = 1
            
            cv2.line(vis_img, (center_x, center_y), (end_x, end_y), color, line_width)

    # Draw robot
    robot_length_px = int(robot_length * scale)
    robot_width_px = int(robot_width * scale)
    robot_pts = np.array([
        [-robot_length_px/2, -robot_width_px/2], [robot_length_px/2, -robot_width_px/2],
        [robot_length_px/2, robot_width_px/2], [-robot_length_px/2, robot_width_px/2]
    ], dtype=np.float32)
    rotation_matrix = np.array([
        [math.cos(robot_theta), -math.sin(robot_theta)],
        [math.sin(robot_theta), math.cos(robot_theta)]
    ])
    robot_pts = np.dot(robot_pts, rotation_matrix.T)
    robot_pts[:, 0] += center_x
    robot_pts[:, 1] = center_y - robot_pts[:, 1]  # Flip y-axis
    cv2.fillPoly(vis_img, [robot_pts.reshape((-1, 1, 2)).astype(np.int32)], (0, 0, 255))  # Red robot

    # Draw robot direction line
    front_x = int(center_x + (robot_length_px/2) * math.cos(robot_theta))
    front_y = int(center_y - (robot_length_px/2) * math.sin(robot_theta))
    cv2.line(vis_img, (center_x, center_y), (front_x, front_y), (255, 0, 0), 2)  # Blue line

    # Draw selected direction
    if selected_direction is not None:
        # selected_direction is relative to robot frame
        # Angle in the visualization frame (relative to image +X axis)
        vis_angle_selected = selected_direction + robot_theta

        # Make slightly longer than max histogram line
        sel_dir_line_length = (vis_size / 2) * 0.9 

        sel_end_x = int(center_x + sel_dir_line_length * math.cos(vis_angle_selected))
        sel_end_y = int(center_y - sel_dir_line_length * math.sin(vis_angle_selected)) # Flipped Y
        
        cv2.line(vis_img, (center_x, center_y), (sel_end_x, sel_end_y), (0, 165, 255), 2) # BGR for Orange

    # Draw goal
    if goal_xy is not None:
        goal_x, goal_y = goal_xy
        goal_rel_x_map = goal_x - robot_x
        goal_rel_y_map = goal_y - robot_y
        goal_img_x = int(center_x + goal_rel_x_map * scale)
        goal_img_y = int(center_y - goal_rel_y_map * scale)  # Flip y-axis
        if 0 <= goal_img_x < vis_size and 0 <= goal_img_y < vis_size:
            cv2.circle(vis_img, (goal_img_x, goal_img_y), 5, (0, 255, 0), -1)  # Green circle
            cv2.circle(vis_img, (goal_img_x, goal_img_y), 8, (0, 0, 0), 1)      # Black outline

    # Draw goal orientation
    if goal_theta is not None and goal_xy is not None:
        # For waypoint mode, only draw orientation at the final waypoint
        if waypoints is not None and len(waypoints) > 0:
            # Use the final waypoint position
            final_waypoint = waypoints[-1]
            goal_x, goal_y = final_waypoint[0], final_waypoint[1]
        else:
            # Use the current goal position
            goal_x, goal_y = goal_xy
            
        goal_rel_x_map = goal_x - robot_x
        goal_rel_y_map = goal_y - robot_y
        goal_img_x = int(center_x + goal_rel_x_map * scale)
        goal_img_y = int(center_y - goal_rel_y_map * scale)  # Flip y-axis
        
        # Calculate goal orientation vector direction in visualization frame
        # goal_theta is already in odom frame, need to adjust for visualization orientation
        goal_dir_length = 30  # Length of direction indicator in pixels
        goal_dir_end_x = int(goal_img_x + goal_dir_length * math.cos(goal_theta))
        goal_dir_end_y = int(goal_img_y - goal_dir_length * math.sin(goal_theta))  # Flip y-axis
        
        # Draw goal orientation arrow
        if 0 <= goal_img_x < vis_size and 0 <= goal_img_y < vis_size:
            cv2.arrowedLine(vis_img, (goal_img_x, goal_img_y), (goal_dir_end_x, goal_dir_end_y), 
                         (255, 0, 255), 4)  # Magenta arrow

    # Add scale bar
    scale_bar_length_px = int(1.0 * scale)
    scale_bar_x = vis_size - scale_bar_length_px - 10
    scale_bar_y = vis_size - 20
    cv2.line(vis_img, (scale_bar_x, scale_bar_y), 
             (scale_bar_x + scale_bar_length_px, scale_bar_y), (0, 0, 0), 2)
    cv2.putText(vis_img, "1m", (scale_bar_x, scale_bar_y - 5), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
                
    # Add status info
    status_text = []
    if waypoints is not None:
        if current_waypoint_index is not None:
            status_text.append(f"WP: {current_waypoint_index}/{len(waypoints)}")
        else:
            status_text.append(f"WPs: {len(waypoints)}")
    
    y_pos = 20
    for text in status_text:
        cv2.putText(vis_img, text, (10, y_pos), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        y_pos += 20

    return vis_img