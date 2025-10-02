#!/usr/bin/env python3

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
Base Local Planner Module for robot navigation.
Subscribes to local costmap, odometry, and path, publishes movement commands.
"""

import threading
import time
from abc import abstractmethod
from typing import Optional

from dimos.core import Module, In, Out, rpc
from dimos.msgs.geometry_msgs import Twist, PoseStamped
from dimos.msgs.nav_msgs import OccupancyGrid, Path
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import get_distance, quaternion_to_euler, normalize_angle

logger = setup_logger("dimos.robot.local_planner")


class BaseLocalPlanner(Module):
    """
    local planner module for obstacle avoidance and path following.

    Subscribes to:
        - /local_costmap: Local occupancy grid for obstacle detection
        - /odom: Robot odometry for current pose
        - /path: Path to follow (continuously updated at ~1Hz)

    Publishes:
        - /cmd_vel: Velocity commands for robot movement
    """

    # LCM inputs
    local_costmap: In[OccupancyGrid] = None
    odom: In[PoseStamped] = None
    path: In[Path] = None

    # LCM outputs
    cmd_vel: Out[Twist] = None

    def __init__(
        self,
        goal_tolerance: float = 0.5,
        orientation_tolerance: float = 0.2,
        control_frequency: float = 10.0,
        **kwargs,
    ):
        """Initialize the local planner module.

        Args:
            goal_tolerance: Distance threshold to consider goal reached (meters)
            orientation_tolerance: Orientation threshold to consider goal reached (radians)
            control_frequency: Frequency for control loop (Hz)
        """
        super().__init__(**kwargs)

        # Parameters
        self.goal_tolerance = goal_tolerance
        self.orientation_tolerance = orientation_tolerance
        self.control_frequency = control_frequency
        self.control_period = 1.0 / control_frequency

        # Latest data
        self.latest_costmap: Optional[OccupancyGrid] = None
        self.latest_odom: Optional[PoseStamped] = None
        self.latest_path: Optional[Path] = None

        # Control thread
        self.planning_thread: Optional[threading.Thread] = None
        self.stop_planning = threading.Event()

        logger.info("Local planner module initialized")

    @rpc
    def start(self):
        """Start the local planner module."""
        # Subscribe to inputs
        self.local_costmap.subscribe(self._on_costmap)
        self.odom.subscribe(self._on_odom)
        self.path.subscribe(self._on_path)

        logger.info("Local planner module started")

    def _on_costmap(self, msg: OccupancyGrid):
        self.latest_costmap = msg

    def _on_odom(self, msg: PoseStamped):
        self.latest_odom = msg

    def _on_path(self, msg: Path):
        self.latest_path = msg

        if msg and len(msg.poses) > 0:
            if self.planning_thread is None or not self.planning_thread.is_alive():
                self._start_planning_thread()

    def _start_planning_thread(self):
        """Start the planning thread."""
        self.stop_planning.clear()
        self.planning_thread = threading.Thread(target=self._follow_path_loop, daemon=True)
        self.planning_thread.start()
        logger.debug("Started follow path thread")

    def _follow_path_loop(self):
        """Main planning loop that runs in a separate thread."""
        while not self.stop_planning.is_set():
            if self.is_goal_reached():
                self.stop_planning.set()
                stop_cmd = Twist()
                self.cmd_vel.publish(stop_cmd)
                break

            # Compute and publish velocity
            self._plan()

            time.sleep(self.control_period)

    def _plan(self):
        """Compute and publish velocity command."""
        cmd_vel = self.compute_velocity()

        if cmd_vel is not None:
            self.cmd_vel.publish(cmd_vel)

    @abstractmethod
    def compute_velocity(self) -> Optional[Twist]:
        """
        Compute velocity commands based on current costmap, odometry, and path.
        Must be implemented by derived classes.

        Returns:
            Twist message with linear and angular velocity commands, or None if no command
        """
        pass

    @rpc
    def is_goal_reached(self) -> bool:
        """
        Check if the robot has reached the goal position and orientation.

        Returns:
            True if goal is reached within tolerance, False otherwise
        """
        if self.latest_odom is None or self.latest_path is None:
            return False

        if len(self.latest_path.poses) == 0:
            return True

        goal_pose = self.latest_path.poses[-1]
        distance = get_distance(self.latest_odom, goal_pose)

        # Check distance tolerance
        if distance >= self.goal_tolerance:
            return False

        # Check orientation tolerance
        current_euler = quaternion_to_euler(self.latest_odom.orientation)
        goal_euler = quaternion_to_euler(goal_pose.orientation)

        # Calculate yaw difference and normalize to [-pi, pi]
        yaw_error = normalize_angle(goal_euler.z - current_euler.z)

        return abs(yaw_error) < self.orientation_tolerance

    @rpc
    def reset(self):
        """Reset the local planner state, clearing the current path."""
        # Clear the latest path
        self.latest_path = None
        self.latest_odom = None
        self.latest_costmap = None
        self.stop()
        logger.info("Local planner reset")

    @rpc
    def stop(self):
        """Stop the local planner and any running threads."""
        if self.planning_thread and self.planning_thread.is_alive():
            self.stop_planning.set()
            self.planning_thread.join(timeout=1.0)
            self.planning_thread = None
        stop_cmd = Twist()
        self.cmd_vel.publish(stop_cmd)

        logger.info("Local planner stopped")

    @rpc
    def stop_planner_module(self):
        self.stop()
        self._close_module()
