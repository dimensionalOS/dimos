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

from abc import abstractmethod
import threading
import time

from reactivex.disposable import Disposable

from dimos.core import In, Module, Out, rpc
from dimos.mapping.occupancy.gradient import gradient
from dimos.mapping.pointclouds.occupancy import general_occupancy
from dimos.msgs.geometry_msgs import PoseStamped, Twist
from dimos.msgs.nav_msgs import OccupancyGrid, Path
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import get_distance, normalize_angle, quaternion_to_euler

logger = setup_logger()


class BaseLocalPlanner(Module):
    """
    local planner module for obstacle avoidance and path following.

    Subscribes to:
        - /lidar: Local lidar for obstacle detection
        - /odom: Robot odometry for current pose
        - /path: Path to follow (continuously updated at ~1Hz)

    Publishes:
        - /cmd_vel: Velocity commands for robot movement
    """

    # LCM inputs
    lidar: In[LidarMessage]
    odom: In[PoseStamped]
    path: In[Path]

    # LCM outputs
    cmd_vel: Out[Twist]

    def __init__(  # type: ignore[no-untyped-def]
        self,
        goal_tolerance: float = 0.5,
        orientation_tolerance: float = 0.2,
        control_frequency: float = 10.0,
        **kwargs,
    ) -> None:
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
        self.latest_costmap: OccupancyGrid | None = None
        self.latest_odom: PoseStamped | None = None
        self.latest_path: Path | None = None

        # Control thread
        self.planning_thread: threading.Thread | None = None
        self.stop_planning = threading.Event()

        logger.info("Local planner module initialized")

    @rpc
    def start(self) -> None:
        super().start()

        self._disposables.add(Disposable(self.lidar.subscribe(self._on_lidar)))
        self._disposables.add(Disposable(self.odom.subscribe(self._on_odom)))
        self._disposables.add(Disposable(self.path.subscribe(self._on_path)))

    @rpc
    def stop(self) -> None:
        self.cancel_planning()
        super().stop()

    def _on_lidar(self, msg: LidarMessage) -> None:
        self.latest_costmap = gradient(
            general_occupancy(
                msg,
                resolution=0.05,
                min_height=0.15,
                max_height=0.6,
            ),
            max_distance=0.25,
        )

    def _on_odom(self, msg: PoseStamped) -> None:
        self.latest_odom = msg

    def _on_path(self, msg: Path) -> None:
        self.latest_path = msg

        if msg and len(msg.poses) > 0:
            if self.planning_thread is None or not self.planning_thread.is_alive():
                self._start_planning_thread()

    def _start_planning_thread(self) -> None:
        self.stop_planning.clear()
        self.planning_thread = threading.Thread(target=self._follow_path_loop, daemon=True)
        self.planning_thread.start()

    def _follow_path_loop(self) -> None:
        while not self.stop_planning.is_set():
            if self.is_goal_reached():
                self.stop_planning.set()
                stop_cmd = Twist()
                self.cmd_vel.publish(stop_cmd)
                break

            # Compute and publish velocity
            self._plan()

            time.sleep(self.control_period)

    def _plan(self) -> None:
        """Compute and publish velocity command."""
        cmd_vel = self.compute_velocity()

        if cmd_vel is not None:
            self.cmd_vel.publish(cmd_vel)

    @abstractmethod
    def compute_velocity(self) -> Twist | None:
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

        return bool(abs(yaw_error) < self.orientation_tolerance)

    @rpc
    def reset(self) -> None:
        """Reset the local planner state, clearing the current path."""
        # Clear the latest path
        self.latest_path = None
        self.latest_odom = None
        self.latest_costmap = None
        self.cancel_planning()
        logger.info("Local planner reset")

    @rpc
    def cancel_planning(self) -> None:
        """Stop the local planner and any running threads."""
        if self.planning_thread and self.planning_thread.is_alive():
            self.stop_planning.set()
            self.planning_thread.join(timeout=1.0)
            self.planning_thread = None
        stop_cmd = Twist()
        self.cmd_vel.publish(stop_cmd)
