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
Navigator module for coordinating global and local planning.
"""

import threading
import time
from enum import Enum
from typing import Callable, Optional

from dimos.core import Module, In, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos_lcm.std_msgs import String
from dimos.navigation.local_planner.local_planner import BaseLocalPlanner
from dimos.navigation.bt_navigator.goal_validator import find_safe_goal
from dimos.navigation.bt_navigator.recovery_server import RecoveryServer
from dimos.protocol.tf import TF
from dimos.utils.logging_config import setup_logger
from dimos_lcm.std_msgs import Bool
from dimos.utils.transform_utils import apply_transform

logger = setup_logger("dimos.navigation.bt_navigator")


class NavigatorState(Enum):
    """Navigator state machine states."""

    IDLE = "idle"
    FOLLOWING_PATH = "following_path"
    RECOVERY = "recovery"


class BehaviorTreeNavigator(Module):
    """
    Navigator module for coordinating navigation tasks.

    Manages the state machine for navigation, coordinates between global
    and local planners, and monitors goal completion.

    Inputs:
        - odom: Current robot odometry

    Outputs:
        - goal: Goal pose for global planner
    """

    # LCM inputs
    odom: In[PoseStamped] = None
    goal_request: In[PoseStamped] = None  # Input for receiving goal requests
    global_costmap: In[OccupancyGrid] = None

    # LCM outputs
    goal: Out[PoseStamped] = None
    goal_reached: Out[Bool] = None
    navigation_state: Out[String] = None

    def __init__(
        self,
        publishing_frequency: float = 1.0,
        reset_local_planner: Callable[[], None] = None,
        check_goal_reached: Callable[[], bool] = None,
        **kwargs,
    ):
        """Initialize the Navigator.

        Args:
            publishing_frequency: Frequency to publish goals to global planner (Hz)
            goal_tolerance: Distance threshold to consider goal reached (meters)
        """
        super().__init__(**kwargs)

        # Parameters
        self.publishing_frequency = publishing_frequency
        self.publishing_period = 1.0 / publishing_frequency

        # State machine
        self.state = NavigatorState.IDLE
        self.state_lock = threading.Lock()

        # Current goal
        self.current_goal: Optional[PoseStamped] = None
        self.original_goal: Optional[PoseStamped] = None
        self.goal_lock = threading.Lock()

        # Goal reached state
        self._goal_reached = False

        # Latest data
        self.latest_odom: Optional[PoseStamped] = None
        self.latest_costmap: Optional[OccupancyGrid] = None

        # Control thread
        self.control_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()

        # TF listener
        self.tf = TF()

        # Local planner
        self.reset_local_planner = reset_local_planner
        self.check_goal_reached = check_goal_reached

        # Recovery server for stuck detection
        self.recovery_server = RecoveryServer(stuck_duration=5.0)

        logger.info("Navigator initialized with stuck detection")

    @rpc
    def start(self):
        """Start the navigator module."""
        # Subscribe to inputs
        self.odom.subscribe(self._on_odom)
        self.goal_request.subscribe(self._on_goal_request)
        self.global_costmap.subscribe(self._on_costmap)

        # Start control thread
        self.stop_event.clear()
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()

        logger.info("Navigator started")

    @rpc
    def cancel_goal(self) -> bool:
        """
        Cancel the current navigation goal.

        Returns:
            True if goal was cancelled, False if no goal was active
        """
        self.stop()
        return True

    @rpc
    def cleanup(self):
        """Clean up resources including stopping the control thread."""
        # First stop navigation
        self.stop()

        # Then clean up the control thread
        self.stop_event.set()
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)

        logger.info("Navigator cleanup complete")

    @rpc
    def set_goal(self, goal: PoseStamped) -> bool:
        """
        Set a new navigation goal.

        Args:
            goal: Target pose to navigate to

        Returns:
            non-blocking: True if goal was accepted, False otherwise
            blocking: True if goal was reached, False otherwise
        """
        transformed_goal = self._transform_goal_to_odom_frame(goal)
        if not transformed_goal:
            logger.error("Failed to transform goal to odometry frame")
            return False

        with self.goal_lock:
            self.current_goal = transformed_goal
            self.original_goal = transformed_goal

        self._goal_reached = False

        with self.state_lock:
            self.state = NavigatorState.FOLLOWING_PATH

        return True

    @rpc
    def get_state(self) -> NavigatorState:
        """Get the current state of the navigator."""
        return self.state

    def _on_odom(self, msg: PoseStamped):
        """Handle incoming odometry messages."""
        self.latest_odom = msg

        if self.state == NavigatorState.FOLLOWING_PATH:
            self.recovery_server.update_odom(msg)

    def _on_goal_request(self, msg: PoseStamped):
        """Handle incoming goal requests."""
        self.set_goal(msg)

    def _on_costmap(self, msg: OccupancyGrid):
        """Handle incoming costmap messages."""
        self.latest_costmap = msg

    def _transform_goal_to_odom_frame(self, goal: PoseStamped) -> Optional[PoseStamped]:
        """Transform goal pose to the odometry frame."""
        if not goal.frame_id:
            return goal

        odom_frame = self.latest_odom.frame_id
        if goal.frame_id == odom_frame:
            return goal

        try:
            transform = self.tf.get(
                parent_frame=odom_frame,
                child_frame=goal.frame_id,
                time_point=goal.ts,
                time_tolerance=1.0,
            )

            if not transform:
                logger.error(f"Could not find transform from '{goal.frame_id}' to '{odom_frame}'")
                return None

            pose = apply_transform(goal, transform)
            transformed_goal = PoseStamped(
                position=pose.position,
                orientation=pose.orientation,
                frame_id=odom_frame,
                ts=goal.ts,
            )
            return transformed_goal

        except Exception as e:
            logger.error(f"Failed to transform goal: {e}")
            return None

    def _control_loop(self):
        """Main control loop running in separate thread."""
        while not self.stop_event.is_set():
            with self.state_lock:
                current_state = self.state
                self.navigation_state.publish(String(data=current_state.value))

            if current_state == NavigatorState.FOLLOWING_PATH:
                with self.goal_lock:
                    goal = self.current_goal
                    original_goal = self.original_goal

                if goal is not None and self.latest_costmap is not None:
                    # Check if robot is stuck
                    if self.recovery_server.check_stuck():
                        logger.warning("Robot is stuck! Cancelling goal and resetting.")
                        self.cancel_goal()
                        continue

                    costmap = self.latest_costmap.inflate(0.1).gradient(max_distance=1.0)

                    # Find safe goal position
                    safe_goal_pos = find_safe_goal(
                        costmap,
                        original_goal.position,
                        algorithm="bfs",
                        cost_threshold=60,
                        min_clearance=0.25,
                        max_search_distance=5.0,
                    )

                    # Create new goal with safe position
                    if safe_goal_pos:
                        safe_goal = PoseStamped(
                            position=safe_goal_pos,
                            orientation=goal.orientation,
                            frame_id=goal.frame_id,
                            ts=goal.ts,
                        )
                        self.goal.publish(safe_goal)
                        self.current_goal = safe_goal
                    else:
                        logger.warning("Could not find safe goal position, cancelling goal")
                        self.cancel_goal()

                    # Check if goal is reached
                    if self.check_goal_reached():
                        reached_msg = Bool()
                        reached_msg.data = True
                        self.goal_reached.publish(reached_msg)
                        self.stop()
                        self._goal_reached = True
                        logger.info("Goal reached, resetting local planner")

            elif current_state == NavigatorState.RECOVERY:
                with self.state_lock:
                    self.state = NavigatorState.IDLE

            time.sleep(self.publishing_period)

    @rpc
    def is_goal_reached(self) -> bool:
        """Check if the current goal has been reached.

        Returns:
            True if goal was reached, False otherwise
        """
        return self._goal_reached

    def stop(self):
        """Stop navigation and return to IDLE state."""
        with self.goal_lock:
            self.current_goal = None

        self._goal_reached = False

        with self.state_lock:
            self.state = NavigatorState.IDLE

        self.reset_local_planner()
        self.recovery_server.reset()  # Reset recovery server when stopping

        logger.info("Navigator stopped")
