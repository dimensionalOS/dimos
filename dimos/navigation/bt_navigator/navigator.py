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
from typing import Optional

from dimos.core import Module, In, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.navigation.local_planner.local_planner import BaseLocalPlanner
from dimos.navigation.bt_navigator.goal_validator import find_safe_goal
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

    def __init__(
        self,
        local_planner: BaseLocalPlanner,
        publishing_frequency: float = 1.0,
        treat_unknown_as_safe: bool = False,
        **kwargs,
    ):
        """Initialize the Navigator.

        Args:
            publishing_frequency: Frequency to publish goals to global planner (Hz)
            treat_unknown_as_safe: Whether to treat UNKNOWN cost values as safe in goal validation (default: False)
        """
        super().__init__(**kwargs)

        # Parameters
        self.publishing_frequency = publishing_frequency
        self.publishing_period = 1.0 / publishing_frequency
        self.treat_unknown_as_safe = treat_unknown_as_safe

        # State machine
        self.state = NavigatorState.IDLE
        self.state_lock = threading.Lock()

        # Current goal
        self.current_goal: Optional[PoseStamped] = None
        self.goal_lock = threading.Lock()

        # Goal reached state
        self._goal_reached = False
        self._goal_reached_lock = threading.Lock()

        # Latest data
        self.latest_odom: Optional[PoseStamped] = None
        self.latest_costmap: Optional[OccupancyGrid] = None

        # Control thread
        self.control_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()

        self.local_planner = local_planner
        # TF listener
        self.tf = TF()

        logger.info("Navigator initialized")

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
    def set_goal(self, goal: PoseStamped, blocking: bool = False) -> bool:
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

        with self._goal_reached_lock:
            self._goal_reached = False

        with self.state_lock:
            self.state = NavigatorState.FOLLOWING_PATH

        if blocking:
            while not self.is_goal_reached():
                if self.state == NavigatorState.IDLE:
                    logger.info("Navigation was cancelled")
                    return False

                time.sleep(self.publishing_period)

        return True

    @rpc
    def get_state(self) -> NavigatorState:
        """Get the current state of the navigator."""
        return self.state

    def _on_odom(self, msg: PoseStamped):
        """Handle incoming odometry messages."""
        self.latest_odom = msg

    def _on_goal_request(self, msg: PoseStamped):
        """Handle incoming goal requests."""
        self.set_goal(msg)

    def _on_costmap(self, msg: OccupancyGrid):
        """Handle incoming costmap messages."""
        self.latest_costmap = msg

    def _transform_goal_to_odom_frame(self, goal: PoseStamped) -> Optional[PoseStamped]:
        """Transform goal pose to the odometry frame."""
        if self.latest_odom is None:
            logger.error("Odometry not available, cannot transform goal.")
            return None

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

            if current_state == NavigatorState.FOLLOWING_PATH:
                with self.goal_lock:
                    goal = self.current_goal

                if goal is not None and self.latest_costmap is not None:
                    # Find safe goal position
                    safe_goal_pos = find_safe_goal(
                        self.latest_costmap,
                        goal.position,
                        algorithm="bfs",
                        cost_threshold=80,
                        min_clearance=0.1,
                        max_search_distance=5.0,
                        treat_unknown_as_safe=self.treat_unknown_as_safe,
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
                    else:
                        logger.info("No safe goal found, cancelling navigation")
                        self.cancel_goal()

                    if self.local_planner.is_goal_reached():
                        with self._goal_reached_lock:
                            self._goal_reached = True
                        logger.info("Goal reached!")
                        reached_msg = Bool()
                        reached_msg.data = True
                        self.goal_reached.publish(reached_msg)
                        self.local_planner.reset()
                        with self.goal_lock:
                            self.current_goal = None
                        with self.state_lock:
                            self.state = NavigatorState.IDLE

            elif current_state == NavigatorState.RECOVERY:
                with self.state_lock:
                    self.state = NavigatorState.IDLE

            time.sleep(self.publishing_period)

    @rpc
    def is_goal_reached(self) -> bool:
        """Check if the current goal has been reached."""
        with self._goal_reached_lock:
            return self._goal_reached

    def stop(self):
        """Stop navigation and return to IDLE state."""
        with self.goal_lock:
            self.current_goal = None

        with self._goal_reached_lock:
            self._goal_reached = False

        with self.state_lock:
            self.state = NavigatorState.IDLE

        self.local_planner.reset()

        logger.info("Navigator stopped")
