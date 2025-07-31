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
from dimos.navigation.local_planner.local_planner import BaseLocalPlanner
from dimos.utils.logging_config import setup_logger
from dimos_lcm.std_msgs import Bool

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

    # LCM outputs
    goal: Out[PoseStamped] = None
    goal_reached: Out[Bool] = None

    def __init__(self, publishing_frequency: float = 1.0, **kwargs):
        """Initialize the Navigator.

        Args:
            publishing_frequency: Frequency to publish goals to global planner (Hz)
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
        self.goal_lock = threading.Lock()

        # Latest data
        self.latest_odom: Optional[PoseStamped] = None

        # Control thread
        self.control_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()

        # Local planner reference (to be connected externally)
        self.local_planner: Optional[BaseLocalPlanner] = None

        logger.info("Navigator initialized")

    @rpc
    def start(self):
        """Start the navigator module."""
        # Subscribe to inputs
        self.odom.subscribe(self._on_odom)

        # Only subscribe to goal_request if it has a transport
        if self.goal_request.transport is not None:
            self.goal_request.subscribe(self._on_goal_request)

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
            True if goal was accepted, False otherwise
        """
        with self.goal_lock:
            self.current_goal = goal

        with self.state_lock:
            self.state = NavigatorState.FOLLOWING_PATH

        logger.info(
            f"New goal set: ({goal.position.x:.2f}, {goal.position.y:.2f}, {goal.position.z:.2f})"
        )
        return True

    @rpc
    def connect_local_planner(self, local_planner):
        """Connect a local planner instance for goal monitoring."""
        self.local_planner = local_planner
        logger.info("Local planner connected")

    def _on_odom(self, msg: PoseStamped):
        """Handle incoming odometry messages."""
        self.latest_odom = msg

    def _on_goal_request(self, msg: PoseStamped):
        """Handle incoming goal requests."""
        self.set_goal(msg)

    def _control_loop(self):
        """Main control loop running in separate thread."""
        while not self.stop_event.is_set():
            with self.state_lock:
                current_state = self.state

            if current_state == NavigatorState.FOLLOWING_PATH:
                # Publish goal to global planner
                with self.goal_lock:
                    goal = self.current_goal

                if goal is not None:
                    self.goal.publish(goal)

                    # Check if goal reached
                    if self.local_planner and self.is_goal_reached():
                        logger.info("Goal reached!")
                        reached_msg = Bool()
                        reached_msg.data = True
                        self.goal_reached.publish(reached_msg)
                        # Reset local planner to clear the path
                        self.local_planner.reset()
                        with self.goal_lock:
                            self.current_goal = None
                        with self.state_lock:
                            self.state = NavigatorState.IDLE

            elif current_state == NavigatorState.RECOVERY:
                # Recovery mode placeholder - transitions back to IDLE
                with self.state_lock:
                    self.state = NavigatorState.IDLE

            # Sleep for control period
            time.sleep(self.publishing_period)

    @rpc
    def is_goal_reached(self) -> bool:
        """
        Check if the current goal has been reached.

        Returns:
            True if goal reached, False otherwise
        """
        if self.local_planner is None:
            return False

        try:
            # Call local planner's is_goal_reached RPC
            return self.local_planner.is_goal_reached()
        except Exception as e:
            logger.error(f"Failed to check goal status: {e}")
            return False

    def stop(self):
        """Stop navigation and return to IDLE state."""
        # Cancel any active goal
        with self.goal_lock:
            self.current_goal = None

        # Set state to IDLE
        with self.state_lock:
            self.state = NavigatorState.IDLE

        # Stop the local planner
        if self.local_planner:
            self.local_planner.reset()

        logger.info("Navigator stopped")
