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

import logging
import time

from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import Joy
from dimos.msgs.std_msgs.Bool import Bool
from dimos.utils.logging_config import setup_logger

logger = setup_logger(level=logging.INFO)


# TODO: Remove, deprecated
class NavigationModule(Module):
    goal_pose: Out[PoseStamped]
    goal_reached: In[Bool]
    cancel_goal: Out[Bool]
    joy: Out[Joy]

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        """Initialize NavigationModule."""
        Module.__init__(self, *args, **kwargs)
        self.goal_reach = None

    @rpc
    def start(self) -> None:
        """Start the navigation module."""
        if self.goal_reached:
            self.goal_reached.subscribe(self._on_goal_reached)
        logger.info("NavigationModule started")

    def _on_goal_reached(self, msg: Bool) -> None:
        """Handle goal reached status messages."""
        self.goal_reach = msg.data  # type: ignore[assignment]

    def _set_autonomy_mode(self) -> None:
        """
        Set autonomy mode by publishing Joy message.
        """

        joy_msg = Joy(
            frame_id="dimos",
            axes=[
                0.0,  # axis 0
                0.0,  # axis 1
                -1.0,  # axis 2
                0.0,  # axis 3
                1.0,  # axis 4
                1.0,  # axis 5
                0.0,  # axis 6
                0.0,  # axis 7
            ],
            buttons=[
                0,  # button 0
                0,  # button 1
                0,  # button 2
                0,  # button 3
                0,  # button 4
                0,  # button 5
                0,  # button 6
                1,  # button 7 - controls autonomy mode
                0,  # button 8
                0,  # button 9
                0,  # button 10
            ],
        )

        if self.joy:
            self.joy.publish(joy_msg)
            logger.info("Setting autonomy mode via Joy message")

    @rpc
    def go_to(self, pose: PoseStamped, timeout: float = 60.0) -> bool:
        """
        Navigate to a target pose by publishing to LCM topics.

        Args:
            pose: Target pose to navigate to
            blocking: If True, block until goal is reached
            timeout: Maximum time to wait for goal (seconds)

        Returns:
            True if navigation was successful (or started if non-blocking)
        """
        logger.info(
            f"Navigating to goal: ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})"
        )

        self.goal_reach = None
        self._set_autonomy_mode()
        self.goal_pose.publish(pose)
        time.sleep(0.2)
        self.goal_pose.publish(pose)

        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.goal_reach is not None:
                return self.goal_reach
            time.sleep(0.1)

        self.stop()

        logger.warning(f"Navigation timed out after {timeout} seconds")
        return False

    @rpc
    def stop(self) -> bool:
        """
        Cancel current navigation by publishing to cancel_goal.

        Returns:
            True if cancel command was sent successfully
        """
        logger.info("Cancelling navigation")

        if self.cancel_goal:
            cancel_msg = Bool(data=True)
            self.cancel_goal.publish(cancel_msg)
            return True

        return False
