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

"""Demo script for testing ROS navigation with the ROSNav module."""

import logging
import signal
import sys
import threading
import time

import rclpy
from rclpy.node import Node

from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger("demo_ros_navigation", level=logging.INFO)

# Global variable to track if we should keep running
running = True
nav_module = None


def signal_handler(signum, frame):
    """Handle shutdown signals gracefully."""
    global running, nav_module
    logger.info("Shutting down...")
    running = False
    if nav_module:
        nav_module.stop_navigation()
    sys.exit(0)


def main():
    """Main function to test ROS navigation - simplified standalone version."""
    global running, nav_module

    logger.info("Starting ROS navigation demo (standalone mode)...")

    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Initialize ROS2 if not already initialized
    if not rclpy.ok():
        rclpy.init()

    # Import here to avoid circular dependencies
    from dimos.navigation.rosnav import ROSNav

    # Create the navigation module instance
    logger.info("Creating ROSNav module instance...")
    nav_module = ROSNav()

    # Since we can't use the full Dimos deployment, we need to handle
    # the ROS spinning manually
    def spin_thread():
        while running and rclpy.ok():
            try:
                rclpy.spin_once(nav_module._node, timeout_sec=0.1)
            except Exception as e:
                if running:
                    logger.error(f"ROS2 spin error: {e}")

    # Start the navigation module
    logger.info("Starting ROSNav module...")
    # Note: We can't call nav_module.start() directly without proper Dimos setup
    # Instead, we'll start the ROS spinning thread manually

    spin_thread_obj = threading.Thread(target=spin_thread, daemon=True)
    spin_thread_obj.start()

    # Give the system time to initialize
    logger.info("Waiting for system initialization...")
    time.sleep(5.0)

    # Create a test pose for navigation
    # Moving 2 meters forward and 1 meter to the left relative to base_link
    test_pose = PoseStamped(
        position=Vector3(2.0, 1.0, 0.0),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="base_link",  # Relative to robot's current position
        ts=time.time(),
    )

    logger.info(
        f"Navigating to test position: ({test_pose.position.x:.2f}, {test_pose.position.y:.2f}, {test_pose.position.z:.2f}) @ {test_pose.frame_id}"
    )

    # Perform navigation
    logger.info("Sending navigation command...")
    success = nav_module.navigate_to(test_pose, timeout=30.0)

    if success:
        logger.info("Navigation successful!")
    else:
        logger.warning("Navigation failed or timed out")

    # Wait a bit before stopping
    time.sleep(2.0)

    # Stop the navigation module
    logger.info("Stopping navigation module...")
    nav_module.stop_navigation()

    # Clean up
    running = False
    if nav_module._node:
        nav_module._node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()

    logger.info("Demo completed")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)
