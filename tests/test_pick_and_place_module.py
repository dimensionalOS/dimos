#!/usr/bin/env python3

import os
import argparse
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
Hardware-agnostic pick and place test module.
Supports both PiperArmRobot and XArmRobot with pick and place functionality.
Subscribes to visualization images and handles mouse/keyboard input.
"""

import cv2
import sys
import threading
import time
import signal
import numpy as np
from concurrent.futures import ThreadPoolExecutor
from typing import Union

try:
    import pyzed.sl
except ImportError:
    print("Error: ZED SDK not installed.")
    sys.exit(1)

from dimos.robot.agilex.piper_arm import PiperArmRobot
from dimos.robot.ufactory.xarm_robot import XArmRobot
from dimos.utils.logging_config import setup_logger

# Import LCM message types
from dimos_lcm.sensor_msgs import Image
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic

logger = setup_logger("dimos.tests.test_pick_and_place_module")

# Global for mouse events
camera_mouse_click = None
current_window = None
pick_location = None  # Store pick location
place_location = None  # Store place location
task_in_progress = False  # Track if task is running


def mouse_callback(event, x, y, _flags, param):
    global camera_mouse_click
    if event == cv2.EVENT_LBUTTONDOWN:
        camera_mouse_click = (x, y)


class CameraVisualizationNode:
    """Node that subscribes to camera images and handles user input."""

    def __init__(self, robot: Union[PiperArmRobot, XArmRobot]):
        self.lcm = LCM()
        self.latest_camera = None
        self._running = False
        self.robot = robot
        self.executor = ThreadPoolExecutor(max_workers=1)  # For running blocking tasks

        # Subscribe to camera topic
        self.camera_topic = Topic("/zed/color_image", Image)

    def start(self):
        """Start the camera visualization node."""
        self._running = True
        self.lcm.start()

        # Subscribe to camera topic for point selection
        self.lcm.subscribe(self.camera_topic, self._on_camera_image)

        logger.info("Camera visualization node started")

    def stop(self):
        """Stop the camera visualization node."""
        self._running = False
        self.executor.shutdown(wait=False)
        cv2.destroyAllWindows()

    def _on_camera_image(self, msg: Image, _topic: str):
        """Handle camera image messages."""
        try:
            # Convert LCM message to numpy array
            data = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding == "rgb8":
                image = data.reshape((msg.height, msg.width, 3))
                # Convert RGB to BGR for OpenCV
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                self.latest_camera = image
        except Exception as e:
            logger.error(f"Error processing camera image: {e}")

    def run_visualization(self):
        """Run the visualization loop with user interaction."""
        global camera_mouse_click, pick_location, place_location, task_in_progress

        # Setup window - only camera feed
        cv2.namedWindow("Camera Feed")
        cv2.setMouseCallback("Camera Feed", mouse_callback, "Camera Feed")

        print(f"=== {self.robot.__class__.__name__} - Pick and Place ===")
        print("Control mode: Simplified module-based with blocking operations")
        print("\nPICK AND PLACE WORKFLOW:")
        print("1. Click on an object to select PICK location")
        print("2. Click again to select PLACE location (blocking pick & place)")
        print("3. OR press 'p' after first click for pick-only task")
        print("\nCONTROLS:")
        print("  'p' - Execute pick-only task (after selecting pick location)")
        print("  'r' - Reset everything")
        print("  'q' - Quit")
        print("  's' - SOFT STOP (emergency stop)")
        print("  'g' - RELEASE GRIPPER (open gripper)")
        print("  'SPACE' - EXECUTE target pose (manual override)")
        print("\nNOTE: Tasks are blocking - wait for completion before new commands")
        print("      Click on objects in the Camera Feed window!")

        while self._running:
            # Show camera feed with status overlay
            if self.latest_camera is not None:
                display_image = self.latest_camera.copy()

                # Add status text
                status_text = ""
                if task_in_progress:
                    status_text = "Task in progress, please wait..."
                    color = (255, 165, 0)  # Orange
                elif pick_location is None:
                    status_text = "Click to select PICK location"
                    color = (0, 255, 0)
                elif place_location is None:
                    status_text = "Click to select PLACE location (or press 'p' for pick-only)"
                    color = (0, 255, 255)
                else:
                    status_text = "Ready to execute pick and place"
                    color = (255, 0, 255)

                cv2.putText(
                    display_image, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2
                )

                # Make local copies to avoid race conditions
                local_pick = pick_location
                local_place = place_location

                # Draw pick location marker if set
                if local_pick is not None:
                    # Simple circle marker
                    cv2.circle(display_image, local_pick, 10, (0, 255, 0), 2)
                    cv2.circle(display_image, local_pick, 2, (0, 255, 0), -1)

                    # Simple label
                    cv2.putText(
                        display_image,
                        "PICK",
                        (local_pick[0] + 15, local_pick[1] + 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )

                # Draw place location marker if set
                if local_place is not None:
                    # Simple circle marker
                    cv2.circle(display_image, local_place, 10, (0, 255, 255), 2)
                    cv2.circle(display_image, local_place, 2, (0, 255, 255), -1)

                    # Simple label
                    cv2.putText(
                        display_image,
                        "PLACE",
                        (local_place[0] + 15, local_place[1] + 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 255),
                        2,
                    )

                    # Draw simple arrow between pick and place
                    if local_pick is not None:
                        cv2.arrowedLine(
                            display_image,
                            local_pick,
                            local_place,
                            (255, 255, 0),
                            2,
                            tipLength=0.05,
                        )

                cv2.imshow("Camera Feed", display_image)

            # Removed visualization window - only showing camera feed

            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key != 255:  # Key was pressed
                if key == ord("q"):
                    logger.info("Quit requested")
                    self._running = False
                    break
                elif key == ord("r"):
                    # Reset everything
                    pick_location = None
                    place_location = None
                    task_in_progress = False
                    logger.info("Reset pick and place selections")
                    # Also send reset to robot
                    action = self.robot.handle_keyboard_command("r")
                    if action:
                        logger.info(f"Action: {action}")
                elif key == ord("p"):
                    # Execute pick-only task if pick location is set and not busy
                    if task_in_progress:
                        logger.warning("Task already in progress")
                    elif pick_location is not None:
                        logger.info(f"Executing pick-only task at {pick_location}")
                        task_in_progress = True

                        # Execute in separate thread to keep UI responsive
                        def execute_pick():
                            global task_in_progress, pick_location, place_location
                            try:
                                result = self.robot.pick_and_place(
                                    pick_location[0],
                                    pick_location[1],
                                    None,  # No place location
                                    None,
                                )
                                if result["success"]:
                                    logger.info(f"Pick task completed: {result['message']}")
                                else:
                                    logger.error(
                                        f"Pick task failed: {result.get('error', 'Unknown error')}"
                                    )
                            finally:
                                # Clear selection after task
                                pick_location = None
                                place_location = None
                                task_in_progress = False

                        self.executor.submit(execute_pick)
                    else:
                        logger.warning("Please select a pick location first!")
                else:
                    # Send keyboard command to robot
                    if key in [82, 84]:  # Arrow keys
                        action = self.robot.handle_keyboard_command(str(key))
                    else:
                        action = self.robot.handle_keyboard_command(chr(key))
                    if action:
                        logger.info(f"Action: {action}")

            # Handle mouse clicks
            if camera_mouse_click and not task_in_progress:
                x, y = camera_mouse_click

                if pick_location is None:
                    # First click - set pick location
                    pick_location = (x, y)
                    logger.info(f"Pick location set at ({x}, {y})")
                elif place_location is None:
                    # Second click - set place location and execute
                    place_location = (x, y)
                    logger.info(f"Place location set at ({x}, {y})")
                    logger.info(f"Executing pick at {pick_location} and place at ({x}, {y})")

                    # Execute in separate thread to keep UI responsive
                    task_in_progress = True

                    def execute_pick_and_place(pick_x, pick_y, place_x, place_y):
                        global task_in_progress, pick_location, place_location
                        try:
                            result = self.robot.pick_and_place(pick_x, pick_y, place_x, place_y)
                            if result["success"]:
                                logger.info(f"Pick and place completed: {result['message']}")
                            else:
                                logger.error(
                                    f"Pick and place failed: {result.get('error', 'Unknown error')}"
                                )
                        finally:
                            # Clear all points after task
                            pick_location = None
                            place_location = None
                            task_in_progress = False

                    self.executor.submit(
                        execute_pick_and_place, pick_location[0], pick_location[1], x, y
                    )

                camera_mouse_click = None

            # Removed viz window mouse handling

            time.sleep(0.03)  # ~30 FPS


# Global robot instance for signal handler
global_robot = None


def signal_handler(_signum, _frame):
    """Handle Ctrl+C gracefully."""
    global global_robot
    logger.info("\nSIGINT received, shutting down gracefully...")
    if global_robot and hasattr(global_robot, "piper_arm"):
        try:
            logger.info("Resetting arm to zero position...")
            arm_module = getattr(global_robot, "piper_arm", None) or getattr(
                global_robot, "xarm", None
            )
            if arm_module:
                if hasattr(arm_module, "reset_to_zero"):
                    arm_module.reset_to_zero()
                elif hasattr(arm_module, "goto_zero"):
                    arm_module.goto_zero()
            time.sleep(2)
        except Exception as e:
            logger.warning(f"Failed to reset arm: {e}")
    sys.exit(0)


def get_robot_class(robot_type: str):
    """Get robot class based on type string."""
    robot_classes = {
        "piper": PiperArmRobot,
        "xarm": XArmRobot,
    }

    if robot_type not in robot_classes:
        raise ValueError(
            f"Unknown robot type: {robot_type}. Available types: {list(robot_classes.keys())}"
        )

    return robot_classes[robot_type]


def run_robot_with_viz(robot_type: str = "piper", **kwargs):
    """Run the specified robot with visualization."""
    global global_robot

    robot_class = get_robot_class(robot_type)
    logger.info(f"Starting {robot_class.__name__}")

    # Create robot instance with appropriate parameters
    if robot_type == "xarm":
        robot = robot_class(
            arm_ip=kwargs.get("arm_ip", "10.0.0.197"),
            arm_type=kwargs.get("arm_type", "xarm7"),
            enable_mobile_base_control=kwargs.get("enable_mobile_base_control", False),
        )
    else:  # piper
        robot = robot_class(
            enable_mobile_base_control=kwargs.get("enable_mobile_base_control", False)
        )
    global_robot = robot  # Set global for signal handler

    try:
        # Start the robot
        robot.start()

        # Give modules time to fully initialize
        time.sleep(2)

        # Create and start camera visualization node
        viz_node = CameraVisualizationNode(robot)
        viz_node.start()

        # Run visualization in separate thread
        viz_thread = threading.Thread(target=viz_node.run_visualization, daemon=True)
        viz_thread.start()

        # Keep running until visualization stops
        while viz_node._running:
            time.sleep(0.1)

        # Stop visualization
        viz_node.stop()

    except Exception as e:
        logger.error(f"Error running robot: {e}")
        import traceback

        traceback.print_exc()

    finally:
        # Clean up
        logger.info("Cleaning up...")

        # Reset arm to zero position before stopping
        try:
            # Handle different robot arm modules
            arm_module = getattr(robot, "piper_arm", None) or getattr(robot, "xarm", None)
            if arm_module:
                logger.info("Resetting arm to zero position...")
                if hasattr(arm_module, "reset_to_zero"):
                    arm_module.reset_to_zero()
                elif hasattr(arm_module, "goto_zero"):
                    arm_module.goto_zero()
                time.sleep(2)  # Give it time to reach zero position
                arm_module.stop()
        except Exception as e:
            logger.warning(f"Failed to reset arm: {e}")

        robot.stop()
        logger.info(f"{robot.__class__.__name__} stopped")


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Hardware-agnostic pick and place test module")
    parser.add_argument(
        "--robot-type",
        choices=["piper", "xarm"],
        default=os.environ.get("ROBOT_TYPE", "piper"),
        help="Robot type to use (default: piper, can also be set via ROBOT_TYPE env var)",
    )
    parser.add_argument(
        "--arm-ip", default="10.0.0.197", help="xArm IP address (only used for xarm robot type)"
    )
    parser.add_argument(
        "--arm-type", default="xarm7", help="xArm type (only used for xarm robot type)"
    )
    parser.add_argument(
        "--enable-mobile-base", action="store_true", help="Enable mobile base control"
    )

    args = parser.parse_args()

    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)

    # Run the robot
    run_robot_with_viz(
        robot_type=args.robot_type,
        arm_ip=args.arm_ip,
        arm_type=args.arm_type,
        enable_mobile_base_control=args.enable_mobile_base,
    )
