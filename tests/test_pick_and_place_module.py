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
Run script for Piper Arm robot with pick and place functionality.
Subscribes to visualization images and handles mouse/keyboard input.
"""

import asyncio
import sys
import threading
import time

import cv2
import numpy as np

try:
    import pyzed.sl as sl
except ImportError:
    print("Error: ZED SDK not installed.")
    sys.exit(1)

# Import LCM message types
from dimos_lcm.sensor_msgs import Image

from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.robot.agilex.piper_arm import PiperArmRobot
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.tests.test_pick_and_place_module")

# Global for mouse events
mouse_click = None
camera_mouse_click = None
current_window = None
pick_location = None  # Store pick location
place_location = None  # Store place location
place_mode = False  # Track if we're in place selection mode


def mouse_callback(event, x, y, _flags, param):
    global mouse_click, camera_mouse_click
    window_name = param
    if event == cv2.EVENT_LBUTTONDOWN:
        if window_name == "Camera Feed":
            camera_mouse_click = (x, y)
        else:
            mouse_click = (x, y)


class VisualizationNode:
    """Node that subscribes to visualization images and handles user input."""

    def __init__(self, robot: PiperArmRobot):
        self.lcm = LCM()
        self.latest_viz = None
        self.latest_camera = None
        self._running = False
        self.robot = robot

        # Subscribe to visualization topic
        self.viz_topic = Topic("/manipulation/viz", Image)
        self.camera_topic = Topic("/zed/color_image", Image)

    def start(self):
        """Start the visualization node."""
        self._running = True
        self.lcm.start()

        # Subscribe to visualization topic
        self.lcm.subscribe(self.viz_topic, self._on_viz_image)
        # Subscribe to camera topic for point selection
        self.lcm.subscribe(self.camera_topic, self._on_camera_image)

        logger.info("Visualization node started")

    def stop(self):
        """Stop the visualization node."""
        self._running = False
        cv2.destroyAllWindows()

    def _on_viz_image(self, msg: Image, topic: str):
        """Handle visualization image messages."""
        try:
            # Convert LCM message to numpy array
            data = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding == "rgb8":
                image = data.reshape((msg.height, msg.width, 3))
                # Convert RGB to BGR for OpenCV
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                self.latest_viz = image
        except Exception as e:
            logger.error(f"Error processing viz image: {e}")

    def _on_camera_image(self, msg: Image, topic: str):
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
        global mouse_click, camera_mouse_click, pick_location, place_location, place_mode

        # Setup windows
        cv2.namedWindow("Pick and Place")
        cv2.setMouseCallback("Pick and Place", mouse_callback, "Pick and Place")

        cv2.namedWindow("Camera Feed")
        cv2.setMouseCallback("Camera Feed", mouse_callback, "Camera Feed")

        print("=== Piper Arm Robot - Pick and Place ===")
        print("Control mode: Module-based with LCM communication")
        print("\nPICK AND PLACE WORKFLOW:")
        print("1. Click on an object to select PICK location")
        print("2. Click again to select PLACE location (auto pick & place)")
        print("3. OR press 'p' after first click for pick-only task")
        print("\nCONTROLS:")
        print("  'p' - Execute pick-only task (after selecting pick location)")
        print("  'r' - Reset everything")
        print("  'q' - Quit")
        print("  's' - SOFT STOP (emergency stop)")
        print("  'g' - RELEASE GRIPPER (open gripper)")
        print("  'SPACE' - EXECUTE target pose (manual override)")
        print("\nNOTE: Click on objects in the Camera Feed window!")

        while self._running:
            # Show camera feed with status overlay
            if self.latest_camera is not None:
                display_image = self.latest_camera.copy()

                # Add status text
                status_text = ""
                if pick_location is None:
                    status_text = "Click to select PICK location"
                    color = (0, 255, 0)
                elif place_location is None:
                    status_text = "Click to select PLACE location (or press 'p' for pick-only)"
                    color = (0, 255, 255)
                else:
                    status_text = "Executing pick and place..."
                    color = (255, 0, 255)

                cv2.putText(
                    display_image, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2
                )

                # Draw pick location marker if set
                if pick_location is not None:
                    # Simple circle marker
                    cv2.circle(display_image, pick_location, 10, (0, 255, 0), 2)
                    cv2.circle(display_image, pick_location, 2, (0, 255, 0), -1)

                    # Simple label
                    cv2.putText(
                        display_image,
                        "PICK",
                        (pick_location[0] + 15, pick_location[1] + 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2,
                    )

                # Draw place location marker if set
                if place_location is not None:
                    # Simple circle marker
                    cv2.circle(display_image, place_location, 10, (0, 255, 255), 2)
                    cv2.circle(display_image, place_location, 2, (0, 255, 255), -1)

                    # Simple label
                    cv2.putText(
                        display_image,
                        "PLACE",
                        (place_location[0] + 15, place_location[1] + 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 255),
                        2,
                    )

                    # Draw simple arrow between pick and place
                    if pick_location is not None:
                        cv2.arrowedLine(
                            display_image,
                            pick_location,
                            place_location,
                            (255, 255, 0),
                            2,
                            tipLength=0.05,
                        )

                cv2.imshow("Camera Feed", display_image)

            # Show visualization if available
            if self.latest_viz is not None:
                cv2.imshow("Pick and Place", self.latest_viz)

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
                    place_mode = False
                    logger.info("Reset pick and place selections")
                    # Also send reset to robot
                    action = self.robot.handle_keyboard_command("r")
                    if action:
                        logger.info(f"Action: {action}")
                elif key == ord("p"):
                    # Execute pick-only task if pick location is set
                    if pick_location is not None:
                        logger.info(f"Executing pick-only task at {pick_location}")
                        result = self.robot.pick_and_place(
                            pick_location[0],
                            pick_location[1],
                            None,  # No place location
                            None,
                        )
                        logger.info(f"Pick task started: {result}")
                        # Clear selection after sending
                        pick_location = None
                        place_location = None
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
            if camera_mouse_click:
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

                    # Start pick and place task with both locations
                    result = self.robot.pick_and_place(pick_location[0], pick_location[1], x, y)
                    logger.info(f"Pick and place task started: {result}")

                    # Clear all points after sending mission
                    pick_location = None
                    place_location = None

                camera_mouse_click = None

            # Handle mouse click from Pick and Place window (if viz is running)
            elif mouse_click and self.latest_viz is not None:
                # Similar logic for viz window clicks
                x, y = mouse_click

                if pick_location is None:
                    # First click - set pick location
                    pick_location = (x, y)
                    logger.info(f"Pick location set at ({x}, {y}) from viz window")
                elif place_location is None:
                    # Second click - set place location and execute
                    place_location = (x, y)
                    logger.info(f"Place location set at ({x}, {y}) from viz window")
                    logger.info(f"Executing pick at {pick_location} and place at ({x}, {y})")

                    # Start pick and place task with both locations
                    result = self.robot.pick_and_place(pick_location[0], pick_location[1], x, y)
                    logger.info(f"Pick and place task started: {result}")

                    # Clear all points after sending mission
                    pick_location = None
                    place_location = None

                mouse_click = None

            time.sleep(0.03)  # ~30 FPS


async def run_piper_arm_with_viz():
    """Run the Piper Arm robot with visualization."""
    logger.info("Starting Piper Arm Robot")

    # Create robot instance
    robot = PiperArmRobot()

    try:
        # Start the robot
        await robot.start()

        # Give modules time to fully initialize
        await asyncio.sleep(2)

        # Create and start visualization node
        viz_node = VisualizationNode(robot)
        viz_node.start()

        # Run visualization in separate thread
        viz_thread = threading.Thread(target=viz_node.run_visualization, daemon=True)
        viz_thread.start()

        # Keep running until visualization stops
        while viz_node._running:
            await asyncio.sleep(0.1)

        # Stop visualization
        viz_node.stop()

    except Exception as e:
        logger.error(f"Error running robot: {e}")
        import traceback

        traceback.print_exc()

    finally:
        # Clean up
        robot.stop()
        logger.info("Robot stopped")


if __name__ == "__main__":
    # Run the robot
    asyncio.run(run_piper_arm_with_viz())
