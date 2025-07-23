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
Test script for pick and place manipulation module.
Subscribes to visualization images and handles mouse/keyboard input.
"""

import cv2
import sys
import asyncio
import threading
import time
import numpy as np
from typing import Optional

try:
    import pyzed.sl as sl
except ImportError:
    print("Error: ZED SDK not installed.")
    sys.exit(1)

from dimos import core
from dimos.hardware.zed_camera import ZEDModule
from dimos.manipulation.visual_servoing.manipulation_module import ManipulationModule
from dimos.protocol import pubsub
from dimos.utils.logging_config import setup_logger

# Import LCM message types
from dimos_lcm.sensor_msgs import Image as LCMImage
from dimos_lcm.sensor_msgs import CameraInfo
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic

logger = setup_logger("test_pick_and_place_module")

# Global for mouse events
mouse_click = None
camera_mouse_click = None
current_window = None


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

    def __init__(self, manipulation_module):
        self.lcm = LCM()
        self.latest_viz = None
        self.latest_camera = None
        self._running = False
        self.manipulation = manipulation_module

        # Subscribe to visualization topic
        self.viz_topic = Topic("/manipulation/viz", LCMImage)
        self.camera_topic = Topic("/zed/color_image", LCMImage)

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

    def _on_viz_image(self, msg: LCMImage, topic: str):
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

    def _on_camera_image(self, msg: LCMImage, topic: str):
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
        global mouse_click, camera_mouse_click

        # Setup windows
        cv2.namedWindow("Pick and Place")
        cv2.setMouseCallback("Pick and Place", mouse_callback, "Pick and Place")

        cv2.namedWindow("Camera Feed")
        cv2.setMouseCallback("Camera Feed", mouse_callback, "Camera Feed")

        print("=== Pick and Place Module Test ===")
        print("Control mode: Module-based with LCM communication")
        print("Click objects to select targets | 'r' - reset | 'q' - quit")
        print("SAFETY CONTROLS:")
        print("  's' - SOFT STOP (emergency stop)")
        print("  'g' - RELEASE GRIPPER (open gripper)")
        print("  'SPACE' - EXECUTE target pose (manual override)")
        print("GRASP PITCH CONTROLS:")
        print("  '↑' - Increase grasp pitch by 15° (towards top-down)")
        print("  '↓' - Decrease grasp pitch by 15° (towards level)")
        print("  'p' - Start pick and place task")
        print("\nNOTE: Click on objects in the Camera Feed window to select targets!")

        while self._running:
            # Show camera feed (always available)
            if self.latest_camera is not None:
                cv2.imshow("Camera Feed", self.latest_camera)

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
                elif key == ord("p"):
                    # Start pick and place task
                    if mouse_click:
                        x, y = mouse_click
                        result = self.manipulation.pick_and_place(x, y)
                        logger.info(f"Pick and place task: {result}")
                        mouse_click = None
                    else:
                        result = self.manipulation.pick_and_place()
                        logger.info(f"Pick and place task (no target): {result}")
                else:
                    # Send keyboard command to manipulation module
                    if key in [82, 84]:  # Arrow keys
                        action = self.manipulation.handle_keyboard_command(str(key))
                    else:
                        action = self.manipulation.handle_keyboard_command(chr(key))
                    if action:
                        logger.info(f"Action: {action}")

            # Handle mouse click from Camera Feed window
            if camera_mouse_click:
                # Start pick and place task with the clicked point
                x, y = camera_mouse_click
                result = self.manipulation.pick_and_place(x, y)
                logger.info(f"Started pick and place at ({x}, {y}) from camera feed: {result}")
                camera_mouse_click = None

            # Handle mouse click from Pick and Place window (if viz is running)
            elif mouse_click and self.latest_viz is not None:
                # If there's a pending click and we're not running a task, start one
                x, y = mouse_click
                result = self.manipulation.pick_and_place(x, y)
                logger.info(f"Started pick and place at ({x}, {y}): {result}")
                mouse_click = None

            time.sleep(0.03)  # ~30 FPS


async def test_pick_and_place_module():
    """Test the pick and place manipulation module."""
    logger.info("Starting Pick and Place Module test")

    # Start Dask
    dimos = core.start(2)  # Need 2 workers for ZED and manipulation modules

    # Enable LCM auto-configuration
    pubsub.lcm.autoconf()

    try:
        # Deploy ZED module
        logger.info("Deploying ZED module...")
        zed = dimos.deploy(
            ZEDModule,
            camera_id=0,
            resolution="HD720",
            depth_mode="NEURAL",
            fps=30,
            enable_tracking=False,  # We don't need tracking for manipulation
            publish_rate=30.0,
            frame_id="zed_camera",
        )

        # Configure ZED LCM transports
        zed.color_image.transport = core.LCMTransport("/zed/color_image", LCMImage)
        zed.depth_image.transport = core.LCMTransport("/zed/depth_image", LCMImage)
        zed.camera_info.transport = core.LCMTransport("/zed/camera_info", CameraInfo)

        # Deploy manipulation module
        logger.info("Deploying manipulation module...")
        manipulation = dimos.deploy(ManipulationModule)

        # Connect manipulation inputs to ZED outputs
        manipulation.rgb_image.connect(zed.color_image)
        manipulation.depth_image.connect(zed.depth_image)
        manipulation.camera_info.connect(zed.camera_info)

        # Configure manipulation output
        manipulation.viz_image.transport = core.LCMTransport("/manipulation/viz", LCMImage)

        # Print module info
        logger.info("Modules configured:")
        print("\nZED Module:")
        print(zed.io().result())
        print("\nManipulation Module:")
        print(manipulation.io().result())

        # Start modules
        logger.info("Starting modules...")
        zed.start()
        manipulation.start()

        # Give modules time to initialize
        await asyncio.sleep(2)

        # Create and start visualization node
        viz_node = VisualizationNode(manipulation)
        viz_node.start()

        # Run visualization in separate thread
        viz_thread = threading.Thread(target=viz_node.run_visualization, daemon=True)
        viz_thread.start()

        # Keep running until visualization stops
        while viz_node._running:
            await asyncio.sleep(0.1)

        # Stop modules
        logger.info("Stopping modules...")
        manipulation.stop()
        zed.stop()

        # Stop visualization
        viz_node.stop()

    except Exception as e:
        logger.error(f"Error in test: {e}")
        import traceback

        traceback.print_exc()

    finally:
        # Clean up
        dimos.close()
        logger.info("Test completed")


if __name__ == "__main__":
    # Run the test
    asyncio.run(test_pick_and_place_module())
