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

"""Test script for XArmTree integrated robot."""

import time
import cv2
import logging
import numpy as np
from concurrent.futures import ThreadPoolExecutor
from typing import Optional

from dimos.robot.xarm_tree.xarm_tree import XArmTree
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.msgs.sensor_msgs import Image
from dimos_lcm.std_msgs import String
from dimos.utils.logging_config import setup_logger

logger = setup_logger("test_xarm_tree")

# Suppress verbose loggers
logging.getLogger("lcm").setLevel(logging.WARNING)

# Global variables for mouse events
mouse_click = None
pick_location = None
place_location = None
task_in_progress = False


def mouse_callback(event, x, y, flags, param):
    """Handle mouse events."""
    global mouse_click
    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_click = (x, y)


class TestInterface:
    """Test interface for XArmTree robot."""

    def __init__(self, robot: XArmTree):
        self.robot = robot
        self.lcm = LCM()
        self.latest_camera = None
        self.mode = "pick_place"  # pick_place, servoing, or mobile_pick_place
        self.executor = ThreadPoolExecutor(max_workers=1)
        self._running = False

        # Subscribe to camera topic only
        self.camera_topic = Topic(robot.ZED_COLOR_TOPIC, Image)

    def start(self):
        """Start the test interface."""
        self._running = True
        self.lcm.start()
        self.lcm.subscribe(self.camera_topic, self._on_camera_image)
        logger.info("Test interface started")

    def _on_camera_image(self, msg: Image, _: str):
        """Handle camera image."""
        try:
            self.latest_camera = msg.to_opencv()
        except Exception as e:
            logger.error(f"Error processing camera image: {e}")

    def execute_pick(self, x, y):
        """Execute pick-only task."""
        global task_in_progress, pick_location, place_location
        self.pose()  # Ensure robot is in pose mode for manipulation
        try:
            result = self.robot.pick_and_place(x, y, None, None)
            if result.get("success"):
                logger.info(f"Pick task completed: {result.get('message', 'Success')}")
            else:
                logger.error(f"Pick task failed: {result.get('error', 'Unknown error')}")
        finally:
            pick_location = None
            place_location = None
            task_in_progress = False

    def execute_pick_and_place(self, pick_x, pick_y, place_x, place_y):
        """Execute pick and place task."""
        global task_in_progress, pick_location, place_location
        self.pose()  # Ensure robot is in pose mode for manipulation
        try:
            result = self.robot.pick_and_place(pick_x, pick_y, place_x, place_y)
            if result.get("success"):
                logger.info(f"Pick and place completed: {result.get('message', 'Success')}")
            else:
                logger.error(f"Pick and place failed: {result.get('error', 'Unknown error')}")
        finally:
            pick_location = None
            place_location = None
            task_in_progress = False

    def execute_servo(self, x, y):
        """Execute visual servoing task."""
        global task_in_progress
        try:
            logger.info(f"Starting servo to object at ({x}, {y})")
            if self.robot.servo_to_object(x, y, target_distance=0.5):
                logger.info("Reached object successfully")
            else:
                logger.error("Failed to servo to object")
        finally:
            task_in_progress = False

    def execute_mobile_pick_and_place(self, x, y):
        """Execute mobile pick and place task."""
        global task_in_progress
        try:
            logger.info(f"Starting mobile pick and place at ({x}, {y})")
            result = self.robot.mobile_pick_and_place(
                x, y, servo_distance=0.5, servo_timeout=35.0, pick_timeout=60.0
            )
            if result.get("success"):
                logger.info(f"Mobile pick and place completed: {result.get('message', 'Success')}")
            else:
                logger.error(
                    f"Mobile pick and place failed: {result.get('error', 'Unknown error')}"
                )
        finally:
            task_in_progress = False

    def stop(self):
        """Stop the test interface."""
        self._running = False
        self.executor.shutdown(wait=False)
        self.lcm.stop()


def main():
    """Main test function."""
    global mouse_click, pick_location, place_location, task_in_progress

    print("=" * 60)
    print("XArmTree Robot Test")
    print("=" * 60)
    print("This test expects:")
    print("  1. XArmRobot running separately")
    print("  2. UnitreeB1 running separately")
    print("  3. ZED camera publishing to LCM topics")
    print("=" * 60)

    # Create and start robot
    robot = XArmTree()
    robot.start()

    # Create test interface
    interface = TestInterface(robot)
    interface.start()

    # Wait for initialization
    time.sleep(2)

    # Put robot in pose mode (stand mode for manipulation)
    logger.info("Setting robot to pose mode...")
    if robot.pose():
        logger.info("Robot in pose mode")
    else:
        logger.error("Failed to set pose mode")

    # Reset arm
    logger.info("Resetting arm to zero position...")
    if robot.reset_arm():
        logger.info("Arm reset")
    else:
        logger.error("Failed to reset arm")

    # Open gripper
    logger.info("Opening gripper...")
    if robot.open_gripper():
        logger.info("Gripper opened")
    else:
        logger.error("Failed to open gripper")

    # Create OpenCV window
    cv2.namedWindow("XArmTree Camera")
    cv2.setMouseCallback("XArmTree Camera", mouse_callback)

    print("\n" + "=" * 60)
    print("CONTROLS:")
    print("  'm' - Switch mode (cycles through pick_place, servoing, mobile_pick_place)")
    print("  'p' - Execute pick-only (after selecting pick location)")
    print("  'r' - Reset everything")
    print("  's' - Stop servoing")
    print("  'g' - Open gripper")
    print("  'i' - Idle mode")
    print("  'o' - Pose mode (stand for manipulation)")
    print("  'w' - Walk mode")
    print("  'c' - Recovery mode")
    print("  'q' - Quit")
    print("\nMOVEMENT:")
    print("  ARROW KEYS - Move robot (↑↓ = forward/back, ←→ = left/right)")
    print("  '8' - Move up (linear Z)")
    print("  '2' - Move down (linear Z)")
    print("  '4' - Pitch forward (in stand mode)")
    print("  '5' - Pitch backward (in stand mode)")
    print("  SPACE - Stop movement")
    print("\nPICK AND PLACE MODE:")
    print("  1. Click to select PICK location")
    print("  2. Click to select PLACE location (executes task)")
    print("  OR press 'p' after first click for pick-only")
    print("\nVISUAL SERVOING MODE:")
    print("  Click on object to servo to it")
    print("\nMOBILE PICK AND PLACE MODE:")
    print("  Click on object to servo to it and pick it up")
    print("=" * 60 + "\n")

    try:
        while interface._running:
            # Show camera feed
            if interface.latest_camera is not None:
                display_image = interface.latest_camera.copy()
                # Add status overlay
                status_text = f"Mode: {interface.mode.upper().replace('_', ' ')}"
                if interface.mode == "pick_place":
                    color = (0, 255, 0)
                elif interface.mode == "servoing":
                    color = (255, 0, 255)
                else:  # mobile_pick_place
                    color = (0, 165, 255)
                cv2.putText(
                    display_image, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2
                )

                if interface.mode == "pick_place":
                    # Show pick/place status
                    if task_in_progress:
                        status = "Task in progress..."
                        color = (255, 165, 0)
                    elif pick_location is None:
                        status = "Click to select PICK location"
                        color = (0, 255, 0)
                    elif place_location is None:
                        status = "Click for PLACE (or 'p' for pick-only)"
                        color = (0, 255, 255)
                    else:
                        status = "Ready to execute"
                        color = (255, 0, 255)

                    cv2.putText(
                        display_image, status, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2
                    )

                    # Draw markers
                    if pick_location:
                        cv2.circle(display_image, pick_location, 10, (0, 255, 0), 2)
                        cv2.putText(
                            display_image,
                            "PICK",
                            (pick_location[0] + 15, pick_location[1] + 5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 255, 0),
                            2,
                        )

                    if place_location:
                        cv2.circle(display_image, place_location, 10, (0, 255, 255), 2)
                        cv2.putText(
                            display_image,
                            "PLACE",
                            (place_location[0] + 15, place_location[1] + 5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 255, 255),
                            2,
                        )

                        if pick_location:
                            cv2.arrowedLine(
                                display_image,
                                pick_location,
                                place_location,
                                (255, 255, 0),
                                2,
                                tipLength=0.05,
                            )
                elif interface.mode == "servoing":
                    # Servoing mode
                    if task_in_progress:
                        status = "Servoing in progress..."
                        color = (255, 165, 0)
                    else:
                        status = "Click object to servo"
                        color = (255, 0, 255)

                    cv2.putText(
                        display_image, status, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2
                    )
                else:
                    # Mobile pick and place mode
                    if task_in_progress:
                        status = "Mobile pick & place in progress..."
                        color = (255, 165, 0)
                    else:
                        status = "Click object to servo & pick"
                        color = (0, 165, 255)

                    cv2.putText(
                        display_image, status, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2
                    )

                cv2.imshow("XArmTree Camera", display_image)

            # Handle keyboard
            key = cv2.waitKey(30) & 0xFF
            if key != 255:  # Key was pressed
                if key == ord("q"):
                    logger.info("Quit requested")
                    interface._running = False
                    break
                elif key == ord("m"):
                    # Cycle through modes
                    if interface.mode == "pick_place":
                        interface.mode = "servoing"
                    elif interface.mode == "servoing":
                        interface.mode = "mobile_pick_place"
                    else:
                        interface.mode = "pick_place"
                    pick_location = None
                    place_location = None
                    task_in_progress = False
                    logger.info(f"Switched to {interface.mode.replace('_', ' ')} mode")
                elif key == ord("r"):
                    # Reset everything
                    pick_location = None
                    place_location = None
                    task_in_progress = False
                    robot.stop_servoing()
                    robot.reset_arm()
                    robot.open_gripper()
                    logger.info("Reset everything")
                elif key == ord("p"):
                    # Execute pick-only if location is set
                    if interface.mode == "pick_place" and pick_location and not task_in_progress:
                        logger.info(f"Executing pick-only at {pick_location}")
                        task_in_progress = True
                        interface.executor.submit(
                            interface.execute_pick, pick_location[0], pick_location[1]
                        )
                    elif task_in_progress:
                        logger.warning("Task already in progress")
                    elif interface.mode != "pick_place":
                        logger.warning("Pick-only available in pick_place mode")
                    else:
                        logger.warning("Select a pick location first")
                elif key == ord("s"):
                    # Stop servoing
                    robot.stop_servoing()
                    logger.info("Stopped servoing")
                elif key == ord("g"):
                    # Open gripper
                    robot.open_gripper()
                    logger.info("Gripper opened")
                elif key == ord("i"):
                    # Idle mode
                    robot.idle()
                    logger.info("Robot set to idle mode")
                elif key == ord("o"):
                    # Pose mode (stand for manipulation)
                    logger.info("Sending pose mode command...")
                    result = robot.pose()
                    logger.info(f"Robot set to pose mode: {result}")
                elif key == ord("w"):
                    # Walk mode
                    robot.walk()
                    logger.info("Robot set to walk mode")
                elif key == ord("c"):
                    # Recovery mode
                    robot.recovery()
                    logger.info("Robot set to recovery mode")
                # Arrow keys for movement (x,y velocities)
                elif key == 82:  # Up arrow
                    # Move forward
                    from dimos.msgs.geometry_msgs import Twist, Vector3

                    linear = Vector3()
                    linear.x, linear.y, linear.z = 0.3, 0.0, 0.0
                    angular = Vector3()
                    angular.x, angular.y, angular.z = 0.0, 0.0, 0.0
                    twist = Twist(linear=linear, angular=angular)
                    robot.move(twist)
                    logger.info("Moving forward")
                elif key == 84:  # Down arrow
                    # Move backward
                    from dimos.msgs.geometry_msgs import Twist, Vector3

                    linear = Vector3()
                    linear.x, linear.y, linear.z = -0.3, 0.0, 0.0
                    angular = Vector3()
                    angular.x, angular.y, angular.z = 0.0, 0.0, 0.0
                    twist = Twist(linear=linear, angular=angular)
                    robot.move(twist)
                    logger.info("Moving backward")
                elif key == 81:  # Left arrow
                    # Move left
                    from dimos.msgs.geometry_msgs import Twist, Vector3

                    linear = Vector3()
                    linear.x, linear.y, linear.z = 0.0, 0.3, 0.0
                    angular = Vector3()
                    angular.x, angular.y, angular.z = 0.0, 0.0, 0.0
                    twist = Twist(linear=linear, angular=angular)
                    robot.move(twist)
                    logger.info("Moving left")
                elif key == 83:  # Right arrow
                    # Move right
                    from dimos.msgs.geometry_msgs import Twist, Vector3

                    linear = Vector3()
                    linear.x, linear.y, linear.z = 0.0, -0.3, 0.0
                    angular = Vector3()
                    angular.x, angular.y, angular.z = 0.0, 0.0, 0.0
                    twist = Twist(linear=linear, angular=angular)
                    robot.move(twist)
                    logger.info("Moving right")
                # Number keys for pitch angles in stand mode
                elif key == ord("4"):
                    # Pitch forward in stand mode
                    from dimos.msgs.geometry_msgs import Twist, Vector3

                    linear = Vector3()
                    linear.x, linear.y, linear.z = 0.0, 0.0, 0.0
                    angular = Vector3()
                    angular.x, angular.y, angular.z = 0.0, 0.5, 0.0
                    twist = Twist(linear=linear, angular=angular)
                    robot.move(twist)
                    logger.info("Pitching forward (stand mode)")
                elif key == ord("5"):
                    # Pitch backward in stand mode
                    from dimos.msgs.geometry_msgs import Twist, Vector3

                    linear = Vector3()
                    linear.x, linear.y, linear.z = 0.0, 0.0, 0.0
                    angular = Vector3()
                    angular.x, angular.y, angular.z = 0.0, -0.5, 0.0
                    twist = Twist(linear=linear, angular=angular)
                    robot.move(twist)
                    logger.info("Pitching backward (stand mode)")
                elif key == ord("8"):
                    # Move up (linear Z positive)
                    from dimos.msgs.geometry_msgs import Twist, Vector3

                    linear = Vector3()
                    linear.x, linear.y, linear.z = 0.0, 0.0, 0.3
                    angular = Vector3()
                    angular.x, angular.y, angular.z = 0.0, 0.0, 0.0
                    twist = Twist(linear=linear, angular=angular)
                    robot.move(twist)
                    logger.info("Moving up (linear Z)")
                elif key == ord("2"):
                    # Move down (linear Z negative)
                    from dimos.msgs.geometry_msgs import Twist, Vector3

                    linear = Vector3()
                    linear.x, linear.y, linear.z = 0.0, 0.0, -0.3
                    angular = Vector3()
                    angular.x, angular.y, angular.z = 0.0, 0.0, 0.0
                    twist = Twist(linear=linear, angular=angular)
                    robot.move(twist)
                    logger.info("Moving down (linear Z)")
                elif key == 32:  # Space bar
                    # Stop movement
                    from dimos.msgs.geometry_msgs import Twist, Vector3

                    linear = Vector3()
                    linear.x, linear.y, linear.z = 0.0, 0.0, 0.0
                    angular = Vector3()
                    angular.x, angular.y, angular.z = 0.0, 0.0, 0.0
                    twist = Twist(linear=linear, angular=angular)
                    robot.move(twist)
                    logger.info("Stopping movement")
                else:
                    # Pass other keys to robot
                    action = robot.handle_keyboard_command(chr(key))
                    if action:
                        logger.info(f"Action: {action}")

            # Handle mouse clicks
            if mouse_click and not task_in_progress:
                x, y = mouse_click

                if interface.mode == "pick_place":
                    if pick_location is None:
                        # First click - set pick location
                        pick_location = (x, y)
                        logger.info(f"Pick location set at ({x}, {y})")
                    elif place_location is None:
                        # Second click - set place and execute
                        place_location = (x, y)
                        logger.info(f"Place location set at ({x}, {y})")
                        task_in_progress = True
                        interface.executor.submit(
                            interface.execute_pick_and_place,
                            pick_location[0],
                            pick_location[1],
                            x,
                            y,
                        )
                elif interface.mode == "servoing":
                    # Execute servoing
                    task_in_progress = True
                    interface.executor.submit(interface.execute_servo, x, y)
                elif interface.mode == "mobile_pick_place":
                    # Execute mobile pick and place
                    task_in_progress = True
                    interface.executor.submit(interface.execute_mobile_pick_and_place, x, y)

                mouse_click = None

            time.sleep(0.03)  # ~30 FPS

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        cv2.destroyAllWindows()
        interface.stop()
        robot.stop()
        logger.info("Test completed")


if __name__ == "__main__":
    main()
