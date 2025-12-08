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

# Copyright 2025 Dimensional Inc.

"""
Test script for PBVS with eye-in-hand configuration.
Uses EE pose as odometry for camera pose estimation.
Click on objects to select targets.
"""

import cv2
import sys

try:
    import pyzed.sl as sl
except ImportError:
    print("Error: ZED SDK not installed.")
    sys.exit(1)

from dimos.hardware.zed_camera import ZEDCamera
from dimos.hardware.piper_arm import PiperArm
from dimos.manipulation.visual_servoing.manipulation import Manipulation


# Global for mouse events
mouse_click = None


def mouse_callback(event, x, y, _flags, _param):
    global mouse_click
    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_click = (x, y)


def main():
    global mouse_click

    # Configuration
    DIRECT_EE_CONTROL = True  # True: direct EE pose control, False: velocity control
    INITIAL_GRASP_PITCH_DEGREES = 30  # 0° = level grasp, 90° = top-down grasp

    print("=== PBVS Eye-in-Hand Test ===")
    print("Using EE pose as odometry for camera pose")
    print(f"Control mode: {'Direct EE Pose' if DIRECT_EE_CONTROL else 'Velocity Commands'}")
    print("Click objects to select targets | 'r' - reset | 'q' - quit")
    if DIRECT_EE_CONTROL:
        print("SAFETY CONTROLS:")
        print("  's' - SOFT STOP (emergency stop)")
        print("  'h' - GO HOME (return to safe position)")
        print("  'SPACE' - EXECUTE target pose (only moves when pressed)")
        print("  'g' - RELEASE GRIPPER (open gripper to 100mm)")
        print("GRASP PITCH CONTROLS:")
        print("  '↑' - Increase grasp pitch by 15° (towards top-down)")
        print("  '↓' - Decrease grasp pitch by 15° (towards level)")

    # Initialize hardware
    zed = ZEDCamera(resolution=sl.RESOLUTION.HD720, depth_mode=sl.DEPTH_MODE.NEURAL)
    if not zed.open():
        print("Camera initialization failed!")
        return

    # Initialize robot arm
    try:
        arm = PiperArm()
        print("Initialized Piper arm")
    except Exception as e:
        print(f"Failed to initialize Piper arm: {e}")
        zed.close()
        return

    # Initialize manipulation system
    try:
        manipulation = Manipulation(
            camera=zed,
            arm=arm,
        )
    except Exception as e:
        print(f"Failed to initialize manipulation system: {e}")
        zed.close()
        arm.disable()
        return

    # Set initial grasp pitch
    manipulation.set_grasp_pitch(INITIAL_GRASP_PITCH_DEGREES)

    # Setup window
    cv2.namedWindow("PBVS")
    cv2.setMouseCallback("PBVS", mouse_callback)

    try:
        while True:
            # Update manipulation system
            feedback = manipulation.update()
            if feedback is None:
                continue

            # Handle mouse click
            if mouse_click:
                x, y = mouse_click
                manipulation.pick_target(x, y)
                mouse_click = None

            # Get and display visualization
            viz = manipulation.get_visualization()
            if viz is not None:
                cv2.imshow("PBVS", viz)

            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            else:
                manipulation.handle_keyboard_command(key)

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        zed.close()
        arm.disable()


if __name__ == "__main__":
    main()
