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
Test script for PBVS with ZED camera supporting robot arm frame.
Click on objects to select targets (requires origin to be set first).
Press 'o' to set manipulator origin at current camera pose.
"""

import cv2
import numpy as np
import sys
import os
import time

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dimos.hardware.zed_camera import ZEDCamera
from dimos.manipulation.ibvs.detection3d import Detection3DProcessor
from dimos.manipulation.ibvs.utils import parse_zed_pose
from dimos.perception.common.utils import find_clicked_object
from dimos.manipulation.ibvs.pbvs import PBVSController

try:
    import pyzed.sl as sl
except ImportError:
    print("Error: ZED SDK not installed.")
    sys.exit(1)


# Global for mouse events
mouse_click = None
warning_message = None
warning_time = None


def mouse_callback(event, x, y, flags, param):
    global mouse_click
    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_click = (x, y)


def main():
    global mouse_click, warning_message, warning_time

    print("=== PBVS Test with Robot Frame Support ===")
    print("IMPORTANT: Press 'o' to set manipulator origin FIRST")
    print("Then click objects to select targets | 'r' - reset | 'q' - quit")

    # Initialize camera
    zed = ZEDCamera(resolution=sl.RESOLUTION.HD720, depth_mode=sl.DEPTH_MODE.NEURAL)
    if not zed.open() or not zed.enable_positional_tracking():
        print("Camera initialization failed!")
        return

    # Get intrinsics
    cam_info = zed.get_camera_info()
    intrinsics = [
        cam_info["left_cam"]["fx"],
        cam_info["left_cam"]["fy"],
        cam_info["left_cam"]["cx"],
        cam_info["left_cam"]["cy"],
    ]

    # Initialize processors
    detector = Detection3DProcessor(intrinsics)
    pbvs = PBVSController(position_gain=0.3, rotation_gain=0.2, target_tolerance=0.1)

    # Setup window
    cv2.namedWindow("PBVS")
    cv2.setMouseCallback("PBVS", mouse_callback)

    try:
        while True:
            # Capture
            bgr, _, depth, pose_data = zed.capture_frame_with_pose()
            if bgr is None or depth is None:
                continue

            # Process
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            camera_pose = parse_zed_pose(pose_data) if pose_data else None
            results = detector.process_frame(rgb, depth, camera_pose)
            detections = results["detections"]

            # Handle click
            if mouse_click:
                clicked = find_clicked_object(mouse_click, detections)
                if clicked:
                    # Try to set target (will fail if no origin)
                    if not pbvs.set_target(clicked):
                        warning_message = "SET ORIGIN FIRST! Press 'o'"
                        warning_time = time.time()
                mouse_click = None

            # Create visualization with position overlays (robot frame if available)
            viz = detector.visualize_detections(rgb, detections, pbvs_controller=pbvs)

            # PBVS control
            if camera_pose:
                vel_cmd, ang_vel_cmd, reached, has_target = pbvs.compute_control(
                    camera_pose, detections
                )

                # Apply PBVS overlay
                viz = pbvs.create_status_overlay(viz, intrinsics)

                # Highlight target
                if has_target and pbvs.current_target and "bbox" in pbvs.current_target:
                    x1, y1, x2, y2 = map(int, pbvs.current_target["bbox"])
                    cv2.rectangle(viz, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    cv2.putText(
                        viz, "TARGET", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                    )

                # Print velocity commands for debugging (only if origin set)
                if vel_cmd and ang_vel_cmd:
                    print(f"Linear vel: ({vel_cmd.x:.3f}, {vel_cmd.y:.3f}, {vel_cmd.z:.3f}) m/s")
                    print(
                        f"Angular vel: ({ang_vel_cmd.x:.3f}, {ang_vel_cmd.y:.3f}, {ang_vel_cmd.z:.3f}) rad/s"
                    )

            # Convert back to BGR for OpenCV display
            viz_bgr = cv2.cvtColor(viz, cv2.COLOR_RGB2BGR)

            # Add camera pose info
            if camera_pose:
                # Show camera pose in appropriate frame
                if pbvs.manipulator_origin is not None:
                    cam_robot = pbvs.get_camera_pose_robot_frame(camera_pose)
                    if cam_robot:
                        pose_text = f"Camera [Robot]: ({cam_robot.pos.x:.2f}, {cam_robot.pos.y:.2f}, {cam_robot.pos.z:.2f})m"
                    else:
                        pose_text = f"Camera [ZED]: ({camera_pose.pos.x:.2f}, {camera_pose.pos.y:.2f}, {camera_pose.pos.z:.2f})m"
                else:
                    pose_text = f"Camera [ZED]: ({camera_pose.pos.x:.2f}, {camera_pose.pos.y:.2f}, {camera_pose.pos.z:.2f})m"

                cv2.putText(
                    viz_bgr, pose_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1
                )

                # Show origin status
                if pbvs.manipulator_origin is not None:
                    cv2.putText(
                        viz_bgr,
                        "Manipulator Origin SET",
                        (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1,
                    )
                else:
                    cv2.putText(
                        viz_bgr,
                        "Press 'o' to set manipulator origin",
                        (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 0, 0),
                        1,
                    )

            # Display warning message if active
            if warning_message and warning_time:
                # Show warning for 3 seconds
                if time.time() - warning_time < 3.0:
                    # Draw warning box
                    height, width = viz_bgr.shape[:2]
                    box_height = 80
                    box_y = height // 2 - box_height // 2

                    # Semi-transparent red background
                    overlay = viz_bgr.copy()
                    cv2.rectangle(
                        overlay, (50, box_y), (width - 50, box_y + box_height), (0, 0, 255), -1
                    )
                    viz_bgr = cv2.addWeighted(viz_bgr, 0.7, overlay, 0.3, 0)

                    # Warning text
                    text_size = cv2.getTextSize(warning_message, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
                    text_x = (width - text_size[0]) // 2
                    text_y = box_y + box_height // 2 + text_size[1] // 2

                    cv2.putText(
                        viz_bgr,
                        warning_message,
                        (text_x, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (255, 255, 255),
                        2,
                    )
                else:
                    warning_message = None
                    warning_time = None

            # Display
            cv2.imshow("PBVS", viz_bgr)

            # Keyboard
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("r"):
                pbvs.clear_target()
            elif key == ord("o") and camera_pose:
                pbvs.set_manipulator_origin(camera_pose)
                print(
                    f"Set manipulator origin at: ({camera_pose.pos.x:.3f}, {camera_pose.pos.y:.3f}, {camera_pose.pos.z:.3f})"
                )

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        detector.cleanup()
        zed.close()


if __name__ == "__main__":
    main()
