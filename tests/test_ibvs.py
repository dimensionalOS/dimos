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
import numpy as np
import sys
import os

import tests.test_header

from dimos.hardware.zed_camera import ZEDCamera
from dimos.hardware.piper_arm import PiperArm
from dimos.manipulation.visual_servoing.detection3d import Detection3DProcessor
from dimos.perception.common.utils import find_clicked_object
from dimos.manipulation.visual_servoing.pbvs import PBVS
from dimos.utils.transform_utils import (
    pose_to_matrix,
    matrix_to_pose,
    create_transform_from_6dof,
    compose_transforms,
    quaternion_to_euler,
)
from dimos.msgs.geometry_msgs import Vector3

try:
    import pyzed.sl as sl
except ImportError:
    print("Error: ZED SDK not installed.")
    sys.exit(1)


# Global for mouse events
mouse_click = None


def mouse_callback(event, x, y, flags, param):
    global mouse_click
    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_click = (x, y)


def execute_grasp(arm, target_object, grasp_width_offset: float = 0.02) -> bool:
    """
    Execute grasping by opening gripper to accommodate target object.
    
    Args:
        arm: Robot arm interface with gripper control
        target_object: ObjectData with size information
        safety_margin: Multiplier for gripper opening (default 1.5x object size)
        
    Returns:
        True if grasp was executed, False if no target or no size data
    """
    if not target_object:
        print("❌ No target object provided for grasping")
        return False
        
    if "size" not in target_object:
        print("❌ Target has no size information for grasping")
        return False
        
    # Get object size from detection3d data (already in meters)
    object_size = target_object["size"]
    object_width = object_size["width"]
    object_height = object_size["height"] 
    object_depth = object_size["depth"]
    
    # Use the larger dimension (width or height) for gripper opening
    # Depth is not relevant for gripper opening (that's approach direction)
    
    # Calculate gripper opening with safety margin
    gripper_opening = object_width + grasp_width_offset
    
    # Clamp gripper opening to reasonable limits (0.5cm to 10cm)
    gripper_opening = max(0.005, min(gripper_opening, 0.1))  # 0.5cm to 10cm
    
    print(f"🤏 Executing grasp: object size w={object_width*1000:.1f}mm h={object_height*1000:.1f}mm d={object_depth*1000:.1f}mm, "
          f"offset={grasp_width_offset*1000:.1f}mm, opening gripper to {gripper_opening*1000:.1f}mm")
    
    # Command gripper to open
    arm.cmd_gripper_ctrl(gripper_opening)
    
    return True


def main():
    global mouse_click

    # Control mode flag
    DIRECT_EE_CONTROL = True  # Set to True for direct EE pose control, False for velocity control
    
    print("=== PBVS Eye-in-Hand Test ===")
    print("Using EE pose as odometry for camera pose")
    print(f"Control mode: {'Direct EE Pose' if DIRECT_EE_CONTROL else 'Velocity Commands'}")
    print("Click objects to select targets | 'r' - reset | 'q' - quit")
    if DIRECT_EE_CONTROL:
        print("SAFETY CONTROLS:")
        print("  's' - SOFT STOP (emergency stop)")
        print("  'h' - GO HOME (return to safe position)")
        print("  'SPACE' - EXECUTE target pose (only moves when pressed)")
        print("  'g' - EXECUTE GRASP (open gripper for target object)")

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
        return

    # Define EE to camera transform (adjust these values for your setup)
    # Format: [x, y, z, rx, ry, rz] in meters and radians
    ee_to_camera_6dof = [-0.06, 0.03, -0.05, 0.0, -1.57, 0.0]
    
    # Create transform matrices
    pos = Vector3(ee_to_camera_6dof[0], ee_to_camera_6dof[1], ee_to_camera_6dof[2])
    rot = Vector3(ee_to_camera_6dof[3], ee_to_camera_6dof[4], ee_to_camera_6dof[5])
    T_ee_to_camera = create_transform_from_6dof(pos, rot)

    # Get camera intrinsics
    cam_info = zed.get_camera_info()
    intrinsics = [
        cam_info["left_cam"]["fx"],
        cam_info["left_cam"]["fy"],
        cam_info["left_cam"]["cx"],
        cam_info["left_cam"]["cy"],
    ]

    # Initialize processors
    detector = Detection3DProcessor(intrinsics)
    pbvs = PBVS(position_gain=0.3, rotation_gain=0.2, target_tolerance=0.05, pregrasp_distance=0.2, direct_ee_control=DIRECT_EE_CONTROL)

    # Setup window
    cv2.namedWindow("PBVS")
    cv2.setMouseCallback("PBVS", mouse_callback)

    # Control state for direct EE mode
    execute_target = False  # Only move when space is pressed
    last_valid_target = None
    
    try:
        while True:
            # Capture
            bgr, _, depth, _ = zed.capture_frame_with_pose()
            if bgr is None or depth is None:
                continue

            # Process
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            
            # Get EE pose from robot (this serves as our odometry)
            ee_pose = arm.get_ee_pose()
            
            # Transform EE pose to camera pose
            ee_transform = pose_to_matrix(ee_pose)
            camera_transform = compose_transforms(ee_transform, T_ee_to_camera)
            camera_pose = matrix_to_pose(camera_transform)

            # Process detections using camera transform
            detections = detector.process_frame(rgb, depth, camera_transform)

            # Handle click
            if mouse_click:
                clicked = find_clicked_object(mouse_click, detections)
                if clicked:
                    pbvs.set_target(clicked)
                mouse_click = None

            # Create visualization with position overlays
            viz = detector.visualize_detections(rgb, detections)

            # PBVS control
            vel_cmd, ang_vel_cmd, reached, target_tracked, target_pose = pbvs.compute_control(
                ee_pose, detections
            )

            # Apply commands to robot based on control mode
            if DIRECT_EE_CONTROL and target_pose and execute_target:
                # Direct EE pose control - only when space is pressed
                print(f"🎯 EXECUTING target pose: pos=({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, {target_pose.position.z:.3f})")
                last_valid_target = pbvs.get_current_target()
                arm.cmd_ee_pose(target_pose)
                execute_target = False  # Reset flag after execution
            elif not DIRECT_EE_CONTROL and vel_cmd and ang_vel_cmd:
                # Velocity control
                arm.cmd_vel_ee(
                    vel_cmd.x, vel_cmd.y, vel_cmd.z,
                    ang_vel_cmd.x, ang_vel_cmd.y, ang_vel_cmd.z
                )

            # Apply PBVS overlay
            viz = pbvs.create_status_overlay(viz)

            # Highlight target
            current_target = pbvs.get_current_target()
            if target_tracked and current_target and "bbox" in current_target:
                x1, y1, x2, y2 = map(int, current_target["bbox"])
                cv2.rectangle(viz, (x1, y1), (x2, y2), (0, 255, 0), 3)
                cv2.putText(
                    viz, "TARGET", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                )

            # Convert back to BGR for OpenCV display
            viz_bgr = cv2.cvtColor(viz, cv2.COLOR_RGB2BGR)

            # Add pose info
            mode_text = "Direct EE" if DIRECT_EE_CONTROL else "Velocity"
            cv2.putText(
                viz_bgr, f"Eye-in-Hand ({mode_text})", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1
            )
            
            camera_text = f"Camera: ({camera_pose.position.x:.2f}, {camera_pose.position.y:.2f}, {camera_pose.position.z:.2f})m"
            cv2.putText(
                viz_bgr, camera_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1
            )
            
            ee_text = f"EE: ({ee_pose.position.x:.2f}, {ee_pose.position.y:.2f}, {ee_pose.position.z:.2f})m"
            cv2.putText(
                viz_bgr, ee_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1
            )
            
            # Add direct EE control status
            if DIRECT_EE_CONTROL:
                if target_pose:
                    status_text = "Target Ready - Press SPACE to execute"
                    status_color = (0, 255, 255)  # Yellow
                else:
                    status_text = "No target selected"
                    status_color = (100, 100, 100)  # Gray
                    
                cv2.putText(
                    viz_bgr, status_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1
                )
                
                cv2.putText(
                    viz_bgr, "s=STOP | h=HOME | SPACE=EXECUTE | g=GRASP", (10, 110), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1
                )

            # Display
            cv2.imshow("PBVS", viz_bgr)

            # Keyboard
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("r"):
                pbvs.clear_target()
            elif key == ord("s"):
                # SOFT STOP - Emergency stop
                print("🛑 SOFT STOP - Emergency stopping robot!")
                arm.softStop()
            elif key == ord("h"):
                # GO HOME - Return to safe position
                print("🏠 GO HOME - Returning to safe position...")
                arm.gotoZero()
            elif key == ord(" "):
                # SPACE - Execute target pose (only in direct EE mode)
                if DIRECT_EE_CONTROL and target_pose:
                    execute_target = True
                    target_euler = quaternion_to_euler(target_pose.orientation, degrees=True)
                    print("⚡ SPACE pressed - Target will execute on next frame!")
                    print(f"📍 Target pose: pos=({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, {target_pose.position.z:.3f}) "
                          f"rot=({target_euler.x:.1f}°, {target_euler.y:.1f}°, {target_euler.z:.1f}°)")
            elif key == ord("g"):
                # G - Execute grasp (open gripper for target object)
                current_target = pbvs.get_current_target()
                if current_target:
                    last_valid_target = current_target
                if last_valid_target:
                    print("🤏 GRASP - Opening gripper for target object...")
                    success = execute_grasp(arm, last_valid_target, grasp_width_offset=0.03)
                    if success:
                        print("✅ Gripper opened successfully")
                    else:
                        print("❌ Failed to execute grasp")
                else:
                    print("❌ No target selected for grasping")

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        detector.cleanup()
        zed.close()
        arm.disable()


if __name__ == "__main__":
    main()
