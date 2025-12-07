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
import time


from dimos.hardware.zed_camera import ZEDCamera
from dimos.hardware.piper_arm import PiperArm
from dimos.manipulation.visual_servoing.detection3d import Detection3DProcessor
from dimos.manipulation.visual_servoing.pbvs import PBVS, GraspStage
from dimos.manipulation.visual_servoing.utils import (
    find_clicked_detection,
    get_detection2d_for_detection3d,
    bbox2d_to_corners,
)
from dimos.utils.transform_utils import (
    pose_to_matrix,
    matrix_to_pose,
    create_transform_from_6dof,
    compose_transforms,
)
from dimos_lcm.geometry_msgs import Vector3

try:
    import pyzed.sl as sl
except ImportError:
    print("Error: ZED SDK not installed.")
    sys.exit(1)


# Global for mouse events
mouse_click = None


def mouse_callback(event, x, y, _flags, _param):
    global mouse_click
    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_click = (x, y)


def execute_grasp(arm, target_object, target_pose, grasp_width_offset: float = 0.02) -> bool:
    """
    Execute grasping by opening gripper to accommodate target object.

    Args:
        arm: Robot arm interface with gripper control
        target_object: Detection3D with size information
        grasp_width_offset: Additional width to add to object size for gripper opening

    Returns:
        True if grasp was executed, False if no target or no size data
    """
    if not target_object:
        print("❌ No target object provided for grasping")
        return False

    if not target_object.bbox or not target_object.bbox.size:
        print("❌ Target has no size information for grasping")
        return False

    # Get object size from detection3d data (already in meters)
    object_size = target_object.bbox.size
    object_width = object_size.x

    # Calculate gripper opening with offset
    gripper_opening = object_width + grasp_width_offset

    # Clamp gripper opening to reasonable limits (0.5cm to 10cm)
    gripper_opening = max(0.005, min(gripper_opening, 0.1))

    print(f"🤏 Executing grasp: opening gripper to {gripper_opening * 1000:.1f}mm")

    # Command gripper to open
    arm.cmd_gripper_ctrl(gripper_opening)
    arm.cmd_ee_pose(target_pose, line_mode=True)

    return True


def main():
    global mouse_click

    # Configuration
    DIRECT_EE_CONTROL = True  # True: direct EE pose control, False: velocity control

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
    pbvs = PBVS(
        position_gain=0.3,
        rotation_gain=0.2,
        target_tolerance=0.05,
        pregrasp_distance=0.25,
        grasp_distance=0.01,
        direct_ee_control=DIRECT_EE_CONTROL,
    )

    # Set custom grasp pitch (60 degrees - between level and top-down)
    GRASP_PITCH_DEGREES = 0  # 0° = level grasp, 90° = top-down grasp
    pbvs.set_grasp_pitch(GRASP_PITCH_DEGREES)

    # Setup window
    cv2.namedWindow("PBVS")
    cv2.setMouseCallback("PBVS", mouse_callback)

    # Control state for direct EE mode
    execute_target = False  # Only move when space is pressed
    last_valid_target = None

    # Rate limiting for pose execution
    MIN_EXECUTION_PERIOD = 1.0  # Minimum seconds between pose executions
    last_execution_time = 0

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
            detection_3d_array, detection_2d_array = detector.process_frame(
                rgb, depth, camera_transform
            )

            # Handle click
            if mouse_click:
                clicked_3d = find_clicked_detection(
                    mouse_click, detection_2d_array.detections, detection_3d_array.detections
                )
                if clicked_3d:
                    pbvs.set_target(clicked_3d)
                mouse_click = None

            # Create visualization with position overlays
            viz = detector.visualize_detections(
                rgb, detection_3d_array.detections, detection_2d_array.detections
            )

            # PBVS control
            vel_cmd, ang_vel_cmd, reached, target_tracked, target_pose = pbvs.compute_control(
                ee_pose, detection_3d_array
            )

            # Apply commands to robot based on control mode
            if DIRECT_EE_CONTROL and target_pose:
                # Check if enough time has passed since last execution
                current_time = time.time()
                if current_time - last_execution_time >= MIN_EXECUTION_PERIOD:
                    # Direct EE pose control
                    print(
                        f"🎯 EXECUTING target pose: pos=({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, {target_pose.position.z:.3f})"
                    )
                    last_valid_target = pbvs.get_current_target()
                    if pbvs.grasp_stage == GraspStage.PRE_GRASP:
                        arm.cmd_ee_pose(target_pose)
                        last_execution_time = current_time
                    elif pbvs.grasp_stage == GraspStage.GRASP and execute_target:
                        execute_grasp(arm, last_valid_target, target_pose, grasp_width_offset=0.03)
                        last_execution_time = current_time
                    execute_target = False  # Reset flag after execution
            elif not DIRECT_EE_CONTROL and vel_cmd and ang_vel_cmd:
                # Velocity control
                arm.cmd_vel_ee(
                    vel_cmd.x, vel_cmd.y, vel_cmd.z, ang_vel_cmd.x, ang_vel_cmd.y, ang_vel_cmd.z
                )

            # Add PBVS status overlay
            viz = pbvs.create_status_overlay(viz)

            # Highlight target
            current_target = pbvs.get_current_target()
            if target_tracked and current_target:
                det_2d = get_detection2d_for_detection3d(
                    current_target, detection_3d_array.detections, detection_2d_array.detections
                )
                if det_2d and det_2d.bbox:
                    x1, y1, x2, y2 = bbox2d_to_corners(det_2d.bbox)
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                    cv2.rectangle(viz, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    cv2.putText(
                        viz, "TARGET", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                    )

            # Convert back to BGR for OpenCV display
            viz_bgr = cv2.cvtColor(viz, cv2.COLOR_RGB2BGR)

            # Add pose info
            mode_text = "Direct EE" if DIRECT_EE_CONTROL else "Velocity"
            cv2.putText(
                viz_bgr,
                f"Eye-in-Hand ({mode_text})",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1,
            )

            camera_text = f"Camera: ({camera_pose.position.x:.2f}, {camera_pose.position.y:.2f}, {camera_pose.position.z:.2f})m"
            cv2.putText(
                viz_bgr, camera_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1
            )

            ee_text = f"EE: ({ee_pose.position.x:.2f}, {ee_pose.position.y:.2f}, {ee_pose.position.z:.2f})m"
            cv2.putText(viz_bgr, ee_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # Add control status
            if DIRECT_EE_CONTROL:
                status_text = (
                    "Target Ready - Press SPACE to execute" if target_pose else "No target selected"
                )
                status_color = (0, 255, 255) if target_pose else (100, 100, 100)
                cv2.putText(
                    viz_bgr, status_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1
                )
                cv2.putText(
                    viz_bgr,
                    "s=STOP | h=HOME | SPACE=EXECUTE | g=RELEASE",
                    (10, 110),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (255, 255, 255),
                    1,
                )

            # Display
            cv2.imshow("PBVS", viz_bgr)

            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("r"):
                pbvs.clear_target()
            elif key == ord("s"):
                print("🛑 SOFT STOP - Emergency stopping robot!")
                arm.softStop()
            elif key == ord("h"):
                print("🏠 GO HOME - Returning to safe position...")
                arm.gotoZero()
            elif key == ord(" ") and DIRECT_EE_CONTROL and target_pose:
                execute_target = True
                if pbvs.grasp_stage == GraspStage.PRE_GRASP:
                    pbvs.set_grasp_stage(GraspStage.GRASP)
                print("⚡ Executing target pose")
            elif key == 82:  # Up arrow - increase pitch
                new_pitch = min(90.0, pbvs.grasp_pitch_degrees + 15.0)
                pbvs.set_grasp_pitch(new_pitch)
                print(f"↑ Grasp pitch: {new_pitch:.0f}°")
            elif key == 84:  # Down arrow - decrease pitch
                new_pitch = max(0.0, pbvs.grasp_pitch_degrees - 15.0)
                pbvs.set_grasp_pitch(new_pitch)
                print(f"↓ Grasp pitch: {new_pitch:.0f}°")
            elif key == ord("g"):
                print("🖐️ Opening gripper")
                arm.release_gripper()

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        detector.cleanup()
        zed.close()
        arm.disable()


if __name__ == "__main__":
    main()
