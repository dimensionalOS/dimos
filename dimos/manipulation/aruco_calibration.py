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
one_time_calibration.py - Complete ArUco calibration for dual-camera robot setup
Run this ONCE, then use the saved calibration forever
"""

from datetime import datetime
import json
import os

import cv2
import numpy as np


class RobotCameraCalibrator:
    def __init__(self):
        # ArUco board parameters (measure your actual board!)
        self.board_squares_x = 6  # Number of squares horizontally
        self.board_squares_y = 4  # Number of squares vertically
        self.square_length = 0.08  # Size of chess square in meters (measure this!)
        self.marker_length = 0.06  # Size of ArUco marker in meters (measure this!)

        # Camera parameters (typical for 720p webcam, adjust if needed)
        self.camera_matrix = np.array([[600, 0, 640], [0, 600, 360], [0, 0, 1]], dtype=float)
        self.dist_coeffs = np.zeros(5)

        # Setup ArUco detector
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.board = cv2.aruco.CharucoBoard_create(
            self.board_squares_x,
            self.board_squares_y,
            self.square_length,
            self.marker_length,
            self.aruco_dict,
        )

    def detect_cameras(self):
        """Find available cameras"""
        print("\n=== DETECTING CAMERAS ===")
        available_cameras = []

        for i in range(10):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    print(f"✓ Camera {i} detected")
                    available_cameras.append(i)
                cap.release()

        if len(available_cameras) < 2:
            raise Exception(f"Need 2 cameras, only found {len(available_cameras)}")

        print(f"Using cameras: {available_cameras[0]} and {available_cameras[1]}")
        return available_cameras[0], available_cameras[1]

    def measure_robot_offset(self):
        """Get robot offset from ArUco board center"""
        print("\n=== ROBOT-TO-BOARD OFFSET ===")
        print("Measure from robot base center to ArUco board center")
        print("Use a tape measure or ruler for accuracy\n")

        print("Looking at your setup:")
        print("  - Positive X = toward the right")
        print("  - Positive Y = toward the back")
        print("  - Robot appears to be LEFT and FRONT of board\n")

        x_offset = float(input("X offset in meters (negative if robot is left of board): "))
        y_offset = float(input("Y offset in meters (negative if robot is front of board): "))
        z_offset = float(input("Z offset in meters (usually 0 if same height): "))

        # Create transformation matrix from board to robot
        board_to_robot = np.array(
            [[1, 0, 0, x_offset], [0, 1, 0, y_offset], [0, 0, 1, z_offset], [0, 0, 0, 1]]
        )

        print(f"\nRobot offset from board: ({x_offset:.3f}, {y_offset:.3f}, {z_offset:.3f}) m")
        return board_to_robot

    def calibrate_camera(self, camera_id, camera_name):
        """Calibrate a single camera to the ArUco board"""
        print(f"\n=== CALIBRATING {camera_name} (Camera {camera_id}) ===")
        print("Position camera to see the ArUco board clearly")
        print("Press SPACE to capture frames, 'c' to compute calibration\n")

        cap = cv2.VideoCapture(camera_id)
        if not cap.isOpened():
            raise Exception(f"Cannot open camera {camera_id}")

        # Set camera resolution
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        all_charuco_corners = []
        all_charuco_ids = []
        captured_frames = 0
        target_frames = 15

        while True:
            ret, frame = cap.read()
            if not ret:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.parameters
            )

            # Prepare display
            display = frame.copy()

            if ids is not None and len(ids) > 0:
                # Draw detected markers
                cv2.aruco.drawDetectedMarkers(display, corners, ids)

                # Get Charuco corners
                ret_charuco, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                    corners, ids, gray, self.board
                )

                if ret_charuco and len(charuco_corners) > 4:
                    # Draw Charuco corners
                    cv2.aruco.drawDetectedCornersCharuco(
                        display, charuco_corners, charuco_ids, (0, 255, 0)
                    )

                    # Try to estimate pose for visualization
                    ret_pose, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                        charuco_corners,
                        charuco_ids,
                        self.board,
                        self.camera_matrix,
                        self.dist_coeffs,
                        None,
                        None,
                    )

                    if ret_pose:
                        # Draw coordinate axes
                        cv2.aruco.drawAxis(
                            display, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1
                        )
                        cv2.putText(
                            display,
                            "GOOD! Press SPACE to capture",
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1,
                            (0, 255, 0),
                            2,
                        )
                else:
                    cv2.putText(
                        display,
                        "Need more markers visible",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 165, 255),
                        2,
                    )
            else:
                cv2.putText(
                    display,
                    "No ArUco markers detected",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 255),
                    2,
                )

            # Show frame count
            cv2.putText(
                display,
                f"Frames: {captured_frames}/{target_frames}",
                (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

            cv2.putText(
                display,
                "Press 'c' to compute with current frames",
                (10, 100),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (200, 200, 200),
                1,
            )

            # Show the frame
            cv2.imshow(f"{camera_name}", display)

            key = cv2.waitKey(1) & 0xFF

            if key == ord(" "):  # Capture frame
                if ids is not None and ret_charuco and len(charuco_corners) > 4:
                    all_charuco_corners.append(charuco_corners)
                    all_charuco_ids.append(charuco_ids)
                    captured_frames += 1
                    print(f"  Captured frame {captured_frames}/{target_frames}")

                    if captured_frames >= target_frames:
                        print("  Target frames reached, computing calibration...")
                        break

            elif key == ord("c"):  # Compute with current frames
                if captured_frames >= 5:  # Minimum frames needed
                    print(f"  Computing with {captured_frames} frames...")
                    break
                else:
                    print(f"  Need at least 5 frames (have {captured_frames})")

            elif key == ord("q"):  # Quit
                cap.release()
                cv2.destroyAllWindows()
                return None

        cap.release()
        cv2.destroyAllWindows()

        # Compute calibration from captured frames
        if len(all_charuco_corners) == 0:
            print("  ERROR: No valid frames captured!")
            return None

        print(f"  Computing pose from {len(all_charuco_corners)} frames...")

        # Get pose for each frame
        all_rvecs = []
        all_tvecs = []

        for corners, ids in zip(all_charuco_corners, all_charuco_ids, strict=False):
            ret, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                corners, ids, self.board, self.camera_matrix, self.dist_coeffs, None, None
            )
            if ret:
                all_rvecs.append(rvec)
                all_tvecs.append(tvec)

        if len(all_rvecs) == 0:
            print("  ERROR: Could not estimate pose from any frame!")
            return None

        # Average all poses for more robust calibration
        avg_rvec = np.mean(all_rvecs, axis=0)
        avg_tvec = np.mean(all_tvecs, axis=0)

        # Convert to 4x4 transformation matrix
        rotation_matrix, _ = cv2.Rodrigues(avg_rvec)

        # Camera-to-board transformation
        cam_to_board = np.eye(4)
        cam_to_board[:3, :3] = rotation_matrix
        cam_to_board[:3, 3] = avg_tvec.squeeze()

        print(f"  ✓ {camera_name} calibration complete!")
        print(f"    Translation: {avg_tvec.squeeze()}")

        return cam_to_board

    def verify_calibration(self, cam1_to_robot, cam2_to_robot):
        """Quick verification of calibration results"""
        print("\n=== VERIFICATION ===")

        # Test point at robot base (should be origin)
        robot_base = np.array([0, 0, 0, 1])

        # Project to each camera
        cam1_point = np.linalg.inv(cam1_to_robot) @ robot_base
        cam2_point = np.linalg.inv(cam2_to_robot) @ robot_base

        print(f"Robot base in camera 1 frame: {cam1_point[:3]}")
        print(f"Robot base in camera 2 frame: {cam2_point[:3]}")
        print("\nThese represent where each camera sees the robot base")

    def run_calibration(self):
        """Main calibration routine"""
        print("\n" + "=" * 50)
        print("    DUAL CAMERA ROBOT CALIBRATION")
        print("=" * 50)

        # Step 1: Detect cameras
        cam1_id, cam2_id = self.detect_cameras()

        # Step 2: Measure robot offset
        board_to_robot = self.measure_robot_offset()

        # Step 3: Calibrate first camera
        input("\nPress Enter to start calibrating Camera 1...")
        cam1_to_board = self.calibrate_camera(cam1_id, "Camera 1")
        if cam1_to_board is None:
            print("ERROR: Camera 1 calibration failed!")
            return False

        # Step 4: Calibrate second camera
        input("\nPress Enter to start calibrating Camera 2...")
        cam2_to_board = self.calibrate_camera(cam2_id, "Camera 2")
        if cam2_to_board is None:
            print("ERROR: Camera 2 calibration failed!")
            return False

        # Step 5: Transform to robot coordinate system
        cam1_to_robot = board_to_robot @ cam1_to_board
        cam2_to_robot = board_to_robot @ cam2_to_board

        # Step 6: Verify
        self.verify_calibration(cam1_to_robot, cam2_to_robot)

        # Step 7: Save calibration
        calibration_data = {
            "timestamp": datetime.now().isoformat(),
            "camera_ids": {"camera1": cam1_id, "camera2": cam2_id},
            "camera_configs": [
                {
                    "camera_id": cam1_id,
                    "intrinsics": self.camera_matrix.tolist(),
                    "distortion": self.dist_coeffs.tolist(),
                    "extrinsics": cam1_to_robot.tolist(),
                },
                {
                    "camera_id": cam2_id,
                    "intrinsics": self.camera_matrix.tolist(),
                    "distortion": self.dist_coeffs.tolist(),
                    "extrinsics": cam2_to_robot.tolist(),
                },
            ],
            "board_params": {
                "squares_x": self.board_squares_x,
                "squares_y": self.board_squares_y,
                "square_length": self.square_length,
                "marker_length": self.marker_length,
            },
            "board_to_robot_offset": board_to_robot.tolist(),
        }

        # Save JSON
        with open("camera_calibration.json", "w") as f:
            json.dump(calibration_data, f, indent=2)

        # Save numpy arrays
        np.savez("camera_calibration.npz", cam1_to_robot=cam1_to_robot, cam2_to_robot=cam2_to_robot)

        print("\n" + "=" * 50)
        print("    CALIBRATION COMPLETE!")
        print("=" * 50)
        print("\nSaved to:")
        print("  - camera_calibration.json (human readable)")
        print("  - camera_calibration.npz (numpy arrays)")

        print("\n=== COPY THIS TO YOUR CODE ===")
        print(f"""
import numpy as np

# Camera transforms (robot base is origin)
cam1_to_robot = np.array({cam1_to_robot.tolist()})

cam2_to_robot = np.array({cam2_to_robot.tolist()})

camera_configs = [
    {{
        'camera_id': {cam1_id},
        'intrinsics': {self.camera_matrix.tolist()[0] + self.camera_matrix.tolist()[1][:1]},
        'extrinsics': cam1_to_robot
    }},
    {{
        'camera_id': {cam2_id},
        'intrinsics': {self.camera_matrix.tolist()[0] + self.camera_matrix.tolist()[1][:1]},
        'extrinsics': cam2_to_robot
    }}
]
        """)

        return True


if __name__ == "__main__":
    calibrator = RobotCameraCalibrator()
    success = calibrator.run_calibration()

    if success:
        print("\n✓ Calibration successful! Cameras are now calibrated to robot frame.")
        print("  The robot base is at (0,0,0) in world coordinates.")
    else:
        print("\n✗ Calibration failed. Please try again.")
