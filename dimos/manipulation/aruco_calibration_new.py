#!/usr/bin/env python3
"""
realsense_aruco_calibration.py - ArUco calibration for RealSense cameras
Adapted from OAK-D calibration for RealSense D435/D435i
"""

import cv2
import numpy as np
import json
import pyrealsense2 as rs
import time
from datetime import datetime

class RealSenseRobotCalibrator:
    def __init__(self):
        # ArUco board parameters - ADJUST TO YOUR BOARD!
        self.board_squares_x = 5  
        self.board_squares_y = 5  
        self.square_length = 0.11176    # 11.76cm
        self.marker_length = 0.089408   # 8.9408cm
        
        print(f"\nBoard config: {self.board_squares_x}x{self.board_squares_y} squares")
        print(f"Square length: {self.square_length*1000:.1f}mm")
        print(f"Marker length: {self.marker_length*1000:.1f}mm")
        print(f"Marker ratio: {self.marker_length/self.square_length:.2f}")
        print(f"Using ArUco dictionary: DICT_4X4_50\n")
        
        # Setup ArUco detector for DICT_4X4_50
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        
        # Relaxed detection parameters
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 50
        self.parameters.adaptiveThreshWinSizeStep = 5
        self.parameters.minMarkerPerimeterRate = 0.01
        self.parameters.maxMarkerPerimeterRate = 4.0
        
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        # Create Charuco board
        self.board = cv2.aruco.CharucoBoard(
            (self.board_squares_x, self.board_squares_y),
            self.square_length,
            self.marker_length,
            self.aruco_dict
        )
        
        self.charuco_detector = cv2.aruco.CharucoDetector(self.board)
    
    def detect_cameras(self):
        """Find available RealSense cameras"""
        print("\n=== DETECTING REALSENSE CAMERAS ===")
        ctx = rs.context()
        devices = ctx.query_devices()
        
        print(f"Found {len(devices)} RealSense devices:")
        for i, device in enumerate(devices):
            serial = device.get_info(rs.camera_info.serial_number)
            name = device.get_info(rs.camera_info.name)
            print(f"  Device {i}: {name} (S/N: {serial})")
        
        if len(devices) == 0:
            raise Exception("No RealSense cameras found!")
        
        return devices
    
    def create_realsense_pipeline(self, device_index=0):
        """Create RealSense pipeline using proven configuration from manipulation code"""
        ctx = rs.context()
        devices = ctx.query_devices()
        
        if device_index >= len(devices):
            raise ValueError(f"Device index {device_index} not found")
        
        device = devices[device_index]
        serial = device.get_info(rs.camera_info.serial_number)
        
        print(f"  Initializing RealSense {serial}")
        
        # STEP 1: Hardware reset (important for D435)
        print(f"    1. Hardware reset...")
        device.hardware_reset()
        time.sleep(3)
        
        # STEP 2: Get fresh context after reset
        ctx = rs.context()
        devices = ctx.query_devices()
        for d in devices:
            if d.get_info(rs.camera_info.serial_number) == serial:
                device = d
                break
        
        # STEP 3: Create pipeline with config
        pipeline = rs.pipeline()
        config = rs.config()
        
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 6)  # 6 FPS for stability
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
        
        # STEP 4: Start pipeline
        print(f"    2. Starting pipeline (640x480 @ 6fps)...")
        profile = pipeline.start(config)
        
        # STEP 5: Create align object
        align = rs.align(rs.stream.color)
        
        # STEP 6: CRITICAL warmup phase for D435
        print(f"    3. Warming up (essential for D435)...")
        for i in range(50):
            try:
                pipeline.wait_for_frames(timeout_ms=500)
            except:
                pass
            if i % 10 == 0:
                print(f"       Warmup {i}/50")
        
        # STEP 7: Validate and get intrinsics
        print(f"    4. Getting intrinsics...")
        frames = pipeline.wait_for_frames(timeout_ms=10000)
        aligned = align.process(frames)
        color_frame = aligned.get_color_frame()
        
        if not color_frame:
            raise RuntimeError("Could not get color frame after warmup")
        
        # Get intrinsics
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        
        # Create camera matrix
        camera_matrix = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ], dtype=float)
        
        # Get distortion coefficients
        dist_coeffs = np.array(intrinsics.coeffs)
        
        print(f"    ✓ Ready! Intrinsics: fx={intrinsics.fx:.1f}, fy={intrinsics.fy:.1f}")
        print(f"      cx={intrinsics.ppx:.1f}, cy={intrinsics.ppy:.1f}")
        
        return pipeline, align, camera_matrix, dist_coeffs, serial
    
    def measure_robot_offset(self):
        """Get robot offset from ArUco board origin"""
        print("\n=== ROBOT-TO-BOARD OFFSET ===")
        print("Measure from ArUco board ORIGIN to robot base center\n")
        
        print("Coordinate frames:")
        print("  Board: X=right, Y=down, Z=forward (from camera view)")
        print("  Robot: X=forward, Y=left, Z=up\n")
        
        print("Enter offset in METERS:")
        x_offset = float(input("X offset (board→robot, meters): "))
        y_offset = float(input("Y offset (board→robot, meters): "))
        z_offset = float(input("Z offset (board→robot, meters): "))
        
        # Ask about rotation
        print("\nIs the board aligned with robot axes? (y/n)")
        aligned = input().lower() == 'y'
        
        if aligned:
            # Simple translation only
            board_to_robot = np.array([
                [1, 0, 0, x_offset],
                [0, 1, 0, y_offset],
                [0, 0, 1, z_offset],
                [0, 0, 0, 1]
            ])
            print("Using identity rotation (board aligned with robot)")
        else:
            print("\nSpecify board rotation (common cases):")
            print("  1. Board rotated 90° CCW from robot X")
            print("  2. Board rotated 180° from robot X")
            print("  3. Custom rotation")
            choice = input("Choice (1/2/3): ")
            
            if choice == '1':
                board_to_robot = np.array([
                    [0, -1, 0, x_offset],
                    [1,  0, 0, y_offset],
                    [0,  0, 1, z_offset],
                    [0,  0, 0, 1]
                ])
            elif choice == '2':
                board_to_robot = np.array([
                    [-1, 0, 0, x_offset],
                    [0, -1, 0, y_offset],
                    [0,  0, 1, z_offset],
                    [0,  0, 0, 1]
                ])
            else:
                # Your original hardcoded rotation
                board_to_robot = np.array([
                    [0, -1, 0, y_offset],   
                    [-1, 0, 0, x_offset],   
                    [0,  0, 1, z_offset],
                    [0,  0, 0, 1]
                ])
        
        print(f"\nBoard offset: ({x_offset:.3f}, {y_offset:.3f}, {z_offset:.3f}) m")
        return board_to_robot
    
    def calibrate_camera(self, device_index, camera_name):
        """Calibrate a single RealSense camera to the ArUco board"""
        print(f"\n=== CALIBRATING {camera_name} (Device {device_index}) ===")
        print("Position camera to see the ArUco board clearly")
        print("Press SPACE to capture frames, 'c' to compute calibration")
        print("Press 'q' to quit\n")
        
        all_charuco_corners = []
        all_charuco_ids = []
        captured_frames = 0
        target_frames = 15
        
        # Create pipeline
        pipeline, align, camera_matrix, dist_coeffs, serial = self.create_realsense_pipeline(device_index)
        
        print(f"✓ Camera connected: {serial}")
        
        # Create window
        window_name = f"{camera_name}"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        
        try:
            while True:
                # Get frames
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                color_frame = aligned_frames.get_color_frame()
                
                if not color_frame:
                    continue
                
                # Convert to numpy array
                frame = np.asanyarray(color_frame.get_data())
                # Convert RGB to BGR for OpenCV
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
                # Detect ArUco markers
                corners, ids, rejected = self.detector.detectMarkers(gray)
                
                # Prepare display
                display = frame.copy()
                
                if ids is not None and len(ids) > 0:
                    # Draw detected markers
                    cv2.aruco.drawDetectedMarkers(display, corners, ids)
                    
                    cv2.putText(display, f"ArUco markers: {len(ids)}", 
                               (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 
                               0.5, (255, 255, 255), 1)
                    
                    # Get Charuco corners
                    charuco_corners, charuco_ids, marker_corners, marker_ids = \
                        self.charuco_detector.detectBoard(gray)
                    
                    corner_count = len(charuco_corners) if charuco_corners is not None else 0
                    cv2.putText(display, f"ChArUco corners: {corner_count}", 
                               (10, 170), cv2.FONT_HERSHEY_SIMPLEX, 
                               0.5, (255, 255, 255), 1)
                    
                    if charuco_corners is not None and corner_count > 4:
                        # Draw Charuco corners
                        cv2.aruco.drawDetectedCornersCharuco(
                            display, charuco_corners, charuco_ids, (0, 255, 0)
                        )
                        
                        # Estimate pose for visualization
                        obj_points, img_points = self.board.matchImagePoints(
                            charuco_corners, charuco_ids
                        )
                        
                        if len(obj_points) > 4:
                            ret_pose, rvec, tvec = cv2.solvePnP(
                                obj_points, img_points,
                                camera_matrix, dist_coeffs
                            )
                            
                            if ret_pose:
                                # Draw coordinate axes
                                cv2.drawFrameAxes(
                                    display, camera_matrix, dist_coeffs,
                                    rvec, tvec, 0.2,  # 20cm axes
                                    thickness=3
                                )
                                
                                cv2.putText(display, "GOOD! Press SPACE to capture", 
                                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                                           0.7, (0, 255, 0), 2)
                    else:
                        cv2.putText(display, f"Too few corners: {corner_count}", 
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                                   0.7, (0, 165, 255), 2)
                else:
                    cv2.putText(display, "No ArUco markers detected", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                               0.7, (0, 0, 255), 2)
                
                # Show frame count
                cv2.putText(display, f"Frames: {captured_frames}/{target_frames}", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.6, (255, 255, 255), 2)
                
                cv2.imshow(window_name, display)
                
                # Key handling
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord(' '):  # Space - capture
                    if (ids is not None and charuco_corners is not None and 
                        len(charuco_corners) > 4):
                        all_charuco_corners.append(charuco_corners)
                        all_charuco_ids.append(charuco_ids)
                        captured_frames += 1
                        print(f"  ✓ Captured frame {captured_frames}/{target_frames}")
                        
                        if captured_frames >= target_frames:
                            print(f"  Target reached, computing calibration...")
                            break
                
                elif key == ord('c'):  # Compute calibration
                    if captured_frames >= 5:
                        print(f"  Computing with {captured_frames} frames...")
                        break
                    else:
                        print(f"  Need at least 5 frames (have {captured_frames})")
                
                elif key == ord('q') or key == 27:  # Quit
                    print("  Calibration cancelled")
                    cv2.destroyAllWindows()
                    pipeline.stop()
                    return None, None, None
        
        finally:
            cv2.destroyAllWindows()
            pipeline.stop()
        
        # Compute calibration
        if len(all_charuco_corners) == 0:
            print("  ERROR: No valid frames captured!")
            return None, None, None
        
        print(f"  Computing pose from {len(all_charuco_corners)} frames...")
        
        # Get pose for each frame
        all_rvecs = []
        all_tvecs = []
        
        for corners, ids in zip(all_charuco_corners, all_charuco_ids):
            obj_points, img_points = self.board.matchImagePoints(corners, ids)
            
            if len(obj_points) > 4:
                ret, rvec, tvec = cv2.solvePnP(
                    obj_points, img_points,
                    camera_matrix, dist_coeffs
                )
                if ret:
                    all_rvecs.append(rvec)
                    all_tvecs.append(tvec)
        
        if len(all_rvecs) == 0:
            print("  ERROR: Could not estimate pose!")
            return None, None, None
        
        # Average all poses
        avg_rvec = np.mean(all_rvecs, axis=0)
        avg_tvec = np.mean(all_tvecs, axis=0)
        
        # Convert to 4x4 transformation matrix
        rotation_matrix, _ = cv2.Rodrigues(avg_rvec)
        
        cam_to_board = np.eye(4)
        cam_to_board[:3, :3] = rotation_matrix
        cam_to_board[:3, 3] = avg_tvec.squeeze()
        
        print(f"  ✓ {camera_name} calibration complete!")
        print(f"    Translation to board: {avg_tvec.squeeze()}")
        print(f"    Serial: {serial}")
        
        # Store intrinsics as 1D array (format used in your pipeline)
        intrinsics_1d = [camera_matrix[0,0], camera_matrix[1,1], 
                        camera_matrix[0,2], camera_matrix[1,2]]
        
        return cam_to_board, intrinsics_1d, serial
    
    def run_calibration(self):
        """Main calibration routine for single RealSense camera"""
        print("\n" + "="*50)
        print("    REALSENSE ROBOT CALIBRATION")
        print("="*50)
        
        # Step 1: Detect cameras
        devices = self.detect_cameras()
        
        # Ask which camera to calibrate
        if len(devices) > 1:
            print("\nWhich camera to calibrate?")
            for i in range(len(devices)):
                print(f"  {i}: Device {i}")
            device_idx = int(input("Enter device number: "))
        else:
            device_idx = 0
            print(f"\nUsing device 0 (only device found)")
        
        # Step 2: Measure robot offset
        board_to_robot = self.measure_robot_offset()
        
        # Step 3: Calibrate camera
        input("\nPress Enter to start calibration...")
        cam_to_board, intrinsics, serial = self.calibrate_camera(device_idx, "RealSense")
        
        if cam_to_board is None:
            print("ERROR: Calibration failed!")
            return False
        
        # Step 4: Transform to robot coordinate system
        cam_to_robot = board_to_robot @ cam_to_board
        
        # Step 5: Verify
        print("\n=== VERIFICATION ===")
        cam_pos = cam_to_robot[:3, 3]
        print(f"Camera position from robot base:")
        print(f"  X={cam_pos[0]:.3f}m, Y={cam_pos[1]:.3f}m, Z={cam_pos[2]:.3f}m")
        print("\n⚠️  VERIFY: Does this match your physical setup?")
        
        # Step 6: Save calibration
        calibration_data = {
            'timestamp': datetime.now().isoformat(),
            'serial': serial,
            'intrinsics': intrinsics,  # Already in [fx, fy, cx, cy] format
            'extrinsics': cam_to_robot.tolist(),
            'board_params': {
                'squares_x': self.board_squares_x,
                'squares_y': self.board_squares_y,
                'square_length': self.square_length,
                'marker_length': self.marker_length
            },
            'board_to_robot_offset': board_to_robot.tolist()
        }
        
        # Save files
        with open('realsense_calibration.json', 'w') as f:
            json.dump(calibration_data, f, indent=2)
        
        np.savez('realsense_calibration.npz',
                 cam_to_robot=cam_to_robot,
                 intrinsics=intrinsics)
        
        print("\n" + "="*50)
        print("    CALIBRATION COMPLETE!")
        print("="*50)
        print("\nSaved to:")
        print("  - realsense_calibration.json")
        print("  - realsense_calibration.npz")
        
        print("\n=== COPY THIS TO YOUR CODE ===")
        print(f"""
# RealSense calibration
cam_to_robot = np.array({cam_to_robot.tolist()})

camera_configs = [{{
    "camera_id": 0,
    "intrinsics": {intrinsics},  # [fx, fy, cx, cy]
    "extrinsics": cam_to_robot,
}}]
        """)
        
        return True

if __name__ == "__main__":
    calibrator = RealSenseRobotCalibrator()
    success = calibrator.run_calibration()
    
    if success:
        print("\n✓ Calibration successful!")
    else:
        print("\n✗ Calibration failed.")