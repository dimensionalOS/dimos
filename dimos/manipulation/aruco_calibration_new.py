#!/usr/bin/env python3
"""
one_time_calibration_oak_improved.py - Enhanced ArUco calibration for dual OAK-D S2 setup
Improved accuracy with validation and better corner detection
"""

import cv2
import numpy as np
import json
import depthai as dai
import contextlib
from datetime import datetime
import os

class OAKRobotCalibrator:
    def __init__(self):
        # ArUco board parameters - ADJUST TO YOUR BOARD!
        self.board_squares_x = 5  # Your board dimensions
        self.board_squares_y = 5  
        self.square_length = 0.11176    # 11.76cm - verified correct
        self.marker_length = 0.089408   # 9.4cm - verified correct
        
        print(f"\nBoard config: {self.board_squares_x}x{self.board_squares_y} squares")
        print(f"Square length: {self.square_length*1000:.1f}mm")
        print(f"Marker length: {self.marker_length*1000:.1f}mm")
        print(f"Marker ratio: {self.marker_length/self.square_length:.2f}")
        print(f"Using ArUco dictionary: DICT_4X4_50\n")
        
        # Camera intrinsics - will be loaded from OAK-D factory calibration
        self.camera_matrix = np.array([
            [500, 0, 320],
            [0, 500, 240],
            [0, 0, 1]
        ], dtype=float)
        self.dist_coeffs = np.zeros(5)
        
        # Setup ArUco detector for DICT_4X4_50
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        
        # IMPROVED: Enhanced detection parameters for better accuracy
        self.parameters = cv2.aruco.DetectorParameters()
        
        # Adaptive thresholding - optimized for OAK-D SR resolution
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 23  # Reduced for 640x480
        self.parameters.adaptiveThreshWinSizeStep = 4
        
        # Contour filtering - more permissive for board detection
        self.parameters.minMarkerPerimeterRate = 0.01
        self.parameters.maxMarkerPerimeterRate = 4.0
        self.parameters.polygonalApproxAccuracyRate = 0.05  # Slightly higher for distorted images
        self.parameters.minCornerDistanceRate = 0.05
        self.parameters.minMarkerDistanceRate = 0.05
        
        # CRITICAL: Enable corner refinement for subpixel accuracy
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters.cornerRefinementWinSize = 5
        self.parameters.cornerRefinementMaxIterations = 30
        self.parameters.cornerRefinementMinAccuracy = 0.1
        
        # Bits extraction
        self.parameters.perspectiveRemovePixelPerCell = 8  # Higher for better bit extraction
        self.parameters.perspectiveRemoveIgnoredMarginPerCell = 0.13
        
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        # Create Charuco board
        self.board = cv2.aruco.CharucoBoard(
            (self.board_squares_x, self.board_squares_y),
            self.square_length,
            self.marker_length,
            self.aruco_dict
        )
        
        self.charuco_detector = cv2.aruco.CharucoDetector(self.board)
        
        # Calibration quality thresholds
        self.min_corners_per_frame = 10  # Minimum corners for valid frame
        self.max_reprojection_error = 1.0  # Maximum acceptable error in pixels
        self.target_frames = 25  # More frames for better calibration
    
    def get_oak_intrinsics(self, device):
        """Get actual camera intrinsics from OAK-D factory calibration"""
        calibData = device.readCalibration()
        
        # Get intrinsics for the RGB camera (CAM_B socket, 640x480)
        intrinsics = calibData.getCameraIntrinsics(
            dai.CameraBoardSocket.CAM_B,
            640, 480
        )
        
        # Create camera matrix
        camera_matrix = np.array([
            [intrinsics[0][0], 0, intrinsics[0][2]],
            [0, intrinsics[1][1], intrinsics[1][2]],
            [0, 0, 1]
        ], dtype=float)
        
        # Get distortion coefficients
        distortion = np.array(calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_B))
        
        print(f"    ✓ Loaded factory calibration:")
        print(f"      fx={intrinsics[0][0]:.1f}, fy={intrinsics[1][1]:.1f}")
        print(f"      cx={intrinsics[0][2]:.1f}, cy={intrinsics[1][2]:.1f}")
        print(f"      Distortion coeffs: {distortion[:5]}")
        
        return camera_matrix, distortion
        
    def detect_cameras(self):
        """Find available OAK-D cameras"""
        print("\n=== DETECTING OAK-D CAMERAS ===")
        device_infos = dai.Device.getAllAvailableDevices()
        
        print(f"Found {len(device_infos)} OAK-D devices:")
        for i, info in enumerate(device_infos):
            print(f"  Device {i}: {info.getMxId()}")
        
        if len(device_infos) < 2:
            raise Exception(f"Need 2 OAK-D cameras, only found {len(device_infos)}")
        
        print(f"Will use first 2 cameras")
        return device_infos[:2]
    
    def create_pipeline(self):
        """Create pipeline for OAK-D S2"""
        pipeline = dai.Pipeline()
        
        # Use ColorCamera for left camera
        cam_left = pipeline.create(dai.node.ColorCamera)
        cam_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        cam_left.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
        cam_left.setPreviewSize(640, 480)
        cam_left.setInterleaved(False)
        cam_left.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        
        # Output color preview
        xout_color = pipeline.create(dai.node.XLinkOut)
        xout_color.setStreamName("color")
        cam_left.preview.link(xout_color.input)
        
        return pipeline
    
    def measure_robot_offset(self):
        """Get robot offset from ArUco board origin with validation"""
        print("\n=== ROBOT-TO-BOARD OFFSET ===")
        print("\nMEASUREMENT GUIDE:")
        print("1. Place ArUco board flat on the table")
        print("2. Board origin = top-left corner of first marker (ID 0)")
        print("3. Measure FROM board origin TO robot base center")
        print("4. Use a ruler/tape measure for accuracy (±5mm)")
        print("5. Double-check all measurements!")
        print("\nCoordinate frames:")
        print("  Board: X=right, Y=back, Z=down (from camera view)")
        print("  Robot: X=forward (toward camera), Y=right, Z=up\n")
        
        # Get measurements with validation
        while True:
            print("Enter measurements in BOARD coordinates (meters):")
            x_board = float(input("  X offset (right from board origin): "))
            y_board = float(input("  Y offset (back from board origin): "))
            z_board = float(input("  Z offset (down from board origin): "))
            
            print(f"\nYou entered:")
            print(f"  X = {x_board*100:.1f}cm (right from board)")
            print(f"  Y = {y_board*100:.1f}cm (back from board)")
            print(f"  Z = {z_board*100:.1f}cm (down from board)")
            
            confirm = input("\nIs this correct? (y/n): ").lower()
            if confirm == 'y':
                break
            print("\nLet's measure again...\n")
        
        # Create transformation with rotation
        board_to_robot = np.array([
            [ 0, -1,  0,  y_board],   # Robot X = -Board Y
            [-1,  0,  0,  x_board],   # Robot Y = -Board X
            [ 0,  0,  1,  z_board],   # Robot Z = -Board Z
            [ 0,  0,  0,  1]
        ])
        
        print(f"\n✓ Board-to-robot offset recorded")
        print(f"  Robot base is {np.linalg.norm([x_board, y_board, z_board])*100:.1f}cm from board origin")
        
        return board_to_robot
    
    def validate_frame(self, corners, ids, camera_matrix, dist_coeffs):
        """Validate captured frame quality using reprojection error"""
        if ids is None or len(ids) < 4:
            return False, float('inf')
        
        # Get object and image points
        obj_points, img_points = self.board.matchImagePoints(corners, ids)
        
        if len(obj_points) < self.min_corners_per_frame:
            return False, float('inf')
        
        # Estimate pose
        ret, rvec, tvec = cv2.solvePnP(
            obj_points, img_points,
            camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        
        if not ret:
            return False, float('inf')
        
        # Calculate reprojection error
        projected, _ = cv2.projectPoints(obj_points, rvec, tvec, 
                                        camera_matrix, dist_coeffs)
        error = cv2.norm(img_points, projected, cv2.NORM_L2) / len(projected)
        
        return error < self.max_reprojection_error, error
    
    def calibrate_camera(self, device_info, camera_name):
        """Calibrate a single OAK-D camera with enhanced accuracy"""
        print(f"\n=== CALIBRATING {camera_name} ({device_info.getMxId()[:12]}) ===")
        
        print("\nCAPTURE STRATEGY FOR BEST CALIBRATION:")
        print("1. Start with board filling ~60% of frame")
        print("2. Capture at different distances (40-80cm for OAK-D SR)")
        print("3. Tilt board ±30° in different directions")
        print("4. Move board to corners and center of frame")
        print("5. Ensure even lighting (avoid shadows)")
        print(f"6. Target: {self.target_frames} high-quality frames\n")
        
        print("Controls:")
        print("  SPACE - Capture frame (when green)")
        print("  'c'   - Compute calibration")
        print("  'q'   - Quit/cancel\n")
        
        all_charuco_corners = []
        all_charuco_ids = []
        captured_frames = 0
        rejected_frames = 0
        
        # Create pipeline and device
        pipeline = self.create_pipeline()
        
        with contextlib.ExitStack() as stack:
            device = stack.enter_context(
                dai.Device(pipeline, device_info, dai.UsbSpeed.SUPER)
            )
            
            # Get real calibration data from OAK-D
            camera_matrix, dist_coeffs = self.get_oak_intrinsics(device)
            
            # Store for refinement
            self.camera_matrix = camera_matrix
            self.dist_coeffs = dist_coeffs
            
            # Get output queue
            q_color = device.getOutputQueue(name="color", maxSize=4, blocking=False)
            
            print(f"✓ Camera connected: {device_info.getMxId()}")
            
            # Create window
            window_name = f"{camera_name}"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            
            while True:
                color_data = q_color.tryGet()
                
                if color_data is None:
                    continue
                
                frame = color_data.getCvFrame()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
                # Detect ArUco markers
                corners, ids, rejected = self.detector.detectMarkers(gray)
                
                # IMPROVED: Try to use refineDetectedMarkers if available (newer OpenCV)
                # Otherwise, just skip this step
                try:
                    if hasattr(cv2.aruco, 'refineDetectedMarkers'):
                        corners, ids, rejected, recovered = cv2.aruco.refineDetectedMarkers(
                            image=gray,
                            board=self.board,
                            detectedCorners=corners,
                            detectedIds=ids,
                            rejectedCorners=rejected,
                            cameraMatrix=camera_matrix,
                            distCoeffs=dist_coeffs
                        )
                        if recovered is not None and len(recovered) > 0:
                            print(f"  Recovered {len(recovered)} additional markers")
                except:
                    # If refineDetectedMarkers is not available, continue without it
                    pass
                
                # Prepare display
                display = frame.copy()
                
                # Status text background
                overlay = display.copy()
                cv2.rectangle(overlay, (0, 0), (640, 100), (0, 0, 0), -1)
                display = cv2.addWeighted(overlay, 0.3, display, 0.7, 0)
                
                if ids is not None and len(ids) > 0:
                    # Draw detected markers
                    cv2.aruco.drawDetectedMarkers(display, corners, ids)
                    
                    # Get Charuco corners
                    charuco_corners, charuco_ids, marker_corners, marker_ids = \
                        self.charuco_detector.detectBoard(gray)
                    
                    corner_count = len(charuco_corners) if charuco_corners is not None else 0
                    
                    # Validate frame quality
                    is_valid, error = self.validate_frame(corners, ids, camera_matrix, dist_coeffs)
                    
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
                                
                                # Status based on validation
                                if is_valid:
                                    status_color = (0, 255, 0)  # Green
                                    status_text = f"GOOD! Error: {error:.2f}px - Press SPACE"
                                else:
                                    status_color = (0, 165, 255)  # Orange
                                    status_text = f"Error too high: {error:.2f}px (max: {self.max_reprojection_error})"
                                
                                cv2.putText(display, status_text, 
                                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                                        0.7, status_color, 2)
                    else:
                        cv2.putText(display, f"Too few corners: {corner_count}/{self.min_corners_per_frame}", 
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                                0.7, (0, 165, 255), 2)
                    
                    # Show detection stats
                    cv2.putText(display, f"Markers: {len(ids)} | Corners: {corner_count}", 
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.5, (255, 255, 255), 1)
                else:
                    cv2.putText(display, "No ArUco markers detected", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.7, (0, 0, 255), 2)
                    cv2.putText(display, "Adjust position/lighting", 
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.5, (255, 255, 255), 1)
                
                # Show frame count
                cv2.putText(display, f"Captured: {captured_frames}/{self.target_frames} | Rejected: {rejected_frames}", 
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, (255, 255, 255), 1)
                
                # Show the frame
                cv2.imshow(window_name, display)
                
                # Key handling
                key = cv2.waitKey(10) & 0xFF
                
                if key == ord(' ') or key == 32:  # Space bar
                    if ids is not None and charuco_corners is not None:
                        is_valid, error = self.validate_frame(corners, ids, camera_matrix, dist_coeffs)
                        
                        if is_valid:
                            all_charuco_corners.append(charuco_corners)
                            all_charuco_ids.append(charuco_ids)
                            captured_frames += 1
                            print(f"  ✓ Frame {captured_frames} captured (error: {error:.3f}px)")
                            
                            if captured_frames >= self.target_frames:
                                print(f"  Target reached! Computing calibration...")
                                break
                        else:
                            rejected_frames += 1
                            print(f"  ✗ Frame rejected - error too high: {error:.3f}px")
                    else:
                        print(f"  ✗ Cannot capture - insufficient detection")
                
                elif key == ord('c') or key == ord('C'):
                    if captured_frames >= 10:  # Minimum frames for calibration
                        print(f"  Computing with {captured_frames} frames...")
                        break
                    else:
                        print(f"  ✗ Need at least 10 frames (have {captured_frames})")
                
                elif key == ord('q') or key == 27:
                    print("  Calibration cancelled")
                    cv2.destroyAllWindows()
                    return None, None, None
        
        cv2.destroyAllWindows()
        
        # Compute calibration from captured frames
        if len(all_charuco_corners) == 0:
            print("  ERROR: No valid frames captured!")
            return None, None, None
        
        print(f"\n  Computing pose from {len(all_charuco_corners)} frames...")
        
        # Get pose for each frame and compute statistics
        all_rvecs = []
        all_tvecs = []
        all_errors = []
        
        for corners, ids in zip(all_charuco_corners, all_charuco_ids):
            obj_points, img_points = self.board.matchImagePoints(corners, ids)
            
            if len(obj_points) > 4:
                ret, rvec, tvec = cv2.solvePnP(
                    obj_points, img_points,
                    camera_matrix, dist_coeffs,
                    flags=cv2.SOLVEPNP_ITERATIVE
                )
                if ret:
                    # Calculate error for this frame
                    projected, _ = cv2.projectPoints(obj_points, rvec, tvec, 
                                                    camera_matrix, dist_coeffs)
                    error = cv2.norm(img_points, projected, cv2.NORM_L2) / len(projected)
                    
                    all_rvecs.append(rvec)
                    all_tvecs.append(tvec)
                    all_errors.append(error)
        
        if len(all_rvecs) == 0:
            print("  ERROR: Could not estimate pose from any frame!")
            return None, None, None
        
        # Report calibration quality
        mean_error = np.mean(all_errors)
        std_error = np.std(all_errors)
        print(f"  Reprojection error: {mean_error:.3f} ± {std_error:.3f} pixels")
        
        if mean_error > 0.5:
            print("  ⚠️  Warning: High reprojection error. Consider recalibrating.")
        
        # Average all poses (could use more sophisticated methods)
        avg_rvec = np.mean(all_rvecs, axis=0)
        avg_tvec = np.mean(all_tvecs, axis=0)
        
        # Convert to 4x4 transformation matrix
        rotation_matrix, _ = cv2.Rodrigues(avg_rvec)
        
        cam_to_board = np.eye(4)
        cam_to_board[:3, :3] = rotation_matrix
        cam_to_board[:3, 3] = avg_tvec.squeeze()
        
        print(f"  ✓ {camera_name} calibration complete!")
        print(f"    Translation: {avg_tvec.squeeze()}")
        print(f"    Mean error: {mean_error:.3f} pixels")
        
        return cam_to_board, camera_matrix, dist_coeffs
    def validate_stereo_calibration(self, cam1_to_robot, cam2_to_robot, device_infos):
        """Validate that both cameras see the same world point correctly"""
        print("\n=== STEREO CALIBRATION VALIDATION ===")
        
        # Test multiple points in robot workspace
        test_points = [
            np.array([0.0, 0.0, 0.0, 1]),   # Robot base
            np.array([0.5, 0.0, 0.1, 1]),   # 50cm forward, 10cm up
            np.array([0.3, 0.2, 0.05, 1]),  # Right workspace
            np.array([0.3, -0.2, 0.05, 1]), # Left workspace
        ]
        
        all_errors = []
        
        for i, point_robot in enumerate(test_points):
            # Transform to each camera
            cam1_point = np.linalg.inv(cam1_to_robot) @ point_robot
            cam2_point = np.linalg.inv(cam2_to_robot) @ point_robot
            
            # Check consistency through relative transformation
            cam1_to_cam2 = np.linalg.inv(cam2_to_robot) @ cam1_to_robot
            cam2_point_from_cam1 = np.linalg.inv(cam1_to_cam2) @ cam1_point
            
            error = np.linalg.norm(cam2_point[:3] - cam2_point_from_cam1[:3])
            all_errors.append(error)
            
            print(f"\nTest point {i+1}: {point_robot[:3]}")
            print(f"  Camera 1 sees at: {cam1_point[:3]}")
            print(f"  Camera 2 sees at: {cam2_point[:3]}")
            print(f"  Cross-validation error: {error*1000:.1f}mm")
        
        mean_error = np.mean(all_errors)
        max_error = np.max(all_errors)
        
        print(f"\n=== VALIDATION RESULTS ===")
        print(f"Mean error: {mean_error*1000:.1f}mm")
        print(f"Max error: {max_error*1000:.1f}mm")
        
        # Extract camera positions
        cam1_pos = cam1_to_robot[:3, 3]
        cam2_pos = cam2_to_robot[:3, 3]
        baseline = np.linalg.norm(cam1_pos - cam2_pos)
        
        print(f"\nCamera positions from robot base:")
        print(f"  Camera 1: X={cam1_pos[0]:.3f}m, Y={cam1_pos[1]:.3f}m, Z={cam1_pos[2]:.3f}m")
        print(f"  Camera 2: X={cam2_pos[0]:.3f}m, Y={cam2_pos[1]:.3f}m, Z={cam2_pos[2]:.3f}m")
        print(f"  Baseline: {baseline*100:.1f}cm")
        
        if mean_error > 0.01:  # More than 10mm average error
            print("\n⚠️  WARNING: Large calibration error between cameras!")
            print("   Recommendations:")
            print("   1. Recalibrate with more frames")
            print("   2. Ensure accurate board-to-robot measurement")
            print("   3. Check camera {}'s calibration".format(
                "2 (second)" if all_errors[0] < all_errors[-1] else "1 (first)"
            ))
        else:
            print("\n✓ Stereo calibration validated successfully!")
            print("  Both cameras agree on 3D positions within tolerance")
    
    def run_calibration(self):
        """Main calibration routine with enhanced validation"""
        print("\n" + "="*60)
        print("    ENHANCED DUAL OAK-D S2 ROBOT CALIBRATION")
        print("="*60)
        print(f"DepthAI version: {dai.__version__}")
        print(f"OpenCV version: {cv2.__version__}")
        
        # Step 1: Detect cameras
        device_infos = self.detect_cameras()
        
        # Step 2: Measure robot offset
        board_to_robot = self.measure_robot_offset()
        
        # Step 3: Calibrate first camera
        input("\nPress Enter to start calibrating Camera 1...")
        cam1_to_board, cam1_intrinsics, cam1_distortion = self.calibrate_camera(device_infos[0], "Camera 1")
        if cam1_to_board is None:
            print("ERROR: Camera 1 calibration failed!")
            return False
        
        # Step 4: Calibrate second camera
        input("\nPress Enter to start calibrating Camera 2...")
        cam2_to_board, cam2_intrinsics, cam2_distortion = self.calibrate_camera(device_infos[1], "Camera 2")
        if cam2_to_board is None:
            print("ERROR: Camera 2 calibration failed!")
            return False
        
        # Step 5: Transform to robot coordinate system
        cam1_to_robot = board_to_robot @ cam1_to_board
        cam2_to_robot = board_to_robot @ cam2_to_board
        
        # Step 6: Validate stereo calibration
        self.validate_stereo_calibration(cam1_to_robot, cam2_to_robot, device_infos)
        
        # Step 7: Save calibration with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        calibration_data = {
            'timestamp': datetime.now().isoformat(),
            'calibration_id': timestamp,
            'camera_ids': {
                'camera1': device_infos[0].getMxId(),
                'camera2': device_infos[1].getMxId()
            },
            'camera_configs': [
                {
                    'mx_id': device_infos[0].getMxId(),
                    'intrinsics': cam1_intrinsics.tolist(),
                    'distortion': cam1_distortion.tolist(),
                    'extrinsics': cam1_to_robot.tolist()
                },
                {
                    'mx_id': device_infos[1].getMxId(),
                    'intrinsics': cam2_intrinsics.tolist(),
                    'distortion': cam2_distortion.tolist(),
                    'extrinsics': cam2_to_robot.tolist()
                }
            ],
            'board_params': {
                'squares_x': self.board_squares_x,
                'squares_y': self.board_squares_y,
                'square_length': self.square_length,
                'marker_length': self.marker_length
            },
            'board_to_robot_offset': board_to_robot.tolist(),
            'calibration_params': {
                'min_corners_per_frame': self.min_corners_per_frame,
                'max_reprojection_error': self.max_reprojection_error,
                'target_frames': self.target_frames
            }
        }
        
        # Save with timestamp
        json_filename = f'camera_calibration_oak_{timestamp}.json'
        npz_filename = f'camera_calibration_oak_{timestamp}.npz'
        
        with open(json_filename, 'w') as f:
            json.dump(calibration_data, f, indent=2)
        
        # Also save latest (for easy access)
        with open('camera_calibration_oak_latest.json', 'w') as f:
            json.dump(calibration_data, f, indent=2)
        
        # Save numpy arrays
        np.savez(npz_filename,
                 cam1_to_robot=cam1_to_robot,
                 cam2_to_robot=cam2_to_robot,
                 cam1_intrinsics=cam1_intrinsics,
                 cam2_intrinsics=cam2_intrinsics,
                 cam1_distortion=cam1_distortion,
                 cam2_distortion=cam2_distortion)
        
        # Also save as latest
        np.savez('camera_calibration_oak_latest.npz',
                 cam1_to_robot=cam1_to_robot,
                 cam2_to_robot=cam2_to_robot,
                 cam1_intrinsics=cam1_intrinsics,
                 cam2_intrinsics=cam2_intrinsics,
                 cam1_distortion=cam1_distortion,
                 cam2_distortion=cam2_distortion)
        
        print("\n" + "="*60)
        print("    CALIBRATION COMPLETE!")
        print("="*60)
        print("\nSaved to:")
        print(f"  - {json_filename} (timestamped)")
        print(f"  - camera_calibration_oak_latest.json (latest)")
        print(f"  - {npz_filename} (numpy arrays)")
        print(f"  - camera_calibration_oak_latest.npz (latest numpy)")
        
        print("\n=== COPY THIS TO YOUR CODE ===")
        print(f"""
import numpy as np

# Camera MxIds
CAMERA_1_MXID = "{device_infos[0].getMxId()}"
CAMERA_2_MXID = "{device_infos[1].getMxId()}"

# Camera transforms (robot base is origin)
cam1_to_robot = np.array({cam1_to_robot.tolist()})

cam2_to_robot = np.array({cam2_to_robot.tolist()})

# Camera intrinsics (for reference)
cam1_intrinsics = np.array({cam1_intrinsics.tolist()})
cam2_intrinsics = np.array({cam2_intrinsics.tolist()})

camera_configs = {{
    CAMERA_1_MXID: {{
        'intrinsics': cam1_intrinsics,
        'extrinsics': cam1_to_robot
    }},
    CAMERA_2_MXID: {{
        'intrinsics': cam2_intrinsics,
        'extrinsics': cam2_to_robot
    }}
}}
        """)
        
        return True

if __name__ == "__main__":
    calibrator = OAKRobotCalibrator()
    success = calibrator.run_calibration()
    
    if success:
        print("\n✓ Calibration successful! OAK-D cameras are now calibrated to robot frame.")
        print("  Use the latest calibration files for your application.")
    else:
        print("\n✗ Calibration failed. Please check setup and try again.")