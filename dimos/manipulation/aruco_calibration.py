#!/usr/bin/env python3
"""
one_time_calibration_oak.py - ArUco calibration for dual OAK-D S2 setup
Adapted for DepthAI cameras with factory calibration data
"""

import cv2
import numpy as np
import json
import depthai as dai
import contextlib
from datetime import datetime

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
        # These are just placeholders
        self.camera_matrix = np.array([
            [500, 0, 320],
            [0, 500, 240],
            [0, 0, 1]
        ], dtype=float)
        self.dist_coeffs = np.zeros(5)
        
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
        """Create pipeline for OAK-D S2 - same as working script"""
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
        """Get robot offset from ArUco board origin"""
        print("\n=== ROBOT-TO-BOARD OFFSET ===")
        print("Measure from ArUco board ORIGIN to robot base center\n")
        
        print("Coordinate frames:")
        print("  Board: X=right, Y=back, Z=down (from camera view)")
        print("  Robot: X=forward (toward camera), Y=right, Z=up\n")
        
        print("Measuring in BOARD coordinates:")
        x_board = float(input("X offset (board frame, meters): "))
        y_board = float(input("Y offset (board frame, meters): "))
        z_board = float(input("Z offset (board frame, meters): "))
        
        # Create transformation with rotation
        # Robot frame is rotated relative to board:
        # - Robot X = -Board Y (forward faces front of board)
        # - Robot Y = -Board X (right points left of board)  
        # - Robot Z = -Board Z (up vs down)
        
        board_to_robot = np.array([
            [ 0, -1,  0,  y_board],   # Robot X = -Board Y
            [-1,  0,  0,  x_board],   # Robot Y = -Board X
            [ 0,  0,  1,  z_board],   # Robot Z = -Board Z
            [ 0,  0,  0,  1]
        ])
        
        print(f"\nBoard frame offset: ({x_board:.3f}, {y_board:.3f}, {z_board:.3f}) m")
        print(f"Transformation includes rotation between frames")
        
        return board_to_robot
    
    def calibrate_camera(self, device_info, camera_name):
        """Calibrate a single OAK-D camera to the ArUco board"""
        print(f"\n=== CALIBRATING {camera_name} ({device_info.getMxId()[:12]}) ===")
        print("Position camera to see the ArUco board clearly")
        print("Press SPACE to capture frames, 'c' to compute calibration")
        print("IMPORTANT: Click on the preview window to make sure it has focus!\n")
        
        all_charuco_corners = []
        all_charuco_ids = []
        captured_frames = 0
        target_frames = 15
        
        # Create pipeline and device
        pipeline = self.create_pipeline()
        
        with contextlib.ExitStack() as stack:
            device = stack.enter_context(
                dai.Device(pipeline, device_info, dai.UsbSpeed.SUPER)
            )
            
            # ✅ GET REAL CALIBRATION DATA FROM OAK-D
            camera_matrix, dist_coeffs = self.get_oak_intrinsics(device)
            
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
                
                # Prepare display
                display = frame.copy()
                
                if ids is not None and len(ids) > 0:
                    # Draw detected markers
                    cv2.aruco.drawDetectedMarkers(display, corners, ids)
                    
                    # Show detection stats
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
                                # Draw BIGGER coordinate axes
                                cv2.drawFrameAxes(
                                    display, camera_matrix, dist_coeffs,
                                    rvec, tvec, 0.2,  # 20cm long axes
                                    thickness=3
                                )
                                
                                # Draw circle at origin
                                origin_2d, _ = cv2.projectPoints(
                                    np.array([[0, 0, 0]], dtype=np.float32),
                                    rvec, tvec,
                                    camera_matrix, dist_coeffs
                                )
                                origin_pt = tuple(origin_2d[0][0].astype(int))
                                cv2.circle(display, origin_pt, 10, (255, 255, 0), -1)
                                cv2.putText(display, "ORIGIN", 
                                           (origin_pt[0] + 15, origin_pt[1]), 
                                           cv2.FONT_HERSHEY_SIMPLEX, 
                                           0.6, (255, 255, 0), 2)
                                
                                cv2.putText(display, "GOOD! Press SPACE to capture", 
                                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                                           0.7, (0, 255, 0), 2)
                        else:
                            cv2.putText(display, f"Need more corners (have {corner_count})", 
                                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                                       0.7, (0, 165, 255), 2)
                    else:
                        cv2.putText(display, f"Too few corners: {corner_count}/4", 
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
                
                cv2.putText(display, "Press 'c' to compute | 'q' to quit", 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.5, (200, 200, 200), 1)
                
                cv2.putText(display, "Click window first, then SPACE", 
                           (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.5, (255, 255, 0), 1)
                
                # Show the frame
                cv2.imshow(window_name, display)
                
                # Key detection
                key = cv2.waitKey(10) & 0xFF
                
                if key != 255:
                    print(f"Key pressed: {key}")
                
                if key == ord(' ') or key == 32:  # Space bar
                    if (ids is not None and charuco_corners is not None and 
                        len(charuco_corners) > 4):
                        all_charuco_corners.append(charuco_corners)
                        all_charuco_ids.append(charuco_ids)
                        captured_frames += 1
                        print(f"  ✓ Captured frame {captured_frames}/{target_frames}")
                        
                        if captured_frames >= target_frames:
                            print(f"  Target frames reached, computing calibration...")
                            break
                    else:
                        print(f"  ✗ Cannot capture - not enough corners detected")
                
                elif key == ord('c') or key == ord('C'):
                    if captured_frames >= 5:
                        print(f"  Computing with {captured_frames} frames...")
                        break
                    else:
                        print(f"  ✗ Need at least 5 frames (have {captured_frames})")
                
                elif key == ord('q') or key == 27:
                    print("  Calibration cancelled")
                    cv2.destroyAllWindows()
                    return None, None, None
        
        cv2.destroyAllWindows()
        
        # Compute calibration from captured frames
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
            print("  ERROR: Could not estimate pose from any frame!")
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
        print(f"    Translation: {avg_tvec.squeeze()}")
        
        return cam_to_board, camera_matrix, dist_coeffs
    
    def verify_calibration(self, cam1_to_robot, cam2_to_robot):
        """Quick verification of calibration results"""
        print("\n=== VERIFICATION ===")
        
        # Test point at robot base
        robot_base = np.array([0, 0, 0, 1])
        
        # Project to each camera
        cam1_point = np.linalg.inv(cam1_to_robot) @ robot_base
        cam2_point = np.linalg.inv(cam2_to_robot) @ robot_base
        
        print(f"Robot base in camera 1 frame: {cam1_point[:3]}")
        print(f"Robot base in camera 2 frame: {cam2_point[:3]}")
        
        # Extract camera positions from robot base
        cam1_pos = cam1_to_robot[:3, 3]
        cam2_pos = cam2_to_robot[:3, 3]
        
        print(f"\nCamera 1 position from robot base: X={cam1_pos[0]:.3f}m, Y={cam1_pos[1]:.3f}m, Z={cam1_pos[2]:.3f}m")
        print(f"Camera 2 position from robot base: X={cam2_pos[0]:.3f}m, Y={cam2_pos[1]:.3f}m, Z={cam2_pos[2]:.3f}m")
        
        print("\n⚠️  VERIFY: Do these camera heights match your physical setup?")
        print("   (Robot base is 7cm below table, cameras are ~20-25cm above table)")
        print("   Expected camera heights: ~0.27m and ~0.22m above robot base")
    
    def run_calibration(self):
        """Main calibration routine"""
        print("\n" + "="*50)
        print("    DUAL OAK-D S2 ROBOT CALIBRATION")
        print("="*50)
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
        
        # Step 6: Verify
        self.verify_calibration(cam1_to_robot, cam2_to_robot)
        
        # Step 7: Save calibration
        calibration_data = {
            'timestamp': datetime.now().isoformat(),
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
            'board_to_robot_offset': board_to_robot.tolist()
        }
        
        # Save JSON
        with open('camera_calibration_oak.json', 'w') as f:
            json.dump(calibration_data, f, indent=2)
        
        # Save numpy arrays
        np.savez('camera_calibration_oak.npz',
                 cam1_to_robot=cam1_to_robot,
                 cam2_to_robot=cam2_to_robot,
                 cam1_intrinsics=cam1_intrinsics,
                 cam2_intrinsics=cam2_intrinsics,
                 cam1_distortion=cam1_distortion,
                 cam2_distortion=cam2_distortion)
        
        print("\n" + "="*50)
        print("    CALIBRATION COMPLETE!")
        print("="*50)
        print("\nSaved to:")
        print("  - camera_calibration_oak.json (human readable)")
        print("  - camera_calibration_oak.npz (numpy arrays)")
        
        print("\n=== COPY THIS TO YOUR CODE ===")
        print(f"""
import numpy as np

# Camera MxIds
CAMERA_1_MXID = "{device_infos[0].getMxId()}"
CAMERA_2_MXID = "{device_infos[1].getMxId()}"

# Camera transforms (robot base is origin)
cam1_to_robot = np.array({cam1_to_robot.tolist()})

cam2_to_robot = np.array({cam2_to_robot.tolist()})

camera_configs = {{
    CAMERA_1_MXID: {{
        'intrinsics': np.array({cam1_intrinsics.tolist()}),
        'distortion': np.array({cam1_distortion.tolist()}),
        'extrinsics': cam1_to_robot
    }},
    CAMERA_2_MXID: {{
        'intrinsics': np.array({cam2_intrinsics.tolist()}),
        'distortion': np.array({cam2_distortion.tolist()}),
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
        print("  The robot base is at (0,0,0) in world coordinates.")
    else:
        print("\n✗ Calibration failed. Please try again.")