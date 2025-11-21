#!/usr/bin/env python3
import sys
import time
import queue
import threading
import cv2
import numpy as np
import pyrealsense2 as rs
from manip_aio_pipeline_depthai import ManipulationPipeline
from Task_Planner import VLMTaskPlanner
from scipy.spatial.transform import Rotation  
from dimos import core
from dimos.hardware.manipulators.xarm.xarm_driver import XArmDriver
from dimos.manipulation.control.cartesian_motion_controller import CartesianMotionController
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.JointCommand import JointCommand
from dimos.msgs.sensor_msgs.RobotState import RobotState
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.geometry_msgs.Quaternion import Quaternion

# Import VLM model - adjust path as needed
# Comment out the real import and force the mock
"""
try:
    from dimos.models.vl.qwen import QwenVlModel
except:
    print("Warning: Could not import QwenVlModel, using mock")
    class QwenVlModel:
        def query(self, img, prompt):
            # Mock response for testing
            return "PIXEL: [320, 240]\nCONFIDENCE: 0.8\nREASONING: Test object"
"""

# Force use of mock for testing
print("Using mock VLM for testing")
class QwenVlModel:
    def query(self, img, prompt):
        # Target object 0 (cup) which is at bbox (387,160)-(448,278)
        # Pick the center of the cup
        x = 469  # = 417
        y = 177 # = 219
        return f"PIXEL: [{x}, {y}]\nCONFIDENCE: 0.8\nREASONING: Targeting cup (object 0)"

print("=" * 60)
print("VLM MANIPULATION TEST WITH REALSENSE")
print("=" * 60)

def create_realsense_pipeline(device_index=0):
    """Create RealSense pipeline using PROVEN working configuration."""
    ctx = rs.context()
    devices = ctx.query_devices()
    
    if device_index >= len(devices):
        raise ValueError(f"Device index {device_index} not found")
    
    device = devices[device_index]
    serial = device.get_info(rs.camera_info.serial_number)
    
    print(f"Initializing RealSense {serial}")
    
    # STEP 1: Hardware reset
    print(f"  1. Hardware reset...")
    device.hardware_reset()
    time.sleep(3)
    
    # STEP 2: Get fresh context after reset
    ctx = rs.context()
    devices = ctx.query_devices()
    for d in devices:
        if d.get_info(rs.camera_info.serial_number) == serial:
            device = d
            break
    
    # STEP 3: Create pipeline with proven config
    pipeline = rs.pipeline()
    config = rs.config()
    
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 6)  # 6 FPS works!
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
    
    # STEP 4: Start pipeline
    print(f"  2. Starting pipeline (640x480 @ 6fps)...")
    profile = pipeline.start(config)
    
    # STEP 5: Create align object
    align = rs.align(rs.stream.color)
    
    # STEP 6: CRITICAL warmup phase
    print(f"  3. Warming up (essential for D435)...")
    for i in range(50):
        try:
            pipeline.wait_for_frames(timeout_ms=500)
        except:
            pass
        if i % 10 == 0:
            print(f"     Warmup {i}/50")
    
    # STEP 7: Validate and get intrinsics
    print(f"  4. Validating stream...")
    frames = pipeline.wait_for_frames(timeout_ms=10000)
    aligned = align.process(frames)
    color_frame = aligned.get_color_frame()
    
    if not color_frame:
        raise RuntimeError("Could not get color frame after warmup")
    
    # Get intrinsics
    intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
    intrinsics_array = [intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy]
    
    print(f"  ✓ Ready! Intrinsics: fx={intrinsics.fx:.1f}, fy={intrinsics.fy:.1f}")
    
    return pipeline, align, intrinsics_array, serial

def rotation_matrix_to_quaternion(rot_matrix):
    """Convert 3x3 rotation matrix to quaternion (x,y,z,w format)"""
    r = Rotation.from_matrix(rot_matrix)
    quat = r.as_quat()  # Returns [x, y, z, w]
    return quat

def create_grasp_pose(grasp_dict, apply_transform=True):
    """Convert grasp dictionary to Pose object"""
    # Get position and rotation from grasp
    position = grasp_dict['translation']
    rot_matrix = np.array(grasp_dict['rotation_matrix'])

    
    if apply_transform:
        # Apply camera to robot transformation
        transform = np.array([
            [ 0.58097254, -0.02551784, -0.81352305,  1.25596331],
            [ 0.78601229, -0.24191228,  0.56891398, -0.8309276 ],
            [ -0.21131867, -0.96996252, -0.12048706,  0.13877683],
            [ 0.0,         0.0,         0.0,         1.0       ]
        ])
        
        # Transform position
        pos_homo = np.append(position, 1)
        position = (transform @ pos_homo)[:3]
        
        # Transform rotation
        #rot_matrix = transform[:3, :3] @ rot_matrix
        # Try using the inverse transform for rotation
        #rot_matrix = np.linalg.inv(transform[:3, :3]) @ rot_matrix
        print(rot_matrix,'rot matrix')
    
    # Convert rotation matrix to quaternion
    #quat = rotation_matrix_to_quaternion(rot_matrix)
    quat = [0, 0, 0, 1]
    print (quat, 'quat')    
    euler = Rotation.from_quat(quat).as_euler('xyz', degrees=True)
    print(f"Grasp wants euler angles: roll={euler[0]:.1f}°, pitch={euler[1]:.1f}°, yaw={euler[2]:.1f}°")
    # Create PoseStamped - it IS a Pose, not HAS a Pose
    pose_msg = PoseStamped(
        ts=time.time(),
        frame_id="base",
        position=[position[0], position[1], position[2]],
        orientation=[quat[0], quat[1], quat[2], quat[3]]
    )
    
    return pose_msg


def test_vlm_manipulation():
    """Test VLM-guided manipulation with exact visualization from working test."""
    
    print("=" * 60)
    print("TESTING VLM MANIPULATION WITH REALSENSE")
    print("=" * 60)

    # Initialize DiMoS cluster for motion control
    print("\nStarting DiMoS cluster for motion control...")
    dimos = core.start(1)
    
    # Deploy xArm driver
    print("Deploying xArm driver...")
    arm_driver = dimos.deploy(
        XArmDriver,
        ip_address="192.168.1.210",  # UPDATE WITH YOUR ROBOT IP
        xarm_type="xarm6",
        report_type="dev",
        enable_on_start=True,
    )
    
    # Set up driver transports
    arm_driver.joint_state.transport = core.LCMTransport("/xarm/joint_states", JointState)
    arm_driver.robot_state.transport = core.LCMTransport("/xarm/robot_state", RobotState)
    arm_driver.joint_position_command.transport = core.LCMTransport(
        "/xarm/joint_position_command", JointCommand
    )
    arm_driver.joint_velocity_command.transport = core.LCMTransport(
        "/xarm/joint_velocity_command", JointCommand
    )
    
    print("Starting xArm driver...")
    arm_driver.start()

    # ============ ADD GRIPPER ENABLE HERE ============
    print("\nInitializing gripper...")
    code, msg = arm_driver.set_gripper_enable(1)
    if code == 0:
        print(f"✓ Gripper enabled: {msg}")
    else:
        print(f"⚠ Warning: Gripper enable failed: {msg}")
    time.sleep(0.5)
    
    # Optional: Open gripper to known state
    print("Opening gripper to start position...")
    code, msg = arm_driver.set_gripper_position(850, wait=True)
    print(f"Gripper initialized: {msg}")
    # ==================================================
    
    # Deploy Cartesian motion controller
    print("Deploying Cartesian motion controller...")
    motion_controller = dimos.deploy(
        CartesianMotionController,
        arm_driver=arm_driver,
        control_frequency=20.0,
        position_kp=1.0,
        position_kd=0.1,
        orientation_kp=2.0,
        orientation_kd=0.2,
        max_linear_velocity=0.15,
        max_angular_velocity=0.8,
        position_tolerance=0.002,
        orientation_tolerance=0.02,
        velocity_control_mode=True,
    )
    
    # Set up controller transports
    motion_controller.joint_state.transport = core.LCMTransport("/xarm/joint_states", JointState)
    motion_controller.robot_state.transport = core.LCMTransport("/xarm/robot_state", RobotState)
    motion_controller.joint_position_command.transport = core.LCMTransport(
        "/xarm/joint_position_command", JointCommand
    )
    motion_controller.target_pose.transport = core.LCMTransport("/target_pose", PoseStamped)
    motion_controller.current_pose.transport = core.LCMTransport("/xarm/current_pose", PoseStamped)
    
    print("Starting motion controller...")
    motion_controller.start()
    print("✓ Motion control system ready")
    
    # Camera transformation
    '''cam_to_robot = np.array([
        [-0.53421983586013, 0.28566181028607796, -0.7956170543154898, 0.98],
        [0.43909663767729723, 0.8980159691658256, 0.027594599175484902, -0.66],
        [-0.7223595432705714, 0.33461119118649363, 0.6051710840569698, 0.62],
        [0.0, 0.0, 0.0, 1.0]
    ])'''

    cam_to_robot = np.array([
        [-0.47055580643315087, -0.37561388946893404, -0.7984306100532885, 0.98], 
        [-0.6600291194015566, 0.750377866382916, 0.03598081689773558, -0.66], 
        [-0.5856097630453686, -0.5439184347681572, 0.6010107667465743, 0.52], 
        [0.0, 0.0, 0.0, 1.0]
    ])

    #cam_to_robot = np.identity(4)

    #print("🔍 DEBUG: Using cam_to_robot matrix:")
    #print(cam_to_robot)
    
    try:
        # Initialize camera
        print("\nInitializing Camera...")
        rs_pipeline, align, intrinsics, serial = create_realsense_pipeline(0)
        
        camera_configs = [{
            "camera_id": 0,
            "intrinsics": intrinsics,
            "extrinsics": cam_to_robot,
        }]
        
        #print(f"🔍 DEBUG: camera_configs[0]['extrinsics']:\n{camera_configs[0]['extrinsics']}")
        # Initialize manipulation pipeline WITHOUT automatic grasps
        print("\nInitializing ManipulationPipeline...")
        manip_pipeline = ManipulationPipeline(
            camera_configs=camera_configs,
            min_confidence=0.4,
            enable_grasp_generation=True,  # Available but NOT automatic
            grasp_server_url="ws://13.59.77.54:8000/ws/grasp",
            enable_segmentation=True,
        )
        
        # Initialize VLM Task Planner
        print("\nInitializing VLM Task Planner...")
        vlm_planner = VLMTaskPlanner(
            manipulation_pipeline=manip_pipeline,
            vlm_model=QwenVlModel(),
            camera_id=0
        )
        
        print("✓ Pipeline initialized")
        
        # Create streaming pipeline
        print("\nCreating streaming pipeline...")
        camera_inputs = [(rs_pipeline, align)]
        output_streams = manip_pipeline.create_pipeline(
            camera_inputs, 
            input_type="realsense"
        )
        print("✓ Streaming pipeline created")
        
        # Create 4 visualization windows (EXACT same as working test)
        print("\nCreating visualization windows...")
        cv2.namedWindow("Detection", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Point Cloud Overlay", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Background Clusters", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Grasp Overlay", cv2.WINDOW_NORMAL)
        
        # Show placeholder images
        placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(placeholder, "Waiting for frames...", (150, 240),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow("Detection", placeholder)
        cv2.imshow("Point Cloud Overlay", placeholder)
        cv2.imshow("Background Clusters", placeholder)
        cv2.imshow("Grasp Overlay", placeholder)
        cv2.waitKey(1)
        print("✓ Windows created")
        
        # Setup visualization queues (EXACT same as working test)
        detection_queue = queue.Queue(maxsize=2)
        pointcloud_queue = queue.Queue(maxsize=2)
        background_queue = queue.Queue(maxsize=2)
        grasp_queue = queue.Queue(maxsize=2)
        
        first_frame_received = [False]
        latest_objects = [None]
        latest_time = [None]
        frame_counter = [0]
        
        # VLM task results
        vlm_task_queue = queue.Queue(maxsize=5)
        
        def update_detection(img):
            if img is not None:
                try:
                    if detection_queue.full():
                        detection_queue.get_nowait()
                    detection_queue.put_nowait(img)
                    if not first_frame_received[0]:
                        print("✓ First detection frame received")
                        first_frame_received[0] = True
                except:
                    pass
        
        def update_pointcloud(img):
            if img is not None:
                try:
                    if pointcloud_queue.full():
                        pointcloud_queue.get_nowait()
                    pointcloud_queue.put_nowait(img)
                except:
                    pass
        
        def update_background(img):
            if img is not None:
                try:
                    if background_queue.full():
                        background_queue.get_nowait()
                    background_queue.put_nowait(img)
                except:
                    pass
        
        def update_grasps(img):
            if img is not None:
                try:
                    if grasp_queue.full():
                        grasp_queue.get_nowait()
                    grasp_queue.put_nowait(img)
                except:
                    pass
        
        def update_objects(objects):
            latest_objects[0] = objects
            frame_counter[0] += 1
            #if frame_counter[0] % 30 == 0:
                #print(f"\nFrame {frame_counter[0]}: {len(objects)} objects detected")
        
        def update_time(t):
            latest_time[0] = t
        
        # Subscribe to streams (EXACT same as working test)
        print("\nSubscribing to output streams...")
        output_streams["detection_viz"].subscribe(on_next=update_detection)
        output_streams["pointcloud_viz"].subscribe(on_next=update_pointcloud)
        output_streams["misc_pointcloud_viz"].subscribe(on_next=update_background)
        output_streams["grasp_overlay"].subscribe(on_next=update_grasps)
        output_streams["all_objects"].subscribe(on_next=update_objects)
        output_streams["processing_time"].subscribe(on_next=update_time)
        
        print("\n" + "=" * 60)
        print("PIPELINE RUNNING - 4 Windows Active")
        print("=" * 60)
        print("\n📝 VLM COMMANDS (Type in terminal):")
        print("  'pick <object>' - e.g., 'pick the red cup'")
        print("  'status' - Show detection status")
        print("  'q' - Quit")
        print("\nWaiting for pipeline to initialize...")
        
        # Wait for first frames
        while not first_frame_received[0]:
            time.sleep(0.1)
        print("✓ Pipeline ready! You can now enter commands.\n")
        
        def execute_hardcoded_commands():
            """Execute hardcoded VLM commands for testing."""
            print("🔍 DEBUG: Starting hardcoded commands thread")
            
            # HARDCODE YOUR COMMANDS HERE
            test_commands = [
                ("pick the red object", 5.0),
            ]
            
            # Wait a bit before starting
            print("🔍 DEBUG: Waiting 3 seconds...")
            time.sleep(3)
            
            print("🔍 DEBUG: Starting command execution")
            
            # Execute each command
            for instruction, delay in test_commands:
                try:
                    print(f"\n{'='*50}")
                    print(f"🤖 AUTO-EXECUTING: '{instruction}'")
                    print('='*50)
                    
                    # Get current RGB image - ONLY hold lock briefly to copy
                    print("🔍 DEBUG: Acquiring lock to get image...")
                    rgb_image = None
                    with manip_pipeline.lock:
                        print("🔍 DEBUG: Lock acquired")
                        if manip_pipeline.latest_rgb_images[0] is not None:
                            rgb_image = manip_pipeline.latest_rgb_images[0].copy()
                            print(f"🔍 DEBUG: Got RGB image shape {rgb_image.shape}")
                    print("🔍 DEBUG: Lock released")
                    
                    # NOW call VLM - OUTSIDE the lock to avoid deadlock
                    if rgb_image is not None:
                        # Execute VLM pick task
                        print("  1. VLM identifying target...")
                        print("🔍 DEBUG: Calling vlm_planner.plan_pick_task...")
                        task_plan = vlm_planner.plan_pick_task(
                            instruction=instruction,
                            rgb_image=rgb_image,
                            retry_on_failure=True
                        )
                        print("🔍 DEBUG: VLM call completed")
                        print(f"🔍 DEBUG: task_plan is: {task_plan}")
                        
                        if task_plan:  # Check if task_plan exists
                            if task_plan and task_plan.selected_grasp:
                                grasp = task_plan.selected_grasp
                                grasp_pose = create_grasp_pose(grasp, apply_transform=True)
                                
                                print(f"\n📍 GRASP POSE (Robot Frame):")
                                print(f"  Position: x={grasp_pose.x:.3f}, y={grasp_pose.y:.3f}, z={grasp_pose.z:.3f}")
                                
                                # GET CURRENT ORIENTATION - DON'T CHANGE IT!
                                current = motion_controller.get_current_pose()
                                if current:
                                    keep_orientation = [
                                        current.orientation.x,
                                        current.orientation.y, 
                                        current.orientation.z,
                                        current.orientation.w
                                    ]
                                else:
                                    keep_orientation = [0, 0, 0, 1]  # Fallback
                                
                                print("\n🤖 EXECUTING GRASP MOTION (position only, keeping orientation):")
                                
                                # Move to grasp position WITHOUT changing orientation
                                print("  1. Moving to grasp position...")
                                position = [grasp_pose.x - 0.05, grasp_pose.y + 0.05, grasp_pose.z + 0.015]
                                motion_controller.set_target_pose(position, keep_orientation, grasp_pose.frame_id)
                                
                                start_time = time.time()
                                while True:
                                    current = motion_controller.get_current_pose()
                                    if current:
                                        dist = np.sqrt((current.x - position[0])**2 + 
                                                    (current.y - position[1])**2 + 
                                                    (current.z - position[2])**2)
                                        if dist < 0.01:  # Within 5mm is good enough
                                            # STOP THE CONTROLLER FROM TRACKING!
                                            motion_controller.clear_target() 
                                            print(f"     ✓ Close enough ({dist*1000:.1f}mm), stopping")
                                            break
                                    
                                    if time.time() - start_time > 9:  # Safety timeout
                                        motion_controller.clear_target()
                                        print("     Timeout reached, stopping")
                                        break
                                    
                                    time.sleep(0.1)
                                
                                # Close gripper
                                print("  2. Closing gripper...")
                                code, msg = arm_driver.set_gripper_position(
                                    position=50,   # 850 = fully open
                                    wait=True,
                                    speed=1000
                                )
                                time.sleep(1)
                                print("     ✓ Object grasped")
                                
                                # Lift up (still keeping same orientation)
                                print("  3. Lifting object...")
                                lift_position = [0.45, 0, 0.65]
                                motion_controller.set_target_pose(lift_position, keep_orientation, grasp_pose.frame_id)
                                tart_time = time.time()
                                if time.time() - start_time > 15:  # Safety timeout
                                    motion_controller.clear_target()
                                    print("     Timeout reached, stopping")
                                    break
                                
                                while not motion_controller.is_converged():
                                    time.sleep(0.1)
                                print("     ✓ Object lifted")
                                
                                print("\n✅ GRASP EXECUTED SUCCESSFULLY!")
                                
                                # Queue result for display
                                vlm_task_queue.put({
                                    'type': 'pick',
                                    'object': task_plan.target_object.get('class_name'),
                                    'position': grasp['translation'],
                                    'pixel': task_plan.pixel_location
                                })
                            else:  # This else is for if task_plan.selected_grasp
                                print("  ❌ DEBUG: No selected grasp found!")
                        else:  # This else is for if task_plan
                            print("  ✗ VLM failed to identify target")
                    else:  # This else is for if rgb_image is not None
                        print("  ✗ No camera data available")
                    
                    # Wait before next command
                    print(f"\nWaiting {delay} seconds before next command...")
                    time.sleep(delay)
                    
                except Exception as e:
                    print(f"❌ ERROR in execute_hardcoded_commands: {e}")
                    import traceback
                    traceback.print_exc()
            
            print("\n✅ All hardcoded commands executed!")
        # Start input thread (this should be at the same indentation level as the function definition)
        input_thread_handle = threading.Thread(target=execute_hardcoded_commands, daemon=True)
        input_thread_handle.start()
        print(f"🔍 DEBUG: Thread started, is_alive: {input_thread_handle.is_alive()}")  # ADD THIS
        # Main visualization loop (EXACT same as working test)
        loop_counter = 0
        try:
            while True:
                # Update detection window
                try:
                    img = detection_queue.get_nowait()
                    bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    
                    # Add VLM target if available
                    try:
                        task = vlm_task_queue.get_nowait()
                        if task and 'pixel' in task:
                            x, y = task['pixel']
                            cv2.drawMarker(bgr, (x, y), (0, 255, 0), cv2.MARKER_CROSS, 30, 3)
                            cv2.putText(bgr, f"VLM: {task['object']}", (x+10, y-10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    except queue.Empty:
                        pass
                    
                    cv2.imshow("Detection", bgr)
                except queue.Empty:
                    pass
                
                # Update pointcloud window
                try:
                    img = pointcloud_queue.get_nowait()
                    bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    cv2.imshow("Point Cloud Overlay", bgr)
                except queue.Empty:
                    pass
                
                # Update background window
                try:
                    img = background_queue.get_nowait()
                    bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    cv2.imshow("Background Clusters", bgr)
                except queue.Empty:
                    pass
                
                # Update grasp window
                try:
                    img = grasp_queue.get_nowait()
                    bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    cv2.imshow("Grasp Overlay", bgr)
                except queue.Empty:
                    pass
                
                # Print status periodically
                loop_counter += 1
                #if loop_counter % 300 == 0:  # Every ~10 seconds
                    #if latest_objects[0] is not None:
                        #print(f"[Status] Objects: {len(latest_objects[0])}, Time: {latest_time[0]:.3f}s")
                
                # Check for quit
                key = cv2.waitKey(1)
                if key == ord('q') or key == 27:
                    break
                    
        except KeyboardInterrupt:
            print("\nStopping...")
        
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        print("\nCleaning up...")
        if 'motion_controller' in locals():
            print("Stopping motion controller...")
            motion_controller.stop()
        if 'arm_driver' in locals():
            print("Stopping arm driver...")
            arm_driver.stop()
        if 'dimos' in locals():
            print("Stopping DiMoS cluster...")
            dimos.stop()
        if 'manip_pipeline' in locals():
            manip_pipeline.stop()
        if 'rs_pipeline' in locals():
            rs_pipeline.stop()
        cv2.destroyAllWindows()
        print("Done!")


if __name__ == "__main__":
    test_vlm_manipulation()