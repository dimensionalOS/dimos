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
        x = (387 + 448) // 2  # = 417
        y = (160 + 278) // 2  # = 219
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


def test_vlm_manipulation():
    """Test VLM-guided manipulation with exact visualization from working test."""
    
    print("=" * 60)
    print("TESTING VLM MANIPULATION WITH REALSENSE")
    print("=" * 60)
    
    # Camera transformation
    cam_to_robot = np.array([
        [-0.53421983586013, 0.28566181028607796, -0.7956170543154898, 0.74],
        [0.43909663767729723, 0.8980159691658256, 0.027594599175484902, -0.65],
        [-0.7223595432705714, 0.33461119118649363, 0.6051710840569698, 0.52],
        [0.0, 0.0, 0.0, 1.0]
    ])
    
    try:
        # Initialize camera
        print("\nInitializing Camera...")
        rs_pipeline, align, intrinsics, serial = create_realsense_pipeline(0)
        
        camera_configs = [{
            "camera_id": 0,
            "intrinsics": intrinsics,
            "extrinsics": cam_to_robot,
        }]
        
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
            if frame_counter[0] % 30 == 0:
                print(f"\nFrame {frame_counter[0]}: {len(objects)} objects detected")
        
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
                        
                        if task_plan:
                            print(f"  ✓ Found: {task_plan.target_object.get('class_name', 'unknown')}")
                            print(f"  ✓ Pixel: {task_plan.pixel_location}")
                            print(f"  ✓ Confidence: {task_plan.confidence:.2f}")
                            
                            # DEBUG: Check if grasps exist
                            print(f"  🔍 DEBUG: Object has {len(task_plan.target_object.get('grasps', []))} grasps")
                            
                            if task_plan.selected_grasp:
                                grasp = task_plan.selected_grasp
                                print(f"\n📍 END-EFFECTOR COORDINATES:")
                                print(f"  Position: {grasp['translation']}")
                                print(f"  Gripper: {grasp.get('width', 0.04)*1000:.1f}mm")
                                
                                # DEBUG: Check grasp details
                                print(f"  🔍 DEBUG: Grasp keys: {grasp.keys()}")
                                
                                # Queue result for display
                                vlm_task_queue.put({
                                    'type': 'pick',
                                    'object': task_plan.target_object.get('class_name'),
                                    'position': grasp['translation'],
                                    'pixel': task_plan.pixel_location
                                })
                            else:
                                print("  ❌ DEBUG: No selected grasp found!")
                        else:
                            print("  ✗ VLM failed to identify target")
                    else:
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
                if loop_counter % 300 == 0:  # Every ~10 seconds
                    if latest_objects[0] is not None:
                        print(f"[Status] Objects: {len(latest_objects[0])}, Time: {latest_time[0]:.3f}s")
                
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
        if 'manip_pipeline' in locals():
            manip_pipeline.stop()
        if 'rs_pipeline' in locals():
            rs_pipeline.stop()
        cv2.destroyAllWindows()
        print("Done!")


if __name__ == "__main__":
    test_vlm_manipulation()