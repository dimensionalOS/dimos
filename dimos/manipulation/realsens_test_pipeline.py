#!/usr/bin/env python3
"""
test_pipeline_with_realsense.py - Test ManipulationPipeline with RealSense D435 cameras
Based on the DepthAI test structure but adapted for RealSense
"""

import sys
import time
import queue
import threading
import cv2
import numpy as np
import pyrealsense2 as rs
from manip_aio_pipeline_depthai import ManipulationPipeline

#print(f"PyRealSense2 version: {rs.__version__}")
print("Testing ManipulationPipeline with RealSense D435\n")


def create_realsense_pipeline(device_index=0, serial_number=None):
    """
    Create RealSense pipeline using PROVEN working configuration.
    Returns: (pipeline, align, intrinsics_array, device_serial)
    """
    ctx = rs.context()
    devices = ctx.query_devices()
    
    if len(devices) == 0:
        raise RuntimeError("No RealSense devices found")
    
    # Select device by index or serial
    if serial_number:
        device = None
        for d in devices:
            if d.get_info(rs.camera_info.serial_number) == serial_number:
                device = d
                break
        if not device:
            raise ValueError(f"Device with serial {serial_number} not found")
    else:
        if device_index >= len(devices):
            raise ValueError(f"Device index {device_index} not found")
        device = devices[device_index]
    
    serial = device.get_info(rs.camera_info.serial_number)
    print(f"Initializing RealSense {serial}")
    
    # STEP 1: Hardware reset (CRITICAL!)
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
            pass  # Expected during warmup
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


def test_pipeline_streaming():
    """Test pipeline with streaming from RealSense cameras"""
    
    print("=" * 60)
    print("TESTING MANIPULATION PIPELINE WITH REALSENSE STREAMING")
    print("=" * 60)
    
    # Camera transformations (same as your DepthAI setup)
    cam1_to_robot = np.array([
        [-0.53421983586013, 0.28566181028607796, -0.7956170543154898, 0.74],
        [0.43909663767729723, 0.8980159691658256, 0.027594599175484902, -0.65],
        [-0.7223595432705714, 0.33461119118649363, 0.6051710840569698, 0.52],
        [0.0, 0.0, 0.0, 1.0]
    ])
    
    cam2_to_robot = np.array([
        [0.37719678033860765, 0.8128571754461661, -0.4438308250086612, -0.09803426042385065], 
        [0.909480284876999, -0.4155789527032386, 0.011821399668918736, 0.5418114101741569], 
        [0.17483763988981624, 0.4081143790602827, 0.8960326184252441, 0.305856197441436], 
        [0.0, 0.0, 0.0, 1.0]
    ])
    
    # Check available devices
    ctx = rs.context()
    devices = ctx.query_devices()
    num_devices = len(devices)
    
    print(f"\nFound {num_devices} RealSense device(s):")
    for i, device in enumerate(devices):
        serial = device.get_info(rs.camera_info.serial_number)
        name = device.get_info(rs.camera_info.name)
        print(f"  Device {i}: {name} ({serial})")
    
    # Determine single or dual camera mode
    use_dual_cameras = num_devices >= 2
    
    if use_dual_cameras:
        print("\n✓ Using DUAL camera mode")
    else:
        print("\n⚠️ Only 1 camera found, using SINGLE camera mode")
    
    try:
        # Initialize cameras
        pipelines = []
        aligns = []
        camera_configs = []
        
        # Camera 1 (always present)
        print("\nInitializing Camera 1...")
        pipeline1, align1, intrinsics1, serial1 = create_realsense_pipeline(0)
        pipelines.append(pipeline1)
        aligns.append(align1)
        camera_configs.append({
            "camera_id": 0,
            "intrinsics": intrinsics1,
            "extrinsics": cam1_to_robot,
        })
        
        # Camera 2 (if available)
        if use_dual_cameras:
            print("\nInitializing Camera 2...")
            pipeline2, align2, intrinsics2, serial2 = create_realsense_pipeline(1)
            pipelines.append(pipeline2)
            aligns.append(align2)
            camera_configs.append({
                "camera_id": 1,
                "intrinsics": intrinsics2,
                "extrinsics": cam2_to_robot,
            })
        
        # Initialize manipulation pipeline
        print("\nInitializing ManipulationPipeline...")
        manip_pipeline = ManipulationPipeline(
            camera_configs=camera_configs,
            min_confidence=0.4,
            enable_grasp_generation=True,
            grasp_server_url="ws://13.59.77.54:8000/ws/grasp",
            enable_segmentation=True,
        )
        
        print("✓ Pipeline initialized")
        
        # Create streaming pipeline (adapt for RealSense)
        print("\nCreating streaming pipeline...")
        
        # For RealSense, we'll pass the pipeline/align pairs
        camera_inputs = [(p, a) for p, a in zip(pipelines, aligns)]
        output_streams = manip_pipeline.create_pipeline(
            camera_inputs, 
            input_type="realsense"  # Specify RealSense input
        )
        print("✓ Streaming pipeline created")
        
        # Create visualization windows
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
        
        # Setup visualization queues
        detection_queue = queue.Queue(maxsize=2)
        pointcloud_queue = queue.Queue(maxsize=2)
        background_queue = queue.Queue(maxsize=2)
        grasp_queue = queue.Queue(maxsize=2)
        
        first_frame_received = [False]
        latest_objects = [None]
        latest_time = [None]
        frame_counter = [0]
        
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
                grasp_count = sum(len(obj.get('grasps', [])) for obj in objects)
                if grasp_count > 0:
                    print(f"  → {grasp_count} total grasps available")
        
        def update_time(t):
            latest_time[0] = t
        
        # Subscribe to streams
        print("\nSubscribing to output streams...")
        output_streams["detection_viz"].subscribe(on_next=update_detection)
        output_streams["pointcloud_viz"].subscribe(on_next=update_pointcloud)
        output_streams["misc_pointcloud_viz"].subscribe(on_next=update_background)
        output_streams["grasp_overlay"].subscribe(on_next=update_grasps)
        output_streams["all_objects"].subscribe(on_next=update_objects)
        output_streams["processing_time"].subscribe(on_next=update_time)
        
        print("\n" + "=" * 60)
        print("PIPELINE RUNNING")
        print("=" * 60)
        print(f"\nMode: {'DUAL' if use_dual_cameras else 'SINGLE'} camera")
        print("Visualization windows should show live video")
        print("Press 'q' to quit\n")
        
        # Main visualization loop
        loop_counter = 0
        try:
            while True:
                # Update detection window
                try:
                    img = detection_queue.get_nowait()
                    bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
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
                if loop_counter % 300 == 0:  # Every ~10 seconds at 30fps
                    if latest_objects[0] is not None:
                        print(f"Objects: {len(latest_objects[0])}")
                    if latest_time[0] is not None:
                        print(f"Processing time: {latest_time[0]:.3f}s")
                
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
        for pipeline in pipelines:
            pipeline.stop()
        cv2.destroyAllWindows()
        print("Done!")


def test_pipeline_single_frame():
    """Test pipeline with single frame processing (no streaming)"""
    
    print("=" * 60)
    print("TESTING PIPELINE WITH SINGLE FRAME")
    print("=" * 60)
    
    # Camera configurations (same as streaming)
    cam1_to_robot = np.array([
        [-0.53421983586013, 0.28566181028607796, -0.7956170543154898, 0.74],
        [0.43909663767729723, 0.8980159691658256, 0.027594599175484902, -0.65],
        [-0.7223595432705714, 0.33461119118649363, 0.6051710840569698, 0.52],
        [0.0, 0.0, 0.0, 1.0]
    ])
    
    cam2_to_robot = np.eye(4)  # Identity for single camera if only one available
    
    # Check devices
    ctx = rs.context()
    devices = ctx.query_devices()
    num_devices = len(devices)
    
    if num_devices == 0:
        print("ERROR: No RealSense devices found")
        return
    
    use_dual = num_devices >= 2
    print(f"Using {'dual' if use_dual else 'single'} camera mode")
    
    try:
        # Initialize cameras
        pipelines = []
        aligns = []
        camera_configs = []
        
        # Camera 1
        pipeline1, align1, intrinsics1, _ = create_realsense_pipeline(0)
        pipelines.append(pipeline1)
        aligns.append(align1)
        camera_configs.append({
            "camera_id": 0,
            "intrinsics": intrinsics1,
            "extrinsics": cam1_to_robot,
        })
        
        # Camera 2 if available
        if use_dual:
            pipeline2, align2, intrinsics2, _ = create_realsense_pipeline(1)
            pipelines.append(pipeline2)
            aligns.append(align2)
            camera_configs.append({
                "camera_id": 1,
                "intrinsics": intrinsics2,
                "extrinsics": cam2_to_robot,
            })
        
        # Initialize pipeline
        print("\nInitializing pipeline...")
        pipeline = ManipulationPipeline(
            camera_configs=camera_configs,
            enable_grasp_generation=False,
        )
        
        # Capture frames
        print("\nCapturing frames...")
        rgb_images = []
        depth_images = []
        
        for i, (rs_pipeline, align) in enumerate(zip(pipelines, aligns)):
            # Get aligned frames
            frames = rs_pipeline.wait_for_frames(timeout_ms=10000)
            aligned = align.process(frames)
            
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            
            if not color_frame or not depth_frame:
                print(f"ERROR: Failed to get frames from camera {i}")
                return
            
            # Convert to numpy
            rgb = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data()).astype(np.float32) / 1000.0
            
            rgb_images.append(rgb)
            depth_images.append(depth)
            
            print(f"  Camera {i}: RGB {rgb.shape}, Depth {depth.shape}")
        
        # Process single frame
        print("\nProcessing frame...")
        results = pipeline.process_single_frame(rgb_images, depth_images)
        
        # Display results
        print("\n" + "=" * 60)
        print("RESULTS")
        print("=" * 60)
        print(f"Processing time: {results['processing_time']:.3f}s")
        print(f"Detected objects: {len(results.get('all_objects', []))}")
        
        if results.get("full_pointcloud"):
            print(f"Point cloud: {len(results['full_pointcloud'].points)} points")
        
        # Show visualizations
        if results.get("pointcloud_viz") is not None:
            viz = cv2.cvtColor(results["pointcloud_viz"], cv2.COLOR_RGB2BGR)
            cv2.imshow("Point Cloud", viz)
            print("\nShowing visualization. Press any key to close.")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        
        pipeline.cleanup()
        print("\n✓ Test complete")
        
    finally:
        for rs_pipeline in pipelines:
            rs_pipeline.stop()


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Test ManipulationPipeline with RealSense")
    parser.add_argument(
        "--mode",
        choices=["stream", "single"],
        default="stream",
        help="Test mode: stream (continuous) or single (one frame)",
    )
    
    args = parser.parse_args()
    
    if args.mode == "stream":
        test_pipeline_streaming()
    else:
        test_pipeline_single_frame()