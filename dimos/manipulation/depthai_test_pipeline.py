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
test_pipeline_with_depthai.py - Test ManipulationPipeline with dual OAK-D S2 cameras
"""

import contextlib
import sys

import cv2
import depthai as dai
from manip_aio_pipeline_depthai import ManipulationPipeline
import numpy as np

print(f"DepthAI version: {dai.__version__}")
print("Testing ManipulationPipeline with Dual OAK-D S2\n")


def create_camera_pipeline():
    """Create DepthAI pipeline for OAK-D S2 with MATCHING RGB and depth sizes"""
    pipeline = dai.Pipeline()

    # Use ColorCamera nodes for both cameras
    cam_left = pipeline.create(dai.node.ColorCamera)
    cam_right = pipeline.create(dai.node.ColorCamera)

    cam_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    cam_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

    # Set resolution
    cam_left.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
    cam_right.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)

    # Configure left camera for color preview
    cam_left.setPreviewSize(640, 480)  # RGB will be 640x480
    cam_left.setInterleaved(False)
    cam_left.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    # Output color preview from left camera
    xout_color = pipeline.create(dai.node.XLinkOut)
    xout_color.setStreamName("color")
    cam_left.preview.link(xout_color.input)

    # Stereo depth using ISP output
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
    stereo.setLeftRightCheck(True)
    stereo.setExtendedDisparity(False)
    stereo.setSubpixel(False)
    
    # ⚠️ ADD THIS LINE TO MATCH RGB SIZE!
    stereo.setOutputSize(640, 480)  # Match the RGB preview size

    # Link ISP outputs to stereo
    cam_left.isp.link(stereo.left)
    cam_right.isp.link(stereo.right)

    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_depth.setStreamName("depth")
    stereo.depth.link(xout_depth.input)

    return pipeline


def test_pipeline_streaming():
    """Test pipeline with streaming from dual OAK-D S2 cameras"""

    print("=" * 60)
    print("TESTING MANIPULATION PIPELINE WITH STREAMING")
    print("=" * 60)

    for i, device_info in enumerate(device_infos[:2]):
        try:
            pipeline = create_camera_pipeline()
            device = stack.enter_context(dai.Device(pipeline, device_info, dai.UsbSpeed.SUPER))
            
            # GET CALIBRATION FROM DEVICE
            calibData = device.readCalibration()
            
            # Get intrinsics for the RGB camera (CAM_B for left camera)
            intrinsics_matrix = calibData.getCameraIntrinsics(
                dai.CameraBoardSocket.CAM_B,  # RGB camera
                640, 480  # Resolution you're using
            )
            
            # Extract [fx, fy, cx, cy] from 3x3 matrix
            fx = intrinsics_matrix[0][0]
            fy = intrinsics_matrix[1][1]
            cx = intrinsics_matrix[0][2]
            cy = intrinsics_matrix[1][2]
            
            print(f"Camera {i} intrinsics: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")
            
            # Build camera config with actual calibration
            if i == 0:
                extrinsics = cam1_to_robot
            else:
                extrinsics = cam2_to_robot
                
            camera_configs.append({
                "camera_id": i,
                "intrinsics": [fx, fy, cx, cy],
                "extrinsics": extrinsics
            })
            
            # Get output queues
            q_color = device.getOutputQueue(name="color", maxSize=4, blocking=False)
            q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            camera_queues.append((q_color, q_depth))
            
            print(f"✓ Camera {i} connected: {device_info.getMxId()}")
        except Exception as e:
            print(f"✗ Camera {i} failed: {e}")
            import traceback
            traceback.print_exc()
            return

    cam1_to_robot = np.array([
        [-0.5676864072910737, 0.20246387231771812, -0.797960226692451, 0.7502531269058595],
        [0.5144743879160396, 0.8439489366835451, -0.15187592452107102, -0.1942131460591212],
        [-0.6426882970424852, 0.49674799715432, 0.5832608165887726, 0.6305214067250764],
        [0.0, 0.0, 0.0, 1.0],
    ])

    cam2_to_robot = np.array([
        [0.5697063789662457, 0.5775554768385595, -0.5846916391902254, 0.4934273137283139],
        [0.8040468878867589, -0.5389612872714936, 0.2510564337002188, 0.28780764457046215],
        [0.1701271402357255, 0.6131479446238394, 0.771431367108426, 0.33694074301593196],
        [0.0, 0.0, 0.0, 1.0],
    ])

    # Convert intrinsics from 3x3 matrix to [fx, fy, cx, cy] format
    camera_configs = [
        {
            "camera_id": 0,
            "intrinsics": [401.05560302734375, 401.05560302734375, 319.18072509765625, 224.92999267578125],
            "extrinsics": cam1_to_robot,
        },
        {
            "camera_id": 1,
            "intrinsics": [399.1941833496094, 399.1941833496094, 305.7402648925781, 238.6328582763672],
            "extrinsics": cam2_to_robot,
        },
    ]
    
    # Initialize manipulation pipeline
    print("\nInitializing ManipulationPipeline...")

    manip_pipeline = ManipulationPipeline(
        camera_configs=camera_configs,
        min_confidence=0.05,
        enable_grasp_generation=True,
        grasp_server_url="ws://13.59.77.54:8000/ws/grasp",
        enable_segmentation=True,  # CHANGED: Disable to save memory
    )

    # Connect to DepthAI cameras
    with contextlib.ExitStack() as stack:
        device_infos = dai.Device.getAllAvailableDevices()

        print(f"\nFound {len(device_infos)} OAK-D devices:")
        for i, info in enumerate(device_infos):
            print(f"  Device {i}: {info.getMxId()}")

        if len(device_infos) < 2:
            print(f"\nERROR: Need 2 devices, found {len(device_infos)}")
            return

        print("\nConnecting cameras...")
        camera_queues = []

        for i, device_info in enumerate(device_infos[:2]):
            try:
                pipeline = create_camera_pipeline()
                device = stack.enter_context(dai.Device(pipeline, device_info, dai.UsbSpeed.SUPER))

                # Get output queues
                q_color = device.getOutputQueue(name="color", maxSize=4, blocking=False)
                q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
                camera_queues.append((q_color, q_depth))

                print(f"✓ Camera {i} connected: {device_info.getMxId()}")
            except Exception as e:
                print(f"✗ Camera {i} failed: {e}")
                import traceback
                traceback.print_exc()
                return

        print("\n✅ Both cameras connected!")

        # Create streaming pipeline
        print("\nCreating streaming pipeline...")
        output_streams = manip_pipeline.create_pipeline(camera_queues)
        print("✓ Pipeline created")
        
        # CREATE WINDOWS BEFORE SUBSCRIBING - THIS IS CRITICAL!
        print("\nCreating visualization windows...")
        cv2.namedWindow("Detection", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Point Cloud Overlay", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Background Clusters", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Grasp Overlay", cv2.WINDOW_NORMAL)
        
        # Show placeholder images immediately
        placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(placeholder, "Waiting for frames...", (150, 240),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow("Detection", placeholder)
        cv2.imshow("Point Cloud Overlay", placeholder)
        cv2.imshow("Background Clusters", placeholder)
        cv2.imshow("Grasp Overlay", placeholder) 
        cv2.waitKey(1)
        print("✓ Windows created and initialized")

        # Subscribe to output streams...
        print("\nSubscribing to output streams...")

        # Use queues to pass images to main thread
        import queue
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
                if detection_queue.full():
                    try:
                        detection_queue.get_nowait()
                    except:
                        pass
                try:
                    detection_queue.put_nowait(img)
                    if not first_frame_received[0]:
                        print("✓ First detection frame received")
                        first_frame_received[0] = True
                except:
                    pass

        def update_pointcloud(img):
            if img is not None:
                if pointcloud_queue.full():
                    try:
                        pointcloud_queue.get_nowait()
                    except:
                        pass
                try:
                    pointcloud_queue.put_nowait(img)
                except:
                    pass

        def update_background(img):
            if img is not None:
                if background_queue.full():
                    try:
                        background_queue.get_nowait()
                    except:
                        pass
                try:
                    background_queue.put_nowait(img)
                except:
                    pass

        def update_grasps(img):
            if img is not None:
                if grasp_queue.full():
                    try:
                        grasp_queue.get_nowait()
                    except:
                        pass
                try:
                    grasp_queue.put_nowait(img)
                except:
                    pass

        def update_objects(objects):
            latest_objects[0] = objects
            # Add grasp counting here
            frame_counter[0] += 1
            if frame_counter[0] % 30 == 0:
                print(f"\nFrame {frame_counter[0]}: {len(objects)} objects detected")
                grasp_count = sum(len(obj.get('grasps', [])) for obj in objects)
                if grasp_count > 0:
                    print(f"  → {grasp_count} total grasps available")

        def update_time(t):
            latest_time[0] = t

        # Subscribe to streams
        output_streams["detection_viz"].subscribe(on_next=update_detection)
        output_streams["pointcloud_viz"].subscribe(on_next=update_pointcloud)
        output_streams["misc_pointcloud_viz"].subscribe(on_next=update_background)
        output_streams["grasp_overlay"].subscribe(on_next=update_grasps)  # ADD THIS
        output_streams["all_objects"].subscribe(on_next=update_objects)
        output_streams["processing_time"].subscribe(on_next=update_time)

        print("\n" + "=" * 60)
        print("PIPELINE RUNNING")
        print("=" * 60)
        print("\nVisualization windows should show live video")
        print("Press 'q' to quit\n")

        # Main loop - UPDATE WINDOWS FROM MAIN THREAD
        loop_counter = 0
        try:
            while True:
                # Check for new detection image
                try:
                    img = detection_queue.get_nowait()
                    bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    cv2.imshow("Detection", bgr)
                except queue.Empty:
                    pass
                except Exception as e:
                    print(f"Detection display error: {e}")
                
                # Check for new pointcloud image
                try:
                    img = pointcloud_queue.get_nowait()
                    bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    cv2.imshow("Point Cloud Overlay", bgr)
                except queue.Empty:
                    pass
                except Exception as e:
                    print(f"Pointcloud display error: {e}")
                
                # Check for new background image
                try:
                    img = background_queue.get_nowait()
                    bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    cv2.imshow("Background Clusters", bgr)
                except queue.Empty:
                    pass
                except Exception as e:
                    print(f"Background display error: {e}")

                try:
                    img = grasp_queue.get_nowait()
                    bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    cv2.imshow("Grasp Overlay", bgr)
                except queue.Empty:
                    pass
                except Exception as e:
                    print(f"Grasp display error: {e}")
                
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

def test_pipeline_single_frame():
    """Test pipeline with single frame processing (no streaming)"""

    print("=" * 60)
    print("TESTING PIPELINE WITH SINGLE FRAME")
    print("=" * 60)

    # Camera configurations
    cam1_to_robot = np.array([
        [-0.5676864072910737, 0.20246387231771812, -0.797960226692451, 0.7502531269058595],
        [0.5144743879160396, 0.8439489366835451, -0.15187592452107102, -0.1942131460591212],
        [-0.6426882970424852, 0.49674799715432, 0.5832608165887726, 0.6305214067250764],
        [0.0, 0.0, 0.0, 1.0],
    ])

    cam2_to_robot = np.array([
        [0.5697063789662457, 0.5775554768385595, -0.5846916391902254, 0.4934273137283139],
        [0.8040468878867589, -0.5389612872714936, 0.2510564337002188, 0.28780764457046215],
        [0.1701271402357255, 0.6131479446238394, 0.771431367108426, 0.33694074301593196],
        [0.0, 0.0, 0.0, 1.0],
    ])

    # Convert intrinsics from 3x3 matrix to [fx, fy, cx, cy] format
    camera_configs = [
        {
            "camera_id": 0,
            "intrinsics": [401.05560302734375, 401.05560302734375, 319.18072509765625, 224.92999267578125],  # [fx, fy, cx, cy]
            "extrinsics": cam1_to_robot,
        },
        {
            "camera_id": 1,
            "intrinsics": [399.1941833496094, 399.1941833496094, 305.7402648925781, 238.6328582763672],  # [fx, fy, cx, cy]
            "extrinsics": cam2_to_robot,
        },
    ]

    # Initialize pipeline
    print("\nInitializing pipeline...")
    pipeline = ManipulationPipeline(
        camera_configs=camera_configs,
        enable_grasp_generation=False,
    )

    # Connect to cameras and grab one frame
    with contextlib.ExitStack() as stack:
        device_infos = dai.Device.getAllAvailableDevices()

        if len(device_infos) < 2:
            print(f"ERROR: Need 2 devices, found {len(device_infos)}")
            return

        print("Connecting cameras...")
        queues = []

        for _i, device_info in enumerate(device_infos[:2]):
            cam_pipeline = create_camera_pipeline()
            device = stack.enter_context(dai.Device(cam_pipeline, device_info))

            q_color = device.getOutputQueue("color", maxSize=1, blocking=False)
            q_depth = device.getOutputQueue("depth", maxSize=1, blocking=False)
            queues.append((q_color, q_depth))

        print("✓ Cameras connected")

        # Get frames
        print("\nCapturing frames...")
        rgb_images = []
        depth_images = []

        for q_color, q_depth in queues:
            # Wait for frames
            color_data = None
            depth_data = None

            for _ in range(100):  # Try up to 100 times
                color_data = q_color.tryGet()
                depth_data = q_depth.tryGet()
                if color_data and depth_data:
                    break
                import time

                time.sleep(0.01)

            if not (color_data and depth_data):
                print("ERROR: Failed to get frames")
                return

            rgb = color_data.getCvFrame()
            rgb = rgb[:, :, ::-1]  # BGR to RGB
            depth = depth_data.getFrame().astype(np.float32) / 1000.0  # mm to m

            rgb_images.append(rgb)
            depth_images.append(depth)

        print(f"✓ Captured {len(rgb_images)} RGB and {len(depth_images)} depth images")

        #Ensure RGB and depth have matching dimensions
        print("\nChecking image dimensions...")
        for i in range(len(rgb_images)):
            print(f"Camera {i}: RGB shape={rgb_images[i].shape}, Depth shape={depth_images[i].shape}")
            if rgb_images[i].shape[:2] != depth_images[i].shape[:2]:
                print(f"  ⚠️ Size mismatch! Resizing depth to match RGB...")
                h, w = rgb_images[i].shape[:2]
                depth_images[i] = cv2.resize(
                    depth_images[i], 
                    (w, h), 
                    interpolation=cv2.INTER_NEAREST  # Important: use NEAREST for depth to preserve values
                )
                print(f"  ✓ Resized depth to {depth_images[i].shape}")

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


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Test ManipulationPipeline")
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
