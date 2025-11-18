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
    """Create DepthAI pipeline for OAK-D S2 (same as your working code)"""
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
    cam_left.setPreviewSize(640, 480)
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

    # Camera configurations (use your ArUco calibration results here!)
    camera_configs = [
        {
            "camera_id": 0,
            "intrinsics": [600.0, 600.0, 320.0, 240.0],  # [fx, fy, cx, cy]
            "extrinsics": np.array(
                [  # Camera 0 to world (from ArUco)
                    [0.999, 0.012, -0.034, 0.152],
                    [-0.011, 0.998, 0.045, -0.082],
                    [0.035, -0.044, 0.997, 0.723],
                    [0, 0, 0, 1],
                ]
            ),
        },
        {
            "camera_id": 1,
            "intrinsics": [600.0, 600.0, 320.0, 240.0],
            "extrinsics": np.array(
                [  # Camera 1 to world (from ArUco)
                    [0.997, -0.023, 0.071, 0.453],
                    [0.025, 0.999, -0.018, 0.105],
                    [-0.070, 0.021, 0.997, 0.695],
                    [0, 0, 0, 1],
                ]
            ),
        },
    ]

    # Initialize manipulation pipeline
    print("\nInitializing ManipulationPipeline...")

    # Set this to True to enable grasp generation (requires grasp server running)
    enable_grasps = False  # Change to True when grasp server is ready

    manip_pipeline = ManipulationPipeline(
        camera_configs=camera_configs,
        min_confidence=0.5,
        enable_grasp_generation=enable_grasps,
        grasp_server_url="ws://localhost:8765" if enable_grasps else None,
        enable_segmentation=True,
    )
    print("✓ Pipeline initialized")
    if enable_grasps:
        print("  Grasp generation: ENABLED (make sure server is running!)")
    else:
        print("  Grasp generation: DISABLED (set enable_grasps=True to enable)")

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

        # Subscribe to output streams
        print("\nSubscribing to output streams...\n")

        # Detection visualization
        output_streams["detection_viz"].subscribe(
            on_next=lambda img: cv2.imshow("Detection", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            if img is not None
            else None
        )

        # Point cloud overlay
        output_streams["pointcloud_viz"].subscribe(
            on_next=lambda img: cv2.imshow(
                "Point Cloud Overlay", cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            )
            if img is not None
            else None
        )

        # Background clusters
        output_streams["misc_pointcloud_viz"].subscribe(
            on_next=lambda img: cv2.imshow(
                "Background Clusters", cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            )
            if img is not None
            else None
        )

        # Grasp overlay (if grasp generation enabled)
        output_streams["grasp_overlay"].subscribe(
            on_next=lambda img: cv2.imshow("Grasp Overlay", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            if img is not None
            else None
        )

        # Object data (print to console with grasp info)
        def on_objects(objects):
            print(f"\n{'=' * 60}")
            print(f"DETECTED {len(objects)} OBJECTS:")
            print(f"{'=' * 60}")
            for i, obj in enumerate(objects):
                print(f"\nObject {i}: {obj.get('class_name', 'unknown')}")
                print(f"  Confidence: {obj.get('confidence', 0):.2f}")
                if obj.get("point_cloud"):
                    print(f"  Points: {len(obj['point_cloud'].points)}")

                # Show grasp information
                if obj.get("grasps"):
                    print(f"  Grasps: {len(obj['grasps'])}")
                    # Show top 3 grasps
                    for j, grasp in enumerate(obj["grasps"][:3]):
                        print(f"    Grasp {j + 1}:")
                        print(f"      Score: {grasp.get('score', 0):.3f}")
                        print(f"      Position: {grasp.get('translation', [0, 0, 0])}")
                        print(f"      Width: {grasp.get('width', 0) * 1000:.1f}mm")
                else:
                    print("  Grasps: None")

        output_streams["all_objects"].subscribe(on_next=on_objects)

        # Processing time
        output_streams["processing_time"].subscribe(
            on_next=lambda t: print(f"Processing time: {t:.3f}s")
        )

        print("=" * 60)
        print("PIPELINE RUNNING")
        print("=" * 60)
        print("\nVisualization windows:")
        print("  - Detection: Object bounding boxes")
        print("  - Point Cloud Overlay: 3D points on depth image")
        print("  - Background Clusters: Scene background")
        if enable_grasps:
            print("  - Grasp Overlay: Grasp poses on objects")
        print("\nConsole output:")
        print("  - Object detections with confidence scores")
        if enable_grasps:
            print("  - Grasp poses per object (position, score, width)")
        print("\nPress 'q' to quit\n")

        # Main loop
        try:
            while True:
                key = cv2.waitKey(1)
                if key == ord("q") or key == 27:
                    break
        except KeyboardInterrupt:
            print("\nStopping...")

        # Cleanup
        print("\nCleaning up...")
        manip_pipeline.stop()
        cv2.destroyAllWindows()
        print("✓ Pipeline stopped")


def test_pipeline_single_frame():
    """Test pipeline with single frame processing (no streaming)"""

    print("=" * 60)
    print("TESTING PIPELINE WITH SINGLE FRAME")
    print("=" * 60)

    # Camera configurations
    camera_configs = [
        {
            "camera_id": 0,
            "intrinsics": [600.0, 600.0, 320.0, 240.0],
            "extrinsics": np.array(
                [
                    [0.999, 0.012, -0.034, 0.152],
                    [-0.011, 0.998, 0.045, -0.082],
                    [0.035, -0.044, 0.997, 0.723],
                    [0, 0, 0, 1],
                ]
            ),
        },
        {
            "camera_id": 1,
            "intrinsics": [600.0, 600.0, 320.0, 240.0],
            "extrinsics": np.array(
                [
                    [0.997, -0.023, 0.071, 0.453],
                    [0.025, 0.999, -0.018, 0.105],
                    [-0.070, 0.021, 0.997, 0.695],
                    [0, 0, 0, 1],
                ]
            ),
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
