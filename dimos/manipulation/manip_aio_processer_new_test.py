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
test_manipulation_processor.py - Test the ManipulationProcessor with dual cameras
"""

import sys

import cv2
from manip_aio_processer_new_depthai import ManipulationProcessor
import numpy as np


def create_dummy_rgbd_pair(width=640, height=480):
    """Create dummy RGB-D data for testing."""
    # Create a simple synthetic RGB image with colored rectangles
    rgb = np.zeros((height, width, 3), dtype=np.uint8)

    # Add some colored objects
    cv2.rectangle(rgb, (100, 100), (200, 200), (255, 0, 0), -1)  # Blue rectangle
    cv2.rectangle(rgb, (300, 150), (400, 250), (0, 255, 0), -1)  # Green rectangle
    cv2.rectangle(rgb, (450, 200), (550, 300), (0, 0, 255), -1)  # Red rectangle

    # Create synthetic depth (everything at 1.0m with some variation)
    depth = np.ones((height, width), dtype=np.float32) * 1.0

    # Add depth variation for "objects"
    depth[100:200, 100:200] = 0.8  # Blue object closer
    depth[150:250, 300:400] = 0.9  # Green object
    depth[200:300, 450:550] = 1.1  # Red object farther

    return rgb, depth


def test_processor_basic():
    """Test basic processor functionality with synthetic data."""
    print("=" * 60)
    print("TESTING MANIPULATION PROCESSOR")
    print("=" * 60)

    # Camera configurations (using your ArUco calibration format)

    cam1_to_robot = np.array(
        [
            [-0.5676864072910737, 0.20246387231771812, -0.797960226692451, 0.7502531269058595],
            [0.5144743879160396, 0.8439489366835451, -0.15187592452107102, -0.1942131460591212],
            [-0.6426882970424852, 0.49674799715432, 0.5832608165887726, 0.6305214067250764],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    cam2_to_robot = np.array(
        [
            [0.5697063789662457, 0.5775554768385595, -0.5846916391902254, 0.4934273137283139],
            [0.8040468878867589, -0.5389612872714936, 0.2510564337002188, 0.28780764457046215],
            [0.1701271402357255, 0.6131479446238394, 0.771431367108426, 0.33694074301593196],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    camera_configs = [
        {
            "camera_id": 0,
            "intrinsics": np.array(
                [
                    [401.05560302734375, 0.0, 319.18072509765625],
                    [0.0, 401.05560302734375, 224.92999267578125],
                    [0.0, 0.0, 1.0],
                ]
            ),
            "distortion": np.array(
                [
                    -9.035920143127441,
                    73.58698272705078,
                    0.002187757520005107,
                    0.0005836759228259325,
                    -44.24618911743164,
                    -9.082674026489258,
                    73.32916259765625,
                    -43.113433837890625,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ]
            ),
            "extrinsics": cam1_to_robot,
        },
        {
            "camera_id": 1,
            "intrinsics": np.array(
                [
                    [399.1941833496094, 0.0, 305.7402648925781],
                    [0.0, 399.1941833496094, 238.6328582763672],
                    [0.0, 0.0, 1.0],
                ]
            ),
            "distortion": np.array(
                [
                    5.883155345916748,
                    5.506092071533203,
                    0.00180531432852149,
                    0.0003752215125132352,
                    -24.15361213684082,
                    5.59039831161499,
                    6.560574531555176,
                    -25.07012367248535,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ]
            ),
            "extrinsics": cam2_to_robot,
        },
    ]

    # Initialize processor WITHOUT grasp generation for basic testing
    print("\nInitializing ManipulationProcessor...")
    processor = ManipulationProcessor(
        camera_configs=camera_configs,
        min_confidence=0.5,
        max_objects=20,
        enable_grasp_generation=False,  # Disable for basic CV testing
        enable_segmentation=True,
    )
    print("✓ Processor initialized")

    # Create synthetic RGB-D data for both cameras
    print("\nCreating synthetic RGB-D data...")
    rgb1, depth1 = create_dummy_rgbd_pair()
    rgb2, depth2 = create_dummy_rgbd_pair()

    # Slightly vary camera 2 to simulate different viewpoint
    rgb2 = cv2.flip(rgb2, 1)  # Horizontal flip
    depth2 = cv2.flip(depth2, 1)

    print("✓ Synthetic data created")
    print(f"  Camera 1: RGB {rgb1.shape}, Depth {depth1.shape}")
    print(f"  Camera 2: RGB {rgb2.shape}, Depth {depth2.shape}")

    # Process frame
    print("\nProcessing frame...")
    results = processor.process_frame(
        rgb_images=[rgb1, rgb2], depth_images=[depth1, depth2], generate_grasps=False
    )

    # Display results
    print("\n" + "=" * 60)
    print("RESULTS")
    print("=" * 60)

    if "error" in results:
        print(f"❌ ERROR: {results['error']}")
        return

    print(f"✓ Processing completed in {results['processing_time']:.3f}s")
    print("\nTiming breakdown:")
    for key, value in results["timing_breakdown"].items():
        print(f"  {key}: {value:.3f}s")

    print(f"\nDetected objects: {len(results.get('detected_objects', []))}")
    print(f"All objects (merged): {len(results.get('all_objects', []))}")

    if results.get("full_pointcloud"):
        pcd = results["full_pointcloud"]
        print(f"Stitched point cloud: {len(pcd.points)} points")

    if results.get("misc_clusters"):
        print(f"Background clusters: {len(results['misc_clusters'])}")

    # Display per-camera data
    print("\nPer-camera breakdown:")
    for cam_data in results.get("per_camera_data", []):
        print(f"  Camera {cam_data['camera_id']}:")
        print(f"    Detection objects: {len(cam_data['detection2d_objects'])}")
        print(f"    Filtered objects: {len(cam_data['detected_objects'])}")

    # Save visualizations instead of showing
    print("\nSaving visualizations to /tmp/...")

    # Save detection visualizations
    for i, cam_data in enumerate(results.get("per_camera_data", [])):
        if cam_data.get("detection_viz") is not None:
            viz_bgr = cv2.cvtColor(cam_data["detection_viz"], cv2.COLOR_RGB2BGR)
            filename = f"/tmp/camera_{i}_detection.png"
            cv2.imwrite(filename, viz_bgr)
            print(f"  ✓ Saved: {filename}")

    # Save pointcloud overlay
    if results.get("pointcloud_viz") is not None:
        viz_bgr = cv2.cvtColor(results["pointcloud_viz"], cv2.COLOR_RGB2BGR)
        cv2.imwrite("/tmp/pointcloud_overlay.png", viz_bgr)
        print("  ✓ Saved: /tmp/pointcloud_overlay.png")

    # Save misc clusters
    if results.get("misc_pointcloud_viz") is not None:
        viz_bgr = cv2.cvtColor(results["misc_pointcloud_viz"], cv2.COLOR_RGB2BGR)
        cv2.imwrite("/tmp/background_clusters.png", viz_bgr)
        print("  ✓ Saved: /tmp/background_clusters.png")

    print("\nVisualization images saved! Check /tmp/ directory")

    # Cleanup
    processor.cleanup()
    print("\n✓ Test completed successfully!")


def test_processor_with_real_cameras():
    """
    Test with real OAK-D cameras.
    Replace the dummy data creation with actual camera streams.
    """
    print("=" * 60)
    print("TESTING WITH REAL CAMERAS")
    print("=" * 60)
    print("\nThis test requires:")
    print("1. Two OAK-D cameras connected")
    print("2. ArUco calibration matrices")
    print("3. DepthAI library installed")
    print("\nTo implement:")
    print("- Use your dual_oak_camera.py to get RGB+Depth streams")
    print("- Replace dummy data with real camera frames")
    print("- Use your actual ArUco calibration matrices")
    print("\nNot implemented yet - use test_processor_basic() first")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Test ManipulationProcessor")
    parser.add_argument(
        "--mode",
        choices=["basic", "real"],
        default="basic",
        help="Test mode: basic (synthetic data) or real (real cameras)",
    )

    args = parser.parse_args()

    if args.mode == "basic":
        test_processor_basic()
    else:
        test_processor_with_real_cameras()
