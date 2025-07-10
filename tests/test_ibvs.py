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
Simple test script for Detection3D processor with ZED camera.
Press 'q' to quit, 's' to save current frame.
"""

import cv2
import numpy as np
import sys
import os

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dimos.hardware.zed_camera import ZEDCamera
from dimos.manipulation.ibvs.detection3d import Detection3DProcessor

try:
    import pyzed.sl as sl
except ImportError:
    print("Error: ZED SDK not installed. Please install pyzed package.")
    sys.exit(1)


def main():
    """Main test function."""
    print("Starting Detection3D test with ZED camera...")

    # Initialize ZED camera
    print("Initializing ZED camera...")
    zed_camera = ZEDCamera(
        camera_id=0,
        resolution=sl.RESOLUTION.HD720,  # 1280x720 for good performance
        depth_mode=sl.DEPTH_MODE.NEURAL,  # Best quality depth
        fps=30,
    )

    # Open camera
    if not zed_camera.open():
        print("Failed to open ZED camera!")
        return

    # Get camera intrinsics
    camera_info = zed_camera.get_camera_info()
    left_cam = camera_info.get("left_cam", {})

    # Extract intrinsics [fx, fy, cx, cy]
    intrinsics = [
        left_cam.get("fx", 700),
        left_cam.get("fy", 700),
        left_cam.get("cx", 640),
        left_cam.get("cy", 360),
    ]

    print(
        f"Camera intrinsics: fx={intrinsics[0]:.1f}, fy={intrinsics[1]:.1f}, "
        f"cx={intrinsics[2]:.1f}, cy={intrinsics[3]:.1f}"
    )

    # Initialize Detection3D processor
    print("Initializing Detection3D processor...")
    detector = Detection3DProcessor(
        camera_intrinsics=intrinsics,
        min_confidence=0.5,  # Lower threshold for more detections
        min_points=20,  # Lower for better real-time performance
        max_depth=3.0,  # Limit to 3 meters
    )

    print("\nStarting detection loop...")
    print("Press 'q' to quit, 's' to save current frame")

    frame_count = 0

    try:
        while True:
            # Capture frame
            left_img, right_img, depth = zed_camera.capture_frame()

            if left_img is None or depth is None:
                print("Failed to capture frame")
                continue

            # Convert BGR to RGB for detection
            rgb_img = cv2.cvtColor(left_img, cv2.COLOR_BGR2RGB)

            # Process frame
            results = detector.process_frame(rgb_img, depth)

            # Create visualization
            viz = detector.visualize_detections(rgb_img, results["detections"], show_3d=True)

            # Convert back to BGR for OpenCV display
            viz_bgr = cv2.cvtColor(viz, cv2.COLOR_RGB2BGR)

            # Add info text
            info_text = [
                f"Frame: {frame_count}",
                f"Detections: {len(results['detections'])}",
                f"3D Valid: {sum(1 for d in results['detections'] if d.get('has_3d', False))}",
                f"Time: {results['processing_time'] * 1000:.1f}ms",
            ]

            y_offset = 20
            for text in info_text:
                cv2.putText(
                    viz_bgr, text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                )
                y_offset += 25

            # Find closest detection
            closest = detector.get_closest_detection(results["detections"])
            if closest:
                text = f"Closest: {closest['class_name']} @ {closest['centroid'][2]:.2f}m"
                cv2.putText(
                    viz_bgr, text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2
                )

            # Display
            cv2.imshow("Detection3D Test", viz_bgr)

            # Handle key press
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("s"):
                # Save current frame
                cv2.imwrite(f"detection3d_frame_{frame_count:04d}.png", viz_bgr)
                print(f"Saved frame {frame_count}")

            frame_count += 1

            # Print detections every 30 frames
            if frame_count % 30 == 0:
                print(f"\nFrame {frame_count}:")
                for det in results["detections"]:
                    if det.get("has_3d", False):
                        print(f"  - {det['class_name']}: {det['centroid'][2]:.2f}m away")

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    finally:
        # Cleanup
        print("\nCleaning up...")
        cv2.destroyAllWindows()
        detector.cleanup()
        zed_camera.close()
        print("Done!")


if __name__ == "__main__":
    main()
