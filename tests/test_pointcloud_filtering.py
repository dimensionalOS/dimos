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

import sys
import time
import threading
from typing import List, Dict, Any
from reactivex import operators as ops
import numpy as np
import cv2

import tests.test_header

from pyzed import sl
from dimos.stream.stereo_camera_streams.zed import ZEDCameraStream
from dimos.web.robot_web_interface import RobotWebInterface
from dimos.utils.logging_config import logger
from dimos.perception.object_detection_stream import ObjectDetectionStream
from dimos.perception.detection2d.detic_2d_det import Detic2DDetector
from dimos.perception.pointcloud.pointcloud_filtering import PointcloudFiltering
from dimos.perception.pointcloud.utils import create_point_cloud_overlay_visualization


def colorize_depth(depth_img, max_depth=5.0):
    """Normalize and colorize depth image in one step."""
    if depth_img is None:
        return None
    valid_mask = np.isfinite(depth_img) & (depth_img > 0)
    depth_norm = np.zeros_like(depth_img)
    depth_norm[valid_mask] = np.clip(depth_img[valid_mask] / max_depth, 0, 1)
    depth_colored = cv2.applyColorMap((depth_norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
    depth_rgb = cv2.cvtColor(depth_colored, cv2.COLOR_BGR2RGB)

    # Make the depth image less bright by scaling down the values
    depth_rgb = (depth_rgb * 0.6).astype(np.uint8)

    return depth_rgb


def main():
    print("Initializing ZED camera with object detection and point cloud filtering...")

    # Configuration
    min_confidence = 0.6
    web_port = 5555

    try:
        # Initialize ZED camera stream
        zed_stream = ZEDCameraStream(resolution=sl.RESOLUTION.HD1080, fps=10)

        # Get camera intrinsics
        camera_intrinsics_dict = zed_stream.get_camera_info()
        camera_intrinsics = [
            camera_intrinsics_dict["fx"],
            camera_intrinsics_dict["fy"],
            camera_intrinsics_dict["cx"],
            camera_intrinsics_dict["cy"],
        ]

        # Create ZED streams
        zed_frame_stream = zed_stream.create_stream().pipe(ops.share())

        # RGB stream for object detection
        video_stream = zed_frame_stream.pipe(
            ops.map(lambda x: x.get("rgb") if x is not None else None),
            ops.filter(lambda x: x is not None),
            ops.share(),
        )

        # Initialize object detection
        detector = Detic2DDetector(vocabulary=None, threshold=min_confidence)
        object_detector = ObjectDetectionStream(
            camera_intrinsics=camera_intrinsics,
            min_confidence=min_confidence,
            class_filter=None,
            detector=detector,
            video_stream=video_stream,
            disable_depth=True,
        )

        # Initialize point cloud filtering
        pointcloud_filter = PointcloudFiltering(
            color_intrinsics=camera_intrinsics,
            depth_intrinsics=camera_intrinsics,  # ZED uses same intrinsics for RGB and depth
        )

    except ImportError:
        print("Error: ZED SDK not installed. Please install pyzed package.")
        sys.exit(1)
    except RuntimeError as e:
        print(f"Error: Failed to open ZED camera: {e}")
        sys.exit(1)

    # Store latest frames for point cloud processing
    latest_rgb = None
    latest_depth = None
    latest_point_cloud_overlay = None
    frame_lock = threading.Lock()

    # Subscribe to combined ZED frames
    def on_zed_frame(zed_data):
        nonlocal latest_rgb, latest_depth
        if zed_data is not None:
            with frame_lock:
                latest_rgb = zed_data.get("rgb")
                latest_depth = zed_data.get("depth")

    # Depth stream for point cloud filtering - shows overlay when available, otherwise regular depth
    def get_depth_or_overlay(zed_data):
        if zed_data is None:
            return None

        # Check if we have a point cloud overlay available
        with frame_lock:
            overlay = latest_point_cloud_overlay

        if overlay is not None:
            return overlay
        else:
            # Return regular colorized depth
            return colorize_depth(zed_data.get("depth"), max_depth=10.0)

    depth_stream = zed_frame_stream.pipe(
        ops.map(get_depth_or_overlay), ops.filter(lambda x: x is not None), ops.share()
    )

    # Process object detection results with point cloud filtering
    def on_detection_next(result):
        nonlocal latest_point_cloud_overlay
        if "objects" in result and result["objects"]:
            # Get latest RGB and depth frames
            with frame_lock:
                rgb = latest_rgb
                depth = latest_depth

            if rgb is not None and depth is not None:
                try:
                    filtered_objects = pointcloud_filter.process_images(
                        rgb, depth, result["objects"]
                    )

                    if filtered_objects:
                        # Create base image (colorized depth)
                        base_image = colorize_depth(depth, max_depth=10.0)

                        # Create point cloud overlay visualization
                        overlay_viz = create_point_cloud_overlay_visualization(
                            base_image=base_image,
                            filtered_objects=filtered_objects,
                            camera_matrix=camera_intrinsics,
                        )

                        # Store the overlay for the stream
                        with frame_lock:
                            latest_point_cloud_overlay = overlay_viz

                        # # Print object stats for debugging
                        # print(f"\nProcessed {len(filtered_objects)} objects with point clouds:")
                        # for i, obj in enumerate(filtered_objects):
                        #     if "point_cloud" in obj and obj["point_cloud"] is not None:
                        #         position = obj.get("position")
                        #         confidence = obj.get("confidence", 0)
                        #         print(f"  Object {i+1}: {obj.get('label', 'unknown')} (confidence: {confidence:.2f})")
                        #         if position and hasattr(position, 'x'):
                        #             print(f"    Position: ({position.x:.2f}, {position.y:.2f}, {position.z:.2f}) m")
                    else:
                        # No filtered objects, clear overlay
                        with frame_lock:
                            latest_point_cloud_overlay = None

                except Exception as e:
                    print(f"Error in point cloud filtering: {e}")
                    with frame_lock:
                        latest_point_cloud_overlay = None

    def on_error(error):
        print(f"Error in stream: {error}")

    def on_completed():
        print("Stream completed")

    def start_subscriptions():
        """Start subscriptions in background thread"""
        # Subscribe to combined ZED frames
        zed_frame_stream.subscribe(on_next=on_zed_frame)

    try:
        # Start subscriptions in background thread
        subscription_thread = threading.Thread(target=start_subscriptions, daemon=True)
        subscription_thread.start()
        time.sleep(2)  # Give subscriptions time to start

        # Subscribe to object detection stream
        object_detector.get_stream().subscribe(
            on_next=on_detection_next, on_error=on_error, on_completed=on_completed
        )

        # Create visualization stream for web interface
        viz_stream = object_detector.get_stream().pipe(
            ops.map(lambda x: x["viz_frame"] if x is not None else None),
            ops.filter(lambda x: x is not None),
        )

        # Set up web interface
        print("Initializing web interface...")
        web_interface = RobotWebInterface(
            port=web_port, 
            object_detection=viz_stream,
            pointcloud_stream=depth_stream  # Use the simplified depth stream that includes overlays
        )

        print("\nZED Stream + Object Detection + Point Cloud Filtering Test Running:")
        print(f"Web Interface: http://localhost:{web_port}")
        print(f"Point cloud overlay visualization displayed in depth stream")
        print(f"Min confidence: {min_confidence}")
        print(f"Each detected object's point cloud shown in a distinct color")
        print("\nPress Ctrl+C to stop the test\n")

        # Start web server (blocking call)
        web_interface.run()

    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error during test: {e}")
    finally:
        print("Cleaning up resources...")
        if "zed_stream" in locals():
            zed_stream.cleanup()
        if "pointcloud_filter" in locals():
            pointcloud_filter.cleanup()
        print("Test completed")


if __name__ == "__main__":
    main()
