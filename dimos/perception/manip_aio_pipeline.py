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

import numpy as np
import cv2
import threading
from collections import deque
from reactivex import Observable
from reactivex import operators as ops
from typing import List, Optional, Dict
import time

from dimos.perception.detection2d.detic_2d_det import Detic2DDetector
from dimos.perception.pointcloud.pointcloud_filtering import PointcloudFiltering
from dimos.perception.object_detection_stream import ObjectDetectionStream
from dimos.perception.pointcloud.utils import create_point_cloud_overlay_visualization
from dimos.perception.common.utils import colorize_depth
from dimos.types.manipulation import ObjectData
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.perception.manip_aio_pipeline")


class ManipulationPipeline:
    """
    Clean separated stream pipeline with frame buffering.

    - Object detection runs independently on RGB stream
    - Point cloud processing subscribes to both detection and ZED streams separately
    - Simple frame buffering to match RGB+depth+objects
    """

    def __init__(
        self,
        camera_intrinsics: List[float],  # [fx, fy, cx, cy]
        min_confidence: float = 0.6,
        max_objects: int = 10,
        vocabulary: Optional[str] = None,
    ):
        """
        Initialize the manipulation pipeline.

        Args:
            camera_intrinsics: [fx, fy, cx, cy] camera parameters
            min_confidence: Minimum detection confidence threshold
            max_objects: Maximum number of objects to process
            max_bbox_size_percent: Maximum bbox size as percentage of image
            vocabulary: Optional vocabulary for Detic detector
        """
        self.camera_intrinsics = camera_intrinsics
        self.min_confidence = min_confidence

        # Initialize object detector
        self.detector = Detic2DDetector(vocabulary=vocabulary, threshold=min_confidence)

        # Initialize point cloud processor
        self.pointcloud_filter = PointcloudFiltering(
            color_intrinsics=camera_intrinsics,
            depth_intrinsics=camera_intrinsics,  # ZED uses same intrinsics
            max_num_objects=max_objects,
        )

        logger.info(f"Initialized ManipulationPipeline with confidence={min_confidence}")

    def create_streams(self, zed_stream: Observable) -> Dict[str, Observable]:
        """
        Create streams using exact old main logic.
        """
        # Create ZED streams (from old main)
        zed_frame_stream = zed_stream.pipe(ops.share())

        # RGB stream for object detection (from old main)
        video_stream = zed_frame_stream.pipe(
            ops.map(lambda x: x.get("rgb") if x is not None else None),
            ops.filter(lambda x: x is not None),
            ops.share(),
        )
        object_detector = ObjectDetectionStream(
            camera_intrinsics=self.camera_intrinsics,
            min_confidence=self.min_confidence,
            class_filter=None,
            detector=self.detector,
            video_stream=video_stream,
            disable_depth=True,
        )

        # Store latest frames for point cloud processing (from old main)
        latest_rgb = None
        latest_depth = None
        latest_point_cloud_overlay = None
        frame_lock = threading.Lock()

        # Subscribe to combined ZED frames (from old main)
        def on_zed_frame(zed_data):
            nonlocal latest_rgb, latest_depth
            if zed_data is not None:
                with frame_lock:
                    latest_rgb = zed_data.get("rgb")
                    latest_depth = zed_data.get("depth")

        # Depth stream for point cloud filtering (from old main)
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

        # Process object detection results with point cloud filtering (from old main)
        def on_detection_next(result):
            nonlocal latest_point_cloud_overlay
            if "objects" in result and result["objects"]:
                # Get latest RGB and depth frames
                with frame_lock:
                    rgb = latest_rgb
                    depth = latest_depth

                if rgb is not None and depth is not None:
                    try:
                        filtered_objects = self.pointcloud_filter.process_images(
                            rgb, depth, result["objects"]
                        )

                        if filtered_objects:
                            # Create base image (colorized depth)
                            base_image = colorize_depth(depth, max_depth=10.0)

                            # Create point cloud overlay visualization
                            overlay_viz = create_point_cloud_overlay_visualization(
                                base_image=base_image,
                                filtered_objects=filtered_objects,
                                camera_matrix=self.camera_intrinsics,
                            )

                            # Store the overlay for the stream
                            with frame_lock:
                                latest_point_cloud_overlay = overlay_viz
                        else:
                            # No filtered objects, clear overlay
                            with frame_lock:
                                latest_point_cloud_overlay = None

                    except Exception as e:
                        logger.error(f"Error in point cloud filtering: {e}")
                        with frame_lock:
                            latest_point_cloud_overlay = None

        def on_error(error):
            logger.error(f"Error in stream: {error}")

        def on_completed():
            logger.info("Stream completed")

        def start_subscriptions():
            """Start subscriptions in background thread (from old main)"""
            # Subscribe to combined ZED frames
            zed_frame_stream.subscribe(on_next=on_zed_frame)

        # Start subscriptions in background thread (from old main)
        subscription_thread = threading.Thread(target=start_subscriptions, daemon=True)
        subscription_thread.start()
        time.sleep(2)  # Give subscriptions time to start

        # Subscribe to object detection stream (from old main)
        object_detector.get_stream().subscribe(
            on_next=on_detection_next, on_error=on_error, on_completed=on_completed
        )

        # Create visualization stream for web interface (from old main)
        viz_stream = object_detector.get_stream().pipe(
            ops.map(lambda x: x["viz_frame"] if x is not None else None),
            ops.filter(lambda x: x is not None),
        )

        return {
            "detection_viz": viz_stream,
            "pointcloud_viz": depth_stream,
            "objects": object_detector.get_stream().pipe(ops.map(lambda x: x.get("objects", []))),
        }

    def cleanup(self):
        """Clean up resources."""
        if hasattr(self.detector, "cleanup"):
            self.detector.cleanup()
        if hasattr(self.pointcloud_filter, "cleanup"):
            self.pointcloud_filter.cleanup()
        logger.info("ManipulationPipeline cleaned up")


def create_manipulation_pipeline(
    camera_intrinsics: List[float],
    min_confidence: float = 0.6,
    max_objects: int = 10,
    vocabulary: Optional[str] = None,
) -> ManipulationPipeline:
    """
    Factory function to create a ManipulationPipeline with sensible defaults.

    Args:
        camera_intrinsics: [fx, fy, cx, cy] camera parameters
        min_confidence: Minimum detection confidence threshold
        max_objects: Maximum number of objects to process
        vocabulary: Optional vocabulary for Detic detector

    Returns:
        Configured ManipulationPipeline instance
    """
    return ManipulationPipeline(
        camera_intrinsics=camera_intrinsics,
        min_confidence=min_confidence,
        max_objects=max_objects,
        vocabulary=vocabulary,
    )
