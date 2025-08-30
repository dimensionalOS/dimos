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

import time
import threading
from typing import Optional

import numpy as np
import cv2

from dimos.core import Module, In, Out, rpc
from dimos.msgs.sensor_msgs import Image, ImageFormat, PointCloud2
from dimos.msgs.geometry_msgs import Transform
from dimos_lcm.sensor_msgs import CameraInfo
from dimos.protocol.tf import TF
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import transform_pointcloud
from dimos.perception.common.utils import project_3d_points_to_2d
from dimos.types.timestamped import align_timestamped

logger = setup_logger(__name__)


class PointCloudToDepth(Module):
    """
    High-performance module that transforms lidar pointclouds to camera frame and generates depth images.

    This module provides real-time conversion of 3D lidar data to camera-frame depth images,
    enabling sensor fusion between lidar and camera data.

    Subscribes to:
        - /lidar: Lidar pointcloud data (LidarMessage)
        - /go2/color_image: RGB camera images (Image)
        - /go2/camera_info: Camera calibration information (CameraInfo)

    Publishes:
        - /go2/depth_image: Depth image generated from lidar pointcloud (Image/DEPTH16)
        - /pointcloud_camera: Transformed pointcloud in camera frame (PointCloud2)
        - /pointcloud_2d_overlay: RGB image with projected pointcloud overlay (Image/RGB)
    """

    # LCM inputs
    lidar: In[LidarMessage] = None
    color_image: In[Image] = None
    camera_info: In[CameraInfo] = None

    # LCM outputs
    depth_image: Out[Image] = None
    pointcloud_camera: Out[PointCloud2] = None
    pointcloud_overlay: Out[Image] = None

    def __init__(
        self,
        max_depth: float = 10.0,
        min_depth: float = 0.1,
        point_size: int = 4,
        **kwargs,
    ):
        """
        Initialize PointCloudToDepth module.

        Args:
            max_depth: Maximum depth value to consider in meters (default: 10.0)
            min_depth: Minimum depth value to consider in meters (default: 0.1)
            point_size: Size of points in overlay visualization (default: 2)
        """
        super().__init__(**kwargs)

        # Configuration
        self.max_depth = max_depth
        self.min_depth = min_depth
        self.point_size = point_size

        # Camera parameters
        self.camera_intrinsics = None
        self.image_width = None
        self.image_height = None

        # TF system
        self.tf = TF()

        # State management
        self._running = False
        self._processing_thread: Optional[threading.Thread] = None
        self._stop_processing = threading.Event()

        # Data buffers with thread safety
        self._latest_lidar: Optional[LidarMessage] = None
        self._latest_image: Optional[Image] = None
        self._latest_camera_info: Optional[CameraInfo] = None
        self._data_lock = threading.Lock()

        # Subscription management
        self._aligned_subscription = None

        # Performance tracking
        self._last_process_time = 0.0
        self._process_count = 0

        logger.info(f"PointCloudToDepth initialized, depth range: [{min_depth}, {max_depth}]m")

    @rpc
    def start(self) -> None:
        """Start the module and begin processing."""
        if self._running:
            logger.warning("PointCloudToDepth module already running")
            return

        self._running = True
        self._process_count = 0

        # Subscribe to camera info for intrinsics
        self.camera_info.subscribe(self._on_camera_info)

        # Create temporally aligned observable for synchronized processing
        aligned_data = align_timestamped(
            self.lidar.observable(),
            self.color_image.observable(),
            buffer_size=2.0,  # 2 second buffer for temporal alignment
            match_tolerance=0.5,  # 100ms tolerance for timestamp matching
        )
        self._aligned_subscription = aligned_data.subscribe(self._on_aligned_data)

        # Start background processing thread
        self._start_processing_thread()

        logger.info("PointCloudToDepth module started successfully")

    @rpc
    def stop(self) -> None:
        """Stop the module and cleanup resources."""
        if not self._running:
            return

        self._running = False
        self._stop_processing.set()

        # Wait for processing thread to complete
        if self._processing_thread and self._processing_thread.is_alive():
            self._processing_thread.join(timeout=2.0)

        # Cleanup subscriptions
        if self._aligned_subscription:
            self._aligned_subscription.dispose()
            self._aligned_subscription = None

        logger.info(f"PointCloudToDepth module stopped (processed {self._process_count} frames)")

    def _on_camera_info(self, msg: CameraInfo) -> None:
        """Extract and store camera intrinsic parameters."""
        with self._data_lock:
            # Extract intrinsics from camera matrix K (row-major order)
            K = msg.K
            self.camera_intrinsics = [K[0], K[4], K[2], K[5]]  # [fx, fy, cx, cy]
            self.image_width = msg.width
            self.image_height = msg.height
            self._latest_camera_info = msg

    def _on_aligned_data(self, data_tuple) -> None:
        """Handle temporally aligned lidar and image data."""
        lidar_msg, image_msg = data_tuple
        logger.debug(
            f"Received aligned data: lidar_msg.ts={lidar_msg.ts}, image_msg.ts={image_msg.ts}"
        )

        with self._data_lock:
            self._latest_lidar = lidar_msg
            self._latest_image = image_msg

        logger.debug("Received aligned sensor data")

    def _start_processing_thread(self) -> None:
        """Initialize and start the background processing thread."""
        self._stop_processing.clear()
        self._processing_thread = threading.Thread(
            target=self._processing_loop, daemon=True, name="PointCloudToDepth-Processing"
        )
        self._processing_thread.start()
        logger.debug("Processing thread started")

    def _processing_loop(self) -> None:
        """Main processing loop running in background thread."""
        logger.info("Starting pointcloud processing loop")

        while not self._stop_processing.is_set():
            try:
                # Safely retrieve latest data
                with self._data_lock:
                    if (
                        self._latest_lidar is not None
                        and self._latest_image is not None
                        and self.camera_intrinsics is not None
                    ):
                        # Extract data for processing
                        lidar_msg = self._latest_lidar
                        image_msg = self._latest_image

                        # Clear buffers to prevent reprocessing
                        self._latest_lidar = None
                        self._latest_image = None
                    else:
                        lidar_msg = None
                        image_msg = None

                # Process if data available
                if lidar_msg is not None and image_msg is not None:
                    start_time = time.time()
                    self._process_pointcloud(lidar_msg, image_msg)

                    # Track performance
                    process_time = time.time() - start_time
                    self._last_process_time = process_time
                    self._process_count += 1

                    if self._process_count % 30 == 0:  # Log every 30 frames
                        logger.debug(f"Processing time: {process_time * 1000:.1f}ms")
                else:
                    # Brief sleep to prevent CPU spinning
                    time.sleep(0.01)

            except Exception as e:
                logger.error(f"Error in processing loop: {e}", exc_info=True)
                time.sleep(0.1)  # Back off on error

        logger.info("Pointcloud processing loop terminated")

    def _process_pointcloud(self, lidar_msg: LidarMessage, image_msg: Image) -> None:
        """
        Core processing pipeline: transform, project, and publish.

        Args:
            lidar_msg: Input lidar pointcloud message
            image_msg: Input RGB image message
        """
        try:
            timestamp = lidar_msg.ts if lidar_msg.ts else time.time()

            # Get frame IDs from messages
            lidar_frame = lidar_msg.frame_id
            camera_frame = image_msg.frame_id

            # Lookup TF transform
            transform = self.tf.get(
                parent_frame=camera_frame,
                child_frame=lidar_frame,
                time_point=timestamp,
                time_tolerance=1.0,
            )

            if transform is None:
                logger.warning(f"Transform lookup failed: {lidar_frame} -> {camera_frame}")
                return

            # Create Transform message
            tf_msg = Transform(
                translation=transform.translation,
                rotation=transform.rotation,
                frame_id=lidar_frame,
                child_frame_id=camera_frame,
                ts=timestamp,
            )

            # Transform pointcloud to camera frame (vectorized operation)
            pointcloud_camera = transform_pointcloud(lidar_msg, tf_msg)

            # Publish transformed pointcloud
            if self.pointcloud_camera:
                self.pointcloud_camera.publish(pointcloud_camera)

            # Generate and publish depth image
            depth_image = self._generate_depth_image(pointcloud_camera, timestamp)
            if depth_image is not None and self.depth_image:
                self.depth_image.publish(depth_image)

            # Generate and publish overlay visualization using depth image
            if depth_image is not None:
                overlay_image = self._generate_overlay(depth_image, image_msg)
                if overlay_image is not None and self.pointcloud_overlay:
                    self.pointcloud_overlay.publish(overlay_image)
        except Exception as e:
            logger.error(f"Pointcloud processing failed: {e}", exc_info=True)

    def _generate_depth_image(self, pointcloud: PointCloud2, timestamp: float) -> Optional[Image]:
        """
        Generate depth image from pointcloud using efficient projection.

        Args:
            pointcloud: Pointcloud in camera frame
            timestamp: Message timestamp

        Returns:
            Depth image as Image message or None if generation fails
        """
        try:
            if self.image_width is None or self.image_height is None:
                return None

            # Extract points efficiently
            pcd = pointcloud.pointcloud
            points = np.asarray(pcd.points)

            if len(points) == 0:
                return None

            # Apply depth filtering AND filter points behind camera (z > 0)
            valid_mask = (points[:, 2] > self.min_depth) & (points[:, 2] < self.max_depth)
            valid_points = points[valid_mask]

            if len(valid_points) == 0:
                return None

            # Project 3D points to 2D image plane
            points_2d = project_3d_points_to_2d(valid_points, self.camera_intrinsics)

            # Initialize depth buffer
            depth_image = np.zeros((self.image_height, self.image_width), dtype=np.float32)

            # Filter points within image boundaries
            valid_pixels = (
                (points_2d[:, 0] >= 0)
                & (points_2d[:, 0] < self.image_width)
                & (points_2d[:, 1] >= 0)
                & (points_2d[:, 1] < self.image_height)
            )

            points_2d_valid = points_2d[valid_pixels]
            depths_valid = valid_points[valid_pixels, 2]

            # Efficient depth buffer filling using minimum depth per pixel
            for i in range(len(points_2d_valid)):
                x, y = points_2d_valid[i]
                depth = depths_valid[i]

                # Z-buffer: keep closest point
                if depth_image[y, x] == 0 or depth < depth_image[y, x]:
                    depth_image[y, x] = depth

            # Convert to uint16 millimeters for efficient transmission
            depth_mm = np.clip(depth_image * 1000, 0, 65535).astype(np.uint16)

            # Get camera frame from pointcloud (it was transformed to camera frame)
            camera_frame = pointcloud.frame_id

            return Image(
                data=depth_mm, format=ImageFormat.DEPTH16, frame_id=camera_frame, ts=timestamp
            )

        except Exception as e:
            logger.error(f"Depth image generation failed: {e}")
            return None

    def _generate_overlay(self, depth_image: Image, image_msg: Image) -> Optional[Image]:
        """
        Generate efficient visualization overlay using pre-computed depth image.

        Args:
            depth_image: Pre-computed depth image from lidar
            image_msg: RGB image for overlay

        Returns:
            Image with overlay visualization or None if generation fails
        """
        try:
            # Extract depth array from depth image (uint16 in mm)
            depth_mm = depth_image.data
            depth_m = depth_mm.astype(np.float32) / 1000.0

            # Create overlay image
            overlay = image_msg.data.copy()

            # Find non-zero depth pixels (subsample for performance)
            valid_pixels = np.where(depth_m > 0)

            if len(valid_pixels[0]) == 0:
                return image_msg

            # Subsample points for visualization (every Nth point)
            subsample_rate = max(1, len(valid_pixels[0]) // 5000)  # Max 5000 points
            y_coords = valid_pixels[0][::subsample_rate]
            x_coords = valid_pixels[1][::subsample_rate]

            # Use single cyan color for all points (fast, no colormap)
            color = (0, 255, 255)  # Cyan in RGB

            # Draw subsampled points
            for i in range(len(x_coords)):
                x, y = x_coords[i], y_coords[i]
                cv2.circle(overlay, (x, y), self.point_size, color, -1)

            # Add statistics overlay
            num_valid = np.count_nonzero(depth_m)
            stats_text = (
                f"Points: {num_valid:,} | Range: [{self.min_depth:.1f}, {self.max_depth:.1f}]m"
            )
            cv2.putText(
                overlay,
                stats_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            return Image(
                data=overlay, format=image_msg.format, frame_id=image_msg.frame_id, ts=image_msg.ts
            )

        except Exception as e:
            logger.error(f"Overlay generation failed: {e}")
            return None

    def cleanup(self) -> None:
        """Clean up resources on module destruction."""
        self.stop()
        logger.info("PointCloudToDepth module cleaned up")
