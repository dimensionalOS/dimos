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

# Copyright 2025 Dimensional Inc.

"""Camera module for drone with depth estimation."""

import threading
import time
from typing import Any

from dimos_lcm.sensor_msgs import CameraInfo  # type: ignore[import-untyped]

from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos.msgs.std_msgs import Header
from dimos.perception.common.utils import colorize_depth
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class DroneCameraModule(Module):
    """
    Camera module for drone that processes RGB images to generate depth using Metric3D.

    Subscribes to:
        - /video: RGB camera images from drone

    Publishes:
        - /drone/color_image: RGB camera images
        - /drone/depth_image: Depth images from Metric3D
        - /drone/depth_colorized: Colorized depth
        - /drone/camera_info: Camera calibration
        - /drone/camera_pose: Camera pose from TF
    """

    # Inputs
    video: In[Image]

    # Outputs
    color_image: Out[Image]
    depth_image: Out[Image]
    depth_colorized: Out[Image]
    camera_info: Out[CameraInfo]
    camera_pose: Out[PoseStamped]

    def __init__(
        self,
        camera_intrinsics: list[float],
        world_frame_id: str = "world",
        camera_frame_id: str = "camera_link",
        base_frame_id: str = "base_link",
        gt_depth_scale: float = 2.0,
        **kwargs: Any,
    ) -> None:
        """Initialize drone camera module.

        Args:
            camera_intrinsics: [fx, fy, cx, cy]
            camera_frame_id: TF frame for camera
            base_frame_id: TF frame for drone base
            gt_depth_scale: Depth scale factor
        """
        super().__init__(**kwargs)

        if len(camera_intrinsics) != 4:
            raise ValueError("Camera intrinsics must be [fx, fy, cx, cy]")

        self.camera_intrinsics = camera_intrinsics
        self.camera_frame_id = camera_frame_id
        self.base_frame_id = base_frame_id
        self.world_frame_id = world_frame_id
        self.gt_depth_scale = gt_depth_scale

        # Metric3D for depth
        self.metric3d: Any = None  # Lazy-loaded Metric3D model

        # Processing state
        self._running = False
        self._latest_frame: Image | None = None
        self._processing_thread: threading.Thread | None = None
        self._stop_processing = threading.Event()

        logger.info(f"DroneCameraModule initialized with intrinsics: {camera_intrinsics}")

    @rpc
    def start(self) -> bool:
        """Start the camera module."""
        if self._running:
            logger.warning("Camera module already running")
            return True

        # Start processing thread for depth (which will init Metric3D and handle video)
        self._running = True
        self._stop_processing.clear()
        self._processing_thread = threading.Thread(target=self._processing_loop, daemon=True)
        self._processing_thread.start()

        logger.info("Camera module started")
        return True

    def _on_video_frame(self, frame: Image) -> None:
        """Handle incoming video frame."""
        if not self._running:
            return

        # Publish color image immediately
        self.color_image.publish(frame)

        # Store for depth processing
        self._latest_frame = frame

    def _processing_loop(self) -> None:
        """Process depth estimation in background."""
        # Initialize Metric3D in the background thread
        if self.metric3d is None:
            try:
                from dimos.models.depth.metric3d import Metric3D

                self.metric3d = Metric3D(camera_intrinsics=self.camera_intrinsics)
                logger.info("Metric3D initialized")
            except Exception as e:
                logger.warning(f"Metric3D not available: {e}")
                self.metric3d = None

        # Subscribe to video once connection is available
        subscribed = False
        while not subscribed and not self._stop_processing.is_set():
            try:
                if self.video.connection is not None:
                    self.video.subscribe(self._on_video_frame)
                    subscribed = True
                    logger.info("Subscribed to video input")
                else:
                    time.sleep(0.1)
            except Exception as e:
                logger.debug(f"Waiting for video connection: {e}")
                time.sleep(0.1)

        logger.info("Depth processing loop started")

        _reported_error = False

        while not self._stop_processing.is_set():
            if self._latest_frame is not None and self.metric3d is not None:
                try:
                    frame = self._latest_frame
                    self._latest_frame = None

                    # Get numpy array from Image
                    img_array = frame.data

                    # Generate depth
                    depth_array = self.metric3d.infer_depth(img_array) / self.gt_depth_scale

                    # Create header
                    header = Header(self.camera_frame_id)

                    # Publish depth
                    depth_msg = Image(
                        data=depth_array,
                        format=ImageFormat.DEPTH,
                        frame_id=header.frame_id,
                        ts=header.ts,
                    )
                    self.depth_image.publish(depth_msg)

                    # Publish colorized depth
                    depth_colorized_array = colorize_depth(
                        depth_array, max_depth=10.0, overlay_stats=True
                    )
                    if depth_colorized_array is not None:
                        depth_colorized_msg = Image(
                            data=depth_colorized_array,
                            format=ImageFormat.RGB,
                            frame_id=header.frame_id,
                            ts=header.ts,
                        )
                        self.depth_colorized.publish(depth_colorized_msg)

                    # Publish camera info
                    self._publish_camera_info(header, img_array.shape)

                    # Publish camera pose
                    self._publish_camera_pose(header)

                except Exception as e:
                    if not _reported_error:
                        _reported_error = True
                        logger.error(f"Error processing depth: {e}")
            else:
                time.sleep(0.01)

        logger.info("Depth processing loop stopped")

    def _publish_camera_info(self, header: Header, shape: tuple[int, ...]) -> None:
        """Publish camera calibration info."""
        try:
            fx, fy, cx, cy = self.camera_intrinsics
            height, width = shape[:2]

            # Camera matrix K (3x3)
            K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]

            # No distortion for now
            D = [0.0, 0.0, 0.0, 0.0, 0.0]

            # Identity rotation
            R = [1, 0, 0, 0, 1, 0, 0, 0, 1]

            # Projection matrix P (3x4)
            P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]

            msg = CameraInfo(
                D_length=len(D),
                header=header,
                height=height,
                width=width,
                distortion_model="plumb_bob",
                D=D,
                K=K,
                R=R,
                P=P,
                binning_x=0,
                binning_y=0,
            )

            self.camera_info.publish(msg)

        except Exception as e:
            logger.error(f"Error publishing camera info: {e}")

    def _publish_camera_pose(self, header: Header) -> None:
        """Publish camera pose from TF."""
        try:
            transform = self.tf.get(
                parent_frame=self.world_frame_id,
                child_frame=self.camera_frame_id,
                time_point=header.ts,
                time_tolerance=1.0,
            )

            if transform:
                pose_msg = PoseStamped(
                    ts=header.ts,
                    frame_id=self.camera_frame_id,
                    position=transform.translation,
                    orientation=transform.rotation,
                )
                self.camera_pose.publish(pose_msg)

        except Exception as e:
            logger.error(f"Error publishing camera pose: {e}")

    @rpc
    def stop(self) -> None:
        """Stop the camera module."""
        if not self._running:
            return

        self._running = False
        self._stop_processing.set()

        # Wait for thread
        if self._processing_thread and self._processing_thread.is_alive():
            self._processing_thread.join(timeout=2.0)

        # Cleanup Metric3D
        if self.metric3d:
            self.metric3d.cleanup()

        logger.info("Camera module stopped")
