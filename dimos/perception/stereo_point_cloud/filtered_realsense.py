# Copyright 2025-2026 Dimensional Inc.
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

"""RealSenseCamera subclass that applies rs.spatial_filter + rs.hole_filling_filter
before converting depth frames to numpy — identical to the original realsense_stereo_nav.py."""

from __future__ import annotations

import time

import cv2
import numpy as np

from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class FilteredRealSenseCamera(RealSenseCamera):
    """Adds rs.spatial_filter + rs.hole_filling_filter to depth frames before publishing."""

    def _capture_loop(self) -> None:
        import pyrealsense2 as rs

        spatial_filter = rs.spatial_filter()
        # No hole_filling_filter: it interpolates a plausible-looking depth value
        # wherever the sensor has no real return — which is exactly what happens in
        # occlusion shadows behind objects. That fabricated depth was showing up as
        # a "sea of voxels" behind real geometry that the camera never actually saw.
        # Leaving those pixels invalid lets the existing NaN/invalid-depth handling
        # in StereoPointCloud._on_depth correctly drop them instead of mapping them.

        while self._running and self._pipeline is not None:
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=1000)
            except (RuntimeError, AttributeError) as e:
                logger.error(f"FilteredRealSenseCamera: capture loop exiting, wait_for_frames failed: {e}")
                break

            ts = time.time()

            if self._align is not None:
                frames = self._align.process(frames)

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame() if self.config.enable_depth else None

            if depth_frame:
                depth_frame = spatial_filter.process(depth_frame)

            color_img = None
            if color_frame and (self.config.publish_color or self.config.enable_pointcloud):
                color_data = np.asanyarray(color_frame.get_data())
                color_data = cv2.cvtColor(color_data, cv2.COLOR_BGR2RGB)
                color_img = Image(
                    data=color_data,
                    format=ImageFormat.RGB,
                    frame_id=self._color_optical_frame,
                    ts=ts,
                )
                if self.config.publish_color:
                    self.color_image.publish(color_img)

            depth_img = None
            if depth_frame:
                depth_data = np.asanyarray(depth_frame.get_data())
                depth_frame_id = (
                    self._color_optical_frame
                    if self.config.align_depth_to_color
                    else self._depth_optical_frame
                )
                depth_img = Image(
                    data=depth_data,
                    format=ImageFormat.DEPTH16,
                    frame_id=depth_frame_id,
                    ts=ts,
                )
                self.depth_image.publish(depth_img)

            if self.config.enable_pointcloud and color_img is not None and depth_img is not None:
                with self._pointcloud_lock:
                    self._latest_color_img = color_img
                    self._latest_depth_img = depth_img

            self._publish_tf(ts)
