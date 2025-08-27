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
FakeZEDModule - Replays recorded ZED data for testing without hardware.
"""

import functools
import logging
import numpy as np
import open3d as o3d

from dimos.core import Module, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos_lcm.sensor_msgs import CameraInfo
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.utils.data import get_data
from dimos.utils.testing import TimedSensorReplay
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__name__, level=logging.INFO)


class FakeZEDModule(Module):
    """
    Fake ZED module that replays recorded data instead of real camera.
    """

    pointcloud_msg: Out[LidarMessage] = None
    pose: Out[PoseStamped] = None
    color_image: Out[Image] = None
    depth_image: Out[Image] = None
    camera_info: Out[CameraInfo] = None

    def __init__(self, recording_path: str = None, frame_id: str = "camera_link", *args, **kwargs):
        """
        Initialize FakeZEDModule with recording path.

        Args:
            recording_path: Path to recorded data directory
        """
        Module.__init__(self, *args, **kwargs)

        self.recording_path = recording_path
        self.frame_id = frame_id
        get_data(self.recording_path)
        if not self.recording_path:
            self.recording_path = "zed"

        self._running = False
        self._subscriptions = []

        logger.info(f"FakeZEDModule initialized with recording: {self.recording_path}")

    @functools.cache
    def _get_lidar_stream(self):
        """Get cached lidar stream."""
        logger.info(f"Loading lidar stream from {self.recording_path}/pointcloud")

        def pointcloud_autocast(x):
            """Convert raw pointcloud data to LidarMessage."""
            if isinstance(x, np.ndarray):
                import time

                points = x.reshape(-1, 4)[:, :3]  # Extract XYZ
                valid = np.isfinite(points).all(axis=1)
                valid_points = points[valid]
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(valid_points)
                return LidarMessage(
                    pointcloud=pcd,
                    origin=[0, 0, 0],
                    resolution=0.05,
                    ts=time.time(),
                    frame_id=self.frame_id,
                )
            elif isinstance(x, LidarMessage):
                return x
            return x

        lidar_replay = TimedSensorReplay(
            f"{self.recording_path}/pointcloud", autocast=pointcloud_autocast
        )
        return lidar_replay.stream()

    @functools.cache
    def _get_pose_stream(self):
        """Get cached pose stream."""
        logger.info(f"Loading pose stream from {self.recording_path}/pose")

        def pose_autocast(x):
            """Convert raw pose dict to PoseStamped."""
            if isinstance(x, dict):
                import time

                return PoseStamped(
                    position=x.get("position", [0, 0, 0]),
                    orientation=x.get("rotation", [0, 0, 0, 1]),
                    ts=time.time(),
                )
            elif isinstance(x, PoseStamped):
                return x
            return x

        pose_replay = TimedSensorReplay(f"{self.recording_path}/pose", autocast=pose_autocast)
        return pose_replay.stream()

    @functools.cache
    def _get_color_stream(self):
        """Get cached color image stream."""
        logger.info(f"Loading color image stream from {self.recording_path}/color")

        def image_autocast(x):
            """Convert raw numpy array to Image."""
            if isinstance(x, np.ndarray):
                import cv2

                rgb_data = cv2.cvtColor(x, cv2.COLOR_BGR2RGB)
                return Image(data=rgb_data, format=ImageFormat.RGB)
            elif isinstance(x, Image):
                return x
            return x

        color_replay = TimedSensorReplay(f"{self.recording_path}/color", autocast=image_autocast)
        return color_replay.stream()

    @functools.cache
    def _get_depth_stream(self):
        """Get cached depth image stream."""
        logger.info(f"Loading depth image stream from {self.recording_path}/depth")

        def depth_autocast(x):
            """Convert raw numpy array to depth Image."""
            if isinstance(x, np.ndarray):
                # Depth images are float32
                return Image(data=x, format=ImageFormat.DEPTH)
            elif isinstance(x, Image):
                return x
            return x

        depth_replay = TimedSensorReplay(f"{self.recording_path}/depth", autocast=depth_autocast)
        return depth_replay.stream()

    @functools.cache
    def _get_camera_info_stream(self):
        """Get cached camera info stream."""
        logger.info(f"Loading camera info stream from {self.recording_path}/camera_info")
        info_replay = TimedSensorReplay(
            f"{self.recording_path}/camera_info",
        )
        return info_replay.stream()

    @rpc
    def start(self):
        """Start replaying recorded data."""
        if self._running:
            return

        logger.info("Starting FakeZEDModule replay...")

        self._running = True

        # Subscribe to all streams and publish
        try:
            # Lidar/pointcloud stream
            sub = self._get_lidar_stream().subscribe(
                lambda msg: self.pointcloud_msg.publish(msg) if self._running else None
            )
            self._subscriptions.append(sub)
            logger.info("Started lidar replay stream")
        except Exception as e:
            logger.warning(f"Lidar stream not available: {e}")

        try:
            # Pose stream
            sub = self._get_pose_stream().subscribe(
                lambda msg: self.pose.publish(msg) if self._running else None
            )
            self._subscriptions.append(sub)
            logger.info("Started pose replay stream")
        except Exception as e:
            logger.warning(f"Pose stream not available: {e}")

        try:
            # Color image stream
            sub = self._get_color_stream().subscribe(
                lambda msg: self.color_image.publish(msg) if self._running else None
            )
            self._subscriptions.append(sub)
            logger.info("Started color image replay stream")
        except Exception as e:
            logger.warning(f"Color image stream not available: {e}")

        try:
            # Depth image stream
            sub = self._get_depth_stream().subscribe(
                lambda msg: self.depth_image.publish(msg) if self._running else None
            )
            self._subscriptions.append(sub)
            logger.info("Started depth image replay stream")
        except Exception as e:
            logger.warning(f"Depth image stream not available: {e}")

        try:
            # Camera info stream
            sub = self._get_camera_info_stream().subscribe(
                lambda msg: self.camera_info.publish(msg) if self._running else None
            )
            self._subscriptions.append(sub)
            logger.info("Started camera info replay stream")
        except Exception as e:
            logger.warning(f"Camera info stream not available: {e}")

        logger.info("FakeZEDModule replay started successfully")

    @rpc
    def stop(self):
        """Stop replaying."""
        if not self._running:
            return

        self._running = False

        # Dispose all subscriptions
        for sub in self._subscriptions:
            if sub:
                sub.dispose()
        self._subscriptions.clear()

        logger.info("FakeZEDModule stopped")

    def record(self, recording_name: str):
        """Fake record method for compatibility."""
        logger.info(f"Record called with {recording_name} (no-op in replay mode)")
