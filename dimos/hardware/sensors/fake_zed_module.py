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

from dataclasses import dataclass
import functools
import logging

from dimos_lcm.sensor_msgs import CameraInfo  # type: ignore[import-untyped]
import numpy as np

from dimos.core import Module, ModuleConfig, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos.msgs.std_msgs import Header
from dimos.protocol.tf import TF
from dimos.utils.logging_config import setup_logger
from dimos.utils.testing import TimedSensorReplay

logger = setup_logger(level=logging.INFO)


@dataclass
class FakeZEDModuleConfig(ModuleConfig):
    frame_id: str = "zed_camera"


class FakeZEDModule(Module[FakeZEDModuleConfig]):
    """
    Fake ZED module that replays recorded data instead of real camera.
    """

    # Define LCM outputs (same as ZEDModule)
    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    pose: Out[PoseStamped]

    default_config = FakeZEDModuleConfig
    config: FakeZEDModuleConfig

    def __init__(self, recording_path: str, **kwargs: object) -> None:
        """
        Initialize FakeZEDModule with recording path.

        Args:
            recording_path: Path to recorded data directory
        """
        super().__init__(**kwargs)

        self.recording_path = recording_path
        self._running = False

        # Initialize TF publisher
        self.tf = TF()

        logger.info(f"FakeZEDModule initialized with recording: {self.recording_path}")

    @functools.cache
    def _get_color_stream(self):  # type: ignore[no-untyped-def]
        """Get cached color image stream."""
        logger.info(f"Loading color image stream from {self.recording_path}/color")

        def image_autocast(x):  # type: ignore[no-untyped-def]
            """Convert raw numpy array to Image."""
            if isinstance(x, np.ndarray):
                return Image(data=x, format=ImageFormat.RGB)
            elif isinstance(x, Image):
                return x
            return x

        color_replay = TimedSensorReplay(f"{self.recording_path}/color", autocast=image_autocast)
        return color_replay.stream()

    @functools.cache
    def _get_depth_stream(self):  # type: ignore[no-untyped-def]
        """Get cached depth image stream."""
        logger.info(f"Loading depth image stream from {self.recording_path}/depth")

        def depth_autocast(x):  # type: ignore[no-untyped-def]
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
    def _get_pose_stream(self):  # type: ignore[no-untyped-def]
        """Get cached pose stream."""
        logger.info(f"Loading pose stream from {self.recording_path}/pose")

        def pose_autocast(x):  # type: ignore[no-untyped-def]
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
    def _get_camera_info_stream(self):  # type: ignore[no-untyped-def]
        """Get cached camera info stream."""
        logger.info(f"Loading camera info stream from {self.recording_path}/camera_info")

        def camera_info_autocast(x):  # type: ignore[no-untyped-def]
            """Convert raw camera info dict to CameraInfo message."""
            if isinstance(x, dict):
                # Extract calibration parameters
                left_cam = x.get("left_cam", {})
                resolution = x.get("resolution", {})

                # Create CameraInfo message
                header = Header(self.frame_id)

                # Create camera matrix K (3x3)
                K = [
                    left_cam.get("fx", 0),
                    0,
                    left_cam.get("cx", 0),
                    0,
                    left_cam.get("fy", 0),
                    left_cam.get("cy", 0),
                    0,
                    0,
                    1,
                ]

                # Distortion coefficients
                D = [
                    left_cam.get("k1", 0),
                    left_cam.get("k2", 0),
                    left_cam.get("p1", 0),
                    left_cam.get("p2", 0),
                    left_cam.get("k3", 0),
                ]

                # Identity rotation matrix
                R = [1, 0, 0, 0, 1, 0, 0, 0, 1]

                # Projection matrix P (3x4)
                P = [
                    left_cam.get("fx", 0),
                    0,
                    left_cam.get("cx", 0),
                    0,
                    0,
                    left_cam.get("fy", 0),
                    left_cam.get("cy", 0),
                    0,
                    0,
                    0,
                    1,
                    0,
                ]

                return CameraInfo(
                    D_length=len(D),
                    header=header,
                    height=resolution.get("height", 0),
                    width=resolution.get("width", 0),
                    distortion_model="plumb_bob",
                    D=D,
                    K=K,
                    R=R,
                    P=P,
                    binning_x=0,
                    binning_y=0,
                )
            elif isinstance(x, CameraInfo):
                return x
            return x

        info_replay = TimedSensorReplay(
            f"{self.recording_path}/camera_info", autocast=camera_info_autocast
        )
        return info_replay.stream()

    @rpc
    def start(self) -> None:
        """Start replaying recorded data."""
        super().start()

        if self._running:
            logger.warning("FakeZEDModule already running")
            return

        logger.info("Starting FakeZEDModule replay...")

        self._running = True

        # Subscribe to all streams and publish
        try:
            # Color image stream
            unsub = self._get_color_stream().subscribe(
                lambda msg: self.color_image.publish(msg) if self._running else None
            )
            self._disposables.add(unsub)
            logger.info("Started color image replay stream")
        except Exception as e:
            logger.warning(f"Color image stream not available: {e}")

        try:
            # Depth image stream
            unsub = self._get_depth_stream().subscribe(
                lambda msg: self.depth_image.publish(msg) if self._running else None
            )
            self._disposables.add(unsub)
            logger.info("Started depth image replay stream")
        except Exception as e:
            logger.warning(f"Depth image stream not available: {e}")

        try:
            # Pose stream
            unsub = self._get_pose_stream().subscribe(
                lambda msg: self._publish_pose(msg) if self._running else None
            )
            self._disposables.add(unsub)
            logger.info("Started pose replay stream")
        except Exception as e:
            logger.warning(f"Pose stream not available: {e}")

        try:
            # Camera info stream
            unsub = self._get_camera_info_stream().subscribe(
                lambda msg: self.camera_info.publish(msg) if self._running else None
            )
            self._disposables.add(unsub)
            logger.info("Started camera info replay stream")
        except Exception as e:
            logger.warning(f"Camera info stream not available: {e}")

        logger.info("FakeZEDModule replay started")

    @rpc
    def stop(self) -> None:
        if not self._running:
            return

        self._running = False

        super().stop()

    def _publish_pose(self, msg) -> None:  # type: ignore[no-untyped-def]
        """Publish pose and TF transform."""
        if msg:
            self.pose.publish(msg)

            # Publish TF transform from world to camera
            import time

            from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3

            transform = Transform(
                translation=Vector3(*msg.position),
                rotation=Quaternion(*msg.orientation),
                frame_id="world",
                child_frame_id=self.frame_id,
                ts=time.time(),
            )
            self.tf.publish(transform)
