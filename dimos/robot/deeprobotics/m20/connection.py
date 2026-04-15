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

"""M20 Connection Module for dimos.

Handles camera (RTSP) and robot state management (heartbeat, gait, usage
mode) for the Deep Robotics M20 quadruped. Velocity commands and sensor
data (lidar, IMU, odometry) are handled by separate NativeModules
(NavCmdPub, DrddsLidarBridge, AriseSLAM).

Reference: M20 Software Development Guide
"""

import logging
import time
from threading import Thread
from typing import Any

from dimos import spec
from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.core.global_config import GlobalConfig, global_config
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image

from ..protocol import (
    GaitType,
    M20Protocol,
    MotionState,
    UsageMode,
)
from .camera import M20RTSPCamera

logger = logging.getLogger(__name__)


class M20Connection(Module, spec.Camera):
    """Deep Robotics M20 quadruped connection.

    Manages camera streaming (RTSP) and robot state (stand, sit, gait,
    navigation mode) via UDP protocol. Velocity commands are handled by
    NavCmdPub NativeModule. Sensor data comes from DrddsLidarBridge +
    AriseSLAM.

    Streams:
        color_image (Out): RGB camera frames (via RTSP)
        camera_info (Out): Camera intrinsics
    """

    # Output streams
    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    # Internal state
    _protocol: M20Protocol
    _global_config: GlobalConfig
    _camera_info_thread: Thread | None = None
    _camera_info_running: bool = False
    _latest_video_frame: Image | None = None

    def __init__(
        self,
        global_config: GlobalConfig = global_config,
        ip: str | None = None,
        port: int = 30000,
        enable_camera: bool = True,
        camera_stream: str = "video1",
        *args: Any,
        **kwargs: Any,
    ) -> None:
        self._global_config = global_config

        ip = ip if ip is not None else self._global_config.robot_ip

        self._protocol = M20Protocol(host=ip, port=port)

        self._camera: M20RTSPCamera | None = None
        if enable_camera:
            self._camera = M20RTSPCamera(
                host=ip, stream_path=camera_stream
            )
            self._camera_info = self._camera.camera_info

        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self) -> None:
        super().start()

        try:
            # Connect to M20 and start protocol services (always UDP)
            self._protocol.connect()
            self._protocol.start_heartbeat()
            self._protocol.start_listener(self._on_status_report)

            # Start RTSP camera
            if self._camera:
                self._camera.start()

                def _on_image(image: Image) -> None:
                    self.color_image.publish(image)
                    self._latest_video_frame = image

                self._disposables.add(
                    self._camera.image_stream().subscribe(_on_image)
                )

                self._camera_info_running = True
                self._camera_info_thread = Thread(
                    target=self._publish_camera_info, daemon=True
                )
                self._camera_info_thread.start()

            # Stand up in navigation mode with agile gait
            self._protocol.send_motion_state(MotionState.STAND)
            time.sleep(1.0)
            self._protocol.send_usage_mode(UsageMode.NAVIGATION)
            self._protocol.send_gait_switch(GaitType.AGILE_FLAT)

            logger.info("M20Connection started in Navigation Mode")
        except Exception:
            logger.exception("M20Connection.start() failed — cleaning up")
            self.stop()
            raise

    @rpc
    def stop(self) -> None:
        """Sit down, stop camera, and close UDP connection."""
        try:
            self._protocol.send_usage_mode(UsageMode.REGULAR)
            self._protocol.send_motion_state(MotionState.SIT)
            time.sleep(1.0)
        except Exception:
            logger.exception("Failed to send sit-down during stop")

        try:
            if self._camera:
                self._camera.stop()
        except Exception:
            logger.exception("Failed to stop camera")

        self._camera_info_running = False
        if self._camera_info_thread and self._camera_info_thread.is_alive():
            self._camera_info_thread.join(timeout=1.0)

        try:
            self._protocol.close()
        except Exception:
            logger.exception("Failed to close protocol")

        super().stop()

    def _on_status_report(self, report: dict) -> None:
        """Handle incoming UDP status reports (battery, fault, motion state)."""
        logger.debug(f"M20 status: type={report.get('type')} items={report.get('items')}")

    def _publish_camera_info(self) -> None:
        """Publish camera intrinsics at 1Hz while camera is running."""
        while self._camera_info_running and self._camera and self._camera._running:
            self.camera_info.publish(self._camera.camera_info.with_ts(time.time()))
            time.sleep(1.0)

    # --- RPC commands ---

    @rpc
    def standup(self) -> bool:
        """Make the robot stand up."""
        self._protocol.send_motion_state(MotionState.STAND)
        return True

    @rpc
    def sitdown(self) -> bool:
        """Make the robot sit down."""
        self._protocol.send_motion_state(MotionState.SIT)
        return True

    @rpc
    def set_gait(self, gait: int) -> bool:
        """Switch gait mode. Standard=0x1001, HighObs=0x1002, Stairs=0x1003, Agile=0x3002."""
        self._protocol.send_gait_switch(gait)
        return True

    @skill
    def observe(self) -> Image | None:
        """Returns the latest video frame from the robot camera."""
        return self._latest_video_frame


m20_connection = M20Connection.blueprint

__all__ = ["M20Connection", "m20_connection"]
