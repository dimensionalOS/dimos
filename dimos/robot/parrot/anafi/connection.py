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

from __future__ import annotations

from typing import Any

from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.parrot.anafi.vendor_connection import (
    AnafiConnection,
    AnafiConnectionProtocol,
    FakeAnafiConnection,
)
from dimos.spec.perception import Camera
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class AnafiConnectionConfig(ModuleConfig):
    ip_address: str = "192.168.42.1"
    num_retries: int = 10
    replay: bool = False
    velocity_gain: float = 50.0
    yaw_rate_gain: float = 50.0
    video_buffer_size: int = 30
    camera_frame_id: str = "camera_optical"


def _camera_info_static(config: AnafiConnectionConfig) -> CameraInfo:
    fx, fy, cx, cy = (933.0, 933.0, 640.0, 360.0)
    width, height = (1280, 720)

    return CameraInfo.from_intrinsics(
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        width=width,
        height=height,
        frame_id=config.camera_frame_id,
    )


class AnafiConnectionModule(Module, Camera):
    config: AnafiConnectionConfig

    cmd_vel: In[Twist]
    odom: Out[PoseStamped]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]
    telemetry: Out[Any]

    connection: AnafiConnectionProtocol | None = None
    _camera_info: CameraInfo
    _latest_video_frame: Image | None = None
    _latest_telemetry: dict[str, Any] | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._camera_info = _camera_info_static(self.config)

    @rpc
    def start(self) -> None:
        super().start()

        if self.config.replay:
            self.connection = FakeAnafiConnection()
        else:
            self.connection = AnafiConnection(
                ip_address=self.config.ip_address,
                num_retries=self.config.num_retries,
                velocity_gain=self.config.velocity_gain,
                yaw_rate_gain=self.config.yaw_rate_gain,
                video_buffer_size=self.config.video_buffer_size,
            )

        if not self.connection.connect():
            logger.error(
                f"{self.__class__.__name__}: failed to connect at {self.config.ip_address}"
            )
            return

        self.register_disposable(self.connection.odom_stream().subscribe(self.odom.publish))
        self.register_disposable(self.connection.video_stream().subscribe(self._on_color_image))
        self.register_disposable(self.connection.telemetry_stream().subscribe(self._on_telemetry))
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))

        logger.info(f"{self.__class__.__name__} started")

    @rpc
    def stop(self) -> None:
        if self.connection is not None:
            self.connection.disconnect()
            self.connection = None
        logger.info(f"{self.__class__.__name__} stopped")
        super().stop()

    @skill
    def takeoff(self, timeout: float = 5.0) -> bool:
        """Arm the drone and lift off to its default hover altitude."""
        return self.connection is not None and self.connection.takeoff(timeout)

    @skill
    def land(self, timeout: float = 5.0) -> bool:
        """Land the drone safely at its current position."""
        return self.connection is not None and self.connection.land(timeout)

    @skill
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send a velocity command to the drone.

        Args:
            twist: Linear/angular velocity in ROS conventions
                (x = forward, y = left, z = up; angular.z = yaw rate).
            duration: Seconds to hold the command (0 = single shot).
        """
        return self.connection is not None and self.connection.move(twist, duration)

    @skill
    def move_relative(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        dyaw: float = 0.0,
    ) -> bool:
        """Move a fixed offset relative to the current pose.

        Args:
            dx: Forward displacement in meters (negative = backward).
            dy: Right displacement in meters (negative = left).
            dz: Down displacement in meters (negative = up).
            dyaw: Yaw rotation in radians (clockwise positive).
        """
        return self.connection is not None and self.connection.move_relative(dx, dy, dz, dyaw)

    @skill
    def stop_motion(self) -> bool:
        """Bring the drone to a hover by issuing a zero-velocity command."""
        return self.move(Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))

    @skill
    def emergency_stop(self) -> bool:
        """Cut motors immediately. The drone will fall — use only as a last resort."""
        return self.connection is not None and self.connection.emergency_stop()

    @skill
    def observe(self) -> Image | None:
        """Return the latest camera frame, or ``None`` if none received yet."""
        return self._latest_video_frame

    @rpc
    def get_camera_info(self) -> CameraInfo:
        """Return the static ``CameraInfo`` used for the front camera stream."""
        return self._camera_info

    @rpc
    def get_telemetry(self) -> dict[str, Any] | None:
        """Return the most recent raw telemetry batch from pyparrot."""
        return self._latest_telemetry

    def _on_color_image(self, frame: Image) -> None:
        self._latest_video_frame = frame
        self.color_image.publish(frame)
        self.camera_info.publish(self._camera_info.with_ts(frame.ts))

    def _on_telemetry(self, t: dict[str, Any]) -> None:
        self._latest_telemetry = t
        self.telemetry.publish(t)


anafi_connection = AnafiConnectionModule.blueprint


__all__ = ["AnafiConnectionModule", "anafi_connection"]
