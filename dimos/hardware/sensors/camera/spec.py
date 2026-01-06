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

from abc import ABC, abstractmethod
from enum import Enum
from typing import Generic, Protocol, TypeVar

from reactivex.observable import Observable

from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.service import Configurable


class SensorStatus(Enum):
    """Status returned by sensor start/stop operations."""

    STARTED = "started"
    STOPPED = "stopped"
    FAILED = "failed"


OPTICAL_ROTATION = Quaternion(-0.5, 0.5, -0.5, 0.5)


def default_base_transform() -> Transform:
    """Default identity transform for camera mounting."""
    return Transform(
        translation=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )


class CameraConfig(Protocol):
    frame_id_prefix: str | None


CameraConfigT = TypeVar("CameraConfigT", bound=CameraConfig)


class CameraHardware(ABC, Configurable[CameraConfigT], Generic[CameraConfigT]):
    @abstractmethod
    def image_stream(self) -> Observable[Image]:
        pass

    @abstractmethod
    def camera_info(self) -> CameraInfo:
        pass


class StereoCameraConfig(Protocol):
    """Protocol for stereo camera configuration."""

    width: int
    height: int
    fps: int
    camera_name: str
    base_frame_id: str
    base_transform: Transform | None
    align_depth_to_color: bool
    enable_depth: bool
    enable_pointcloud: bool
    pointcloud_fps: float
    camera_info_fps: float


StereoCameraConfigT = TypeVar("StereoCameraConfigT", bound=StereoCameraConfig)


class StereoCameraHardware(Protocol[StereoCameraConfigT]):
    """Protocol for stereo cameras (RealSense, ZED, etc.)."""

    config: StereoCameraConfigT

    def color_image_stream(self) -> Observable[Image]:
        """Get the color image stream."""
        ...

    def depth_image_stream(self) -> Observable[Image]:
        """Get the depth image stream."""
        ...

    def pointcloud_stream(self) -> Observable[PointCloud2]:
        """Get the pointcloud stream."""
        ...

    def color_camera_info(self) -> CameraInfo:
        """Get color camera intrinsics."""
        ...

    def depth_camera_info(self) -> CameraInfo:
        """Get depth camera intrinsics."""
        ...

    def depth_scale(self) -> float:
        """Get the depth scale factor (meters per unit)."""
        ...

    @property
    def _camera_link(self) -> str: ...

    @property
    def _color_frame(self) -> str: ...

    @property
    def _color_optical_frame(self) -> str: ...

    @property
    def _depth_frame(self) -> str: ...

    @property
    def _depth_optical_frame(self) -> str: ...

    def start(self) -> SensorStatus:
        """Start the camera and begin streaming."""
        ...

    def stop(self) -> None:
        """Stop the camera."""
        ...
