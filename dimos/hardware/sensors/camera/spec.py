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
from typing import Generic, Protocol, TypeVar

from reactivex.observable import Observable

from dimos.msgs.geometry_msgs import Quaternion, Transform
from dimos.msgs.sensor_msgs import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.protocol.service import Configurable  # type: ignore[attr-defined]

OPTICAL_ROTATION = Quaternion(-0.5, 0.5, -0.5, 0.5)


class CameraConfig(Protocol):
    frame_id_prefix: str | None


CameraConfigT = TypeVar("CameraConfigT", bound=CameraConfig)


class CameraHardware(ABC, Configurable[CameraConfigT], Generic[CameraConfigT]):
    @abstractmethod
    def image_stream(self) -> Observable[Image]:
        pass

    @property
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


class StereoCamera(ABC):
    """Abstract class for stereo camera modules (RealSense, ZED, etc.)."""

    @abstractmethod
    def get_color_camera_info(self) -> CameraInfo | None:
        """Get color camera intrinsics."""
        pass

    @abstractmethod
    def get_depth_camera_info(self) -> CameraInfo | None:
        """Get depth camera intrinsics."""
        pass

    @abstractmethod
    def get_depth_scale(self) -> float:
        """Get the depth scale factor (meters per unit)."""
        pass

    @property
    @abstractmethod
    def _camera_link(self) -> str:
        pass

    @property
    @abstractmethod
    def _color_frame(self) -> str:
        pass

    @property
    @abstractmethod
    def _color_optical_frame(self) -> str:
        pass

    @property
    @abstractmethod
    def _depth_frame(self) -> str:
        pass

    @property
    @abstractmethod
    def _depth_optical_frame(self) -> str:
        pass
