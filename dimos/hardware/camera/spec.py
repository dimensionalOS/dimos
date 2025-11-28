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

from abc import ABC, abstractmethod, abstractproperty
from typing import Generic, Protocol, TypeVar

from dimos_lcm.sensor_msgs import CameraInfo  # type: ignore[import-untyped]
from reactivex.observable import Observable

from dimos.msgs.sensor_msgs import Image
from dimos.protocol.service import Configurable  # type: ignore[attr-defined]


class CameraConfig(Protocol):
    frame_id_prefix: str | None


CameraConfigT = TypeVar("CameraConfigT", bound=CameraConfig)


class CameraHardware(ABC, Configurable[CameraConfigT], Generic[CameraConfigT]):
    @abstractmethod
    def image_stream(self) -> Observable[Image]:
        pass

    @abstractproperty
    def camera_info(self) -> CameraInfo:
        pass


# This is an example, feel free to change spec for stereo cameras
# e.g., separate camera_info or streams for left/right, etc.
class StereoCameraHardware(ABC, Configurable[CameraConfigT], Generic[CameraConfigT]):
    @abstractmethod
    def image_stream(self) -> Observable[Image]:
        pass

    @abstractmethod
    def depth_stream(self) -> Observable[Image]:
        pass

    @abstractproperty
    def camera_info(self) -> CameraInfo:
        pass
