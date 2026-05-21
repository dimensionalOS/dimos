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

"""Camera ``Module`` that publishes caller-supplied color frames and ``CameraInfo``.

Implemented as a normal ``dimos.perception.fiducial`` submodule so
``ModuleCoordinator`` workers can import the class when it is deployed (not
as a class nested under a pytest-collected module path).
"""

from __future__ import annotations

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image


class ManualFrameCameraModule(Module):
    """Streams ``Image`` and ``CameraInfo`` provided via ``publish_frame``."""

    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @rpc
    def publish_frame(self, image: Image, info: CameraInfo) -> None:
        self.camera_info.publish(info)
        self.color_image.publish(image)
