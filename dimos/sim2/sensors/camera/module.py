# Copyright 2026 Dimensional Inc.
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

"""Camera device description and public outputs, served by a shared rendering worker."""

from typing import Any, cast

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image
from dimos.sim2.sensors.camera.config import CameraConfig, camera_usd


class CameraModuleConfig(ModuleConfig):
    sensor: CameraConfig
    renderer_name: str = "CameraRenderModule"


class CompressedCameraModuleConfig(CameraModuleConfig):
    # The quality CompressedImage.from_image uses, so a simulated camera and a
    # hardware driver that encodes with the default send comparable bytes.
    jpeg_quality: int = Field(default=75, ge=1, le=100)


class _SimCameraDevice(Module):
    """What the rendering worker needs of a camera device, whatever its color encoding."""

    config: CameraModuleConfig
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]

    @rpc
    def get_usd(self) -> list[str]:
        return ["", camera_usd(self.config.sensor)]

    @rpc
    def jpeg_quality(self) -> int | None:
        """JPEG quality of the color output; None when color leaves as raw pixels."""
        return None

    @rpc
    def camera_config(self, renderer_name: str) -> CameraConfig:
        """Return the optical configuration to this device's selected renderer."""
        if renderer_name != self.config.renderer_name:
            raise ValueError(f"camera belongs to renderer {self.config.renderer_name!r}")
        return self.config.sensor

    @rpc
    def build(self) -> None:
        super().build()
        self._renderer = RPCClient(
            None, None, self.config.renderer_name, {"camera_statistics"}, rpc=self.rpc
        )
        self.register_disposable(Disposable(self._renderer.stop_rpc_client))

    @rpc
    def start(self) -> None:
        # All builds have completed: fail if the selected owner omitted this device.
        self._renderer.camera_statistics(self.config.sensor.frame_id)
        super().start()

    @rpc
    def statistics(self) -> dict[str, Any]:
        return cast("dict[str, Any]", self._renderer.camera_statistics(self.config.sensor.frame_id))


class SimCameraModule(_SimCameraDevice):
    """RGB-D camera whose color leaves as raw pixels."""

    color_image: Out[Image]


class SimCompressedCameraModule(_SimCameraDevice):
    """RGB-D camera whose color leaves as JPEG, as from a driver that publishes encoder bytes.

    Depth stays raw: JPEG cannot carry metric depth. Recorders keep the bytes as
    they are and browsers decode them directly, with no transcoding in between.
    """

    config: CompressedCameraModuleConfig
    color_image: Out[CompressedImage]

    @rpc
    def jpeg_quality(self) -> int | None:
        return self.config.jpeg_quality
