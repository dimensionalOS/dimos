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

from dataclasses import replace
from typing import Any, cast

from pydantic import Field, model_validator

from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.CompressedImage import CompressedImage
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.assets.model import UsdModel
from dimos.sim2.hardware import SimHardwareConfig, SimHardwareModule
from dimos.sim2.sensors.camera.config import CameraConfig, camera_usd


class CameraModuleConfig(SimHardwareConfig):
    sensor: CameraConfig
    renderer_name: str = "CameraRenderModule"

    @model_validator(mode="before")
    @classmethod
    def describe_camera(cls, data: Any) -> Any:
        if isinstance(data, dict) and data.get("usd") is None:
            sensor = data["sensor"]
            if isinstance(sensor, dict):
                sensor = CameraConfig(**sensor)
            model = UsdModel.from_layer(camera_usd(sensor))
            return {**data, "usd": model, "sensor": replace(sensor, frame_id=model.root_link)}
        return data


class CompressedCameraModuleConfig(CameraModuleConfig):
    # The quality CompressedImage.from_image uses, so a simulated camera and a
    # hardware driver that encodes with the default send comparable bytes.
    jpeg_quality: int = Field(default=75, ge=1, le=100)


class _SimCameraDevice(SimHardwareModule):
    """What the rendering worker needs of a camera device, whatever it publishes."""

    config: CameraModuleConfig
    camera_info: Out[CameraInfo]

    @rpc
    def jpeg_quality(self) -> int | None:
        """JPEG quality of the color output; None when color leaves as raw pixels."""
        return None

    @rpc
    def camera_config(self, renderer_name: str) -> CameraConfig:
        """Return the optical configuration to this device's selected renderer."""
        if renderer_name != self.config.renderer_name:
            raise ValueError(f"camera belongs to renderer {self.config.renderer_name!r}")
        return replace(self.config.sensor, frame_id=self.tf_frames[self.config.sensor.frame_id])

    @rpc
    def on_system_modules(self, modules: list[Any]) -> None:
        matches = [m for m in modules if m.instance_name == self.config.renderer_name]
        if len(matches) != 1 or "camera_statistics" not in matches[0].rpcs:
            raise ValueError(f"no camera renderer named {self.config.renderer_name!r}")
        self._renderer = matches[0]

    @rpc
    def on_system_modules_ready(self, modules: list[Any]) -> None:
        # All renderers have configured their devices by this discovery phase.
        self._renderer.camera_statistics(self.camera_config(self.config.renderer_name).frame_id)

    @rpc
    def statistics(self) -> dict[str, Any]:
        return cast(
            "dict[str, Any]",
            self._renderer.camera_statistics(
                self.camera_config(self.config.renderer_name).frame_id
            ),
        )


class SimCameraModule(_SimCameraDevice):
    """RGB-D camera whose color leaves as raw pixels."""

    color_image: Out[Image]
    depth_image: Out[Image]


class _JpegColorDevice(_SimCameraDevice):
    """A device whose color leaves as JPEG, as from a driver that publishes encoder bytes.

    Recorders keep the bytes as they are and browsers decode them directly, with
    no transcoding in between.
    """

    config: CompressedCameraModuleConfig
    color_image: Out[CompressedImage]

    @rpc
    def jpeg_quality(self) -> int | None:
        return self.config.jpeg_quality


class SimCompressedCameraModule(_JpegColorDevice):
    """RGB-D camera with JPEG color. Depth stays raw: JPEG cannot carry metric depth."""

    depth_image: Out[Image]


class SimColorCameraModule(_JpegColorDevice):
    """Video camera: JPEG color and no depth. Its sensor is a ``CameraConfig(depth=False)``."""


class SimDepthCameraModule(_SimCameraDevice):
    """Depth camera: metric depth and no color. Its sensor is a ``CameraConfig(color=False)``."""

    depth_image: Out[Image]
