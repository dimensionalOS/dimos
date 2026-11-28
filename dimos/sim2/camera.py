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

"""A simulated camera on any mount: a MuJoCo camera drawn with the stage's tone curve, sending what a real camera sends."""

import asyncio
from collections.abc import Coroutine
import math
from typing import Any

import numpy as np
from numpy.typing import NDArray
from pydantic import model_validator

from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.sim2.mujoco.renders import RenderQuality
from dimos.sim2.mujoco.sensor_mount import SensorMount
from dimos.sim2.sensor import SimSensorConfig
from dimos.sim2.sensor_usd import OPTICAL_FRAME, camera_usd
from dimos.sim2.view_sensor import ViewSensor
from dimos.spec import perception


class SimCameraConfig(SimSensorConfig):
    width: int = 640
    height: int = 480
    # Its vertical field of view, in degrees.
    fovy: float = 60.0
    rate_hz: float = 10.0
    # Its frames are drawn on this many views at once (each its own render process on macOS), so one draws while another is tone mapped.
    views: int = 4
    # Shadows, multisampling, reflections, sky, textures, tone curve and headlight.
    render: RenderQuality = RenderQuality()

    @model_validator(mode="before")
    @classmethod
    def _camera_with_this_fovy(cls, data: Any) -> Any:
        # Unless one is given, the camera's USD is as wide as the images it renders.
        if not isinstance(data, dict) or data.get("usd") is not None:
            return data
        fields = cls.model_fields
        fovy, width, height = (data.get(n, fields[n].default) for n in ("fovy", "width", "height"))
        usd = camera_usd("camera_link", name="sim_camera", fovy=fovy, aspect=width / height)
        return {**data, "usd": usd}


class SimCamera(ViewSensor, perception.Camera):
    """A camera on its mount, sending what a real camera module sends: color images and their camera_info, in its optical frame."""

    config: SimCameraConfig
    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    @property
    def optical_frame_id(self) -> str:
        """Its optical frame (z forward, x right, y down), which its images and camera_info are in."""
        return self.tf_frames[f"{self.config.usd.root_link}/{OPTICAL_FRAME}"]

    def loops(self) -> list[Coroutine[Any, Any, None]]:
        return [self._capture()]

    def lens(self) -> CameraInfo:
        """Its pinhole lens: square pixels, ``fovy`` tall, centred, no distortion."""
        config = self.config
        focal = config.height / 2 / math.tan(math.radians(config.fovy) / 2)
        cx, cy = config.width / 2, config.height / 2
        return CameraInfo(
            height=config.height,
            width=config.width,
            distortion_model="plumb_bob",
            D=[0.0] * 5,
            K=[focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0],
            R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            P=[focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
            frame_id=self.optical_frame_id,
        )

    async def _capture(self) -> None:
        config, optical = self.config, self.optical_frame_id
        lens = self.lens()
        # The world it views is composed once the sim runs, which its first tick says.
        async for _ in self._world.ticks(config.rate_hz):
            break
        mount = SensorMount("camera", optical, optical, config.fovy)
        view = await asyncio.to_thread(self.open_view, mount, config.render)
        drawers = [view.twin() for _ in range(config.views)]
        try:
            await self.film(
                drawers,
                optical,
                config.width,
                config.height,
                config.rate_hz,
                config.stamp_tolerance,
                lambda rgb, stamp: self._publish_frame(lens, rgb, stamp),
            )
        finally:
            for drawer in drawers:
                drawer.close()
            view.close()

    def _publish_frame(self, lens: CameraInfo, rgb: NDArray[np.uint8], stamp: float) -> None:
        self.color_image.publish(
            Image(data=rgb, format=ImageFormat.RGB, frame_id=lens.frame_id, ts=stamp)
        )
        self.camera_info.publish(lens.with_ts(stamp))
        self.publish_moving_tf(stamp)
