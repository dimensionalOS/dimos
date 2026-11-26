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

import asyncio
from collections.abc import Coroutine
from typing import Any

from pydantic import model_validator

from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.sim2.mujoco.renders import RenderQuality
from dimos.sim2.mujoco.sensor_mount import SensorMount
from dimos.sim2.sensor import SimSensorConfig
from dimos.sim2.sensor_usd import camera_usd
from dimos.sim2.view_sensor import ViewSensor


class SimCameraConfig(SimSensorConfig):
    width: int = 640
    height: int = 480
    fovy: float = 60.0
    rate_hz: float = 10.0
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


class SimCamera(ViewSensor):
    config: SimCameraConfig
    color_image: Out[Image]

    def loops(self) -> list[Coroutine[Any, Any, None]]:
        return [self._capture()]

    async def _capture(self) -> None:
        site, view = self.frame_id, None
        try:
            async for _ in self._world.ticks(self.config.rate_hz):
                if view is None:
                    mount = SensorMount("camera", site, site, self.config.fovy)
                    view = await asyncio.to_thread(self.open_view, mount, self.config.render)
                    # Opening took this tick, so the next tick is the first read on time.
                    continue
                view.update(self._world)
                rgb = view.render(site, self.config.width, self.config.height)
                stamp = view.time
                period, tolerance = 1 / self.config.rate_hz, self.config.stamp_tolerance
                if not self.on_time("color_image", stamp, period, tolerance):
                    continue
                self.color_image.publish(
                    Image(data=rgb, format=ImageFormat.RGB, frame_id=site, ts=stamp)
                )
                self.publish_moving_tf(stamp)
        finally:
            if view is not None:
                view.close()
