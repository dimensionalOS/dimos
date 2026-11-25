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
from collections.abc import AsyncIterator
from typing import Any

from pydantic import model_validator

from dimos.core.hardware_module import HardwareModuleConfig
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.sim2.mujoco.renders import RenderQuality
from dimos.sim2.mujoco.sensor_world import open_view, robot_ids
from dimos.sim2.mujoco.world import SensorMount
from dimos.sim2.sensor import SimSensor
from dimos.sim2.sensor_usd import camera_usd
from dimos.sim2.sim import until_it_stops


class SimCameraConfig(HardwareModuleConfig):
    width: int = 640
    height: int = 480
    fovy: float = 60.0
    rate_hz: float = 10.0
    # Lights cast shadows (MuJoCo's mjRND_SHADOW).
    shadows: bool = True
    # Shadow map texels per side; None: the scene's (MuJoCo's default 4096).
    shadow_size: int | None = None
    # Multisampling for anti-aliasing; None: the scene's (MuJoCo's default 4).
    samples: int | None = None
    # Reflective materials mirror the scene (mjRND_REFLECTION).
    reflections: bool = True
    # The dome's sky is drawn behind everything (mjRND_SKYBOX).
    skybox: bool = True
    # Materials show their textures (mjVIS_TEXTURE).
    textures: bool = True
    # The tone curve ("agx", "neutral", "linear", "none"); None: the one the stage's RenderSettings name.
    tone_map: str | None = None
    # MuJoCo's headlight; None: on only when the stage brings no lights.
    headlight: bool | None = None

    def quality(self) -> RenderQuality:
        """The render knobs above, as the view takes them."""
        return RenderQuality(
            shadows=self.shadows,
            shadow_size=self.shadow_size,
            samples=self.samples,
            reflections=self.reflections,
            skybox=self.skybox,
            textures=self.textures,
            tone_map=self.tone_map,
            headlight=self.headlight,
        )

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


class SimCamera(SimSensor):
    config: SimCameraConfig
    color_image: Out[Image]

    async def main(self) -> AsyncIterator[None]:
        self._world = self.mirror_world()
        self._capturing = asyncio.create_task(
            until_it_stops(f"{self.frame_id} camera", self._capture())
        )
        yield
        self._capturing.cancel()

    async def _capture(self) -> None:
        site, view = self.frame_id, None
        try:
            async for _ in self._world.ticks(self.config.rate_hz):
                if view is None:
                    layer = await asyncio.to_thread(self.world_layer)
                    view = open_view(
                        layer,
                        robot_ids(self._world),
                        SensorMount("camera", site, site, self.config.fovy),
                        self.config.quality(),
                    )
                    # Opening took this tick's sim time, so the next tick is the first read on time.
                    continue
                view.update(self._world)
                rgb = view.render(site, self.config.width, self.config.height)
                stamp = view.time
                self.color_image.publish(
                    Image(data=rgb, format=ImageFormat.RGB, frame_id=site, ts=stamp)
                )
                self.publish_usd_tf(ts=stamp)
        finally:
            if view is not None:
                view.close()
