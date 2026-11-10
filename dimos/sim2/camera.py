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
from pathlib import Path

from dimos_lcm.world_msgs import WorldObservations

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.sim2.mujoco.sensor_world import WorldView, open_view, robot_ids
from dimos.sim2.mujoco.world import SensorMount
from dimos.sim2.sensor_usd import camera_usd
from dimos.sim2.sim import SceneConfig, scene_from
from dimos.sim2.usd_layer import usd_layer
from dimos.sim2.world_observations import WorldMirror
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class SimCameraConfig(SceneConfig):
    frame_id: str = "camera_optical"
    width: int = 640
    height: int = 480
    fovy: float = 60.0
    rate_hz: float = 10.0


class SimCamera(Module):
    config: SimCameraConfig
    color_image: Out[Image]
    world_observations: In[WorldObservations]

    @rpc
    def get_usd(self) -> list[str]:
        return camera_usd(self.frame_id, name="sim_camera", fovy=self.config.fovy)

    async def main(self) -> AsyncIterator[None]:
        self._world = WorldMirror()
        self._world.subscribe(self.world_observations)
        self._capturing = asyncio.create_task(self._watched_capture())
        yield
        self._capturing.cancel()

    async def _watched_capture(self) -> None:
        """Say so when the capture loop dies.

        It is a bare task, so anything it raises is kept in the task object and nothing ever
        reads it: the camera simply stops publishing and the only evidence is an empty topic.
        A renderer that cannot start -- no display, a GL context that nine workers raced each
        other for -- looks from the outside exactly like a camera whose rate is set to zero.
        """
        try:
            await self._capture()
        except asyncio.CancelledError:
            raise
        except BaseException as error:
            logger.exception(f"sim2: {self.frame_id} stopped capturing: {error!r}")
            raise

    async def _capture(self) -> None:
        scene = scene_from(self.config.scene_manager, self.rpc, self.config.scene)
        if scene is None:
            raise ValueError(
                f"{self.frame_id}: nothing says what the scene is, so there is nothing to"
                f" render. Give this camera the scene_manager that composes the world."
            )
        layer, view = usd_layer(scene), None
        try:
            async for _ in self._world.ticks(self.config.rate_hz):
                if view is None:
                    view = self._open(layer)
                view.update(self._world)
                rgb = view.render(self._name, self.config.width, self.config.height)
                self.color_image.publish(
                    Image(
                        data=rgb,
                        format=ImageFormat.RGB,
                        frame_id=self.frame_id,
                        ts=self._world.stamp(),
                    )
                )
        finally:
            if view is not None:
                view.close()

    def _open(self, layer: Path) -> WorldView:
        return open_view(
            layer,
            robot_ids(self._world),
            SensorMount("camera", self.frame_id, self._name, self.config.fovy),
        )

    @property
    def _name(self) -> str:
        return self.frame_id
