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

"""An RGB camera, rendered from the world_change frames by its own view of the world."""

import asyncio
from collections.abc import AsyncIterator
from typing import Any

from dimos_lcm.world_msgs import WorldChange

from dimos.core.module import Module, ModuleConfig
from dimos.core.shared_config import shared_config
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.sim2.module import Simulation, SimulationSpec
from dimos.sim2.spec import Mount, SimulatedSensor
from dimos.sim2.view import WorldView, open_view
from dimos.sim2.world_change import WorldMirror


class SimCameraConfig(ModuleConfig):
    name: str = "camera"
    mount: Mount = Mount("world")
    robot_id: str | None = None
    frame_id: str | None = "camera_optical"
    width: int = 640
    height: int = 480
    fovy: float = 60.0
    rate_hz: float = 10.0


class SimCamera(Module):
    config: SimCameraConfig
    color_image: Out[Image]
    world_change: In[WorldChange]
    _simulation: SimulationSpec

    @shared_config(Simulation)
    def simulated(self) -> dict[str, Any]:
        config = self.config
        return {
            "simulated": SimulatedSensor(
                config.name, "camera", config.mount, config.robot_id, config.fovy
            )
        }

    async def main(self) -> AsyncIterator[None]:
        self._world = WorldMirror()
        self._world.subscribe(self.world_change)
        view = open_view(self._simulation.describe())
        self._capturing = asyncio.create_task(self._capture(view))
        yield
        self._capturing.cancel()
        view.close()

    async def _capture(self, view: WorldView) -> None:
        camera = f"{self.config.robot_id or 'world'}/{self.config.name}"
        # rate_hz of sim time, so a world slower or faster than real gets as many frames per sim second.
        async for _ in self._world.ticks(self.config.rate_hz):
            view.update(self._world)
            rgb = view.render(camera, self.config.width, self.config.height)
            self.color_image.publish(
                Image(
                    data=rgb, format=ImageFormat.RGB, frame_id=self.frame_id, ts=self._world.stamp()
                )
            )
