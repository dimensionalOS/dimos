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

"""The DimSim backend: one SimWorld, its world run by DimSim's own engine (Rapier), stepped in real time (times ``speed``)."""

import asyncio
from typing import Any

from dimos.core.module import ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.files.File import File
from dimos.msgs.std_msgs.String import String
from dimos.robot.assets.model import UsdModel
from dimos.sim2.dimsim.backend import DimSimBackend
from dimos.sim2.sim import SimWorld
from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.world_files import WorldFiles


class DimSimSimConfig(ModuleConfig):
    # Seconds of world time per Rapier step, as DimSim's own server physics steps (50 Hz).
    timestep: float = 0.02
    speed: float = 1.0
    # Most steps one step() catches up on; past it the world runs slower than real time instead of spiralling.
    max_burst_steps: int = 5


class DimSimSim(SimWorld):
    """The DimSim backend: robots are boxes DimSim's character controller slides on their x, y and yaw servos; the rest is Rapier rigid bodies."""

    config: DimSimSimConfig
    render_file_request: In[String]
    file_info: Out[File]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._files: WorldFiles | None = None

    def _build(self, world: UsdModel, simulated: list[SimulatedRobot]) -> DimSimBackend:
        self._files = WorldFiles(world.usd)
        return DimSimBackend(
            world,
            simulated,
            timestep=self.config.timestep,
            speed=self.config.speed,
            max_burst_steps=self.config.max_burst_steps,
        )

    async def _run(self) -> None:
        await asyncio.to_thread(self._built.wait)
        backend = self._backend
        if not isinstance(backend, DimSimBackend):
            return
        await asyncio.to_thread(self.world_observations.publish, self._first_frame)
        backend.restart_clock()
        while True:
            _when, frame = await asyncio.to_thread(backend.step)
            await asyncio.to_thread(self.world_observations.publish, frame)
            await asyncio.sleep(backend.timestep)

    async def handle_render_file_request(self, request: String) -> None:
        if self._files is None:
            return
        await self._files.answer(request.data, self.file_info.publish)
