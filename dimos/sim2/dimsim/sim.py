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
from threading import Event
from typing import Any

from dimos_lcm.world_msgs import WorldAction, WorldObservations

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.sim2.actions import read_actions
from dimos.sim2.dimsim.backend import TIMESTEP, DimSimBackend
from dimos.sim2.sim import SimulatedWorldConfig, scene_from, until_it_stops
from dimos.sim2.spec import validate
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class DimSimSimConfig(SimulatedWorldConfig):
    max_burst_steps: int = 5


class DimSimSim(Module):
    config: DimSimSimConfig
    actions: In[WorldAction]
    world_observations: Out[WorldObservations]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # Before validate, because stop() reads both: a duplicate robot id would otherwise come
        # out as an AttributeError from the cleanup and say nothing about the configuration.
        self._backend: DimSimBackend | None = None
        self._all_started = Event()
        try:
            validate(self.config.robots)
        except Exception:
            self.stop()
            raise
        self.simulated = self.config.robots

    async def main(self) -> AsyncIterator[None]:
        self.actions.subscribe(self._on_actions)
        running = asyncio.create_task(until_it_stops("the world", self._run()))
        yield
        running.cancel()

    def _engine(self) -> DimSimBackend:
        usd = scene_from(self.config.scene_manager, self.rpc, self.config.scene)
        if usd is None:
            raise ValueError(
                "nothing says what this world is: pass scene= a USD layer or a file holding one,"
                " or scene_manager= the UsdManager that composes it"
            )
        return DimSimBackend(
            usd,
            self.simulated,
            speed=self.config.speed,
            max_burst_steps=self.config.max_burst_steps,
        )

    @rpc
    def on_system_modules(self, _modules: list[Any]) -> None:
        self._all_started.set()

    async def _run(self) -> None:
        backend = self._backend = await asyncio.to_thread(self._engine)
        await asyncio.to_thread(self._all_started.wait)
        period = TIMESTEP / max(self.config.speed, 1e-6)
        while True:
            _when, frame = await asyncio.to_thread(backend.step)
            await asyncio.to_thread(self.world_observations.publish, frame)
            await asyncio.sleep(period)

    def _on_actions(self, message: WorldAction) -> None:
        backend = self._backend
        if backend is None:
            return
        try:
            actions = read_actions(message, backend.world)
        except (KeyError, ValueError) as error:
            logger.warning(f"sim2: an action this world cannot read: {error}")
            return
        backend.apply(actions)

    @rpc
    def describe(self) -> dict[str, Any]:
        return {
            **(self._backend.describe() if self._backend is not None else {}),
            "simulated": list(self.simulated),
            "robots": sorted(d.robot_id for d in self.simulated),
        }

    @rpc
    def stop(self) -> None:
        super().stop()
        # _run waits on this in a worker thread before it steps anything, and cancelling the
        # task does not reach a thread already blocked in Event.wait -- so a world stopped
        # before on_system_modules arrives would keep that thread, and the Python worker it
        # belongs to, alive forever.
        self._all_started.set()
        if self._backend is not None:
            self._backend.close()
            self._backend = None
