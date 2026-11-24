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
from dimos.core.hardware_module import HardwareSpec
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.sim2.actions import read_actions
from dimos.sim2.dimsim.backend import DimSimBackend
from dimos.sim2.sim import SimulatedRobotSpec, compose_world, until_it_stops
from dimos.sim2.spec import validate
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class DimSimSimConfig(ModuleConfig):
    # Seconds of world time per Rapier step, as DimSim's own server physics steps (50 Hz).
    timestep: float = 0.02
    speed: float = 1.0
    # Most steps one step() catches up on; past it the world runs slower than real time instead of spiralling.
    max_burst_steps: int = 5
    # Farthest a robot picks an object up from, in meters: DimSim's own interact distance.
    reach: float = 1.5


class DimSimSim(Module):
    """The DimSim backend: one PhysicsBackend, its world run by DimSim's own engine."""

    config: DimSimSimConfig
    actions: In[WorldAction]
    world_observations: Out[WorldObservations]
    hardware: list[HardwareSpec]
    robots: list[SimulatedRobotSpec]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._backend: DimSimBackend | None = None
        self._built = Event()

    async def main(self) -> AsyncIterator[None]:
        self.actions.subscribe(self._on_actions)
        running = asyncio.create_task(until_it_stops("the world", self._run()))
        yield
        running.cancel()

    @rpc
    def on_system_modules(self, _modules: list[Any]) -> None:
        simulated = [robot.simulated_robot() for robot in self.robots]
        validate(simulated)
        self._backend = DimSimBackend(
            compose_world(self.hardware),
            simulated,
            timestep=self.config.timestep,
            speed=self.config.speed,
            max_burst_steps=self.config.max_burst_steps,
            reach=self.config.reach,
        )
        # The first frame exists before anyone can ask for a snapshot, so a snapshot always has one.
        self._first_frame = self._backend.frame()
        self._built.set()

    @rpc
    def get_world_usd(self) -> str:
        self._built.wait()
        assert self._backend is not None
        return self._backend.usd

    @rpc
    def snapshot(self) -> dict[str, Any]:
        self._built.wait()
        assert self._backend is not None
        return self._backend.snapshot()

    async def _run(self) -> None:
        await asyncio.to_thread(self._built.wait)
        backend = self._backend
        assert backend is not None
        await asyncio.to_thread(self.world_observations.publish, self._first_frame)
        backend.restart_clock()
        while True:
            _when, frame = await asyncio.to_thread(backend.step)
            await asyncio.to_thread(self.world_observations.publish, frame)
            await asyncio.sleep(backend.timestep)

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
        return self._backend.describe() if self._backend is not None else {}

    @rpc
    def stop(self) -> None:
        super().stop()
        # Releases a get_world_usd() caller blocked on a world that is never going to be built.
        self._built.set()
        if self._backend is not None:
            self._backend.close()
            self._backend = None
