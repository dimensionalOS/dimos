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

"""DimSim as a module: it takes actions, it publishes what moved, and it owns its own loop.

Reached through `sim_module("dimsim", ...)` rather than named directly, so nothing outside this
package has to know which engine is running.
"""

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
from dimos.sim2.sim import SimulatedWorldConfig, scene_from
from dimos.sim2.spec import validate
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class DimSimSimConfig(SimulatedWorldConfig):
    # How many steps of physics this world will catch up on in one go. It steps in bursts rather
    # than one step at a time because macOS oversleeps a short wait, and lag past one burst is
    # dropped: a command must not land a whole control period late.
    max_burst_steps: int = 5


class DimSimSim(Module):
    """One Rapier world in a Deno subprocess, stepped in real time, publishing a frame of what moved.

    A `SimulatedWorld` by shape rather than by inheritance -- actions in, observations out -- so
    that what core asks of a backend is something a native module could satisfy too.

    It steps only once every module has started: a module that misses a frame never gets it,
    because a frame says what *changed* and the first one is the only one that says a robot exists
    at all.
    """

    config: DimSimSimConfig
    # The contract core states, and all of it.
    actions: In[WorldAction]
    world_observations: Out[WorldObservations]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        try:
            validate(self.config.robots)
        except Exception:
            # Nothing else will stop a module that threw in its own constructor, and it has
            # already started the threads `Module.__init__` starts.
            self.stop()
            raise
        self.simulated = self.config.robots
        self._backend: DimSimBackend | None = None
        self._all_started = Event()

    async def main(self) -> AsyncIterator[None]:
        # Every action is applied: one dropped while busy would be a lost velocity command.
        self.actions.subscribe(self._on_actions)
        running = asyncio.create_task(self._run())
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
        """Build the world, then step it, publishing what moved and pacing on the clock.

        Built here rather than in `build()`, because building it means asking the manager for the
        scene and a manager that is not constructed yet cannot be asked. By the time a module is
        running, every module exists. On a thread, since the ask is blocking and fetching a room's
        worth of meshes should not stop this module answering anything else.
        """
        backend = self._backend = await asyncio.to_thread(self._engine)
        await asyncio.to_thread(self._all_started.wait)
        period = TIMESTEP / max(self.config.speed, 1e-6)
        while True:
            _when, frame = await asyncio.to_thread(backend.step)
            # Straight from the loop and not off a publisher thread: a step is a round trip to the
            # Deno world, so a frame is never ready while the one before it is still going out.
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
        """The robots in this world, and where its layer landed. Nothing reads it to build one."""
        return {
            **(self._backend.describe() if self._backend is not None else {}),
            "simulated": list(self.simulated),
            "robots": sorted(d.robot_id for d in self.simulated),
        }

    @rpc
    def stop(self) -> None:
        super().stop()
        if self._backend is not None:
            self._backend.close()
            self._backend = None
