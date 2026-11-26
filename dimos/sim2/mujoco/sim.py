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


"""The MuJoCo backend: one SimWorld, stepped in real time (times ``speed``)."""

import asyncio
from collections.abc import AsyncIterator
from threading import Event, Lock, Thread
from typing import Any

from dimos_lcm.world_msgs import WorldObservations

from dimos.core.core import rpc
from dimos.core.module import ModuleConfig
from dimos.sim2.actions import Action, JointCommand
from dimos.sim2.mujoco.backend import MujocoBackend
from dimos.sim2.sim import SimWorld, compose_world
from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.world_observations import merged
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class MujocoSimConfig(ModuleConfig):
    speed: float = 1.0
    # Most sim time one step() catches up on; past it the world runs slower than real time instead of spiralling.
    max_burst_seconds: float = 0.01


class MujocoSim(SimWorld):
    config: MujocoSimConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._publisher: _LatestPublisher | None = None
        self._composed = Event()
        self._world_usd = ""
        self._commanded: set[str] = set()
        self._held: set[str] = set()
        self._all_commanded = Event()

    async def main(self) -> AsyncIterator[None]:
        self._publisher = _LatestPublisher(self.world_observations.publish)
        async for _ in super().main():
            yield

    def _build(self, simulated: list[SimulatedRobot], hardware: list[Any]) -> MujocoBackend:
        self._held = {robot.robot_id for robot in simulated if robot.held_until_commanded}
        world = compose_world(hardware)
        # Sensors build their views from the composed world alone, so they start on it while physics compiles.
        self._world_usd = world.usd
        self._composed.set()
        return MujocoBackend(
            world,
            simulated,
            speed=self.config.speed,
            max_burst_seconds=self.config.max_burst_seconds,
        )

    @rpc
    def get_world_usd(self) -> str:
        self._composed.wait()
        return self._world_usd

    async def _run(self) -> None:
        await asyncio.to_thread(self._built.wait)
        backend, publisher = self._backend, self._publisher
        assert isinstance(backend, MujocoBackend) and publisher is not None
        publisher.offer(self._first_frame)
        backend.restart_clock()
        # Held still until every held robot is commanded, with the clock running so the readings its policy waits on stay fresh.
        held = True
        while True:
            if held and (not self._held or self._all_commanded.is_set()):
                held = False
                logger.info("sim2: every robot is being driven; starting the world")
            _when, frame = backend.step(hold=held)
            publisher.offer(frame)
            await asyncio.sleep(backend.timestep)

    def _applied(self, actions: list[Action]) -> None:
        if not self._all_commanded.is_set():
            self._commanded |= {a.entity for a in actions if isinstance(a, JointCommand)}
            if self._held <= self._commanded:
                self._all_commanded.set()

    @rpc
    def stop(self) -> None:
        super().stop()
        # Releases a get_world_usd() caller blocked on a world that is never going to be built.
        self._composed.set()
        if self._publisher is not None:
            self._publisher.close()
            self._publisher = None


class _LatestPublisher:
    """Publishes on its own thread; frames offered while one is in flight merge into the next."""

    def __init__(self, publish: Any) -> None:
        self._publish = publish
        self._pending: WorldObservations | None = None
        self._lock = Lock()
        self._wake = Event()
        self._closed = False
        self._thread = Thread(target=self._loop, name="world_observations", daemon=True)
        self._thread.start()

    def offer(self, frame: WorldObservations) -> None:
        # A frame carries only what changed, so a later one cannot stand in for it.
        with self._lock:
            self._pending = frame if self._pending is None else merged(self._pending, frame)
            self._wake.set()

    def close(self) -> None:
        self._closed = True
        self._wake.set()
        self._thread.join()

    def _loop(self) -> None:
        while True:
            self._wake.wait()
            with self._lock:
                self._wake.clear()
                frame, self._pending = self._pending, None
            if self._closed:
                return
            if frame is None:
                continue
            try:
                self._publish(frame)
            except Exception:
                logger.exception("sim2: publishing a frame failed")
