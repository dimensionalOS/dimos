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


"""The MuJoCo backend: one PhysicsBackend, stepped in real time (times ``speed``)."""

import asyncio
from collections.abc import AsyncIterator
from threading import Event, Lock, Thread
from typing import Any

from dimos_lcm.world_msgs import WorldAction, WorldObservations

from dimos.core.core import rpc
from dimos.core.hardware_module import HardwareSpec
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.files.File import File
from dimos.msgs.std_msgs.String import String
from dimos.sim2.actions import JointCommand, read_actions
from dimos.sim2.mujoco.backend import MujocoBackend
from dimos.sim2.sim import SimulatedRobotSpec, compose_world, until_it_stops
from dimos.sim2.spec import validate
from dimos.sim2.world_observations import merged
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class MujocoSimConfig(ModuleConfig):
    speed: float = 1.0
    # Most sim time one step() catches up on; past it the world runs slower than real time instead of spiralling.
    max_burst_seconds: float = 0.01


class MujocoSim(Module):
    config: MujocoSimConfig
    actions: In[WorldAction]
    world_observations: Out[WorldObservations]
    render_file_request: In[String]
    file_info: Out[File]
    hardware: list[HardwareSpec]
    robots: list[SimulatedRobotSpec]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._backend: MujocoBackend | None = None
        self._publisher: _LatestPublisher | None = None
        self._built = Event()
        self._commanded: set[str] = set()
        self._held: set[str] = set()
        self._all_commanded = asyncio.Event()

    async def main(self) -> AsyncIterator[None]:
        self.actions.subscribe(self._on_actions)
        self._publisher = _LatestPublisher(self.world_observations.publish)
        running = asyncio.create_task(until_it_stops("the world", self._run()))
        yield
        running.cancel()

    @rpc
    def on_system_modules(self, _modules: list[Any]) -> None:
        simulated = [robot.simulated_robot() for robot in self.robots]
        validate(simulated)
        self._held = {robot.robot_id for robot in simulated if robot.held_until_commanded}
        self._backend = MujocoBackend(
            compose_world(self.hardware),
            simulated,
            speed=self.config.speed,
            max_burst_seconds=self.config.max_burst_seconds,
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
        return self._backend.world.snapshot()

    async def _run(self) -> None:
        await asyncio.to_thread(self._built.wait)
        backend, publisher = self._backend, self._publisher
        assert backend is not None and publisher is not None
        # Held still until every held robot is commanded; each command re-offers the frame its policy reads joints from.
        publisher.offer(self._first_frame)
        if self._held:
            await self._all_commanded.wait()
        logger.info("sim2: every robot is being driven; starting the world")
        backend.restart_clock()
        while True:
            _when, frame = backend.step()
            publisher.offer(frame)
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
        if not self._all_commanded.is_set():
            self._commanded |= {a.entity for a in actions if isinstance(a, JointCommand)}
            if self._held <= self._commanded:
                # main() runs on the module's loop, so the event it awaits is set from there.
                loop = self._loop
                assert loop is not None
                loop.call_soon_threadsafe(self._all_commanded.set)
            elif self._publisher is not None:
                self._publisher.offer(backend.frame())

    async def handle_render_file_request(self, request: String) -> None:
        if self._backend is None:
            return
        found = self._backend.assets().get(request.data)
        if found is None:
            logger.debug(f"render_file_request for {request.data!r}, which the world does not name")
            return
        data = found if isinstance(found, bytes) else await asyncio.to_thread(found.read_bytes)
        self.file_info.publish(File(request.data, data))

    @rpc
    def describe(self) -> dict[str, Any]:
        return self._backend.describe() if self._backend is not None else {}

    @rpc
    def stop(self) -> None:
        super().stop()
        # Releases a get_world_usd() caller blocked on a world that is never going to be built.
        self._built.set()
        if self._publisher is not None:
            self._publisher.close()
            self._publisher = None
        self._backend = None


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
