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

"""MuJoCo as a module: it takes actions, it publishes what moved, and it owns its own loop.

Reached through `sim_module("mujoco", ...)` rather than named directly, so nothing outside this
package has to know which engine is running.
"""

import asyncio
from collections.abc import AsyncIterator
from threading import Event, Thread
import time
from typing import Any

from dimos_lcm.world_msgs import WorldAction, WorldObservations

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.files.File import File
from dimos.msgs.std_msgs.String import String
from dimos.sim2.actions import read_actions
from dimos.sim2.mujoco.backend import MujocoBackend
from dimos.sim2.sim import SimulatedWorldConfig, scene_from
from dimos.sim2.spec import validate
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class MujocoSimConfig(SimulatedWorldConfig):
    # How much physics this world will catch up on in one go. It steps in bursts rather than one
    # step at a time because macOS oversleeps a short wait, and lag past one burst is dropped: a
    # controller's command must not land a whole control period late. Smaller means a world that
    # falls behind sooner but never runs ahead of what is watching it.
    max_burst_seconds: float = 0.01


class MujocoSim(Module):
    """One compiled MuJoCo world, stepped in real time, publishing a frame of what moved.

    A `SimulatedWorld` by shape rather than by inheritance -- actions in, observations out -- so
    that what core asks of a backend is something a native module could satisfy too.

    It steps only once every module has started: a module that misses a frame never gets it,
    because a frame says what *changed* and the first one is the only one that says a robot
    exists at all.
    """

    config: MujocoSimConfig
    # The contract core states, and all of it.
    actions: In[WorldAction]
    world_observations: Out[WorldObservations]
    # A request names the world's layer, or a file it names, and gets it back on file_info --
    # the same pair a UsdManager serves a robot's meshes on, for the same reason.
    render_file_request: In[String]
    file_info: Out[File]
    # The Controller's simulated world layer: it finds the world on scene_info, then asks on
    # scene_data_request. Geometry as JSON, which is this engine's idea of a scene and nobody
    # else's, so it lives here.
    scene_info: Out[String]
    scene_data_request: In[String]
    scene_data: Out[String]

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
        self._backend: MujocoBackend | None = None
        self._publisher: _LatestPublisher | None = None
        self._all_started = Event()

    async def main(self) -> AsyncIterator[None]:
        # Every action is applied: one dropped while busy would be a lost motor command.
        self.actions.subscribe(self._on_actions)
        self._publisher = _LatestPublisher(self.world_observations.publish)
        running = asyncio.create_task(self._run())
        announcing = asyncio.create_task(self._announce())
        yield
        announcing.cancel()
        running.cancel()

    def _engine(self) -> MujocoBackend:
        usd = scene_from(self.config.scene_manager, self.rpc, self.config.scene)
        if usd is None:
            raise ValueError(
                "nothing says what this world is: pass scene= a USD layer or a file holding one,"
                " or scene_manager= the UsdManager that composes it"
            )
        return MujocoBackend(
            usd,
            self.simulated,
            speed=self.config.speed,
            max_burst_seconds=self.config.max_burst_seconds,
        )

    @rpc
    def on_system_modules(self, _modules: list[Any]) -> None:
        self._all_started.set()

    async def _run(self) -> None:
        """Build the world, then step it, publishing what moved and holding at each tick.

        Built here rather than in `build()`, because building it means asking the manager for the
        scene and a manager that is not constructed yet cannot be asked. By the time a module is
        running, every module exists. On a thread, since the ask is blocking and fetching a
        room's worth of meshes should not stop this module answering anything else.
        """
        backend = self._backend = await asyncio.to_thread(self._engine)
        publisher = self._publisher
        assert publisher is not None
        await asyncio.to_thread(self._all_started.wait)
        while True:
            _when, frame = backend.step()
            if backend.waiting_for:
                # A tick's frame is what its controllers are waiting on, so it waits out a busy
                # publisher rather than being dropped the way an ordinary frame is.
                while publisher.busy:
                    await asyncio.sleep(0)
                publisher.offer(frame)
                deadline = time.monotonic() + self.config.controller_timeout
                while not backend.ready() and time.monotonic() < deadline:
                    await asyncio.sleep(0)
            elif not publisher.busy:
                publisher.offer(frame)
            await asyncio.sleep(0.001)

    async def _announce(self) -> None:
        while True:
            self.scene_info.publish(String("mujoco"))
            await asyncio.sleep(1.0)

    async def handle_scene_data_request(self, request: String) -> None:
        if self._backend is not None:
            scene = await asyncio.to_thread(self._backend.scene_data)
            self.scene_data.publish(String(scene))

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

    async def handle_render_file_request(self, request: String) -> None:
        if self._backend is None:
            return
        found = self._backend.assets().get(request.data)
        # Only files the world names: a request is never an arbitrary path into the filesystem.
        # Others on the topic (a UsdManager) may answer the rest.
        if found is None:
            logger.debug(f"render_file_request for {request.data!r}, which the world does not name")
            return
        data = found if isinstance(found, bytes) else await asyncio.to_thread(found.read_bytes)
        self.file_info.publish(File(request.data, data))

    @rpc
    def describe(self) -> dict[str, Any]:
        """The robots in this world, and where its layer landed. Nothing reads it to build one.

        A sensor used to: it asked for this and compiled the world the description named, which
        made the simulation something every renderer had to be able to reach. A sensor reads the
        composed scene instead, so this is left as what it sounds like -- something to look at
        when a world is not doing what it was expected to.
        """
        return {
            **(self._backend.describe() if self._backend is not None else {}),
            "simulated": list(self.simulated),
            "robots": sorted(d.robot_id for d in self.simulated),
        }

    @rpc
    def stop(self) -> None:
        super().stop()
        if self._publisher is not None:
            self._publisher.close()
            self._publisher = None
        self._backend = None


class _LatestPublisher:
    """Publishes on its own thread. A frame is offered only when it is not busy, so none is half-sent."""

    def __init__(self, publish: Any) -> None:
        self._publish = publish
        self._pending: WorldObservations | None = None
        self._wake = Event()
        self._closed = False
        self._thread = Thread(target=self._loop, name="world_observations", daemon=True)
        self._thread.start()

    @property
    def busy(self) -> bool:
        return self._wake.is_set()

    def offer(self, frame: WorldObservations) -> None:
        if not self.busy:
            self._pending = frame
            self._wake.set()

    def close(self) -> None:
        self._closed = True
        self._wake.set()
        self._thread.join()

    def _loop(self) -> None:
        while True:
            self._wake.wait()
            if self._closed:
                return
            try:
                self._publish(self._pending)
            except Exception:
                # A frame that fails to publish must not stop the world.
                logger.exception("sim2: publishing a frame failed")
            self._wake.clear()
