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

Reached through `sim_module("mujoco", ...)` rather than named directly, and described to as
`@shared_config(SimulatedWorld)` rather than as this class, so nothing outside this package has
to know which engine is running.
"""

import asyncio
from collections.abc import AsyncIterator
from threading import Event, Thread
import time
from typing import Any

from dimos_lcm.world_msgs import WorldAction, WorldObservations

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.files.File import File
from dimos.msgs.std_msgs.String import String
from dimos.sim2.actions import read_actions
from dimos.sim2.mujoco.backend import MujocoBackend
from dimos.sim2.sim import SimulatedWorld
from dimos.sim2.spec import SimulatedRobot, SimulatedSensor
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class MujocoSim(SimulatedWorld):
    """One compiled MuJoCo world, stepped in real time, publishing a frame of what moved.

    It steps only once every module has started: a module that misses a frame never gets it,
    because a frame says what *changed* and the first one is the only one that says a robot
    exists at all.
    """

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
        self._backend: MujocoBackend | None = None
        self._publisher: _LatestPublisher | None = None
        self._all_started = Event()

    @rpc
    def build(self) -> None:
        super().build()
        usd = self.world()
        if usd is None:
            raise ValueError(
                "nothing says what this world is: pass scene= a USD layer or a file holding one,"
                " or scene_manager= the UsdManager that composes it"
            )
        self._backend = MujocoBackend(usd, simulated=self.simulated, speed=self.config.speed)

    async def main(self) -> AsyncIterator[None]:
        assert self._backend is not None
        # Every action is applied: one dropped while busy would be a lost motor command.
        self.actions.subscribe(self._on_actions)
        self._publisher = _LatestPublisher(self.world_observations.publish)
        running = asyncio.create_task(self._run())
        announcing = asyncio.create_task(self._announce())
        yield
        announcing.cancel()
        running.cancel()

    @rpc
    def on_system_modules(self, _modules: list[Any]) -> None:
        self._all_started.set()

    async def _run(self) -> None:
        """Step, publish, and at a controller's tick wait for its actions before stepping on."""
        backend, publisher = self._backend, self._publisher
        assert backend is not None and publisher is not None
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
        """The robots and sensors in this world, and where its layer landed."""
        values = self.simulated.values()
        return {
            **(self._backend.describe() if self._backend is not None else {}),
            # Everything a sensor needs to build the same world, with this engine's scene.
            "simulated": list(values),
            "robots": sorted(d.robot_id for d in values if isinstance(d, SimulatedRobot)),
            "sensors": sorted(
                f"{d.robot_id or 'world'}/{d.frame_id}"
                for d in values
                if isinstance(d, SimulatedSensor)
            ),
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
