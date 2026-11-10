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
from dimos.sim2.sim import SimulatedWorldConfig, driving, scene_from, until_it_stops
from dimos.sim2.spec import validate
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class MujocoSimConfig(SimulatedWorldConfig):
    max_burst_seconds: float = 0.01


# How often the world republishes itself while it is held before the first step.
_HOLD_PERIOD = 0.05
# How often it asks the modules in `wait_for` whether they are driving yet.
_PROBE_PERIOD = 0.5


class MujocoSim(Module):
    config: MujocoSimConfig
    actions: In[WorldAction]
    world_observations: Out[WorldObservations]
    render_file_request: In[String]
    file_info: Out[File]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._backend: MujocoBackend | None = None
        self._publisher: _LatestPublisher | None = None
        self._all_started = Event()
        try:
            validate(self.config.robots)
        except Exception:
            self.stop()
            raise
        self.simulated = self.config.robots

    async def main(self) -> AsyncIterator[None]:
        self.actions.subscribe(self._on_actions)
        self._publisher = _LatestPublisher(self.world_observations.publish)
        running = asyncio.create_task(until_it_stops("the world", self._run()))
        yield
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
        backend = self._backend = await asyncio.to_thread(self._engine)
        publisher = self._publisher
        assert publisher is not None
        await asyncio.to_thread(self._all_started.wait)
        await self._settle_before_stepping(backend, publisher)
        while True:
            _when, frame = backend.step()
            if backend.waiting_for:
                while publisher.busy:
                    await asyncio.sleep(0)
                publisher.offer(frame)
                deadline = time.monotonic() + self.config.controller_timeout
                while not backend.ready() and time.monotonic() < deadline:
                    await asyncio.sleep(0)
            elif not publisher.busy:
                publisher.offer(frame)
            await asyncio.sleep(0.001)

    async def _settle_before_stepping(self, backend: Any, publisher: Any) -> None:
        """Publish the world until the robots are being driven, then start it.

        See `startup_delay` in `SimulatedWorldConfig` for why the wait is here at all. The
        world is published throughout, because that is what the policies are waiting on: they
        read the joints out of these observations, and until they have them they send nothing.
        Nothing is stepped, so none of the wait is time a robot can fall over in.

        `wait_for` is what ends it: the modules named there are asked, every half second,
        whether their tasks have armed, and the first moment they all have the world starts.
        `startup_delay` is the cap on that, and the whole wait when nothing is named -- so
        leaving `wait_for` empty is the blind timer, and filling it can only make the wait
        shorter, never longer.
        """
        deadline = time.monotonic() + self.config.startup_delay
        waiting = list(self.config.wait_for)
        asked = 0.0
        while time.monotonic() < deadline:
            if not publisher.busy:
                publisher.offer(backend.frame())
            if waiting and time.monotonic() >= asked:
                asked = time.monotonic() + _PROBE_PERIOD
                waiting = await asyncio.to_thread(self._still_waiting, waiting)
                if not waiting:
                    logger.info("sim2: every robot is being driven; starting the world")
                    break
            # Slowly. One frame is all a policy needs to see the joints and start commanding,
            # and the whole reason for the wait is that this machine is busy -- every module
            # that wants a view of the room is compiling its own copy of it right now. Holding
            # at 200 Hz would be the sim competing with the arming it is waiting for.
            await asyncio.sleep(_HOLD_PERIOD)
        else:
            if waiting:
                logger.warning(
                    f"sim2: started the world on the {self.config.startup_delay}s cap with "
                    f"{[m.remote_name for m in waiting]} not driving yet"
                )
        backend.restart_clock()

    def _still_waiting(self, mounts: list[Any]) -> list[Any]:
        """Those of `mounts` that are not driving yet. One that cannot say is not waited for."""
        left = []
        for mount in mounts:
            try:
                if not driving(mount, self.rpc):
                    left.append(mount)
            except Exception as error:
                logger.info(
                    f"sim2: {mount.remote_name} cannot say whether it is driving: {error!r}"
                )
        return left

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
        if found is None:
            logger.debug(f"render_file_request for {request.data!r}, which the world does not name")
            return
        data = found if isinstance(found, bytes) else await asyncio.to_thread(found.read_bytes)
        self.file_info.publish(File(request.data, data))

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
        if self._publisher is not None:
            self._publisher.close()
            self._publisher = None
        self._backend = None


class _LatestPublisher:
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
                logger.exception("sim2: publishing a frame failed")
            self._wake.clear()
            if self._closed:
                return
