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

"""One simulated world, built from every module that sends Simulation a description.

class Go2Sim(Module):
    @shared_config(Simulation)
    def simulated(self) -> dict[str, Any]:
        return {"simulated": SimulatedRobot("go2", ...)}
"""

import asyncio
from collections.abc import AsyncIterator
from pathlib import Path
import tempfile
from threading import Event, Thread
from typing import Any, Protocol

from dimos_lcm.world_msgs import WorldAction, WorldChange
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import ModuleConfig
from dimos.core.shared_config import ConfigSenders, SharedConfigModule
from dimos.core.stream import In, Out
from dimos.msgs.files.File import File
from dimos.msgs.std_msgs.String import String
from dimos.sim2.actions import read_actions
from dimos.sim2.backend import BACKENDS, SimulationBackend, load_backend
from dimos.sim2.spec import Simulated, SimulatedRobot, SimulatedSensor, validate
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class SimulationSpec(Spec, Protocol):
    def describe(self) -> dict[str, Any]: ...


class SimulationConfig(ModuleConfig):
    scene: Path | None = Field(default_factory=lambda m: m["g"].scene_package)
    backend: str = "mujoco"
    # Sim seconds per wall second.
    speed: float = 1.0
    # Wall seconds the world waits on a stalled controller (one that has acted before) before stepping without it.
    controller_timeout: float = 0.5


class Simulation(SharedConfigModule):
    """Publishes the world on world_change and acts on actions, through whichever backend config names.

    It steps only once every module has started: a module that misses a frame never gets it.
    """

    config: SimulationConfig
    world_change: Out[WorldChange]
    actions: In[WorldAction]
    # Like UrdfManager: a request names the world file, or a file it names, and gets it on file_info.
    render_file_request: In[String]
    file_info: Out[File]
    # The Controller's simulated world layer: it finds the world on scene_info, asks on scene_data_request.
    scene_info: Out[String]
    scene_data_request: In[String]
    scene_data: Out[String]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._backend: SimulationBackend | None = None
        self._directory: tempfile.TemporaryDirectory[str] | None = None
        self._publisher: _LatestPublisher | None = None
        self._all_started = Event()
        # Each sender's "simulated" value, keyed by its instance name.
        self.simulated: dict[str, Simulated] = {}

    def config_receive(self, config_senders: ConfigSenders) -> None:
        simulated: dict[str, Simulated] = {}
        for name, config in config_senders:
            description = config.get("simulated")
            if set(config) != {"simulated"} or not isinstance(description, Simulated):
                raise TypeError(
                    f"{name}'s @shared_config(Simulation) must return"
                    f" {{'simulated': SimulatedRobot | SimulatedSensor}}, not {config!r}"
                )
            simulated[name] = description
        self.simulated = simulated

    @rpc
    def build(self) -> None:
        super().build()
        validate(self.simulated.values())
        if self.config.backend not in BACKENDS:
            raise ValueError(f"unknown backend {self.config.backend!r}; one of {sorted(BACKENDS)}")
        self._directory = tempfile.TemporaryDirectory(prefix="dimos-sim2-")
        self._backend = load_backend(self.config.backend)(
            self.config, self.simulated, Path(self._directory.name)
        )

    async def main(self) -> AsyncIterator[None]:
        assert self._backend is not None
        # Every action is applied: one dropped while busy would be a lost motor command.
        self.actions.subscribe(self._on_actions)
        self._publisher = _LatestPublisher(self.world_change.publish)
        self._running = asyncio.create_task(self._run())
        announcing = asyncio.create_task(self._announce())
        yield
        announcing.cancel()

    @rpc
    def on_system_modules(self, _modules: list[Any]) -> None:
        self._all_started.set()

    async def _run(self) -> None:
        assert self._backend is not None and self._publisher is not None
        await asyncio.to_thread(self._all_started.wait)
        await self._backend.run(self._publisher)

    async def _announce(self) -> None:
        while True:
            self.scene_info.publish(String(self.config.backend))
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
        # Others on the topic (a UrdfManager) may answer the rest.
        if found is None:
            logger.debug(f"render_file_request for {request.data!r}, which the world does not name")
            return
        data = found if isinstance(found, bytes) else await asyncio.to_thread(found.read_bytes)
        self.file_info.publish(File(request.data, data))

    @rpc
    def describe(self) -> dict[str, Any]:
        """The robots and sensors in this world."""
        values = self.simulated.values()
        return {
            **(self._backend.describe() if self._backend is not None else {}),
            "backend": self.config.backend,
            # Everything a sensor's view builds the world from, with the backend's scene.
            "simulated": list(values),
            "robots": sorted(d.robot_id for d in values if isinstance(d, SimulatedRobot)),
            "sensors": sorted(
                f"{d.robot_id or 'world'}/{d.name}"
                for d in values
                if isinstance(d, SimulatedSensor)
            ),
        }

    @rpc
    def stop(self) -> None:
        super().stop()
        if self._publisher is not None:
            self._publisher.close()
        self._backend = None
        if self._directory is not None:
            self._directory.cleanup()
            self._directory = None


class _LatestPublisher:
    """Publishes on its own thread. A backend offers a frame only when it is not busy, so none is dropped."""

    def __init__(self, publish: Any) -> None:
        self._publish = publish
        self._pending: WorldChange | None = None
        self._wake = Event()
        self._closed = False
        self._thread = Thread(target=self._loop, name="world_change", daemon=True)
        self._thread.start()

    @property
    def busy(self) -> bool:
        return self._wake.is_set()

    def offer(self, change: WorldChange) -> None:
        if not self.busy:
            self._pending = change
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
