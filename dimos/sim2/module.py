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

"""Normal DimOS lifecycle around an ordinary, robot-independent simulation owner."""

from collections.abc import Callable
from concurrent.futures import Future
from queue import Empty, Full, Queue
import threading
import time
from typing import Any, ClassVar, Protocol

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.sim2.runtime import Runtime
from dimos.sim2.types import (
    Frame,
    JointEfforts,
    JointTargets,
    Layout,
    PhysicsBackend,
    World,
    WorldState,
    Wrench,
)
from dimos.spec.utils import Spec


class SimulationSpec(Spec, Protocol):
    def describe(self) -> Layout: ...
    def state(self) -> WorldState: ...
    def reset(self) -> WorldState: ...
    def place(self, body: str, pose: tuple[float, ...]) -> WorldState: ...


class SimConfig(ModuleConfig):
    world: World
    backend: Callable[[World], PhysicsBackend]
    publish_hz: float = 50.0


class SimModule(Module):
    config: SimConfig
    dedicated_worker: ClassVar[bool] = True
    tf: IO[TFMessage]
    targets: In[JointTargets]
    wrench: In[Wrench]
    world_frame: Out[Frame]
    efforts: In[JointEfforts]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._done = threading.Event()
        self._ready: Future[Runtime] = Future()
        self._operations: Queue[tuple[str, tuple[Any, ...], Future[WorldState]]] = Queue(32)
        self._frames: Queue[Frame] = Queue(1)
        self._latest: WorldState | None = None
        self._error = ""
        self._late_steps = 0
        self._dropped_publications = 0
        self._owner: threading.Thread | None = None
        self._publisher: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        if not 0 < self.config.publish_hz <= 1 / self.config.world.dt:
            raise ValueError("publication rate must be positive and no faster than physics")
        self._owner = threading.Thread(target=self._run, name="sim2-physics", daemon=True)
        self._owner.start()
        self._ready.result(timeout=30)
        self._publisher = threading.Thread(target=self._publish, name="sim2-publish", daemon=True)
        self._publisher.start()

    def _run(self) -> None:
        try:
            runtime = Runtime(self.config.backend(self.config.world))
            self._ready.set_result(runtime)
            every = max(1, round(1 / (self.config.publish_hz * runtime.backend.dt)))
            deadline = time.monotonic()
            while not self._done.is_set():
                try:
                    operation, args, future = self._operations.get_nowait()
                except Empty:
                    pass
                else:
                    try:
                        result = runtime.reset() if operation == "reset" else runtime.place(*args)
                        future.set_result(result)
                        deadline = time.monotonic()
                    except Exception as error:
                        future.set_exception(error)
                runtime.advance()
                if runtime.tick % every == 0:
                    frame = runtime.capture()
                    self._latest = frame.state
                    try:
                        self._frames.put_nowait(frame)
                    except Full:
                        try:
                            self._frames.get_nowait()
                        except Empty:
                            pass
                        self._frames.put_nowait(frame)
                        self._dropped_publications += 1
                deadline += runtime.backend.dt
                delay = deadline - time.monotonic()
                if delay < -runtime.backend.dt:
                    self._late_steps += 1
                    # Slow simulation time under load; never reset state or skip physics steps.
                    deadline = time.monotonic()
                self._done.wait(max(0, delay))
        except Exception as error:
            self._error = repr(error)
            if not self._ready.done():
                self._ready.set_exception(error)
            self._done.set()

    def _publish(self) -> None:
        try:
            self._publish_frames()
        except Exception as error:
            self._error = repr(error)
            self._done.set()

    def _publish_frames(self) -> None:
        layout = self.describe()
        roots = [(name, layout.bodies.index(name)) for name in layout.roots]
        while not self._done.is_set():
            try:
                frame = self._frames.get(timeout=0.1)
            except Empty:
                continue
            state = frame.state
            self.world_frame.publish(frame)
            self.joint_state.publish(
                JointState(
                    ts=state.wall_time,
                    name=list(layout.joints),
                    position=state.position.tolist(),
                    velocity=state.velocity.tolist(),
                    effort=state.effort.tolist(),
                )
            )
            # UrdfManager owns every internal joint/mount edge. Sim owns only world -> roots.
            self.tfbuffer.publish(
                *(
                    Transform(
                        frame_id="world",
                        child_frame_id=name,
                        ts=state.wall_time,
                        translation=Vector3(*state.poses[index, :3]),
                        rotation=Quaternion(*state.poses[index, 3:]),
                    )
                    for name, index in roots
                )
            )

    async def handle_targets(self, command: JointTargets) -> None:
        self._ready.result(timeout=30).submit(command)

    async def handle_efforts(self, command: JointEfforts) -> None:
        self._ready.result(timeout=30).submit(command)

    async def handle_wrench(self, command: Wrench) -> None:
        self._ready.result(timeout=30).submit(command)

    @rpc
    def describe(self) -> Layout:
        return self._ready.result(timeout=30).backend.layout

    @rpc
    def state(self) -> WorldState:
        if self._latest is None:
            raise RuntimeError("simulation has not published its first sample")
        return self._latest

    @rpc
    def reset(self) -> WorldState:
        return self._operation("reset", ())

    @rpc
    def place(self, body: str, pose: tuple[float, ...]) -> WorldState:
        return self._operation("place", (body, pose))

    def _operation(self, name: str, args: tuple[Any, ...]) -> WorldState:
        if self._done.is_set():
            raise RuntimeError(self._error or "simulation stopped")
        future: Future[WorldState] = Future()
        self._operations.put_nowait((name, args, future))
        return future.result(timeout=5)

    @rpc
    def apply_wrench(self, wrench: Wrench) -> None:
        """Queue an operator intervention for the next tick; read timing() for rejection details."""
        self._ready.result(timeout=30).submit(wrench)

    @rpc
    def timing(self) -> dict[str, Any]:
        runtime = self._ready.result(timeout=30)
        return {
            "late_steps": self._late_steps,
            "dropped_publications": self._dropped_publications,
            "rejected_commands": runtime.rejected,
            "last_rejection": runtime.last_error,
            "error": self._error,
            "tick": runtime.tick,
        }

    @rpc
    def stop(self) -> None:
        self._done.set()
        for thread in (self._owner, self._publisher):
            if thread is not None:
                thread.join(timeout=5)
        super().stop()
