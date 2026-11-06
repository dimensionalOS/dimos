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

"""DimOS simulation module: time, commands, stepping and physical truth."""

from collections.abc import Callable
from concurrent.futures import Future
from dataclasses import dataclass
import math
from queue import Empty, Queue
import threading
import time
from typing import Any, ClassVar, cast
import uuid

from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.control_msgs import ActionCommand
from dimos_lcm.sim_msgs import WorldState
from dimos_lcm.std_msgs import Header, Time

from dimos.core.core import rpc
from dimos.core.module import ModuleConfig
from dimos.core.shared_config import ConfigSenders, SharedConfigModule, shared_config
from dimos.core.stream import In, Out
from dimos.sim2.cbor import validate_cbor_entries
from dimos.sim2.description import SceneConfig
from dimos.sim2.physics.interface import PhysicsBackend


@dataclass(frozen=True)
class ControllerConfig:
    controller_id: str
    period_ns: int
    joint_ids: tuple[str, ...]


class SimConfig(ModuleConfig):
    scene: SceneConfig
    backend: Callable[[], PhysicsBackend]
    dt_ns: int = 2_000_000
    controllers: tuple[ControllerConfig, ...] = ()
    control_timeout_s: float = 2.0
    startup_timeout_s: float = 30.0
    pace_to_wall_time: bool = True
    observation_hz: float = 500
    joint_ids: tuple[str, ...] = ()


class SimModule(SharedConfigModule):
    config: SimConfig
    dedicated_worker: ClassVar[bool] = True
    actions: In[ActionCommand]
    observations: Out[WorldState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._backend = self.config.backend()
        self._threads: list[threading.Thread] = []
        self._clock_id = uuid.uuid4().hex
        self._epoch = self._time_ns = 0
        self._observations: Queue[WorldState] = Queue(512)
        self._ready: Future[None] = Future()
        self._commands: Queue[ActionCommand] = Queue(256)
        self._pending: list[ActionCommand] = []
        self._operations: Queue[tuple[str, tuple[Any, ...], Future[Any]]] = Queue(32)
        self._done = threading.Event()
        self._status_lock = threading.Lock()
        self._status: dict[str, Any] = {
            "status": "starting",
            "error": "",
            "publication_error": "",
            "rejected_commands": 0,
            "late_steps": 0,
            "time_ns": 0,
            "epoch": 0,
            "control_wait_s": 0.0,
        }
        self._sequences: dict[str, int] = {}
        self._answered: dict[str, int] = {}
        self._expires: dict[str, int] = {}
        self._next_sample_ns = 0
        self._last_state: WorldState | None = None
        self._waiting_since: float | None = None
        self._last_repeat = 0.0
        self._ever_controlled = False

    def config_receive(self, config_senders: ConfigSenders) -> None:
        self.config.scene = self.config.scene.resolve(config_senders)

    @shared_config(SharedConfigModule)
    def simulation_config(self) -> dict[str, Any]:
        """Deliver run identity and fixed gravity before any module starts."""
        return {
            "sim_clock_id": self._clock_id,
            "sim_gravity_world": self.config.scene.gravity_world,
        }

    @rpc
    def start(self) -> None:
        super().start()
        self._threads = [threading.Thread(target=self._run, name="sim2-physics", daemon=True)]
        try:
            self._threads[0].start()
            self._ready.result(timeout=self.config.startup_timeout_s)
            publisher = threading.Thread(
                target=self._publish, name="sim2-observations", daemon=True
            )
            self._threads.append(publisher)
            publisher.start()
        except Exception:
            self.stop()
            raise

    @rpc
    def submit(self, command: ActionCommand) -> None:
        self._commands.put_nowait(command)

    async def handle_actions(self, command: ActionCommand) -> None:
        self.submit(command)

    @rpc
    def inspect(self) -> WorldState:
        return cast("WorldState", self._request("inspect"))

    @rpc
    def place(self, body_id: str, pose: tuple[float, ...]) -> WorldState:
        return cast("WorldState", self._request("place", body_id, pose))

    @rpc
    def status(self) -> dict[str, Any]:
        with self._status_lock:
            return dict(self._status)

    @rpc
    def stop(self) -> None:
        self._done.set()
        for thread in self._threads:
            thread.join(timeout=5)
            if thread.is_alive():
                raise RuntimeError(f"{thread.name} did not stop")
        super().stop()

    def _load(self) -> None:
        c = self.config
        if not 0 < c.dt_ns <= 20_000_000:
            raise ValueError("physics dt must be in (0, 20 ms]")
        for rate in (c.observation_hz,):
            if not math.isfinite(rate) or not 0 < rate <= 1e9 / c.dt_ns:
                raise ValueError("truth rate must be positive and no faster than physics")
        if not all(math.isfinite(t) and t > 0 for t in (c.control_timeout_s, c.startup_timeout_s)):
            raise ValueError("control deadlines must be positive")
        self._backend.load(self.config.scene, c.dt_ns)
        assigned: set[str] = set()
        controllers: set[str] = set()
        for controller in c.controllers:
            if controller.period_ns <= 0 or controller.period_ns % c.dt_ns:
                raise ValueError("control period must be a positive multiple of physics dt")
            names = set(controller.joint_ids)
            if (
                not names
                or len(names) != len(controller.joint_ids)
                or names & assigned
                or not names <= set(self._backend.driven_joint_ids)
                or controller.controller_id in controllers
            ):
                raise ValueError("controllers require distinct IDs and disjoint driven joints")
            assigned.update(names)
            controllers.add(controller.controller_id)
        if not set(c.joint_ids) <= set(self._backend.joint_ids):
            raise ValueError("unknown exported joint")
        for controller in c.controllers:
            if controller.period_ns % round(1e9 / c.observation_hz):
                raise ValueError("observation schedule must include every control boundary")
            if c.joint_ids and not set(controller.joint_ids) <= set(c.joint_ids):
                raise ValueError("controller joints must be included in WorldState")
        self._set_status(status="running")
        self._ready.set_result(None)

    def _header(self, source: str, frame: str = "") -> SampleHeader:
        return SampleHeader(
            clock_id=self._clock_id,
            epoch=self._epoch,
            time_ns=self._time_ns,
            source_id=source,
            sequence=self._time_ns // self.config.dt_ns,
            frame_id=frame,
        )

    def _inspect(self) -> WorldState:
        """Capture one complete acquisition on the physics owner thread."""
        bodies = self._backend.read_bodies()
        joints = self._backend.read_joints(self.config.joint_ids or self._backend.joint_ids)
        cbor_observations = self._backend.read_cbor_observations()
        validate_cbor_entries(
            cbor_observations, {"world", *self._backend.body_ids, *self._backend.joint_ids}
        )
        header = self._header("sim/observations", "world")
        sec, nsec = divmod(self._time_ns, 1_000_000_000)
        joints.header = Header(stamp=Time(sec=sec, nsec=nsec))
        return WorldState(
            header=header,
            model_revision=self.config.scene.revision,
            bodies_length=len(bodies),
            bodies=bodies,
            joint_truth=joints,
            cbor_observations_length=len(cbor_observations),
            cbor_observations=cbor_observations,
        )

    def _apply_commands(self) -> None:
        for _ in range(256 - len(self._pending)):
            try:
                self._pending.append(self._commands.get_nowait())
            except Empty:
                break
        pending, self._pending = self._pending, []
        for command in pending:
            try:
                h = command.header
                if h.clock_id != self._clock_id or h.epoch != self._epoch:
                    raise ValueError("wrong clock or stale epoch")
                if h.sequence <= self._sequences.get(h.source_id, -1):
                    raise ValueError("duplicate or out-of-order command")
                if command.apply_at_ns > self._time_ns:
                    if command.apply_at_ns - self._time_ns > 10_000_000_000:
                        raise ValueError("command scheduled too far ahead")
                    self._pending.append(command)
                    continue
                if not self._time_ns < command.valid_until_ns <= self._time_ns + 10_000_000_000:
                    raise ValueError("command validity must be in (0, 10] simulation seconds")
                if command.joint_commands_length != len(command.joint_commands):
                    raise ValueError("invalid joint target count")
                if command.cbor_commands_length != len(command.cbor_commands):
                    raise ValueError("invalid CBOR command count")
                if not command.joint_commands and not command.cbor_commands:
                    raise ValueError("empty action")
                if command.based_on_time_ns > command.apply_at_ns:
                    raise ValueError("command cannot depend on a future measurement")
                validate_cbor_entries(
                    command.cbor_commands,
                    {"world", *self._backend.body_ids, *self._backend.joint_ids},
                )
                names = {name for row in command.joint_commands for name in row.joint_names}
                answered = []
                for controller in self.config.controllers:
                    if names.intersection(controller.joint_ids):
                        if (
                            h.source_id != controller.controller_id
                            or names != set(controller.joint_ids)
                            or command.based_on_time_ns != self._time_ns
                            or command.apply_at_ns != self._time_ns
                            or command.valid_until_ns < self._time_ns + controller.period_ns
                            or self._time_ns % controller.period_ns
                        ):
                            raise ValueError("controlled joints require their exact current sample")
                        answered.append(controller.controller_id)
                # Backend validates the whole action before changing any native state.
                self._backend.apply_action(command.joint_commands, command.cbor_commands)
                self._expires.update((n, command.valid_until_ns) for n in names)
                # An unrelated CBOR operation is never a controller's motor reply.
                self._answered.update((source, self._time_ns) for source in answered)
                self._sequences[h.source_id] = h.sequence
            except ValueError as error:
                with self._status_lock:
                    self._status["rejected_commands"] += 1
                    self._status["last_rejection"] = str(error)

    def _step(self) -> None:
        expired = tuple(n for n, end in self._expires.items() if end <= self._time_ns)
        if expired:
            self._backend.hold(expired)
            for name in expired:
                del self._expires[name]
        self._backend.step()
        self._time_ns += self.config.dt_ns
        with self._status_lock:
            self._status.update(time_ns=self._time_ns, epoch=self._epoch)

    def _set_status(self, **fields: Any) -> None:
        with self._status_lock:
            self._status.update(fields, time_ns=self._time_ns, epoch=self._epoch)

    def _sample_truth(self) -> None:
        if self._time_ns >= self._next_sample_ns:
            sample = self._inspect()
            self._observations.put_nowait(sample)
            self._last_state = sample
            period = round(1e9 / self.config.observation_hz)
            self._next_sample_ns = (self._time_ns // period + 1) * period

    def _run(self) -> None:
        try:
            self._load()
            deadline = time.monotonic()
            # Inference/transport belongs inside the control-period wall budget.
            # Allow bounded catch-up after a reply, never an unlimited burst.
            catchup_s = (
                max((c.period_ns for c in self.config.controllers), default=self.config.dt_ns) / 1e9
            )
            while not self._done.is_set():
                self._process_operations()
                if self.status()["status"] == "fault":
                    self._done.wait(0.01)
                    continue
                self._apply_commands()
                self._sample_truth()
                pending = [
                    c
                    for c in self.config.controllers
                    if not self._time_ns % c.period_ns
                    and self._answered.get(c.controller_id) != self._time_ns
                ]
                now = time.monotonic()
                if pending:
                    if self._waiting_since is None:
                        self._waiting_since = now
                        self._set_status(status="waiting")
                    timeout = (
                        self.config.control_timeout_s
                        if self._ever_controlled
                        else self.config.startup_timeout_s
                    )
                    if now - self._waiting_since > timeout:
                        self._set_status(status="fault", error="controller deadline exceeded")
                        continue
                    if now - self._last_repeat >= 0.1:
                        # Retransmit the SAME acquisition; sensors deduplicate their models.
                        if (
                            self._last_state is not None
                            and self._last_state.header.time_ns == self._time_ns
                        ):
                            self._observations.put_nowait(self._last_state)
                        self._last_repeat = now
                    self._done.wait(0.001)
                    continue
                if self._waiting_since is not None:
                    self._set_status(status="running", control_wait_s=now - self._waiting_since)
                    self._waiting_since = None
                    self._ever_controlled = True
                self._step()
                if self.config.pace_to_wall_time:
                    deadline += self.config.dt_ns / 1e9
                    delay = deadline - time.monotonic()
                    if delay < -catchup_s:
                        with self._status_lock:
                            self._status["late_steps"] += 1
                        deadline = time.monotonic()
                    self._done.wait(max(0, delay))
        except Exception as error:
            # Thread boundary: preserve the error for RPC/status and unblock startup.
            self._set_status(status="fault", error=repr(error))
            if not self._ready.done():
                self._ready.set_exception(error)
        finally:
            self._done.set()
            self._backend.close()

    def _request(self, operation: str, *args: Any) -> Any:
        if self._done.is_set():
            raise RuntimeError("simulation stopped")
        future: Future[Any] = Future()
        self._operations.put_nowait((operation, args, future))
        return future.result(timeout=5)

    def _process_operations(self) -> None:
        try:
            operation, args, future = self._operations.get_nowait()
        except Empty:
            return
        try:
            if operation == "place":
                self._backend.place(*args)
            elif operation != "inspect":
                raise ValueError("unknown simulation operation")
            future.set_result(self._inspect())
        except (ValueError, KeyError) as error:
            future.set_exception(error)

    def _publish(self) -> None:
        try:
            while not self._done.is_set():
                try:
                    sample = self._observations.get_nowait()
                except Empty:
                    self._done.wait(0.001)
                    continue
                self.observations.publish(sample)
        except Exception as error:
            self._publication_failed(error)

    def _publication_failed(self, error: Exception) -> None:
        with self._status_lock:
            self._status.update(status="fault", publication_error=repr(error))
        self._done.set()
