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
import math
from queue import Empty, Full, Queue
import threading
import time
from typing import Any, ClassVar, cast
import uuid

from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.control_msgs import ActionCommand
from dimos_lcm.sim_msgs import WorldState
from dimos_lcm.std_msgs import Header, Time

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.sim2.cbor import validate_cbor_entries
from dimos.sim2.description import SceneConfig
from dimos.sim2.physics.interface import PhysicsBackend
from dimos.sim2.publication import PublicationRate
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
WARNING_INTERVAL_S = 5.0


class SimConfig(SceneConfig):
    gravity_world: tuple[float, float, float] = (0.0, 0.0, -9.81)
    backend: Callable[[], PhysicsBackend]
    dt_ns: int = 2_000_000
    pace_to_wall_time: bool = True
    observation_hz: float = 500
    joint_ids: tuple[str, ...] = ()


class SimModule(Module):
    """Advance fixed physics steps independently of command and sensor arrival.

    Due, unexpired actions apply before the next step. Joint targets persist
    until replaced or expired; expiry holds the current joint position.
    """

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
            "accepted_commands": 0,
            "late_commands": 0,
            "expired_targets": 0,
            "dropped_observations": 0,
            "real_time_factor": 0.0,
        }
        self._sequences: dict[str, int] = {}
        self._expires: dict[str, int] = {}
        self._next_sample_ns = 0
        self._next_warning = 0.0
        self._publication_rate = PublicationRate(self, "observations", self.config.observation_hz)

    @rpc
    def simulation_config(self) -> dict[str, Any]:
        """Read this run's identity and gravity; available before physics starts."""
        return {
            "sim_clock_id": self._clock_id,
            "sim_gravity_world": self.config.gravity_world,
        }

    @rpc
    def start(self) -> None:
        super().start()
        self._threads = [threading.Thread(target=self._run, name="sim2-physics", daemon=True)]
        try:
            self._threads[0].start()
            self._ready.result(timeout=120)
            self._publication_rate.begin()
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
        if not math.isfinite(c.observation_hz) or not 0 < c.observation_hz <= 1e9 / c.dt_ns:
            raise ValueError("truth rate must be positive and no faster than physics")
        self._scene = c.load_scene(self.rpc)
        self._backend.load(self._scene, c.dt_ns, c.gravity_world)
        if not set(c.joint_ids) <= set(self._backend.joint_ids):
            raise ValueError("unknown exported joint")
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
            model_revision=self._scene.revision,
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
                # Backend validates the whole action before changing any native state.
                self._backend.apply_action(command.joint_commands, command.cbor_commands)
                self._expires.update((n, command.valid_until_ns) for n in names)
                self._sequences[h.source_id] = h.sequence
                with self._status_lock:
                    self._status["accepted_commands"] += 1
                    self._status["late_commands"] += int(command.apply_at_ns < self._time_ns)
            except ValueError as error:
                with self._status_lock:
                    self._status["rejected_commands"] += 1
                    self._status["last_rejection"] = str(error)
                self._warn("Simulation command rejected", reason=str(error))

    def _step(self) -> None:
        expired = tuple(n for n, end in self._expires.items() if end <= self._time_ns)
        if expired:
            self._backend.hold(expired)
            for name in expired:
                del self._expires[name]
            with self._status_lock:
                self._status["expired_targets"] += len(expired)
            self._warn(
                "Simulation joint targets expired; holding current positions", joints=expired
            )
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
            try:
                self._observations.put_nowait(sample)
            except Full:
                # Publishing must not stall physics. Keep the newest acquisition.
                try:
                    self._observations.get_nowait()
                except Empty:
                    pass
                self._observations.put_nowait(sample)
                with self._status_lock:
                    self._status["dropped_observations"] += 1
                self._warn("Simulation publication is falling behind; dropping observations")
            period = round(1e9 / self.config.observation_hz)
            self._next_sample_ns = (self._time_ns // period + 1) * period

    def _warn(self, message: str, **fields: Any) -> None:
        now = time.monotonic()
        if now >= self._next_warning:
            logger.warning(message, **fields)
            self._next_warning = now + WARNING_INTERVAL_S

    def _run(self) -> None:
        try:
            self._load()
            deadline = time.monotonic()
            report_at, report_time_ns = deadline, self._time_ns
            step_s = self.config.dt_ns / 1e9
            while not self._done.is_set():
                self._process_operations()
                if self.status()["status"] == "fault":
                    self._done.wait(0.01)
                    continue
                self._apply_commands()
                self._sample_truth()
                self._step()
                now = time.monotonic()
                if now - report_at >= 1.0:
                    factor = (self._time_ns - report_time_ns) / 1e9 / (now - report_at)
                    self._set_status(real_time_factor=factor)
                    if factor < 0.9:
                        self._warn("Simulation is running below real time", real_time_factor=factor)
                    report_at, report_time_ns = now, self._time_ns
                if self.config.pace_to_wall_time:
                    deadline += step_s
                    delay = deadline - now
                    if delay < -step_s:
                        with self._status_lock:
                            self._status["late_steps"] += 1
                        deadline = now
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
                self._publication_rate.published()
        except Exception as error:
            self._publication_failed(error)

    def _publication_failed(self, error: Exception) -> None:
        with self._status_lock:
            self._status.update(status="fault", publication_error=repr(error))
        self._done.set()
