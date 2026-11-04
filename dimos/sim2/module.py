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

"""DimOS simulation module: clock, commands, stepping, reset and physical truth."""

from collections.abc import Callable
from concurrent.futures import Future
from dataclasses import dataclass
import math
from queue import Empty, Full, Queue
import threading
import time
from typing import Any, ClassVar, cast
import uuid

from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.control_msgs import BodyWrench, JointCommand
from dimos_lcm.sensor_msgs import JointState
from dimos_lcm.sim_msgs import BodyMotion, BodyPoses, SimulationClock

from dimos.core.core import rpc
from dimos.core.module import ModuleConfig
from dimos.core.shared_config import ConfigSenders, SharedConfigModule
from dimos.core.stream import In, Out
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
    pose_hz: float = 200
    motion_hz: float = 500
    joint_hz: float = 500
    motion_frames: tuple[str, ...] = ()
    joint_ids: tuple[str, ...] = ()


class SimModule(SharedConfigModule):
    config: SimConfig
    dedicated_worker: ClassVar[bool] = True
    joint_command: In[JointCommand]
    body_wrench: In[BodyWrench]
    body_poses: Out[BodyPoses]
    body_motion: Out[BodyMotion]
    joint_truth: Out[JointState]
    clock: Out[SimulationClock]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._backend = self.config.backend()
        self._threads: list[threading.Thread] = []
        self._clock_id = uuid.uuid4().hex
        self._epoch = self._time_ns = 0
        self._pose_queue: Queue[BodyPoses] = Queue(1)
        self._motion_queue: Queue[BodyMotion] = Queue(512)
        self._joint_queue: Queue[JointState] = Queue(512)
        self._clock_queue: Queue[SimulationClock] = Queue(1)
        self._ready: Future[None] = Future()
        self._commands: Queue[JointCommand | BodyWrench] = Queue(256)
        self._pending: list[JointCommand | BodyWrench] = []
        self._operations: Queue[tuple[str, tuple[Any, ...], Future[Any]]] = Queue(32)
        self._done = threading.Event()
        self._status_lock = threading.Lock()
        self._status: dict[str, Any] = {
            "status": "starting",
            "error": "",
            "publication_error": "",
            "rejected_commands": 0,
            "dropped_poses": 0,
            "late_steps": 0,
            "time_ns": 0,
            "epoch": 0,
            "control_wait_s": 0.0,
        }
        self._sequences: dict[str, int] = {}
        self._answered: dict[str, int] = {}
        self._expires: dict[str, int] = {}
        self._wrenches: list[BodyWrench] = []
        self._next = {"poses": 0, "motion": 0, "joints": 0}
        self._last_motion: list[BodyMotion] = []
        self._last_joints: JointState | None = None
        self._waiting_since: float | None = None
        self._last_repeat = 0.0
        self._ever_controlled = False

    def config_receive(self, config_senders: ConfigSenders) -> None:
        self.config.scene = self.config.scene.resolve(config_senders)

    @rpc
    def start(self) -> None:
        super().start()
        self._threads = [threading.Thread(target=self._run, name="sim2-physics", daemon=True)]
        try:
            self._threads[0].start()
            self._ready.result(timeout=30)
            # Scene serialization cannot block high-rate motion/joint publication.
            for name, target in (
                ("sim2-truth", self._publish_truth),
                ("sim2-scene", self._publish_scene),
            ):
                thread = threading.Thread(target=target, name=name, daemon=True)
                self._threads.append(thread)
                thread.start()
        except Exception:
            self.stop()
            raise

    @rpc
    def submit(self, command: JointCommand | BodyWrench) -> None:
        self._commands.put_nowait(command)

    async def handle_joint_command(self, command: JointCommand) -> None:
        self.submit(command)

    async def handle_body_wrench(self, command: BodyWrench) -> None:
        self.submit(command)

    @rpc
    def inspect(self, joints: tuple[str, ...] = ()) -> tuple[BodyPoses, JointState]:
        return cast("tuple[BodyPoses, JointState]", self._request("inspect", joints))

    @rpc
    def reset(self) -> tuple[BodyPoses, JointState]:
        return cast("tuple[BodyPoses, JointState]", self._request("reset"))

    @rpc
    def place(self, body_id: str, pose: tuple[float, ...]) -> tuple[BodyPoses, JointState]:
        return cast("tuple[BodyPoses, JointState]", self._request("place", body_id, pose))

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
        for rate in (c.pose_hz, c.motion_hz, c.joint_hz):
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
        if not set(c.motion_frames) <= set(self._backend.body_ids):
            raise ValueError("unknown exported rigid frame")
        for controller in c.controllers:
            for rate in (c.motion_hz, c.joint_hz):
                if controller.period_ns % round(1e9 / rate):
                    raise ValueError("truth schedules must include every control boundary")
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

    def _inspect(self, joints: tuple[str, ...] = ()) -> tuple[BodyPoses, JointState]:
        """Synchronous only on the owner thread; RPC handlers call _request("inspect")."""
        poses = self._backend.read_body_poses()
        joint = self._backend.read_joints(joints)
        joint.header = self._header("sim/joint_truth")
        return BodyPoses(
            header=self._header("sim/body_poses", "world"),
            model_revision=self.config.scene.revision,
            count=len(poses),
            bodies=poses,
        ), joint

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
                if isinstance(command, JointCommand):
                    if command.targets_length != len(command.targets):
                        raise ValueError("invalid target count")
                    if command.based_on_time_ns > command.apply_at_ns:
                        raise ValueError("command cannot depend on a future measurement")
                    names = {t.joint_id for t in command.targets}
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
                                raise ValueError(
                                    "controlled joints require their exact current sample"
                                )
                    self._backend.set_joint_targets(command.targets)
                    self._expires.update((n, command.valid_until_ns) for n in names)
                    self._answered[h.source_id] = self._time_ns
                else:
                    self._backend.validate_wrench(command)
                    if len(self._wrenches) >= 256:
                        raise ValueError("too many active wrenches")
                    self._wrenches.append(command)
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
        self._wrenches = [w for w in self._wrenches if w.valid_until_ns > self._time_ns]
        self._backend.step(tuple(self._wrenches))
        self._time_ns += self.config.dt_ns
        with self._status_lock:
            self._status.update(time_ns=self._time_ns, epoch=self._epoch)

    def _set_status(self, **fields: Any) -> None:
        with self._status_lock:
            self._status.update(fields, time_ns=self._time_ns, epoch=self._epoch)
        sample = SimulationClock(
            header=self._header("sim/clock"),
            dt_ns=self.config.dt_ns,
            status=self._status["status"],
            detail=self._status["error"],
        )
        self._latest(self._clock_queue, sample)

    @staticmethod
    def _latest(queue: Queue[Any], value: Any) -> bool:
        dropped = False
        try:
            queue.put_nowait(value)
        except Full:
            try:
                queue.get_nowait()
                dropped = True
            except Empty:
                pass
            queue.put_nowait(value)
        return dropped

    def _sample_truth(self) -> None:
        if self._time_ns >= self._next["joints"]:
            joint = self._backend.read_joints(self.config.joint_ids)
            joint.header = self._header("sim/joint_truth")
            self._joint_queue.put_nowait(joint)
            self._last_joints = joint
            period = round(1e9 / self.config.joint_hz)
            self._next["joints"] = (self._time_ns // period + 1) * period
        if self._time_ns >= self._next["motion"]:
            self._last_motion = []
            for frame in self.config.motion_frames:
                motion = self._backend.read_motion(frame)
                motion.header = self._header("sim/body_motion", frame)
                self._motion_queue.put_nowait(motion)
                self._last_motion.append(motion)
            period = round(1e9 / self.config.motion_hz)
            self._next["motion"] = (self._time_ns // period + 1) * period
        if self._time_ns >= self._next["poses"]:
            poses, _ = self._inspect()
            if self._latest(self._pose_queue, poses):
                with self._status_lock:
                    self._status["dropped_poses"] += 1
            period = round(1e9 / self.config.pose_hz)
            self._next["poses"] = (self._time_ns // period + 1) * period

    def _run(self) -> None:
        try:
            self._load()
            deadline = time.monotonic()
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
                            self._last_joints is not None
                            and self._last_joints.header.time_ns == self._time_ns
                        ):
                            self._joint_queue.put_nowait(self._last_joints)
                        for motion in self._last_motion:
                            if motion.header.time_ns == self._time_ns:
                                self._motion_queue.put_nowait(motion)
                        self._last_repeat = now
                    deadline = now
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
                    if delay < -self.config.dt_ns / 1e9:
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

    def _invalidate(self) -> None:
        self._epoch += 1
        self._time_ns = 0
        self._answered.clear()
        self._expires.clear()
        self._sequences.clear()
        self._wrenches.clear()
        self._pending.clear()
        self._next = dict.fromkeys(self._next, 0)
        self._waiting_since = None
        self._ever_controlled = False
        self._last_motion = []
        self._last_joints = None
        self._backend.hold(self._backend.driven_joint_ids)
        self._set_status(status="running", error="")

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
            if operation == "reset":
                self._backend.reset()
                self._invalidate()
            elif operation == "place":
                self._backend.place(*args)
                self._invalidate()
            elif operation != "inspect":
                raise ValueError("unknown simulation operation")
            future.set_result(self._inspect(*args) if operation == "inspect" else self._inspect())
        except (ValueError, KeyError) as error:
            future.set_exception(error)

    def _publish_truth(self) -> None:
        try:
            while not self._done.is_set():
                sent = False
                for queue, output in (
                    (self._joint_queue, self.joint_truth),
                    (self._motion_queue, self.body_motion),
                    (self._clock_queue, self.clock),
                ):
                    try:
                        sample = queue.get_nowait()
                    except Empty:
                        continue
                    output.publish(sample)
                    sent = True
                if not sent:
                    self._done.wait(0.001)
        except Exception as error:
            self._publication_failed(error)

    def _publish_scene(self) -> None:
        try:
            while not self._done.is_set():
                try:
                    poses = self._pose_queue.get(timeout=0.1)
                except Empty:
                    continue
                self.body_poses.publish(poses)
        except Exception as error:
            self._publication_failed(error)

    def _publication_failed(self, error: Exception) -> None:
        with self._status_lock:
            self._status.update(status="fault", publication_error=repr(error))
        self._done.set()
