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

"""Standalone dynamics owner: time, validated commands and physical truth.

No Module, transport, renderer or sensor implementation is imported here.
Native state is touched only by the thread calling load/run/advance.
"""

from concurrent.futures import Future
from dataclasses import dataclass
import math
from queue import Empty, Full, Queue
import threading
import time
from typing import Any
import uuid

from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.control_msgs import BodyWrench, JointCommand
from dimos_lcm.sensor_msgs import JointState
from dimos_lcm.sim_msgs import BodyMotion, BodyPoses, SimulationClock

from dimos.sim2.description import SceneConfig
from dimos.sim2.physics.interface import PhysicsBackend


@dataclass(frozen=True)
class ControllerConfig:
    controller_id: str
    period_ns: int
    joint_ids: tuple[str, ...]


@dataclass(frozen=True)
class SimulationConfig:
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


class Simulation:
    def __init__(
        self, backend: PhysicsBackend, scene: SceneConfig, config: SimulationConfig
    ) -> None:
        self.backend, self.scene, self.config = backend, scene, config
        self.clock_id = uuid.uuid4().hex
        self.epoch = self.time_ns = 0
        self.body_poses: Queue[BodyPoses] = Queue(1)
        self.body_motion: Queue[BodyMotion] = Queue(512)
        self.joint_truth: Queue[JointState] = Queue(512)
        self.clock: Queue[SimulationClock] = Queue(1)
        self.ready: Future[None] = Future()
        self._commands: Queue[JointCommand | BodyWrench] = Queue(256)
        self._pending: list[JointCommand | BodyWrench] = []
        self._operations: Queue[tuple[str, tuple[Any, ...], Future[Any]]] = Queue(32)
        self._done = threading.Event()
        self._status_lock = threading.Lock()
        self._status: dict[str, Any] = {
            "status": "starting",
            "error": "",
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

    def load(self) -> None:
        c = self.config
        if not 0 < c.dt_ns <= 20_000_000:
            raise ValueError("physics dt must be in (0, 20 ms]")
        for rate in (c.pose_hz, c.motion_hz, c.joint_hz):
            if not math.isfinite(rate) or not 0 < rate <= 1e9 / c.dt_ns:
                raise ValueError("truth rate must be positive and no faster than physics")
        if min(c.control_timeout_s, c.startup_timeout_s) <= 0:
            raise ValueError("control deadlines must be positive")
        self.backend.load(self.scene, c.dt_ns)
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
                or not names <= set(self.backend.driven_joint_ids)
                or controller.controller_id in controllers
            ):
                raise ValueError("controllers require distinct IDs and disjoint driven joints")
            assigned.update(names)
            controllers.add(controller.controller_id)
        if not set(c.joint_ids) <= set(self.backend.joint_ids):
            raise ValueError("unknown exported joint")
        if not set(c.motion_frames) <= set(self.backend.body_ids):
            raise ValueError("unknown exported rigid frame")
        for controller in c.controllers:
            for rate in (c.motion_hz, c.joint_hz):
                if controller.period_ns % round(1e9 / rate):
                    raise ValueError("truth schedules must include every control boundary")
        self._set_status(status="running")
        self.ready.set_result(None)

    def header(self, source: str, frame: str = "") -> SampleHeader:
        return SampleHeader(
            clock_id=self.clock_id,
            epoch=self.epoch,
            time_ns=self.time_ns,
            source_id=source,
            sequence=self.time_ns // self.config.dt_ns,
            frame_id=frame,
        )

    def inspect(self, joints: tuple[str, ...] = ()) -> tuple[BodyPoses, JointState]:
        """Synchronous only on the owner thread; modules call request("inspect")."""
        poses = self.backend.read_body_poses()
        joint = self.backend.read_joints(joints)
        joint.header = self.header("sim/joint_truth")
        return BodyPoses(
            header=self.header("sim/body_poses", "world"),
            model_revision=self.scene.revision,
            count=len(poses),
            bodies=poses,
        ), joint

    def submit(self, command: JointCommand | BodyWrench) -> None:
        self._commands.put_nowait(command)

    def prepare(self) -> None:
        for _ in range(256 - len(self._pending)):
            try:
                self._pending.append(self._commands.get_nowait())
            except Empty:
                break
        pending, self._pending = self._pending, []
        for command in pending:
            try:
                h = command.header
                if h.clock_id != self.clock_id or h.epoch != self.epoch:
                    raise ValueError("wrong clock or stale epoch")
                if h.sequence <= self._sequences.get(h.source_id, -1):
                    raise ValueError("duplicate or out-of-order command")
                if command.apply_at_ns > self.time_ns:
                    if command.apply_at_ns - self.time_ns > 10_000_000_000:
                        raise ValueError("command scheduled too far ahead")
                    self._pending.append(command)
                    continue
                if not self.time_ns < command.valid_until_ns <= self.time_ns + 10_000_000_000:
                    raise ValueError("command validity must be in (0, 10] simulation seconds")
                if isinstance(command, JointCommand):
                    if command.targets_length != len(command.targets):
                        raise ValueError("invalid target count")
                    names = {t.joint_id for t in command.targets}
                    for controller in self.config.controllers:
                        if names.intersection(controller.joint_ids):
                            if (
                                h.source_id != controller.controller_id
                                or names != set(controller.joint_ids)
                                or command.based_on_time_ns != self.time_ns
                                or self.time_ns % controller.period_ns
                            ):
                                raise ValueError(
                                    "controlled joints require their exact current sample"
                                )
                    self.backend.set_joint_targets(command.targets)
                    self._expires.update((n, command.valid_until_ns) for n in names)
                    self._answered[h.source_id] = self.time_ns
                else:
                    self.backend.validate_wrench(command)
                    if len(self._wrenches) >= 256:
                        raise ValueError("too many active wrenches")
                    self._wrenches.append(command)
                self._sequences[h.source_id] = h.sequence
            except ValueError as error:
                with self._status_lock:
                    self._status["rejected_commands"] += 1
                    self._status["last_rejection"] = str(error)

    def advance(self, steps: int = 1) -> None:
        for _ in range(steps):
            self.prepare()
            expired = tuple(n for n, end in self._expires.items() if end <= self.time_ns)
            if expired:
                self.backend.hold(expired)
                for name in expired:
                    del self._expires[name]
            self._wrenches = [w for w in self._wrenches if w.valid_until_ns > self.time_ns]
            self.backend.step(tuple(self._wrenches))
            self.time_ns += self.config.dt_ns
            with self._status_lock:
                self._status.update(time_ns=self.time_ns, epoch=self.epoch)

    def _set_status(self, **fields: Any) -> None:
        with self._status_lock:
            self._status.update(fields, time_ns=self.time_ns, epoch=self.epoch)
        sample = SimulationClock(
            header=self.header("sim/clock"),
            dt_ns=self.config.dt_ns,
            status=self._status["status"],
            detail=self._status["error"],
        )
        self._latest(self.clock, sample)

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

    def status(self) -> dict[str, Any]:
        with self._status_lock:
            return dict(self._status)

    def publish_truth(self) -> None:
        if self.time_ns >= self._next["joints"]:
            joint = self.backend.read_joints(self.config.joint_ids)
            joint.header = self.header("sim/joint_truth")
            self.joint_truth.put_nowait(joint)
            self._last_joints = joint
            period = round(1e9 / self.config.joint_hz)
            self._next["joints"] = (self.time_ns // period + 1) * period
        if self.time_ns >= self._next["motion"]:
            self._last_motion = []
            for frame in self.config.motion_frames:
                motion = self.backend.read_motion(frame)
                motion.header = self.header("sim/body_motion", frame)
                self.body_motion.put_nowait(motion)
                self._last_motion.append(motion)
            period = round(1e9 / self.config.motion_hz)
            self._next["motion"] = (self.time_ns // period + 1) * period
        if self.time_ns >= self._next["poses"]:
            poses, _ = self.inspect()
            if self._latest(self.body_poses, poses):
                with self._status_lock:
                    self._status["dropped_poses"] += 1
            period = round(1e9 / self.config.pose_hz)
            self._next["poses"] = (self.time_ns // period + 1) * period

    def run(self) -> None:
        try:
            self.load()
            deadline = time.monotonic()
            while not self._done.is_set():
                self._process_operations()
                if self.status()["status"] == "fault":
                    self._done.wait(0.01)
                    continue
                self.prepare()
                self.publish_truth()
                pending = [
                    c
                    for c in self.config.controllers
                    if not self.time_ns % c.period_ns
                    and self._answered.get(c.controller_id) != self.time_ns
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
                            and self._last_joints.header.time_ns == self.time_ns
                        ):
                            self.joint_truth.put_nowait(self._last_joints)
                        for motion in self._last_motion:
                            if motion.header.time_ns == self.time_ns:
                                self.body_motion.put_nowait(motion)
                        self._last_repeat = now
                    deadline = now
                    self._done.wait(0.001)
                    continue
                if self._waiting_since is not None:
                    self._set_status(status="running", control_wait_s=now - self._waiting_since)
                    self._waiting_since = None
                    self._ever_controlled = True
                self.advance()
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
            if not self.ready.done():
                self.ready.set_exception(error)
        finally:
            self._done.set()
            self.backend.close()

    def reset(self) -> None:
        self.backend.reset()
        self._invalidate()

    def place(self, body_id: str, pose: tuple[float, ...]) -> None:
        self.backend.place(body_id, pose)
        self._invalidate()

    def _invalidate(self) -> None:
        self.epoch += 1
        self.time_ns = 0
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
        self.backend.hold(self.backend.driven_joint_ids)
        self._set_status(status="running", error="")

    def request(self, operation: str, *args: Any) -> Any:
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
                self.reset()
            elif operation == "place":
                self.place(*args)
            elif operation != "inspect":
                raise ValueError("unknown simulation operation")
            future.set_result(self.inspect(*args) if operation == "inspect" else self.inspect())
        except (ValueError, KeyError) as error:
            future.set_exception(error)

    def request_stop(self) -> None:
        self._done.set()
