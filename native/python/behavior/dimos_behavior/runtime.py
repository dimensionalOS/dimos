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

"""Continuous simulator owner loop with nonblocking RPC operation handles."""

from collections.abc import Callable, Iterator
import copy
import queue
import signal
import threading
import time
from typing import Any
from uuid import uuid4

from dimos.core.core import rpc
from dimos.experimental.isolated_python.bootstrap import load_class
from dimos.simulation.behavior.connection import BehaviorConnection
from dimos.simulation.behavior.control import RobotControl
from dimos.simulation.behavior.types import (
    BehaviorStatus,
    ControlMode,
    Episode,
    Operation,
    TaskSelection,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class BehaviorRuntime(BehaviorConnection):
    def __init__(self, **kwargs: Any) -> None:
        self._lock = threading.RLock()
        self._ready = threading.Event()
        self._started = threading.Event()
        self._shutdown = threading.Event()
        self._requests: queue.SimpleQueue[tuple[str, str, Any]] = queue.SimpleQueue()
        self._operations: dict[str, Operation] = {}
        self._cancelled: set[str] = set()
        self._busy: str | None = None
        self._primitive: Iterator[Any] | None = None
        self._primitive_id: str | None = None
        self._mailbox: dict[str, tuple[int, Any, float]] = {}
        self._description: dict[str, Any] = {}
        self._truth: dict[str, Any] = {}
        self._tasks: list[TaskSelection] = []
        self._scenes: list[str] = []
        self._state = BehaviorStatus(episode=Episode(id=uuid4().hex))
        self._engine: Any = None
        self._subscriptions: list[Callable[[], None]] = []
        super().__init__(**kwargs)
        self._control = RobotControl(self.config.command_timeout)

    @rpc
    def build(self) -> None:
        super().build()
        if not self._ready.wait(1700):
            raise TimeoutError("OmniGibson initialization timed out")
        if self._state.state == "error":
            raise RuntimeError(self._state.error)

    @rpc
    def start(self) -> None:
        super().start()
        self._subscriptions = [
            self.cmd_vel.subscribe(
                lambda message: self._command("velocity", message, ControlMode.DIMOS)
            ),
            self.joint_command.subscribe(
                lambda message: self._command("joints", message, ControlMode.DIMOS)
            ),
            self.native_action.subscribe(
                lambda message: self._command("native", message, ControlMode.NATIVE)
            ),
        ]
        self._started.set()

    @rpc
    def stop(self) -> None:
        self._shutdown.set()
        for subscription in self._subscriptions:
            subscription()
        self._subscriptions.clear()
        super().stop()

    def _command(self, kind: str, message: Any, mode: ControlMode) -> None:
        with self._lock:
            if self._state.state != "running" or self._control.mode != mode:
                return
            # While a takeover/reset waits for the engine, do not accept commands.
            if self._busy is not None:
                return
            self._mailbox[kind] = (self._control.generation, message, time.monotonic())

    @rpc
    def describe(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._description)

    @rpc
    def list_tasks(self) -> list[TaskSelection]:
        with self._lock:
            return [task.model_copy(deep=True) for task in self._tasks]

    @rpc
    def list_scenes(self) -> list[str]:
        with self._lock:
            return list(self._scenes)

    @rpc
    def get_status(self) -> BehaviorStatus:
        with self._lock:
            return self._state.model_copy(deep=True)

    @rpc
    def get_ground_truth(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._truth)

    @rpc
    def get_operation(self, operation_id: str) -> Operation:
        with self._lock:
            return self._operations[operation_id].model_copy(deep=True)

    def _submit(self, kind: str, payload: Any = None, interrupt: bool = False) -> str:
        with self._lock:
            if not self._ready.is_set() or self._state.state in ("error", "stopped"):
                raise RuntimeError(f"Simulator is {self._state.state}: {self._state.error}")
            if self._busy is not None:
                if not interrupt:
                    raise RuntimeError("busy: an operation is already running")
                self._cancelled.add(self._busy)
            op = Operation(id=uuid4().hex, kind=kind, episode=self._state.episode.id)
            self._operations[op.id] = op
            self._state.operation = op
            self._busy = op.id
            self._mailbox.clear()
            self._requests.put((op.id, kind, payload))
            return op.id

    @rpc
    def load_task(self, task: TaskSelection) -> str:
        if not self.config.allow_task_changes:
            raise RuntimeError("Restart the navigation blueprint to change scenes")
        return self._submit("load", task, interrupt=True)

    @rpc
    def reset_task(self) -> str:
        if not self.config.allow_task_changes:
            raise RuntimeError("Restart the navigation blueprint to reset its map and scene")
        return self._submit("reset", interrupt=True)

    @rpc
    def cancel_operation(self, operation_id: str) -> None:
        with self._lock:
            if self._operations[operation_id].state == "running":
                self._cancelled.add(operation_id)

    @rpc
    def take_control(self, mode: ControlMode) -> str:
        return self._submit("control", ControlMode(mode), interrupt=True)

    @rpc
    def stop_motion(self) -> str:
        return self.take_control(ControlMode.PRIMITIVE)

    @rpc
    def pause(self) -> str:
        return self._submit("pause", interrupt=True)

    @rpc
    def resume(self) -> str:
        return self._submit("resume")

    @rpc
    def start_primitive(self, kind: str, primitive: str, target: str = "") -> str:
        with self._lock:
            if self._control.mode != ControlMode.PRIMITIVE:
                raise RuntimeError("Call take_control('primitive') and wait for completion first")
            if self._state.state != "running":
                raise RuntimeError("Primitives require a running, nonterminal episode")
            if kind not in ("physical", "symbolic"):
                raise ValueError("kind must be physical or symbolic")
            if primitive not in self._description[f"{kind}_primitives"]:
                raise ValueError(f"Unsupported {kind} primitive: {primitive}")
            return self._submit("primitive", (kind, primitive, target))

    def _finish(self, operation_id: str, error: Exception | None = None) -> None:
        with self._lock:
            operation = self._operations[operation_id]
            operation.state = (
                "cancelled"
                if operation_id in self._cancelled
                else "failed"
                if error
                else "succeeded"
            )
            operation.error = str(error) if error else None
            self._cancelled.discard(operation_id)
            if self._busy == operation_id:
                self._busy = None

    def _hold(self, mode: ControlMode = ControlMode.PRIMITIVE) -> None:
        measured = self._engine.measured()
        with self._lock:
            self._control.transfer(mode, measured)
            self._state.control = mode
            self._mailbox.clear()

    def _close_primitive(self, error: Exception | None = None) -> None:
        generator, operation_id = self._primitive, self._primitive_id
        self._primitive = None
        self._primitive_id = None
        if generator is not None:
            close = getattr(generator, "close", None)
            if close is not None:
                close()
        if operation_id is not None:
            self._finish(operation_id, error)
        self._hold()

    def _process_requests(self) -> None:
        if self._primitive_id is not None and self._primitive_id in self._cancelled:
            self._close_primitive()
        while not self._requests.empty():
            operation_id, kind, payload = self._requests.get_nowait()
            if operation_id in self._cancelled:
                self._finish(operation_id)
                continue
            try:
                if kind == "primitive":
                    self._primitive_id = operation_id
                    self._primitive = self._engine.primitive(*payload)
                    with self._lock:
                        kinds = self._state.episode.execution_kinds
                        if payload[0] not in kinds:
                            kinds.append(payload[0])
                    continue
                if kind in ("load", "reset"):
                    self._close_primitive()
                    with self._lock:
                        self._state.state = "loading"
                    self._engine.reset(payload if kind == "load" else None)
                    with self._lock:
                        self._state.episode = Episode(id=uuid4().hex, task=self._engine.task)
                        self._state.state = (
                            "paused" if operation_id in self._cancelled else "running"
                        )
                    self._hold()
                    self._refresh()
                elif kind == "control":
                    self._close_primitive()
                    self._hold(payload)
                elif kind == "pause":
                    self._close_primitive()
                    with self._lock:
                        if self._state.state != "finished":
                            self._state.state = "paused"
                elif kind == "resume":
                    with self._lock:
                        if self._state.state == "finished":
                            raise RuntimeError("Episode finished; reset before resuming")
                        self._state.state = "running"
                    self._hold(self._control.mode)
                self._finish(operation_id)
            except Exception as error:
                self._primitive = None
                self._primitive_id = None
                self._finish(operation_id, error)
                if kind in ("load", "reset"):
                    with self._lock:
                        self._state.state, self._state.error = "error", str(error)

    def _consume_commands(self) -> None:
        with self._lock:
            commands, self._mailbox = self._mailbox, {}
        for kind, (generation, message, received) in commands.items():
            if (
                generation != self._control.generation
                or time.monotonic() - received > self.config.command_timeout
            ):
                continue
            try:
                if kind == "velocity":
                    if message.linear.z != 0 or message.angular.x != 0 or message.angular.y != 0:
                        raise ValueError("R1 base accepts planar x/y/yaw velocity only")
                    self._control.set_velocity(
                        (message.linear.x, message.linear.y, message.angular.z), received
                    )
                elif kind == "joints":
                    if message.velocity or message.effort:
                        raise ValueError("Only absolute joint positions are supported")
                    self._control.set_joints(message.name, message.position, self._engine.limits)
                else:
                    self._control.set_action(message, self._engine.bounds, received)
            except ValueError as error:
                logger.warning("Rejected BEHAVIOR command", error=str(error))

    def _refresh(self) -> None:
        truth = self._engine.ground_truth()
        with self._lock:
            tags = {"episode": self._state.episode.id, "step": self._state.episode.step}
            self._truth = {**tags, **truth}
        if self._started.is_set():
            for name, message in self._engine.messages(time.time()).items():
                getattr(self, name).publish(message)

    def _tick(self) -> None:
        self._process_requests()
        if self._state.state != "running":
            return
        self._consume_commands()
        if (
            self._control.mode == ControlMode.NATIVE
            and self._control.action is not None
            and self._control.get_action(time.monotonic()) is None
        ):
            self._hold(ControlMode.NATIVE)
        action = self._engine.action(self._control, time.monotonic())
        if self._primitive is not None:
            try:
                generated = next(self._primitive)
                if generated is not None:
                    action = generated
            except StopIteration:
                self._close_primitive()
                action = self._engine.action(self._control, time.monotonic())
            except Exception as error:
                self._close_primitive(error)
                action = self._engine.action(self._control, time.monotonic())
        # A cancellation received during planning takes effect before its action is applied.
        if self._primitive_id is not None and self._primitive_id in self._cancelled:
            self._close_primitive()
            action = self._engine.action(self._control, time.monotonic())
        if self._shutdown.is_set():
            return
        reward, terminated, truncated, info = self._engine.step(action)
        with self._lock:
            episode = self._state.episode
            episode.step += 1
            episode.reward = reward
            episode.terminated, episode.truncated, episode.info = terminated, truncated, info
            episode.success = bool(info.get("done", {}).get("success", False))
            if terminated or truncated:
                self._state.state = "finished"
        if terminated or truncated:
            # Evaluator success is distinct from the primitive's postcondition result.
            if self._primitive_id is not None:
                if not self._state.episode.success:
                    self._close_primitive(
                        RuntimeError("Episode terminated before action completed")
                    )
                else:
                    self._close_primitive()
            self._hold()
        self._refresh()

    def run_runtime(self, stopping: threading.Event) -> None:
        try:
            # Load the engine only here: its imports and all simulator calls belong to this thread.
            handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
            try:
                engine_class = load_class("dimos_behavior.engine:OmniEngine")
                self._engine = engine_class(self.config)
                self._engine.initialize()
            finally:
                # OmniGibson installs its own handlers; bootstrap owns process lifecycle.
                for sig, handler in handlers.items():
                    signal.signal(sig, handler)
            self._description = self._engine.describe()
            self._tasks = self._engine.list_tasks()
            self._scenes = self._engine.list_scenes()
            self._state.episode.task = self.config.task
            self._hold(ControlMode.DIMOS)
            self._refresh()
            self._state.state = "running"
            self._ready.set()
            last = time.monotonic()
            while not stopping.is_set() and not self._shutdown.is_set():
                start = time.monotonic()
                if self._started.is_set():
                    self._tick()
                    elapsed = start - last
                    with self._lock:
                        self._state.achieved_hz = 1 / elapsed if elapsed else 0
                    self.status.publish(self.get_status())
                last = start
                self._shutdown.wait(max(0, 1 / self.config.action_hz - (time.monotonic() - start)))
        except Exception as error:
            logger.exception("BEHAVIOR runtime failed", error=str(error))
            with self._lock:
                self._state.state, self._state.error = "error", str(error)
            self._ready.set()
            # Keep status RPCs available so a failed asynchronous load is inspectable.
            while not stopping.is_set() and not self._shutdown.wait(0.1):
                pass
        finally:
            if self._engine is not None:
                self._engine.close()
            with self._lock:
                self._state.state = "stopped"
