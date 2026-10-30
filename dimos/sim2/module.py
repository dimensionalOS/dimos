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

"""Run a physics backend in a DimOS worker and expose its streams/RPCs.

Construction, stepping, reset and destruction all happen on the same thread,
including any upstream renderer's OpenGL context. Existing tasks keep their
native action frequency, horizon, reset distribution and observation dictionary.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from concurrent.futures import Future
import functools
import queue
import threading
import time
from typing import Any, TypeVar

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.sim2.messages import EnvironmentStep, StateFrame
from dimos.sim2.physics import (
    CommandBatch,
    CommandReport,
    PhysicsBackend,
    WorldDescription,
    WorldState,
)
from dimos.sim2.sensors.composition import Sensor
from dimos.sim2.sensors.observations import ObservationFrame
from dimos.sim2.spec import RobotInstanceSpec
from dimos.sim2.visuals import VisualScene
from dimos.sim2.world.runtime import SimulationRuntime
from dimos.sim2.world.scene_types import SceneDescription
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
OwnerResult = TypeVar("OwnerResult")

COMMAND_PORT = "motor_command"
COMMAND_PORTS = (COMMAND_PORT, "cmd_vel")
FEEDBACK_PORTS = ("motor_states", "imu", "odom", "tf")
PORT_KINDS = (*COMMAND_PORTS, *FEEDBACK_PORTS)
PORT_TYPES: dict[str, type] = {
    COMMAND_PORT: MotorCommandArray,
    "cmd_vel": Twist,
    "motor_states": JointState,
    "imu": Imu,
    "odom": PoseStamped,
    "tf": TFMessage,
}


def robot_ports(module: type[Module], robots: Iterable[str]) -> dict[str, dict[str, str]]:
    """Map each robot to the port attributes a module class declares for it.

    A port named for a robot that is not in the world is a configuration error,
    so it raises instead of being silently unused.
    """
    known = set(robots)
    ports: dict[str, dict[str, str]] = {robot: {} for robot in known}
    names: set[str] = set()
    for base in reversed(module.__mro__):
        names.update(getattr(base, "__annotations__", {}))
    for name in sorted(names):
        for kind in PORT_KINDS:
            suffix = "_" + kind
            if name.endswith(suffix) and len(name) > len(suffix):
                robot = name[: -len(suffix)]
                if robot not in known:
                    raise ValueError(
                        f"{module.__name__}.{name} names robot {robot!r}, which is not in this world"
                    )
                ports[robot][kind] = name
    return ports


class SimulationModuleConfig(ModuleConfig):
    environment: Callable[[], Any] | None = None
    backend: Callable[[], PhysicsBackend] | None = None
    build_timeout_s: float = 180.0
    robots: dict[str, RobotInstanceSpec] = {}
    sensors: tuple[tuple[str | None, Sensor], ...] = ()
    snapshot_hz: float = 60.0
    robot_topics: dict[str, dict[str, str]] = {}


class SimulationModule(Module):
    config: SimulationModuleConfig
    dedicated_worker = True
    world_state: Out[WorldState]
    commands: In[CommandBatch]
    command_reports: Out[CommandReport]
    snapshots: Out[StateFrame]
    observations: Out[ObservationFrame]
    environment_result: Out[EnvironmentStep]
    actions: In[list[float]]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._runtime: SimulationRuntime | None = None
        self._stop = threading.Event()
        self._running = threading.Event()
        self._ready = threading.Event()
        self._thread: threading.Thread | None = None
        self._lifecycle_lock = threading.RLock()
        self._requests: queue.Queue[tuple[Callable[[SimulationRuntime], Any], Future[Any]]] = (
            queue.Queue()
        )
        self._failure: str | None = None
        self._outputs: dict[str, dict[str, Out[Any] | PubSubTransport[Any]]] = {}
        self._rejected: dict[str, int] = {}

    @rpc
    def build(self) -> None:
        with self._lifecycle_lock:
            if self._thread is None:
                self._stop.clear()
                self._ready.clear()
                self._failure = None
                self._thread = threading.Thread(target=self._run, name="physics-owner", daemon=True)
                self._thread.start()
        if not self._ready.wait(timeout=self.config.build_timeout_s):
            self._stop.set()
            raise TimeoutError("physics backend did not initialize within build_timeout_s")
        self._require_runtime()

    def _on_owner(self, fn: Callable[[SimulationRuntime], OwnerResult]) -> OwnerResult:
        future: Future[OwnerResult] = Future()
        # Admission and owner shutdown share this lock: a request cannot arrive
        # after the owner's final queue drain.
        with self._lifecycle_lock:
            self._require_runtime()
            self._requests.put((fn, future))
        return future.result(timeout=self.config.build_timeout_s)

    @rpc
    def describe(self) -> dict[str, Any]:
        runtime = self._require_runtime()
        with runtime.lock:
            if runtime is not self._runtime:
                raise RuntimeError("simulation stopped")
            return runtime.describe()

    @rpc
    def describe_scene(self) -> SceneDescription:
        return self._require_runtime().description

    @rpc
    def status(self) -> dict[str, Any]:
        runtime = self._runtime
        if runtime is None:
            return {"running": False, "error": self._failure}
        with runtime.lock:
            if runtime is not self._runtime:
                return {"running": False, "error": self._failure}
            return {
                "running": self._running.is_set()
                and not self._stop.is_set()
                and self._failure is None,
                "sim_time": runtime.sim_time,
                "steps": runtime.tick,
                "episode": runtime.episode,
                "paused": runtime.paused,
                "waiting_for_control": sorted(runtime.waiting_for_control),
                "rejected_commands": dict(self._rejected),
                "error": self._failure,
                "robots": {
                    name: [float(value) for value in state.base.position]
                    for name, state in runtime.last_state.robots.items()
                },
                "done": runtime.backend.done,
                "backend": runtime.backend.name,
            }

    @rpc
    def describe_world(self) -> WorldDescription:
        runtime = self._require_runtime()
        with runtime.lock:
            return runtime.backend.description

    @rpc
    def describe_visuals(self) -> VisualScene:
        return self._on_owner(lambda runtime: runtime.backend.describe_visuals())

    @rpc
    def read_state(self) -> WorldState:
        runtime = self._require_runtime()
        with runtime.lock:
            return runtime.last_state

    @rpc
    def submit(self, batch: CommandBatch) -> CommandReport:
        return self._require_runtime().submit(batch)

    @rpc
    def reset(self) -> None:
        """Reset the upstream environment and invalidate commands and sensor history."""
        self._on_owner(lambda runtime: runtime.reset())

    @rpc
    def set_paused(self, paused: bool) -> None:
        self._on_owner(lambda runtime: runtime.set_paused(paused))

    @rpc
    def set_action(self, action: list[float]) -> None:
        """Supply a native upstream action, e.g. the Panda Lift environment's action."""
        runtime = self._require_runtime()
        with runtime.lock:
            if runtime is not self._runtime or self._stop.is_set():
                raise RuntimeError("simulation stopped")
            runtime.accept_action(action)

    @rpc
    def start(self) -> None:
        super().start()
        self.build()
        runtime = self._require_runtime()
        for robot_id, names in robot_ports(type(self), self.config.robots).items():
            outputs: dict[str, Out[Any] | PubSubTransport[Any]] = {}
            for kind, attribute in names.items():
                port = getattr(self, attribute)
                if kind in COMMAND_PORTS:
                    assert isinstance(port, In)
                    self.register_disposable(
                        Disposable(port.subscribe(functools.partial(self._command, robot_id)))
                    )
                else:
                    assert isinstance(port, Out)
                    outputs[kind] = port
            self._outputs[robot_id] = outputs
        for robot_id, topics in self.config.robot_topics.items():
            if robot_id not in self.config.robots:
                raise ValueError(f"stream configuration names unknown robot {robot_id!r}")
            declared = robot_ports(type(self), self.config.robots)[robot_id]
            if declared.keys() & topics.keys():
                raise ValueError(f"{robot_id}: ports and robot_topics configure the same IO")
            for kind, topic in topics.items():
                if kind not in PORT_TYPES:
                    raise ValueError(f"unknown robot IO kind {kind!r}")
                transport = make_transport(topic, PORT_TYPES[kind], g=self.config.g)
                self.register_disposable(Disposable(transport.stop))
                transport.start()
                if kind in COMMAND_PORTS:
                    self.register_disposable(
                        Disposable(transport.subscribe(functools.partial(self._command, robot_id)))
                    )
                else:
                    self._outputs[robot_id][kind] = transport
        runtime.publish_feedback = self._publish_feedback
        runtime.publish_snapshot = self.snapshots.publish
        runtime.publish_state = self.world_state.publish
        runtime.publish_report = self.command_reports.publish
        self.register_disposable(Disposable(self.commands.subscribe(self.submit)))
        runtime.publish_observation = self.observations.publish
        runtime.publish_environment = self.environment_result.publish
        self.register_disposable(Disposable(self.actions.subscribe(self.set_action)))
        self._running.set()

    def _command(self, robot_id: str, message: MotorCommandArray | Twist) -> None:
        runtime = self._runtime
        if runtime is None:
            return
        with runtime.lock:
            if runtime is not self._runtime or self._stop.is_set():
                return
            accepted = (
                runtime.accept_velocity(robot_id, message)
                if isinstance(message, Twist)
                else runtime.accept_command(robot_id, message)
            )
            if accepted:
                return
        count = self._rejected.get(robot_id, 0) + 1
        self._rejected[robot_id] = count
        if count == 1 or count % 100 == 0:
            logger.warning("sim2 rejected a motor command", robot=robot_id, rejected=count)

    def _publish_feedback(
        self, robot_id: str, joints: JointState, imu: Imu | None, odom: PoseStamped
    ) -> None:
        outputs = self._outputs.get(robot_id)
        if not outputs:
            return
        if (port := outputs.get("motor_states")) is not None:
            port.publish(joints)
        if imu is not None and (port := outputs.get("imu")) is not None:
            port.publish(imu)
        if (port := outputs.get("odom")) is not None:
            port.publish(odom)
        if (port := outputs.get("tf")) is not None:
            port.publish(
                TFMessage(
                    Transform(
                        translation=odom.position,
                        rotation=odom.orientation,
                        frame_id=odom.frame_id,
                        child_frame_id=joints.frame_id,
                        ts=odom.ts,
                    )
                )
            )

    def _run(self) -> None:
        runtime: SimulationRuntime | None = None
        try:
            if self.config.environment is None and self.config.backend is None:
                raise ValueError("simulation needs a backend or a native task factory")
            runtime = SimulationRuntime(
                self.config.environment,
                self.config.robots,
                self.config.sensors,
                self.config.snapshot_hz,
                backend=self.config.backend() if self.config.backend is not None else None,
            )
            runtime.build()
            with self._lifecycle_lock:
                self._runtime = runtime
                self._ready.set()
            deadline = time.monotonic()
            while not self._stop.is_set():
                while True:
                    try:
                        fn, future = self._requests.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        future.set_result(fn(runtime))
                    except Exception as error:
                        future.set_exception(error)
                if self._running.is_set():
                    runtime.step()
                    deadline += runtime.step_seconds
                    now = time.monotonic()
                    if now - deadline > 0.1:
                        deadline = now
                    self._stop.wait(max(0.0, deadline - now))
                else:
                    self._stop.wait(0.01)
                    deadline = time.monotonic()
        except Exception as error:
            self._failure = str(error)
            logger.exception("physics backend stopped")
        finally:
            with self._lifecycle_lock:
                self._runtime = None
                self._running.clear()
                self._fail_requests()
            try:
                if runtime is not None:
                    with runtime.lock:
                        runtime.close()
            except Exception as error:
                cleanup_error = f"backend close failed: {type(error).__name__}: {error}"
                self._failure = (
                    f"{self._failure}; {cleanup_error}" if self._failure else cleanup_error
                )
                logger.exception("physics backend cleanup failed")
            finally:
                self._ready.set()

    def _fail_requests(self) -> None:
        """Fail requests still queued, with the lifecycle lock held."""
        while True:
            try:
                _, future = self._requests.get_nowait()
            except queue.Empty:
                break
            if not future.done():
                future.set_exception(RuntimeError(self._failure or "simulation stopped"))

    def _require_runtime(self) -> SimulationRuntime:
        if self._failure is not None:
            raise RuntimeError(f"physics backend failed: {self._failure}")
        if self._stop.is_set():
            raise RuntimeError("simulation stopped")
        if self._runtime is None:
            raise RuntimeError("simulation is not running")
        return self._runtime

    @rpc
    def stop(self) -> None:
        with self._lifecycle_lock:
            self._stop.set()
            self._running.clear()
            self._fail_requests()
            thread = self._thread
        try:
            if thread is not None:
                thread.join(timeout=10)
                if thread.is_alive():
                    raise RuntimeError("robosuite owner thread did not stop")
                with self._lifecycle_lock:
                    if self._thread is thread:
                        self._thread = None
        finally:
            super().stop()
