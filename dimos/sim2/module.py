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

"""DimOS lifecycle owner of one MuJoCo world.

Retained connections use the private standard-message transports in
``robot_topics``. Explicit compositions may instead declare per-robot ports on
a subclass, e.g. ``g1_motor_command: In[MotorCommandArray]``. A robot without
either IO configuration holds its start pose. Sensor workers read snapshots.
"""

from __future__ import annotations

from collections.abc import Iterable
import functools
from pathlib import Path
import tempfile
import threading
import time
from typing import Any

import mujoco
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.sim2.messages import StateFrame
from dimos.sim2.spec import WorldConfig
from dimos.sim2.world.runtime import SimulationRuntime
from dimos.sim2.world.scene_types import SceneDescription
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

COMMAND_PORT = "motor_command"
FEEDBACK_PORTS = ("motor_states", "imu", "odom", "tf")
PORT_KINDS = (COMMAND_PORT, *FEEDBACK_PORTS)
PORT_TYPES: dict[str, type] = {
    COMMAND_PORT: MotorCommandArray,
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
    world: WorldConfig
    robot_topics: dict[str, dict[str, str]] = {}


class SimulationModule(Module):
    config: SimulationModuleConfig
    dedicated_worker = True
    snapshots: Out[StateFrame]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._runtime: SimulationRuntime | None = None
        self._directory: tempfile.TemporaryDirectory[str] | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lifecycle_lock = threading.RLock()
        self._failure: str | None = None
        self._outputs: dict[str, dict[str, Out[Any] | PubSubTransport[Any]]] = {}
        self._rejected: dict[str, int] = {}

    @rpc
    def build(self) -> None:
        with self._lifecycle_lock:
            if self._runtime is not None:
                return
            runtime = SimulationRuntime(self.config.world)
            directory = tempfile.TemporaryDirectory(prefix="dimos-sim2-")
            try:
                mujoco.mj_saveModel(runtime.model, str(Path(directory.name) / "world.mjb"), None)
            except BaseException:
                runtime.close()
                directory.cleanup()
                raise
            self._runtime, self._directory = runtime, directory
            logger.info(
                "sim2 world prepared", robots=list(self.config.world.robots), nq=runtime.model.nq
            )

    @rpc
    def describe(self) -> dict[str, Any]:
        """The compiled model file and snapshot timing a sensor worker needs."""
        with self._lifecycle_lock:
            if self._runtime is None or self._directory is None:
                raise RuntimeError("SimulationModule.build() has not completed")
            model = Path(self._directory.name) / "world.mjb"
            return {"model": str(model), **self._runtime.describe()}

    @rpc
    def describe_scene(self) -> SceneDescription:
        """The scene sidecar: named entities, fixture joints, regions and spawns."""
        with self._lifecycle_lock:
            return self._require_runtime().description

    @rpc
    def status(self) -> dict[str, Any]:
        with self._lifecycle_lock:
            runtime = self._runtime
            if runtime is None:
                return {"running": False, "error": self._failure}
            with runtime.lock:
                return {
                    "running": self._failure is None
                    and not self._stop.is_set()
                    and self._thread is not None
                    and self._thread.is_alive(),
                    "sim_time": float(runtime.data.time),
                    "steps": runtime.tick,
                    "episode": runtime.episode,
                    "paused": runtime.paused,
                    "waiting_for_control": sorted(runtime.waiting_for_control),
                    "rejected_commands": dict(self._rejected),
                    "error": self._failure,
                    "robots": {
                        name: runtime.data.xpos[b.root].tolist()
                        for name, b in runtime.robots.items()
                    },
                }

    @rpc
    def reset(self) -> None:
        """Restore the authored physical state; controller reset is the application's job."""
        with self._lifecycle_lock:
            self._require_runtime().reset()

    @rpc
    def set_paused(self, paused: bool) -> None:
        """Pause only physics; RPCs and state inspection stay available."""
        with self._lifecycle_lock:
            self._require_runtime().set_paused(paused)

    @rpc
    def start(self) -> None:
        super().start()
        self.build()
        runtime = self._require_runtime()
        for robot_id, names in robot_ports(type(self), self.config.world.robots).items():
            outputs: dict[str, Out[Any] | PubSubTransport[Any]] = {}
            for kind, attribute in names.items():
                port = getattr(self, attribute)
                if kind == COMMAND_PORT:
                    assert isinstance(port, In)
                    self.register_disposable(
                        Disposable(port.subscribe(functools.partial(self._command, robot_id)))
                    )
                else:
                    assert isinstance(port, Out)
                    outputs[kind] = port
            self._outputs[robot_id] = outputs
        for robot_id, topics in self.config.robot_topics.items():
            if robot_id not in self.config.world.robots:
                raise ValueError(f"stream configuration names unknown robot {robot_id!r}")
            declared = robot_ports(type(self), self.config.world.robots)[robot_id]
            if declared.keys() & topics.keys():
                raise ValueError(f"{robot_id}: ports and robot_topics configure the same IO")
            for kind, topic in topics.items():
                if kind not in PORT_TYPES:
                    raise ValueError(f"unknown robot IO kind {kind!r}")
                transport = make_transport(topic, PORT_TYPES[kind], g=self.config.g)
                self.register_disposable(Disposable(transport.stop))
                transport.start()
                if kind == COMMAND_PORT:
                    self.register_disposable(
                        Disposable(transport.subscribe(functools.partial(self._command, robot_id)))
                    )
                else:
                    self._outputs[robot_id][kind] = transport
        runtime.publish_feedback = self._publish_feedback
        runtime.publish_snapshot = self.snapshots.publish
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="sim2-physics", daemon=True)
        self._thread.start()

    def _command(self, robot_id: str, message: MotorCommandArray) -> None:
        runtime = self._runtime
        if runtime is None or runtime.accept_command(robot_id, message):
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
        runtime = self._require_runtime()
        deadline = time.monotonic()
        try:
            while not self._stop.is_set():
                runtime.step()
                deadline += self.config.world.timestep
                now = time.monotonic()
                if now - deadline > 0.1:
                    deadline = now
                self._stop.wait(max(0.0, deadline - now))
        except Exception as error:
            logger.exception("sim2 physics stopped")
            with self._lifecycle_lock:
                self._failure = str(error)

    def _require_runtime(self) -> SimulationRuntime:
        if self._runtime is None:
            raise RuntimeError("simulation is not running")
        if self._failure is not None:
            raise RuntimeError(f"simulation physics failed: {self._failure}")
        return self._runtime

    @rpc
    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5)
            if self._thread.is_alive():
                raise RuntimeError("sim2 physics did not stop")
        with self._lifecycle_lock:
            if self._runtime is not None:
                self._runtime.close()
                self._runtime = None
            if self._directory is not None:
                self._directory.cleanup()
                self._directory = None
        super().stop()
