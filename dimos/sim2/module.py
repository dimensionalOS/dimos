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

"""DimOS lifecycle and typed ports around the standalone Simulation owner."""

from collections.abc import Callable
from queue import Empty
import threading
from typing import Any, ClassVar, cast

from dimos_lcm.control_msgs import BodyWrench, JointCommand
from dimos_lcm.sensor_msgs import JointState
from dimos_lcm.sim_msgs import BodyMotion, BodyPoses, SimulationClock

from dimos.core.core import rpc
from dimos.core.module import ModuleConfig
from dimos.core.shared_config import ConfigSenders, SharedConfigModule
from dimos.core.stream import In, Out
from dimos.sim2.description import SceneConfig
from dimos.sim2.physics.interface import PhysicsBackend
from dimos.sim2.physics.simulation import Simulation, SimulationConfig


class SimConfig(ModuleConfig):
    scene: SceneConfig
    backend: Callable[[], PhysicsBackend]
    timing: SimulationConfig = SimulationConfig()


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
        self._done = threading.Event()
        self._threads: list[threading.Thread] = []
        self._simulation: Simulation | None = None
        self._error = ""
        self._lock = threading.Lock()

    def config_receive(self, config_senders: ConfigSenders) -> None:
        self.config.scene = self.config.scene.resolve(config_senders)

    @rpc
    def start(self) -> None:
        super().start()
        sim = Simulation(self.config.backend(), self.config.scene, self.config.timing)
        self._simulation = sim
        self._threads = [threading.Thread(target=sim.run, name="sim2-physics", daemon=True)]
        self._threads[0].start()
        sim.ready.result(timeout=30)
        # Scene serialization cannot block high-rate motion/joint publication.
        for name, target in (
            ("sim2-truth", self._publish_truth),
            ("sim2-scene", self._publish_scene),
        ):
            thread = threading.Thread(target=target, name=name, daemon=True)
            self._threads.append(thread)
            thread.start()

    def _owner(self) -> Simulation:
        if self._simulation is None:
            raise RuntimeError("simulation has not started")
        return self._simulation

    def _publish_truth(self) -> None:
        sim = self._owner()
        try:
            while not self._done.is_set():
                sent = False
                for queue, output in (
                    (sim.joint_truth, self.joint_truth),
                    (sim.body_motion, self.body_motion),
                    (sim.clock, self.clock),
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
        sim = self._owner()
        try:
            while not self._done.is_set():
                try:
                    poses = sim.body_poses.get(timeout=0.1)
                except Empty:
                    continue
                self.body_poses.publish(poses)
        except Exception as error:
            self._publication_failed(error)

    def _publication_failed(self, error: Exception) -> None:
        with self._lock:
            self._error = repr(error)
        self._owner().request_stop()
        self._done.set()

    @rpc
    def submit(self, command: JointCommand | BodyWrench) -> None:
        self._owner().submit(command)

    async def handle_joint_command(self, command: JointCommand) -> None:
        self._owner().submit(command)

    async def handle_body_wrench(self, command: BodyWrench) -> None:
        self._owner().submit(command)

    @rpc
    def inspect(self, joints: tuple[str, ...] = ()) -> tuple[BodyPoses, JointState]:
        return cast("tuple[BodyPoses, JointState]", self._owner().request("inspect", joints))

    @rpc
    def reset(self) -> tuple[BodyPoses, JointState]:
        return cast("tuple[BodyPoses, JointState]", self._owner().request("reset"))

    @rpc
    def place(self, body_id: str, pose: tuple[float, ...]) -> tuple[BodyPoses, JointState]:
        return cast("tuple[BodyPoses, JointState]", self._owner().request("place", body_id, pose))

    @rpc
    def status(self) -> dict[str, Any]:
        result = self._owner().status()
        with self._lock:
            result["publication_error"] = self._error
        return result

    @rpc
    def stop(self) -> None:
        self._done.set()
        if self._simulation is not None:
            self._simulation.request_stop()
        for thread in self._threads:
            thread.join(timeout=5)
            if thread.is_alive():
                raise RuntimeError(f"{thread.name} did not stop")
        super().stop()
