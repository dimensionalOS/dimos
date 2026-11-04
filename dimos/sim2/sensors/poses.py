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

"""Shared bounded input/lifecycle for independent camera and lidar modules."""

from abc import ABC, abstractmethod
from queue import Empty, Full, Queue
import threading
from typing import Any, ClassVar

from dimos_lcm.sim_msgs import BodyPoses

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.sim2.description import ModelSourceSpec, SceneConfig


class PoseSensorConfig(ModuleConfig):
    scene: SceneConfig


class _PoseSensorModule(Module, ABC):
    config: PoseSensorConfig
    dedicated_worker: ClassVar[bool] = True
    _models: ModelSourceSpec
    body_poses: In[BodyPoses]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._queue: Queue[BodyPoses] = Queue(128)
        self._done = threading.Event()
        self._thread: threading.Thread | None = None
        self._stats_lock = threading.Lock()
        self._stats: dict[str, Any] = {
            "acquisitions": 0,
            "dropped_inputs": 0,
            "gaps": 0,
            "error": "",
        }

    @rpc
    def start(self) -> None:
        super().start()
        scene = self.config.scene.resolve(self._models.resolved_models())
        self._thread = threading.Thread(target=self._run, args=(scene,), daemon=True)
        self._thread.start()

    async def handle_body_poses(self, sample: BodyPoses) -> None:
        try:
            self._queue.put_nowait(sample)
        except Full:
            self.count("dropped_inputs")
            try:
                self._queue.get_nowait()
            except Empty:
                pass
            self._queue.put_nowait(sample)

    def _run(self, scene: SceneConfig) -> None:
        try:
            self.open_sensor(scene)
            try:
                while not self._done.is_set():
                    try:
                        sample = self._queue.get(timeout=0.1)
                    except Empty:
                        continue
                    self.acquire(sample, latest=self._queue.empty())
            finally:
                self.close_sensor()
        except Exception as error:
            # Worker boundary; visible to caller through statistics(), never swallowed.
            with self._stats_lock:
                self._stats["error"] = repr(error)

    @abstractmethod
    def open_sensor(self, scene: SceneConfig) -> None:
        raise NotImplementedError

    @abstractmethod
    def acquire(self, sample: BodyPoses, *, latest: bool) -> None:
        raise NotImplementedError

    @abstractmethod
    def close_sensor(self) -> None:
        raise NotImplementedError

    def count(self, key: str) -> None:
        with self._stats_lock:
            self._stats[key] += 1

    @rpc
    def statistics(self) -> dict[str, Any]:
        with self._stats_lock:
            return dict(self._stats)

    @rpc
    def stop(self) -> None:
        self._done.set()
        if self._thread is not None:
            self._thread.join(timeout=5)
            if self._thread.is_alive():
                raise RuntimeError("sensor worker did not stop")
        super().stop()
