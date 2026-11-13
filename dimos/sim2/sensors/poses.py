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
import time
from typing import Any, ClassVar

from dimos_lcm.sim_msgs import WorldState
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.robot.assets.model import UsdModel
from dimos.sim2.description import SceneConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PoseSensorConfig(SceneConfig):
    pass


class _PoseSensorModule(Module, ABC):
    config: PoseSensorConfig
    dedicated_worker: ClassVar[bool] = True
    observations: In[WorldState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._queue: Queue[WorldState] = Queue(128)
        self._done = threading.Event()
        self._thread: threading.Thread | None = None
        self._stats_lock = threading.Lock()
        self._stats: dict[str, Any] = {
            "acquisitions": 0,
            "dropped_inputs": 0,
            "gaps": 0,
            "overruns": 0,
            "error": "",
        }
        self._next_warning = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        # Rolling sensors need history, before the async handler's LATEST mailbox.
        self.register_disposable(Disposable(self.observations.subscribe(self._on_observations)))
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _on_observations(self, sample: WorldState) -> None:
        try:
            self._queue.put_nowait(sample)
        except Full:
            self.count("dropped_inputs")
            try:
                self._queue.get_nowait()
            except Empty:
                pass
            self._queue.put_nowait(sample)

    def _run(self) -> None:
        try:
            self.open_sensor(self.config.load_scene(self.rpc))
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
    def open_sensor(self, scene: UsdModel) -> None:
        raise NotImplementedError

    @abstractmethod
    def acquire(self, sample: WorldState, *, latest: bool) -> None:
        raise NotImplementedError

    @abstractmethod
    def close_sensor(self) -> None:
        raise NotImplementedError

    def count(self, key: str) -> None:
        with self._stats_lock:
            self._stats[key] += 1
            now = time.monotonic()
            if key in ("dropped_inputs", "gaps", "overruns") and now >= self._next_warning:
                logger.warning(
                    "Simulated sensor cannot keep up with its input or acquisition rate",
                    sensor=self.config.instance_name or type(self).__name__,
                    reason=key,
                    count=self._stats[key],
                )
                self._next_warning = now + 5.0

    @rpc
    def statistics(self) -> dict[str, Any]:
        with self._stats_lock:
            return dict(self._stats)

    def interrupt_sensor(self) -> None:
        """Wake an optional blocking native acquisition before joining its worker."""

    @rpc
    def stop(self) -> None:
        self._done.set()
        self.interrupt_sensor()
        if self._thread is not None:
            self._thread.join(timeout=5)
            if self._thread.is_alive():
                raise RuntimeError("sensor worker did not stop")
        super().stop()
