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

"""Memory module — record input streams into persistent memory."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, TypeVar

import cv2

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.memory.impl.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import sharpness_barrier
from dimos.utils.logging_config import setup_logger

cv2.setNumThreads(1)

if TYPE_CHECKING:
    from reactivex.observable import Observable

    from dimos.core.stream import In
    from dimos.memory.stream import Stream
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

T = TypeVar("T")

logger = setup_logger()


@dataclass
class MemoryModuleConfig(ModuleConfig):
    db_path: str = "memory.db"
    world_frame: str = "world"
    robot_frame: str = "base_link"


class MemoryModule(Module[MemoryModuleConfig]):
    default_config: type[MemoryModuleConfig] = MemoryModuleConfig

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._store: SqliteStore | None = None

    def pose(self) -> PoseStamped | None:
        return self.tf.get_pose(self.config.world_frame, self.config.robot_frame)  # type: ignore[no-any-return]

    @rpc
    def start(self) -> None:
        super().start()
        self._store = SqliteStore(self.config.db_path)
        self._disposables.add(self._store)
        logger.info("MemoryModule started (db=%s)", self.config.db_path)

    def memory(
        self,
        input: In[T],
        name: str | None = None,  # can be infered from input
        payload_type: type | None = None,  #  can be infered from input
        fps: float = 0,
    ) -> Stream[T]:
        assert self._store is not None, "record() called before start()"

        if name is None:
            name = input.name
        if payload_type is None:
            payload_type = input.type

        session = self._store.session()
        self._disposables.add(session)

        memory_stream = session.stream(name, payload_type, pose_provider=self.pose)

        obs: Observable[Any] = input.observable()
        if fps > 0:
            obs = obs.pipe(sharpness_barrier(fps))

        self._disposables.add(obs.subscribe(on_next=memory_stream.append))

        return memory_stream

    @rpc
    def stop(self) -> None:
        super().stop()


memory_module = MemoryModule.blueprint
