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

"""Replay a recording as a module: one ``Out`` port per recorded stream.

The port set depends on the recording, so :func:`replay_module` builds the class at
import time of the blueprint that uses it (see ``dimos.memory.blueprints``). Workers
rebuild the same class on import, as long as they see the same ``REPLAY_DB`` /
``REPLAY_TOPICS`` environment; ``dimos replay`` sets both before ``dimos run``.
"""

from __future__ import annotations

import sys
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.memory.cli.dataset import open_dataset, stream_payload_types
from dimos.memory.store.base import Store
from dimos.memory.tap import check_topics, matching
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ReplayModuleConfig(ModuleConfig):
    dataset: str = ""
    speed: float = 1.0
    loop: bool = False
    seek: float | None = None
    duration: float | None = None


class ReplayModule(Module):
    """Publishes every ``Out`` port from the recording at recorded timing."""

    config: ReplayModuleConfig
    stream_types: dict[str, type] = {}  # set by replay_module()
    _store: Store | None = None

    @rpc
    def start(self) -> None:
        super().start()
        if not self.config.dataset:
            raise ValueError("no recording: run `dimos replay <memory.db>`")
        self._store = open_dataset(self.config.dataset)
        self._store.start()
        replay = self._store.replay(
            speed=self.config.speed,
            loop=self.config.loop,
            seek=self.config.seek,
            duration=self.config.duration,
        )
        replay.pin_anchor()
        for name, port in self.outputs.items():
            logger.info("Replaying %s -> %s", name, port)
            self.register_disposable(replay.stream(name).observable().subscribe(port.publish))

    @rpc
    def stop(self) -> None:
        super().stop()
        if self._store is not None:
            self._store.stop()
            self._store = None


def replay_module(dataset: str, topics: str = "*", name: str = "Replay") -> type[ReplayModule]:
    """Build a :class:`ReplayModule` subclass with an ``Out`` per stream in *dataset*.

    Assign the result to *name* at module level: deploying to a worker pickles the
    class by that path. An empty *dataset* yields a port-less class.
    """
    ports: dict[str, Any] = {}
    if dataset:
        store = open_dataset(dataset)
        store.start()
        try:
            types = stream_payload_types(store)
        finally:
            store.stop()
        check_topics(topics, types)
        ports = {n: Out[types[n]] for n in sorted(matching(topics, types))}  # type: ignore[valid-type]
    caller = sys._getframe(1).f_globals.get("__name__", __name__)
    namespace = {
        "__annotations__": ports,
        "__module__": caller,
        "stream_types": {n: types[n] for n in ports} if dataset else {},
    }
    return type(name, (ReplayModule,), namespace)


def rerun_layout(stream_types: dict[str, type]) -> Any:
    """A Rerun blueprint showing every recorded stream: one 3D world view, one 2D view per image."""
    import rerun.blueprint as rrb

    from dimos.msgs.sensor_msgs.Image import Image

    images = [
        rrb.Spatial2DView(origin=f"world/{n}", name=n)
        for n, t in stream_types.items()
        if issubclass(t, Image)
    ]
    world = rrb.Spatial3DView(origin="world", name="3D")
    if not images:
        return rrb.Blueprint(world)
    return rrb.Blueprint(rrb.Horizontal(rrb.Vertical(*images), world, column_shares=[1, 2]))
