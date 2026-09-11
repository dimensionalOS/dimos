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

The port set depends on the recording. :func:`replay_module` builds a class with the
ports as annotations so ``autoconnect`` can wire them; the instance also creates any
missing ``Out`` from its ``dataset`` config, so a worker that imported a port-less
class still ends up with the same ports.
"""

from __future__ import annotations

from functools import partial
from pathlib import Path
import re
import sys
from typing import TYPE_CHECKING, Any

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.memory.cli.dataset import open_dataset, stream_payload_types
from dimos.memory.store.base import Store
from dimos.memory.tap import check_topics, matching
from dimos.utils.generic import classproperty
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from reactivex import Observable

    from dimos.memory.replay import Replay, ReplayStream
    from dimos.msgs.protocol import DimosMsg

logger = setup_logger()


class ReplayModuleConfig(ModuleConfig):
    dataset: str = ""
    topics: str = "*"
    speed: float = 1.0
    loop: bool = False
    seek: float | None = None
    duration: float | None = None


def stream_types_of(dataset: str) -> dict[str, type]:
    """Stream name -> payload type for every stream in *dataset*."""
    store = open_dataset(dataset)
    store.start()
    try:
        return stream_payload_types(store)
    finally:
        store.stop()


class ReplayModule(Module):
    """Publishes every ``Out`` port from the recording at recorded timing."""

    config: ReplayModuleConfig
    stream_types: dict[str, type] = {}  # set by replay_module()
    topics: str = "*"  # set by replay_module(); carried to workers via the blueprint kwargs
    _store: Store | None = None

    @classproperty
    def blueprint(cls) -> Any:  # noqa: N805
        from dimos.core.coordination.blueprints import Blueprint

        return partial(Blueprint.create, cls, topics=cls.topics)  # type: ignore[arg-type]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        if self.config.dataset:
            try:
                self._add_ports()
            except Exception:
                self.stop()  # Module.__init__ already spun up its threads
                raise

    def _add_ports(self) -> None:
        types = stream_types_of(self.config.dataset)
        existing = self.outputs
        for name in _port_names(self.config.topics, types):
            if name in existing:
                continue
            _reject_clash(name, self)
            setattr(self, name, Out(types[name], name, self))

    @rpc
    def start(self) -> None:
        super().start()
        if not self.config.dataset:
            raise ValueError("no recording: pass --replay-db <memory.db>")
        store: Store = open_dataset(self.config.dataset)
        store.start()
        self._store = store
        replay: Replay = store.replay(
            speed=self.config.speed,
            loop=self.config.loop,
            seek=self.config.seek,
            duration=self.config.duration,
        )
        # Open every stream before pinning the anchor, so setup time is not
        # counted as lateness that observable() would skip past.
        streams: dict[str, ReplayStream[DimosMsg]] = {
            name: replay.stream(name) for name in self.outputs
        }
        replay.pin_anchor()
        port: Out[DimosMsg]
        for name, port in self.outputs.items():
            timed: Observable[DimosMsg] = streams[name].observable()
            logger.info("Replaying %s -> %s", name, port)
            self.register_disposable(timed.subscribe(port.publish))

    @rpc
    def stop(self) -> None:
        super().stop()
        if self._store is not None:
            self._store.stop()
            self._store = None


def _port_names(topics: str, types: dict[str, type]) -> list[str]:
    check_topics(topics, types)
    return sorted(matching(topics, types))


def _reject_clash(name: str, owner: Any) -> None:
    if hasattr(owner, name):
        raise ValueError(f"recorded stream {name!r} clashes with a ReplayModule attribute")


def replay_module(dataset: str, topics: str = "*", name: str = "Replay") -> type[ReplayModule]:
    """Build a :class:`ReplayModule` subclass with an ``Out`` per stream in *dataset*.

    Assign the result to *name* at module level: deploying to a worker pickles the
    class by that path. An empty *dataset* yields a port-less class.
    """
    ports: dict[str, Any] = {}
    if dataset:
        types = stream_types_of(dataset)
        for n in _port_names(topics, types):
            _reject_clash(n, ReplayModule)
            ports[n] = Out[types[n]]  # type: ignore[valid-type]
    caller = sys._getframe(1).f_globals.get("__name__", __name__)
    namespace = {
        "__annotations__": ports,
        "__module__": caller,
        "stream_types": {n: types[n] for n in ports} if dataset else {},
        "topics": topics,
    }
    return type(name, (ReplayModule,), namespace)


_VIS_KEYS = ("static", "visual_override", "max_hz", "tf_axes")


def recorded_rerun_config(dataset: str) -> dict[str, Any]:
    """Viewer config of the blueprint that made *dataset*: robot body, converters, rate caps.

    Run dirs are ``<stamp>-<blueprint>`` (``generate_run_id``). Anything else, or a
    blueprint without a Rerun bridge, yields ``{}``.
    """
    m = re.fullmatch(r"\d{8}-\d{6}-(.+)", Path(dataset).parent.name)
    if not m:
        return {}
    from dimos.robot.all_blueprints import all_blueprints
    from dimos.robot.get_all_blueprints import get_blueprint_by_name
    from dimos.visualization.rerun.bridge import RerunBridgeModule

    if m.group(1) not in all_blueprints:
        return {}
    for atom in get_blueprint_by_name(m.group(1)).blueprints:
        if atom.module is RerunBridgeModule:
            return {k: atom.kwargs[k] for k in _VIS_KEYS if k in atom.kwargs}
    return {}


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
