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
import sys
from typing import TYPE_CHECKING, Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.memory.cli.dataset import open_dataset, open_store, resolve_dataset, stream_payload_types
from dimos.memory.replay_progress import ReplayProgress
from dimos.memory.store.base import Store
from dimos.memory.tap import check_topics, matching
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.generic import classproperty
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from reactivex import Observable

    from dimos.memory.replay import Replay, ReplayStream
    from dimos.memory.stream import Stream
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.protocol import DimosMsg

logger = setup_logger()


class ReplayModuleConfig(ModuleConfig):
    dataset: str = ""
    topics: str = "*"
    speed: float = 1.0
    loop: bool = False
    seek: float | None = None
    duration: float | None = None


def dataset_path(name: str, *, explicit: bool = True) -> str:
    """``--replay-db`` as a file path: a path, or a bare dataset name resolved like
    ``--replay`` does (cwd, data/, then LFS via ``get_data``).

    A bare name is only resolved when the user passed it (*explicit*), so importing the
    blueprint with the config default (registry, tests) never opens a database or pulls
    from LFS.
    """
    if not explicit and not Path(name).is_file():
        return ""
    try:
        return str(resolve_dataset(name))
    except (OSError, ValueError):
        return ""


def _wire_type(t: type) -> type:
    """The class that owns *t*'s ``msg_name``: the type a port must declare so a consumer
    of the base message shares its topic (unitree ``Odometry`` -> ``PoseStamped``)."""
    for cls in t.__mro__:
        if "msg_name" in vars(cls):
            return cls
    return t


def stream_types_of(dataset: str) -> dict[str, type]:
    """Stream name -> port type for every stream in *dataset*."""
    store = open_dataset(dataset)
    store.start()
    try:
        return {n: _wire_type(t) for n, t in stream_payload_types(store).items()}
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
            if name not in existing:
                setattr(self, name, Out(types[name], name, self))

    @rpc
    def start(self) -> None:
        super().start()
        if not self.config.dataset:
            raise ValueError("no recording: pass --replay-db <memory.db>")
        path = resolve_dataset(self.config.dataset)
        store: Store = open_store(path)
        store.start()
        self._store = store
        replay: Replay = store.replay(
            speed=self.config.speed,
            loop=self.config.loop,
        )
        # A ``*_static`` TF stream is static: its latest transform per edge is sent once
        # when playback starts, on the matching ``*_static`` port, else on ``tf``.
        static_tf: dict[str, dict[tuple[str, str], Transform]] = {}
        timed_streams: dict[str, ReplayStream[DimosMsg]] = {}
        timed_starts = []
        outputs = self.outputs
        has_window = self.config.seek is not None or self.config.duration is not None
        # Relative offsets use the whole recording, even when only one topic is published.
        types = (
            stream_payload_types(store) if has_window else {n: p.type for n, p in outputs.items()}
        )
        for name, payload_type in types.items():
            if issubclass(payload_type, TFMessage) and name.endswith("_static"):
                port_name = name if _is_tf_port(outputs, name) else "tf"
                if _is_tf_port(outputs, port_name):
                    port_edges = static_tf.setdefault(port_name, {})
                    for transform in _latest_transforms(store.stream(name, TFMessage)).transforms:
                        port_edges[(transform.frame_id, transform.child_frame_id)] = transform
                continue
            stream: ReplayStream[DimosMsg] = replay.stream(name)
            first_ts = stream.first_ts()
            if first_ts is not None:
                timed_starts.append(first_ts)
                if name in outputs:
                    timed_streams[name] = stream

        if timed_starts and has_window:
            replay.set_relative_window(
                min(timed_starts), seek=self.config.seek, duration=self.config.duration
            )
            # Pin the wall clock to the first frame inside the requested window.
            timed_starts = [
                ts for stream in timed_streams.values() if (ts := stream.first_ts()) is not None
            ]

        # Consumers keep the last value, so static tf is sent once, before any timed frame.
        for port_name, transforms in static_tf.items():
            outputs[port_name].publish(TFMessage(*transforms.values()))
        progress = ReplayProgress(str(path))
        progress.start(replay, timed_streams.values())
        replay.pin_anchor(min(timed_starts) if timed_starts else None)
        for name, stream in timed_streams.items():
            port = outputs[name]
            logger.info("Replaying %s -> %s", name, port)
            # Not a late subscriber, just behind by the setup of the streams
            # subscribed before it, which on a loaded machine exceeds the
            # late tolerance and would drop its first frames.
            timed: Observable[DimosMsg] = stream.observable(skip_late=False)
            self.register_disposable(progress.track(timed).subscribe(port.publish))
        progress.seal()

    @rpc
    def stop(self) -> None:
        super().stop()
        if self._store is not None:
            self._store.stop()
            self._store = None


def _is_tf_port(outputs: dict[str, Out[Any]], name: str) -> bool:
    return name in outputs and issubclass(outputs[name].type, TFMessage)


def _latest_transforms(rows: Stream[TFMessage]) -> TFMessage:
    """The last recorded transform of every edge."""
    latest: dict[tuple[str, str], Transform] = {}
    for row in rows.order_by("ts"):
        for transform in row.data.transforms:
            latest[(transform.frame_id, transform.child_frame_id)] = transform
    return TFMessage(*latest.values())


def _port_names(topics: str, types: dict[str, type]) -> list[str]:
    check_topics(topics, types)
    names = []
    for n in sorted(matching(topics, types)):
        if hasattr(ReplayModule, n):
            logger.warning(
                "Skipping recorded stream %r: it clashes with a ReplayModule attribute", n
            )
            continue
        names.append(n)
    return names


def replay_module(dataset: str, topics: str = "*", name: str = "Replay") -> type[ReplayModule]:
    """Build a :class:`ReplayModule` subclass with an ``Out`` per stream in *dataset*.

    Assign the result to *name* at module level: deploying to a worker sends the
    class by that import path. An empty *dataset* yields a port-less class.
    """
    ports: dict[str, Any] = {}
    if dataset:
        types = stream_types_of(dataset)
        for n in _port_names(topics, types):
            ports[n] = Out[types[n]]  # type: ignore[valid-type]
    caller = sys._getframe(1).f_globals.get("__name__", __name__)
    namespace = {
        "__annotations__": ports,
        "__module__": caller,
        "stream_types": {n: types[n] for n in ports} if dataset else {},
        "topics": topics,
    }
    return type(name, (ReplayModule,), namespace)


_VIS_KEYS = ("blueprint", "static", "visual_override", "max_hz", "tf_axes")


def recorded_rerun_config(dataset: str) -> dict[str, Any]:
    """Viewer config of the blueprint that made *dataset*: layout, robot body, converters, rate caps.

    Run dirs are named after their run ID (``generate_run_id``). Anything else, or
    a blueprint without a Rerun bridge, yields ``{}``.
    """
    from dimos.core.run_registry import blueprint_from_run_id

    name = blueprint_from_run_id(Path(dataset).parent.name)
    if name is None:
        return {}
    from dimos.robot.all_blueprints import all_blueprints
    from dimos.robot.get_all_blueprints import get_blueprint_by_name
    from dimos.visualization.rerun.bridge import RerunBridgeModule

    if name not in all_blueprints:
        return {}
    for atom in autoconnect(get_blueprint_by_name(name)).solidify().blueprints:
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
    plots = [
        rrb.TimeSeriesView(origin="plots/odom", name="odom"),
        rrb.TimeSeriesView(origin="plots/cmd_vel", name="cmd_vel"),
    ]
    world = rrb.Spatial3DView(origin="world", name="3D")
    if not images:
        return rrb.Blueprint(world)
    return rrb.Blueprint(rrb.Horizontal(rrb.Vertical(*images, *plots), world, column_shares=[1, 2]))
