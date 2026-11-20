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

from reactivex import interval

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
    from dimos.memory.type.observation import Observation
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
            seek=self.config.seek,
            duration=self.config.duration,
        )
        # Open every stream before pinning the anchor, so setup time does not
        # put the playback behind the wall clock from the start.
        streams: dict[str, ReplayStream[DimosMsg]] = {
            name: replay.stream(name) for name in self.outputs
        }
        # A stream whose rows all carry one timestamp (camera_info recorded from a
        # constant stamped at import, often long before the run) is republished at
        # 1 Hz instead of replayed, and does not set the anchor: otherwise the replay
        # would spend that lead time emitting nothing.
        static: dict[str, DimosMsg] = {}
        timed_streams: dict[str, Observable[DimosMsg]] = {}
        timed_starts = []
        for name, stream in streams.items():
            if issubclass(self.outputs[name].type, TFMessage):
                # Frozen frames describe the coordinate system even when their
                # original source stamp is outside the selected time window.
                rows = store.stream(name, TFMessage)
                frozen, edges = _frozen_tf(rows)
                if frozen.transforms:
                    static[name] = frozen
                    dynamic = rows.filter(partial(_has_dynamic_tf, edges))
                    try:
                        start = dynamic.order_by("ts").first().ts
                    except LookupError:
                        continue
                    stream = streams[name] = replay.stream(
                        name,
                        autocast=partial(_dynamic_tf, edges),
                        source=dynamic.time_range(start, float("inf")),
                    )
            elif stream.count() > 1 and stream.first_ts() == stream.last_ts():
                message = stream.first()
                if message is not None:
                    static[name] = message
                continue
            first_ts = stream.first_ts()
            if first_ts is not None:
                timed_starts.append(first_ts)
                timed_streams[name] = stream.observable(skip_late=False)
        for name, message in static.items():
            if isinstance(message, TFMessage):
                self.outputs[name].publish(message)
        progress = ReplayProgress(str(path))
        progress.start(replay, (streams[n] for n in timed_streams))
        replay.pin_anchor(min(timed_starts) if timed_starts else None)
        port: Out[DimosMsg]
        for name, port in self.outputs.items():
            logger.info("Replaying %s -> %s", name, port)
            if name in static:
                self.register_disposable(
                    interval(1.0).subscribe(partial(_republish, port, static[name]))
                )
            if name in timed_streams:
                self.register_disposable(
                    progress.track(timed_streams[name]).subscribe(port.publish)
                )
        progress.seal()

    @rpc
    def stop(self) -> None:
        super().stop()
        if self._store is not None:
            self._store.stop()
            self._store = None


def _has_dynamic_tf(edges: set[tuple[str, str]], row: Observation[TFMessage]) -> bool:
    return not row.data.transforms or any(
        (t.frame_id, t.child_frame_id) not in edges for t in row.data.transforms
    )


def _dynamic_tf(edges: set[tuple[str, str]], message: TFMessage) -> TFMessage:
    # Older recordings can carry both fixed and changing edges in one row.
    return TFMessage(
        *(t for t in message.transforms if (t.frame_id, t.child_frame_id) not in edges)
    )


def _frozen_tf(rows: Stream[TFMessage]) -> tuple[TFMessage, set[tuple[str, str]]]:
    """Find fixed edges in a finite recording, independent of playback bounds."""
    all_static = rows.count() > 1 and rows.order_by("ts").first().ts == rows.last().ts
    # A changing stamp disqualifies the edge. Only the last stamp and count
    # are kept for each edge, irrespective of the length of the recording.
    stamps: dict[tuple[str, str], tuple[float, int] | None] = {}
    for row in rows:
        for transform in row.data.transforms:
            edge = (transform.frame_id, transform.child_frame_id)
            previous = stamps.get(edge, (transform.ts, 0))
            if previous is not None:
                stamps[edge] = (
                    (transform.ts, previous[1] + 1) if previous[0] == transform.ts else None
                )
    edges = {
        edge for edge, stamp in stamps.items() if stamp is not None and (all_static or stamp[1] > 1)
    }
    transforms: dict[bytes, Transform] = {}
    if edges:
        for row in rows:
            for transform in row.data.transforms:
                if (transform.frame_id, transform.child_frame_id) in edges:
                    transforms[transform.lcm_encode()] = transform
    return TFMessage(*transforms.values()), edges


def _republish(port: Out[Any], msg: Any, _tick: int) -> None:
    port.publish(msg)


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
