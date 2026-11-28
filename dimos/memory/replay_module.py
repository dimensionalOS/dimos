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

from collections import deque
from functools import partial
import heapq
from math import inf
from pathlib import Path
import sys
from threading import Condition, Event, Thread, current_thread
import time
from typing import TYPE_CHECKING, Any

from pydantic import Field
from reactivex import interval

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import Out
from dimos.memory.cli.dataset import open_dataset, open_store, resolve_dataset, stream_payload_types
from dimos.memory.store.base import Store
from dimos.memory.tap import check_topics, matching
from dimos.memory.utils.time_window import RecordingWindow
from dimos.utils.generic import classproperty
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Generator

    from dimos.memory.stream import Stream

logger = setup_logger()


class ReplayModuleConfig(ModuleConfig):
    # A recording path, a dataset name (data/, LFS), or a directory: its newest recording.
    dataset: str = ""
    topics: str = "*"
    speed: float = 1.0
    loop: bool = False
    # Shut the run down once every replayed stream has ended.
    exit_on_end: bool = False
    # With ``loop``, recording seconds between the last frame of a pass and the first of the next.
    loop_gap: float = Field(default=0.0, ge=0.0)
    seek: float | None = None
    duration: float | None = None
    # Decoded frames read ahead of the publisher; start() returns once this many are ready.
    buffer_size: int = Field(default=128, gt=0)
    # Seconds the publisher may fall behind schedule before it warns.
    lag_warning: float = 0.5


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
    """Publishes each ``Out`` port from the recorded stream of the same name, in recorded order.

    Subclass it and declare the ports to replay; a class that declares none gets one
    per stream in its ``dataset``. ``start()`` fills the playback buffer, the clock
    starts in ``on_system_modules_ready`` once every subscriber is listening, and
    ``stop()`` stops reading before the recording closes.
    """

    config: ReplayModuleConfig
    stream_types: dict[str, type] = {}  # set by replay_module()
    topics: str = "*"  # set by replay_module(); carried to workers via the blueprint kwargs
    _store: Store | None = None
    _static: list[tuple[Out[Any], Any]] = []

    @classproperty
    def blueprint(cls) -> Any:  # noqa: N805
        from dimos.core.coordination.blueprints import Blueprint

        return partial(Blueprint.create, cls, topics=cls.topics)  # type: ignore[arg-type]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._rows: dict[str, Stream[Any]] = {}
        self._stopped = Event()
        self._buffer = _Buffer(self.config.buffer_size, self._stopped)
        self._reader: Thread | None = None
        self._read_done = Event()
        self._publisher: Thread | None = None
        self._error: Exception | None = None
        self.replay_done = Event()  # set once the last frame is published, or the replay failed
        if self.config.dataset and not self.outputs:
            try:
                self._add_ports()
            except Exception:
                self.stop()  # Module.__init__ already spun up its threads
                raise

    def _add_ports(self) -> None:
        types = stream_types_of(self.config.dataset)
        for name in _port_names(self.config.topics, types):
            setattr(self, name, Out(types[name], name, self))

    @rpc
    def start(self) -> None:
        super().start()
        if not self.config.dataset:
            raise ValueError("no recording: pass --replay-db <memory.db>")
        path = recording_path(self.config.dataset)
        store: Store = open_store(path)
        store.start()
        self._store = store
        window = RecordingWindow.from_store(store, self.config.seek or 0.0, self.config.duration)
        available = set(store.list_streams())
        missing = sorted(set(self.outputs) - available)
        if missing:
            logger.warning("Recording has no stream for these ports", path=str(path), ports=missing)
        rows: dict[str, Stream[Any]] = {
            n: window.apply(store.stream(n)).order_by("ts") for n in self.outputs if n in available
        }
        bounds = {n: (r.first().ts, r.last().ts) for n, r in rows.items() if r.count()}
        # A stream whose rows all carry one timestamp (camera_info recorded from a
        # constant stamped at import, often long before the run) is republished at
        # 1 Hz instead of replayed, and does not start the timeline: otherwise the
        # replay would spend that lead time emitting nothing.
        static = {n for n, (first, last) in bounds.items() if first == last and rows[n].count() > 1}
        self._static = [(self.outputs[n], rows[n].first().data) for n in static]
        self._rows = {n: r for n, r in rows.items() if n not in static}
        timed = [bounds[n] for n in self._rows if n in bounds]
        logger.info(
            "Replay started",
            source=self.config.dataset,
            path=str(path),
            duration_s=max(b for _, b in timed) - min(a for a, _ in timed) if timed else 0.0,
            speed=self.config.speed,
            loop=self.config.loop,
        )
        for name in self._rows:
            logger.debug("Replaying %s -> %s", name, self.outputs[name])
        # Pay the first reads and decodes here, before the clock starts.
        self._reader = Thread(target=self._read, name="replay-reader", daemon=True)
        self._reader.start()
        self._buffer.wait_full(self._read_done)

    @rpc
    def on_system_modules_ready(self, _modules: list[RPCClient]) -> None:
        # Every module has started, so every subscriber is listening for the first frame.
        for port, msg in self._static:
            self.register_disposable(interval(1.0).subscribe(partial(_republish, port, msg)))
        self._static = []
        if self._reader is not None and self._publisher is None and not self._stopped.is_set():
            self._publisher = Thread(
                target=self._publish, args=(time.monotonic(),), name="replay-publisher", daemon=True
            )
            self._publisher.start()

    @rpc
    def stop(self) -> None:
        # Stop reading and publishing before the recording closes.
        self._stopped.set()
        self._buffer.wake()
        for thread in (self._reader, self._publisher):
            if thread is not None and thread is not current_thread():
                thread.join()
        super().stop()
        if self._store is not None:
            self._store.stop()
            self._store = None

    def _read(self) -> None:
        try:
            if self._read_passes():
                self._buffer.put((inf, "", _END))
        finally:
            self._read_done.set()
            self._buffer.wake()

    def _read_passes(self) -> bool:
        """Buffer every pass of the recording. False once stopped or failed."""
        offset = 0.0
        while True:
            rows = [self._rows_of(name) for name in self._rows]
            first = last = None
            try:
                for ts, name, obs in heapq.merge(*rows, key=lambda row: row[0]):
                    if obs is _FAILED:
                        self._buffer.put((ts, name, _FAILED))
                        return False
                    first = ts if first is None else first
                    last = ts
                    try:
                        value = obs.data
                    except Exception as error:
                        self._error = error
                        self._buffer.put((ts, name, _FAILED))
                        return False
                    if not self._buffer.put((ts + offset, name, value)):
                        return False
            finally:
                for stream_rows in rows:
                    stream_rows.close()
            if not self.config.loop or first is None or last is None:
                return True
            offset += last - first + self.config.loop_gap

    def _rows_of(self, name: str) -> Generator[tuple[float, str, Any], None, None]:
        """*name*'s rows in ts order; a read error ends it at once."""
        try:
            rows = iter(self._rows[name])
            try:
                for obs in rows:
                    yield float(obs.ts), name, obs
            finally:
                close = getattr(rows, "close", None)
                if close is not None:
                    close()
        except Exception as error:
            self._error = error
            yield -inf, name, _FAILED

    def _publish(self, wall_t0: float) -> None:
        first_ts: float | None = None
        behind = False
        while (item := self._buffer.get()) is not None:
            ts, name, value = item
            if value is _END:
                logger.info("Replay finished", source=self.config.dataset)
                self.replay_done.set()
                if self.config.exit_on_end:
                    # Keep the RPC round trip off the publisher thread.
                    Thread(target=self._request_shutdown, daemon=True).start()
                return
            if value is _FAILED:
                logger.error(
                    "Replay stopped: recording could not be read", stream=name, exc_info=self._error
                )
                self.replay_done.set()
                return
            if first_ts is None:
                first_ts = ts
            early = wall_t0 + (ts - first_ts) / self.config.speed - time.monotonic()
            if early > 0:
                behind = False  # back on schedule
                if self._stopped.wait(early):
                    return
            elif -early > self.config.lag_warning and not behind:
                behind = True
                logger.warning("Replay is behind schedule", lag_s=round(-early, 3), stream=name)
            try:
                self.outputs[name].publish(value)
            except Exception:
                # A subscriber's bug is logged, not allowed to stop the replay for the others.
                logger.exception("Replay subscriber failed", stream=name)

    def _request_shutdown(self) -> None:
        logger.info("Requesting coordinator shutdown after replay", source=self.config.dataset)
        from dimos.core.coordination.coordinator_rpc import CoordinatorRPC

        try:
            client = CoordinatorRPC.connect(timeout=10.0)
            try:
                client.call("shutdown", requesting_module=self.config.rpc_name)
            finally:
                client.stop()
        except Exception:
            logger.error("Failed to request coordinator shutdown", exc_info=True)


# A buffered item is (ts, port, value): ts is recording seconds plus the loop passes before
# it, and value is a decoded frame or one of these.
_END = object()  # the recording has no more frames
_FAILED = object()  # a stream could not be read or decoded; ReplayModule._error says why


class _Buffer:
    """Bounded FIFO between the reader and the publisher that a stop wakes up."""

    def __init__(self, size: int, stopped: Event) -> None:
        self._size = size
        self._items: deque[tuple[float, str, Any]] = deque()
        self._stopped = stopped
        self._changed = Condition()

    def put(self, item: tuple[float, str, Any]) -> bool:
        """False once stopped, without adding *item*."""
        with self._changed:
            self._changed.wait_for(lambda: len(self._items) < self._size or self._stopped.is_set())
            if self._stopped.is_set():
                return False
            self._items.append(item)
            self._changed.notify_all()
            return True

    def get(self) -> tuple[float, str, Any] | None:
        """None once stopped."""
        with self._changed:
            self._changed.wait_for(lambda: self._items or self._stopped.is_set())
            if self._stopped.is_set():
                return None
            item = self._items.popleft()
            self._changed.notify_all()
            return item

    def wait_full(self, done: Event) -> None:
        """Until full, *done* is set, or stopped."""
        with self._changed:
            self._changed.wait_for(
                lambda: len(self._items) >= self._size or done.is_set() or self._stopped.is_set()
            )

    def wake(self) -> None:
        with self._changed:
            self._changed.notify_all()


def recording_path(dataset: str) -> Path:
    """*dataset* resolved to one recording; a directory resolves to its newest."""
    path = resolve_dataset(Path(dataset).expanduser())
    if not path.is_dir():
        return path
    recordings = [p for p in path.iterdir() if p.suffix in (".db", ".mcap")]
    if not recordings:
        raise FileNotFoundError(f"No .db or .mcap recordings in {path}")
    return max(recordings, key=lambda p: p.stat().st_mtime)


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
