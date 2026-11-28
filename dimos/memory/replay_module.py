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
from threading import Thread
from typing import TYPE_CHECKING, Any

from pydantic import Field
from reactivex import interval

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import Out
from dimos.memory.cli.dataset import open_dataset, open_store, resolve_dataset, stream_payload_types
from dimos.memory.replay import Replay
from dimos.memory.replay_progress import ReplayProgress
from dimos.memory.store.base import Store
from dimos.memory.tap import check_topics, matching
from dimos.utils.generic import classproperty
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.memory.replay import ReplayStream
    from dimos.msgs.protocol import DimosMsg

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
    _replay: Replay | None = None
    _static: list[tuple[Out[Any], Any]] = []

    @classproperty
    def blueprint(cls) -> Any:  # noqa: N805
        from dimos.core.coordination.blueprints import Blueprint

        return partial(Blueprint.create, cls, topics=cls.topics)  # type: ignore[arg-type]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
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
        config = self.config
        replay = store.replay(
            speed=config.speed,
            loop=config.loop,
            loop_gap=config.loop_gap,
            seek=config.seek,
            duration=config.duration,
            buffer_size=config.buffer_size,
            lag_warning=config.lag_warning,
        )
        available = set(replay.list_streams())
        missing = sorted(set(self.outputs) - available)
        if missing:
            logger.warning("Recording has no stream for these ports", path=str(path), ports=missing)
        streams: dict[str, ReplayStream[DimosMsg]] = {
            name: replay.stream(name) for name in self.outputs if name in available
        }
        # A stream whose rows all carry one timestamp (camera_info recorded from a
        # constant stamped at import, often long before the run) is republished at
        # 1 Hz instead of replayed, and does not start the timeline: otherwise the
        # replay would spend that lead time emitting nothing.
        static = {n for n, s in streams.items() if s.count() > 1 and s.first_ts() == s.last_ts()}
        timed = {n: s for n, s in streams.items() if n not in static}
        progress = ReplayProgress(str(path), on_complete=self._replay_finished)
        progress.start(replay, timed.values())
        self._replay = replay
        self._static = [(self.outputs[n], streams[n].first()) for n in static]
        for name, stream in timed.items():
            port = self.outputs[name]
            logger.info("Replaying %s -> %s", name, port)
            self.register_disposable(progress.track(replay.add(stream)).subscribe(port.publish))
        progress.seal()
        replay.load()

    def _replay_finished(self) -> None:
        if self.config.exit_on_end:
            logger.info("Requesting coordinator shutdown after replay", source=self.config.dataset)
            # Keep the RPC round trip off the replay thread.
            Thread(target=self._request_shutdown, daemon=True).start()

    def _request_shutdown(self) -> None:
        from dimos.core.coordination.coordinator_rpc import CoordinatorRPC

        try:
            client = CoordinatorRPC.connect(timeout=10.0)
            try:
                client.call("shutdown", requesting_module=self.config.rpc_name)
            finally:
                client.stop()
        except Exception:
            logger.error("Failed to request coordinator shutdown", exc_info=True)

    @rpc
    def on_system_modules_ready(self, _modules: list[RPCClient]) -> None:
        # Every module has started, so every subscriber is listening for the first frame.
        for port, msg in self._static:
            self.register_disposable(interval(1.0).subscribe(partial(_republish, port, msg)))
        self._static = []
        if self._replay is not None:
            self._replay.play()

    @rpc
    def stop(self) -> None:
        if self._replay is not None:
            self._replay.stop()
        super().stop()
        if self._store is not None:
            self._store.stop()
            self._store = None


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
