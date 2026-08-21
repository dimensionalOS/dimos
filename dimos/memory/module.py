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

from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
import enum
import inspect
import os
from pathlib import Path
import sqlite3
import threading
import time
from typing import TYPE_CHECKING, Any, Generic, TypeVar, cast

from pydantic import Field, field_validator
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.memory.buffer import Unbounded
from dimos.memory.embed import EmbedImages
from dimos.memory.store.null import NullStore
from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.stream import Stream
from dimos.memory.transform import QualityWindow
from dimos.memory.type.observation import EmbeddedObservation, Observation
from dimos.models.embedding.base import EmbeddingModel
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.data import backup_file
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from reactivex.abc import DisposableBase

    from dimos.core.stream import Out
    from dimos.msgs.geometry_msgs.Pose import Pose

logger = setup_logger()

T = TypeVar("T")
TIn = TypeVar("TIn")
TOut = TypeVar("TOut")


def stream_to_port(stream: Stream[T], out: Out[T]) -> DisposableBase:
    """Forward each observation's ``data`` from *stream* to a Module ``Out`` port.

    Iteration runs on the dimos thread pool via :meth:`Stream.observable`.
    """

    def _on_error(e: Exception) -> None:
        logger.error("stream_to_port() pipeline error: %s", e, exc_info=True)

    return stream.observable().subscribe(
        on_next=lambda obs: out.publish(obs.data),
        on_error=_on_error,
    )


class StreamModule(Module, Generic[TIn, TOut]):
    """Module base class that wires a memory stream pipeline
    and deploys it as a dimos module

    Parameterize with the In/Out data types so the pipeline is
    statically typed end-to-end::

        class VoxelGridMapper(StreamModule[PointCloud2, PointCloud2]):
            pipeline = Stream().transform(VoxelMapTransformer())
            lidar: In[PointCloud2]
            global_map: Out[PointCloud2]

    **Config-driven pipeline**

        class VoxelGridMapper(StreamModule[PointCloud2, PointCloud2]):
            config: VoxelGridMapperConfig
            def pipeline(self, stream: Stream[PointCloud2]) -> Stream[PointCloud2]:
                return stream.transform(VoxelMap(**self.config.model_dump()))

            lidar: In[PointCloud2]
            global_map: Out[PointCloud2]

    On start, the single ``In`` port feeds a MemoryStore, and the pipeline
    is applied to the live stream, publishing results to the single ``Out`` port.

    The MemoryStore acts as a bridge between the push-based Module In port
    and the pull-based memory stream pipeline — it also enables replay and
    persistence if the store is swapped for a persistent backend later.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

    @rpc
    def start(self) -> None:
        super().start()

        if len(self.inputs) != 1 or len(self.outputs) != 1:
            raise TypeError(
                f"{self.__class__.__name__} must have exactly one In and one Out port, "
                f"found {len(self.inputs)} In and {len(self.outputs)} Out"
            )

        ((in_name, in_port_raw),) = self.inputs.items()
        ((_, out_port_raw),) = self.outputs.items()
        in_port = cast("In[TIn]", in_port_raw)
        out_port = cast("Out[TOut]", out_port_raw)

        store = self.register_disposable(NullStore())
        store.start()

        stream: Stream[TIn] = store.stream(in_name, in_port.type)

        # we push input into the stream
        self.register_disposable(Disposable(in_port.subscribe(stream.append)))

        # and we push stream output to the output port
        self.register_disposable(stream_to_port(self._apply_pipeline(stream.live()), out_port))

    def _apply_pipeline(self, stream: Stream[TIn]) -> Stream[TOut]:
        """Apply the pipeline to a live stream.

        Handles both static (class attr) and dynamic (method) pipelines.
        """
        pipeline = getattr(self.__class__, "pipeline", None)
        if pipeline is None:
            raise TypeError(
                f"{self.__class__.__name__} must define a 'pipeline' attribute or method"
            )

        # Method pipeline: self.pipeline(stream) -> stream
        if inspect.isfunction(pipeline):
            result = pipeline(self, stream)
            if not isinstance(result, Stream):
                raise TypeError(
                    f"{self.__class__.__name__}.pipeline() must return a Stream, got {type(result).__name__}"
                )
            return result

        # Static class attr: Stream (unbound chain) or Transformer
        if isinstance(pipeline, Stream):
            return stream.chain(pipeline)
        return stream.transform(pipeline)

    @rpc
    def stop(self) -> None:
        super().stop()


class MemoryModuleConfig(ModuleConfig):
    db_path: str | Path = "recording.db"

    @field_validator("db_path", mode="before")
    @classmethod
    def _resolve_path(cls, v: str | Path) -> Path:
        p = Path(os.fspath(v))
        if not p.is_absolute():
            p = DIMOS_PROJECT_ROOT / p
        return p


class MemoryModule(Module):
    """Base class for memory-related modules, like recorders and search systems.
    Provides a config with a db_path for the module's MemoryStore, and common start/stop logic.

    If changing the backend globally in dimos, this class will be replaced
    """

    config: MemoryModuleConfig
    _store: SqliteStore | None = None

    @property
    def store(self) -> SqliteStore:
        if self._store is not None:
            return self._store

        self._store = self.register_disposable(
            SqliteStore(path=str(self.config.db_path)),
        )
        self._store.start()
        return self._store


class SemanticSearchConfig(MemoryModuleConfig):
    embedding_model: type[EmbeddingModel] | None = None


class SemanticSearch(MemoryModule):
    config: SemanticSearchConfig
    model: EmbeddingModel | None = None
    embeddings: Stream[Any] | None = None

    @rpc
    def start(self) -> None:
        super().start()

        embedding_cls = self.config.embedding_model
        if embedding_cls is None:
            from dimos.models.embedding.clip import CLIPModel

            embedding_cls = CLIPModel

        self.model = self.register_disposable(embedding_cls())
        self.model.start()

        self.embeddings = self.store.stream("color_image_embedded", Image)

        # fmt: off
        self.store.streams.color_image \
           .live() \
           .filter(lambda obs: obs.data.brightness > 0.1) \
           .transform(QualityWindow(lambda img: img.sharpness, window=0.5)) \
           .transform(EmbedImages(self.model, batch_size=2)) \
           .save(self.embeddings) \
           .drain_thread()
        # fmt: on

    @skill
    def search(self, query: str) -> PoseStamped:
        from dimos.memory.transform import peaks

        assert self.model is not None and self.embeddings is not None, (
            "SemanticSearch.search() called before start()"
        )

        query_vector = self.model.embed_text(query)

        # TODO(lesh): cluster results by peaks, then sort by time/distance
        # depending on the desired weighting.
        results = self.embeddings.search(query_vector)

        def _similarity(obs: Observation[Any]) -> float:
            return cast("EmbeddedObservation[Any]", obs).similarity or 0.0

        best = results.transform(peaks(key=_similarity, distance=1.0)).last()
        if best.pose_stamped is None:
            raise LookupError("No pose on best search result")
        return best.pose_stamped


class OnExisting(str, enum.Enum):
    OVERWRITE = "overwrite"
    ERROR = "error"
    BACKUP = "backup"
    APPEND = "append"


class RecorderConfig(MemoryModuleConfig):
    on_existing: OnExisting = OnExisting.BACKUP
    backup_keep_last: int = Field(default=10, ge=0)
    root_frame: str = "world"
    default_frame_id: str = "base_link"
    tf_tolerance: float = 0.5
    db_path: str | Path = "recording.db"
    # Also record the live tf stream (under "tf") alongside the In ports.
    record_tf: bool = True
    # Rename recorded streams: {port_name: db_stream_name}. Conceptually this is
    # what the wiring layer's .remappings() expresses, but there's no easy way to
    # read the active remappings from inside the module (AFAIK), so this config
    # arg does the per-stream rename directly.
    stream_remapping: dict[str, str] = Field(default_factory=dict)
    # ex: {"depth_image_1": "lz4+lcm"} for lossless depth recording
    stream_codecs: dict[str, str] = Field(default_factory=dict)
    # Port names that inherently have no pose to anchor (command streams, etc.).
    poseless_streams: list[str] = Field(default_factory=list)


PoseSetter = Callable[[Any], "Awaitable[Pose | None]"]

# A writer thread this far behind means the disk cannot take the data; say so
# once in a while rather than per message.
BACKLOG_WARN_DEPTH = 200
BACKLOG_WARN_INTERVAL = 5.0
MISSING_POSE_WARN_INTERVAL = 5.0
POSE_SETTER_TIMEOUT = 1.0
# Time to finish writing what is already queued once the ports are unsubscribed.
WRITER_FLUSH_TIMEOUT = 30.0
WRITE_STATS_INTERVAL = 10.0


def merge_stream_dbs(base: Path, stream_names: list[str] | None = None) -> None:
    """Combine per-stream db files (``<base>.<stream>.db``) into ``base``.

    Each shard holds only its own stream's tables (``"{s}"``, ``"{s}_blob"``,
    ``"{s}_rtree"`` + the ``_streams`` registry row), so the merge is a straight
    schema copy + INSERT..SELECT per shard. The shards are left on disk; delete
    them once the merged db checks out. With *stream_names* omitted, merges
    every ``<base>.<stream>.db`` found next to *base* — usable standalone after
    a hard kill that skipped the recorder's stop()."""
    if stream_names is None:
        prefix = f"{base.stem}."
        stream_names = sorted(
            shard.name[len(prefix) : -len(base.suffix)]
            for shard in base.parent.glob(f"{prefix}*{base.suffix}")
        )
    started = time.perf_counter()
    dest = sqlite3.connect(str(base))
    try:
        dest.execute("PRAGMA journal_mode=WAL")
        # The merge is re-runnable, so skip fsyncs and checkpoint once at the
        # end — halves the IO on a disk that is the bottleneck to begin with.
        dest.execute("PRAGMA synchronous=OFF")
        dest.execute("PRAGMA wal_autocheckpoint=0")
        dest.execute(
            "CREATE TABLE IF NOT EXISTS _streams (name TEXT PRIMARY KEY, config TEXT NOT NULL)"
        )
        for stream_name in stream_names:
            shard = base.with_name(f"{base.stem}.{stream_name}{base.suffix}")
            dest.execute("ATTACH DATABASE ? AS src", (str(shard),))
            try:
                schema = dest.execute(
                    "SELECT type, name, sql FROM src.sqlite_master"
                    " WHERE sql IS NOT NULL AND name NOT LIKE 'sqlite_%'"
                    " AND name != '_streams' AND name NOT GLOB '*_rtree_*'"
                ).fetchall()
                for _, _, sql in schema:
                    try:
                        dest.execute(sql)
                    except sqlite3.OperationalError as e:
                        if "already exists" not in str(e):
                            raise
                for kind, table, _ in schema:
                    if kind == "table":
                        dest.execute(f'INSERT INTO main."{table}" SELECT * FROM src."{table}"')
                dest.execute("INSERT OR REPLACE INTO main._streams SELECT * FROM src._streams")
                dest.commit()
            finally:
                dest.execute("DETACH DATABASE src")
    finally:
        dest.close()
    logger.info(
        "Merged %d per-stream dbs into %s in %.1fs",
        len(stream_names),
        base,
        time.perf_counter() - started,
    )


def pose_setter_for(*stream_names: str) -> Callable[[Any], Any]:
    """Mark an ``async def`` method ``(self, msg) -> Pose | None`` as the pose
    setter for the given recorded stream(s). Streams without a setter fall back
    to the tf-based ``world <- frame_id`` lookup."""

    def decorate(fn: Any) -> Any:
        if not inspect.iscoroutinefunction(fn):
            raise TypeError(
                f"@pose_setter_for must decorate an `async def` method; "
                f"{getattr(fn, '__qualname__', fn)} is not async"
            )
        fn._pose_setter_for = tuple(stream_names)
        return fn

    return decorate


class Recorder(MemoryModule):
    """Records all ``In`` ports to a memory SQLite database, plus the live tf tree.

    Subclass with the topics you want to record::

        class MyRecorder(Recorder):
            color_image: In[Image]
            lidar: In[PointCloud2]

        blueprint.add(MyRecorder, db_path="session.db")

    Each stream's pose defaults to a ``world <- frame_id`` tf lookup; decorate a
    method with ``@pose_setter_for("stream")`` to source it elsewhere (e.g. from
    an odometry stream). Setters run on the module's event loop and may be
    ``async def``::

        @pose_setter_for("lidar")
        async def _lidar_pose(self, msg):
            return self._last_odom_pose
    """

    config: RecorderConfig

    tf: In[TFMessage]

    _pose_setters: dict[str, Any] = {}
    # One store (own db file), queue, and writer thread per stream: the WAL
    # write lock is per file, so per-stream files remove all contention. The
    # per-stream dbs are merged into config.db_path afterwards.
    _stream_stores: dict[str, SqliteStore] = {}
    _writers: dict[str, tuple[Unbounded[Callable[[], None]], threading.Thread]] = {}
    _write_subscriptions: list[Any] = []
    _missing_pose: dict[str, tuple[int, float]] = {}
    _backlog_warned: dict[str, float] = {}

    @rpc
    def start(self) -> None:
        super().start()

        self._stream_stores = {}
        self._writers = {}
        self._write_subscriptions = []
        self._missing_pose = {}
        self._backlog_warned = {}

        if self.config.g.replay:
            logger.info(
                "Replay mode active — Recorder disabled, leaving %s untouched", self.config.db_path
            )
            return

        self._pose_setters = self._collect_pose_setters()

        # TODO: store reset API/logic is not implemented yet. This module
        # shouldn't need to know about files (SqliteStore specific), and
        # .live() subs need to know how to re-sub in case of a restart of
        # this module in a deployed blueprint.
        db_path = Path(self.config.db_path)
        if db_path.exists():
            if self.config.on_existing is OnExisting.APPEND:
                pass  # keep the db; the merge in stop() adds this run's streams
            elif self.config.on_existing is OnExisting.OVERWRITE:
                db_path.unlink()
                logger.info("Deleted existing recording %s", db_path)
            elif self.config.on_existing is OnExisting.BACKUP:
                backup = backup_file(db_path, keep_last=self.config.backup_keep_last)
                if backup is None:
                    logger.info("Removed existing recording %s (backup_keep_last=0)", db_path)
                else:
                    logger.info("Backed up existing recording %s -> %s", db_path, backup)
            else:
                raise FileExistsError(f"Recording already exists: {db_path}")

        if getattr(self.tf, "_transport", None) is not None:
            self._tf = self.tfbuffer

        if not self._data_ports() and not self.config.record_tf:
            logger.warning("Recorder has no In ports — nothing to record, subclass the Recorder")
            return

        for name, port in self._data_ports().items():
            stream_name = self.config.stream_remapping.get(name, name)
            codec = self.config.stream_codecs.get(stream_name)
            overrides = {"codec": codec} if codec is not None else {}
            stream: Stream[Any] = self._stream_store(stream_name).stream(
                stream_name, port.type, **overrides
            )
            self._start_writer(name)
            self._port_to_stream(name, port, stream)
            logger.info("Recording %s -> %s (%s)", name, stream_name, port.type.__name__)

        if self.config.record_tf:
            self._record_tf()

    def _stream_store(self, stream_name: str) -> SqliteStore:
        """A dedicated store (own db file) for one stream: <base>.<stream>.db.

        A fresh file every run — the merge into config.db_path happens after
        the recording stops."""
        base = Path(self.config.db_path)
        path = base.with_name(f"{base.stem}.{stream_name}{base.suffix}")
        for stale in (path, Path(f"{path}-wal"), Path(f"{path}-shm")):
            if stale.exists():
                stale.unlink()
        store = self.register_disposable(SqliteStore(path=str(path)))
        store.start()
        self._stream_stores[stream_name] = store
        return store

    def _start_writer(self, name: str) -> None:
        queue: Unbounded[Callable[[], None]] = Unbounded()
        thread = threading.Thread(
            target=self._write_queued, args=(name, queue), name=f"record-{name}", daemon=True
        )
        self._writers[name] = (queue, thread)
        thread.start()

    def _data_ports(self) -> dict[str, In[Any]]:
        """The In ports to record generically — everything but the tf port."""
        return {name: port for name, port in self.inputs.items() if port is not self.tf}

    def _port_to_stream(self, name: str, input_topic: In[Any], stream: Stream[Any]) -> None:
        """Append each message from *input_topic* to *stream*, attaching world pose via tf.

        Stamped messages use their own ``.frame_id`` and ``.ts``; unstamped
        messages (or ones whose frame isn't in the tf graph, e.g. a payload
        already in world coords) fall back to ``config.default_frame_id`` —
        so every observation gets a robot-pose anchor when tf is publishing.

        Messages are queued and written by the stream's own writer thread.
        Writing on the module's event loop loses data: the dispatcher behind
        ``process_observable`` coalesces to LATEST, so whatever piles up behind
        a slow write is discarded rather than delayed. Each stream writes to
        its own db file — the WAL write lock is per file, so writers never
        contend (a shared file starves low-rate streams behind the image blobs
        until appends fail with "database is locked").
        """

        def write_one(recv_ts: float, msg: Any) -> None:
            ts = self._resolve_ts(name, msg)
            pose = self._resolve_pose(name, msg, ts)
            stream.append(msg, ts=ts, pose=pose, tags={"reception_ts": recv_ts})

        def enqueue(msg: Any) -> None:
            recv_ts = time.time()
            self._enqueue_write(name, lambda: write_one(recv_ts, msg))

        subscription = self.register_disposable(Disposable(input_topic.subscribe(enqueue)))
        self._write_subscriptions.append(subscription)

    def _enqueue_write(self, name: str, write: Callable[[], None]) -> None:
        writer = self._writers.get(name)
        if writer is None:
            return
        queue, _ = writer
        queue.put(write)
        depth = len(queue)
        now = time.time()
        if (
            depth > BACKLOG_WARN_DEPTH
            and now - self._backlog_warned.get(name, 0.0) > BACKLOG_WARN_INTERVAL
        ):
            self._backlog_warned[name] = now
            logger.warning(
                "[%s] %d observations waiting to be written — writes are not keeping up",
                name,
                depth,
            )

    def _write_queued(self, name: str, queue: Unbounded[Callable[[], None]]) -> None:
        count, spent = 0, 0.0
        stats_logged_at = time.time()
        for write in queue:
            started = time.perf_counter()
            try:
                write()
            except sqlite3.ProgrammingError:
                return  # The store closed under us during teardown.
            except Exception:
                logger.exception("[%s] Failed to record an observation", name)
            count += 1
            spent += time.perf_counter() - started
            now = time.time()
            if now - stats_logged_at >= WRITE_STATS_INTERVAL:
                interval = now - stats_logged_at
                logger.info(
                    "[%s] writer: %.0f%% busy, backlog %d, %d writes @ %.1fms",
                    name,
                    100 * spent / interval,
                    len(queue),
                    count,
                    1000 * spent / count,
                )
                stats_logged_at = now
                count, spent = 0, 0.0

    def _resolve_ts(self, name: str, msg: Any) -> float:
        """Timestamp to record *msg* at. Override to re-base onto another clock."""
        return getattr(msg, "ts", None) or time.time()

    def _resolve_pose(self, name: str, msg: Any, ts: float) -> Pose | None:
        """Pose to anchor *msg* with. Dispatches to the stream's (async)
        ``@pose_setter_for`` if one is defined, else falls back to a
        ``world <- frame_id`` tf lookup.

        Called from the stream's writer thread. The tf buffer is thread-safe;
        setters are coroutines, so they go to the module's event loop and are
        given a deadline — a wedged loop must not silently stop a recording.
        """
        pose: Pose | None = None
        setter = self._pose_setters.get(name)
        if setter is not None:
            try:
                pose = cast(
                    "Pose | None",
                    asyncio.run_coroutine_threadsafe(setter(msg), self._loop).result(
                        POSE_SETTER_TIMEOUT
                    ),
                )
            except TimeoutError:
                logger.warning("[%s] Pose setter timed out, storing without pose", name)
        elif self._tf is not None:
            frame_id = getattr(msg, "frame_id", None) or self.config.default_frame_id
            transform = self._tf.get(
                self.config.root_frame,
                frame_id,
                time_point=ts,
                time_tolerance=self.config.tf_tolerance,
            )
            pose = transform.to_pose() if transform is not None else None
        if pose is None and name not in self.config.poseless_streams:
            self._warn_missing_pose(name, ts)
        return pose

    def _warn_missing_pose(self, name: str, ts: float) -> None:
        """One line per stream per interval. Warning per message is its own
        outage: at full sensor rate the formatting and the pipe to the log
        collector cost more than the write they are describing."""
        missed, warned_at = self._missing_pose.get(name, (0, 0.0))
        missed += 1
        now = time.time()
        if now - warned_at < MISSING_POSE_WARN_INTERVAL:
            self._missing_pose[name] = (missed, warned_at)
            return
        self._missing_pose[name] = (0, now)
        logger.warning(
            "[%s] No pose for %d observation(s) (latest ts %s), storing them without pose",
            name,
            missed,
            ts,
        )

    def _collect_pose_setters(self) -> dict[str, PoseSetter]:
        """Map stream name -> bound ``@pose_setter_for`` method."""
        setters: dict[str, PoseSetter] = {}
        for attr_name in dir(type(self)):
            fn = getattr(type(self), attr_name, None)
            for stream in getattr(fn, "_pose_setter_for", ()):
                setters[stream] = getattr(self, attr_name)
        return setters

    def _record_tf(self) -> None:
        """Record the live tf stream under "tf" (no-op without a wired tf port)."""
        if getattr(self.tf, "_transport", None) is None:
            logger.warning("Recorder: tf port has no transport — not recording tf")
            return
        tf_stream = self._stream_store("tf").stream("tf", TFMessage)
        self._start_writer("tf")

        def write_tf(msg: TFMessage) -> None:
            for transform in msg.transforms:
                tf_stream.append(TFMessage(transform), ts=transform.ts, pose=None)

        def on_tf(msg: TFMessage) -> None:
            self._enqueue_write("tf", lambda: write_tf(msg))

        subscription = self.register_disposable(Disposable(self.tf.subscribe(on_tf)))
        self._write_subscriptions.append(subscription)

    @rpc
    def stop(self) -> None:
        """Stop taking messages, write what is already queued, then tear down.

        Order matters: ``super().stop()`` closes the sqlite store, so anything
        still sitting in a writer queue has to reach the disk before that.
        """
        for subscription in self._write_subscriptions:
            subscription.dispose()
        self._write_subscriptions = []
        writers = self._writers
        self._writers = {}
        for queue, _ in writers.values():
            queue.close()
        deadline = time.monotonic() + WRITER_FLUSH_TIMEOUT
        for name, (queue, thread) in writers.items():
            thread.join(max(0.0, deadline - time.monotonic()))
            if thread.is_alive():
                logger.warning(
                    "[%s] Writer did not finish in %.0fs — %d observations dropped",
                    name,
                    WRITER_FLUSH_TIMEOUT,
                    len(queue),
                )
        stream_names = list(self._stream_stores)
        self._stream_stores = {}
        super().stop()  # disposes the per-stream stores, flushing their WALs
        if stream_names:
            merge_stream_dbs(Path(self.config.db_path), stream_names)
