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
from dataclasses import dataclass
import enum
import inspect
import os
from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING, Any, Generic, Literal, TypeVar, cast

from pydantic import Field, field_validator
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
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
    queue_maxsize: int = Field(default=1024, ge=1)
    queue_overflow: Literal["drop_new"] = "drop_new"
    drain_timeout: float = Field(default=30.0, gt=0)
    drop_warning_interval: float = Field(default=5.0, ge=0)


PoseSetter = Callable[[Any], "Awaitable[Pose | None]"]


@dataclass
class _RecorderMetrics:
    received: int = 0
    written: int = 0
    queued: int = 0
    dropped: int = 0
    failed: int = 0


@dataclass
class _RecordItem:
    name: str
    stream: Stream[Any]
    msg: Any
    recv_ts: float
    ts: float | None = None
    resolve_pose: bool = True


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

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._record_queue: asyncio.Queue[_RecordItem | None] | None = None
        self._writer_task: asyncio.Task[None] | None = None
        self._record_subscriptions: list[DisposableBase] = []
        self._record_metrics: dict[str, _RecorderMetrics] = {}
        self._writer_failures = 0
        self._metrics_lock = threading.Lock()
        self._accepting_records = False
        self._last_drop_warning: dict[str, float] = {}

    @rpc
    def start(self) -> None:
        super().start()

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
                pass  # keep the db; _prepare_streams handles any per-stream replacement
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

        self._prepare_streams()

        if not self._data_ports() and not self.config.record_tf:
            logger.warning("Recorder has no In ports — nothing to record, subclass the Recorder")
            return

        self._start_record_queue()

        for name, port in self._data_ports().items():
            stream_name = self.config.stream_remapping.get(name, name)
            codec = self.config.stream_codecs.get(stream_name)
            overrides = {"codec": codec} if codec is not None else {}
            stream: Stream[Any] = self.store.stream(stream_name, port.type, **overrides)
            self._port_to_stream(name, port, stream)
            logger.info("Recording %s -> %s (%s)", name, stream_name, port.type.__name__)

        if self.config.record_tf:
            self._record_tf()

    def _data_ports(self) -> dict[str, In[Any]]:
        """The In ports to record generically — everything but the tf port."""
        return {name: port for name, port in self.inputs.items() if port is not self.tf}

    def _port_to_stream(self, name: str, input_topic: In[Any], stream: Stream[Any]) -> None:
        """Append each message from *input_topic* to *stream*, attaching world pose via tf.

        Stamped messages use their own ``.frame_id`` and ``.ts``; unstamped
        messages (or ones whose frame isn't in the tf graph, e.g. a payload
        already in world coords) fall back to ``config.default_frame_id`` —
        so every observation gets a robot-pose anchor when tf is publishing.

        Each port feeds the recorder's bounded FIFO. A single writer task drains
        that queue in arrival order and attaches poses before writing to SQLite.
        """

        def on_msg(msg: Any) -> None:
            self._submit_record(_RecordItem(name, stream, msg, time.time()))

        self._record_subscriptions.append(input_topic.pure_observable().subscribe(on_msg))

    def _start_record_queue(self) -> None:
        loop = self._loop
        if loop is None or not loop.is_running():
            raise RuntimeError(f"{type(self).__name__}._loop is not running")

        async def bootstrap() -> tuple[asyncio.Queue[_RecordItem | None], asyncio.Task[None]]:
            queue: asyncio.Queue[_RecordItem | None] = asyncio.Queue(
                maxsize=self.config.queue_maxsize
            )
            self._record_queue = queue
            return queue, asyncio.create_task(self._record_writer())

        self._record_queue, self._writer_task = asyncio.run_coroutine_threadsafe(
            bootstrap(), loop
        ).result(timeout=5.0)
        with self._metrics_lock:
            self._accepting_records = True

    def _submit_record(self, item: _RecordItem) -> None:
        loop = self._loop
        with self._metrics_lock:
            metrics = self._record_metrics.setdefault(item.name, _RecorderMetrics())
            metrics.received += 1
            if not self._accepting_records or loop is None or not loop.is_running():
                metrics.dropped += 1
                self._warn_record_drop(item.name, metrics.dropped)
                return
            loop.call_soon_threadsafe(self._enqueue_record, item)

    def _enqueue_record(self, item: _RecordItem) -> None:
        queue = self._record_queue
        if queue is None:
            return
        with self._metrics_lock:
            metrics = self._record_metrics[item.name]
            if queue.full():
                metrics.dropped += 1
                self._warn_record_drop(item.name, metrics.dropped)
                return
            queue.put_nowait(item)
            metrics.queued += 1

    def _warn_record_drop(self, name: str, dropped: int) -> None:
        now = time.monotonic()
        last = self._last_drop_warning.get(name, float("-inf"))
        if now - last < self.config.drop_warning_interval:
            return
        self._last_drop_warning[name] = now
        logger.warning(
            "Recorder queue saturated; dropping new message for %s (dropped=%d, maxsize=%d)",
            name,
            dropped,
            self.config.queue_maxsize,
        )

    async def _record_writer(self) -> None:
        queue = self._record_queue
        assert queue is not None
        while True:
            item = await queue.get()
            if item is None:
                queue.task_done()
                return
            with self._metrics_lock:
                self._record_metrics[item.name].queued -= 1
            try:
                ts = item.ts if item.ts is not None else self._resolve_ts(item.name, item.msg)
                pose = (
                    await self._resolve_pose(item.name, item.msg, ts) if item.resolve_pose else None
                )
                if not pose and item.resolve_pose and item.name not in self.config.poseless_streams:
                    logger.warning(
                        "[%s] No pose for time %s (msg ts: %s), storing without pose",
                        item.name,
                        ts,
                        getattr(item.msg, "ts", None),
                    )
                item.stream.append(
                    item.msg,
                    ts=ts,
                    pose=pose,
                    tags={"reception_ts": item.recv_ts},
                )
                with self._metrics_lock:
                    self._record_metrics[item.name].written += 1
            except Exception:
                with self._metrics_lock:
                    self._record_metrics[item.name].failed += 1
                    self._writer_failures += 1
                logger.exception("Failed to record message for stream %s", item.name)
            finally:
                queue.task_done()

    @rpc
    def recording_metrics(self) -> dict[str, dict[str, int]]:
        """Return per-stream recorder queue and write counters."""
        with self._metrics_lock:
            return {
                name: {
                    "received": metrics.received,
                    "written": metrics.written,
                    "queued": metrics.queued,
                    "dropped": metrics.dropped,
                    "failed": metrics.failed,
                }
                for name, metrics in self._record_metrics.items()
            }

    @rpc
    def stop(self) -> None:
        if self.config.g.replay or self._record_queue is None:
            super().stop()
            return

        for subscription in self._record_subscriptions:
            subscription.dispose()
        self._record_subscriptions.clear()
        with self._metrics_lock:
            self._accepting_records = False

        loop = self._loop
        if loop is None or not loop.is_running():
            raise RuntimeError("Recorder event loop stopped before queued writes drained")

        async def drain() -> None:
            assert self._record_queue is not None
            assert self._writer_task is not None
            await self._record_queue.join()
            await self._record_queue.put(None)
            await self._writer_task

        try:
            asyncio.run_coroutine_threadsafe(drain(), loop).result(
                timeout=self.config.drain_timeout
            )
        except TimeoutError as exc:
            pending = sum(
                metrics.received - metrics.written - metrics.dropped - metrics.failed
                for metrics in self._record_metrics.values()
            )
            raise RuntimeError(
                f"Recorder failed to drain {pending} queued messages within "
                f"{self.config.drain_timeout}s; store left open"
            ) from exc

        if self._store is not None:
            self._store.checkpoint()
        failures = self._writer_failures
        self._record_queue = None
        self._writer_task = None
        super().stop()
        if failures:
            raise RuntimeError(f"Recorder failed to write {failures} accepted messages")

    def _prepare_streams(self) -> None:
        """On APPEND, drop the streams this recorder is about to (re)write — the
        remapped In-port streams plus ``tf`` — so a re-run replaces them instead
        of duplicating, while leaving any other streams in the db untouched."""
        if self.config.on_existing is not OnExisting.APPEND:
            return
        targets = {self.config.stream_remapping.get(name, name) for name in self._data_ports()}
        if self.config.record_tf:
            targets.add("tf")
        for stream in targets.intersection(self.store.list_streams()):
            self.store.delete_stream(stream)

    def _resolve_ts(self, name: str, msg: Any) -> float:
        """Timestamp to record *msg* at. Override to re-base onto another clock."""
        return getattr(msg, "ts", None) or time.time()

    async def _resolve_pose(self, name: str, msg: Any, ts: float) -> Pose | None:
        """Pose to anchor *msg* with. Dispatches to the stream's (async)
        ``@pose_setter_for`` if one is defined, else falls back to a
        ``world <- frame_id`` tf lookup."""
        if name in self.config.poseless_streams:
            return None
        setter = self._pose_setters.get(name)
        if setter is not None:
            return cast("Pose | None", await setter(msg))
        if self._tf is None:
            return None
        frame_id = getattr(msg, "frame_id", None) or self.config.default_frame_id
        transform = self._tf.get(
            self.config.root_frame, frame_id, time_point=ts, time_tolerance=self.config.tf_tolerance
        )
        return transform.to_pose() if transform is not None else None

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
        tf_stream = self.store.stream("tf", TFMessage)

        def on_tf(msg: TFMessage) -> None:
            for transform in msg.transforms:
                self._submit_record(
                    _RecordItem(
                        "tf",
                        tf_stream,
                        TFMessage(transform),
                        time.time(),
                        ts=transform.ts,
                        resolve_pose=False,
                    )
                )

        self._record_subscriptions.append(Disposable(self.tf.subscribe(on_tf)))
