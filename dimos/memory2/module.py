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

from collections.abc import Awaitable, Callable
import enum
import inspect
import os
from pathlib import Path
import sqlite3
import time
from typing import TYPE_CHECKING, Any, Generic, TypeVar, cast

from pydantic import Field, field_validator
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.memory2.db_tf import TfGraph
from dimos.memory2.embed import EmbedImages
from dimos.memory2.store.null import NullStore
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.stream import Stream
from dimos.memory2.transform import QualityWindow
from dimos.memory2.type.observation import EmbeddedObservation, Observation
from dimos.models.embedding.base import EmbeddingModel
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.DeformationNode import DeformationNode
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
    """Module base class that wires a memory2 stream pipeline
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
    and the pull-based memory2 stream pipeline — it also enables replay and
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
    # The pose of a match is resolved at QUERY time via store.tf (so it reflects loop
    # closure), looking up root_frame <- the observation's frame_id at its timestamp.
    root_frame: str = "world"
    tf_tolerance: float = 0.5


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
        from dimos.memory2.transform import peaks

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

        # Resolve the match's world pose at QUERY time via tf, so it reflects any loop
        # closure since the observation was recorded (the baked pose_stamped is stale).
        frame_id = getattr(best.data, "frame_id", None) or self.config.root_frame
        transform = self.store.tf.get(
            self.config.root_frame, frame_id, best.ts, self.config.tf_tolerance
        )
        if transform is not None:
            return transform.to_pose(ts=best.ts)
        # No tf chain at that time — fall back to the pose baked at record time.
        if best.pose_stamped is None:
            raise LookupError("No tf and no baked pose on best search result")
        logger.warning(
            "SemanticSearch: no tf for %s @ %s; using stale baked pose", frame_id, best.ts
        )
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


PoseSetter = Callable[[Any], "Awaitable[Pose | None]"]


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
    """Records all ``In`` ports to a memory2 SQLite database, plus the live tf tree.

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

    tf_static: In[TFMessage]
    tf_deformation_nodes: In[DeformationNode]

    _pose_setters: dict[str, Any] = {}
    # Per-stream count of frames lost to the dispatcher's LATEST coalescing
    # (sink slower than input). Populated lazily as drops happen.
    _dropped_frames: dict[str, int] = {}

    @rpc
    def start(self) -> None:
        super().start()

        if self.config.g.replay:
            logger.info(
                "Replay mode active — Recorder disabled, leaving %s untouched", self.config.db_path
            )
            return

        self._pose_setters = self._collect_pose_setters()
        self._dropped_frames = {}

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

        self._prepare_streams()

        if not self.inputs and not self.config.record_tf:
            logger.warning("Recorder has no In ports — nothing to record, subclass the Recorder")
            return

        for name, port in self.inputs.items():
            if name == "tf_static":
                continue  # folded into the "tf" stream + graph by _record_tf
            if name == "tf_deformation_nodes":
                continue  # recorded by _record_tf into its own stream (carries its own pose)
            stream_name = self.config.stream_remapping.get(name, name)
            stream: Stream[Any] = self.store.stream(stream_name, port.type)
            self._port_to_stream(name, port, stream)
            logger.info("Recording %s -> %s (%s)", name, stream_name, port.type.__name__)

        if self.config.record_tf:
            self._record_tf()

    def _port_to_stream(self, name: str, input_topic: In[Any], stream: Stream[Any]) -> None:
        """Append each message from *input_topic* to *stream*, attaching world pose via tf.

        Stamped messages use their own ``.frame_id`` and ``.ts``; unstamped
        messages (or ones whose frame isn't in the tf graph, e.g. a payload
        already in world coords) fall back to ``config.default_frame_id`` —
        so every observation gets a robot-pose anchor when tf is publishing.

        Each port is recorded by an async callback dispatched on the module's
        event loop via :meth:`process_observable`, which serialises invocations
        and registers the subscription for cleanup on stop().
        """

        async def on_msg(msg: Any) -> None:
            ts = self._resolve_ts(name, msg)
            pose = await self._resolve_pose(name, msg, ts)
            if not pose:
                logger.warning(
                    "[%s] No pose for time %s (msg ts: %s), storing without pose",
                    name,
                    ts,
                    getattr(msg, "ts", None),
                )
            stream.append(msg, ts=ts, pose=pose)

        self.process_observable(
            input_topic.pure_observable(), on_msg, on_drop=lambda: self._on_frame_dropped(name)
        )

    def _on_frame_dropped(self, name: str) -> None:
        """A frame for *name* was dropped because the sink couldn't keep up with
        the input rate (dispatcher LATEST coalescing). Count it and warn — once,
        then on each power-of-ten — so silent data loss is visible without
        flooding the log."""
        count = self._dropped_frames.get(name, 0) + 1
        self._dropped_frames[name] = count
        if count == 1 or count % 1000 == 0:
            logger.warning(
                "[%s] Recorder dropped %d frame(s) — sink slower than input; recording is lossy",
                name,
                count,
            )

    def _prepare_streams(self) -> None:
        """On APPEND, clear only the streams this recorder will actually (re)write —
        i.e. those with a connected source — so a re-run overwrites them cleanly.
        Streams whose source is unconnected are left untouched: recording into an
        existing db never drops data it won't repopulate (e.g. pcap_to_db, which has
        no tf source, must not wipe an existing tf / tf_deformation_nodes)."""
        if self.config.on_existing is not OnExisting.APPEND:
            return
        targets: set[str] = set()
        for name, port in self.inputs.items():
            if name in ("tf_static", "tf_deformation_nodes"):
                continue  # tf-family, handled below only when their source is connected
            if port.transport is None:
                continue  # unconnected -> nothing to write, leave any existing data
            targets.add(self.config.stream_remapping.get(name, name))
        if self.config.record_tf:
            topic = getattr(self.tf.config, "topic", None)
            pubsub = getattr(self.tf, "pubsub", None)
            if (topic and pubsub is not None) or self.tf_static.transport is not None:
                targets.update(("tf", "tf_graph"))
            if self.tf_deformation_nodes.transport is not None:
                targets.add("tf_deformation_nodes")
        for stream in targets.intersection(self.store.list_streams()):
            self.store.delete_stream(stream)

    def _resolve_ts(self, name: str, msg: Any) -> float:
        """Timestamp to record *msg* at. Override to re-base onto another clock."""
        return getattr(msg, "ts", None) or time.time()

    async def _resolve_pose(self, name: str, msg: Any, ts: float) -> Pose | None:
        """Pose to anchor *msg* with. Dispatches to the stream's (async)
        ``@pose_setter_for`` if one is defined, else falls back to a
        ``world <- frame_id`` tf lookup."""
        setter = self._pose_setters.get(name)
        if setter is not None:
            return cast("Pose | None", await setter(msg))
        frame_id = getattr(msg, "frame_id", None) or self.config.default_frame_id
        transform = self.tf.get(
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
        """Record tf into the "tf" stream + the topology change-log ("tf_graph").

        Two inputs, both folded into one tf stream: the live (dynamic) tf via the
        module's tf interface, and the optional ``tf_static`` In-port stream (a
        future system publishes latched mount/extrinsic transforms there). Frames
        from tf_static are flagged static in the graph; latched statics resolve
        for all time, dynamic frames bracket+interpolate."""
        tf_stream = self.store.stream("tf", TFMessage)
        graph_stream = self.store.stream("tf_graph", TfGraph)
        # Running tf topology; a TfGraph snapshot is appended whenever it changes, so
        # the "tf_graph" stream is the topology change-log transform lookups walk.
        structure: dict[str, dict[str, Any]] = {}

        def record(msg: TFMessage, is_static: bool) -> None:
            try:
                for transform in msg.transforms:
                    tf_stream.append(
                        TFMessage(transform),
                        ts=transform.ts,
                        pose=None,
                        tags={"child_frame": transform.child_frame_id},
                    )
                    entry = {"parent": transform.frame_id, "static": is_static}
                    if structure.get(transform.child_frame_id) != entry:
                        structure[transform.child_frame_id] = entry
                        graph_stream.append(TfGraph(structure), ts=transform.ts)
            except sqlite3.ProgrammingError:
                # A late callback raced teardown and hit the closed store.
                pass

        # static tf: the tf_static In-port stream. Only subscribe when something is
        # wired to it (its transport is set on connect) — unconnected today.
        if self.tf_static.transport is not None:

            async def on_static(msg: TFMessage) -> None:
                record(msg, is_static=True)

            self.process_observable(self.tf_static.pure_observable(), on_static)

        self._record_deformation_nodes()

        # dynamic tf: the module's live tf interface
        topic = getattr(self.tf.config, "topic", None)
        pubsub = getattr(self.tf, "pubsub", None)
        if not topic or pubsub is None:
            logger.warning("Recorder: no pubsub tf available — recording static tf only")
            return
        self.register_disposable(
            Disposable(pubsub.subscribe(topic, lambda msg, _t: record(msg, False)))
        )

    def _record_deformation_nodes(self) -> None:
        """Record the optional ``tf_deformation_nodes`` In-port into its own stream.

        Each DeformationNode is one pose-graph keyframe; the backend re-publishes a
        node (same ``id``) when the optimizer moves it, so rows accumulate and a query
        takes the latest per ``id``. Tagged by ``tf_id`` (which transform edge) and
        ``id`` so lookups can filter by edge and dedup by node. Unconnected = no-op."""
        if self.tf_deformation_nodes.transport is None:
            return
        stream = self.store.stream("tf_deformation_nodes", DeformationNode)

        async def on_node(node: DeformationNode) -> None:
            try:
                stream.append(
                    node,
                    ts=node.pose.ts,
                    pose=None,
                    tags={"tf_id": str(node.tf_id), "id": str(node.id)},
                )
            except sqlite3.ProgrammingError:
                pass  # late callback raced teardown and hit the closed store

        self.process_observable(self.tf_deformation_nodes.pure_observable(), on_node)
