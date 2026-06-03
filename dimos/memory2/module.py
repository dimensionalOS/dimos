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

import enum
import inspect
import os
from pathlib import Path
import time
from typing import TYPE_CHECKING, Any, Generic, TypeVar, cast

from pydantic import Field, field_validator
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.memory2.embed import EmbedImages
from dimos.memory2.fanio import normalize_to_bundle, scatter_to_ports
from dimos.memory2.store.base import StreamAccessor
from dimos.memory2.store.null import NullStore
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.stream import Stream
from dimos.memory2.transform import QualityWindow
from dimos.memory2.type.observation import EmbeddedObservation, Observation
from dimos.models.embedding.base import EmbeddingModel
from dimos.models.embedding.clip import CLIPModel
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.data import backup_file
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from reactivex.abc import DisposableBase

    from dimos.core.stream import In, Out

logger = setup_logger()

T = TypeVar("T")
TIn = TypeVar("TIn")
TOut = TypeVar("TOut")


def stream_to_port(stream: Stream[T], out: Out[T]) -> DisposableBase:
    """Forward each observation's ``data`` from *stream* to a Module ``Out`` port.

    Thin back-compat alias kept so existing imports keep working: it normalizes a
    raw single-output stream into the bundle-only scatter contract
    (:func:`normalize_to_bundle`) and fans it to the one port. A stream that
    already yields a :class:`Bundle` routes by key instead. Iteration runs on the
    dimos thread pool via :meth:`Stream.observable`.
    """
    ports = {out.name: out}
    return scatter_to_ports(normalize_to_bundle(stream, ports), ports)


class _LiveInputs:
    """Adapts the In-port stream dict to the :class:`StreamAccessor` protocol.

    ``stream(name)`` returns a fresh ``.live()`` view each call so a sibling
    reached via ``self.streams.<port>`` tails new data rather than replaying an
    already-consumed iterator.
    """

    def __init__(self, streams: dict[str, Stream[Any]]) -> None:
        self._streams = streams

    def list_streams(self) -> list[str]:
        return list(self._streams)

    def stream(self, name: str) -> Stream[Any]:
        return self._streams[name].live()


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

    **Fan-I/O (N inputs / M outputs)**

    Port count is data, not control flow: declare any number of ``In`` and
    ``Out`` ports. On start, *every* ``In`` port feeds a MemoryStore and is
    exposed as a live ``Stream`` via ``self.streams.<port>``; the pipeline runs
    over the first declared ``In`` (the primary) and reaches siblings inside
    ``pipeline()`` to align them::

        class MyFusion(StreamModule):
            lidar: In[PointCloud2]
            pose:  In[PoseStamped]
            map:   Out[PointCloud2]

            def pipeline(self, lidar: Stream[PointCloud2]) -> Stream[PointCloud2]:
                return lidar.align(self.streams.pose, tolerance=0.1).transform(...)

    The pipeline ends in a :class:`Bundle` keyed by ``Out`` port name and
    :func:`scatter_to_ports` fans it out in one subscribe, so a fused pipeline
    computes once per tick regardless of output count. Scatter is M-agnostic: a
    single ``Out`` reads ``bundle[its_name]`` exactly as several ``Out``\\s each
    read their own key, so a 1->1 module with a ``Bundle`` tail needs no second
    port to "turn on" scatter. A 1:1 pipeline that still yields its raw payload is
    wrapped into a one-key bundle at the start boundary
    (:func:`normalize_to_bundle`), so the same scatter path serves every module.

    The MemoryStore acts as a bridge between the push-based Module In port
    and the pull-based memory2 stream pipeline — it also enables replay and
    persistence if the store is swapped for a persistent backend later.
    """

    _in_streams: dict[str, Stream[Any]]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

    @property
    def streams(self) -> StreamAccessor[Stream[Any]]:
        """Live ``Stream`` view of every In port, by name (mirrors ``store.streams``).

        Use inside ``pipeline()`` to reach sibling inputs for alignment::

            image.align(self.streams.pose, tolerance=0.1)

        Each access returns a fresh ``.live()`` view, so reference a sibling
        once; use ``.materialize()`` to fan one secondary into two aligns. Do
        not re-read the primary through this accessor (it would open a second
        live subscription). A missing port name raises ``AttributeError``
        listing the available streams.
        """
        return StreamAccessor(_LiveInputs(self._in_streams))

    def ingest(self, name: str, stream: Stream[Any], msg: Any) -> None:
        """Append an incoming *msg* from In port *name* to its backing *stream*.

        Override to enrich (e.g. anchor a robot pose), tag, or drop messages
        before they enter the pipeline. This is the seam that replaces copying
        ``start()``. The default stamps each observation with the message's own
        ``.ts`` when present (so cross-port ``.align()`` is meaningful), falling
        back to arrival time for unstamped payloads such as bare ints.
        """
        stream.append(msg, ts=getattr(msg, "ts", None) or time.time())

    def _wire_input(self, name: str, port: In[Any], store: NullStore) -> Stream[Any]:
        """Create the backing stream for an In *port* and route the port into it.

        Subscribing through :meth:`ingest` (rather than ``stream.append``
        directly) is what lets subclasses customize ingestion without copying
        ``start()``.
        """
        stream: Stream[Any] = store.stream(name, port.type)
        unsub = port.subscribe(lambda m: self.ingest(name, stream, m))
        self.register_disposable(Disposable(unsub) if callable(unsub) else unsub)
        return stream

    @rpc
    def start(self) -> None:
        super().start()

        if not self.inputs or not self.outputs:
            raise TypeError(
                f"{self.__class__.__name__} needs at least one In and one Out port, "
                f"found {len(self.inputs)} In and {len(self.outputs)} Out"
            )

        store = self.register_disposable(NullStore())
        store.start()

        # Every In port becomes a live-capable stream; ingest() bridges push -> pull.
        self._in_streams = {
            name: self._wire_input(name, port, store) for name, port in self.inputs.items()
        }

        # First declared In is the primary the pipeline runs over (C1); siblings
        # are reached inside pipeline() via self.streams.<port> for .align().
        primary_name = next(iter(self.inputs))
        produced = self._apply_pipeline(self._in_streams[primary_name].live())

        # Normalize a raw 1:1 payload into a one-key Bundle at the start boundary so
        # scatter stays bundle-only and M-agnostic; a Bundle-tail pipeline (1->1 or
        # N->M) passes through. One subscribe regardless of port count -> the
        # pipeline runs once per tick (C3).
        bundled = normalize_to_bundle(produced, self.outputs)
        self.register_disposable(scatter_to_ports(bundled, self.outputs))

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
    embedding_model: type[EmbeddingModel] = CLIPModel


class SemanticSearch(MemoryModule):
    config: SemanticSearchConfig
    model: EmbeddingModel | None = None
    embeddings: Stream[Any] | None = None

    @rpc
    def start(self) -> None:
        super().start()

        self.model = self.register_disposable(self.config.embedding_model())
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
        if best.pose_stamped is None:
            raise LookupError("No pose on best search result")
        return best.pose_stamped


class OnExisting(str, enum.Enum):
    OVERWRITE = "overwrite"
    ERROR = "error"
    BACKUP = "backup"


class RecorderConfig(MemoryModuleConfig):
    on_existing: OnExisting = OnExisting.BACKUP
    backup_keep_last: int = Field(default=10, ge=0)
    default_frame_id: str = "base_link"
    tf_tolerance: float = 0.5
    db_path: str | Path = "recording.db"


class Recorder(MemoryModule):
    """Records all ``In`` ports to a memory2 SQLite database.

    Subclass with the topics you want to record::

        class MyRecorder(Recorder):
            color_image: In[Image]
            lidar: In[PointCloud2]

        blueprint.add(MyRecorder, db_path="session.db")
    """

    config: RecorderConfig

    @rpc
    def start(self) -> None:
        super().start()

        if self.config.g.replay:
            logger.info(
                "Replay mode active — Recorder disabled, leaving %s untouched", self.config.db_path
            )
            return

        # TODO: store reset API/logic is not implemented yet. This module
        # shouldn't need to know about files (SqliteStore specific), and
        # .live() subs need to know how to re-sub in case of a restart of
        # this module in a deployed blueprint.
        db_path = Path(self.config.db_path)
        if db_path.exists():
            if self.config.on_existing is OnExisting.OVERWRITE:
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

        if not self.inputs:
            logger.warning("Recorder has no In ports — nothing to record, subclass the Recorder")
            return

        for name, port in self.inputs.items():
            stream: Stream[Any] = self.store.stream(name, port.type)
            self._port_to_stream(name, port, stream)
            logger.info("Recording %s (%s)", name, port.type.__name__)

    def _port_to_stream(self, name: str, input_topic: In[Any], stream: Stream[Any]) -> None:
        """Append each message from *input_topic* to *stream*, attaching world pose via tf.

        Stamped messages use their own ``.frame_id`` and ``.ts``; unstamped
        messages (or ones whose frame isn't in the tf graph, e.g. a payload
        already in world coords) fall back to ``config.default_frame_id`` —
        so every observation gets a robot-pose anchor when tf is publishing.

        Registers the subscription as a disposable on this module.
        """

        default_frame_id = self.config.default_frame_id
        tf_tolerance = self.config.tf_tolerance

        def on_msg(msg: Any) -> None:
            ts = getattr(msg, "ts", None) or time.time()
            frame_id = getattr(msg, "frame_id", None) or default_frame_id
            transform = self.tf.get("world", frame_id, time_point=ts, time_tolerance=tf_tolerance)
            pose = transform.to_pose() if transform is not None else None

            if not pose:
                logger.warning(
                    "[%s] No tf available for frame '%s' at time %s (msg ts: %s), storing without pose",
                    name,
                    frame_id,
                    ts,
                    getattr(msg, "ts", None),
                )
            stream.append(msg, ts=ts, pose=pose)

        self.register_disposable(Disposable(input_topic.subscribe(on_msg)))
