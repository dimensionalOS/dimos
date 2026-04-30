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

import inspect
import os
from pathlib import Path
from typing import TYPE_CHECKING, Any, Generic, TypeVar, cast

from pydantic import field_validator
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.memory2.embed import EmbedImages
from dimos.memory2.store.null import NullStore
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.stream import Stream
from dimos.memory2.transform import QualityWindow
from dimos.memory2.type.observation import EmbeddedObservation, Observation
from dimos.models.embedding.base import EmbeddingModel
from dimos.models.embedding.clip import CLIPModel
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
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

    Iteration runs on the dimos thread pool via :meth:`Stream.observable`.
    """

    def _on_error(e: Exception) -> None:
        logger.error("stream_to_port() pipeline error: %s", e, exc_info=True)

    return stream.observable().subscribe(
        on_next=lambda obs: out.publish(obs.data),
        on_error=_on_error,
    )


def port_to_stream(in_: In[T], stream: Stream[T]) -> DisposableBase:
    """Append each message received on a Module ``In`` port to *stream*."""
    return Disposable(in_.subscribe(stream.append))


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
        self.register_disposable(port_to_stream(in_port, stream))

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


class RecorderConfig(MemoryModuleConfig):
    overwrite: bool = True


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

        return results.transform(peaks(key=_similarity, distance=1.0)).last().pose_stamped


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

        # TODO: store reset API/logic is not implemented yet. This module
        # shouldn't need to know about files (SqliteStore specific), and
        # .live() subs need to know how to re-sub in case of a restart of
        # this module in a deployed blueprint.
        db_path = Path(self.config.db_path)
        if db_path.exists():
            if self.config.overwrite:
                db_path.unlink()
                logger.info("Deleted existing recording %s", db_path)
            else:
                raise FileExistsError(f"Recording already exists: {db_path}")

        if not self.inputs:
            logger.warning("Recorder has no In ports — nothing to record, subclass the Recorder")
            return

        for name, port in self.inputs.items():
            stream: Stream[Any] = self.store.stream(name, port.type)
            self.register_disposable(port_to_stream(port, stream))
            logger.info("Recording %s (%s)", name, port.type.__name__)
            logger.info("Recording %s (%s)", name, port.type.__name__)
