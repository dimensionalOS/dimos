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

from typing import Any, TypeVar, cast

from dimos.core.resource import CompositeResource
from dimos.memory2.backend import Backend
from dimos.memory2.blobstore.base import BlobStore
from dimos.memory2.codecs.base import Codec, codec_for, codec_from_id
from dimos.memory2.notifier.base import Notifier
from dimos.memory2.notifier.subject import SubjectNotifier
from dimos.memory2.observationstore.base import ObservationStore
from dimos.memory2.observationstore.memory import ListObservationStore
from dimos.memory2.stream import Stream
from dimos.memory2.vectorstore.base import VectorStore
from dimos.protocol.service.spec import BaseConfig, Configurable

T = TypeVar("T")


class StreamAccessor:
    """Attribute-style access: ``store.streams.name`` -> ``store.stream(name)``."""

    __slots__ = ("_store",)

    def __init__(self, store: Store) -> None:
        self._store = store

    def __getattr__(self, name: str) -> Stream[Any]:
        if name.startswith("_"):
            raise AttributeError(name)
        if name not in self._store.list_streams():
            raise AttributeError(f"No stream {name!r}. Available: {self._store.list_streams()}")
        return self._store.stream(name)

    def __getitem__(self, name: str) -> Stream[Any]:
        if name not in self._store.list_streams():
            raise KeyError(name)
        return self._store.stream(name)

    def __dir__(self) -> list[str]:
        return self._store.list_streams()

    def __repr__(self) -> str:
        return f"StreamAccessor({self._store.list_streams()})"

    def items(self) -> list[tuple[str, Stream[Any]]]:
        return [(name, self._store.stream(name)) for name in self._store.list_streams()]


class StoreConfig(BaseConfig):
    """Store-level config. These are defaults inherited by all streams.

    Component fields accept either a class (instantiated per-stream) or
    a live instance (used directly). Classes are the default; instances
    are for overrides (e.g. spy stores in tests, shared external stores).
    """

    observation_store: type[ObservationStore] | ObservationStore | None = None  # type: ignore[type-arg]
    blob_store: type[BlobStore] | BlobStore | None = None
    vector_store: type[VectorStore] | VectorStore | None = None
    notifier: type[Notifier] | Notifier | None = None  # type: ignore[type-arg]
    eager_blobs: bool = False


class Store(Configurable, CompositeResource):
    """Top-level entry point — wraps a storage location (file, URL, etc.).

    Store directly manages streams. No Session layer.
    """

    config: StoreConfig

    def __init__(self, **kwargs: Any) -> None:
        Configurable.__init__(self, **kwargs)
        CompositeResource.__init__(self)
        self._streams: dict[str, Stream[Any]] = {}

    @property
    def streams(self) -> StreamAccessor:
        """Attribute-style access to streams: ``store.streams.name``."""
        return StreamAccessor(self)

    @staticmethod
    def _resolve_codec(
        payload_type: type[Any] | None, raw_codec: Codec[Any] | str | None
    ) -> Codec[Any]:
        if isinstance(raw_codec, Codec):
            return raw_codec
        if isinstance(raw_codec, str):
            module = (
                f"{payload_type.__module__}.{payload_type.__qualname__}"
                if payload_type
                else "builtins.object"
            )
            return codec_from_id(raw_codec, module)
        return codec_for(payload_type)

    def _create_backend(
        self, name: str, payload_type: type[Any] | None = None, **config: Any
    ) -> Backend[Any]:
        """Create a Backend for the named stream. Called once per stream name."""
        codec = self._resolve_codec(payload_type, config.pop("codec", None))

        # Instantiate or use provided instances
        obs = config.pop("observation_store", self.config.observation_store)
        if obs is None or isinstance(obs, type):
            obs = (obs or ListObservationStore)(name=name)

        bs = config.pop("blob_store", self.config.blob_store)
        if isinstance(bs, type):
            bs = bs()

        vs = config.pop("vector_store", self.config.vector_store)
        if isinstance(vs, type):
            vs = vs()

        notifier = config.pop("notifier", self.config.notifier)
        if notifier is None or isinstance(notifier, type):
            notifier = (notifier or SubjectNotifier)()

        return Backend(
            metadata_store=obs,
            codec=codec,
            data_type=payload_type or object,
            blob_store=bs,
            vector_store=vs,
            notifier=notifier,
            eager_blobs=config.get("eager_blobs", False),
        )

    def stream(self, name: str, payload_type: type[T] | None = None, **overrides: Any) -> Stream[T]:
        """Get or create a named stream. Returns the same Stream on repeated calls.

        Per-stream ``overrides`` (e.g. ``blob_store=``, ``codec=``) are merged
        on top of the store-level defaults from :class:`StoreConfig`.
        """
        if name not in self._streams:
            resolved = {**self.config.model_dump(exclude_none=True), **overrides}
            backend = self._create_backend(name, payload_type, **resolved)
            backend.start()
            self._streams[name] = Stream(source=backend)
        return cast("Stream[T]", self._streams[name])

    def list_streams(self) -> list[str]:
        """Return names of all streams in this store."""
        return list(self._streams.keys())

    def delete_stream(self, name: str) -> None:
        """Delete a stream by name (from cache and underlying storage)."""
        stream = self._streams.pop(name, None)
        if stream is not None:
            stream.stop()

    def stop(self) -> None:
        for stream in self._streams.values():
            stream.stop()
        super().stop()
