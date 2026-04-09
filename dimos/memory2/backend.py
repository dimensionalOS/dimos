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

"""Concrete composite Backend that orchestrates ObservationStore + BlobStore + VectorStore + Notifier."""

from __future__ import annotations

from dataclasses import replace
from typing import TYPE_CHECKING, Any, Generic, TypeVar

from dimos.core.resource import CompositeResource
from dimos.memory2.codecs.base import Codec, codec_id
from dimos.memory2.notifier.subject import SubjectNotifier
from dimos.memory2.type.observation import _UNLOADED

if TYPE_CHECKING:
    from collections.abc import Iterator

    from reactivex.abc import DisposableBase

    from dimos.memory2.blobstore.base import BlobStore
    from dimos.memory2.buffer import BackpressureBuffer
    from dimos.memory2.notifier.base import Notifier
    from dimos.memory2.observationstore.base import ObservationStore
    from dimos.memory2.type.filter import StreamQuery
    from dimos.memory2.type.observation import Observation
    from dimos.memory2.vectorstore.base import VectorStore

T = TypeVar("T")


class Backend(CompositeResource, Generic[T]):
    """Orchestrates metadata, blob, vector, and live stores for one stream.
    (encode → insert → store blob → index vector → notify) lives here,
    """

    def __init__(
        self,
        *,
        metadata_store: ObservationStore[T],
        codec: Codec[Any],
        blob_store: BlobStore | None = None,
        vector_store: VectorStore | None = None,
        notifier: Notifier[T] | None = None,
        eager_blobs: bool = False,
    ) -> None:
        super().__init__()
        self.metadata_store = self.register_disposable(metadata_store)
        self.codec = codec
        self.blob_store = self.register_disposable(blob_store) if blob_store else None
        self.vector_store = self.register_disposable(vector_store) if vector_store else None
        self.notifier: Notifier[T] = self.register_disposable(notifier or SubjectNotifier())
        self.eager_blobs = eager_blobs

    def start(self) -> None:
        self.metadata_store.start()
        if self.blob_store is not None:
            self.blob_store.start()
        if self.vector_store is not None:
            self.vector_store.start()

    @property
    def name(self) -> str:
        return self.metadata_store.name

    def _make_loader(self, row_id: int) -> Any:
        bs = self.blob_store
        if bs is None:
            raise RuntimeError("BlobStore required but not configured")
        name, codec = self.name, self.codec

        def loader() -> Any:
            raw = bs.get(name, row_id)
            return codec.decode(raw)

        return loader

    def append(self, obs: Observation[T]) -> Observation[T]:
        # Encode payload before any locking (avoids holding locks during IO)
        encoded: bytes | None = None
        if self.blob_store is not None:
            encoded = self.codec.encode(obs._data)

        try:
            # Insert metadata, get assigned id
            row_id = self.metadata_store.insert(obs)
            obs.id = row_id

            # Store blob
            if encoded is not None:
                assert self.blob_store is not None
                self.blob_store.put(self.name, row_id, encoded)
                # Replace inline data with lazy loader
                obs._data = _UNLOADED
                obs._loader = self._make_loader(row_id)

            # Store embedding vector
            if self.vector_store is not None:
                emb = getattr(obs, "embedding", None)
                if emb is not None:
                    self.vector_store.put(self.name, row_id, emb)

            # Commit if the metadata store supports it (e.g. SqliteObservationStore)
            if hasattr(self.metadata_store, "commit"):
                self.metadata_store.commit()
        except BaseException:
            if hasattr(self.metadata_store, "rollback"):
                self.metadata_store.rollback()
            raise

        self.notifier.notify(obs)
        return obs

    def iterate(self, query: StreamQuery) -> Iterator[Observation[T]]:
        if query.search_vec is not None and query.live_buffer is not None:
            raise TypeError("Cannot combine .search() with .live() — search is a batch operation.")
        buf = query.live_buffer
        if buf is not None:
            sub = self.notifier.subscribe(buf)
            return self._iterate_live(query, buf, sub)
        return self._iterate_snapshot(query)

    def _attach_loaders(self, it: Iterator[Observation[T]]) -> Iterator[Observation[T]]:
        """Attach lazy blob loaders to observations from the metadata store."""
        if self.blob_store is None:
            yield from it
            return
        for obs in it:
            if obs._loader is None and isinstance(obs._data, type(_UNLOADED)):
                obs._loader = self._make_loader(obs.id)
            yield obs

    def _iterate_snapshot(self, query: StreamQuery) -> Iterator[Observation[T]]:
        if query.search_vec is not None and self.vector_store is not None:
            yield from self._vector_search(query)
            return

        it: Iterator[Observation[T]] = self._attach_loaders(self.metadata_store.query(query))

        # Apply python post-filters after loaders are attached (so obs.data works)
        python_filters = getattr(self.metadata_store, "_pending_python_filters", None)
        pending_query = getattr(self.metadata_store, "_pending_query", None)
        if python_filters:
            from itertools import islice as _islice

            it = (obs for obs in it if all(f.matches(obs) for f in python_filters))
            if pending_query and pending_query.offset_val:
                it = _islice(it, pending_query.offset_val, None)
            if pending_query and pending_query.limit_val is not None:
                it = _islice(it, pending_query.limit_val)

        if self.eager_blobs and self.blob_store is not None:
            for obs in it:
                _ = obs.data  # trigger lazy loader
                yield obs
        else:
            yield from it

    def _vector_search(self, query: StreamQuery) -> Iterator[Observation[T]]:
        vs = self.vector_store
        assert vs is not None and query.search_vec is not None

        hits = vs.search(self.name, query.search_vec, query.search_k or 10)
        if not hits:
            return

        ids = [h[0] for h in hits]
        obs_list = list(self._attach_loaders(iter(self.metadata_store.fetch_by_ids(ids))))
        obs_by_id = {obs.id: obs for obs in obs_list}

        # Preserve VectorStore ranking order
        ranked: list[Observation[T]] = []
        for obs_id, sim in hits:
            match = obs_by_id.get(obs_id)
            if match is not None:
                ranked.append(
                    match.derive(data=match.data, embedding=query.search_vec, similarity=sim)
                )

        # Apply remaining query ops (skip vector search)
        rest = replace(query, search_vec=None, search_k=None)
        yield from rest.apply(iter(ranked))

    def _iterate_live(
        self,
        query: StreamQuery,
        buf: BackpressureBuffer[Observation[T]],
        sub: DisposableBase,
    ) -> Iterator[Observation[T]]:
        from dimos.memory2.buffer import ClosedError

        eager = self.eager_blobs and self.blob_store is not None

        try:
            # Backfill phase
            last_id = -1
            for obs in self._iterate_snapshot(query):
                last_id = max(last_id, obs.id)
                yield obs

            # Live tail
            filters = query.filters
            while True:
                obs = buf.take()
                if obs.id <= last_id:
                    continue
                last_id = obs.id
                if filters and not all(f.matches(obs) for f in filters):
                    continue
                if eager:
                    _ = obs.data  # trigger lazy loader
                yield obs
        except (ClosedError, StopIteration):
            pass
        finally:
            sub.dispose()

    def count(self, query: StreamQuery) -> int:
        if query.search_vec:
            return sum(1 for _ in self.iterate(query))
        return self.metadata_store.count(query)

    def serialize(self) -> dict[str, Any]:
        """Serialize the fully-resolved backend config to a dict."""
        return {
            "codec_id": codec_id(self.codec),
            "eager_blobs": self.eager_blobs,
            "metadata_store": self.metadata_store.serialize()
            if hasattr(self.metadata_store, "serialize")
            else None,
            "blob_store": self.blob_store.serialize() if self.blob_store else None,
            "vector_store": self.vector_store.serialize() if self.vector_store else None,
            "notifier": self.notifier.serialize(),
        }
