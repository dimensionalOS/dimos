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

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Generic, Protocol, TypeVar, runtime_checkable

from dimos.core.resource import Resource

if TYPE_CHECKING:
    from collections.abc import Iterator

    from reactivex.abc import DisposableBase

    from dimos.memory2.buffer import BackpressureBuffer
    from dimos.memory2.filter import StreamQuery
    from dimos.memory2.type import Observation
    from dimos.models.embedding.base import Embedding

T = TypeVar("T")


# ── Backend configuration ───────────────────────────────────────


@dataclass
class BackendConfig:
    """Configuration for backend capabilities.

    Session-level defaults are merged with per-stream overrides and
    forwarded here by ``Session.stream()``.
    """

    live_channel: LiveChannel[Any] | None = None
    blob_store: BlobStore | None = None
    vector_store: VectorStore | None = None


@runtime_checkable
class Backend(Protocol[T]):
    """Data source protocol for stored observations.

    The backend is fully responsible for applying query filters.
    How it does so (SQL, R-tree, Python predicates) is its business.

    Every backend supports live mode via a built-in ``LiveChannel``.
    """

    @property
    def name(self) -> str: ...

    @property
    def live_channel(self) -> LiveChannel[T]: ...

    def iterate(self, query: StreamQuery) -> Iterator[Observation[T]]: ...

    def append(self, obs: Observation[T]) -> Observation[T]:
        """Store an observation, assigning it a backend-managed id.

        The caller builds the Observation (or EmbeddedObservation);
        the backend assigns the canonical ``id`` and persists it.
        """
        ...

    def count(self, query: StreamQuery) -> int: ...


# ── Live notification channel ────────────────────────────────────


class LiveChannel(ABC, Generic[T]):
    """Push-notification channel for live observation delivery.

    Decouples the notification mechanism from storage.  The built-in
    ``SubjectChannel`` handles same-session fan-out (thread-safe, zero
    config).  External implementations (Redis pub/sub, Postgres
    LISTEN/NOTIFY, inotify) can be injected for cross-process use.
    """

    @abstractmethod
    def subscribe(self, buf: BackpressureBuffer[Observation[T]]) -> DisposableBase:
        """Register *buf* to receive new observations. Returns a
        disposable that unsubscribes when disposed."""
        ...

    @abstractmethod
    def notify(self, obs: Observation[T]) -> None:
        """Fan out *obs* to all current subscribers."""
        ...


# ── Blob storage ──────────────────────────────────────────────────


class BlobStore(Resource, ABC):
    """Persistent storage for encoded payload blobs.

    Separates payload data from metadata indexing so that large blobs
    (images, point clouds) don't penalize metadata queries.

    Extends Resource (start/stop) but does NOT manage its dependencies'
    lifecycle — the caller owns the session / connection.
    """

    @abstractmethod
    def put(self, stream: str, key: int, data: bytes) -> None:
        """Store a blob for the given stream and observation id."""
        ...

    @abstractmethod
    def get(self, stream: str, key: int) -> bytes:
        """Retrieve a blob by stream name and observation id."""
        ...

    @abstractmethod
    def delete(self, stream: str, key: int) -> None:
        """Delete a blob by stream name and observation id."""
        ...


# ── Vector storage ───────────────────────────────────────────────


class VectorStore(Resource, ABC):
    """Pluggable storage and ANN index for embedding vectors.

    Separates vector indexing from metadata so backends can swap
    search strategies (brute-force, vec0, FAISS, Qdrant) independently.

    Same shape as BlobStore: ``put`` / ``search`` / ``delete``, keyed
    by ``(stream, observation_id)``.  Index creation is lazy — the
    first ``put`` for a stream determines dimensionality.
    """

    @abstractmethod
    def put(self, stream: str, key: int, embedding: Embedding) -> None:
        """Store an embedding vector for the given stream and observation id."""
        ...

    @abstractmethod
    def search(self, stream: str, query: Embedding, k: int) -> list[tuple[int, float]]:
        """Return top-k (observation_id, similarity) pairs, descending."""
        ...

    @abstractmethod
    def delete(self, stream: str, key: int) -> None:
        """Remove a vector. Silent if missing."""
        ...
