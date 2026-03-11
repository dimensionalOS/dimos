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
from typing import TYPE_CHECKING, Any, Protocol, TypeVar, runtime_checkable

from dimos.core.resource import Resource

if TYPE_CHECKING:
    from collections.abc import Iterator

    from reactivex.abc import DisposableBase

    from dimos.memory2.buffer import BackpressureBuffer
    from dimos.memory2.filter import StreamQuery
    from dimos.memory2.type import Observation

T = TypeVar("T")


@runtime_checkable
class Backend(Protocol[T]):
    """Data source protocol for stored observations.

    The backend is fully responsible for applying query filters.
    How it does so (SQL, R-tree, Python predicates) is its business.
    """

    @property
    def name(self) -> str: ...

    def iterate(self, query: StreamQuery) -> Iterator[Observation[T]]: ...

    def append(
        self,
        payload: T,
        *,
        ts: float | None = None,
        pose: Any | None = None,
        tags: dict[str, Any] | None = None,
    ) -> Observation[T]: ...

    def count(self, query: StreamQuery) -> int: ...


@runtime_checkable
class LiveBackend(Backend[T], Protocol[T]):
    """Backend that also supports live subscriptions."""

    def subscribe(self, buf: BackpressureBuffer[Observation[T]]) -> DisposableBase: ...


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
