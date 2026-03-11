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

import threading
from typing import TYPE_CHECKING, Any, Generic, TypeVar

from dimos.memory2.backend import BackendConfig
from dimos.memory2.livechannel.subject import SubjectChannel
from dimos.memory2.store import Session, Store
from dimos.protocol.service.spec import Configurable

if TYPE_CHECKING:
    from collections.abc import Iterator

    from reactivex.abc import DisposableBase

    from dimos.memory2.backend import Backend, LiveChannel
    from dimos.memory2.buffer import BackpressureBuffer
    from dimos.memory2.filter import StreamQuery
    from dimos.memory2.type import Observation

T = TypeVar("T")


class ListBackend(Configurable[BackendConfig], Generic[T]):
    """In-memory backend for experimentation. Thread-safe."""

    default_config: type[BackendConfig] = BackendConfig

    def __init__(self, name: str = "<memory>", **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._name = name
        self._observations: list[Observation[T]] = []
        self._next_id = 0
        self._lock = threading.Lock()
        self._channel: LiveChannel[T] = self.config.live_channel or SubjectChannel()

    @property
    def name(self) -> str:
        return self._name

    @property
    def live_channel(self) -> LiveChannel[T]:
        return self._channel

    def append(self, obs: Observation[T]) -> Observation[T]:
        with self._lock:
            obs.id = self._next_id
            self._next_id += 1
            self._observations.append(obs)

        self._channel.notify(obs)
        return obs

    def iterate(self, query: StreamQuery) -> Iterator[Observation[T]]:
        """Snapshot + apply all filters/ordering/offset/limit in Python.

        If query.live_buffer is set, subscribes before backfill, then
        switches to a live tail that blocks for new observations.
        """
        if query.search_vec is not None and query.live_buffer is not None:
            raise TypeError("Cannot combine .search() with .live() — search is a batch operation.")
        buf = query.live_buffer
        if buf is not None:
            # Subscribe BEFORE backfill to avoid missing items
            sub = self._channel.subscribe(buf)
            return self._iterate_live(query, buf, sub)
        return self._iterate_snapshot(query)

    def _iterate_snapshot(self, query: StreamQuery) -> Iterator[Observation[T]]:
        with self._lock:
            snapshot = list(self._observations)
        yield from query.apply(iter(snapshot))

    def _iterate_live(
        self,
        query: StreamQuery,
        buf: BackpressureBuffer[Observation[T]],
        sub: DisposableBase,
    ) -> Iterator[Observation[T]]:
        from dimos.memory2.buffer import ClosedError

        # Backfill phase — use snapshot query (without live) for the backfill
        last_id = -1
        for obs in self._iterate_snapshot(query):
            last_id = max(last_id, obs.id)
            yield obs

        # Live tail
        filters = query.filters
        try:
            while True:
                obs = buf.take()
                if obs.id <= last_id:
                    continue
                last_id = obs.id
                if filters and not all(f.matches(obs) for f in filters):
                    continue
                yield obs
        except (ClosedError, StopIteration):
            sub.dispose()

    def count(self, query: StreamQuery) -> int:
        return sum(1 for _ in self.iterate(query))


class MemorySession(Session):
    """In-memory session. Each stream is backed by a ListBackend."""

    def _create_backend(
        self, name: str, payload_type: type[Any] | None = None, **config: Any
    ) -> Backend[Any]:
        return ListBackend(name, **config)

    def list_streams(self) -> list[str]:
        return list(self._streams.keys())

    def delete_stream(self, name: str) -> None:
        self._streams.pop(name, None)


class MemoryStore(Store):
    """In-memory store for experimentation."""

    def session(self, **kwargs: Any) -> MemorySession:
        return MemorySession(**kwargs)
