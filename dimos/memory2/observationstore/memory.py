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

from collections import deque
import threading
from typing import TYPE_CHECKING, Any, TypeVar

from dimos.memory2.observationstore.base import ObservationStore, ObservationStoreConfig

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.type.filter import StreamQuery
    from dimos.memory2.type.observation import Observation

T = TypeVar("T")


class ListObservationStoreConfig(ObservationStoreConfig):
    name: str = "<memory>"
    max_size: int | None = None


class ListObservationStore(ObservationStore[T]):
    """In-memory metadata store for experimentation. Thread-safe.

    ``max_size`` controls how many observations are retained:
    - ``None`` (default) — keep all (unbounded).
    - ``N`` — rolling window of the most recent N observations.
    - ``0`` — discard immediately (live-only, no history).
    """

    config: ListObservationStoreConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._name = self.config.name
        max_size = self.config.max_size
        self._observations: deque[Observation[T]] = deque(maxlen=max_size)
        self._next_id = 0
        self._lock = threading.Lock()

    @property
    def name(self) -> str:
        return self._name

    def insert(self, obs: Observation[T]) -> int:
        with self._lock:
            obs.id = self._next_id
            row_id = self._next_id
            self._next_id += 1
            self._observations.append(obs)
        return row_id

    def query(self, q: StreamQuery) -> Iterator[Observation[T]]:
        with self._lock:
            snapshot = list(self._observations)

        # Text search — substring match
        if q.search_text is not None:
            needle = q.search_text.lower()
            it: Iterator[Observation[T]] = (
                obs for obs in snapshot if needle in str(obs.data).lower()
            )
            return q.apply(it)

        return q.apply(iter(snapshot))

    def count(self, q: StreamQuery) -> int:
        return sum(1 for _ in self.query(q))

    def fetch_by_ids(self, ids: list[int]) -> list[Observation[T]]:
        id_set = set(ids)
        with self._lock:
            return [obs for obs in self._observations if obs.id in id_set]
