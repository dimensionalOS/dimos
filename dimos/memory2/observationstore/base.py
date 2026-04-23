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

from abc import abstractmethod
from typing import TYPE_CHECKING, Any, Generic, TypeVar

from dimos.core.resource import CompositeResource
from dimos.memory2.registry import qual
from dimos.protocol.service.spec import BaseConfig, Configurable

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.type.filter import StreamQuery
    from dimos.memory2.type.observation import Observation

T = TypeVar("T")


class ObservationStoreConfig(BaseConfig):
    pass


class ObservationStore(Configurable, CompositeResource, Generic[T]):
    """Core metadata storage and query engine for observations.

    Handles only observation metadata storage, query pushdown, and count.
    Blob/vector/live orchestration is handled by the concrete Backend class.
    """

    config: ObservationStoreConfig

    def __init__(self, **kwargs: Any) -> None:
        Configurable.__init__(self, **kwargs)
        CompositeResource.__init__(self)

    @property
    @abstractmethod
    def name(self) -> str: ...

    @abstractmethod
    def insert(self, obs: Observation[T]) -> int:
        """Insert observation metadata, return assigned id."""
        ...

    @abstractmethod
    def query(self, q: StreamQuery) -> Iterator[Observation[T]]:
        """Execute query against metadata. Blobs are NOT loaded here."""
        ...

    @abstractmethod
    def count(self, q: StreamQuery) -> int: ...

    @abstractmethod
    def fetch_by_ids(self, ids: list[int]) -> list[Observation[T]]:
        """Batch fetch by id (for vector search results)."""
        ...

    def delete_range(self, t1: float, t2: float) -> list[int]:
        """Delete observations with ts in [t1, t2]. Returns deleted IDs."""
        raise NotImplementedError

    def serialize(self) -> dict[str, Any]:
        return {"class": qual(type(self)), "config": self.config.model_dump()}
