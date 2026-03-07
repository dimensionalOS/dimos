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

from collections.abc import Callable
from dataclasses import dataclass, field
import math
from typing import TYPE_CHECKING, Any, Generic, TypeAlias, TypeVar

from dimos.models.embedding.base import Embedding

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

PoseProvider: TypeAlias = Callable[[], Any]  # () -> PoseLike | None

T = TypeVar("T")


class _Unset:
    """Sentinel indicating no data has been loaded yet."""

    __slots__ = ()


_UNSET = _Unset()


@dataclass
class Observation(Generic[T]):
    id: int
    ts: float | None = None
    pose: PoseStamped | None = None
    tags: dict[str, Any] = field(default_factory=dict)
    parent_id: int | None = field(default=None, repr=False)
    _data: T | _Unset = field(default_factory=lambda: _UNSET, repr=False)
    _data_loader: Callable[[], T] | None = field(default=None, repr=False, compare=False)

    @property
    def data(self) -> T:
        if not isinstance(self._data, _Unset):
            return self._data
        if self._data_loader is not None:
            loaded = self._data_loader()
            self._data = loaded
            return loaded
        raise LookupError("No data available; observation was not fetched with payload")


@dataclass
class EmbeddingObservation(Observation[Embedding]):
    """Returned by EmbeddingStream terminals.

    .data returns the Embedding stored in this stream.
    .embedding is a convenience alias for .data (typed as Embedding).
    .similarity is populated (0..1) when fetched via search_embedding (vec0 cosine).

    To get source data (e.g. the original Image), use .project_to(source_stream).
    """

    similarity: float | None = field(default=None, repr=True)

    @property
    def embedding(self) -> Embedding:
        return self.data


@dataclass
class StreamInfo:
    name: str
    payload_type: str | None = None
    count: int = 0
    stream_kind: str = "stream"


# ── Filter types ──────────────────────────────────────────────────────


@dataclass(frozen=True)
class AfterFilter:
    t: float

    def matches(self, obs: Observation[Any]) -> bool:
        return obs.ts is not None and obs.ts > self.t

    def __str__(self) -> str:
        return f"after(t={self.t})"


@dataclass(frozen=True)
class BeforeFilter:
    t: float

    def matches(self, obs: Observation[Any]) -> bool:
        return obs.ts is not None and obs.ts < self.t

    def __str__(self) -> str:
        return f"before(t={self.t})"


@dataclass(frozen=True)
class TimeRangeFilter:
    t1: float
    t2: float

    def matches(self, obs: Observation[Any]) -> bool:
        return obs.ts is not None and self.t1 <= obs.ts <= self.t2

    def __str__(self) -> str:
        return f"time_range({self.t1}, {self.t2})"


@dataclass(frozen=True)
class AtFilter:
    t: float
    tolerance: float

    def matches(self, obs: Observation[Any]) -> bool:
        return obs.ts is not None and abs(obs.ts - self.t) <= self.tolerance

    def __str__(self) -> str:
        return f"at(t={self.t}, tol={self.tolerance})"


@dataclass(frozen=True)
class NearFilter:
    pose: Any  # PoseLike
    radius: float

    def matches(self, obs: Observation[Any]) -> bool:
        if obs.pose is None:
            return False
        p1 = obs.pose.position
        p2 = self.pose.position
        dist = math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)
        return dist <= self.radius

    def __str__(self) -> str:
        return f"near(radius={self.radius})"


@dataclass(frozen=True)
class TagsFilter:
    tags: tuple[tuple[str, Any], ...]

    def matches(self, obs: Observation[Any]) -> bool:
        return all(obs.tags.get(k) == v for k, v in self.tags)

    def __str__(self) -> str:
        pairs = ", ".join(f"{k}={v!r}" for k, v in self.tags)
        return f"tags({pairs})"


@dataclass(frozen=True)
class EmbeddingSearchFilter:
    query: list[float]
    k: int

    def matches(self, obs: Observation[Any]) -> bool:
        return True  # top-k handled as special pass in ListBackend

    def __str__(self) -> str:
        return f"search(k={self.k})"


@dataclass(frozen=True)
class TextSearchFilter:
    text: str
    k: int | None

    def matches(self, obs: Observation[Any]) -> bool:
        return self.text.lower() in str(obs.data).lower()

    def __str__(self) -> str:
        return f"text({self.text!r})"


@dataclass(frozen=True)
class LineageFilter:
    """Filter to rows that are ancestors of observations in another stream.

    Used by ``project_to`` — compiles to a nested SQL subquery that walks the
    ``parent_id`` chain from *source_table* through *hops* to the target.
    """

    source_table: str
    source_query: StreamQuery
    hops: tuple[str, ...]  # intermediate tables between source and target

    def matches(self, obs: Observation[Any]) -> bool:
        raise NotImplementedError("LineageFilter requires a database backend")

    def __str__(self) -> str:
        hops = " -> ".join(self.hops) if self.hops else "direct"
        return f"lineage({self.source_table} -> {hops})"


Filter: TypeAlias = (
    AfterFilter
    | BeforeFilter
    | TimeRangeFilter
    | AtFilter
    | NearFilter
    | TagsFilter
    | EmbeddingSearchFilter
    | TextSearchFilter
    | LineageFilter
)


@dataclass(frozen=True)
class StreamQuery:
    """Immutable bundle of query parameters passed to backends."""

    filters: tuple[Filter, ...] = ()
    order_field: str | None = None
    order_desc: bool = False
    limit_val: int | None = None
    offset_val: int | None = None

    def __str__(self) -> str:
        parts: list[str] = [str(f) for f in self.filters]
        if self.order_field:
            direction = "desc" if self.order_desc else "asc"
            parts.append(f"order({self.order_field}, {direction})")
        if self.limit_val is not None:
            parts.append(f"limit({self.limit_val})")
        if self.offset_val is not None:
            parts.append(f"offset({self.offset_val})")
        return " | ".join(parts)
