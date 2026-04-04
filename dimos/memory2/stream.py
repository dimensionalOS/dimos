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

import time
from typing import TYPE_CHECKING, Any, Generic, TypeVar

from dimos.core.resource import Resource
from dimos.memory2.buffer import BackpressureBuffer, KeepLast
from dimos.memory2.transform import FnIterTransformer, FnTransformer, Transformer
from dimos.memory2.type.filter import (
    AfterFilter,
    AtFilter,
    BeforeFilter,
    Filter,
    NearFilter,
    PredicateFilter,
    StreamQuery,
    TagsFilter,
    TimeRangeFilter,
)
from dimos.memory2.type.observation import EmbeddedObservation, Observation

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

    import reactivex
    from reactivex.abc import DisposableBase, ObserverBase

    from dimos.memory2.backend import Backend
    from dimos.models.embedding.base import Embedding

T = TypeVar("T")
R = TypeVar("R")


class Stream(Resource, Generic[T]):
    """Lazy, pull-based stream over observations.

    Every filter/transform method returns a new Stream — no computation
    happens until iteration. Backends handle query application for stored
    data; transform sources apply filters as Python predicates.

    Implements Resource so live streams can be cleanly stopped via
    ``stop()`` or used as a context manager.
    """

    def __init__(
        self,
        source: Backend[T] | Stream[Any],
        *,
        xf: Transformer[Any, T] | None = None,
        query: StreamQuery = StreamQuery(),
    ) -> None:
        self._source = source
        self._xf = xf
        self._query = query

    def start(self) -> None:
        pass

    def stop(self) -> None:
        """Close the live buffer (if any), unblocking iteration."""
        buf = self._query.live_buffer
        if buf is not None:
            buf.close()
        if isinstance(self._source, Stream):
            self._source.stop()

    def __str__(self) -> str:
        # Walk the source chain to collect (xf, query) pairs
        chain: list[tuple[Any, StreamQuery]] = []
        current: Any = self
        while isinstance(current, Stream):
            chain.append((current._xf, current._query))
            current = current._source
        chain.reverse()  # innermost first

        # current is the Backend
        name = getattr(current, "name", "?")
        result = f'Stream("{name}")'

        for xf, query in chain:
            if xf is not None:
                result += f" -> {xf}"
            q_str = str(query)
            if q_str:
                result += f" | {q_str}"

        return result

    def is_live(self) -> bool:
        """True if this stream (or any ancestor in the chain) is in live mode."""
        if self._query.live_buffer is not None:
            return True
        if isinstance(self._source, Stream):
            return self._source.is_live()
        return False

    def __iter__(self) -> Iterator[Observation[T]]:
        return self._build_iter()

    def _build_iter(self) -> Iterator[Observation[T]]:
        if isinstance(self._source, Stream):
            return self._iter_transform()
        # Backend handles all query application (including live if requested)
        return self._source.iterate(self._query)

    def _iter_transform(self) -> Iterator[Observation[T]]:
        """Iterate a transform source, applying query filters in Python."""
        assert isinstance(self._source, Stream) and self._xf is not None
        it: Iterator[Observation[T]] = self._xf(iter(self._source))
        return self._query.apply(it, live=self.is_live())

    def _replace_query(self, **overrides: Any) -> Stream[T]:
        q = self._query
        new_q = StreamQuery(
            filters=overrides.get("filters", q.filters),
            order_field=overrides.get("order_field", q.order_field),
            order_desc=overrides.get("order_desc", q.order_desc),
            limit_val=overrides.get("limit_val", q.limit_val),
            offset_val=overrides.get("offset_val", q.offset_val),
            live_buffer=overrides.get("live_buffer", q.live_buffer),
            search_vec=overrides.get("search_vec", q.search_vec),
            search_k=overrides.get("search_k", q.search_k),
            search_text=overrides.get("search_text", q.search_text),
        )
        return Stream(self._source, xf=self._xf, query=new_q)

    def _with_filter(self, f: Filter) -> Stream[T]:
        return self._replace_query(filters=(*self._query.filters, f))

    def after(self, t: float) -> Stream[T]:
        return self._with_filter(AfterFilter(t))

    def before(self, t: float) -> Stream[T]:
        return self._with_filter(BeforeFilter(t))

    def time_range(self, t1: float, t2: float) -> Stream[T]:
        return self._with_filter(TimeRangeFilter(t1, t2))

    def at(self, t: float, tolerance: float = 1.0) -> Stream[T]:
        return self._with_filter(AtFilter(t, tolerance))

    def near(self, pose: Any, radius: float) -> Stream[T]:
        return self._with_filter(NearFilter(pose, radius))

    def tags(self, **tags: Any) -> Stream[T]:
        return self._with_filter(TagsFilter(tags))

    def order_by(self, field: str, desc: bool = False) -> Stream[T]:
        return self._replace_query(order_field=field, order_desc=desc)

    def limit(self, k: int) -> Stream[T]:
        return self._replace_query(limit_val=k)

    def offset(self, n: int) -> Stream[T]:
        return self._replace_query(offset_val=n)

    def search(self, query: Embedding, k: int) -> Stream[T]:
        """Return top-k observations by cosine similarity to *query*.

        The backend handles the actual computation. ListObservationStore does
        brute-force cosine; SqliteObservationStore pushes down to vec0.
        """
        return self._replace_query(search_vec=query, search_k=k)

    def search_text(self, text: str) -> Stream[T]:
        """Filter observations whose data contains *text*.

        ListObservationStore does case-insensitive substring match;
        SqliteObservationStore (future) pushes down to FTS5.
        """
        return self._replace_query(search_text=text)

    def filter(self, pred: Callable[[Observation[T]], bool]) -> Stream[T]:
        """Filter by arbitrary predicate on the full Observation."""
        return self._with_filter(PredicateFilter(pred))

    def map(self, fn: Callable[[Observation[T]], Observation[R]]) -> Stream[R]:
        """Transform each observation's data via callable."""
        return self.transform(FnTransformer(lambda obs: fn(obs)))

    def transform(
        self,
        xf: Transformer[T, R] | Callable[[Iterator[Observation[T]]], Iterator[Observation[R]]],
    ) -> Stream[R]:
        """Wrap this stream with a transformer. Returns a new lazy Stream.

        Accepts a ``Transformer`` subclass or a bare callable / generator
        function with the same ``Iterator[Obs] → Iterator[Obs]`` signature::

            def detect(upstream):
                for obs in upstream:
                    yield obs.derive(data=run_detector(obs.data))

            images.transform(detect).save(detections)
        """
        if not isinstance(xf, Transformer):
            xf = FnIterTransformer(xf)
        return Stream(source=self, xf=xf, query=StreamQuery())

    def live(self, buffer: BackpressureBuffer[Observation[Any]] | None = None) -> Stream[T]:
        """Return a stream whose iteration never ends — backfill then live tail.

        All backends support live mode via their built-in ``Notifier``.
        Call .live() before .transform(), not after.

        Default buffer: KeepLast(). The backend handles subscription, dedup,
        and backpressure — how it does so is its business.
        """
        if isinstance(self._source, Stream):
            raise TypeError(
                "Cannot call .live() on a transform stream. "
                "Call .live() on the source stream, then .transform()."
            )
        buf = buffer if buffer is not None else KeepLast()
        return self._replace_query(live_buffer=buf)

    def save(self, target: Stream[T]) -> Stream[T]:
        """Sync terminal: iterate self, append each obs to target's backend.

        Returns the target stream for continued querying.
        """
        if isinstance(target._source, Stream):
            raise TypeError("Cannot save to a transform stream. Target must be backend-backed.")
        backend = target._source
        for obs in self:
            backend.append(obs)
        return target

    def fetch(self) -> list[Observation[T]]:
        """Materialize all observations into a list."""
        if self.is_live():
            raise TypeError(
                ".fetch() on a live stream would block forever. "
                "Use .drain() or .save(target) instead."
            )
        return list(self)

    def first(self) -> Observation[T]:
        """Return the first matching observation."""
        it = iter(self.limit(1))
        try:
            return next(it)
        except StopIteration:
            raise LookupError("No matching observation") from None

    def last(self) -> Observation[T]:
        """Return the last matching observation (by timestamp)."""
        return self.order_by("ts", desc=True).first()

    def count(self) -> int:
        """Count matching observations."""
        if not isinstance(self._source, Stream):
            return self._source.count(self._query)
        if self.is_live():
            raise TypeError(".count() on a live transform stream would block forever.")
        return sum(1 for _ in self)

    def exists(self) -> bool:
        """Check if any matching observation exists."""
        return next(iter(self.limit(1)), None) is not None

    def get_time_range(self) -> tuple[float, float]:
        """Return (min_ts, max_ts) for matching observations."""
        first = self.first()
        last = self.last()
        return (first.ts, last.ts)

    def summary(self) -> str:
        """Return a short human-readable summary: count, time range, duration."""
        from datetime import datetime, timezone

        n = self.count()
        if n == 0:
            return f"{self}: empty"

        (t0, t1) = self.get_time_range()

        fmt = "%Y-%m-%d %H:%M:%S"
        dt0 = datetime.fromtimestamp(t0, tz=timezone.utc).strftime(fmt)
        dt1 = datetime.fromtimestamp(t1, tz=timezone.utc).strftime(fmt)
        dur = t1 - t0
        return f"{self}: {n} items, {dt0} — {dt1} ({dur:.1f}s)"

    def drain(self) -> int:
        """Consume all observations, discarding results. Returns count consumed.

        Use for side-effect pipelines (e.g. live embed-and-store) where you
        don't need to collect results in memory.
        """
        n = 0
        for _ in self:
            n += 1
        return n

    def observable(self) -> reactivex.Observable[Observation[T]]:
        """Convert this stream to an RxPY Observable.

        Iteration is scheduled on the dimos thread pool so subscribe() never
        blocks the calling thread.
        """
        import reactivex
        import reactivex.operators as ops

        from dimos.utils.threadpool import get_scheduler

        return reactivex.from_iterable(self).pipe(
            ops.subscribe_on(get_scheduler()),
        )

    def subscribe(
        self,
        on_next: Callable[[Observation[T]], None] | ObserverBase[Observation[T]] | None = None,
        on_error: Callable[[Exception], None] | None = None,
        on_completed: Callable[[], None] | None = None,
    ) -> DisposableBase:
        """Subscribe to this stream as an RxPY Observable."""
        return self.observable().subscribe(  # type: ignore[call-overload]
            on_next=on_next,
            on_error=on_error,
            on_completed=on_completed,
        )

    def append(
        self,
        payload: T,
        *,
        ts: float | None = None,
        pose: Any | None = None,
        tags: dict[str, Any] | None = None,
        embedding: Embedding | None = None,
    ) -> Observation[T]:
        """Append to the backing store. Only works if source is a Backend."""
        if isinstance(self._source, Stream):
            raise TypeError("Cannot append to a transform stream. Append to the source stream.")
        _ts = ts if ts is not None else time.time()
        _tags = tags or {}
        if embedding is not None:
            obs: Observation[T] = EmbeddedObservation(
                id=-1,
                ts=_ts,
                pose=pose,
                tags=_tags,
                _data=payload,
                embedding=embedding,
            )
        else:
            obs = Observation(id=-1, ts=_ts, pose=pose, tags=_tags, _data=payload)
        return self._source.append(obs)
