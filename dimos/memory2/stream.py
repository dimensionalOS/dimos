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

import sys
import time
from typing import TYPE_CHECKING, Any, Generic, cast, overload

if sys.version_info >= (3, 13):
    from typing import TypeVar
else:
    from typing_extensions import TypeVar

from dimos.core.resource import CompositeResource
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
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

    import reactivex
    from reactivex.abc import DisposableBase, ObserverBase

    from dimos.memory2.backend import Backend
    from dimos.models.embedding.base import Embedding

T = TypeVar("T")
R = TypeVar("R")
O = TypeVar("O", bound=Observation[Any], default=Observation[T])
logger = setup_logger()


class Stream(CompositeResource, Generic[T, O]):
    """Lazy, pull-based stream over observations.

    Every filter/transform method returns a new Stream — no computation
    happens until iteration. Backends handle query application for stored
    data; transform sources apply filters as Python predicates.

    Implements CompositeResource so subscriptions created via ``.subscribe()``
    are tracked and disposed on ``stop()``.

    An *unbound* stream (``Stream()``) records a chain of transforms
    without a real source. Use ``.chain()`` to apply it to a bound stream::

        pipeline = Stream().transform(VoxelMapTransformer()).map(postprocess)
        store.stream("lidar", PointCloud2).live().chain(pipeline)
    """

    def __init__(
        self,
        source: Backend[T] | Stream[Any, Any] | None = None,
        *,
        transform: Transformer[Any, T] | None = None,
        query: StreamQuery = StreamQuery(),
    ) -> None:
        super().__init__()
        self._source = source
        if source is not None:
            self.register_disposable(source)
        self._transform = transform
        self._query = query

    def stop(self) -> None:
        buf = self._query.live_buffer
        if buf is not None:
            buf.close()
        super().stop()

    def __str__(self) -> str:
        # Walk the source chain to collect (xf, query) pairs
        chain: list[tuple[Any, StreamQuery]] = []
        current: Any = self
        while isinstance(current, Stream):
            chain.append((current._transform, current._query))
            current = current._source
        chain.reverse()  # innermost first

        # current is the Backend (or None for unbound)
        if current is None:
            result = "Stream(unbound)"
        else:
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

    @property
    def data_type(self) -> type:
        """Payload type of the root backend, or ``object`` if unbound."""
        current: Any = self
        while isinstance(current._source, Stream):
            current = current._source
        if current._source is None:
            return object
        return cast("type", current._source.data_type)

    def __iter__(self) -> Iterator[O]:
        if self._source is None:
            raise TypeError(
                "Cannot iterate an unbound stream. Use .chain() to apply it to a real stream first."
            )
        if isinstance(self._source, Stream):
            return self._iter_transform()
        # Backend handles all query application (including live if requested)
        return cast("Iterator[O]", self._source.iterate(self._query))

    def _iter_transform(self) -> Iterator[O]:
        """Iterate a transform source, applying query filters in Python."""
        assert isinstance(self._source, Stream) and self._transform is not None
        it = self._transform(iter(self._source))
        return cast("Iterator[O]", self._query.apply(it, live=self.is_live()))

    def _replace_query(self, **overrides: Any) -> Stream[T, O]:
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
        return Stream(self._source, transform=self._transform, query=new_q)

    def _with_filter(self, f: Filter) -> Stream[T, O]:
        return self._replace_query(filters=(*self._query.filters, f))

    def after(self, t: float) -> Stream[T, O]:
        return self._with_filter(AfterFilter(t))

    def before(self, t: float) -> Stream[T, O]:
        return self._with_filter(BeforeFilter(t))

    def time_range(self, t1: float, t2: float) -> Stream[T, O]:
        return self._with_filter(TimeRangeFilter(t1, t2))

    def at(self, t: float, tolerance: float = 1.0) -> Stream[T, O]:
        return self._with_filter(AtFilter(t, tolerance))

    def at_relative(self, t: float, tolerance: float = 1.0) -> Stream[T, O]:
        """Like `at` but ``t`` is seconds from the first observation."""
        t0 = self.first().ts
        return self.at(t0 + t, tolerance=tolerance)

    def near(self, pose: Any, radius: float) -> Stream[T, O]:
        return self._with_filter(NearFilter(pose, radius))

    def tags(self, **tags: Any) -> Stream[T, O]:
        return self._with_filter(TagsFilter(tags))

    def order_by(self, field: str, desc: bool = False) -> Stream[T, O]:
        return self._replace_query(order_field=field, order_desc=desc)

    def limit(self, k: int) -> Stream[T, O]:
        return self._replace_query(limit_val=k)

    def offset(self, n: int) -> Stream[T, O]:
        return self._replace_query(offset_val=n)

    def search(self, query: Embedding, k: int | None = None) -> Stream[T, EmbeddedObservation[T]]:
        """Rank observations by cosine similarity to *query*.

        Returns a stream whose observations are :class:`EmbeddedObservation`
        with ``.similarity`` populated.

        If *k* is omitted, unbounded backends return all scored hits and
        bounded backends (e.g. sqlite-vec) apply their own default cap.
        """
        new_q = StreamQuery(
            filters=self._query.filters,
            order_field=self._query.order_field,
            order_desc=self._query.order_desc,
            limit_val=self._query.limit_val,
            offset_val=self._query.offset_val,
            live_buffer=self._query.live_buffer,
            search_vec=query,
            search_k=k,
            search_text=self._query.search_text,
        )
        return Stream(self._source, transform=self._transform, query=new_q)

    def search_text(self, text: str) -> Stream[T, O]:
        """Filter observations whose data contains *text*.

        ListObservationStore does case-insensitive substring match;
        SqliteObservationStore (future) pushes down to FTS5.
        """
        return self._replace_query(search_text=text)

    def filter(self, pred: Callable[[O], bool]) -> Stream[T, O]:
        """Filter by arbitrary predicate on the full Observation."""
        return self._with_filter(PredicateFilter(cast("Callable[[Observation[Any]], bool]", pred)))

    def tap(self, fn: Callable[[O], Any]) -> Stream[T, O]:
        """Call *fn* on each observation without changing it."""

        def _tap(upstream: Iterator[Observation[T]]) -> Iterator[Observation[T]]:
            for obs in upstream:
                fn(cast("O", obs))
                yield obs

        return cast("Stream[T, O]", self.transform(FnIterTransformer(_tap)))

    def scan_data(self, state: Any, fn: Callable[[Any, O], tuple[Any, R]]) -> Stream[R]:
        """Stateful map: ``fn(state, obs) -> (new_state, new_data)``.

        Each observation is yielded with ``.data`` replaced by ``new_data``.
        """

        def _scan(upstream: Iterator[Observation[T]]) -> Iterator[Observation[R]]:
            s = state
            for obs in upstream:
                s, val = fn(s, cast("O", obs))
                yield obs.derive(data=val)

        return self.transform(FnIterTransformer(_scan))

    def map(self, fn: Callable[[O], Observation[R]]) -> Stream[R]:
        """Map each observation to a new observation (possibly of a new data type)."""
        return self.transform(FnTransformer(lambda obs: fn(cast("O", obs))))

    def map_data(self, fn: Callable[[O], R]) -> Stream[R]:
        """Transform each observation's data via callable."""
        return self.transform(FnTransformer(lambda obs: obs.derive(data=fn(cast("O", obs)))))

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
        return Stream(source=self, transform=xf, query=StreamQuery())

    def live(self, buffer: BackpressureBuffer[Observation[Any]] | None = None) -> Stream[T, O]:
        """Return a stream whose iteration never ends — backfill then live tail.

        All backends support live mode via their built-in ``Notifier``.
        Call .live() before .transform(), not after.

        Default buffer: KeepLast(). The backend handles subscription, dedup,
        and backpressure — how it does so is its business.
        """
        if isinstance(self._source, Stream) or self._source is None:
            raise TypeError(
                "Cannot call .live() on a transform/unbound stream. "
                "Call .live() on the source stream, then .transform()."
            )
        buf = buffer if buffer is not None else KeepLast()
        return self._replace_query(live_buffer=buf)

    def save(self, target: Stream[T, O]) -> Stream[T, O]:
        """Lazy pass-through that appends each observation to *target*'s backend.

        Iteration drives both the passthrough and the appends — pick a terminal
        (``.drain()`` sync, ``.drain_thread()`` background, ``.to_list()``,
        ``for obs in ...``).
        """
        if isinstance(target._source, Stream) or target._source is None:
            raise TypeError(
                "Cannot save to a transform/unbound stream. Target must be backend-backed."
            )
        backend = target._source

        def _save(upstream: Iterator[Observation[T]]) -> Iterator[Observation[T]]:
            for obs in upstream:
                backend.append(obs)
                yield obs

        return cast("Stream[T, O]", self.transform(FnIterTransformer(_save)))

    def to_list(self) -> list[O]:
        """Materialize all observations into a list."""
        if self.is_live():
            raise TypeError(
                ".to_list() on a live stream would block forever. "
                "Use .drain() or .save(target) instead."
            )
        return list(self)

    def first(self) -> O:
        """Return the first matching observation."""
        it = iter(self.limit(1))
        try:
            return next(it)
        except StopIteration:
            raise LookupError("No matching observation") from None

    def last(self) -> O:
        """Return the last matching observation (by timestamp)."""
        return self.order_by("ts", desc=True).first()

    def count(self) -> int:
        """Count matching observations."""
        if self._source is not None and not isinstance(self._source, Stream):
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

    def materialize(self) -> Stream[T, O]:
        """Materialize into memory and return a replayable stream.

        Useful when you need to iterate the same results multiple times
        without re-running the upstream query.
        """
        from dimos.memory2.store.memory import MemoryStore

        mem = MemoryStore()
        target = cast("Stream[T, O]", mem.stream("materialize"))
        self.save(target).drain()
        return target

    def drain(self) -> int:
        """Consume all observations, discarding results. Returns count consumed.

        Use for side-effect pipelines (e.g. live embed-and-store) where you
        don't need to collect results in memory.
        """
        n = 0
        for _ in self:
            n += 1
        return n

    def drain_thread(self) -> DisposableBase:
        """Drain this stream on the dimos thread pool; returns a disposable."""
        return self.subscribe(lambda _: None)

    def observable(self) -> reactivex.Observable[O]:
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
        on_next: Callable[[O], None] | ObserverBase[O] | None = None,
        on_error: Callable[[Exception], None] | None = None,
        on_completed: Callable[[], None] | None = None,
    ) -> DisposableBase:
        """Subscribe to this stream as an RxPY Observable.

        The subscription is tracked and disposed when this stream is stopped.
        """
        return self.register_disposable(
            self.observable().subscribe(  # type: ignore[call-overload]
                on_next=on_next,
                on_error=on_error,
                on_completed=on_completed,
            )
        )

    def delete_range(self, t1: float, t2: float) -> int:
        """Delete all observations with timestamps in [t1, t2]. Returns count deleted."""
        if isinstance(self._source, Stream):
            raise TypeError("Cannot delete from a transform stream.")
        if self._source is None:
            raise TypeError("No source available.")
        return self._source.delete_range(t1, t2)

    def publish(self, out: Any) -> DisposableBase:
        """Publish each observation's data to a Module ``Out`` port.

        Iteration runs on the dimos thread pool (via :meth:`subscribe`).
        Returns a ``DisposableBase`` suitable for ``register_disposable()``.

        Example::

            lidar.live().transform(VoxelMapTransformer()).publish(self.global_map)
        """

        def _on_error(e: Exception) -> None:
            logger.error("Stream.publish() pipeline error: %s", e, exc_info=True)

        return self.subscribe(
            on_next=lambda obs: out.publish(obs.data),
            on_error=_on_error,
        )

    def chain(self, other: Stream[R, Any]) -> Stream[R]:
        """Append operations from an unbound stream to this stream.

        Extracts the transform/filter chain from *other* (which must be
        unbound) and replays it on top of ``self``::

            pipeline = Stream().transform(VoxelMapTransformer()).map(postprocess)
            store.stream("lidar").live().chain(pipeline)
        """
        ops: list[tuple[Transformer[Any, Any] | None, StreamQuery]] = []
        current: Stream[Any, Any] | None | Any = other
        found_root = False
        while isinstance(current, Stream):
            ops.append((current._transform, current._query))
            if current._source is None:
                found_root = True
                break
            current = current._source
        if not found_root:
            raise TypeError("Can only chain an unbound stream (created with Stream())")

        # Validate no unsupported query fields in the unbound chain
        for _, query in ops:
            if query.search_vec is not None or query.search_text is not None:
                raise TypeError("search() / search_text() cannot be used on unbound streams")
            if query.live_buffer is not None:
                raise TypeError("live() cannot be used on unbound streams")

        result: Stream[Any, Any] = self
        for xf, query in reversed(ops):
            if xf is not None:
                result = result.transform(xf)
            for f in query.filters:
                result = result._with_filter(f)
            if query.limit_val is not None:
                result = result.limit(query.limit_val)
            if query.offset_val is not None and query.offset_val != 0:
                result = result.offset(query.offset_val)
            if query.order_field is not None:
                result = result.order_by(query.order_field, desc=query.order_desc)
        return cast("Stream[R]", result)

    @overload
    def append(
        self,
        payload: T,
        *,
        ts: float | None = ...,
        pose: Any | None = ...,
        tags: dict[str, Any] | None = ...,
        embedding: None = None,
    ) -> Observation[T]: ...
    @overload
    def append(
        self,
        payload: T,
        *,
        ts: float | None = ...,
        pose: Any | None = ...,
        tags: dict[str, Any] | None = ...,
        embedding: Embedding,
    ) -> EmbeddedObservation[T]: ...
    def append(
        self,
        payload: T,
        *,
        ts: float | None = None,
        pose: Any | None = None,
        tags: dict[str, Any] | None = None,
        embedding: Embedding | None = None,
    ) -> Observation[T]:
        """Append to the backing store. Only works if source is a Backend.

        Returns :class:`EmbeddedObservation` when *embedding* is provided,
        else a plain :class:`Observation`.
        """
        if isinstance(self._source, Stream) or self._source is None:
            raise TypeError(
                "Cannot append to a transform/unbound stream. Append to the source stream."
            )
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
