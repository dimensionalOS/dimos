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

"""Multi-stream replay over a memory2 :class:`Store` with a shared anchor.

``store.replay()`` returns a :class:`Replay` view. The first ``.observable()``
subscribe across all streams pins a single ``(wall_t0, replay_t0)`` anchor;
subsequent subscribers schedule emissions against the same anchor, so
``replay.streams.lidar.observable()`` and ``replay.streams.odom.observable()``
advance together. Late subscribers skip past data already behind wall time.
"""

from __future__ import annotations

from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING, Any, Generic, TypeVar, cast

import reactivex as rx
from reactivex.abc import DisposableBase, ObserverBase, SchedulerBase
from reactivex.disposable import Disposable
from reactivex.observable import Observable
from reactivex.scheduler import TimeoutScheduler

from dimos.memory2.store.base import Store, StreamAccessor
from dimos.protocol.service.spec import BaseConfig, Configurable
from dimos.utils.data import get_data

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

    from dimos.memory2.stream import Stream

T = TypeVar("T")

_LOOP_GAP = 0.05  # min wall-time gap inserted between loop wraps (seconds)
_LATE_TOLERANCE = 0.05  # don't skip frames within this many seconds of "now"


def resolve_db_path(dataset: str | Path) -> Path:
    """Map a dataset name to an on-disk .db path (LFS-downloading on miss).

    Absolute or existing relative paths are used as-is; bare names resolve
    via :func:`dimos.utils.data.get_data`.
    """
    p = Path(dataset)
    if p.is_absolute() or p.exists():
        return p
    return get_data(f"{dataset}.db")


class ReplayConfig(BaseConfig):
    speed: float = 1.0
    seek: float | None = None
    duration: float | None = None
    from_timestamp: float | None = None
    loop: bool = False


class Replay(Configurable):
    """Time-bounded view over a :class:`Store` with a shared replay anchor.

    Constructed via :meth:`Store.replay`. Pass ``speed``, ``seek``,
    ``duration``, ``from_timestamp``, ``loop`` to control playback.
    """

    config: ReplayConfig

    def __init__(self, store: Store, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.store = store
        self._anchor: tuple[float, float] | None = None
        self._anchor_lock = threading.Lock()

    @property
    def streams(self) -> StreamAccessor[ReplayStream[Any]]:
        return StreamAccessor(self)

    def list_streams(self) -> list[str]:
        return self.store.list_streams()

    def stream(self, name: str, autocast: Callable[[Any], T] | None = None) -> ReplayStream[T]:
        return ReplayStream(replay=self, name=name, autocast=autocast)

    def first_ts(self) -> float | None:
        """Earliest first_ts across non-empty streams in the underlying store."""
        candidates: list[float] = []
        for name in self.store.list_streams():
            try:
                candidates.append(float(self.store.stream(name).first().ts))
            except LookupError:
                continue
        return min(candidates) if candidates else None

    def _resolve_anchor(self, candidate_first_ts: float) -> tuple[float, float]:
        """Pin (wall_t0, replay_t0) on first call; return shared anchor."""
        with self._anchor_lock:
            if self._anchor is None:
                self._anchor = (time.time(), candidate_first_ts)
            return self._anchor

    def reset_anchor(self) -> None:
        """Forget the pinned anchor. Next ``.observable()`` re-pins it."""
        with self._anchor_lock:
            self._anchor = None


class ReplayStream(Generic[T]):
    """One stream view inside a :class:`Replay`.

    Owns the seek/duration window and provides a timed ``.observable()``
    scheduled against the parent :class:`Replay`'s shared anchor.
    """

    def __init__(
        self,
        *,
        replay: Replay,
        name: str,
        autocast: Callable[[Any], T] | None = None,
    ) -> None:
        self._replay = replay
        self._name = name
        self._autocast = autocast

    @property
    def name(self) -> str:
        return self._name

    def _decode(self, obs: Any) -> T:
        data = obs.data
        if self._autocast is not None:
            data = self._autocast(data)
        return cast("T", data)

    def _base_stream(self) -> Stream[Any]:
        """Memory2 Stream bounded by the replay window, ordered by ts."""
        cfg = self._replay.config
        s: Stream[Any] = self._replay.store.stream(self._name)

        start: float | None = None
        if cfg.from_timestamp is not None:
            start = cfg.from_timestamp
        elif cfg.seek is not None:
            recording_first = self._replay.first_ts()
            if recording_first is None:
                return s.order_by("ts")
            start = recording_first + cfg.seek

        end: float | None = None
        if cfg.duration is not None:
            if start is not None:
                end = start + cfg.duration
            else:
                recording_first = self._replay.first_ts()
                if recording_first is None:
                    return s.order_by("ts")
                end = recording_first + cfg.duration

        if start is not None and end is not None:
            bound = s.time_range(start, end)
        elif start is not None:
            bound = s.time_range(start, float("inf"))
        elif end is not None:
            bound = s.before(end)
        else:
            bound = s

        return bound.order_by("ts")

    def first_ts(self) -> float | None:
        """First ts within the replay window (post seek/duration)."""
        try:
            return float(self._base_stream().first().ts)
        except LookupError:
            return None

    def count(self) -> int:
        return int(self._base_stream().count())

    def iterate_ts(self) -> Iterator[tuple[float, T]]:
        """Yield ``(ts, data)`` within the replay window. Honors ``loop``."""
        while True:
            emitted = False
            obs: Any
            for obs in self._base_stream():
                emitted = True
                yield (obs.ts, self._decode(obs))
            if not self._replay.config.loop or not emitted:
                break

    def iterate(self) -> Iterator[T]:
        for _, data in self.iterate_ts():
            yield data

    def first(self) -> T | None:
        try:
            return self._decode(self._base_stream().first())
        except LookupError:
            return None

    def find_closest(self, timestamp: float, tolerance: float = 1.0) -> T | None:
        s: Stream[Any] = self._replay.store.stream(self._name)
        try:
            obs: Any = s.at(timestamp, tolerance).first()
        except LookupError:
            return None
        return self._decode(obs)

    def observable(self) -> Observable[T]:
        """Timed Observable scheduled against the Replay's shared anchor.

        The first subscribe across the whole :class:`Replay` pins
        ``(wall_t0, replay_t0)``. Late subscribers compute their entry from
        the same anchor and skip past frames already behind wall time.
        Adapted from the legacy ``timed_playback`` (which pinned a fresh
        anchor per subscribe).
        """
        replay = self._replay
        speed = replay.config.speed
        loop = replay.config.loop
        decode = self._decode
        base = self._base_stream

        def subscribe(
            observer: ObserverBase[T],
            scheduler: SchedulerBase | None = None,
        ) -> DisposableBase:
            sched = scheduler or TimeoutScheduler()
            is_disposed = False

            def make_iterator() -> Iterator[tuple[float, T]]:
                while True:
                    emitted = False
                    obs: Any
                    for obs in base():
                        emitted = True
                        yield (obs.ts, decode(obs))
                    if not loop or not emitted:
                        break

            iterator = make_iterator()

            try:
                first_ts, first_data = next(iterator)
            except StopIteration:
                observer.on_completed()
                return Disposable()

            wall_t0, replay_t0 = replay._resolve_anchor(first_ts)
            now_replay = replay_t0 + (time.time() - wall_t0) * speed

            # Late-subscribe skip: drop frames behind the wall clock. Stop
            # if the iterator wraps (ts < prev) — that means we've exhausted
            # one full pass without finding a forward frame.
            wrap_offset = 0.0
            prev_skip = first_ts
            while first_ts < now_replay - _LATE_TOLERANCE:
                try:
                    cand_ts, cand_data = next(iterator)
                except StopIteration:
                    observer.on_completed()
                    return Disposable()
                if cand_ts < prev_skip:
                    wrap_offset += (prev_skip - cand_ts) + _LOOP_GAP
                    first_ts, first_data = cand_ts, cand_data
                    break
                first_ts, first_data = cand_ts, cand_data
                prev_skip = cand_ts

            prev_ts = first_ts

            def schedule(message: tuple[float, T], wrap_off: float, prev: float) -> None:
                ts, data = message
                if ts < prev:
                    wrap_off += (prev - ts) + _LOOP_GAP
                target = wall_t0 + ((ts + wrap_off) - replay_t0) / speed
                delay = max(0.0, target - time.time())

                def emit(_s: SchedulerBase, _state: object) -> DisposableBase | None:
                    nonlocal wrap_offset, prev_ts
                    if is_disposed:
                        return None
                    observer.on_next(data)
                    try:
                        nxt = next(iterator)
                    except StopIteration:
                        observer.on_completed()
                        return None
                    wrap_offset = wrap_off
                    prev_ts = ts
                    schedule(nxt, wrap_offset, prev_ts)
                    return None

                sched.schedule_relative(delay, emit)

            schedule((first_ts, first_data), wrap_offset, prev_ts)

            def dispose() -> None:
                nonlocal is_disposed
                is_disposed = True

            return Disposable(dispose)

        return rx.create(subscribe)
