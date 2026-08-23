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

"""Multi-stream replay over a memory :class:`Store` with a shared anchor.

``store.replay()`` returns a :class:`Replay` view. The first ``.observable()``
subscribe across all streams pins a single ``(wall_t0, replay_t0)`` anchor;
subsequent subscribers schedule emissions against the same anchor, so
``replay.streams.lidar.observable()`` and ``replay.streams.odom.observable()``
advance together. Frames whose slot on that clock has already passed are
dropped — a live sensor stays on the wall clock and a loaded pipeline loses
frames, so a slow consumer sees the same thing here instead of silently
receiving an ever-older stream.
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

from dimos.memory.store.base import Store, StreamAccessor
from dimos.protocol.service.spec import BaseConfig, Configurable
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

    from dimos.memory.stream import Stream

logger = setup_logger()

T = TypeVar("T")

_LOOP_GAP = 0.05  # min wall-time gap inserted between loop wraps (seconds)
_LATE_TOLERANCE = 0.05  # don't drop frames within this many wall seconds of "now"
_DROP_LOG_INTERVAL = 5.0  # min wall seconds between dropped-frame log lines


def resolve_db_path(dataset: str | Path) -> Path:
    """Map a dataset name to an on-disk .db path (LFS-downloading on miss)."""
    return resolve_named_path(dataset, ".db")


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
        """Memory Stream bounded by the replay window, ordered by ts."""
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
        ``(wall_t0, replay_t0)``; every frame's emission slot follows from
        that anchor alone. A frame whose slot has already passed is dropped
        (and the drops logged): emitting it late instead would let a slow
        consumer or decode pull this stream ever further behind the others,
        which a live pipeline can never do. Late subscribers enter the same
        way, skipping straight to the first frame still ahead of the clock.
        """
        replay = self._replay
        speed = replay.config.speed
        loop = replay.config.loop
        decode = self._decode
        base = self._base_stream
        name = self._name

        def subscribe(
            observer: ObserverBase[T],
            scheduler: SchedulerBase | None = None,
        ) -> DisposableBase:
            is_disposed = False
            dropped = 0
            last_drop_log = 0.0
            wrap_offset = 0.0
            prev_ts: float | None = None

            # Decode is deferred to emission so dropped frames cost a db
            # fetch, not an image decode.
            def make_iterator() -> Iterator[tuple[float, Any]]:
                while True:
                    emitted = False
                    obs: Any
                    for obs in base():
                        emitted = True
                        yield (obs.ts, obs)
                    if not loop or not emitted:
                        break

            iterator = make_iterator()

            try:
                first_ts, _ = candidate = next(iterator)
            except StopIteration:
                observer.on_completed()
                return Disposable()

            wall_t0, replay_t0 = replay._resolve_anchor(first_ts)

            def target_of(ts: float) -> float:
                """Wall time this frame is due, advancing the loop-wrap offset."""
                nonlocal wrap_offset, prev_ts
                if prev_ts is not None and ts < prev_ts:
                    wrap_offset += (prev_ts - ts) + _LOOP_GAP
                prev_ts = ts
                return wall_t0 + ((ts + wrap_offset) - replay_t0) / speed

            def next_due(candidate: tuple[float, Any]) -> tuple[float, Any, float] | None:
                """First frame still ahead of the wall clock, dropping late ones."""
                nonlocal dropped, last_drop_log
                ts, raw = candidate
                target = target_of(ts)
                while target < time.time() - _LATE_TOLERANCE:
                    dropped += 1
                    if time.time() - last_drop_log > _DROP_LOG_INTERVAL:
                        last_drop_log = time.time()
                        logger.warning(
                            "replay stream %s is behind schedule, dropping late frames"
                            " (%d dropped so far)",
                            name,
                            dropped,
                        )
                    try:
                        ts, raw = next(iterator)
                    except StopIteration:
                        return None
                    target = target_of(ts)
                return (ts, raw, target)

            def finish() -> None:
                if dropped:
                    logger.info("replay stream %s dropped %d late frames", name, dropped)
                observer.on_completed()

            def run(due: tuple[float, Any, float]) -> None:
                while not is_disposed:
                    _, raw, target = due
                    # Sleep in small chunks: macOS timer coalescing multiplies a
                    # background process's sleeps severalfold, so one request for
                    # the whole delay can overshoot by hundreds of milliseconds.
                    while not is_disposed:
                        remaining = target - time.time()
                        if remaining <= 0:
                            break
                        time.sleep(min(max(remaining / 8, 0.0005), 0.5))
                    if is_disposed:
                        return
                    observer.on_next(decode(raw))
                    try:
                        candidate = next(iterator)
                    except StopIteration:
                        finish()
                        return
                    nxt = next_due(candidate)
                    if nxt is None:
                        finish()
                        return
                    due = nxt

            first_due = next_due(candidate)
            if first_due is None:
                finish()
                return Disposable()
            threading.Thread(
                target=run, args=(first_due,), name=f"replay-{name}", daemon=True
            ).start()

            def dispose() -> None:
                nonlocal is_disposed
                is_disposed = True

            return Disposable(dispose)

        return rx.create(subscribe)
