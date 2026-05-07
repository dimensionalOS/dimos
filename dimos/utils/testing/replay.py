# Copyright 2025-2026 Dimensional Inc.
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

"""Shim layer exposing the legacy ``TimedSensorReplay`` API over memory2.

``TimedSensorReplay(name, autocast=...)`` opens the memory2 SQLite database at
``{get_data_dir}/{dataset}.db`` and reads the named stream. ``name`` is expected
to be ``"<dataset>/<stream>"``.

Callers that still need to read from legacy pickle dirs should import
``LegacyPickleStore`` directly from ``dimos.memory.timeseries.legacy``. The
write-side (``TimedSensorStorage``/``SensorStorage``) still points at
``LegacyPickleStore`` — out of scope for the memory2 migration.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from pathlib import Path
import threading
import time
from typing import Any, Generic, TypeVar, cast

import reactivex as rx
from reactivex.abc import DisposableBase, ObserverBase, SchedulerBase
from reactivex.disposable import Disposable
from reactivex.observable import Observable

from dimos.memory.timeseries.legacy import LegacyPickleStore
from dimos.memory2.store.sqlite import SqliteStore
from dimos.utils.data import get_data

T = TypeVar("T")


# Shared SqliteStore per .db path — ReplayConnection opens three adapters
# (lidar, odom, color_image) for the same dataset, so sharing a connection
# avoids redundant opens.
_stores: dict[str, SqliteStore] = {}


def _resolve_db_path(dataset: str) -> Path:
    """Map a dataset name to an on-disk .db path (LFS-downloading on miss).

    - ``"go2_bigoffice"`` → ``{data_dir}/go2_bigoffice.db``
    - Absolute/relative paths are used as-is.
    """
    p = Path(dataset)
    if p.is_absolute() or p.exists():
        return p
    return get_data(f"{dataset}.db")


def _get_store(dataset: str) -> SqliteStore:
    db_path = _resolve_db_path(dataset)
    key = str(db_path)
    store = _stores.get(key)
    if store is None:
        store = SqliteStore(path=key, must_exist=True)
        store.start()
        _stores[key] = store
    return store


def _close_all() -> None:
    """Close every cached SqliteStore. For test teardown."""
    for store in _stores.values():
        store.stop()
    _stores.clear()


def timed_playback(
    source: Callable[[], Iterator[tuple[float, T]]],
    speed: float = 1.0,
    detect_loop: bool = True,
) -> Observable[T]:
    """Replay a ``(ts, data)`` iterator as an Observable at real-time speed.

    Anchors on the first timestamp and runs a dedicated worker thread that
    sleeps between emissions. The thread is joined on dispose (unlike
    ``TimeoutScheduler``) so test thread-leak checks do not see a stray worker.
    When ``detect_loop`` is set, a backwards-going timestamp re-anchors — use
    this when the source iterator loops.
    """

    def subscribe(
        observer: ObserverBase[T],
        _scheduler: SchedulerBase | None = None,
    ) -> DisposableBase:
        cancel_event = threading.Event()
        is_disposed = False

        def loop_action() -> None:
            try:
                iterator = source()
                try:
                    first_ts, first_data = next(iterator)
                except StopIteration:
                    observer.on_completed()
                    return

                if is_disposed:
                    return

                start_local_time = time.time()
                start_replay_time = first_ts
                observer.on_next(first_data)

                prev_ts = first_ts

                for ts, data in iterator:
                    if is_disposed:
                        return

                    if detect_loop and ts < prev_ts:
                        start_local_time = time.time()
                        start_replay_time = ts
                    prev_ts = ts

                    target_time = start_local_time + (ts - start_replay_time) / speed
                    delay = target_time - time.time()

                    if delay < -0.2:
                        # We fell significantly behind (e.g. queue blocking, computation heavy).
                        # Re-anchor time to prevent a "death spiral" where we blast 100% CPU
                        # to emit stale frames as fast as possible, which chokes queues.
                        start_local_time = time.time()
                        start_replay_time = ts
                        delay = 0.0

                    if delay > 0:
                        cancel_event.wait(delay)

                    if is_disposed:
                        return

                    observer.on_next(data)

                if not is_disposed:
                    observer.on_completed()
            except Exception as exc:
                observer.on_error(exc)

        worker = threading.Thread(
            target=loop_action,
            name="timed_playback",
            daemon=False,
        )
        worker.start()

        def dispose() -> None:
            nonlocal is_disposed
            is_disposed = True
            cancel_event.set()
            worker.join(timeout=60.0)

        return Disposable(dispose)

    return rx.create(subscribe)


class Memory2ReplayAdapter(Generic[T]):
    """Memory2-backed replacement for the legacy ``TimedSensorReplay``.

    Accepts names shaped like ``"<dataset>/<stream>"`` (e.g.
    ``"go2_bigoffice/lidar"``). ``autocast`` is applied after the codec
    decode, matching legacy behavior.
    """

    def __init__(self, name: str | Path, autocast: Callable[[Any], T] | None = None) -> None:
        parts = str(name).split("/", 1)
        if len(parts) != 2:
            raise ValueError(
                f"Expected '<dataset>/<stream>' name, got {name!r}. "
                "E.g. TimedSensorReplay('go2_bigoffice/lidar')."
            )
        self._dataset, self._stream_name = parts
        self._autocast = autocast

    @property
    def _stream(self) -> Any:
        return _get_store(self._dataset).stream(self._stream_name)

    def _decode(self, obs: Any) -> T:
        data = obs.data
        if self._autocast is not None:
            data = self._autocast(data)
        return cast("T", data)

    def iterate_ts(
        self,
        seek: float | None = None,
        duration: float | None = None,
        from_timestamp: float | None = None,
        loop: bool = False,
    ) -> Iterator[tuple[float, T]]:
        s = self._stream

        first_ts = self.first_timestamp()
        if first_ts is None:
            return

        start: float | None = None
        if from_timestamp is not None:
            start = from_timestamp
        elif seek is not None:
            start = first_ts + seek

        end: float | None = None
        if duration is not None:
            start_ts = start if start is not None else first_ts
            end = start_ts + duration

        # Time-bound stream using memory2 filters. time_range is inclusive on
        # both sides; .after is exclusive. Use time_range with +inf for the
        # inclusive-start semantics legacy callers rely on.
        if start is not None and end is not None:
            bound = s.time_range(start, end)
        elif start is not None:
            bound = s.time_range(start, float("inf"))
        elif end is not None:
            bound = s.before(end)  # no start → include from beginning
        else:
            bound = s

        while True:
            emitted = False
            for obs in bound:
                emitted = True
                yield (obs.ts, self._decode(obs))
            if not loop or not emitted:
                break

    def iterate(self) -> Iterator[T]:
        for _, data in self.iterate_ts():
            yield data

    def first_timestamp(self) -> float | None:
        try:
            return float(self._stream.first().ts)
        except LookupError:
            return None

    def first(self) -> T | None:
        try:
            return self._decode(self._stream.first())
        except LookupError:
            return None

    def find_closest(self, timestamp: float, tolerance: float = 1.0) -> T | None:
        try:
            obs = self._stream.at(timestamp, tolerance).first()
        except LookupError:
            return None
        return self._decode(obs)

    def find_closest_seek(self, seconds: float) -> T | None:
        first_ts = self.first_timestamp()
        if first_ts is None:
            return None
        try:
            obs = self._stream.time_range(first_ts + seconds, float("inf")).first()
        except LookupError:
            return None
        return self._decode(obs)

    def count(self) -> int:
        return int(self._stream.count())

    @property
    def files(self) -> list[Path]:
        """Compat stub — memory2 has no per-frame files."""
        return []

    def load_one(self, name: int | str | Path) -> tuple[float, T]:
        """Compat stub — index-based access by offset."""
        if not isinstance(name, int):
            raise TypeError(
                f"Memory2ReplayAdapter.load_one only supports integer offsets; got {name!r}"
            )
        obs = self._stream.limit(1).offset(int(name)).first()
        return (obs.ts, self._decode(obs))

    def stream(
        self,
        speed: float = 1.0,
        seek: float | None = None,
        duration: float | None = None,
        from_timestamp: float | None = None,
        loop: bool = False,
    ) -> Observable[T]:
        """Real-time scheduled playback as an RxPY Observable."""
        return timed_playback(
            lambda: self.iterate_ts(
                seek=seek, duration=duration, from_timestamp=from_timestamp, loop=loop
            ),
            speed=speed,
        )


TimedSensorReplay = Memory2ReplayAdapter

# Write-side + non-timed read-side stay on legacy pickle.
SensorReplay = LegacyPickleStore
SensorStorage = LegacyPickleStore
TimedSensorStorage = LegacyPickleStore
