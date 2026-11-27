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

"""Multi-stream replay over a memory :class:`Store`.

``store.replay()`` returns a :class:`Replay` view. ``replay.playback()`` plays
the streams added to it as one timeline: every frame is published in recorded
order, at its recorded offset from the first frame, and none is dropped.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import heapq
from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING, Any, Generic, TypeVar, cast

from pydantic import Field
from reactivex.subject import Subject

from dimos.memory.cli.dataset import resolve_dataset
from dimos.memory.store.base import Store, StreamAccessor
from dimos.protocol.service.spec import BaseConfig, Configurable
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

    from reactivex.observable import Observable

    from dimos.memory.stream import Stream

T = TypeVar("T")

logger = setup_logger()


def resolve_db_path(dataset: str | Path) -> Path:
    """Resolve a recording, trying the exact input before appending ``.db``."""
    return resolve_dataset(dataset, suffix=".db")


class ReplayConfig(BaseConfig):
    speed: float = 1.0
    seek: float | None = None
    duration: float | None = None
    from_timestamp: float | None = None
    loop: bool = False
    # With ``loop``, recording seconds between the last frame of a pass and the first of the next.
    loop_gap: float = Field(default=0.0, ge=0.0)
    # Frames read ahead of the publisher.
    buffer_size: int = Field(default=128, gt=0)
    # Seconds the publisher may fall behind schedule before it warns.
    lag_warning: float = 0.5


class Replay(Configurable):
    """Time-bounded view over a :class:`Store`.

    Constructed via :meth:`Store.replay`. Pass ``speed``, ``seek``,
    ``duration``, ``from_timestamp``, ``loop``, ``buffer_size`` and
    ``lag_warning`` to control playback.
    """

    config: ReplayConfig

    def __init__(self, store: Store, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.store = store
        self._first_ts: float | None = None

    @property
    def streams(self) -> StreamAccessor[ReplayStream[Any]]:
        return StreamAccessor(self)

    def list_streams(self) -> list[str]:
        return self.store.list_streams()

    def stream(self, name: str, autocast: Callable[[Any], T] | None = None) -> ReplayStream[T]:
        return ReplayStream(replay=self, name=name, autocast=autocast)

    def first_ts(self) -> float | None:
        """Earliest source timestamp across non-empty streams in the underlying store.

        Found once and reused. A replay reads a finished recording, and every
        stream measures seek and duration from the same start.
        """
        if self._first_ts is None:
            candidates: list[float] = []
            for name in self.store.list_streams():
                try:
                    candidates.append(float(self.store.stream(name).order_by("ts").first().ts))
                except (LookupError, ImportError, AttributeError):
                    continue
            if candidates:
                self._first_ts = min(candidates)
        return self._first_ts

    def playback(self) -> Playback:
        """An empty chronological timeline to :meth:`Playback.add` streams to."""
        return Playback(self)


class ReplayStream(Generic[T]):
    """One stream view inside a :class:`Replay`, bounded by its seek/duration window."""

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

    def last_ts(self) -> float | None:
        try:
            return float(self._base_stream().last().ts)
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


@dataclass(frozen=True)
class _Frame:
    ts: float  # recording seconds, plus the loop passes before this one
    name: str
    value: Any


@dataclass(frozen=True)
class _Ended:
    name: str
    error: Exception | None = None


class _Buffer:
    """Bounded FIFO between the reader and the publisher that a stop wakes up."""

    def __init__(self, size: int, stopped: threading.Event) -> None:
        self._size = size
        self._items: deque[_Frame | _Ended] = deque()
        self._stopped = stopped
        self._changed = threading.Condition()

    def put(self, item: _Frame | _Ended) -> bool:
        """False once stopped, without adding *item*."""
        with self._changed:
            self._changed.wait_for(lambda: len(self._items) < self._size or self._stopped.is_set())
            if self._stopped.is_set():
                return False
            self._items.append(item)
            self._changed.notify_all()
            return True

    def get(self) -> _Frame | _Ended | None:
        """None once stopped."""
        with self._changed:
            self._changed.wait_for(lambda: self._items or self._stopped.is_set())
            if self._stopped.is_set():
                return None
            item = self._items.popleft()
            self._changed.notify_all()
            return item

    def wait_full(self, done: threading.Event) -> None:
        """Until full, *done* is set, or stopped."""
        with self._changed:
            self._changed.wait_for(
                lambda: len(self._items) >= self._size or done.is_set() or self._stopped.is_set()
            )

    def wake(self) -> None:
        with self._changed:
            self._changed.notify_all()


class Playback:
    """Plays several :class:`ReplayStream` as one chronological timeline.

    A reader thread merges the streams by timestamp, decodes each frame and
    keeps a buffer of ``buffer_size`` of them full. A publisher thread takes each frame
    in order, waits until its recorded offset from the first frame (divided by
    ``speed``) has passed since :meth:`play`, and publishes it. No frame is
    dropped: a publisher that falls behind publishes the late frames at once and
    warns when it is more than ``lag_warning`` seconds behind.

    :meth:`add` every stream and subscribe to what it returns. :meth:`load`
    returns once the buffer is full (or the recording ends), so calling it in a
    module's ``start()`` pays the first reads and decodes there, before
    :meth:`play` starts the clock. :meth:`stop` waits for an in-flight publish and closes
    every read cursor, so the store can be closed after it. Called from a
    subscriber's callback, nothing more is read or decoded once it returns.

    TODO: messages keep their recorded timestamps; shift them to playback time,
    so a replayed message is stamped like a live one.
    """

    def __init__(self, replay: Replay) -> None:
        self._replay = replay
        self._streams: dict[str, ReplayStream[Any]] = {}
        self._subjects: dict[str, Subject[Any]] = {}
        self._stopped = threading.Event()
        self._buffer = _Buffer(replay.config.buffer_size, self._stopped)
        self._failed: set[str] = set()  # streams ended by a read or decode error
        self._state_lock = threading.Lock()
        self._reader: threading.Thread | None = None
        self._read_done = threading.Event()
        self._loaded = threading.Event()
        self._publisher: threading.Thread | None = None

    def add(self, stream: ReplayStream[T]) -> Observable[T]:
        """Put *stream* on the timeline. Its frames complete when it ends (never, with ``loop``)."""
        with self._state_lock:
            if self._reader is not None:
                raise RuntimeError("Playback.add() after load()")
            if stream.name in self._streams:
                raise ValueError(f"stream {stream.name!r} is already on the timeline")
            self._streams[stream.name] = stream
            subject: Subject[T] = Subject()
            self._subjects[stream.name] = subject
            return subject

    def load(self) -> None:
        """Fill the buffer, returning once it is full. Later calls return at once."""
        if self._loaded.is_set():
            return
        with self._state_lock:
            if self._reader is None and not self._stopped.is_set():
                self._reader = threading.Thread(
                    target=self._read, name="replay-reader", daemon=True
                )
                self._reader.start()
        self._buffer.wait_full(self._read_done)
        self._loaded.set()

    def play(self) -> None:
        """Start the clock at the first frame and publish. Idempotent."""
        self.load()
        with self._state_lock:
            if self._publisher is not None or self._stopped.is_set():
                return
            self._publisher = threading.Thread(
                target=self._publish, args=(time.monotonic(),), name="replay-publisher", daemon=True
            )
            self._publisher.start()

    def stop(self) -> None:
        with self._state_lock:  # a concurrent load() or play() starts its thread first
            self._stopped.set()
            threads = (self._reader, self._publisher)
        self._buffer.wake()
        current = threading.current_thread()
        for thread in threads:
            if thread is not None and thread is not current:
                thread.join()

    def _read(self) -> None:
        try:
            # A looping stream never ends on its own: end them all once the recording runs dry.
            if self._read_passes() and self._replay.config.loop:
                for name in self._streams:
                    if not self._buffer.put(_Ended(name)):
                        return
        finally:
            self._read_done.set()
            self._buffer.wake()

    def _read_passes(self) -> bool:
        """Buffer every pass of the recording. False once stopped."""
        offset = 0.0
        while True:
            rows: dict[str, Iterator[Any]] = {}
            heap: list[tuple[float, int, str, Any]] = []
            try:
                for order, (name, stream) in enumerate(self._streams.items()):
                    if name not in self._failed and not self._open(name, stream, order, rows, heap):
                        return False
                first = last = None
                while heap:
                    ts, order, name, obs = heapq.heappop(heap)
                    first = ts if first is None else first
                    last = ts
                    try:
                        value = self._streams[name]._decode(obs)
                    except Exception as error:
                        self._close(rows.pop(name))
                        self._failed.add(name)
                        if not self._buffer.put(_Ended(name, error)):
                            return False
                        continue
                    if not self._buffer.put(_Frame(ts + offset, name, value)):
                        return False
                    if not self._advance(name, order, rows, heap):
                        return False
            finally:
                for iterator in rows.values():
                    self._close(iterator)
            if not self._replay.config.loop or first is None or last is None:
                return True
            offset += last - first + self._replay.config.loop_gap

    def _open(
        self,
        name: str,
        stream: ReplayStream[Any],
        order: int,
        rows: dict[str, Iterator[Any]],
        heap: list[tuple[float, int, str, Any]],
    ) -> bool:
        """Push *name*'s first row. False once stopped."""
        try:
            rows[name] = iter(stream._base_stream())
        except Exception as error:
            self._failed.add(name)
            return self._buffer.put(_Ended(name, error))
        return self._advance(name, order, rows, heap)

    def _advance(
        self,
        name: str,
        order: int,
        rows: dict[str, Iterator[Any]],
        heap: list[tuple[float, int, str, Any]],
    ) -> bool:
        """Push *name*'s next row, or end it. False once stopped."""
        try:
            obs = next(rows[name])
        except StopIteration:
            self._close(rows.pop(name))
            return self._replay.config.loop or self._buffer.put(_Ended(name))
        except Exception as error:
            self._close(rows.pop(name))
            self._failed.add(name)
            return self._buffer.put(_Ended(name, error))
        heapq.heappush(heap, (float(obs.ts), order, name, obs))
        return True

    @staticmethod
    def _close(iterator: Iterator[Any]) -> None:
        close = getattr(iterator, "close", None)
        if close is not None:
            close()

    def _publish(self, wall_t0: float) -> None:
        speed = self._replay.config.speed
        lag_warning = self._replay.config.lag_warning
        ended: set[str] = set()
        first_ts: float | None = None
        behind = False
        while len(ended) < len(self._subjects):
            item = self._buffer.get()
            if item is None:
                return
            if isinstance(item, _Ended):
                ended.add(item.name)
                subject = self._subjects[item.name]
                if item.error is None:
                    self._deliver(item.name, subject.on_completed)
                else:
                    self._deliver(item.name, subject.on_error, item.error)
                continue
            if first_ts is None:
                first_ts = item.ts
            due = wall_t0 + (item.ts - first_ts) / speed
            early = due - time.monotonic()
            if early > 0:
                behind = False  # back on schedule
                if self._stopped.wait(early):
                    return
            elif -early > lag_warning and not behind:
                behind = True
                logger.warning(
                    "Replay is behind schedule", lag_s=round(-early, 3), stream=item.name
                )
            self._deliver(item.name, self._subjects[item.name].on_next, item.value)

    @staticmethod
    def _deliver(name: str, notify: Callable[..., None], *args: Any) -> None:
        # A subscriber that raises (or has no on_error) must not stop the other streams.
        try:
            notify(*args)
        except Exception:
            logger.exception("Replay subscriber failed", stream=name)
