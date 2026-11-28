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

``store.replay()`` returns a :class:`Replay`, which plays the streams added to
it as one timeline: every frame is published in recorded order, at its recorded
offset from the first frame, and none is dropped.
"""

from __future__ import annotations

from collections import deque
from functools import cached_property
import heapq
from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING, Any, Generic, TypeVar, cast

from pydantic import Field
from reactivex.subject import Subject

from dimos.memory.cli.dataset import resolve_dataset
from dimos.memory.store.base import Store, StreamAccessor
from dimos.memory.utils.time_window import RecordingWindow
from dimos.protocol.service.spec import BaseConfig, Configurable
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable, Generator

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
    loop: bool = False
    # With ``loop``, recording seconds between the last frame of a pass and the first of the next.
    loop_gap: float = Field(default=0.0, ge=0.0)
    # Frames read ahead of the publisher.
    buffer_size: int = Field(default=128, gt=0)
    # Seconds the publisher may fall behind schedule before it warns.
    lag_warning: float = 0.5


class Replay(Configurable):
    """Plays a :class:`Store`'s streams, within a seek/duration window, as one timeline.

    Constructed via :meth:`Store.replay`. A reader thread merges the :meth:`add`-ed streams by
    timestamp, decodes each frame and keeps ``buffer_size`` of them buffered. A publisher thread
    publishes each at its recorded offset from the first frame, divided by ``speed``, counted from
    :meth:`play`. No frame is dropped: a publisher that falls behind publishes late frames at once
    and warns past ``lag_warning`` seconds. A frame that cannot be read or decoded stops the
    replay and errors every stream.

    :meth:`load` returns once the buffer is full, so a module's ``start()`` pays the first reads
    there, before :meth:`play` starts the clock. :meth:`stop` waits for an in-flight publish and
    closes every read cursor, so the store can be closed after it.

    TODO: messages keep their recorded timestamps; shift them to playback time,
    so a replayed message is stamped like a live one.
    """

    config: ReplayConfig

    def __init__(self, store: Store, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.store = store
        self._streams: dict[str, ReplayStream[Any]] = {}
        self._subjects: dict[str, Subject[Any]] = {}
        self._stopped = threading.Event()
        self._buffer = _Buffer(self.config.buffer_size, self._stopped)
        self._state_lock = threading.Lock()
        self._reader: threading.Thread | None = None
        self._read_done = threading.Event()
        self._loaded = threading.Event()
        self._publisher: threading.Thread | None = None
        self._error: Exception | None = None

    @property
    def streams(self) -> StreamAccessor[ReplayStream[Any]]:
        return StreamAccessor(self)

    def list_streams(self) -> list[str]:
        return self.store.list_streams()

    def stream(self, name: str, autocast: Callable[[Any], T] | None = None) -> ReplayStream[T]:
        return ReplayStream(replay=self, name=name, autocast=autocast)

    @cached_property
    def window(self) -> RecordingWindow:
        """The seek/duration window every stream is cut to, found once."""
        return RecordingWindow.from_store(self.store, self.config.seek or 0.0, self.config.duration)

    def add(self, stream: ReplayStream[T]) -> Observable[T]:
        """Put *stream* on the timeline. Its frames complete when it ends (never, with ``loop``)."""
        with self._state_lock:
            if self._reader is not None:
                raise RuntimeError("Replay.add() after load()")
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
            if self._read_passes() and self.config.loop:
                for name in self._streams:
                    if not self._buffer.put((0.0, name, _END)):
                        return
        finally:
            self._read_done.set()
            self._buffer.wake()

    def _read_passes(self) -> bool:
        """Buffer every pass of the recording. False once stopped or failed."""
        offset = 0.0
        while True:
            rows = [self._rows(name, stream) for name, stream in self._streams.items()]
            first = last = None
            try:
                for ts, name, obs in heapq.merge(*rows, key=lambda row: row[0]):
                    if obs is _FAILED:
                        self._buffer.put((ts, name, _FAILED))
                        return False
                    if obs is _END:
                        if not self._buffer.put((ts, name, _END)):
                            return False
                        continue
                    first = ts if first is None else first
                    last = ts
                    try:
                        value = self._streams[name]._decode(obs)
                    except Exception as error:
                        self._error = error
                        self._buffer.put((ts, name, _FAILED))
                        return False
                    if not self._buffer.put((ts + offset, name, value)):
                        return False
            finally:
                for stream_rows in rows:
                    stream_rows.close()
            if not self.config.loop or first is None or last is None:
                return True
            offset += last - first + self.config.loop_gap

    def _rows(
        self, name: str, stream: ReplayStream[Any]
    ) -> Generator[tuple[float, str, Any], None, None]:
        """*name*'s rows in ts order, then _END unless looping; a read error ends it at once."""
        try:
            rows = iter(stream.rows())
            try:
                ts = float("-inf")
                for obs in rows:
                    ts = float(obs.ts)
                    yield ts, name, obs
            finally:
                close = getattr(rows, "close", None)
                if close is not None:
                    close()
        except Exception as error:
            self._error = error
            yield float("-inf"), name, _FAILED
            return
        if not self.config.loop:
            yield ts, name, _END

    def _publish(self, wall_t0: float) -> None:
        speed = self.config.speed
        lag_warning = self.config.lag_warning
        ended: set[str] = set()
        first_ts: float | None = None
        behind = False
        while len(ended) < len(self._subjects):
            item = self._buffer.get()
            if item is None:
                return
            ts, name, value = item
            if value is _FAILED:
                self._fail(name, ended)
                return
            if value is _END:
                ended.add(name)
                self._deliver(name, self._subjects[name].on_completed)
                continue
            if first_ts is None:
                first_ts = ts
            due = wall_t0 + (ts - first_ts) / speed
            early = due - time.monotonic()
            if early > 0:
                behind = False  # back on schedule
                if self._stopped.wait(early):
                    return
            elif -early > lag_warning and not behind:
                behind = True
                logger.warning("Replay is behind schedule", lag_s=round(-early, 3), stream=name)
            self._deliver(name, self._subjects[name].on_next, value)

    def _fail(self, failed: str, ended: set[str]) -> None:
        """A bad recording or a config that cannot decode it: stop, and error every stream."""
        error = cast("Exception", self._error)
        logger.error("Replay stopped: recording could not be read", stream=failed, exc_info=error)
        for name, subject in self._subjects.items():
            if name not in ended:
                try:
                    subject.on_error(error)
                except Exception as raised:
                    if raised is not error:  # a subscriber without on_error re-raises it
                        logger.exception("Replay subscriber failed", stream=name)

    @staticmethod
    def _deliver(name: str, notify: Callable[..., None], *args: Any) -> None:
        # A subscriber's bug is logged, not allowed to stop the replay for every other subscriber.
        try:
            notify(*args)
        except Exception:
            logger.exception("Replay subscriber failed", stream=name)


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

    def rows(self) -> Stream[Any]:
        """The recorded observations inside the replay window, in ts order."""
        return self._replay.window.apply(self._replay.store.stream(self._name)).order_by("ts")

    def first_ts(self) -> float | None:
        """First ts within the replay window (post seek/duration)."""
        try:
            return float(self.rows().first().ts)
        except LookupError:
            return None

    def last_ts(self) -> float | None:
        try:
            return float(self.rows().last().ts)
        except LookupError:
            return None

    def count(self) -> int:
        return int(self.rows().count())

    def first(self) -> T | None:
        try:
            return self._decode(self.rows().first())
        except LookupError:
            return None


# A buffered item is (ts, stream name, value): ts is recording seconds plus the loop passes
# before it, and value is a decoded frame or one of these.
_END = object()  # the stream has no more frames
_FAILED = object()  # the stream could not be read or decoded; Replay._error says why


class _Buffer:
    """Bounded FIFO between the reader and the publisher that a stop wakes up."""

    def __init__(self, size: int, stopped: threading.Event) -> None:
        self._size = size
        self._items: deque[tuple[float, str, Any]] = deque()
        self._stopped = stopped
        self._changed = threading.Condition()

    def put(self, item: tuple[float, str, Any]) -> bool:
        """False once stopped, without adding *item*."""
        with self._changed:
            self._changed.wait_for(lambda: len(self._items) < self._size or self._stopped.is_set())
            if self._stopped.is_set():
                return False
            self._items.append(item)
            self._changed.notify_all()
            return True

    def get(self) -> tuple[float, str, Any] | None:
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
