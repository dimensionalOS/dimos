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
advance together. Late subscribers skip past data already behind wall time.

Setup metadata (intrinsics and mount frames that keep one stamp) is not part of
that clock: :meth:`ReplayStream.split_static` separates it from the timed rows.
"""

from __future__ import annotations

from functools import partial
from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING, Any, Generic, TypeVar, cast

import reactivex as rx
from reactivex.abc import DisposableBase, ObserverBase, SchedulerBase
from reactivex.disposable import (
    CompositeDisposable,
    Disposable,
    SerialDisposable,
    SingleAssignmentDisposable,
)
from reactivex.observable import Observable
from reactivex.scheduler import TimeoutScheduler
from reactivex.subject import AsyncSubject

from dimos.memory.cli.dataset import resolve_dataset
from dimos.memory.store.base import Store, StreamAccessor
from dimos.memory.tf import STATIC_SUFFIX
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.service.spec import BaseConfig, Configurable

if TYPE_CHECKING:
    from collections.abc import Callable, Generator, Iterator

    from dimos.memory.stream import Stream
    from dimos.memory.type.observation import Observation
    from dimos.msgs.geometry_msgs.Transform import Transform

T = TypeVar("T")

_LOOP_GAP = 0.05  # min wall-time gap inserted between loop wraps (seconds)
_LATE_TOLERANCE = 0.05  # don't skip frames within this many seconds of "now"


def resolve_db_path(dataset: str | Path) -> Path:
    """Resolve a recording, trying the exact input before appending ``.db``."""
    return resolve_dataset(dataset, suffix=".db")


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
        self._splits: dict[str, tuple[Any, ReplayStream[Any] | None]] = {}
        self._first_ts: float | None = None
        self._anchor: tuple[float, float] | None = None
        self._anchor_lock = threading.Lock()

    @property
    def streams(self) -> StreamAccessor[ReplayStream[Any]]:
        return StreamAccessor(self)

    def list_streams(self) -> list[str]:
        return self.store.list_streams()

    def stream(
        self,
        name: str,
        autocast: Callable[[Any], T] | None = None,
        *,
        source: Stream[Any] | None = None,
    ) -> ReplayStream[T]:
        """Replay a named stream, optionally from a filtered observation query."""
        return ReplayStream(replay=self, name=name, autocast=autocast, source=source)

    def first_ts(self) -> float | None:
        """Common origin for relative seek and duration: the earliest timed row in the store.

        Static metadata is often stamped long before the run, so a clip measured
        from it would hold no sensor data.
        """
        if self._first_ts is None:
            candidates: list[float] = []
            for name in self.store.list_streams():
                stream: ReplayStream[Any] = self.stream(name)
                try:
                    _, timed = stream.split_static()
                    if timed is not None:
                        rows = timed._source_stream()
                        candidates.append(float(rows.order_by("ts").first().ts))
                except (LookupError, ImportError, AttributeError):
                    continue
            if candidates:
                self._first_ts = min(candidates)
        return self._first_ts

    def _resolve_anchor(self, candidate_first_ts: float) -> tuple[float, float]:
        """Pin (wall_t0, replay_t0) on first call; return shared anchor."""
        with self._anchor_lock:
            if self._anchor is None:
                self._anchor = (time.time(), candidate_first_ts)
            return self._anchor

    def pin_anchor(self, ts: float | None = None) -> None:
        """Pin the anchor at *ts* (default: the earliest timed row) now, before any stream
        subscribes, so streams that start earlier than the first-subscribed one keep their
        frames."""
        first = self.first_ts() if ts is None else ts
        if first is not None:
            self._resolve_anchor(first)

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
        source: Stream[Any] | None = None,
    ) -> None:
        self._replay = replay
        self._name = name
        self._autocast = autocast
        self._source = source

    @property
    def name(self) -> str:
        return self._name

    def _source_stream(self) -> Stream[Any]:
        return self._replay.store.stream(self._name) if self._source is None else self._source

    def _decode(self, obs: Any) -> T:
        data = obs.data
        if self._autocast is not None:
            data = self._autocast(data)
        return cast("T", data)

    def _base_stream(self) -> Stream[Any]:
        """Memory Stream bounded by the replay window, ordered by ts."""
        cfg = self._replay.config
        s = self._source_stream()

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

    def split_static(self) -> tuple[T | None, ReplayStream[T] | None]:
        """Separate setup metadata from the rows that play on the recording clock.

        Returns the message to republish and the stream to schedule; either can be
        missing. Repeated copies of one stamped camera calibration and unchanged
        TF edges can predate the run. Other sensor samples keep their recorded
        timing, even when they share a stamp or a value. A stream given its own
        source or cast plays as given.

        A declared static TF channel (``tf_static``) holds the last recorded mount
        for each edge throughout the clip.
        """
        if self._source is not None or self._autocast is not None:
            return None, self
        splits = self._replay._splits
        if self._name not in splits:
            splits[self._name] = self._split_static()
        return splits[self._name]

    def _split_static(self) -> tuple[Any, ReplayStream[Any] | None]:
        rows = self._source_stream()
        payload_type = rows.data_type
        if payload_type is not None and issubclass(payload_type, TFMessage):
            if self._name.endswith(STATIC_SUFFIX):
                held = {
                    (t.frame_id, t.child_frame_id): t
                    for row in rows.order_by("ts")
                    for t in row.data.transforms
                }
                return (TFMessage(*held.values()), None) if held else (None, self)
            frozen, edges = _frozen_tf(rows)
            if not frozen.transforms:
                return None, self
            dynamic = rows.filter(partial(_has_dynamic_tf, edges))
            try:
                start = dynamic.order_by("ts").first().ts
            except LookupError:
                return frozen, None
            return frozen, ReplayStream(
                replay=self._replay,
                name=self._name,
                autocast=partial(_dynamic_tf, edges),
                source=dynamic.time_range(start, float("inf")),
            )
        if payload_type is not None and issubclass(payload_type, CameraInfo) and rows.count() > 1:
            first = rows.order_by("ts").first()
            if first.ts == rows.last().ts:
                encoded = first.data.lcm_encode()
                if all(row.data.lcm_encode() == encoded for row in rows):
                    return first.data, None
        return None, self

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
        s = self._source_stream()
        try:
            obs: Any = s.at(timestamp, tolerance).first()
        except LookupError:
            return None
        return self._decode(obs)

    def observable(
        self, *, skip_late: bool = True, start_on: AsyncSubject[None] | None = None
    ) -> Observable[T]:
        """Timed Observable scheduled against the Replay's shared anchor.

        The first subscribe across the whole :class:`Replay` pins
        ``(wall_t0, replay_t0)``. Late subscribers compute their entry from
        the same anchor and skip past frames already behind wall time.
        Adapted from the legacy ``timed_playback`` (which pinned a fresh
        anchor per subscribe).

        With *skip_late* off nothing is skipped: frames already behind wall
        time are emitted at once, in order, and the rest keep their timing.
        For grouped startup, *start_on* delays scheduling until the signal while
        opening the cursor at subscription time. Every selected stream keeps its first row.

        Disposing cancels pending emissions and waits for an active callback
        before releasing its iterator, so the store can then be closed safely.
        Read and decode failures close the iterator before notifying on_error.
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
            disposed = threading.Event()
            emission_lock = threading.RLock()
            pending = SerialDisposable()

            def make_iterator() -> Generator[tuple[float, Any, bool], None, None]:
                wrapped = False
                while True:
                    emitted = False
                    obs: Any
                    rows = iter(base())
                    try:
                        for obs in rows:
                            emitted = True
                            yield (obs.ts, obs, wrapped)
                            wrapped = False
                    finally:
                        close = getattr(rows, "close", None)
                        if close is not None:
                            close()
                    if not loop or not emitted:
                        break
                    wrapped = True

            iterator = make_iterator()

            def dispose() -> None:
                disposed.set()
                pending.dispose()
                # Reentrant for disposal from observer callbacks. An external
                # stop waits here until decoding and cursor reads have finished.
                with emission_lock:
                    iterator.close()

            def terminate(error: Exception | None = None) -> None:
                if disposed.is_set():
                    return
                dispose()
                if error is None:
                    observer.on_completed()
                else:
                    observer.on_error(error)

            try:
                first_ts, first_data, _ = next(iterator)
            except StopIteration:
                terminate()
                return Disposable()
            except Exception as error:
                terminate(error)
                return Disposable()

            now_replay = first_ts
            if skip_late and start_on is None:
                wall_t0, replay_t0 = replay._resolve_anchor(first_ts)
                now_replay = replay_t0 + (time.time() - wall_t0) * speed

            # Late-subscribe skip: drop frames behind the wall clock. Stop
            # if the iterator wraps (ts < prev) — that means we've exhausted
            # one full pass without finding a forward frame.
            wrap_offset = 0.0
            prev_skip = first_ts
            while skip_late and first_ts < now_replay - _LATE_TOLERANCE:
                try:
                    cand_ts, cand_data, wrapped = next(iterator)
                except StopIteration:
                    terminate()
                    return Disposable()
                except Exception as error:
                    terminate(error)
                    return Disposable()
                if wrapped or cand_ts < prev_skip:
                    wrap_offset += (prev_skip - cand_ts) + _LOOP_GAP
                    first_ts, first_data = cand_ts, cand_data
                    break
                first_ts, first_data = cand_ts, cand_data
                prev_skip = cand_ts

            prev_ts = first_ts

            def schedule(message: tuple[float, Any, bool], wrap_off: float, prev: float) -> None:
                if disposed.is_set():
                    return
                ts, data, wrapped = message
                if wrapped or ts < prev:
                    wrap_off += (prev - ts) + _LOOP_GAP
                target = wall_t0 + ((ts + wrap_off) - replay_t0) / speed
                delay = max(0.0, target - time.time())

                def emit(_s: SchedulerBase, _state: object) -> DisposableBase | None:
                    nonlocal wrap_offset, prev_ts
                    with emission_lock:
                        if disposed.is_set():
                            return None
                        try:
                            value = decode(data)
                        except Exception as error:
                            terminate(error)
                            return None
                        observer.on_next(value)
                        # An observer may dispose from on_next, including closing
                        # the store. Do not advance its cursor after that.
                        if disposed.is_set():
                            return None
                        try:
                            nxt = next(iterator)
                        except StopIteration:
                            terminate()
                            return None
                        except Exception as error:
                            terminate(error)
                            return None
                        wrap_offset = wrap_off
                        prev_ts = ts
                        schedule(nxt, wrap_offset, prev_ts)
                    return None

                # A zero-delay callback can schedule its successor before this
                # call returns. Install the slot first so it cannot overwrite
                # the successor's cancellation handle.
                scheduled = SingleAssignmentDisposable()
                pending.disposable = scheduled
                scheduled.disposable = sched.schedule_relative(delay, emit)

            def begin(_: None = None) -> None:
                nonlocal wall_t0, replay_t0
                if disposed.is_set():
                    return
                if start_on is not None or not skip_late:
                    wall_t0, replay_t0 = replay._resolve_anchor(first_ts)
                schedule((first_ts, first_data, False), wrap_offset, prev_ts)

            if start_on is not None:
                return CompositeDisposable(
                    Disposable(dispose), start_on.subscribe(begin, on_error=terminate)
                )
            begin()
            return Disposable(dispose)

        return rx.create(subscribe)


def _has_dynamic_tf(edges: set[tuple[str, str]], row: Observation[TFMessage]) -> bool:
    return not row.data.transforms or any(
        (t.frame_id, t.child_frame_id) not in edges for t in row.data.transforms
    )


def _dynamic_tf(edges: set[tuple[str, str]], message: TFMessage) -> TFMessage:
    # Older recordings can carry both fixed and changing edges in one row.
    return TFMessage(
        *(t for t in message.transforms if (t.frame_id, t.child_frame_id) not in edges)
    )


def _frozen_tf(rows: Stream[TFMessage]) -> tuple[TFMessage, set[tuple[str, str]]]:
    """Hold unchanged edges that repeat or belong to several rows at a single timestamp."""
    one_timestamp = rows.count() > 1 and rows.order_by("ts").first().ts == rows.last().ts
    candidates: dict[tuple[str, str], Transform | None] = {}
    repeated: set[tuple[str, str]] = set()
    for row in rows:
        for transform in row.data.transforms:
            edge = (transform.frame_id, transform.child_frame_id)
            if edge not in candidates:
                candidates[edge] = transform
            elif (previous := candidates[edge]) is not None:
                if (
                    previous.ts == transform.ts
                    and previous.translation.to_tuple() == transform.translation.to_tuple()
                    and previous.rotation.to_tuple() == transform.rotation.to_tuple()
                ):
                    repeated.add(edge)
                else:
                    candidates[edge] = None
    fixed = {
        edge: t
        for edge, t in candidates.items()
        if t is not None and (one_timestamp or edge in repeated)
    }
    return TFMessage(*fixed.values()), set(fixed)
