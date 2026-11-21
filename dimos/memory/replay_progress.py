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

"""Source metadata and completion reporting for module-owned replay subscriptions."""

from collections.abc import Callable, Iterable
from functools import partial
from threading import Lock
from typing import Any, TypeVar

from reactivex import Observable, create, defer, interval, operators as ops
from reactivex.abc import DisposableBase, ObserverBase, SchedulerBase
from reactivex.disposable import CompositeDisposable
from reactivex.subject import AsyncSubject

from dimos.memory.replay import Replay, ReplayStream
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
T = TypeVar("T")


class ReplayProgress:
    """Report completion once all tracked subscriptions finish and setup is sealed.

    Only natural completion counts: cancellation, errors and loop wraps do not.
    Call ``start`` during setup, then ``seal`` after wiring every stream.
    Exclude indefinitely republished static metadata from ``track``.
    """

    def __init__(self, source: str, on_complete: Callable[[], None] | None = None) -> None:
        self._source = source
        self._on_complete = on_complete
        self._lock = Lock()
        self._subscribed = 0
        self._completed = 0
        self._sealed = False
        self._finished = False
        self._ready: AsyncSubject[None] = AsyncSubject()
        self._anchor: tuple[Replay, float] | None = None

    def start(
        self,
        replay: Replay,
        streams: Iterable[ReplayStream[Any]],
        *,
        path: str | None = None,
    ) -> None:
        """Select the first timed source timestamp without starting the playback clock."""
        timed = (stream.split_static()[1] for stream in streams)
        bounds = [(stream.first_ts(), stream.last_ts()) for stream in timed if stream is not None]
        starts = [start for start, _ in bounds if start is not None]
        ends = [end for _, end in bounds if end is not None]
        if starts:
            self._anchor = (replay, min(starts))
        logger.info(
            "Replay started",
            source=self._source,
            path=path or self._source,
            duration_s=max(ends) - min(starts) if starts else 0.0,
            speed=replay.config.speed,
            loop=replay.config.loop,
        )

    def play(self, stream: ReplayStream[T]) -> Observable[T]:
        """Start initial subscribers together and let later subscribers join the running clock.

        Setup metadata stays off that clock and is republished every second instead.
        """
        static, timed = stream.split_static()
        sources: list[Observable[T]] = []
        if timed is not None:
            sources.append(self.track(defer(partial(self._open, timed))))
        if static is None:
            return sources[0]
        sources.append(interval(1.0).pipe(ops.map(lambda _tick: static)))

        def subscribe(
            observer: ObserverBase[T], scheduler: SchedulerBase | None = None
        ) -> DisposableBase:
            if isinstance(static, TFMessage):
                observer.on_next(static)  # The first sensor frames need the frame tree.
            return CompositeDisposable(
                [
                    source.subscribe(observer.on_next, observer.on_error, scheduler=scheduler)
                    for source in sources
                ]
            )

        return create(subscribe)

    def _open(self, stream: ReplayStream[T], _scheduler: SchedulerBase | None) -> Observable[T]:
        with self._lock:
            if self._sealed:
                return stream.observable()
        return stream.observable(skip_late=False, start_on=self._ready)

    def track(self, source: Observable[T]) -> Observable[T]:
        def factory(_scheduler: SchedulerBase | None) -> Observable[T]:
            with self._lock:
                self._subscribed += 1
            return source.pipe(ops.do_action(on_completed=self._stream_completed))

        return defer(factory)

    def seal(self) -> None:
        """Allow completion after all subscriptions have been registered.

        An empty stream may complete synchronously during setup, before later
        subscriptions are registered, so counters alone are insufficient.
        With no timed subscriptions, playback is already complete.
        """
        with self._lock:
            if self._sealed:
                return
            if self._anchor is not None:
                replay, ts = self._anchor
                replay.pin_anchor(ts)
            self._sealed = True
        self._ready.on_next(None)
        self._ready.on_completed()
        self._maybe_finish()

    def _stream_completed(self) -> None:
        with self._lock:
            self._completed += 1
        self._maybe_finish()

    def _maybe_finish(self) -> None:
        with self._lock:
            done = self._sealed and not self._finished and self._completed == self._subscribed
            if done:
                self._finished = True
        if done:
            logger.info("Replay finished", source=self._source)
            if self._on_complete is not None:
                self._on_complete()
