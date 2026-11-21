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
from threading import Lock
from typing import Any, TypeVar

from reactivex import Observable, defer, operators as ops
from reactivex.abc import SchedulerBase

from dimos.memory.replay import Replay, ReplayStream
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

    def start(
        self,
        replay: Replay,
        streams: Iterable[ReplayStream[Any]],
        *,
        path: str | None = None,
    ) -> None:
        """Log the selected replay window's duration in source seconds."""
        logger.info(
            "Replay started",
            source=self._source,
            path=path or self._source,
            duration_s=replay.set_window_duration(streams),
            speed=replay.config.speed,
            loop=replay.config.loop,
        )

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
            self._sealed = True
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
