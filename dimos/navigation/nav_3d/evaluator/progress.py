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

"""One bar per stage per dataset, fed over a queue from any process."""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
import multiprocessing
import threading
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator
    from contextlib import AbstractContextManager
    import queue
    from types import TracebackType

    from rich.progress import Progress, TaskID

    from dimos.navigation.nav_3d.evaluator.cases import Suite

    Tick = Callable[[], None]
    ProgressFactory = Callable[[int, str], AbstractContextManager[Tick]]

# What a stage sends the display: the kind of event, the bar, and a count.
Message = tuple[str, str, int]

# Ceiling on steps per queue message, so a 12k-frame replay costs a few hundred
# sends. A short bar batches less, so it still moves about this many times.
MAX_TICK_BATCH = 25
TARGET_UPDATES = 100
_STOP = "stop"
_DRAIN_TIMEOUT_S = 5.0


def _noop() -> None:
    """Stand-in tick, so a caller never guards its loop on a None."""


@dataclass(frozen=True)
class QueueProgress:
    """A ProgressFactory a worker process can pickle."""

    queue: queue.Queue[Message]

    @contextmanager
    def __call__(self, total: int, label: str) -> Iterator[Tick]:
        self.queue.put(("open", label, total))
        batch = max(1, min(MAX_TICK_BATCH, total // TARGET_UPDATES))
        pending = 0
        seen = 0

        def tick() -> None:
            nonlocal pending, seen
            pending += 1
            seen += 1
            if pending >= batch:
                self.queue.put(("step", label, pending))
                pending = 0

        try:
            yield tick
        finally:
            # A frame total is an estimate, so the closing count is the truth.
            self.queue.put(("close", label, seen))


class RunProgress:
    """The live display. Hand factory() to evaluate for a bar per stage."""

    def __enter__(self) -> RunProgress:
        # Lazy: rich stays off the CLI startup path.
        from rich.progress import (
            BarColumn,
            MofNCompleteColumn,
            Progress,
            TaskProgressColumn,
            TextColumn,
            TimeElapsedColumn,
        )

        self._manager = multiprocessing.Manager()
        self._queue = self._manager.Queue()
        self._bars: Progress = Progress(
            TextColumn("[progress.description]{task.description}"),
            BarColumn(),
            TaskProgressColumn(),
            MofNCompleteColumn(),
            TimeElapsedColumn(),
            refresh_per_second=10,
        )
        self._tasks: dict[str, TaskID] = {}
        self._bars.start()
        self._pump = threading.Thread(target=self._drain, daemon=True)
        self._pump.start()
        return self

    def factory(self) -> ProgressFactory:
        return QueueProgress(self._queue)

    def _drain(self) -> None:
        while True:
            kind, label, count = self._queue.get()
            if kind == _STOP:
                return
            if kind == "open":
                self._tasks[label] = self._bars.add_task(label, total=count)
            elif label not in self._tasks:
                continue
            elif kind == "close":
                self._bars.update(self._tasks[label], total=count, completed=count)
            elif count:
                self._bars.advance(self._tasks[label], count)

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self._queue.put((_STOP, "", 0))
        self._pump.join(_DRAIN_TIMEOUT_S)
        self._bars.stop()
        self._manager.shutdown()


@contextmanager
def stage_progress(progress: ProgressFactory | None, total: int, label: str) -> Iterator[Tick]:
    """A bar over total steps, or a no-op tick when nobody is watching."""
    if progress is None:
        yield _noop
        return
    with progress(total, label) as tick:
        yield tick


@contextmanager
def frame_progress(progress: ProgressFactory | None, suite: Suite, label: str) -> Iterator[Tick]:
    """A bar over one replay of a suite's recording."""
    if progress is None:
        yield _noop
        return
    with progress(suite.frame_count(), f"{suite.dataset} {label}") as tick:
        yield tick
