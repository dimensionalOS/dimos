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

"""A per-observation progress bar, shaped for ``Stream.tap``.

    stream.tap(progress(stream.count(), "render")).drain()

Renders a rich progress bar: label, percent, count, data-seconds covered,
speed relative to wall-clock (``x rt``), and per-frame latency. On completion
the bar persists as one plain line. All bars share a single live display, so
sequential or interleaved streams compose; call :meth:`ProgressBar.close` when
abandoning a stream early (e.g. a bounded time window).
"""

from __future__ import annotations

import atexit
import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from rich.progress import Progress, TaskID

    from dimos.memory2.type.observation import Observation

_REFRESH_S = 0.1  # manual refresh cap; keeps 200 Hz streams from redrawing per frame

_shared: Progress | None = None


def _shared_progress() -> Progress:
    """One Progress (one live display) shared by every bar in the process."""
    global _shared
    if _shared is None:
        from rich.progress import (
            BarColumn,
            MofNCompleteColumn,
            Progress,
            TaskProgressColumn,
            TextColumn,
        )

        # auto_refresh=False: no background render thread to leak if a stream
        # ends short of ``total`` — bars refresh (throttled) from the callback.
        # transient: close() prints its own persisted line, the live bar clears.
        _shared = Progress(
            TextColumn("[progress.description]{task.description}"),
            BarColumn(),
            TaskProgressColumn(),
            MofNCompleteColumn(),
            TextColumn("{task.fields[stats]}", style="dim"),
            auto_refresh=False,
            transient=True,
        )
        atexit.register(_shared.stop)  # restore the cursor if a bar is abandoned
    return _shared


class ProgressBar:
    """Callable per-observation progress; ``close()`` finalizes an unfinished bar."""

    def __init__(self, total: int, label: str = "") -> None:
        self._total = total
        self._label = label or "processing"
        self._prog = _shared_progress()
        self._task: TaskID | None = self._prog.add_task(self._label, total=total, stats="")
        self._seen = 0
        self._wall_start: float | None = None
        self._last_wall: float | None = None
        self._last_refresh: float | None = None
        self._first_ts: float | None = None
        self._stats = ""

    def __call__(self, obs: Observation[Any]) -> None:
        if self._task is None:
            return  # closed; observations may keep flowing (e.g. a tap after close)
        now = time.monotonic()
        if self._wall_start is None:
            self._wall_start = now
            self._first_ts = obs.ts
            self._prog.start()
        assert self._first_ts is not None  # narrowed by the same `if` above
        frame_ms = (now - self._last_wall) * 1000 if self._last_wall is not None else 0.0
        self._last_wall = now
        self._seen += 1
        wall = now - self._wall_start
        data = obs.ts - self._first_ts
        speed = data / wall if wall > 0 else 0.0
        self._stats = f"{data:.1f}s ({speed:.1f} x rt) {frame_ms:.0f}ms/frame"
        self._prog.update(self._task, advance=1, stats=self._stats)
        if self._seen >= self._total:
            self.close()
        elif self._last_refresh is None or now - self._last_refresh >= _REFRESH_S:
            self._last_refresh = now
            self._prog.refresh()

    def close(self) -> None:
        """Persist the bar as a plain line and release it from the live display."""
        if self._task is None:
            return  # already closed
        self._prog.remove_task(self._task)  # before print: Live re-renders on print
        self._task = None
        pct = 100 * self._seen // self._total if self._total else 100
        self._prog.console.print(
            f"{self._label} {pct}% [{self._seen}/{self._total}] {self._stats}",
            markup=False,
            highlight=False,
        )
        if not self._prog.tasks:
            self._prog.stop()


def progress(total: int, label: str = "") -> ProgressBar:
    """Return a callback that renders streaming progress, one call per observation."""
    return ProgressBar(total, label)
