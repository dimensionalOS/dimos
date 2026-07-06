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
the live bar is replaced by one plain persisted line. One bar is live at a
time — starting a new bar finalizes a dangling one (rich allows a single live
display); call :meth:`ProgressBar.close` when abandoning a stream early
(e.g. a bounded time window).
"""

from __future__ import annotations

import atexit
import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.memory2.type.observation import Observation

_REFRESH_S = 0.1  # refresh cap; keeps 200 Hz streams from redrawing per frame

_current: ProgressBar | None = None


def _close_current() -> None:
    if _current is not None:
        _current.close()


atexit.register(_close_current)  # restore the cursor if a bar is abandoned


class ProgressBar:
    """Callable per-observation progress; ``close()`` finalizes an unfinished bar."""

    def __init__(self, total: int, label: str = "") -> None:
        from rich.progress import (
            BarColumn,
            MofNCompleteColumn,
            Progress,
            TaskProgressColumn,
            TextColumn,
        )

        self._total = total
        self._label = label or "processing"
        # transient: close() prints its own persisted line, the live bar clears.
        # auto_refresh=False: no render thread; refreshes (throttled) from the callback.
        self._prog = Progress(
            TextColumn("[progress.description]{task.description}"),
            BarColumn(),
            TaskProgressColumn(),
            MofNCompleteColumn(),
            TextColumn("{task.fields[stats]}", style="dim"),
            transient=True,
            auto_refresh=False,
        )
        self._task = self._prog.add_task(self._label, total=total, stats="")
        self._closed = False
        self._seen = 0
        self._wall_start: float | None = None
        self._last_wall: float | None = None
        self._last_refresh: float | None = None
        self._first_ts: float | None = None
        self._stats = ""

    def __call__(self, obs: Observation[Any]) -> None:
        if self._closed:
            return  # observations may keep flowing after close (e.g. a tap downstream)
        now = time.monotonic()
        if self._wall_start is None:
            global _current
            if _current is not None:
                _current.close()  # only one live display may be active
            _current = self
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
        """Clear the live bar and persist it as a plain line."""
        global _current
        if self._closed:
            return
        self._closed = True
        if _current is self:
            _current = None
        self._prog.stop()  # transient: wipes the live bar
        pct = 100 * self._seen // self._total if self._total else 100
        print(f"{self._label} {pct}% [{self._seen}/{self._total}] {self._stats}")


def progress(total: int, label: str = "") -> ProgressBar:
    """Return a callback that renders streaming progress, one call per observation."""
    return ProgressBar(total, label)
