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

"""A per-observation progress callback, shaped for ``Stream.tap``.

    stream.tap(progress(stream.count(), "render")).drain()

Prints a single rewritten line: percent, count, data-seconds covered, speed
relative to wall-clock (``x rt``), and per-frame latency.
"""

from __future__ import annotations

from collections.abc import Callable
import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.memory2.type.observation import Observation


def progress(total: int, label: str = "") -> Callable[[Observation[Any]], None]:
    """Return a callback that prints streaming progress, one call per observation."""
    seen = 0
    wall_start: float | None = None
    last_wall: float | None = None
    first_ts: float | None = None

    def _progress(obs: Observation[Any]) -> None:
        nonlocal seen, wall_start, last_wall, first_ts
        now = time.monotonic()
        if wall_start is None:
            wall_start = now
            first_ts = obs.ts
        assert first_ts is not None  # narrowed by the same `if` above
        frame_ms = (now - last_wall) * 1000 if last_wall is not None else 0.0
        last_wall = now
        seen += 1
        pct = 100 * seen // total if total else 100
        wall = now - wall_start
        data = obs.ts - first_ts
        speed = data / wall if wall > 0 else 0.0
        end = "\n" if seen >= total else ""
        prefix = f"{label} " if label else ""
        print(
            f"\r{prefix}{pct:>3}% [{seen}/{total}] {data:.1f}s ({speed:.1f} x rt) {frame_ms:.0f}ms/frame",
            end=end,
            flush=True,
        )

    return _progress
