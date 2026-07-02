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

"""Stat helpers for teleop streams (latency / jitter / rate).

* **`pcts`** — percentile helper shared with the post-hoc report writer.
* **`LiveStreamStats`** — rolling window the robot measures over the inbound
  command wire, then ships each ``snapshot()`` to the operator HUD (the robot
  doesn't consume the stats locally — it's compute-and-forward).
"""

from __future__ import annotations

from collections import deque
from collections.abc import Sequence
from itertools import pairwise
import threading
import time

import numpy as np


def pcts(values: Sequence[float]) -> dict[str, float] | None:
    """p50/p95/p99/max of *values* in their native unit, or None if empty."""
    if not values:
        return None
    a = np.asarray(values, dtype=float)
    return {
        "p50": float(np.percentile(a, 50)),
        "p95": float(np.percentile(a, 95)),
        "p99": float(np.percentile(a, 99)),
        "max": float(a.max()),
    }


class LiveStreamStats:
    """Rolling-window health of an inbound stream, for forwarding to a remote HUD.

    ``record()`` notes each arrival in a bounded deque; ``snapshot()`` returns
    the window's median E2E latency, jitter, arrival rate, and throughput —
    which the robot ships to the operator (it doesn't use them locally).
    Thread-safe: ``record()`` on the transport callback, ``snapshot()`` on a
    separate reader.
    """

    def __init__(self, window: int = 120) -> None:
        self._lock = threading.Lock()
        # (wall_arrival, ts, seq, nbytes); ts/seq/nbytes are None when absent.
        self._samples: deque[tuple[float, float | None, int | None, int | None]] = deque(
            maxlen=window
        )

    def record(self, ts: float | None, seq: int | None = None, nbytes: int | None = None) -> None:
        """Note an inbound message's send-stamp, seq, and wire size (any None)."""
        with self._lock:
            self._samples.append((time.time(), ts, seq, nbytes))

    def snapshot(self) -> dict[str, float | None] | None:
        """Median latency/jitter (ms), rate (Hz), throughput. None until 2 samples."""
        with self._lock:
            samples = list(self._samples)
        if len(samples) < 2:
            return None

        arrivals = [w for w, _, _, _ in samples]
        intervals_ms = [(b - a) * 1000.0 for a, b in pairwise(arrivals)]
        # `is not None` — ts=0.0 is a real value, only None means absent.
        e2e_ms = [(w - ts) * 1000.0 for w, ts, _, _ in samples if ts is not None]
        sizes = [n for _, _, _, n in samples if n is not None]

        e2e = pcts(e2e_ms)
        jit = pcts(intervals_ms)
        span = arrivals[-1] - arrivals[0]
        return {
            "latency_ms": e2e["p50"] if e2e else None,
            "jitter_ms": jit["p50"] if jit else None,
            "rate_hz": (len(samples) - 1) / span if span > 0 else None,
            "throughput_bps": (sum(sizes) / span) if (sizes and span > 0) else None,
        }


__all__ = ["LiveStreamStats", "pcts"]
