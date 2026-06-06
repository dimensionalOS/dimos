#!/usr/bin/env python3
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

"""Stat helpers for teleop streams (latency / jitter / loss / reorder).

Two flavors live here:

* **Pure functions** (`pcts`, `loss_pct`, `reorder_count`, `classify_e2e`) —
  shared by the post-hoc report writer (``teleop/utils/report.py``) and any
  live stats consumer.
* **`LiveStreamStats`** — a rolling-window class for always-on consumers that
  only need a recent snapshot (e.g. the operator HUD's command-plane telemetry).
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


def loss_pct(seqs: Sequence[int]) -> float | None:
    """Loss % from gaps in a monotonic sequence; None if fewer than 2 samples.

    ``loss = 1 - distinct_received / (max_seq - min_seq + 1)``. Reorders and
    duplicates do not inflate it — only genuinely missing seq values count.
    """
    valid = [s for s in seqs if s]
    if len(valid) < 2:
        return None
    expected = max(valid) - min(valid) + 1
    received = len(set(valid))
    return max(0.0, (1.0 - received / expected) * 100.0)


def reorder_count(seqs: Sequence[int]) -> int:
    """Count messages that arrived with a seq below an already-seen maximum."""
    count = 0
    running_max = -1
    for s in seqs:
        if not s:
            continue
        if s < running_max:
            count += 1
        else:
            running_max = s
    return count


def classify_e2e(
    p50_ms: float,
    bands: Sequence[tuple[float, str]] = (
        (50.0, "excellent"),
        (100.0, "good"),
        (150.0, "usable"),
    ),
) -> str:
    """Map an E2E p50 latency to an acceptance band label.

    ``bands`` is an ascending list of (threshold_ms, label) pairs; anything past
    the last threshold falls into ``"degraded"``. A negative p50 (operator/robot
    clocks not synced) returns ``"clock skew"``.
    """
    if p50_ms < 0:
        return "clock skew"
    for threshold, label in bands:
        if p50_ms < threshold:
            return label
    return "degraded"


class LiveStreamStats:
    """Rolling-window health for an always-on stream consumer.

    Records ``(wall, ts, seq)`` per arrival in a bounded deque so old samples
    fall off automatically; ``snapshot()`` returns the current window's median
    E2E latency, median inter-arrival jitter, seq-gap loss, and arrival rate.
    Thread-safe — ``record()`` runs on the transport callback,
    ``snapshot()`` on a separate reader.
    """

    def __init__(self, window: int = 120) -> None:
        self._lock = threading.Lock()
        # (wall_arrival, ts, seq); ts/seq are None when the stream is unstamped.
        self._samples: deque[tuple[float, float | None, int | None]] = deque(maxlen=window)

    def record(self, ts: float | None, seq: int | None) -> None:
        """Note an inbound message's send-stamp + monotonic counter."""
        with self._lock:
            self._samples.append((time.time(), ts or None, seq or None))

    def snapshot(self) -> dict[str, float | None] | None:
        """Median latency/jitter (ms), loss (%), rate (Hz), or None.

        Returns ``None`` until at least two samples have landed (one inter-arrival
        interval is needed). Uses the module's shared ``pcts`` / ``loss_pct`` so
        the math matches the benchmark module's report.
        """
        with self._lock:
            samples = list(self._samples)
        if len(samples) < 2:
            return None

        arrivals = [w for w, _, _ in samples]
        intervals_ms = [(b - a) * 1000.0 for a, b in pairwise(arrivals)]
        e2e_ms = [(w - ts) * 1000.0 for w, ts, _ in samples if ts]
        seqs = [s for _, _, s in samples if s]

        e2e = pcts(e2e_ms)
        jit = pcts(intervals_ms)
        span = arrivals[-1] - arrivals[0]
        return {
            "latency_ms": e2e["p50"] if e2e else None,
            "jitter_ms": jit["p50"] if jit else None,
            "loss_pct": loss_pct(seqs),
            "rate_hz": (len(samples) - 1) / span if span > 0 else None,
        }


__all__ = ["LiveStreamStats", "classify_e2e", "loss_pct", "pcts", "reorder_count"]
