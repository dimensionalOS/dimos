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

"""Health monitoring for tick-based modules — drops are metrics, not errors.

Under ``KeepLast`` backpressure a controller dropping most of its ticks is
the system *working as designed*, so per-drop warnings are wrong. The
mature ladder implemented here:

1. **Count always** — ticks resolved/stepped, drops by reason
   (``backpressure``, ``missing_input``), step latency, input rates, ages
   of consumed values, output rates.
2. **Report continuously** — an aggregated :class:`Health` snapshot every
   ``interval_s``, handed to a sink (the module appends it to a
   ``_health`` memory2 stream: live-only on a NullStore, recorded next to
   the data it explains on a storage-backed store).
3. **Log on transitions** — one warmup line comparing observed input rates
   to declared expectations, one line on entering ``DEGRADED``/``STALLED``
   (with the violated contracts), a throttled reminder while unhealthy,
   one line on recovery.

Contracts come from two places, deliberately split:

- *semantic tolerances in code* — ``latest(max_age=...)`` already declares
  "older than this is unacceptable"; its violation shows up as
  ``missing_input`` drops;
- *rates in deployment config* — ``expected_hz`` per input and
  ``min_output_hz``, because sim, replay, and the robot legitimately
  differ.

The monitor is pure bookkeeping with an injectable clock — no threads, no
streams — so the contract messages are unit-testable.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import threading
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, Field

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable

logger = setup_logger()

OK = "OK"
DEGRADED = "DEGRADED"
STALLED = "STALLED"


class ModuleContracts(BaseModel):
    """Module-wide contracts: the tick loop's promises, not any one port's."""

    min_output_hz: float | None = Field(default=None, gt=0.0)
    """Absolute liveness floor: rate of ticks that emit at least one output."""

    max_drop_ratio: float | None = Field(default=None, gt=0.0, le=1.0)
    """Scale-free "the step keeps up": fraction of viable ticks skipped by backpressure."""

    max_tick_latency_s: float | None = Field(default=None, gt=0.0)
    """p99 end-to-end latency, trigger arrival to outputs published."""

    max_missing_ratio: float = Field(default=0.5, gt=0.0, le=1.0)
    """Global default per-input missing threshold (per-input contracts override)."""


class HealthConfig(BaseModel):
    """Monitor mechanics — how health is measured and reported, not what it promises."""

    interval_s: float = Field(default=1.0, gt=0.0)
    warmup_s: float = Field(default=5.0, ge=0.0)
    unhealthy_log_every_s: float = Field(default=10.0, gt=0.0)
    stall_after_s: float = Field(default=5.0, gt=0.0)
    ratio_min_samples: int = Field(default=10, ge=1)
    """Ratio/latency contracts only evaluate on windows with at least this
    many samples — tiny windows are noise, zero traffic passes vacuously."""

    stream: bool = True
    """Append snapshots to a ``_health`` stream in the module store."""


def _percentile(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    vals = sorted(values)
    i = min(len(vals) - 1, max(0, round(q * (len(vals) - 1))))
    return vals[i]


@dataclass(frozen=True)
class Health:
    """One aggregated health snapshot (the payload of the ``_health`` stream)."""

    ts: float
    state: str  # OK | DEGRADED | STALLED
    violations: tuple[str, ...]
    metrics: dict[str, float]


@dataclass
class _Window:
    """Counters accumulated since the last report."""

    inputs: dict[str, int] = field(default_factory=dict)
    resolved: int = 0
    queued: int = 0  # viable ticks handed to the backpressure buffer
    stepped: int = 0
    emitted: int = 0  # steps that produced at least one output
    missing: dict[str, int] = field(default_factory=dict)  # drops by field
    blocked: int = 0  # ticks evicted while waiting for interpolation brackets
    outputs: dict[str, int] = field(default_factory=dict)
    latencies_s: list[float] = field(default_factory=list)  # trigger ts -> publish


class HealthMonitor:
    """Counts tick-loop events and turns them into contract messages.

    Thread-safe; all hooks are O(1). Call :meth:`maybe_report`
    opportunistically from the worker loops — it no-ops until
    ``interval_s`` has elapsed.
    """

    def __init__(
        self,
        name: str,
        *,
        contracts: ModuleContracts | None = None,
        health: HealthConfig | None = None,
        expected_hz: dict[str, float] | None = None,
        out_min_hz: dict[str, float] | None = None,
        missing_ratio_by_input: dict[str, float] | None = None,
        rate_tolerance: float = 0.5,
        sink: Callable[[Health], None] | None = None,
        clock: Callable[[], float] | None = None,
    ) -> None:
        """``contracts`` = module-wide promises; ``health`` = reporting
        mechanics; the three dicts are the *resolved* per-port contracts
        (class declarations already merged with deployment overrides by
        the caller)."""
        import time

        contracts = contracts if contracts is not None else ModuleContracts()
        cfg = health if health is not None else HealthConfig()

        self.name = name
        self.expected_hz = dict(expected_hz or {})
        self.min_output_hz = contracts.min_output_hz
        self.out_min_hz = dict(out_min_hz or {})
        self.max_drop_ratio = contracts.max_drop_ratio
        self.max_tick_latency_s = contracts.max_tick_latency_s
        self.max_missing_ratio = contracts.max_missing_ratio
        self.missing_ratio_by_input = dict(missing_ratio_by_input or {})
        # Ratio contracts are vacuous on tiny windows (3 drops of 4 ticks =
        # 75%) and at zero traffic — they only evaluate at this many samples.
        # Absolute contracts (min_output_hz, expected_hz) remain the liveness floor.
        self.ratio_min_samples = cfg.ratio_min_samples
        self.interval_s = cfg.interval_s
        self.warmup_s = cfg.warmup_s
        self.unhealthy_log_every_s = cfg.unhealthy_log_every_s
        self.rate_tolerance = rate_tolerance
        self.sink = sink
        self.clock = clock if clock is not None else time.time
        # A stall must persist this many reporting windows before we call it —
        # a single zero-step window is normal for a step slower than the interval.
        self._stall_windows = max(1, round(cfg.stall_after_s / cfg.interval_s))
        self._zero_step_windows = 0
        self._zero_resolve_windows = 0

        self._lock = threading.Lock()
        self._t0 = self.clock()
        self._last_report = self._t0
        self._last_unhealthy_log = 0.0
        self._warmup_done = False
        self._state = OK
        self._state_since = self._t0
        self._latest: Health | None = None
        self._win = _Window()
        self._total_inputs: dict[str, int] = {}
        self._step_ms: deque[float] = deque(maxlen=256)
        self._latency_ms: deque[float] = deque(maxlen=256)
        self._ages: dict[str, deque[float]] = {}
        self._buffer_len: Callable[[], int] = lambda: 0
        self._pending_len: Callable[[], int] = lambda: 0

    # -- hooks (called from the tick loops) -----------------------------------

    def attach_gauges(self, buffer_len: Callable[[], int], pending_len: Callable[[], int]) -> None:
        """Wire gauges sampled at report time (buffer depth, alignment-pending)."""
        self._buffer_len = buffer_len
        self._pending_len = pending_len

    def on_input(self, name: str) -> None:
        with self._lock:
            self._win.inputs[name] = self._win.inputs.get(name, 0) + 1
            self._total_inputs[name] = self._total_inputs.get(name, 0) + 1

    def on_resolved(self) -> None:
        with self._lock:
            self._win.resolved += 1

    def on_missing(self, fields: list[str]) -> None:
        """A resolved tick was dropped — required inputs missing."""
        with self._lock:
            for f in fields:
                self._win.missing[f] = self._win.missing.get(f, 0) + 1

    def on_queued(self) -> None:
        """A viable tick was handed to the backpressure buffer."""
        with self._lock:
            self._win.queued += 1

    def on_blocked(self, n: int) -> None:
        """*n* ticks were evicted while waiting for interpolation brackets."""
        if n:
            with self._lock:
                self._win.blocked += n

    def on_step(
        self,
        duration_s: float,
        ages: dict[str, float],
        emitted: bool,
        latency_s: float | None = None,
    ) -> None:
        """One step completed. ``latency_s`` is end-to-end tick latency —
        trigger timestamp to outputs published — covering queue wait,
        alignment wait, buffer wait, and the step itself."""
        with self._lock:
            self._win.stepped += 1
            if emitted:
                self._win.emitted += 1
            self._step_ms.append(duration_s * 1000.0)
            if latency_s is not None:
                self._latency_ms.append(latency_s * 1000.0)
                if len(self._win.latencies_s) < 4096:
                    self._win.latencies_s.append(latency_s)
            for name, age in ages.items():
                self._ages.setdefault(name, deque(maxlen=256)).append(age)

    def on_output(self, name: str) -> None:
        with self._lock:
            self._win.outputs[name] = self._win.outputs.get(name, 0) + 1

    # -- reporting ---------------------------------------------------------------

    def maybe_report(self) -> Health | None:
        """Emit a snapshot if ``interval_s`` elapsed; otherwise no-op."""
        now = self.clock()
        with self._lock:
            if now - self._last_report < self.interval_s:
                return None
            win, self._win = self._win, _Window()
            dt = now - self._last_report
            self._last_report = now
        return self._report(win, dt, now)

    def _report(self, win: _Window, dt: float, now: float) -> Health:
        metrics = self._metrics(win, dt)

        if not self._warmup_done and now - self._t0 >= self.warmup_s:
            self._warmup_done = True
            self._log_warmup(now)

        violations = self._violations(win, dt, metrics) if self._warmup_done else []
        state = self._next_state(win, violations)
        self._transition(state, violations, now)

        health = Health(ts=now, state=state, violations=tuple(violations), metrics=metrics)
        self._latest = health
        if self.sink is not None:
            try:
                self.sink(health)
            except Exception:
                logger.exception("%s: health sink failed", self.name)
        return health

    def _metrics(self, win: _Window, dt: float) -> dict[str, float]:
        m = {
            "ticks_resolved_hz": win.resolved / dt,
            "ticks_stepped_hz": win.stepped / dt,
            "drops_backpressure_hz": max(0, win.queued - win.stepped - self._buffer_len()) / dt,
            "drops_missing_hz": sum(win.missing.values()) / dt,
            "drops_blocked_hz": win.blocked / dt,
            "emitted_hz": win.emitted / dt,
            "step_p50_ms": _percentile(list(self._step_ms), 0.50),
            "step_p99_ms": _percentile(list(self._step_ms), 0.99),
            "tick_latency_p50_ms": _percentile(list(self._latency_ms), 0.50),
            "tick_latency_p99_ms": _percentile(list(self._latency_ms), 0.99),
            "buffer_len": float(self._buffer_len()),
            "pending_len": float(self._pending_len()),
        }
        for name, n in win.inputs.items():
            m[f"in_{name}_hz"] = n / dt
        for name, n in win.outputs.items():
            m[f"out_{name}_hz"] = n / dt
        for name, ages in self._ages.items():
            m[f"age_{name}_p99_s"] = _percentile(list(ages), 0.99)
        return m

    def _violations(self, win: _Window, dt: float, metrics: dict[str, float]) -> list[str]:
        v: list[str] = []
        for name, expected in self.expected_hz.items():
            observed = win.inputs.get(name, 0) / dt
            if observed < expected * self.rate_tolerance:
                v.append(f"input '{name}' at {observed:.1f} Hz, expected {expected:g} Hz")
        if self.min_output_hz is not None:
            emitted_hz = win.emitted / dt
            if emitted_hz < self.min_output_hz:
                v.append(f"output {emitted_hz:.1f} Hz < contract {self.min_output_hz:g} Hz")
        for name, min_hz in self.out_min_hz.items():
            port_hz = win.outputs.get(name, 0) / dt
            if port_hz < min_hz:
                v.append(f"output '{name}' at {port_hz:.1f} Hz < contract {min_hz:g} Hz")
        if self.max_drop_ratio is not None and win.queued >= self.ratio_min_samples:
            dropped = max(0, win.queued - win.stepped - self._buffer_len())
            ratio = dropped / win.queued
            if ratio > self.max_drop_ratio:
                v.append(
                    f"backpressure drop ratio {ratio:.0%} > contract {self.max_drop_ratio:.0%} "
                    f"(step not keeping up)"
                )
        if self.max_tick_latency_s is not None and len(win.latencies_s) >= self.ratio_min_samples:
            p99 = _percentile(win.latencies_s, 0.99)
            if p99 > self.max_tick_latency_s:
                v.append(
                    f"tick latency p99 {p99 * 1000:.0f} ms > contract "
                    f"{self.max_tick_latency_s * 1000:.0f} ms"
                )
        for name, n in win.missing.items():
            threshold = self.missing_ratio_by_input.get(name, self.max_missing_ratio)
            if win.resolved >= self.ratio_min_samples and n / win.resolved > threshold:
                v.append(f"input '{name}' missing on {n}/{win.resolved} ticks (stale or dead?)")
        return v

    def _next_state(self, win: _Window, violations: list[str]) -> str:
        if win.stepped > 0:
            self._zero_step_windows = 0
        elif win.queued > 0 or self._buffer_len() > 0:
            self._zero_step_windows += 1
        if win.resolved > 0:
            self._zero_resolve_windows = 0
        elif sum(win.inputs.values()) > 0 and self._pending_len() > 0:
            self._zero_resolve_windows += 1

        if not self._warmup_done:
            return self._state
        if self._zero_step_windows >= self._stall_windows:
            violations.insert(0, "ticks queued but none stepped (step stuck?)")
            return STALLED
        if self._zero_resolve_windows >= self._stall_windows:
            violations.insert(0, "inputs flowing but no ticks resolving (interpolate input dead?)")
            return STALLED
        return DEGRADED if violations else OK

    def _transition(self, state: str, violations: list[str], now: float) -> None:
        if state != self._state:
            if state == OK:
                logger.info("%s %s: recovered after %.0fs", self.name, OK, now - self._state_since)
            else:
                logger.warning("%s %s: %s", self.name, state, "; ".join(violations))
            self._state = state
            self._state_since = now
            self._last_unhealthy_log = now
        elif state != OK and now - self._last_unhealthy_log >= self.unhealthy_log_every_s:
            logger.warning(
                "%s still %s (%.0fs): %s",
                self.name,
                state,
                now - self._state_since,
                "; ".join(violations),
            )
            self._last_unhealthy_log = now

    def _log_warmup(self, now: float) -> None:
        if not self.expected_hz:
            return
        elapsed = now - self._t0
        parts = []
        for name, expected in sorted(self.expected_hz.items()):
            observed = self._total_inputs.get(name, 0) / elapsed
            note = "" if observed >= expected * self.rate_tolerance else " — LOW"
            parts.append(f"{name} {observed:.1f} Hz (expected {expected:g}{note})")
        logger.info("%s warmup: %s", self.name, ", ".join(parts))

    @property
    def state(self) -> str:
        return self._state

    @property
    def latest(self) -> Health | None:
        """The most recent snapshot, or ``None`` before the first report."""
        return self._latest

    def counters(self) -> dict[str, Any]:
        """Current window counters — for tests and debugging."""
        with self._lock:
            w = self._win
            return {
                "inputs": dict(w.inputs),
                "resolved": w.resolved,
                "queued": w.queued,
                "stepped": w.stepped,
                "missing": dict(w.missing),
            }
