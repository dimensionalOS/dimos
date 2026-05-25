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

"""Hosted teleop benchmarking module (Phase 1).

Sibling of ``HostedTeleopRecorder`` — same ``In`` ports, but instead of writing
a SQLite recording it accumulates transport statistics, prints a live console
summary while running, and writes ``report.md`` + latency/jitter PNGs on stop.

Phase 1 scope (see ``data/notes/hosted_teleop_benchmarking_plan.md``):
clock-independent metrics — rate, inter-arrival jitter, loss, reorder, stalls —
plus best-effort *uncalibrated* end-to-end latency.

Compose at the CLI::

    dimos run teleop-hosted-go2   teleop-benchmark
    dimos run teleop-hosted-xarm7 teleop-benchmark
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import threading
import time
from typing import Any, NamedTuple

import numpy as np
from reactivex.disposable import Disposable

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.teleop.quest.quest_types import Buttons
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# E2E latency acceptance bands (ms), keyed on p50. See benchmarking plan §2.
_E2E_BANDS = [(50.0, "excellent"), (100.0, "good"), (150.0, "usable")]
_LOSS_THRESHOLD_PCT = 1.0


def _pcts(values: list[float]) -> dict[str, float] | None:
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


def _loss_pct(seqs: list[int]) -> float | None:
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


def _reorder_count(seqs: list[int]) -> int:
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


def _classify_e2e(p50_ms: float) -> str:
    """Map an E2E p50 latency to an acceptance band label."""
    if p50_ms < 0:
        return "clock skew"
    for threshold, label in _E2E_BANDS:
        if p50_ms < threshold:
            return label
    return "degraded"


class _Record(NamedTuple):
    """One message arrival.

    ``perf`` — monotonic arrival clock (rate/jitter). ``wall`` — wall-clock
    arrival (E2E vs ``ts``). ``ts`` — sender stamp, None if absent. ``seq`` —
    sender's monotonic counter, None when unset.
    """

    perf: float
    wall: float
    ts: float | None
    seq: int | None


class StreamStats:
    """Rolling-window + whole-run statistics for a single teleop stream.

    One instance per ``In`` port. ``record()`` is called from the port's
    subscribe callback (transport thread); ``live_summary()`` is read by the
    printer thread and ``final_summary()`` by ``stop()`` — so access to the
    shared buffers is guarded by an internal lock.
    """

    def __init__(self, stall_factor: float = 3.0) -> None:
        """``stall_factor`` — an arrival gap longer than ``stall_factor`` x the
        median inter-arrival interval counts as a stall.
        """
        self.stall_factor = stall_factor
        self._lock = threading.Lock()
        self._records: list[_Record] = []

    def record(self, msg: Any) -> None:
        """Record the arrival of one message.

        Captures the monotonic + wall arrival clocks and, when present, the
        sender stamp ``msg.ts`` and monotonic counter ``msg.seq`` (0 == unset).
        """
        perf = time.perf_counter()
        wall = time.time()
        ts = getattr(msg, "ts", None)
        seq = getattr(msg, "seq", None)
        record = _Record(
            perf=perf,
            wall=wall,
            ts=ts if (ts and ts > 0) else None,
            seq=seq if (seq and seq > 0) else None,
        )
        with self._lock:
            self._records.append(record)

    def _summary(self, records: list[_Record]) -> dict[str, Any]:
        """Compute stats over an already-sliced, arrival-ordered record list.

        Always returns the same keys; metrics needing ≥2 samples are ``None``
        (or 0 for counts) when the record list is too short.
        """
        count = len(records)
        perfs = [r.perf for r in records]
        span = perfs[-1] - perfs[0] if count >= 2 else 0.0
        intervals_ms = (np.diff(perfs) * 1000.0).tolist() if count >= 2 else []

        stalls: list[float] = []
        if intervals_ms:
            stall_thresh = self.stall_factor * float(np.median(intervals_ms))
            stalls = [iv for iv in intervals_ms if iv > stall_thresh]

        seqs = [r.seq for r in records if r.seq]
        e2e_ms = [(r.wall - r.ts) * 1000.0 for r in records if r.ts]

        return {
            "count": count,
            "rate_hz": (count - 1) / span if span > 0 else None,
            "jitter_ms": _pcts(intervals_ms),
            "loss_pct": _loss_pct(seqs),
            "reorder_count": _reorder_count(seqs),
            "stall_count": len(stalls),
            "stall_total_s": sum(stalls) / 1000.0,
            "e2e_ms": _pcts(e2e_ms),
        }

    def live_summary(self, window_s: float) -> dict[str, Any]:
        """Windowed stats over roughly the last *window_s* seconds.

        Returns the same shape as :meth:`final_summary` but computed only over
        recent records — enough for the one-line console summary.
        """
        cutoff = time.perf_counter() - window_s
        with self._lock:
            window = [r for r in self._records if r.perf >= cutoff]
        return self._summary(window)

    def final_summary(self) -> dict[str, Any]:
        """Whole-run aggregate stats for the report."""
        with self._lock:
            records = list(self._records)
        return self._summary(records)

    def series(self) -> dict[str, list[float]]:
        """Raw whole-run series for the report sparklines (ms units)."""
        with self._lock:
            records = list(self._records)
        perfs = [r.perf for r in records]
        return {
            "intervals_ms": (np.diff(perfs) * 1000.0).tolist() if len(perfs) >= 2 else [],
            "e2e_ms": [(r.wall - r.ts) * 1000.0 for r in records if r.ts],
        }


class TeleopBenchmarkConfig(ModuleConfig):
    """Config for :class:`TeleopBenchmarkModule`."""

    print_hz: float = 1.0
    """Cadence of the live console summary line."""

    window_s: float = 5.0
    """Rolling window the live line is computed over."""

    stall_factor: float = 3.0
    """Arrival gap > factor x median interval counts as a stall."""

    out_dir: str = "data/hosted_teleop/reports"
    """Reports are written to ``out_dir/<timestamp>/`` (relative to repo root)."""


class TeleopBenchmarkModule(Module):
    """Benchmarks hosted teleop streams — live console summary + on-stop report.

    Subscribes to whatever ``In`` ports the connected blueprint feeds (VR
    controller poses + buttons for xarm7 sim, ``cmd_vel_stamped`` for Go2);
    unconnected ports simply stay empty. Mirrors ``HostedTeleopRecorder``'s
    port set so the two are interchangeable at the CLI.
    """

    config: TeleopBenchmarkConfig

    right_controller_output: In[PoseStamped]
    left_controller_output: In[PoseStamped]
    buttons: In[Buttons]
    cmd_vel_stamped: In[TwistStamped]

    def __init__(self, **kwargs: Any) -> None:
        """Initialise per-stream stats holders and threading primitives."""
        super().__init__(**kwargs)
        self._stats: dict[str, StreamStats] = {}
        self._stop_event = threading.Event()
        self._printer_thread: threading.Thread | None = None
        self._start_perf: float | None = None
        self._start_timestamp: str | None = None
        self._report_lock = threading.Lock()
        self._report_written = False

    @rpc
    def start(self) -> None:
        """Subscribe to every connected ``In`` port and start the printer thread."""
        super().start()

        if not self.inputs:
            logger.warning("TeleopBenchmarkModule has no In ports — nothing to benchmark")
            return

        for name, port in self.inputs.items():
            stats = StreamStats(stall_factor=self.config.stall_factor)
            self._stats[name] = stats
            unsubscribe = port.subscribe(lambda msg, n=name: self._on_message(n, msg))
            self.register_disposable(Disposable(unsubscribe))
            logger.info("Benchmarking %s (%s)", name, port.type.__name__)

        self._start_perf = time.perf_counter()
        self._start_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._report_written = False
        self._stop_event.clear()
        self._printer_thread = threading.Thread(
            target=self._printer_loop, daemon=True, name="TeleopBenchmarkPrinter"
        )
        self._printer_thread.start()

    @rpc
    def stop(self) -> None:
        """Stop the printer thread, compute final stats, write the report.

        Guarded against double-invocation — teardown can fire ``stop()`` more
        than once (e.g. Ctrl-C + module-coordinator shutdown), and we don't
        want a second report folder one second after the first.
        """
        self._stop_event.set()
        if self._printer_thread is not None:
            self._printer_thread.join(timeout=2.0)
            self._printer_thread = None
        with self._report_lock:
            if self._report_written:
                super().stop()
                return
            self._report_written = True
        try:
            self._write_report()
        except Exception:
            logger.exception("Failed to write benchmark report")
        super().stop()

    def _on_message(self, name: str, msg: Any) -> None:
        """Port subscribe callback — route *msg* into the stream's stats."""
        self._stats[name].record(msg)

    def _printer_loop(self) -> None:
        """Print a one-line live summary per active stream every ``1/print_hz`` s."""
        interval = 1.0 / max(self.config.print_hz, 0.1)
        while not self._stop_event.is_set():
            for name, stats in self._stats.items():
                s = stats.live_summary(self.config.window_s)
                if not s.get("rate_hz"):
                    continue
                jitter = s["jitter_ms"]
                loss = s["loss_pct"]
                loss_str = f"{loss:.1f}%" if loss is not None else "n/a"
                # Uncalibrated E2E (wall - sender ts): only present on stamped
                # streams (cmd_vel_stamped, poses), and reads noisy / can go
                # negative without clock-offset correction — hence the "~"
                # prefix. Omitted entirely when there's no ts (e.g. buttons).
                e2e = s["e2e_ms"]
                e2e_str = f" | ~e2e p50 {e2e['p50']:.1f}ms" if e2e else ""
                print(
                    f"[benchmark] {name}: {s['rate_hz']:.1f}Hz | "
                    f"jitter p95 {jitter['p95']:.1f}ms | loss {loss_str}{e2e_str} | "
                    f"n={s['count']}",
                    flush=True,
                )
            self._stop_event.wait(interval)

    def _write_report(self) -> None:
        """Write the run's ``report.md`` plus latency/jitter PNGs alongside it.

        Lands in ``config.out_dir/<timestamp>/`` under the repo root. Includes
        per-stream final stats, the uncalibrated-E2E caveat, and a pass/fail
        check against the acceptance thresholds from the benchmarking plan.
        """
        # Use the start-of-run timestamp (captured in start()) so the folder
        # name reflects when the run began, not when stop() fired.
        timestamp = self._start_timestamp or datetime.now().strftime("%Y%m%d_%H%M%S")
        out_root = Path(self.config.out_dir)
        if not out_root.is_absolute():
            out_root = DIMOS_PROJECT_ROOT / out_root
        out_dir = out_root / timestamp
        out_dir.mkdir(parents=True, exist_ok=True)

        duration_s = (time.perf_counter() - self._start_perf) if self._start_perf else 0.0
        summaries = {name: stats.final_summary() for name, stats in self._stats.items()}
        active = {n: s for n, s in summaries.items() if s.get("rate_hz")}

        graph_lines = self._write_graphs(out_dir, list(active))
        report_path = out_dir / "report.md"
        report_path.write_text(self._format_report(timestamp, duration_s, active, graph_lines))
        logger.info("Benchmark report written to %s", report_path)

    def _write_graphs(self, out_dir: Path, names: list[str]) -> list[str]:
        """Render ``latency.png`` + ``jitter.png`` into *out_dir*.

        Returns the markdown lines that embed them, or an empty list if there's
        nothing to plot or matplotlib is unavailable (the report still writes).
        """
        try:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception:
            logger.warning("matplotlib unavailable — benchmark report has no graphs")
            return []

        series = {n: self._stats[n].series() for n in names}
        refs: list[str] = []

        latency = [(n, series[n]["e2e_ms"]) for n in names if series[n]["e2e_ms"]]
        if latency:
            fig, ax = plt.subplots(figsize=(9, 3.2))
            for n, e2e in latency:
                ax.plot(e2e, linewidth=0.8, label=n)
            ax.axhline(0, color="grey", linewidth=0.6)
            ax.set(
                xlabel="message #",
                ylabel="E2E latency (ms)",
                title="End-to-end latency (uncalibrated)",
            )
            ax.legend(fontsize=8)
            fig.tight_layout()
            fig.savefig(out_dir / "latency.png", dpi=110)
            plt.close(fig)
            refs += ["![latency](latency.png)", ""]

        jitter = [(n, series[n]["intervals_ms"]) for n in names if series[n]["intervals_ms"]]
        if jitter:
            fig, ax = plt.subplots(figsize=(9, 3.2))
            for n, intervals in jitter:
                ax.hist(intervals, bins=40, alpha=0.6, label=n)
            ax.set(
                xlabel="inter-arrival interval (ms)",
                ylabel="count",
                title="Inter-arrival jitter",
            )
            ax.legend(fontsize=8)
            fig.tight_layout()
            fig.savefig(out_dir / "jitter.png", dpi=110)
            plt.close(fig)
            refs += ["![jitter](jitter.png)", ""]

        return ["## Graphs", "", *refs] if refs else []

    def _format_report(
        self,
        timestamp: str,
        duration_s: float,
        active: dict[str, dict[str, Any]],
        graph_lines: list[str],
    ) -> str:
        """Render the markdown report body for the active (non-empty) streams."""
        # If an external tool (e.g. data/notes/benchmarks/netem/apply.sh)
        # left a profile name at this path, record it in the report header
        # so the conditions of the run are part of the artifact itself.
        netem_profile: str | None = None
        try:
            netem_profile = Path("/tmp/dimos_netem_profile").read_text().strip() or None
        except OSError:
            pass

        lines = [
            "# Hosted Teleop Benchmark Report",
            "",
            f"- **Timestamp:** {timestamp}",
            f"- **Duration:** {duration_s:.1f} s",
            f"- **Active streams:** {len(active)}",
            *([f"- **netem profile:** {netem_profile}"] if netem_profile else []),
            "",
            "> E2E latency is **uncalibrated** — browser/robot clocks are not "
            "synced (Phase 1.5 adds a clock-sync handshake). Treat it as "
            "indicative only. Rate, jitter, loss and stalls are clock-independent.",
            "",
        ]
        if not active:
            lines.append("_No messages received on any stream._")
            return "\n".join(lines) + "\n"

        for name, s in active.items():
            jitter = s["jitter_ms"]
            e2e = s["e2e_ms"]
            loss = s["loss_pct"]

            checks = []
            if loss is not None:
                checks.append(
                    f"loss {'PASS' if loss < _LOSS_THRESHOLD_PCT else 'WARN'} ({loss:.2f}%)"
                )
            if e2e is not None:
                checks.append(f"E2E {_classify_e2e(e2e['p50'])} (p50 {e2e['p50']:.0f}ms)")

            loss_line = f"{loss:.2f}%" if loss is not None else "n/a (no seq)"
            if e2e is not None:
                e2e_line = (
                    f"- E2E latency (ms, uncalibrated): p50 {e2e['p50']:.1f} "
                    f"/ p95 {e2e['p95']:.1f} / p99 {e2e['p99']:.1f} / max {e2e['max']:.1f}"
                )
            else:
                e2e_line = "- E2E latency: n/a (no sender stamp)"

            lines += [
                f"## {name}",
                "",
                f"- Messages: {s['count']}",
                f"- Rate: {s['rate_hz']:.2f} Hz",
                f"- Jitter (ms): p50 {jitter['p50']:.1f} / p95 {jitter['p95']:.1f} "
                f"/ p99 {jitter['p99']:.1f} / max {jitter['max']:.1f}",
                f"- Loss: {loss_line}",
                f"- Reorder: {s['reorder_count']}",
                f"- Stalls: {s['stall_count']} ({s['stall_total_s']:.2f} s total)",
                e2e_line,
                f"- **Checks:** {', '.join(checks) if checks else 'n/a'}",
                "",
            ]
        lines += graph_lines
        return "\n".join(lines) + "\n"


__all__ = [
    "TeleopBenchmarkConfig",
    "TeleopBenchmarkModule",
]
