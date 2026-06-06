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

"""Transport-stats report from a recorded teleop ``.db``.

Reads the streams a ``TeleopRecorder`` writes (twist, poses, buttons, video
stats) and emits ``report.md`` + ``latency.png`` + ``jitter.png`` next to it.
The math (percentiles, loss, reorder, stalls) is the same one the live HUD
uses — both go through ``stream_stats``.

Importable from ``TeleopRecorder.stop()`` (post-hoc on the run's own .db) or
runnable standalone over an old recording::

    python -m dimos.teleop.utils.report <path/to/recording.db>
"""

from __future__ import annotations

from collections import Counter
from datetime import datetime
from pathlib import Path
import sys
from typing import Any

import numpy as np

from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.VideoStats import VideoStats
from dimos.teleop.quest.quest_types import Buttons
from dimos.teleop.utils.stream_stats import classify_e2e, loss_pct, pcts, reorder_count
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# E2E latency acceptance bands (ms), keyed on p50.
_LOSS_THRESHOLD_PCT = 1.0

# Streams the recorder declares + the dimos msg type to decode each as. Order
# here drives the order in the report.
_STREAM_TYPES = {
    "cmd_vel_stamped": TwistStamped,
    "left_controller_output": PoseStamped,
    "right_controller_output": PoseStamped,
    "buttons": Buttons,
    "video_stats": VideoStats,
}


def generate_report(db_path: Path, out_dir: Path | None = None) -> Path:
    """Write ``report.md`` (+ PNGs) for the recording at *db_path*.

    Output lands in *out_dir* if given, else next to the .db. Returns the
    written report.md path. Raises if the .db is missing or unreadable.
    """
    db_path = Path(db_path)
    if not db_path.exists():
        raise FileNotFoundError(f"Recording not found: {db_path}")
    if out_dir is None:
        out_dir = db_path.parent
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Pull each stream's rows out of the SqliteStore + decode by typed schema.
    store = SqliteStore(path=str(db_path))
    store.start()
    try:
        records = _read_all(store)
    finally:
        store.stop()

    # Per-message-stream → summary stats. video_stats is a separate shape.
    twist_streams = {n: r for n, r in records.items() if n != "video_stats" and r}
    summaries = {name: _summary(rs, stall_factor=3.0) for name, rs in twist_streams.items()}
    active = {n: s for n, s in summaries.items() if s.get("rate_hz")}
    video_summary = _summarize_video(records.get("video_stats", []))

    duration_s = _run_duration(records)
    timestamp = datetime.fromtimestamp(db_path.stat().st_mtime).strftime("%Y%m%d_%H%M%S")

    graph_lines = _write_graphs(out_dir, active, twist_streams)
    md = _format_report(timestamp, duration_s, active, video_summary, graph_lines)

    report_path = out_dir / "report.md"
    report_path.write_text(md)
    logger.info("Report written to %s", report_path)
    return report_path


def _read_all(store: SqliteStore) -> dict[str, list[Any]]:
    """Pull every known teleop stream out of *store*, decoded to typed msgs.

    Streams not in this recording yield empty lists. Each list is ordered by
    insertion (which equals arrival order, the recorder writes synchronously
    in the message-arrival thread).
    """
    available = set(store.list_streams())
    out: dict[str, list[Any]] = {}
    for name, msg_type in _STREAM_TYPES.items():
        if name not in available:
            out[name] = []
            continue
        stream: Any = store.stream(name, msg_type)
        # Stream.__iter__ yields Observation[T]; we want (ts, payload) so the
        # stats math (which reads .ts / .seq on the payload) matches what the
        # benchmark module did with in-memory msgs.
        out[name] = [obs.data for obs in stream]
    return out


def _run_duration(records: dict[str, list[Any]]) -> float:
    """Wall-clock span across every stream in this recording."""
    all_ts: list[float] = []
    for rs in records.values():
        all_ts.extend(getattr(m, "ts", 0.0) for m in rs if getattr(m, "ts", 0.0) > 0)
    if len(all_ts) < 2:
        return 0.0
    return max(all_ts) - min(all_ts)


def _summary(records: list[Any], stall_factor: float = 3.0) -> dict[str, Any]:
    """Stats for one twist/pose/buttons stream.

    Computed from each message's ``.ts`` (sender stamp, clock-sync calibrated)
    and ``.seq`` where present. We treat .ts as the arrival time too because
    the recorder doesn't persist a separate wall-arrival stamp — for these
    streams in practice the recorder writes within microseconds of arrival, so
    inter-stamp deltas track inter-arrival deltas closely.

    Buttons lacks ``.ts``/``.seq``, so rate/jitter/loss are all ``None``.
    """
    count = len(records)
    tss = [float(m.ts) for m in records if getattr(m, "ts", 0.0) > 0]
    seqs = [int(m.seq) for m in records if getattr(m, "seq", 0)]

    intervals_ms = (np.diff(sorted(tss)) * 1000.0).tolist() if len(tss) >= 2 else []
    span = (tss[-1] - tss[0]) if len(tss) >= 2 else 0.0

    stalls: list[float] = []
    if intervals_ms:
        stall_thresh = stall_factor * float(np.median(intervals_ms))
        stalls = [iv for iv in intervals_ms if iv > stall_thresh]

    return {
        "count": count,
        "rate_hz": (len(tss) - 1) / span if span > 0 else None,
        "jitter_ms": pcts(intervals_ms),
        "loss_pct": loss_pct(seqs),
        "reorder_count": reorder_count(seqs),
        "stall_count": len(stalls),
        "stall_total_s": sum(stalls) / 1000.0,
        # No wall-arrival stored, so report .ts-based span as E2E is moot
        # here. The live HUD has the real E2E (it knows wall − sender ts).
        "e2e_ms": None,
    }


def _summarize_video(samples: list[VideoStats]) -> dict[str, Any] | None:
    """Aggregate per-sample VideoStats into report figures, or None.

    fps/kbps/loss/jbuf/decode → p50+p95 percentiles. Resolution → modal WxH.
    dropped/freezes → run totals (the operator's monotonic counters).
    """
    if not samples:
        return None

    def col(attr: str) -> list[float]:
        return [float(getattr(s, attr)) for s in samples]

    resolutions = [f"{s.width}x{s.height}" for s in samples if s.width and s.height]
    resolution = Counter(resolutions).most_common(1)[0][0] if resolutions else "n/a"

    return {
        "count": len(samples),
        "resolution": resolution,
        "fps": pcts(col("fps")),
        "kbps": pcts(col("kbps")),
        "loss_pct": pcts(col("loss_pct")),
        "jitter_buffer_ms": pcts(col("jitter_buffer_ms")),
        "decode_ms": pcts(col("decode_ms")),
        "frames_dropped": max((s.frames_dropped for s in samples), default=0),
        "freezes": max((s.freezes for s in samples), default=0),
    }


def _write_graphs(
    out_dir: Path,
    active: dict[str, dict[str, Any]],
    records: dict[str, list[Any]],
) -> list[str]:
    """Render ``latency.png`` + ``jitter.png`` into *out_dir*.

    Returns markdown lines that embed them, or an empty list if there's
    nothing to plot or matplotlib is unavailable (the report still writes).
    """
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        logger.warning("matplotlib unavailable — report has no graphs")
        return []

    refs: list[str] = []

    jitter = []
    for name in active:
        msgs = records.get(name, [])
        tss = sorted(float(m.ts) for m in msgs if getattr(m, "ts", 0.0) > 0)
        if len(tss) >= 2:
            jitter.append((name, (np.diff(tss) * 1000.0).tolist()))

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
    timestamp: str,
    duration_s: float,
    active: dict[str, dict[str, Any]],
    video: dict[str, Any] | None,
    graph_lines: list[str],
) -> str:
    # If an external tool (e.g. data/notes/benchmarks/netem/apply.sh) left a
    # profile name at this path, record it in the report header.
    netem_profile: str | None = None
    try:
        netem_profile = Path("/tmp/dimos_netem_profile").read_text().strip() or None
    except OSError:
        pass

    lines = [
        "# Hosted Teleop Recording Report",
        "",
        f"- **Timestamp:** {timestamp}",
        f"- **Duration:** {duration_s:.1f} s",
        f"- **Active streams:** {len(active)}",
        *([f"- **netem profile:** {netem_profile}"] if netem_profile else []),
        "",
        "> Generated from the recording's `.db` at session stop. Stream stats "
        "are computed from each message's sender timestamp (clock-sync "
        "calibrated) and monotonic `seq`. Rate, jitter, loss are clock-"
        "independent; video stats come from the operator's `getStats()`.",
        "",
    ]
    if not active:
        lines.append("_No messages received on any stream._")
        lines += _video_lines(video)
        return "\n".join(lines) + "\n"

    for name, s in active.items():
        jitter = s["jitter_ms"]
        loss = s["loss_pct"]

        checks: list[str] = []
        if loss is not None:
            checks.append(f"loss {'PASS' if loss < _LOSS_THRESHOLD_PCT else 'WARN'} ({loss:.2f}%)")
        e2e = s["e2e_ms"]
        if e2e is not None:
            checks.append(f"E2E {classify_e2e(e2e['p50'])} (p50 {e2e['p50']:.0f}ms)")

        loss_line = f"{loss:.2f}%" if loss is not None else "n/a (no seq)"
        jitter_line = (
            f"- Jitter (ms): p50 {jitter['p50']:.1f} / p95 {jitter['p95']:.1f} "
            f"/ p99 {jitter['p99']:.1f} / max {jitter['max']:.1f}"
            if jitter
            else "- Jitter: n/a (need ≥2 messages)"
        )

        lines += [
            f"## {name}",
            "",
            f"- Messages: {s['count']}",
            f"- Rate: {s['rate_hz']:.2f} Hz" if s["rate_hz"] else "- Rate: n/a",
            jitter_line,
            f"- Loss: {loss_line}",
            f"- Reorder: {s['reorder_count']}",
            f"- Stalls: {s['stall_count']} ({s['stall_total_s']:.2f} s total)",
            f"- **Checks:** {', '.join(checks) if checks else 'n/a'}",
            "",
        ]
    lines += _video_lines(video)
    lines += graph_lines
    return "\n".join(lines) + "\n"


def _video_lines(video: dict[str, Any] | None) -> list[str]:
    """Render the operator-side video health section, or a hint if absent.

    These come from the operator's ``pc.getStats()`` (receive side) relayed
    over ``state_reliable`` — the robot's send side can't see what actually
    arrived. Empty when no operator was streaming video during the run.
    """
    if not video:
        return [
            "## Video (operator receive-side)",
            "",
            "_No video_stats received — connect an operator with video to capture them._",
            "",
        ]

    def pp(stats: dict[str, float] | None, unit: str = "") -> str:
        if not stats:
            return "n/a"
        return f"p50 {stats['p50']:.1f}{unit} / p95 {stats['p95']:.1f}{unit}"

    return [
        "## Video (operator receive-side)",
        "",
        f"- Samples: {video['count']}",
        f"- Resolution (modal): {video['resolution']}",
        f"- FPS: {pp(video['fps'])}",
        f"- Bitrate: {pp(video['kbps'], ' kbps')}",
        f"- Packet loss: {pp(video['loss_pct'], '%')}",
        f"- Jitter buffer: {pp(video['jitter_buffer_ms'], ' ms')}",
        f"- Decode time: {pp(video['decode_ms'], ' ms')}",
        f"- Frames dropped (total): {video['frames_dropped']}",
        f"- Freezes (total): {video['freezes']}",
        "",
    ]


def main() -> None:
    """CLI: ``python -m dimos.teleop.utils.report <db_path>``."""
    if len(sys.argv) != 2:
        print(f"usage: python -m {__name__} <recording.db>", file=sys.stderr)
        sys.exit(2)
    out = generate_report(Path(sys.argv[1]))
    print(out)


if __name__ == "__main__":
    main()


__all__ = ["generate_report"]
