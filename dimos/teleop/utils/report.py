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
stats, robot_telemetry) and emits ``report.json`` next to it. Command-link
latency is read straight off the robot's recorded ``robot_telemetry`` frames.
The math (percentiles, rate, jitter, stalls) is the same one the live HUD
uses — both go through ``stream_stats``.

JSON (not markdown) so two runs are diffable, regression-gateable in CI, and
plottable — the summary dicts are emitted verbatim.

Importable from ``TeleopRecorder.stop()`` (post-hoc on the run's own .db) or
runnable standalone over an old recording::

    python -m dimos.teleop.utils.report <path/to/recording.db>
"""

from __future__ import annotations

from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import sys
from typing import Any

import numpy as np

from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.teleop.quest.quest_types import Buttons
from dimos.teleop.utils.stream_stats import pcts
from dimos.teleop.utils.video_stats import VideoStats
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Streams the recorder declares + the dimos msg type to decode each as. Order
# here drives the order in the report.
_STREAM_TYPES = {
    "cmd_vel_stamped": TwistStamped,
    "left_controller_output": PoseStamped,
    "right_controller_output": PoseStamped,
    "teleop_buttons": Buttons,
    "video_stats": VideoStats,
}


def generate_report(db_path: Path, out_dir: Path | None = None) -> Path:
    """Write ``report_<ts>.json`` for the recording at *db_path*.

    Named after the .db stem so runs don't clobber. Output lands in *out_dir*
    if given, else next to the .db. Returns the written path. Raises if the .db
    is missing or unreadable.
    """
    if not db_path.exists():
        raise FileNotFoundError(f"Recording not found: {db_path}")
    if out_dir is None:
        out_dir = db_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    # Pull each stream's rows out of the SqliteStore + decode by typed schema.
    store = SqliteStore(path=str(db_path))
    store.start()
    try:
        records = _read_all(store)
        telemetry = _read_telemetry(store)
    finally:
        store.stop()

    # Per-message-stream → summary stats. video_stats is a separate shape.
    twist_streams = {n: r for n, r in records.items() if n != "video_stats" and r}
    summaries = {name: _summary(rs, stall_factor=3.0) for name, rs in twist_streams.items()}
    active = {n: s for n, s in summaries.items() if s.get("rate_hz")}
    video_summary = _summarize_video(records.get("video_stats", []))
    telemetry_summary = _summarize_telemetry(telemetry)

    duration_s = _run_duration(records)
    timestamp = datetime.fromtimestamp(db_path.stat().st_mtime).strftime("%Y%m%d_%H%M%S")

    report = {
        "timestamp": timestamp,
        "duration_s": round(duration_s, 3),
        "streams": active,
        "video": video_summary,
        "telemetry": telemetry_summary,
    }

    # Name the report after the .db stem so runs don't clobber and the pair
    # stays together: recording_teleop_<ts>.db → report_<ts>.json.
    suffix = db_path.stem.replace("recording_teleop", "").replace("recording", "").lstrip("_")
    report_name = f"report_{suffix}.json" if suffix else "report.json"
    report_path = out_dir / report_name
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True))
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
        out[name] = [obs.data for obs in stream]
    return out


def _read_telemetry(store: SqliteStore) -> list[dict[str, Any]]:
    """Decode the recorded ``robot_telemetry`` JSON frames (raw bytes stream).

    Each frame carries the robot's own live cmd-link stats (``cmd``: latency /
    jitter / rate, already computed by ``LiveStreamStats``) plus soc/state, so
    the report reads latency straight off these instead of recomputing it.
    """
    if "robot_telemetry" not in set(store.list_streams()):
        return []
    frames: list[dict[str, Any]] = []
    for obs in store.stream("robot_telemetry", bytes):
        try:
            frames.append(json.loads(obs.data))
        except (ValueError, TypeError):
            continue
    return frames


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

    Rate/jitter come from each message's ``.ts`` (sender stamp, clock-sync
    calibrated). Command-link latency is reported separately from the recorded
    ``robot_telemetry`` stream (see ``_summarize_telemetry``).

    Buttons lacks ``.ts``, so rate/jitter are ``None``.
    """
    count = len(records)
    tss = [float(m.ts) for m in records if getattr(m, "ts", None) is not None]

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
        "stall_count": len(stalls),
        "stall_total_s": sum(stalls) / 1000.0,
    }


def _summarize_telemetry(frames: list[dict[str, Any]]) -> dict[str, Any] | None:
    """Aggregate recorded ``robot_telemetry`` frames, or None if none recorded.

    Latency/jitter/rate come straight off the robot's own live ``cmd`` stats
    (``LiveStreamStats`` snapshots, arrival-time minus send-stamp) — no
    recomputation. soc is summarized as the run's min/last.
    """
    if not frames:
        return None

    def cmd_col(key: str) -> list[float]:
        return [
            float(f["cmd"][key])
            for f in frames
            if isinstance(f.get("cmd"), dict) and f["cmd"].get(key) is not None
        ]

    socs = [int(f["soc"]) for f in frames if f.get("soc") is not None]
    return {
        "count": len(frames),
        "latency_ms": pcts(cmd_col("latency_ms")),
        "jitter_ms": pcts(cmd_col("jitter_ms")),
        "rate_hz": pcts(cmd_col("rate_hz")),
        "soc_min": min(socs) if socs else None,
        "soc_last": socs[-1] if socs else None,
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
        # 0 when the robot isn't stamping — summarize only real readings.
        "e2e_latency_ms": pcts([v for v in col("e2e_latency_ms") if v > 0]),
        "frames_dropped": max((s.frames_dropped for s in samples), default=0),
        "freezes": max((s.freezes for s in samples), default=0),
    }


def main() -> None:
    """CLI: ``python -m dimos.teleop.utils.report <db_path>``."""
    if len(sys.argv) != 2:
        print(f"usage: python -m {__name__} <recording.db>", file=sys.stderr)
        sys.exit(2)
    out = generate_report(Path(sys.argv[1]))
    print(out)


if __name__ == "__main__":
    main()
