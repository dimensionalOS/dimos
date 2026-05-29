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

"""Plot speed (|v| from pose deltas) for every attempt under the segment runs root.

One Plotly HTML output, one line per attempt_NNN/fastlio.db. X-axis is
seconds-into-recording (rec_t), Y-axis is |v| in m/s on a symlog scale so
both the bounded ~m/s regime and any divergent excursions to 100+ m/s are
visible in one chart.

Hardcoded config — bump the constants and recommit when you want a
different segment / different runs root.

Run from the dimos venv:

    cd ~/repos/dimos
    source .venv/bin/activate
    python -m dimos.hardware.sensors.lidar.fastlio2.tools.plot_segment_1330_1390
"""

from __future__ import annotations

import math
from pathlib import Path
import sqlite3
import sys

import plotly.graph_objects as go

# ---------------- Configuration (must match replay_segment_1330_1390.py) ----

RUNS_ROOT = Path("/media/dimos/USB/fastlio_recordings/segment_replays_1330_1390")
OUT_HTML = RUNS_ROOT / "segment_replays.html"

# Recording reference. The fastlio binary publishes ts in sensor-boot
# seconds when deterministic_clock=True. Add the offset to get epoch, then
# subtract REC_START_EPOCH to get rec_t.
REC_START_EPOCH = 1780020531.706
SENSOR_BOOT_EPOCH_OFFSET = 1780018948.01

T_LO_REC_SEC = 1330.0
T_HI_REC_SEC = 1390.0


# ---------------- Speed derivation ------------------------------------------


def _load_attempt(db_path: Path) -> tuple[list[float], list[float]]:
    """Return (rec_t_sec, abs_v_mps) for the attempt's fastlio_odometry rows.

    Rows with NULL pose_x/y/z (the first few while IESKF is initialising)
    are skipped. Speed is the L2 norm of the per-step pose delta divided
    by the per-step ts delta. ts is sensor-boot seconds (deterministic
    clock mode), converted to rec_t for the plot.
    """
    con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    try:
        rows = con.execute(
            "SELECT ts, pose_x, pose_y, pose_z FROM fastlio_odometry "
            "WHERE pose_x IS NOT NULL ORDER BY ts"
        ).fetchall()
    finally:
        con.close()

    if len(rows) < 2:
        return [], []

    rec_ts: list[float] = []
    speeds: list[float] = []
    prev_ts, prev_x, prev_y, prev_z = rows[0]
    for ts, x, y, z in rows[1:]:
        dt = ts - prev_ts
        if dt <= 0:
            prev_ts, prev_x, prev_y, prev_z = ts, x, y, z
            continue
        dx, dy, dz = x - prev_x, y - prev_y, z - prev_z
        v = math.sqrt(dx * dx + dy * dy + dz * dz) / dt
        epoch = ts + SENSOR_BOOT_EPOCH_OFFSET
        rec_t = epoch - REC_START_EPOCH
        rec_ts.append(rec_t)
        speeds.append(v)
        prev_ts, prev_x, prev_y, prev_z = ts, x, y, z
    return rec_ts, speeds


def _list_attempts() -> list[Path]:
    return sorted(
        (p for p in RUNS_ROOT.iterdir() if p.is_dir() and p.name.startswith("attempt_")),
        key=lambda p: p.name,
    )


# ---------------- Plot ------------------------------------------------------


def _build_figure(attempts: list[Path]) -> go.Figure:
    fig = go.Figure()
    n_traces = 0
    for attempt_dir in attempts:
        db_path = attempt_dir / "fastlio.db"
        if not db_path.exists():
            print(f"[plot_segment] skip {attempt_dir.name}: no fastlio.db", flush=True)
            continue
        rec_ts, speeds = _load_attempt(db_path)
        if not rec_ts:
            print(f"[plot_segment] skip {attempt_dir.name}: no valid odom rows", flush=True)
            continue
        peak = max(speeds)
        fig.add_trace(
            go.Scatter(
                x=rec_ts,
                y=speeds,
                mode="lines",
                name=f"{attempt_dir.name} (peak {peak:.2f} m/s)",
                line={"width": 1.0},
                hovertemplate="rec_t=%{x:.2f}s |v|=%{y:.3f} m/s<extra></extra>",
            )
        )
        n_traces += 1

    fig.update_layout(
        title=(
            f"FAST-LIO replay speed — segment [{T_LO_REC_SEC:.0f}, {T_HI_REC_SEC:.0f}] s "
            f"({n_traces} attempt{'' if n_traces == 1 else 's'})"
        ),
        xaxis_title="seconds into recording",
        yaxis_title="|v| (m/s, symlog)",
        yaxis={"type": "log"},
        hovermode="closest",
        template="plotly_white",
    )
    return fig


def main() -> int:
    if not RUNS_ROOT.exists():
        print(f"[plot_segment] runs root does not exist: {RUNS_ROOT}", file=sys.stderr)
        return 2
    attempts = _list_attempts()
    if not attempts:
        print(f"[plot_segment] no attempt_*/ dirs under {RUNS_ROOT}", file=sys.stderr)
        return 2
    fig = _build_figure(attempts)
    OUT_HTML.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(str(OUT_HTML), include_plotlyjs="cdn")
    print(f"[plot_segment] wrote {OUT_HTML}  attempts={len(attempts)}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
