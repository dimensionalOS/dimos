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

"""Plot trajectory control tick JSONL exports (issue 921 P2-3).

Reads UTF-8 JSONL as documented in ``dimos/navigation/trajectory_control_tick_jsonl.md``.
Produces a figure with the primary speed-vs-divergence view plus time-series sanity panels.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


def _load_ticks(path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    with path.open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rows.append(json.loads(line))
    if not rows:
        raise SystemExit(f"No JSON lines in {path}")
    for i, row in enumerate(rows):
        ver = row.get("schema_version")
        if ver != 1:
            raise SystemExit(f"Line {i + 1}: unsupported schema_version {ver!r} (expected 1)")
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot trajectory control tick JSONL (speed vs divergence + time series)."
    )
    parser.add_argument(
        "jsonl",
        type=Path,
        help="Path to trajectory control tick export (.jsonl).",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="Write PNG to this path (default: <input_stem>_921_plot.png next to the input file).",
    )
    args = parser.parse_args()
    inp = args.jsonl.expanduser().resolve()
    if not inp.is_file():
        raise SystemExit(f"Not a file: {inp}")

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError as e:
        raise SystemExit(
            "matplotlib and numpy are required. Example: "
            "uv run --with matplotlib python scripts/plot_trajectory_control_ticks.py ..."
        ) from e

    rows = _load_ticks(inp)
    n = len(rows)

    def col(name: str) -> np.ndarray:
        return np.array([float(r[name]) for r in rows], dtype=np.float64)

    div_m = col("planar_position_divergence_m")
    speed_m_s = col("commanded_planar_speed_m_s")
    ref_t = col("ref_time_s")
    meas_t = col("meas_time_s")
    e_xt = col("e_cross_track_m")
    e_at = col("e_along_track_m")
    dt_s = col("dt_s")

    out = args.output
    if out is None:
        out = inp.parent / f"{inp.stem}_921_plot.png"
    else:
        out = out.expanduser().resolve()

    fig, axes = plt.subplots(2, 2, figsize=(10, 8), constrained_layout=True)
    ax_sd, ax_ts_div, ax_ts_speed, ax_ts_err = axes[0, 0], axes[0, 1], axes[1, 0], axes[1, 1]

    t_plot = ref_t if np.any(np.diff(ref_t) != 0) or np.max(np.abs(ref_t)) > 0 else meas_t

    # Primary: commanded planar speed vs planar position divergence (921 performance graph).
    ax_sd.scatter(div_m, speed_m_s, c=t_plot, cmap="viridis", s=36, zorder=2)
    ax_sd.plot(div_m, speed_m_s, color="0.5", linewidth=0.8, alpha=0.7, zorder=1)
    ax_sd.set_xlabel("Planar position divergence (m)")
    ax_sd.set_ylabel("Commanded planar speed (m/s)")
    ax_sd.set_title("Speed vs divergence (color = time basis)")
    ax_sd.grid(True, alpha=0.3)

    ax_ts_div.plot(t_plot, div_m, marker="o", markersize=3, linewidth=1)
    ax_ts_div.set_xlabel("ref_time_s or meas_time_s (s)")
    ax_ts_div.set_ylabel("planar_position_divergence_m")
    ax_ts_div.set_title("Divergence over time")
    ax_ts_div.grid(True, alpha=0.3)

    ax_ts_speed.plot(t_plot, speed_m_s, marker="o", markersize=3, linewidth=1, color="C1")
    ax_ts_speed.set_xlabel("ref_time_s or meas_time_s (s)")
    ax_ts_speed.set_ylabel("commanded_planar_speed_m_s")
    ax_ts_speed.set_title("Commanded speed over time")
    ax_ts_speed.grid(True, alpha=0.3)

    ax_ts_err.plot(t_plot, e_at, label="e_along_track_m", linewidth=1)
    ax_ts_err.plot(t_plot, e_xt, label="e_cross_track_m", linewidth=1)
    ax_ts_err.set_xlabel("ref_time_s or meas_time_s (s)")
    ax_ts_err.set_ylabel("error (m)")
    ax_ts_err.set_title("Along- / cross-track error and control dt")
    ax_ts_err.grid(True, alpha=0.3)
    ax_dt = ax_ts_err.twinx()
    ax_dt.plot(t_plot, dt_s, color="C2", linestyle="--", linewidth=1, alpha=0.85, label="dt_s")
    ax_dt.set_ylabel("dt_s", color="C2")
    ax_dt.tick_params(axis="y", labelcolor="C2")
    h1, l1 = ax_ts_err.get_legend_handles_labels()
    h2, l2 = ax_dt.get_legend_handles_labels()
    ax_ts_err.legend(h1 + h2, l1 + l2, loc="best", fontsize="small")

    fig.suptitle(f"Trajectory control ticks: {inp.name} ({n} samples)", fontsize=11)

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"Wrote {out}")


if __name__ == "__main__":
    try:
        main()
    except BrokenPipeError:
        sys.exit(0)
