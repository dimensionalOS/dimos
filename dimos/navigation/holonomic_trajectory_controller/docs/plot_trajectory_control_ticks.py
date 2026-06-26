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

"""Plot trajectory control tick JSONL exports.

Reads UTF-8 JSONL as documented in ``dimos/navigation/trajectory_control_tick_jsonl.md``.

Outputs:
- **Summary** (``*_plot.png``): cross-track vs time and speed scatter.
- **Path profile** (``*_path_profile.png``): measured arc length on the x-axis with
  |cross-track error| (red), commanded speed, reference-path curvature, and heading error
  stacked vertically so you can see where the route turns and where error spikes.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Literal

PLOT_REQUIRED_FIELDS = (
    "schema_version",
    "ref_time_s",
    "meas_time_s",
    "e_cross_track_m",
    "e_heading_rad",
    "commanded_planar_speed_m_s",
    "dt_s",
)

PATH_PROFILE_REQUIRED_FIELDS = PLOT_REQUIRED_FIELDS + (
    "ref_x_m",
    "ref_y_m",
    "ref_yaw_rad",
    "meas_x_m",
    "meas_y_m",
)

MOVING_SPEED_THRESHOLD_M_S = 0.05

# Reference-path |curvature| bins (1/m) for printed summaries on moving ticks.
CURVATURE_BIN_EDGES = (0.0, 0.05, 0.2, 0.5, float("inf"))
CURVATURE_BIN_LABELS = (
    "straight |kappa|<0.05",
    "gentle 0.05-0.2",
    "medium 0.2-0.5",
    "tight |kappa|>=0.5",
)


def _load_ticks(path: Path, *, required_fields: tuple[str, ...] = PLOT_REQUIRED_FIELDS) -> list[dict[str, object]]:
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
        missing = [field for field in required_fields if field not in row]
        if missing:
            raise SystemExit(f"Line {i + 1}: missing required fields: {', '.join(missing)}")
    return rows


def _angle_diff(from_rad: float, to_rad: float) -> float:
    return math.atan2(math.sin(to_rad - from_rad), math.cos(to_rad - from_rad))


def _col(rows: list[dict[str, object]], name: str) -> list[float]:
    return [float(row[name]) for row in rows]


def _time_basis(rows: list[dict[str, object]], ref_t, meas_t) -> object:
    import numpy as np

    return (
        ref_t
        if np.any(np.diff(ref_t) != 0) or np.max(np.abs(ref_t)) > 0
        else meas_t
    )


def _print_cross_track_summary(rows: list[dict[str, object]], inp: Path) -> None:
    moving = [
        row
        for row in rows
        if float(row["commanded_planar_speed_m_s"]) > MOVING_SPEED_THRESHOLD_M_S
    ]
    if not moving:
        print(f"{inp.name}: no moving ticks (cmd speed > {MOVING_SPEED_THRESHOLD_M_S} m/s)")
        return

    abs_ct = [abs(float(row["e_cross_track_m"])) for row in moving]
    signed_ct = [float(row["e_cross_track_m"]) for row in moving]
    abs_ct_sorted = sorted(abs_ct)
    p95_index = max(0, int(math.ceil(0.95 * len(abs_ct_sorted))) - 1)
    rms = math.sqrt(sum(v * v for v in signed_ct) / len(signed_ct))

    print(f"Cross-track summary for {inp.name} ({len(moving)} moving / {len(rows)} ticks):")
    print(f"  mean |e_cross_track_m| = {sum(abs_ct) / len(abs_ct):.4f} m")
    print(f"  max  |e_cross_track_m| = {max(abs_ct):.4f} m")
    print(f"  p95  |e_cross_track_m| = {abs_ct_sorted[p95_index]:.4f} m")
    print(f"  RMS  e_cross_track_m  = {rms:.4f} m  (signed, industry reporting style)")
    print(
        "  Note: e_cross_track is lateral offset vs the controller reference pose "
        "(lookahead on path), not arc-length progress."
    )


def _plot_cross_track(
    *,
    inp: Path,
    out: Path,
    rows: list[dict[str, object]],
    n: int,
) -> None:
    import matplotlib.pyplot as plt
    import numpy as np

    def col(name: str) -> np.ndarray:
        return np.array([float(r[name]) for r in rows], dtype=np.float64)

    speed_m_s = col("commanded_planar_speed_m_s")
    ref_t = col("ref_time_s")
    meas_t = col("meas_time_s")
    e_xt = col("e_cross_track_m")
    e_h = col("e_heading_rad")
    dt_s = col("dt_s")
    abs_e_xt = np.abs(e_xt)

    t_plot = _time_basis(rows, ref_t, meas_t)

    fig, axes = plt.subplots(2, 2, figsize=(10, 8), constrained_layout=True)
    ax_sc, ax_ts_ct, ax_ts_speed, ax_ts_h = axes[0, 0], axes[0, 1], axes[1, 0], axes[1, 1]

    ax_sc.scatter(abs_e_xt, speed_m_s, c=t_plot, cmap="viridis", s=36, zorder=2)
    ax_sc.plot(abs_e_xt, speed_m_s, color="0.5", linewidth=0.8, alpha=0.7, zorder=1)
    ax_sc.set_xlabel("|e_cross_track_m| (m)")
    ax_sc.set_ylabel("Commanded planar speed (m/s)")
    ax_sc.set_title("Speed vs |cross-track error| (color = time)")
    ax_sc.grid(True, alpha=0.3)

    ax_ts_ct.plot(t_plot, e_xt, marker="o", markersize=3, linewidth=1, color="C1", label="e_cross_track_m")
    ax_ts_ct.axhline(0.0, color="0.4", linewidth=0.8, linestyle=":")
    ax_ts_ct.set_xlabel("ref_time_s or meas_time_s (s)")
    ax_ts_ct.set_ylabel("e_cross_track_m (m)")
    ax_ts_ct.set_title("Cross-track error over time (+ = left of reference)")
    ax_ts_ct.grid(True, alpha=0.3)
    ax_ts_ct.legend(loc="best", fontsize="small")

    ax_ts_speed.plot(t_plot, speed_m_s, marker="o", markersize=3, linewidth=1, color="C0")
    ax_ts_speed.set_xlabel("ref_time_s or meas_time_s (s)")
    ax_ts_speed.set_ylabel("commanded_planar_speed_m_s")
    ax_ts_speed.set_title("Commanded speed over time")
    ax_ts_speed.grid(True, alpha=0.3)

    ax_ts_h.plot(t_plot, e_h, marker="o", markersize=3, linewidth=1, color="C3", label="e_heading_rad")
    ax_ts_h.set_xlabel("ref_time_s or meas_time_s (s)")
    ax_ts_h.set_ylabel("e_heading_rad (rad)")
    ax_ts_h.set_title("Heading error and control dt")
    ax_ts_h.grid(True, alpha=0.3)
    ax_dt = ax_ts_h.twinx()
    ax_dt.plot(t_plot, dt_s, color="C2", linestyle="--", linewidth=1, alpha=0.85, label="dt_s")
    ax_dt.set_ylabel("dt_s", color="C2")
    ax_dt.tick_params(axis="y", labelcolor="C2")
    h1, l1 = ax_ts_h.get_legend_handles_labels()
    h2, l2 = ax_dt.get_legend_handles_labels()
    ax_ts_h.legend(h1 + h2, l1 + l2, loc="best", fontsize="small")

    fig.suptitle(
        f"Cross-track tracking: {inp.name} ({n} samples)",
        fontsize=11,
    )

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150)
    plt.close(fig)


def _derive_path_profile(rows: list[dict[str, object]]) -> dict[str, object]:
    """Build arc-length, curvature, and segment-since-stop series from tick poses."""
    import numpy as np

    n = len(rows)
    ref_x = np.array(_col(rows, "ref_x_m"), dtype=np.float64)
    ref_y = np.array(_col(rows, "ref_y_m"), dtype=np.float64)
    ref_yaw = np.array(_col(rows, "ref_yaw_rad"), dtype=np.float64)
    meas_x = np.array(_col(rows, "meas_x_m"), dtype=np.float64)
    meas_y = np.array(_col(rows, "meas_y_m"), dtype=np.float64)
    speed = np.array(_col(rows, "commanded_planar_speed_m_s"), dtype=np.float64)
    abs_ct = np.abs(np.array(_col(rows, "e_cross_track_m"), dtype=np.float64))
    e_h = np.array(_col(rows, "e_heading_rad"), dtype=np.float64)

    ds_meas = np.zeros(n, dtype=np.float64)
    ds_ref = np.zeros(n, dtype=np.float64)
    dpsi_ref = np.zeros(n, dtype=np.float64)
    for i in range(1, n):
        ds_meas[i] = math.hypot(meas_x[i] - meas_x[i - 1], meas_y[i] - meas_y[i - 1])
        ds_ref[i] = math.hypot(ref_x[i] - ref_x[i - 1], ref_y[i] - ref_y[i - 1])
        dpsi_ref[i] = _angle_diff(ref_yaw[i - 1], ref_yaw[i])

    s_meas = np.cumsum(ds_meas)
    kappa_ref = np.zeros(n, dtype=np.float64)
    for i in range(1, n):
        if ds_ref[i] > 1e-6:
            kappa_ref[i] = dpsi_ref[i] / ds_ref[i]

    # Smooth reference curvature so single-tick lookahead jumps do not dominate the plot.
    smooth_window = 5
    kappa_smooth = np.zeros(n, dtype=np.float64)
    for i in range(n):
        j0 = max(0, i - smooth_window + 1)
        ds_sum = float(np.sum(ds_ref[j0 : i + 1]))
        psi_sum = float(np.sum(np.abs(dpsi_ref[j0 : i + 1])))
        if ds_sum > 1e-6:
            kappa_smooth[i] = psi_sum / ds_sum

    stopped = speed <= MOVING_SPEED_THRESHOLD_M_S
    moving = ~stopped

    segment_s = np.zeros(n, dtype=np.float64)
    turn_since_stop = np.zeros(n, dtype=np.float64)
    for i in range(1, n):
        if stopped[i]:
            segment_s[i] = 0.0
            turn_since_stop[i] = 0.0
        else:
            segment_s[i] = segment_s[i - 1] + ds_meas[i]
            turn_since_stop[i] = turn_since_stop[i - 1] + abs(dpsi_ref[i])

    return {
        "s_meas_m": s_meas,
        "segment_s_m": segment_s,
        "turn_since_stop_rad": turn_since_stop,
        "kappa_ref_rad_m": kappa_ref,
        "abs_kappa_ref_rad_m": np.abs(kappa_ref),
        "kappa_smooth_rad_m": kappa_smooth,
        "abs_kappa_smooth_rad_m": kappa_smooth,
        "commanded_speed_m_s": speed,
        "abs_cross_track_m": abs_ct,
        "e_heading_rad": e_h,
        "stopped": stopped,
        "moving": moving,
    }


def _print_path_profile_summary(rows: list[dict[str, object]], inp: Path, profile: dict[str, object]) -> None:
    import numpy as np

    moving = profile["moving"]
    assert isinstance(moving, np.ndarray)
    if not np.any(moving):
        print(f"{inp.name}: path profile - no moving ticks")
        return

    abs_ct = profile["abs_cross_track_m"]
    kappa = profile["abs_kappa_smooth_rad_m"]
    speed = profile["commanded_speed_m_s"]
    s_meas = profile["s_meas_m"]
    assert isinstance(abs_ct, np.ndarray)
    assert isinstance(kappa, np.ndarray)
    assert isinstance(speed, np.ndarray)
    assert isinstance(s_meas, np.ndarray)

    print(f"Path-profile summary for {inp.name} ({int(moving.sum())} moving ticks):")
    print("  By reference-path |curvature| (where error tends to concentrate):")
    for lo, hi, label in zip(
        CURVATURE_BIN_EDGES[:-1],
        CURVATURE_BIN_EDGES[1:],
        CURVATURE_BIN_LABELS,
        strict=True,
    ):
        mask = moving & (kappa >= lo) & (kappa < hi)
        count = int(mask.sum())
        if count == 0:
            print(f"    {label}: n=0")
            continue
        vals = abs_ct[mask]
        print(
            f"    {label}: n={count}, mean|CTE|={float(vals.mean()):.4f} m, "
            f"max|CTE|={float(vals.max()):.4f} m"
        )

    # Find worst 0.5 m windows along measured arc length (moving ticks only).
    window_m = 0.5
    s_mov = s_meas[moving]
    ct_mov_arr = abs_ct[moving]
    if len(s_mov) >= 2:
        worst: list[tuple[float, float, float]] = []
        i0 = 0
        for i1 in range(len(s_mov)):
            while i0 < i1 and s_mov[i1] - s_mov[i0] > window_m:
                i0 += 1
            if i1 > i0:
                seg = ct_mov_arr[i0 : i1 + 1]
                worst.append((float(seg.mean()), float(s_mov[i0]), float(s_mov[i1])))
        worst.sort(reverse=True)
        print(f"  Worst {window_m:.1f} m windows by mean |CTE| (measured arc length):")
        for rank, (mean_ct, s0, s1) in enumerate(worst[:5], start=1):
            print(f"    #{rank}: s={s0:.2f}-{s1:.2f} m, mean|CTE|={mean_ct:.4f} m")

    i_max = int(np.argmax(abs_ct[moving]))
    mov_idx = int(np.flatnonzero(moving)[i_max])
    print(
        f"  Global max |CTE|={float(abs_ct[mov_idx]):.4f} m at s={float(s_meas[mov_idx]):.2f} m "
        f"(|kappa|={float(kappa[mov_idx]):.3f} 1/m, speed={float(speed[mov_idx]):.2f} m/s)"
    )


def _shade_stopped_regions(axes: list[object], s: object, stopped: object) -> None:
    import numpy as np

    assert isinstance(s, np.ndarray)
    assert isinstance(stopped, np.ndarray)
    if len(s) == 0:
        return
    in_stop = False
    start_s = 0.0
    for i in range(len(s)):
        if stopped[i] and not in_stop:
            in_stop = True
            start_s = float(s[i])
        elif not stopped[i] and in_stop:
            in_stop = False
            end_s = float(s[i])
            for ax in axes:
                ax.axvspan(start_s, end_s, color="0.92", zorder=0)  # type: ignore[attr-defined]
    if in_stop:
        end_s = float(s[-1])
        for ax in axes:
            ax.axvspan(start_s, end_s, color="0.92", zorder=0)  # type: ignore[attr-defined]


def _plot_path_profile(
    *,
    inp: Path,
    out: Path,
    rows: list[dict[str, object]],
    n: int,
    profile: dict[str, object],
) -> None:
    import matplotlib.pyplot as plt
    import numpy as np

    s = profile["s_meas_m"]
    abs_ct = profile["abs_cross_track_m"]
    speed = profile["commanded_speed_m_s"]
    kappa = profile["abs_kappa_smooth_rad_m"]
    turn_stop = profile["turn_since_stop_rad"]
    e_h = profile["e_heading_rad"]
    stopped = profile["stopped"]
    moving = profile["moving"]
    assert isinstance(s, np.ndarray)
    assert isinstance(abs_ct, np.ndarray)
    assert isinstance(speed, np.ndarray)
    assert isinstance(kappa, np.ndarray)
    assert isinstance(turn_stop, np.ndarray)
    assert isinstance(e_h, np.ndarray)
    assert isinstance(stopped, np.ndarray)
    assert isinstance(moving, np.ndarray)

    fig_h = max(12.0, 0.004 * float(s[-1]) if len(s) else 12.0)
    fig = plt.figure(figsize=(11, fig_h))
    gs = fig.add_gridspec(
        5,
        1,
        height_ratios=[2.2, 1.2, 1.2, 1.0, 1.1],
        hspace=0.28,
    )
    ax_ct = fig.add_subplot(gs[0])
    ax_speed = fig.add_subplot(gs[1], sharex=ax_ct)
    ax_kappa = fig.add_subplot(gs[2], sharex=ax_ct)
    ax_turn = fig.add_subplot(gs[3], sharex=ax_ct)
    ax_scatter = fig.add_subplot(gs[4])
    axes = [ax_ct, ax_speed, ax_kappa, ax_turn]

    _shade_stopped_regions(axes, s, stopped)

    ax_ct.fill_between(s, 0.0, abs_ct, color="#d62728", alpha=0.35, zorder=1)
    ax_ct.plot(s, abs_ct, color="#b01020", linewidth=1.2, label="|e_cross_track_m|", zorder=2)
    i_peak = int(np.argmax(abs_ct))
    ax_ct.scatter([float(s[i_peak])], [float(abs_ct[i_peak])], color="#b01020", s=40, zorder=3)
    ax_ct.annotate(
        f"max {abs_ct[i_peak]:.3f} m @ {s[i_peak]:.1f} m",
        xy=(float(s[i_peak]), float(abs_ct[i_peak])),
        xytext=(8, 8),
        textcoords="offset points",
        fontsize=8,
        color="#b01020",
    )
    ax_ct.set_ylabel("|cross-track| (m)")
    ax_ct.set_title("Cross-track error along measured path (red)")
    ax_ct.grid(True, alpha=0.3)
    ax_ct.legend(loc="upper right", fontsize="small")

    ax_speed.plot(s, speed, color="C0", linewidth=1.0)
    ax_speed.set_ylabel("cmd speed (m/s)")
    ax_speed.set_title("Commanded planar speed")
    ax_speed.grid(True, alpha=0.3)

    ax_kappa.plot(s, kappa, color="C2", linewidth=1.0)
    kappa_cap = float(np.percentile(kappa[moving], 95)) * 1.2 if np.any(moving) else 1.0
    if kappa_cap > 0.05:
        ax_kappa.set_ylim(0.0, max(kappa_cap, 0.5))
    ax_kappa.set_ylabel("|kappa| (1/m)")
    ax_kappa.set_title("Reference-path curvature, 5-tick smoothed (|d yaw| / ds on ref)")
    ax_kappa.grid(True, alpha=0.3)

    ax_turn.plot(s, np.degrees(turn_stop), color="C4", linewidth=1.0)
    ax_turn.set_ylabel("deg since stop")
    ax_turn.set_title("Cumulative |ref heading change| since last stop")
    ax_turn.grid(True, alpha=0.3)

    ax_turn.set_xlabel("Measured arc length s (m)")

    sc = ax_scatter.scatter(
        kappa[moving],
        abs_ct[moving],
        c=speed[moving],
        cmap="viridis",
        s=14,
        alpha=0.75,
    )
    ax_scatter.set_xlabel("|kappa| (1/m)")
    ax_scatter.set_ylabel("|cross-track| (m)")
    ax_scatter.set_title("Curvature vs |cross-track| on moving ticks (color = cmd speed)")
    ax_scatter.grid(True, alpha=0.3)
    cbar = fig.colorbar(sc, ax=ax_scatter, fraction=0.046, pad=0.02)
    cbar.set_label("cmd speed (m/s)")

    fig.suptitle(
        f"Path profile: {inp.name} ({n} ticks, gray = stopped)",
        fontsize=11,
    )

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.subplots_adjust(left=0.08, right=0.96, top=0.94, bottom=0.06, hspace=0.38)
    fig.savefig(out, dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot trajectory control tick JSONL (cross-track and path-profile views)."
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
        help="Summary PNG path (default: <input_stem>_plot.png).",
    )
    parser.add_argument(
        "--profile-output",
        type=Path,
        default=None,
        help="Path-profile PNG (default: <input_stem>_path_profile.png).",
    )
    parser.add_argument(
        "--plot",
        choices=("summary", "profile", "both"),
        default="both",
        help="Which figures to generate (default: both).",
    )
    args = parser.parse_args()
    inp = args.jsonl.expanduser().resolve()
    if not inp.is_file():
        raise SystemExit(f"Not a file: {inp}")

    try:
        import matplotlib

        matplotlib.use("Agg")
    except ImportError as e:
        raise SystemExit(
            "matplotlib and numpy are required. Example: "
            "uv run --with matplotlib python "
            "dimos/navigation/holonomic_trajectory_controller/docs/"
            "plot_trajectory_control_ticks.py ..."
        ) from e

    plot_mode: Literal["summary", "profile", "both"] = args.plot
    need_profile = plot_mode in ("profile", "both")
    required = PATH_PROFILE_REQUIRED_FIELDS if need_profile else PLOT_REQUIRED_FIELDS
    rows = _load_ticks(inp, required_fields=required)
    n = len(rows)

    if args.output is None:
        summary_out = inp.parent / f"{inp.stem}_plot.png"
    else:
        summary_out = args.output.expanduser().resolve()

    if args.profile_output is None:
        profile_out = inp.parent / f"{inp.stem}_path_profile.png"
    else:
        profile_out = args.profile_output.expanduser().resolve()

    if plot_mode in ("summary", "both"):
        _print_cross_track_summary(rows, inp)
        _plot_cross_track(inp=inp, out=summary_out, rows=rows, n=n)
        print(f"Wrote {summary_out}")

    if need_profile:
        profile = _derive_path_profile(rows)
        _print_path_profile_summary(rows, inp, profile)
        _plot_path_profile(inp=inp, out=profile_out, rows=rows, n=n, profile=profile)
        print(f"Wrote {profile_out}")


if __name__ == "__main__":
    try:
        main()
    except BrokenPipeError:
        sys.exit(0)
