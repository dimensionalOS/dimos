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

"""Offline scoring for path-following benchmark recordings.

A SEPARATE step from acquisition: the benchmark (``benchmark.py``) records each
run as a flat per-run JSON; this reads those recordings, scores each geometrically
against its reference path (``score_run``), and emits the operating-point map +
tolerance->max-safe-speed inversion in the existing JSON metric format.

    python -m dimos.control.benchmarking.score <recordings_dir> [--tolerances 5,10,15]
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
from typing import Any

from dimos.control.benchmarking.benchmark import RunRecording
from dimos.control.benchmarking.scoring import ExecutedTrajectory, TrajectoryTick, score_run
from dimos.control.benchmarking.tuning import OperatingPoint, OperatingPointMap, invert_tolerance
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _executed_from_recording(rec: RunRecording) -> ExecutedTrajectory:
    ticks: list[TrajectoryTick] = []
    for t, x, y, yaw, cvx, cvy, cwz in rec.ticks:
        ticks.append(
            TrajectoryTick(
                t=t,
                pose=PoseStamped(
                    ts=t,
                    position=Vector3(x, y, 0.0),
                    orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
                ),
                cmd_twist=Twist(linear=Vector3(cvx, cvy, 0.0), angular=Vector3(0.0, 0.0, cwz)),
                actual_twist=Twist(),
            )
        )
    return ExecutedTrajectory(ticks=ticks, arrived=rec.arrived)


def _looks_like_recording(data: object) -> bool:
    """A run recording has a trace + a reference + the schema tag. Other JSONs in
    the directory (old operating-point maps, this scorer's own output, etc.) lack
    these and are skipped silently."""
    return isinstance(data, dict) and {"ticks", "reference", "schema"} <= data.keys()


def load_recordings(recordings_dir: str | Path) -> list[RunRecording]:
    """Load every run-recording ``*.json`` in ``recordings_dir`` (sorted),
    quietly skipping any non-recording JSON files that share the directory."""
    d = Path(recordings_dir)
    recs: list[RunRecording] = []
    skipped = 0
    for f in sorted(d.glob("*.json")):
        try:
            data = json.loads(f.read_text())
        except json.JSONDecodeError:
            skipped += 1
            continue
        if not _looks_like_recording(data):
            skipped += 1
            continue
        try:
            rec = RunRecording(**data)
            rec.speed = float(rec.speed)
            # A non-finite speed can be picked as the max "safe" speed, and an
            # empty reference scores as a perfect zero-CTE run — both would forge
            # the recommendation. Reject rather than score.
            if not math.isfinite(rec.speed) or not rec.reference or not rec.ticks:
                raise ValueError("non-finite speed, empty reference, or empty tick trace")
        except (TypeError, ValueError):
            skipped += 1
            continue
        recs.append(rec)
    if skipped:
        logger.info(f"skipped {skipped} non-recording JSON file(s) in {d}")
    return recs


def score_recordings(
    recs: list[RunRecording], tolerances_cm: list[float]
) -> tuple[OperatingPointMap, list[dict[str, Any]]]:
    """Score every recording into an operating-point map + per-run diagnostics."""
    points: list[OperatingPoint] = []
    runs: list[dict[str, Any]] = []
    for rec in recs:
        try:
            ref = rec.reference_path()
            executed = _executed_from_recording(rec)
            s = score_run(ref, executed)
            run = {
                "path": rec.path,
                "speed": rec.speed,
                "cte_max": s.cte_max,
                "cte_rms": s.cte_rms,
                "heading_err_rms": s.heading_err_rms,
                "heading_err_max": s.heading_err_max,
                "arrived": s.arrived,
                "reason": rec.reason,
                "ref": [(p[0], p[1]) for p in rec.reference],
                "exec": [(tk[1], tk[2]) for tk in rec.ticks],
            }
        except (ValueError, TypeError, IndexError) as e:
            logger.warning(
                f"skipping malformed recording (path={rec.path!r}, speed={rec.speed}): {e}"
            )
            continue
        points.append(
            OperatingPoint(
                path=rec.path,
                speed=rec.speed,
                cte_max=s.cte_max,
                cte_rms=s.cte_rms,
                arrived=s.arrived,
                heading_err_rms=s.heading_err_rms,
                heading_err_max=s.heading_err_max,
            )
        )
        runs.append(run)
    speeds = sorted({p.speed for p in points})
    inversion = invert_tolerance(points, tolerances_cm)
    return OperatingPointMap(speeds=speeds, points=points, tolerance_inversion=inversion), runs


# Diagnostic plots (optional; reused from the prior inline benchmark)


def _plot_cte_vs_speed(points: list[OperatingPoint], out: Path, robot: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 4.5))
    for name in sorted({p.path for p in points}):
        xs = [p.speed for p in points if p.path == name]
        ys = [p.cte_max * 100 for p in points if p.path == name]
        ax.plot(xs, ys, marker="o", label=name)
    ax.set_xlabel("commanded speed (m/s)")
    ax.set_ylabel("cte_max (cm)")
    ax.set_title(f"{robot}: cross-track error vs speed")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)


def _canonicalize(
    ref: list[tuple[float, float]], exec_: list[tuple[float, float]]
) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    """Rigid-transform a run into the canonical path frame: reference start ->
    (0,0), initial heading -> +x; same transform on the executed trajectory."""
    if len(ref) < 2:
        return ref, exec_
    ox, oy = ref[0]
    th = 0.0
    for px, py in ref[1:]:
        if math.hypot(px - ox, py - oy) > 1e-6:
            th = math.atan2(py - oy, px - ox)
            break
    c, s = math.cos(-th), math.sin(-th)

    def tf(pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
        return [((x - ox) * c - (y - oy) * s, (x - ox) * s + (y - oy) * c) for x, y in pts]

    return tf(ref), tf(exec_)


def _plot_xy(runs: list[dict[str, Any]], out: Path, robot: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if not runs:
        return
    paths = list(dict.fromkeys(r["path"] for r in runs))
    n = len(paths)
    cols = min(n, 2)
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(6.0 * cols, 5.0 * rows), squeeze=False)
    flat = [ax for row in axes for ax in row]
    for ax, name in zip(flat, paths, strict=False):
        prs = [r for r in runs if r["path"] == name]
        ref_drawn = False
        for r in prs:
            ref_c, ex_c = _canonicalize(r["ref"], r["exec"])
            if not ref_drawn:
                ax.plot(
                    [p[0] for p in ref_c], [p[1] for p in ref_c], "k-", lw=2.0, label="reference"
                )
                ax.plot(0.0, 0.0, "ko", ms=5)
                ref_drawn = True
            if not ex_c:
                continue
            ax.plot(
                [p[0] for p in ex_c],
                [p[1] for p in ex_c],
                lw=1.3,
                label=f"v={r['speed']:g} (cte_max={r['cte_max'] * 100:.0f}cm, "
                f"he_rms={math.degrees(r.get('heading_err_rms', 0.0)):.0f}deg"
                f"{'' if r['arrived'] else ', NOT arrived'})",
            )
        ax.set_title(name)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)
    for ax in flat[n:]:
        ax.set_visible(False)
    fig.suptitle(f"{robot}: executed trajectory vs reference path")
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)


def score_dir(
    recordings_dir: str | Path,
    *,
    tolerances_cm: list[float],
    out_path: str | Path | None = None,
    plots: bool = True,
) -> OperatingPointMap:
    """Score a directory of recordings; write the metric JSON (+ plots) beside it."""
    recs = load_recordings(recordings_dir)
    if not recs:
        raise SystemExit(f"no run recordings found in {recordings_dir}")
    opm, runs = score_recordings(recs, tolerances_cm)

    d = Path(recordings_dir)
    robot = recs[0].robot
    out_json = Path(out_path) if out_path else d / f"{robot}_benchmark_scores.json"
    out_json.write_text(json.dumps(asdict(opm), indent=2))
    logger.info(f"scored {len(recs)} run(s) -> {out_json}")
    for row in opm.tolerance_inversion:
        if row.max_speed is None:
            logger.info(
                f"  tolerance {row.tol_cm:g} cm: NO tested speed keeps every path within tolerance"
            )
        else:
            logger.info(
                f"  tolerance {row.tol_cm:g} cm: run at {row.max_speed:.2f} m/s "
                f"(binding path: {row.binding_path})"
            )

    if plots:
        try:
            _plot_cte_vs_speed(opm.points, d / f"{robot}_benchmark_cte_vs_speed.png", robot)
            _plot_xy(runs, d / f"{robot}_benchmark_xy.png", robot)
            logger.info(f"  plots -> {d}/{robot}_benchmark_*.png")
        except Exception as e:  # plotting is best-effort
            logger.warning(f"plotting failed: {e}")
    return opm


def main() -> None:
    ap = argparse.ArgumentParser(description="Score path-following benchmark recordings")
    ap.add_argument("recordings_dir", help="directory of per-run *.json recordings")
    ap.add_argument("--tolerances", default="5,10,15", help="cm, comma-separated")
    ap.add_argument(
        "--out",
        default=None,
        help="output JSON path (default: <dir>/<robot>_benchmark_scores.json)",
    )
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()

    tolerances = [float(t) for t in args.tolerances.split(",") if t.strip()]
    score_dir(
        args.recordings_dir,
        tolerances_cm=tolerances,
        out_path=args.out,
        plots=not args.no_plots,
    )


if __name__ == "__main__":
    main()
