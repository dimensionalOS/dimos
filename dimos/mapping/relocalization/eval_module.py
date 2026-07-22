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

"""RelocEval -- the in-process collector ``dimos run --eval`` attaches, plus the
pure analysis it shares with the offline held-out driver.

WHAT IT MEASURES. A run publishes world->map fixes to ``/tf`` as the robot drives.
RelocEval listens on the real streams (``/tf`` for the transform, ``/odom`` for the
path), so trajectory, fix count, coverage and first-fix all come from the streams --
LIVE and under ``--replay`` alike. The run log is a SUPPLEMENT only: each accept's
WINNING PRIOR and fitness (the TF carries neither) plus the per-cycle proposal
census live there. Without it, trajectory + counts still stand; per-source labels
degrade to ``unknown``.

HELD-OUT ACCURACY. Premap + marker map come from run A; run B is a DIFFERENT
traversal of the same scene. A fix is ``world_B_T_map_A`` -- in A's map frame, while
B's PGO truth lives in B's independent map frame, so a raw distance is inflated by
the map_A<->map_B offset. ``resolve_heldout_alignment`` recovers the rigid
``map_B_T_map_A`` from markers surveyed in both frames (Umeyama), making per-source
``med_err`` a real metre value. Without those shared references the error prints
``-`` with the reason, never a fabricated number (a house non-negotiable).

The Module always runs "live" (no in-process truth -> med_err/success omitted); the
offline driver runs "held_out" (columns shown, Umeyama-aligned or ``-``). Both
render through the SAME ``format_report``/``plot_trajectory``/``write_report``, so
the two paths cannot drift.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import re
import time
from typing import Any

import numpy as np
import yaml

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import get_run_log_dir, setup_logger

logger = setup_logger()

FRAME_WORLD = "world"
FRAME_MAP = "map"
TF_TOPIC = "/tf"  # RelocalizationModule republishes world->map here (LCMTF -> /tf)
ODOM_TOPIC = "/odom"  # recording-timestamped PoseStamped, preserved through replay

SOURCES: tuple[str, ...] = ("ransac", "fiducial", "last_pose")
SOURCE_COLORS: dict[str, str] = {
    "ransac": "#2b6cb0",
    "fiducial": "#dd6b20",
    "last_pose": "#718096",
    "unknown": "#a0aec0",
}
SUCCESS_T_M = 1.0  # held-out translation gate (m); matches score_replay.SUCCESS_T_M
_TF_DEDUP_EPS_M = 1e-4  # two world->map translations within this are the same accept
_TF_DEDUP_EPS_RAD = 1e-4  # ...and within this rotation (a yaw-only fix keeps the origin)
_COLLINEAR_RATIO = 1e-2  # shared-tag 2nd/1st singular value below this -> collinear
_TAG_SPREAD_MIN_M = 1e-6  # principal spread below this -> the shared tags coincide


# --------------------------------------------------------------------------- #
# Log parsing (supplement) -- one key-based parser for BOTH the console
# .replay_run.log and the structured main.jsonl. Key-based (not positional) so it
# survives `source=` moving to the front of the health line.
# --------------------------------------------------------------------------- #
_TOD_RE = re.compile(r"^(\d\d):(\d\d):(\d\d)\.(\d+)")  # console time-of-day prefix
_KEY_RE: dict[str, re.Pattern[str]] = {
    "fitness": re.compile(r"fitness=([0-9.]+)"),
    "time_cost_s": re.compile(r"time_cost=([0-9.]+)s"),
    "n_pts": re.compile(r"n_pts=(\d+)"),
    "source": re.compile(r"source=(\S+)"),
    "published_t": re.compile(r"published_t=\[([^\]]+)\]"),
}
_COUNT_RE = re.compile(r"\b(ransac|fiducial|last_pose)=(\d+)")


@dataclass
class HealthLine:
    """One accepted relocalize, parsed from a ``relocalize:`` health log line."""

    source: str
    fitness: float
    published_t: tuple[float, float, float]  # world_T_map translation (the TF join key)


def _line_event(line: str) -> str:
    """The message text of a log line, from either format: the ``event`` field of
    a main.jsonl JSON object, else the raw console line."""
    stripped = line.strip()
    if stripped.startswith("{"):
        try:
            obj = json.loads(stripped)
        except json.JSONDecodeError:
            return line
        event = obj.get("event")
        return event if isinstance(event, str) else line
    return line


def _floats(csv: str) -> list[float]:
    return [float(x) for x in csv.split(",")]


def parse_health_lines(log_text: str) -> list[HealthLine]:
    """Every accepted fix (source + fitness + published translation). Gated on
    ``published_t=``, which only accept lines carry -- never rejects/census."""
    out: list[HealthLine] = []
    for raw in log_text.splitlines():
        event = _line_event(raw)
        if "relocalize:" not in event or "published_t=" not in event:
            continue
        src = _KEY_RE["source"].search(event)
        fit = _KEY_RE["fitness"].search(event)
        pub = _KEY_RE["published_t"].search(event)
        if pub is None or fit is None:
            continue
        px, py, pz = _floats(pub.group(1))
        out.append(
            HealthLine(
                source=src.group(1) if src else "ransac",
                fitness=float(fit.group(1)),
                published_t=(px, py, pz),
            )
        )
    return out


def parse_census(log_text: str) -> list[dict[str, int]]:
    """One dict of proposal counts per ``relocalize candidates:`` cycle."""
    out: list[dict[str, int]] = []
    for raw in log_text.splitlines():
        event = _line_event(raw)
        if "relocalize candidates:" not in event:
            continue
        out.append({name: int(n) for name, n in _COUNT_RE.findall(event)})
    return out


def count_rejects(log_text: str) -> int:
    return sum("relocalize rejected:" in _line_event(raw) for raw in log_text.splitlines())


def resolve_run_log(run_log_file: str | None) -> Path | None:
    """The active run's log: an explicit override, else this run's main.jsonl
    (set_run_log_dir wires every worker to it). None if neither resolves."""
    if run_log_file:
        p = Path(run_log_file)
        return p if p.exists() else None
    run_dir = get_run_log_dir()
    if run_dir is not None:
        jsonl = Path(run_dir) / "main.jsonl"
        if jsonl.exists():
            return jsonl
    return None


# --------------------------------------------------------------------------- #
# Held-out frame alignment (Umeyama)
# --------------------------------------------------------------------------- #
def umeyama_alignment(
    src_xyz: np.ndarray, dst_xyz: np.ndarray, *, with_scale: bool = False
) -> tuple[np.ndarray, np.ndarray, float]:
    """Least-squares transform ``dst ~= scale * R @ src + t`` from corresponding
    points. Umeyama 1991: https://web.stanford.edu/class/cs273/refs/umeyama.pdf
    (the SVD reflection fix ``diag(1,1,det(U)det(V))`` is the point of using it over
    a bare Kabsch). Two independent metric PGO frames differ by a rigid motion, so
    ``with_scale`` defaults False; scale is still returned as a health signal (far
    from 1.0 means the correspondences are wrong).

    Args:
        src_xyz: (N, 3) source points (here: marker centres in map_A).
        dst_xyz: (N, 3) destination points (here: the same markers in map_B).
    Returns:
        (R 3x3, t 3, scale) with ``dst ~= scale * R @ src + t``.
    Raises:
        ValueError: fewer than 3 correspondences (a rigid frame needs 3).
    """
    src = np.asarray(src_xyz, dtype=float)
    dst = np.asarray(dst_xyz, dtype=float)
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 3:
        raise ValueError(f"umeyama: need matching (N,3) arrays, got {src.shape} vs {dst.shape}")
    n = src.shape[0]
    if n < 3:
        raise ValueError(f"umeyama: need >=3 correspondences for a rigid frame, got {n}")

    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_c = src - src_mean
    dst_c = dst - dst_mean
    cov = (dst_c.T @ src_c) / n
    u_mat, sing, vt_mat = np.linalg.svd(cov)
    d = np.ones(3)
    if np.linalg.det(u_mat) * np.linalg.det(vt_mat) < 0:
        d[2] = -1.0  # reflection fix -- keep a proper rotation (det = +1)
    rot = u_mat @ np.diag(d) @ vt_mat
    if with_scale:
        var_src = (src_c**2).sum() / n
        scale = float((sing * d).sum() / var_src) if var_src > 0 else 1.0
    else:
        scale = 1.0
    trans = dst_mean - scale * rot @ src_mean
    return rot, trans, scale


def _rt_to_matrix(rot: np.ndarray, trans: np.ndarray) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = trans
    return m


def _degenerate_reason(pts: np.ndarray, frame: str) -> str | None:
    """Why this shared-tag set cannot pin a rotation, or None if it can. sv[0] is the
    point cloud's principal extent (m), sv[1] the spread off that axis: coincident
    points have no extent, a line has no spread off it, and either leaves rotation
    free about that axis -- Umeyama's SVD then returns an ARBITRARY proper rotation
    with a ~0 residual, and errors from it would read as real accuracy."""
    sv = np.linalg.svd(pts - pts.mean(axis=0), compute_uv=False)
    if sv[0] <= _TAG_SPREAD_MIN_M:
        return f"coincide in {frame} (spread {sv[0]:.2e} m)"
    if sv[1] / sv[0] < _COLLINEAR_RATIO:
        return f"are (near-)collinear in {frame} (spread ratio {sv[1] / sv[0]:.2e})"
    return None


def resolve_heldout_alignment(
    markers_map_a: dict[int, tuple[float, float, float]],
    markers_map_b: dict[int, tuple[float, float, float]] | None,
) -> tuple[np.ndarray | None, str]:
    """The rigid ``map_B_T_map_A`` from markers surveyed in both frames.

    Returns ``(map_B_T_map_A 4x4 or None, reason)``. None whenever fewer than 3
    tag ids are shared -- the reason states exactly why (so the table's ``-`` is
    explained, never silent)."""
    if not markers_map_b:
        return None, (
            "no run-B marker survey (map_B_T_tag) to anchor the map_A->map_B "
            "alignment; med_err held out until run B is surveyed or PGO-tagged"
        )
    shared = sorted(set(markers_map_a) & set(markers_map_b))
    if len(shared) < 3:
        return None, (
            f"only {len(shared)} tag id(s) shared between run A and run B "
            f"({shared}); Umeyama needs >=3 correspondences"
        )
    src = np.array([markers_map_a[i] for i in shared], dtype=float)
    dst = np.array([markers_map_b[i] for i in shared], dtype=float)
    # Either frame's tags being degenerate underdetermines the rotation (cov =
    # dst_c.T @ src_c goes rank-deficient), so BOTH sides are gated -- a collapsed
    # run-B survey is as fabricated a number as a collinear run-A one.
    for pts, frame in ((src, "map_A"), (dst, "map_B")):
        why = _degenerate_reason(pts, frame)
        if why is not None:
            return None, (
                f"{len(shared)} shared tags {shared} {why}; rotation "
                "underdetermined, med_err held out"
            )
    rot, trans, scale = umeyama_alignment(src, dst, with_scale=True)
    reason = f"Umeyama over {len(shared)} shared tags {shared} (scale={scale:.4f})"
    return _rt_to_matrix(rot, trans), reason


def aligned_err_t_m(
    world_map_fix: np.ndarray, truth_map_t_world_t: np.ndarray, map_b_t_map_a: np.ndarray
) -> float:
    """Held-out translation error (m): the fix's world origin, carried from map_A
    into map_B by the alignment, versus B's PGO-truth world origin.

    ``est = map_B_T_map_A @ inv(world_map_fix)`` gives ``map_B_T_world_B``; compare
    its translation to ``truth`` (also map_B_T_world_B, from score_replay)."""
    est_map_a = np.linalg.inv(world_map_fix)  # map_A_T_world_B
    est_map_b = map_b_t_map_a @ est_map_a  # map_B_T_world_B
    return float(np.linalg.norm(est_map_b[:3, 3] - truth_map_t_world_t[:3, 3]))


# --------------------------------------------------------------------------- #
# Fixes + stats (shared by the Module and the offline driver)
# --------------------------------------------------------------------------- #
@dataclass
class Fix:
    """One accepted relocalization, in ONE timebase (wall for the Module, recording
    seconds for the offline driver -- always the same clock as the odom samples it
    is scored against)."""

    ts: float
    world_map_fix: np.ndarray | None  # 4x4 world_T_map (None if rotation not captured)
    source: str
    fitness: float
    err_t_m: float | None = None
    success: bool | None = None


@dataclass
class SourceRow:
    source: str
    won: int
    pct_traj: float
    med_err_m: float | None
    med_fit: float | None
    n_success: int | None
    n_judged: int | None


@dataclass
class EvalStats:
    rows: list[SourceRow]
    accepts: int
    rejects: int | None
    coverage_pct: float
    first_fix_s: float | None
    fiducial_proposed_cycles: int | None
    census_cycles: int | None
    fiducial_won: int
    mode: str
    held_out_note: str


def _rot_angle_rad(rot_a: np.ndarray, rot_b: np.ndarray) -> float:
    """Geodesic angle (rad) between two rotations: the residual rotation's angle,
    ||log(Ra^T Rb)||. arccos clamped for numerical safety."""
    r_rel = rot_a.T @ rot_b
    cos = (float(np.trace(r_rel)) - 1.0) / 2.0
    return float(np.arccos(max(-1.0, min(1.0, cos))))


def dedup_tf_fixes(
    tf_samples: list[tuple[float, np.ndarray]],
) -> list[tuple[float, np.ndarray]]:
    """Collapse republished world->map TFs (every PUBLISH_INTERVAL_S) to distinct
    accepts. Each accept publishes a new pose that then repeats until the next, so a
    change beyond _TF_DEDUP_EPS_M OR _TF_DEDUP_EPS_RAD starts a new fix -- recovering
    the accept sequence from the stream alone (no log). Rotation is part of the key:
    a yaw-only correction (e.g. a symmetric-scene flip) keeps the origin, so on
    translation alone it would be silently dropped."""
    fixes: list[tuple[float, np.ndarray]] = []
    last_t: np.ndarray | None = None
    last_r: np.ndarray | None = None
    for wall, mat in sorted(tf_samples, key=lambda s: s[0]):
        pose = np.asarray(mat)
        t = pose[:3, 3]
        r = pose[:3, :3]
        moved = last_t is None or float(np.linalg.norm(t - last_t)) > _TF_DEDUP_EPS_M
        turned = last_r is None or _rot_angle_rad(last_r, r) > _TF_DEDUP_EPS_RAD
        if moved or turned:
            fixes.append((wall, mat))
            last_t, last_r = t, r
    return fixes


def label_fixes_from_log(
    tf_fixes: list[tuple[float, np.ndarray]], health: list[HealthLine]
) -> list[Fix]:
    """Attach (source, fitness) from health lines to stream-derived TF fixes by
    nearest published translation (<=2 cm), the same join replay_bench uses. Each
    health line is a distinct accept, so it is consumed once: two accepts from
    different priors within 2 cm cannot both claim the same line, and equal-distance
    lines resolve to the earliest (log order) -- a later tie never overwrites.
    Unmatched fixes keep source ``unknown`` -- the trajectory still stands."""
    fixes: list[Fix] = []
    used: set[int] = set()
    for wall, mat in tf_fixes:
        t = np.asarray(mat)[:3, 3]
        best_i: int | None = None
        best_d = 0.02  # 2 cm join tolerance (matches replay_bench)
        for i, h in enumerate(health):
            if i in used:
                continue
            d = float(np.linalg.norm(np.asarray(h.published_t) - t))
            if d <= best_d and (best_i is None or d < best_d):  # strict: first wins ties
                best_i, best_d = i, d
        if best_i is not None:
            used.add(best_i)
        matched = health[best_i] if best_i is not None else None
        fixes.append(
            Fix(
                ts=wall,
                world_map_fix=mat,
                source=matched.source if matched else "unknown",
                fitness=matched.fitness if matched else float("nan"),
            )
        )
    return fixes


def _active_index(fix_ts_sorted: np.ndarray, sample_ts: np.ndarray) -> np.ndarray:
    """Index of the most-recent fix at/before each sample; -1 where uncovered."""
    return np.searchsorted(fix_ts_sorted, sample_ts, side="right") - 1


def _median(xs: list[float]) -> float | None:
    return float(np.median(xs)) if xs else None


def compute_stats(
    fixes: list[Fix],
    odom_ts: np.ndarray,
    census: list[dict[str, int]],
    n_rejects: int | None,
    *,
    mode: str,
    held_out_note: str,
) -> EvalStats:
    """Per-source + overall stats. ``fixes`` and ``odom_ts`` MUST share a clock."""
    order = np.argsort([f.ts for f in fixes]) if fixes else np.array([], dtype=int)
    fixes_sorted = [fixes[i] for i in order]
    fix_ts_sorted = np.array([f.ts for f in fixes_sorted], dtype=float)

    if fixes_sorted and odom_ts.size:
        idx = _active_index(fix_ts_sorted, odom_ts)
        covered = idx >= 0
        active_src = [fixes_sorted[i].source if i >= 0 else None for i in idx]
        n_covered = int(covered.sum())
    else:
        active_src, n_covered = [], 0

    rows: list[SourceRow] = []
    for s in SOURCES:
        s_fixes = [f for f in fixes if f.source == s]
        traj = sum(1 for a in active_src if a == s)
        judged = [f for f in s_fixes if f.success is not None]
        rows.append(
            SourceRow(
                source=s,
                won=len(s_fixes),
                pct_traj=(100.0 * traj / n_covered) if n_covered else 0.0,
                med_err_m=_median([f.err_t_m for f in s_fixes if f.err_t_m is not None]),
                med_fit=_median([f.fitness for f in s_fixes if not np.isnan(f.fitness)]),
                n_success=sum(1 for f in judged if f.success) if judged else None,
                n_judged=len(judged) if judged else None,
            )
        )

    first_fix_s: float | None = None
    if fix_ts_sorted.size and odom_ts.size:
        first_fix_s = float(fix_ts_sorted[0] - odom_ts.min())

    return EvalStats(
        rows=rows,
        accepts=len(fixes),
        rejects=n_rejects,
        coverage_pct=(100.0 * n_covered / odom_ts.size) if odom_ts.size else 0.0,
        first_fix_s=first_fix_s,
        fiducial_proposed_cycles=(
            sum(1 for c in census if c.get("fiducial", 0) > 0) if census else None
        ),
        census_cycles=len(census) if census else None,
        fiducial_won=sum(1 for f in fixes if f.source == "fiducial"),
        mode=mode,
        held_out_note=held_out_note,
    )


# --------------------------------------------------------------------------- #
# Rendering
# --------------------------------------------------------------------------- #
def _fmt(x: float | None, nd: int, suffix: str = "") -> str:
    return "-" if x is None else f"{x:.{nd}f}{suffix}"


def format_report(stats: EvalStats, *, title: str) -> str:
    """The exact table: [source | won | %traj | (med_err) | med_fit | (success)],
    then the prior-activity line, then overall, then the held-out note. med_err and
    success are dropped in live mode (no truth)."""
    truth = stats.mode == "held_out"
    header = ["source", "won", "%traj", "med_err", "med_fit", "success"]
    keep = [True, True, True, truth, True, truth]
    header = [h for h, k in zip(header, keep, strict=True) if k]

    body: list[list[str]] = [header]
    for r in stats.rows:
        succ = "-" if r.n_judged is None else f"{r.n_success}/{r.n_judged}"
        cells = [
            r.source,
            str(r.won),
            _fmt(r.pct_traj, 1, "%"),
            _fmt(r.med_err_m, 3, "m"),
            _fmt(r.med_fit, 3),
            succ,
        ]
        body.append([c for c, k in zip(cells, keep, strict=True) if k])

    widths = [max(len(row[i]) for row in body) for i in range(len(header))]
    lines = [f"=== RelocEval: {title} [{stats.mode}] ==="]
    for i, row in enumerate(body):
        lines.append("  ".join(c.ljust(widths[j]) for j, c in enumerate(row)))
        if i == 0:
            lines.append("  ".join("-" * widths[j] for j in range(len(header))))

    if stats.census_cycles is None:
        lines.append("prior activity: census unavailable (no run log parsed)")
    else:
        lines.append(
            f"prior activity: fiducial proposed {stats.fiducial_proposed_cycles}/"
            f"{stats.census_cycles} cycles, won {stats.fiducial_won}"
        )
    lines.append(
        f"overall: accepts={stats.accepts} rejects={stats.rejects if stats.rejects is not None else '-'} "
        f"coverage={stats.coverage_pct:.1f}% first-fix={_fmt(stats.first_fix_s, 1, 's')}"
    )
    lines.append(stats.held_out_note)
    return "\n".join(lines)


def stats_to_dict(stats: EvalStats, *, title: str) -> dict[str, Any]:
    return {
        "title": title,
        "mode": stats.mode,
        "accepts": stats.accepts,
        "rejects": stats.rejects,
        "coverage_pct": round(stats.coverage_pct, 2),
        "first_fix_s": None if stats.first_fix_s is None else round(stats.first_fix_s, 2),
        "prior_activity": {
            "fiducial_proposed_cycles": stats.fiducial_proposed_cycles,
            "census_cycles": stats.census_cycles,
            "fiducial_won": stats.fiducial_won,
        },
        "per_source": [
            {
                "source": r.source,
                "won": r.won,
                "pct_traj": round(r.pct_traj, 2),
                "med_err_m": None if r.med_err_m is None else round(r.med_err_m, 4),
                "med_fit": None if r.med_fit is None else round(r.med_fit, 4),
                "n_success": r.n_success,
                "n_judged": r.n_judged,
            }
            for r in stats.rows
        ],
        "held_out_note": stats.held_out_note,
    }


def _active_fix(fixes_sorted: list[Fix], fix_ts_sorted: np.ndarray, ts: float) -> Fix | None:
    i = int(np.searchsorted(fix_ts_sorted, ts, side="right")) - 1
    return fixes_sorted[i] if i >= 0 else None


def plot_trajectory(
    odom_xy: np.ndarray,
    fixes: list[Fix],
    markers_xy: dict[int, tuple[float, float]],
    out_png: Path,
    *,
    title: str,
) -> Path:
    """Top-down robot path in map_A frame, coloured by the winning prior of the fix
    active at each sample, with surveyed markers as stars. odom_xy: (N,3) [ts,x,y]
    in the world (odom) frame; each covered sample is mapped by inv(active fix)."""
    import matplotlib

    matplotlib.use("Agg")  # headless: no display on the robot / CI
    from matplotlib.lines import Line2D
    import matplotlib.pyplot as plt

    fx = sorted((f for f in fixes if f.world_map_fix is not None), key=lambda f: f.ts)
    fx_ts = np.array([f.ts for f in fx], dtype=float)
    pts: list[np.ndarray] = []
    cols: list[str] = []
    for ts, x, y in odom_xy:
        f = _active_fix(fx, fx_ts, float(ts)) if fx_ts.size else None
        if f is None or f.world_map_fix is None:
            continue
        map_t_world = np.linalg.inv(f.world_map_fix)  # map_A_T_world
        pm = map_t_world @ np.array([x, y, 0.0, 1.0])
        pts.append(pm[:2])
        cols.append(SOURCE_COLORS.get(f.source, SOURCE_COLORS["unknown"]))

    fig, ax = plt.subplots(figsize=(9, 7))
    if pts:
        arr = np.array(pts)
        ax.scatter(arr[:, 0], arr[:, 1], c=cols, s=6, zorder=2)
    for tid, (mx, my) in sorted(markers_xy.items()):
        ax.scatter([mx], [my], marker="*", s=320, c="#c53030", edgecolors="k", zorder=4)
        ax.annotate(
            f"tag {tid}", (mx, my), textcoords="offset points",
            xytext=(6, 6), fontsize=9, weight="bold",
        )
    present = [s for s in ("ransac", "fiducial", "last_pose") if any(f.source == s for f in fx)]
    legend = [
        Line2D([0], [0], marker="o", color="w", markerfacecolor=SOURCE_COLORS[s],
               label=f"{s} won", markersize=8)
        for s in (present or ["unknown"])
    ]
    if markers_xy:
        legend.append(
            Line2D([0], [0], marker="*", color="w", markerfacecolor="#c53030",
                   markeredgecolor="k", label="marker (map_T_tag)", markersize=14)
        )
    ax.legend(handles=legend, loc="upper right", fontsize=9)
    ax.set_aspect("equal")
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    ax.set_title(title)
    ax.grid(alpha=0.25)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    plt.close(fig)
    return out_png


def write_report(
    stats: EvalStats,
    odom_xy: np.ndarray,
    fixes: list[Fix],
    markers_xy: dict[int, tuple[float, float]],
    out_dir: Path,
    key: str,
    *,
    title: str,
) -> dict[str, Path]:
    """Print the table, then write ``<key>.eval.json`` + ``<key>.trajectory.png``."""
    out_dir.mkdir(parents=True, exist_ok=True)
    print(format_report(stats, title=title))
    json_path = out_dir / f"{key}.eval.json"
    json_path.write_text(json.dumps(stats_to_dict(stats, title=title), indent=2))
    png_path = plot_trajectory(
        odom_xy, fixes, markers_xy, out_dir / f"{key}.trajectory.png", title=title
    )
    print(f"[releval] wrote {json_path}")
    print(f"[releval] wrote {png_path}")
    return {"json": json_path, "png": png_path}


# --------------------------------------------------------------------------- #
# Marker / odom loaders (offline driver + plot overlay)
# --------------------------------------------------------------------------- #
def load_markers_xyz(marker_file: Path) -> dict[int, tuple[float, float, float]]:
    """map_T_tag translations by tag id, from the reloc module's marker map
    (``.yaml`` or ``.json``; both key ``markers.<id>.translation``)."""
    text = marker_file.read_text()
    doc = json.loads(text) if marker_file.suffix == ".json" else yaml.safe_load(text)
    markers = doc.get("markers", {})
    out: dict[int, tuple[float, float, float]] = {}
    for tag_id, val in markers.items():
        t = val["translation"]
        out[int(tag_id)] = (float(t[0]), float(t[1]), float(t[2]))
    return out


def load_odom_xy(recording_db: Path) -> np.ndarray:
    """(N,3) [recording_ts, x, y] from a recording's ``odom`` stream."""
    from dimos.memory2.store.sqlite import SqliteStore

    store = SqliteStore(path=str(recording_db), must_exist=True)
    rows: list[tuple[float, float, float]] = []
    with store:
        for o in store.stream("odom", PoseStamped).to_list():
            rows.append((float(o.ts), float(o.data.x), float(o.data.y)))
    return np.array(rows, dtype=float) if rows else np.empty((0, 3), dtype=float)


def _fixes_from_replay_json(fixes_json: list[dict[str, Any]]) -> list[Fix]:
    out: list[Fix] = []
    for f in fixes_json:
        mat = f.get("world_map_fix")
        out.append(
            Fix(
                ts=float(f["ts"]),
                world_map_fix=np.asarray(mat, dtype=float) if mat is not None else None,
                source=str(f.get("source", "unknown")),
                fitness=float(f.get("fitness", float("nan"))),
            )
        )
    return out


def run_offline_report(
    replay_json: Path,
    recording_db: Path,
    marker_map: Path,
    out_dir: Path,
    key: str,
    *,
    title: str,
    run_log: Path | None = None,
    score_json: Path | None = None,
    markers_map_b: Path | None = None,
) -> dict[str, Any]:
    """Held-out report from CAPTURED artifacts -- no replay launched. Consumes the
    real pipeline's published fixes (replay_bench's replay.json), B's odom, and A's
    marker survey; enriches per-source ``med_err`` via Umeyama IF run-B references
    (markers_map_b) and per-fix truth (score_json) are both available, else ``-``.

    The scoring itself is score_replay's job -- this only aligns published fixes and
    reports; it re-implements no dimos data-path step.
    """
    replay = json.loads(replay_json.read_text())
    fixes = _fixes_from_replay_json(replay["fixes"])
    meta = replay.get("meta", {})

    markers_a = load_markers_xyz(marker_map)
    markers_b = load_markers_xyz(markers_map_b) if markers_map_b and markers_map_b.exists() else None
    align, align_reason = resolve_heldout_alignment(markers_a, markers_b)

    # Per-fix held-out error needs BOTH the alignment and per-fix PGO truth.
    if align is not None and score_json is not None and score_json.exists():
        truth_by_ts = {
            round(float(s["ts"]), 3): np.asarray(s["truth_t_m"], dtype=float)
            for s in json.loads(score_json.read_text()).get("fixes", [])
        }
        for f in fixes:
            truth_t = truth_by_ts.get(round(f.ts, 3))
            if f.world_map_fix is not None and truth_t is not None:
                truth_mat = np.eye(4)
                truth_mat[:3, 3] = truth_t
                f.err_t_m = aligned_err_t_m(f.world_map_fix, truth_mat, align)
                f.success = bool(f.err_t_m < SUCCESS_T_M)

    census: list[dict[str, int]] = []
    log_text = ""
    if run_log is None:
        cand = replay_json.with_name(replay_json.name.replace(".replay.json", ".replay_run.log"))
        run_log = cand if cand.exists() else None
    if run_log is not None:
        log_text = run_log.read_text(errors="replace")
        census = parse_census(log_text)
    n_rejects = meta.get("n_rejects")
    if n_rejects is None and log_text:
        n_rejects = count_rejects(log_text)

    note = (
        "held-out: premap+markers from run A, replay run B of the same scene; map "
        f"frames are independent PGO runs. med_err alignment: {align_reason}."
    )
    odom_xy = load_odom_xy(recording_db)
    odom_ts = odom_xy[:, 0] if odom_xy.size else np.empty((0,), dtype=float)
    stats = compute_stats(fixes, odom_ts, census, n_rejects, mode="held_out", held_out_note=note)
    markers_xy = {i: (x, y) for i, (x, y, _z) in markers_a.items()}
    paths = write_report(stats, odom_xy, fixes, markers_xy, out_dir, key, title=title)
    return {
        "stats": stats_to_dict(stats, title=title),
        "paths": {k: str(v) for k, v in paths.items()},
    }


# --------------------------------------------------------------------------- #
# The Module (attached by `dimos run --eval`)
# --------------------------------------------------------------------------- #
class EvalConfig(ModuleConfig):
    # Where the eval artifacts land (json + trajectory png). Relative -> CWD.
    out_dir: str = "out/eval"
    # Output-file key; keeps concurrent runs' artifacts from colliding.
    tag: str = "releval"
    # Surveyed marker map (map_T_tag) to overlay on the plot; the SAME yaml/json
    # the reloc module loads. None -> no markers drawn (stats unaffected).
    marker_map_file: str | None = None
    # Run log for the census supplement. None -> this run's own main.jsonl.
    run_log_file: str | None = None


class RelocEval(Module):
    """In-process collector: subscribes to the real ``/tf`` + ``/odom`` streams and,
    at stop(), prints the per-source table + writes json + a top-down trajectory
    PNG. Always "live" -- no in-process PGO truth, so med_err/success are the offline
    driver's job. Best-effort: a finalize failure must not crash the run.
    """

    config: EvalConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # Callbacks fire on the LCM handle thread; list.append is atomic under the
        # GIL (no lock). Each sample carries the wall time it was seen -- the one
        # clock shared by odom and /tf, so no clock mapping is required.
        self._odom: list[tuple[float, float, float]] = []  # (wall, x, y)
        self._world_map: list[tuple[float, np.ndarray]] = []  # (wall, world_T_map 4x4)
        self._tf_t: LCMTransport[TFMessage] = LCMTransport(TF_TOPIC, TFMessage)
        self._odom_t: LCMTransport[PoseStamped] = LCMTransport(ODOM_TOPIC, PoseStamped)
        self._unsub: list[Any] = []

    @rpc
    def start(self) -> None:
        super().start()
        self._unsub.append(self._odom_t.subscribe(self._on_odom))
        self._unsub.append(self._tf_t.subscribe(self._on_tf))
        logger.info("RelocEval collector started", tf_topic=TF_TOPIC, odom_topic=ODOM_TOPIC)

    def _on_odom(self, msg: PoseStamped) -> None:
        self._odom.append((time.time(), float(msg.position.x), float(msg.position.y)))

    def _on_tf(self, msg: TFMessage) -> None:
        wall = time.time()
        for tf in msg.transforms:
            if tf.frame_id == FRAME_WORLD and tf.child_frame_id == FRAME_MAP:
                self._world_map.append((wall, tf.to_matrix()))

    @rpc
    def stop(self) -> None:
        try:
            self._finalize()
        except Exception:
            logger.exception("RelocEval finalize failed (run unaffected)")
        for unsub in self._unsub:
            try:
                unsub()
            except Exception:
                logger.exception("RelocEval unsubscribe failed")
        self._tf_t.stop()
        self._odom_t.stop()
        super().stop()

    def _finalize(self) -> None:
        tf_fixes = dedup_tf_fixes(self._world_map)
        log_path = resolve_run_log(self.config.run_log_file)
        health: list[HealthLine] = []
        census: list[dict[str, int]] = []
        n_rejects: int | None = None
        if log_path is not None:
            log_text = log_path.read_text(errors="replace")
            health = parse_health_lines(log_text)
            census = parse_census(log_text)
            n_rejects = count_rejects(log_text)
        fixes = label_fixes_from_log(tf_fixes, health)

        odom = np.array(self._odom, dtype=float) if self._odom else np.empty((0, 3), dtype=float)
        odom_ts = odom[:, 0] if odom.size else np.empty((0,), dtype=float)
        markers_xy: dict[int, tuple[float, float]] = {}
        if self.config.marker_map_file:
            markers_xy = {
                i: (x, y)
                for i, (x, y, _z) in load_markers_xyz(Path(self.config.marker_map_file)).items()
            }

        note = (
            "live capture: no PGO truth in-process -> med_err/success omitted; the "
            "scored held-out accuracy comes from the offline held-out driver."
        )
        stats = compute_stats(
            fixes, odom_ts, census, n_rejects, mode="live", held_out_note=note
        )
        # odom_xy for the plot uses (ts, x, y): reuse the wall ts as the sample key.
        write_report(stats, odom, fixes, markers_xy, Path(self.config.out_dir), self.config.tag,
                     title="RelocEval live capture")
