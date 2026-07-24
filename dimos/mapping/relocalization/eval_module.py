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
pure analysis it shares with the offline held-out driver. Trajectory/coverage/counts
come from the real ``/tf`` + ``/odom`` streams; the run log is a supplement carrying
each accept's winning prior, fitness, and the per-cycle census."""

from __future__ import annotations

import atexit
from collections import Counter
from dataclasses import dataclass
import json
from pathlib import Path
import re
import socket
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
from dimos.visualization.rerun.constants import RERUN_GRPC_PORT

logger = setup_logger()

FRAME_WORLD = "world"
FRAME_MAP = "map"
TF_TOPIC = "/tf"  # RelocalizationModule republishes world->map here (LCMTF -> /tf)
ODOM_TOPIC = "/odom"  # recording-timestamped PoseStamped, preserved through replay

SOURCES: tuple[str, ...] = ("ransac", "fiducial")
# ONE palette for both renderers (trajectory PNG + the rerun overlay); unknown is a
# dim RED because an unlabelled fix is a JOIN FAILURE and must read as one.
SOURCE_COLORS: dict[str, str] = {
    "ransac": "#2b6cb0",  # blue
    "fiducial": "#dd6b20",  # orange
    "unknown": "#9b2c2c",  # dim red
}
_TF_DEDUP_EPS_M = 1e-4  # two world->map translations within this are the same accept
_TF_DEDUP_EPS_RAD = 1e-4  # ...and within this rotation (a yaw-only fix keeps the origin)

# The live rerun overlay. "world/" is the bridge's entity_prefix, so the overlay
# sits under the same Spatial3DView origin as every other run entity.
RERUN_ACCEPTED_ENTITY = "world/eval/accepted"
# The premap is logged with frame_id "map" and the bridge parents it to tf#/map, so
# the overlay names the same frame to land on top of it.
RERUN_ACCEPTED_PARENT_FRAME = "tf#/map"
RERUN_ACCEPTED_RADIUS_M = 0.25  # premap voxels draw at ~0.025 m radius; 10x reads over them
_RERUN_PROBE_TIMEOUT_S = 0.2  # the bridge's proxy is local -- it answers or it is not up


# --------------------------------------------------------------------------- #
# Log parsing (supplement) -- ONE key-based parser for BOTH renderings the structlog
# call produces (main.jsonl JSON object + tee'd console line) AND the LEGACY f-string
# format from before the emitters moved to structlog kwargs.
#
# Key-anchored and never positional: the legacy line puts `source=` LAST, so a
# regex that assumed field order would miss it and every accept would fall back to
# ransac -- "fiducial won 0" as a parser artifact rather than a measurement.
# --------------------------------------------------------------------------- #
_EVENT_ACCEPT = "relocalize accepted"
_EVENT_REJECT = "relocalize rejected"
_EVENT_CENSUS = "relocalize candidates"
# Legacy accepts are the only `relocalize:` lines carrying a fitness (the prior-
# enabled / skipped lines have their own shapes).
_LEGACY_ACCEPT_RE = re.compile(r"relocalize:\s*fitness=")

# dimos colorizes the console only when stdout is a tty (logging_config.py), so a
# log captured through a pty carries SGR codes BETWEEN key and `=`. Strip first.
_ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
# Console kwargs render as `key=value` with the value verbatim -- `[1.0, 2.0]` and
# `{'ransac': 34}` contain spaces, so each field gets its own key-anchored regex;
# splitting the line on whitespace would tear those values apart. The `_m` suffix
# is optional: current emitters name the unit, legacy ones do not.
_CONSOLE_RE: dict[str, re.Pattern[str]] = {
    "source": re.compile(r"\bsource=([A-Za-z_]\w*)"),
    "fitness": re.compile(r"\bfitness=([-\d.eE+]+)"),
    "published_t_m": re.compile(r"\bpublished_t(?:_m)?=\[([^\]]+)\]"),
    "counts": re.compile(r"\bcounts=\{([^}]*)\}"),
}
_COUNT_RE = re.compile(r"'(\w+)':\s*(\d+)")  # one entry of counts={...}'s dict repr
_LEGACY_COUNT_RE = re.compile(r"\b([a-z_]+)=(\d+)")  # legacy census: `ransac=34 fiducial=2`


@dataclass
class HealthLine:
    """One accepted relocalize, parsed from a ``relocalize accepted`` log record.
    A QUIET line has no ``published_t_m`` -> positionless (nothing to join by)."""

    source: str
    fitness: float
    published_t_m: tuple[float, float, float] | None  # world_T_map translation (join key)


def _floats(csv: str) -> list[float]:
    return [float(x) for x in csv.split(",")]


def _console_fields(text: str) -> dict[str, Any]:
    """The kwargs a console line carries, typed like their main.jsonl twins."""
    fields: dict[str, Any] = {}
    src = _CONSOLE_RE["source"].search(text)
    if src is not None:
        fields["source"] = src.group(1)
    fit = _CONSOLE_RE["fitness"].search(text)
    if fit is not None:
        fields["fitness"] = float(fit.group(1))
    pub = _CONSOLE_RE["published_t_m"].search(text)
    if pub is not None:
        fields["published_t_m"] = _floats(pub.group(1))
    counts = _CONSOLE_RE["counts"].search(text)
    if counts is not None:
        fields["counts"] = {k: int(n) for k, n in _COUNT_RE.findall(counts.group(1))}
    elif _EVENT_CENSUS in text:
        # Legacy census: the counts ARE the kwargs. Read only the tail, so nothing
        # before the event name (a timestamp, a pid) is mistaken for a count.
        legacy = _LEGACY_COUNT_RE.findall(text.split(_EVENT_CENSUS, 1)[1])
        if legacy:
            fields["counts"] = {k: int(n) for k, n in legacy}
    return fields


def _event_of(text: str, obj: dict[str, Any] | None) -> str | None:
    """Which record this line is, in either format and either rendering."""
    if obj is not None:
        ev = obj.get("event")
        if not isinstance(ev, str):
            return None
        text = ev
    for event in (_EVENT_ACCEPT, _EVENT_REJECT, _EVENT_CENSUS):
        if event in text:
            return event
    return _EVENT_ACCEPT if _LEGACY_ACCEPT_RE.search(text) else None


def _parse_line(line: str) -> tuple[str, dict[str, Any]] | None:
    """``(event, kwargs)`` for one relocalize log line in either rendering and wire
    format, or None when the line is none of them. Console matches and jsonl keys are
    merged, jsonl winning."""
    text = _ANSI_RE.sub("", line).strip()
    obj: dict[str, Any] | None = None
    if text.startswith("{"):
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError:
            return None
        if not isinstance(parsed, dict):
            return None
        obj = parsed
    event = _event_of(text, obj)
    if event is None:
        return None
    fields = _console_fields(text)
    if obj is not None:
        fields.update({k: v for k, v in obj.items() if v is not None})
    return event, fields


def parse_health_lines(log_text: str) -> list[HealthLine]:
    """Every accepted fix (source + fitness, plus the published translation when the
    line carries one). ``source`` defaults to ransac on the single-source path;
    ``fitness`` is the one required field -- it is what makes a line an accept."""
    out: list[HealthLine] = []
    for raw in log_text.splitlines():
        record = _parse_line(raw)
        if record is None or record[0] != _EVENT_ACCEPT:
            continue
        fields = record[1]
        fit = fields.get("fitness")
        if fit is None:
            continue
        pub = fields.get("published_t_m")
        out.append(
            HealthLine(
                source=str(fields.get("source", "ransac")),
                fitness=float(fit),
                published_t_m=(
                    (float(pub[0]), float(pub[1]), float(pub[2]))
                    if isinstance(pub, list) and len(pub) == 3
                    else None
                ),
            )
        )
    return out


def parse_census(log_text: str) -> list[dict[str, int]]:
    """One dict of proposal counts per ``relocalize candidates`` cycle. Every source
    name is kept, not just SOURCES -- a new prior must show up in the census."""
    out: list[dict[str, int]] = []
    for raw in log_text.splitlines():
        record = _parse_line(raw)
        if record is None or record[0] != _EVENT_CENSUS:
            continue
        counts = record[1].get("counts")
        if isinstance(counts, dict):
            out.append({str(k): int(n) for k, n in counts.items()})
    return out


def parse_reject_lines(log_text: str) -> list[str]:
    """The refused source of every ``relocalize rejected`` record -- ``unknown`` when
    the line named none (only the legacy format predates the field). Length is the
    reject count."""
    out: list[str] = []
    for raw in log_text.splitlines():
        record = _parse_line(raw)
        if record is None or record[0] != _EVENT_REJECT:
            continue
        out.append(str(record[1].get("source", "unknown")))
    return out


def parse_run_log(path: Path) -> tuple[list[HealthLine], list[dict[str, int]], list[str]]:
    """Every relocalize record in a run log FILE: ``(accepts, census, reject_sources)``.
    A log the parser reads NOTHING out of is warned, because downstream it is
    indistinguishable from a real measurement (empty census reads as "never proposed").
    """
    text = path.read_text(errors="replace")
    health = parse_health_lines(text)
    census = parse_census(text)
    reject_sources = parse_reject_lines(text)
    if not health and not census and not reject_sources:
        logger.warning(
            "run log yielded no relocalize records; census and per-source labels "
            "will be empty -- check the log format against this parser",
            run_log=str(path),
            n_lines=text.count("\n") + 1,
        )
    return health, census, reject_sources


def resolve_run_log(run_log_file: str | None) -> Path | None:
    """The active run's log: an explicit override, else this run's main.jsonl.
    None if neither resolves."""
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
# Fixes + stats (shared by the Module and the offline driver)
# --------------------------------------------------------------------------- #
@dataclass
class Fix:
    """One accepted relocalization, in ONE timebase (wall for the Module, recording
    seconds for the offline driver -- always the same clock as the odom it scores against)."""

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
    # Per-source activity from the run-log events -- independent of the /tf join, so
    # these never degrade to ``unknown`` the way a join-labelled fix's source can.
    proposed: int = 0  # cycles this source put >=1 candidate forward (census PRESENCE)
    accepted: int = 0  # `relocalize accepted` log lines this source won
    rejected: int | None = None  # `relocalize rejected` lines; None = no source parsed
    n_false: int | None = None  # accepts beyond the held-out gate (a wrong accept); None = no truth


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
    """Geodesic angle (rad) between two rotations: ||log(Ra^T Rb)||, arccos clamped."""
    r_rel = rot_a.T @ rot_b
    cos = (float(np.trace(r_rel)) - 1.0) / 2.0
    return float(np.arccos(max(-1.0, min(1.0, cos))))


def dedup_tf_fixes(
    tf_samples: list[tuple[float, np.ndarray]],
) -> list[tuple[float, np.ndarray]]:
    """Collapse republished world->map TFs to distinct accepts: a change beyond
    _TF_DEDUP_EPS_M OR _TF_DEDUP_EPS_RAD starts a new fix. Rotation is part of the key
    because a yaw-only correction keeps the origin and would else be dropped."""
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
    health line is consumed once (ties resolve to earliest in log order); unmatched
    fixes keep source ``unknown``. A positionless (QUIET) line labels nothing."""
    fixes: list[Fix] = []
    used: set[int] = set()
    for wall, mat in tf_fixes:
        t = np.asarray(mat)[:3, 3]
        best_i: int | None = None
        best_d = 0.02  # 2 cm join tolerance (matches replay_bench)
        for i, h in enumerate(health):
            if i in used or h.published_t_m is None:
                continue
            d = float(np.linalg.norm(np.asarray(h.published_t_m) - t))
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
    odom_txyz: np.ndarray,
    census: list[dict[str, int]],
    n_rejects: int | None,
    *,
    mode: str,
    held_out_note: str,
    accept_sources: list[str] | None = None,
    reject_sources: list[str] | None = None,
) -> EvalStats:
    """Per-source + overall stats. ``fixes`` and ``odom_txyz`` MUST share a clock.

    Args:
        odom_txyz: (N, 4) [ts, x, y, z] robot positions in the world (LIO/odom) frame,
            or an empty array. A wrong-width array raises rather than mis-indexing.
        accept_sources: the source of every ``relocalize accepted`` log line. None ->
            fall back to the joined fixes' sources (what the held-out driver wants).
        reject_sources: the source of every reject. None -> not parsed, so per-source
            ``rej`` is held out (``-``), not 0.
    """
    odom_txyz = np.asarray(odom_txyz, dtype=float)
    if odom_txyz.size and (odom_txyz.ndim != 2 or odom_txyz.shape[1] != 4):
        raise ValueError(f"compute_stats: odom must be (N,4) [ts,x,y,z], got {odom_txyz.shape}")
    odom_ts = odom_txyz[:, 0] if odom_txyz.size else np.empty((0,), dtype=float)
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

    # PROPOSED is per-CYCLE PRESENCE, not candidate count: counting candidates would
    # reward a chatty prior. Same convention as fiducial_proposed_cycles below.
    proposed_by = {
        s: sum(1 for c in census if c.get(s, 0) > 0) for s in {k for c in census for k in c}
    }
    acc_src = accept_sources if accept_sources is not None else [f.source for f in fixes]
    accepted_by = Counter(acc_src)
    rejected_by = Counter(reject_sources) if reject_sources is not None else None
    # SOURCES first (always shown, even at zero), then any other source appearing in
    # the census / accepts / rejects / joined fixes (a new prior, or ``unknown``).
    extra = sorted(
        (set(proposed_by) | set(acc_src) | set(reject_sources or []) | {f.source for f in fixes})
        - set(SOURCES)
    )

    rows: list[SourceRow] = []
    for s in (*SOURCES, *extra):
        s_fixes = [f for f in fixes if f.source == s]
        traj = sum(1 for a in active_src if a == s)
        judged = [f for f in s_fixes if f.success is not None]
        n_success = sum(1 for f in judged if f.success) if judged else None
        n_judged = len(judged) if judged else None
        rows.append(
            SourceRow(
                source=s,
                won=len(s_fixes),
                pct_traj=(100.0 * traj / n_covered) if n_covered else 0.0,
                med_err_m=_median([f.err_t_m for f in s_fixes if f.err_t_m is not None]),
                med_fit=_median([f.fitness for f in s_fixes if not np.isnan(f.fitness)]),
                n_success=n_success,
                n_judged=n_judged,
                proposed=proposed_by.get(s, 0),
                accepted=accepted_by.get(s, 0),
                rejected=None if rejected_by is None else rejected_by.get(s, 0),
                # FALSE reuses the held-out accuracy gate (Fix.success): a judged accept that failed it.
                n_false=sum(1 for f in judged if f.success is False) if judged else None,
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


def _count_cell(x: int | None) -> str:
    """A count cell: the number, or ``-`` when the datum was not derivable. Never a fabricated 0."""
    return "-" if x is None else str(x)


def format_report(stats: EvalStats, *, title: str) -> str:
    """The per-source table, a TOTAL row, then the prior-activity line, overall, and
    the held-out note. ``false`` and ``med_err`` are dropped in live mode (no truth)."""
    truth = stats.mode == "held_out"
    header = ["source", "prop", "acc", "rej", "false", "%traj", "med_err", "med_fit"]
    keep = [True, True, True, True, truth, True, truth, True]
    header = [h for h, k in zip(header, keep, strict=True) if k]

    def _row_cells(
        source: str, prop: str, acc: str, rej: str, false: str, traj: str, err: str, fit: str
    ) -> list[str]:
        cells = [source, prop, acc, rej, false, traj, err, fit]
        return [c for c, k in zip(cells, keep, strict=True) if k]

    body: list[list[str]] = [header]
    for r in stats.rows:
        body.append(
            _row_cells(
                r.source,
                str(r.proposed),
                str(r.accepted),
                _count_cell(r.rejected),
                _count_cell(r.n_false),
                _fmt(r.pct_traj, 1, "%"),
                _fmt(r.med_err_m, 3, "m"),
                _fmt(r.med_fit, 3),
            )
        )
    # TOTAL: the count columns sum; medians and %traj do not aggregate, so they blank.
    total_rej = (
        None
        if any(r.rejected is None for r in stats.rows)
        else sum(r.rejected or 0 for r in stats.rows)
    )
    false_vals = [r.n_false for r in stats.rows if r.n_false is not None]
    total_row = _row_cells(
        "TOTAL",
        str(sum(r.proposed for r in stats.rows)),
        str(sum(r.accepted for r in stats.rows)),
        _count_cell(total_rej),
        _count_cell(sum(false_vals) if false_vals else None),
        "",
        "",
        "",
    )

    widths = [max(len(row[i]) for row in (*body, total_row)) for i in range(len(header))]
    sep = "  ".join("-" * widths[j] for j in range(len(header)))
    lines = [f"=== RelocEval: {title} [{stats.mode}] ==="]
    for i, row in enumerate(body):
        lines.append("  ".join(c.ljust(widths[j]) for j, c in enumerate(row)))
        if i == 0:
            lines.append(sep)
    lines.append(sep)
    lines.append("  ".join(c.ljust(widths[j]) for j, c in enumerate(total_row)))

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
                "proposed": r.proposed,
                "accepted": r.accepted,
                "rejected": r.rejected,
                "n_false": r.n_false,
            }
            for r in stats.rows
        ],
        "held_out_note": stats.held_out_note,
    }


def _active_fix(fixes_sorted: list[Fix], fix_ts_sorted: np.ndarray, ts: float) -> Fix | None:
    i = int(np.searchsorted(fix_ts_sorted, ts, side="right")) - 1
    return fixes_sorted[i] if i >= 0 else None


def plot_trajectory(
    odom_txy: np.ndarray,
    fixes: list[Fix],
    markers_xy: dict[int, tuple[float, float]],
    out_png: Path,
    *,
    title: str,
) -> Path:
    """Top-down robot path in map_A frame, coloured by the active fix's winning prior,
    markers as stars. odom_txy: (N,3) [ts,x,y] in world; each sample mapped by inv(fix)."""
    import matplotlib

    matplotlib.use("Agg")  # headless: no display on the robot / CI
    from matplotlib.lines import Line2D
    import matplotlib.pyplot as plt

    fx = sorted((f for f in fixes if f.world_map_fix is not None), key=lambda f: f.ts)
    fx_ts = np.array([f.ts for f in fx], dtype=float)
    pts: list[np.ndarray] = []
    cols: list[str] = []
    for ts, x, y in odom_txy:
        f = _active_fix(fx, fx_ts, float(ts)) if fx_ts.size else None
        if f is None or f.world_map_fix is None:
            continue
        map_T_world = np.linalg.inv(f.world_map_fix)  # map_A_T_world
        pm = map_T_world @ np.array([x, y, 0.0, 1.0])
        pts.append(pm[:2])
        cols.append(SOURCE_COLORS.get(f.source, SOURCE_COLORS["unknown"]))

    fig, ax = plt.subplots(figsize=(9, 7))
    if pts:
        arr = np.array(pts)
        ax.scatter(arr[:, 0], arr[:, 1], c=cols, s=6, zorder=2)
    for tid, (mx, my) in sorted(markers_xy.items()):
        ax.scatter([mx], [my], marker="*", s=320, c="#c53030", edgecolors="k", zorder=4)
        ax.annotate(
            f"tag {tid}",
            (mx, my),
            textcoords="offset points",
            xytext=(6, 6),
            fontsize=9,
            weight="bold",
        )
    present = [s for s in SOURCES if any(f.source == s for f in fx)]
    legend = [
        Line2D(
            [0],
            [0],
            marker="o",
            color="w",
            markerfacecolor=SOURCE_COLORS[s],
            label=f"{s} won",
            markersize=8,
        )
        for s in (present or ["unknown"])
    ]
    if markers_xy:
        legend.append(
            Line2D(
                [0],
                [0],
                marker="*",
                color="w",
                markerfacecolor="#c53030",
                markeredgecolor="k",
                label="marker (map_T_tag)",
                markersize=14,
            )
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


def _rgb(hex_color: str) -> tuple[int, int, int]:
    """'#rrggbb' -> (r, g, b) 0-255, the form rerun wants."""
    h = hex_color.lstrip("#")
    return (int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16))


@dataclass
class AcceptedPoints:
    """The rerun overlay for a run's accepted fixes: one coloured, labelled point each,
    in the MAP frame. Pure data -- built without rerun, logged by the Module."""

    positions_map_m: list[tuple[float, float, float]]
    colors_rgb: list[tuple[int, int, int]]
    labels: list[str]

    def __len__(self) -> int:
        return len(self.positions_map_m)


def accepted_points_in_map(fixes: list[Fix], odom_txyz: np.ndarray) -> AcceptedPoints:
    """One point per accepted fix, at the ROBOT's position when that fix landed,
    expressed in the premap's map frame (``map_T_world = inv(fix)``), coloured by the
    prior that won. Same mapping plot_trajectory uses, so PNG and viewer cannot drift.

    The robot's position is the odom sample nearest in the shared wall clock. The
    fix's own translation is deliberately NOT used: that is the map ORIGIN in world,
    which barely moves between accepts and would draw every prior's win in one blob.

    Args:
        odom_txyz: (N, 4) [wall_ts, x, y, z] robot positions in the world frame.
    Returns:
        AcceptedPoints, 1:1 with ``fixes`` -- empty when no odom was captured.
    """
    positions: list[tuple[float, float, float]] = []
    colors: list[tuple[int, int, int]] = []
    labels: list[str] = []
    if odom_txyz.size == 0:
        return AcceptedPoints(positions, colors, labels)
    odom_ts = odom_txyz[:, 0]
    for f in fixes:
        if f.world_map_fix is None:
            continue
        i = int(np.argmin(np.abs(odom_ts - f.ts)))
        p_map = np.linalg.inv(f.world_map_fix) @ np.array([*odom_txyz[i, 1:4], 1.0])
        positions.append((float(p_map[0]), float(p_map[1]), float(p_map[2])))
        colors.append(_rgb(SOURCE_COLORS.get(f.source, SOURCE_COLORS["unknown"])))
        fit = "-" if np.isnan(f.fitness) else f"{f.fitness:.2f}"
        labels.append(f"{f.source} fit={fit}")
    return AcceptedPoints(positions, colors, labels)


def write_report(
    stats: EvalStats,
    odom_txyz: np.ndarray,
    fixes: list[Fix],
    markers_xy: dict[int, tuple[float, float]],
    out_dir: Path,
    key: str,
    *,
    title: str,
) -> dict[str, Path]:
    """Print the table, then write ``<key>.eval.json`` and ``<key>.trajectory.png``.
    ``odom_txyz``: (N, 4) [ts, x, y, z] in the world frame; the plot takes [ts, x, y]."""
    out_dir.mkdir(parents=True, exist_ok=True)
    print(format_report(stats, title=title))
    json_path = out_dir / f"{key}.eval.json"
    json_path.write_text(json.dumps(stats_to_dict(stats, title=title), indent=2))
    png_path = plot_trajectory(
        odom_txyz[:, :3], fixes, markers_xy, out_dir / f"{key}.trajectory.png", title=title
    )
    print(f"[releval] wrote {json_path}")
    print(f"[releval] wrote {png_path}")
    return {"json": json_path, "png": png_path}


# --------------------------------------------------------------------------- #
# Marker loader (plot overlay)
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


# --------------------------------------------------------------------------- #
# The Module (attached by `dimos run --eval`)
# --------------------------------------------------------------------------- #
class EvalConfig(ModuleConfig):
    # Where the eval artifacts land (json + trajectory png). Relative -> CWD.
    out_dir: str = "out/eval"
    # Output-file key; keeps concurrent runs' artifacts from colliding.
    tag: str = "releval"
    # Surveyed marker map (map_T_tag) to overlay on the plot; the SAME yaml/json the
    # reloc module loads. None -> no markers drawn (stats unaffected).
    marker_map_file: str | None = None
    # Run log for the census supplement. None -> this run's own main.jsonl.
    run_log_file: str | None = None


class RelocEval(Module):
    """In-process collector: subscribes to ``/tf`` + ``/odom``, draws each accept into
    rerun as it lands, and at stop() prints the per-source table + writes json + a
    trajectory PNG. Always "live" (no PGO truth). Best-effort: neither the overlay nor
    a finalize failure may crash the run."""

    config: EvalConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # Callbacks fire on the LCM handle thread; list.append is atomic under the
        # GIL (no lock). Each sample carries the wall time it was seen -- the one
        # clock shared by odom and /tf, so no clock mapping is required.
        self._odom: list[tuple[float, float, float, float]] = []  # (wall, x, y, z)
        self._world_map: list[tuple[float, np.ndarray]] = []  # (wall, world_T_map 4x4)
        self._tf_transport: LCMTransport[TFMessage] = LCMTransport(TF_TOPIC, TFMessage)
        self._odom_transport: LCMTransport[PoseStamped] = LCMTransport(ODOM_TOPIC, PoseStamped)
        self._unsub: list[Any] = []
        # Live-overlay state (all touched only from the /tf callback thread).
        self._health: list[HealthLine] = []  # accepts tailed out of the run log so far
        self._log_pos = 0  # byte offset that tail has consumed
        self._n_drawn = 0  # accepts already in the overlay
        self._rr_ready = False  # connected to the bridge's rerun recording
        self._rr_enabled = True  # cleared after a failure, so it cannot spam
        # stop() reaches this module up to three times on one shutdown -- the
        # coordinator's RPC, the worker's own teardown, and the exit hook below.
        self._finalized = False

    @rpc
    def start(self) -> None:
        super().start()
        self._unsub.append(self._odom_transport.subscribe(self._on_odom))
        self._unsub.append(self._tf_transport.subscribe(self._on_tf))
        # Ctrl+C and a crashing run tear the worker down without ever running stop()
        # here -- the coordinator sends it fire-and-forget (rpc_client.RpcCall) and
        # does not wait -- so process exit is the last chance to write the report.
        atexit.register(self.stop)
        logger.info("RelocEval collector started", tf_topic=TF_TOPIC, odom_topic=ODOM_TOPIC)

    def _on_odom(self, msg: PoseStamped) -> None:
        # z is carried for the rerun overlay (a 3D point); the PNG uses 0:3 only.
        self._odom.append(
            (time.time(), float(msg.position.x), float(msg.position.y), float(msg.position.z))
        )

    def _on_tf(self, msg: TFMessage) -> None:
        wall = time.time()
        got_fix = False
        for tf in msg.transforms:
            if tf.frame_id == FRAME_WORLD and tf.child_frame_id == FRAME_MAP:
                self._world_map.append((wall, tf.to_matrix()))
                got_fix = True
        if got_fix and self._rr_enabled:
            try:
                self._log_accepted()
            except Exception:
                # Boundary: the overlay is an operator aid. A rerun or run-log
                # failure must not take down the /tf callback, and must not repeat
                # every republish interval -- one line, then stay quiet.
                self._rr_enabled = False
                logger.exception("RelocEval rerun overlay disabled after a failure")

    def _tail_health_lines(self) -> list[HealthLine]:
        """The accepts appended to the run log since the last read. Tailing by byte
        offset keeps the /tf callback bounded (no re-read from top per accept); a
        trailing partial line stays unconsumed, so its accept is not lost."""
        path = resolve_run_log(self.config.run_log_file)
        if path is None:
            return []
        with path.open("rb") as fh:
            fh.seek(self._log_pos)
            chunk = fh.read()
        cut = chunk.rfind(b"\n") + 1
        self._log_pos += cut
        return parse_health_lines(chunk[:cut].decode(errors="replace"))

    def _rerun_ready(self) -> bool:
        """True once this worker logs into the SAME rerun recording the bridge draws
        the premap into; False (a no-op overlay) whenever no viewer is up.

        rr.init derives its default recording id from multiprocessing's authkey, so
        bridge worker and eval worker (both forkserver) share ONE recording and these
        points land beside the premap. https://rerun.io/docs/howto/logging/shared-recordings
        """
        if self._rr_ready:
            return True
        host = self.config.g.rerun_host or self.config.g.listen_host
        if host in ("0.0.0.0", ""):
            host = "127.0.0.1"  # a wildcard LISTEN address is not a connect address
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.settimeout(_RERUN_PROBE_TIMEOUT_S)
            if probe.connect_ex((host, RERUN_GRPC_PORT)) != 0:
                return False  # retried on the next accept -- the bridge may still be starting

        import rerun as rr

        from dimos.visualization.rerun.init import rerun_init

        rerun_init()  # same app id + turbo annotation context the bridge registers
        rr.connect_grpc(url=f"rerun+http://{host}:{RERUN_GRPC_PORT}/proxy")
        # Parent the overlay to the premap's frame once -- static, because unlike the
        # bridge's per-message attachment this entity never changes frame.
        rr.log(
            RERUN_ACCEPTED_ENTITY,
            rr.Transform3D(parent_frame=RERUN_ACCEPTED_PARENT_FRAME),
            static=True,
        )
        self._rr_ready = True
        logger.info("RelocEval rerun overlay connected", entity=RERUN_ACCEPTED_ENTITY, host=host)
        return True

    def _log_accepted(self) -> None:
        """Redraw ``world/eval/accepted`` when a NEW accept lands. The full set is
        re-logged (not appended) so a fix that was ``unknown`` when its /tf arrived
        gets its colour the moment its log line shows up."""
        tf_fixes = dedup_tf_fixes(self._world_map)
        if len(tf_fixes) <= self._n_drawn:
            return  # a republish of the accept already drawn
        self._n_drawn = len(tf_fixes)
        self._health.extend(self._tail_health_lines())
        odom = np.array(self._odom, dtype=float) if self._odom else np.empty((0, 4), dtype=float)
        points = accepted_points_in_map(label_fixes_from_log(tf_fixes, self._health), odom)
        if not len(points) or not self._rerun_ready():
            return

        import rerun as rr

        rr.log(
            RERUN_ACCEPTED_ENTITY,
            rr.Points3D(
                positions=points.positions_map_m,
                colors=points.colors_rgb,
                # rerun shows labels below its instance-count threshold, on hover above.
                labels=points.labels,
                radii=RERUN_ACCEPTED_RADIUS_M,
            ),
        )

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
        self._tf_transport.stop()
        self._odom_transport.stop()
        super().stop()

    def _finalize(self) -> None:
        if self._finalized:
            return
        self._finalized = True
        tf_fixes = dedup_tf_fixes(self._world_map)
        log_path = resolve_run_log(self.config.run_log_file)
        health: list[HealthLine] = []
        census: list[dict[str, int]] = []
        reject_sources: list[str] | None = None
        if log_path is not None:
            health, census, reject_sources = parse_run_log(log_path)
        fixes = label_fixes_from_log(tf_fixes, health)

        odom = np.array(self._odom, dtype=float) if self._odom else np.empty((0, 4), dtype=float)
        markers_xy: dict[int, tuple[float, float]] = {}
        if self.config.marker_map_file:
            markers_xy = {
                i: (x, y)
                for i, (x, y, _z) in load_markers_xyz(Path(self.config.marker_map_file)).items()
            }

        note = (
            "live capture: no PGO truth in-process -> med_err/false omitted; the "
            "scored held-out accuracy comes from the offline held-out driver."
        )
        # accept_sources from the LOG accepts (not the /tf join): every accept names
        # its prior, so ACCEPTED per source is exact even where a join failed.
        n_rejects = None if reject_sources is None else len(reject_sources)
        stats = compute_stats(
            fixes,
            odom,
            census,
            n_rejects,
            mode="live",
            held_out_note=note,
            accept_sources=[h.source for h in health],
            reject_sources=reject_sources,
        )
        write_report(
            stats,
            odom,
            fixes,
            markers_xy,
            Path(self.config.out_dir),
            self.config.tag,
            title="RelocEval live capture",
        )
