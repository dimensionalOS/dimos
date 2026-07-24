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

"""RelocEval -- the in-process collector ``dimos run --eval`` attaches, plus the pure
analysis it drives. Trajectory/coverage/counts come from the real ``/tf`` + ``/odom``
streams; the run log is a supplement carrying each accept's winning prior, fitness, and
the per-cycle census."""

from __future__ import annotations

import atexit
from collections import Counter
from dataclasses import dataclass
import json
from pathlib import Path
import re
import time
from typing import Any

import numpy as np

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

SOURCES: tuple[str, ...] = ("ransac", "fiducial")
_TF_DEDUP_EPS_M = 1e-4  # two world->map translations within this are the same accept
_TF_DEDUP_EPS_RAD = 1e-4  # ...and within this rotation (a yaw-only fix keeps the origin)


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
# Fixes + stats
# --------------------------------------------------------------------------- #
@dataclass
class Fix:
    """One accepted relocalization, timestamped in wall seconds -- the same clock as
    the odom it is scored against."""

    ts: float
    world_map_fix: np.ndarray | None  # 4x4 world_T_map (None if rotation not captured)
    source: str
    fitness: float


@dataclass
class SourceRow:
    source: str
    won: int
    pct_traj: float
    med_fit: float | None
    # Per-source activity from the run-log events -- independent of the /tf join, so
    # these never degrade to ``unknown`` the way a join-labelled fix's source can.
    proposed: int = 0  # cycles this source put >=1 candidate forward (census PRESENCE)
    accepted: int = 0  # `relocalize accepted` log lines this source won
    rejected: int | None = None  # `relocalize rejected` lines; None = no source parsed


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
    accept_sources: list[str] | None = None,
    reject_sources: list[str] | None = None,
) -> EvalStats:
    """Per-source + overall stats. ``fixes`` and ``odom_txyz`` MUST share a clock.

    Args:
        odom_txyz: (N, 4) [ts, x, y, z] robot positions in the world (LIO/odom) frame,
            or an empty array. A wrong-width array raises rather than mis-indexing.
        accept_sources: the source of every ``relocalize accepted`` log line. None ->
            fall back to the joined fixes' sources.
        reject_sources: the source of every reject. None -> not parsed, so per-source
            ``rej`` renders ``-``, not 0.
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
        rows.append(
            SourceRow(
                source=s,
                won=len(s_fixes),
                pct_traj=(100.0 * traj / n_covered) if n_covered else 0.0,
                med_fit=_median([f.fitness for f in s_fixes if not np.isnan(f.fitness)]),
                proposed=proposed_by.get(s, 0),
                accepted=accepted_by.get(s, 0),
                rejected=None if rejected_by is None else rejected_by.get(s, 0),
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
    """The per-source table, a TOTAL row, then the prior-activity line and overall."""
    header = ["source", "prop", "acc", "rej", "%traj", "med_fit"]

    body: list[list[str]] = [header]
    for r in stats.rows:
        body.append(
            [
                r.source,
                str(r.proposed),
                str(r.accepted),
                _count_cell(r.rejected),
                _fmt(r.pct_traj, 1, "%"),
                _fmt(r.med_fit, 3),
            ]
        )
    # TOTAL: the count columns sum; medians and %traj do not aggregate, so they blank.
    total_rej = (
        None
        if any(r.rejected is None for r in stats.rows)
        else sum(r.rejected or 0 for r in stats.rows)
    )
    total_row = [
        "TOTAL",
        str(sum(r.proposed for r in stats.rows)),
        str(sum(r.accepted for r in stats.rows)),
        _count_cell(total_rej),
        "",
        "",
    ]

    widths = [max(len(row[i]) for row in (*body, total_row)) for i in range(len(header))]
    sep = "  ".join("-" * widths[j] for j in range(len(header)))
    lines = [f"=== RelocEval: {title} ==="]
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
    return "\n".join(lines)


# --------------------------------------------------------------------------- #
# The Module (attached by `dimos run --eval`)
# --------------------------------------------------------------------------- #
class EvalConfig(ModuleConfig):
    # Run log for the census supplement. None -> this run's own main.jsonl.
    run_log_file: str | None = None


class RelocEval(Module):
    """In-process collector: subscribes to ``/tf`` + ``/odom`` and at stop() prints the
    per-source reloc table. Best-effort: a finalize failure may not crash the run."""

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
        # z keeps odom at (N, 4) [ts, x, y, z], the shape compute_stats validates.
        self._odom.append(
            (time.time(), float(msg.position.x), float(msg.position.y), float(msg.position.z))
        )

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
        # accept_sources from the LOG accepts (not the /tf join): every accept names
        # its prior, so ACCEPTED per source is exact even where a join failed.
        n_rejects = None if reject_sources is None else len(reject_sources)
        stats = compute_stats(
            fixes,
            odom,
            census,
            n_rejects,
            accept_sources=[h.source for h in health],
            reject_sources=reject_sources,
        )
        print(format_report(stats, title="RelocEval live capture"))
