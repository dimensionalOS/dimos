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

That supplement needs the module's VERBOSE trace
(``relocalizationmodule.verbose_eval_logging=true``), which is what carries
``published_t_m`` -- the translation this collector joins a log line to its /tf fix
by -- and the per-cycle census. An operating run logs one QUIET accept line instead
(source/fitness/margin/time_cost_s, no position); the parsers read it too, but such
a line cannot be joined to a fix, so its source degrades to ``unknown``.

HELD-OUT ACCURACY. Premap + marker map come from run A; run B is a DIFFERENT
traversal of the same scene. A fix is ``world_B_T_map_A`` -- in A's map frame, while
B's PGO truth lives in B's independent map frame, so a raw distance is inflated by
the map_A<->map_B offset. ``resolve_heldout_alignment`` recovers the rigid
``map_B_T_map_A`` from markers surveyed in both frames (Umeyama), making per-source
``med_err`` a real metre value. Without those shared references the error prints
``-`` with the reason, never a fabricated number (a house non-negotiable).

HOW MUCH IT CORRECTS. Accuracy says whether a fix is right; ``CorrectionStats`` says
how much work it did -- the jump in the ROBOT's believed map-frame position between
consecutive accepts, and that jump per metre driven between them (the LIO drift rate
relocalization is removing). Measured at the robot, never at the map origin: the
fix's own translation is the map origin, which barely moves between accepts and would
report a flatteringly-near-zero correction.

WHERE EACH PRIOR WINS. Under ``--eval`` the Module also draws every accept into the
run's rerun viewer as it lands -- ``world/eval/accepted``, one point per accepted fix
at the robot's position in the premap's map frame, coloured by the winning prior
(``accepted_points_in_map``). The spatial answer the table cannot give; a no-op when
no viewer is up.

The Module always runs "live" (no in-process truth -> med_err/success omitted); the
offline driver runs "held_out" (columns shown, Umeyama-aligned or ``-``). Both
render through the SAME ``format_report``/``plot_trajectory``/``write_report``, so
the two paths cannot drift.
"""

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

SOURCES: tuple[str, ...] = ("ransac", "fiducial", "last_pose")
# ONE palette for both renderers (trajectory PNG + the rerun overlay), so the two
# can never disagree about which prior a colour means. unknown is a dim RED, not a
# second grey: an unlabelled fix is a JOIN FAILURE, and it must read as one next to
# last_pose rather than hide inside it.
SOURCE_COLORS: dict[str, str] = {
    "ransac": "#2b6cb0",  # blue
    "fiducial": "#dd6b20",  # orange
    "last_pose": "#718096",  # grey
    "unknown": "#9b2c2c",  # dim red
}
SUCCESS_T_M = 1.0  # held-out translation gate (m); matches score_replay.SUCCESS_T_M
_TF_DEDUP_EPS_M = 1e-4  # two world->map translations within this are the same accept
_TF_DEDUP_EPS_RAD = 1e-4  # ...and within this rotation (a yaw-only fix keeps the origin)
_COLLINEAR_RATIO = 1e-2  # shared-tag 2nd/1st singular value below this -> collinear
_TAG_SPREAD_MIN_M = 1e-6  # principal spread below this -> the shared tags coincide
# Correction-per-metre needs a real baseline: the go2 walks ~0.5 m/s against a ~2 s
# accept cadence, so 5 cm between two accepts means the robot stood still and the
# ratio would be odom noise in the denominator. Report the magnitude, no rate.
_MIN_TRAVEL_M = 0.05

# The live rerun overlay (see accepted_points_in_map / RelocEval._log_accepted).
# "world/" is the bridge's entity_prefix, so the overlay sits under the same
# Spatial3DView origin as every other run entity.
RERUN_ACCEPTED_ENTITY = "world/eval/accepted"
# The premap is logged with frame_id "map" (module.py) and the bridge parents that
# entity to tf#/map, so the overlay names the same frame to land on top of it.
RERUN_ACCEPTED_PARENT_FRAME = "tf#/map"
RERUN_ACCEPTED_RADIUS_M = 0.25  # premap voxels draw at ~0.025 m radius; 10x reads over them
_RERUN_PROBE_TIMEOUT_S = 0.2  # the bridge's proxy is local -- it answers or it is not up


# --------------------------------------------------------------------------- #
# Log parsing (supplement) -- ONE key-based parser for BOTH renderings the same
# structlog call produces: main.jsonl's JSON object (what resolve_run_log picks by
# default) and the console line a tee'd `.replay_run.log` holds. Both carry the
# module's kwarg NAMES, so a record reads the same either way and downstream code
# never learns which file it came from. What the emitters actually write:
#
#   module.py:_try_relocalize accept, VERBOSE (verbose_eval_logging=True)
#     jsonl   {"source": "fiducial", "fitness": 0.87, "published_t_m": [1.234, -0.5,
#              0.02], ..., "event": "relocalize accepted", "level": "info", ...}
#     console `08:34:01.794 [inf][...module.py] relocalize accepted fitness=0.87
#              n_pts=55828 published_t_m=[1.234, -0.5, 0.02] ... source=fiducial ...`
#   module.py:_try_relocalize accept, QUIET (the operating default)
#     console `... relocalize accepted fitness=0.873 margin=0.178 source=fiducial
#              time_cost_s=1.4`
#   module.py reject          -> event "relocalize rejected"
#   priors.py census          -> event "relocalize candidates", counts a dict;
#                                console renders it as a python repr,
#                                `counts={'ransac': 34, 'fiducial': 2}`.
#                                VERBOSE only -- a quiet run emits no census.
#
# AND the LEGACY rendering, from before the emitters moved to structlog kwargs --
# one f-string per record, bare key names:
#
#   `relocalize: fitness=0.756 time_cost=13.0s n_pts=89003 reloc_t=[...]
#    TF 'world' -> 'map' published_t=[-8.849, -1.587, 0.073] source=ransac`
#   census: `relocalize candidates: fiducial=1 ransac=34`
#
# Every archived capture is in that format, so dropping it does not fail -- it
# returns [] and the census reads as "the fiducial prior never proposed". Same
# two formats, same key-anchored approach, as trial/harness/reloc_log.py.
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
    "margin": re.compile(r"\bmargin=([-\d.eE+]+)"),  # quiet accepts only
    "published_t_m": re.compile(r"\bpublished_t(?:_m)?=\[([^\]]+)\]"),
    "counts": re.compile(r"\bcounts=\{([^}]*)\}"),
}
_COUNT_RE = re.compile(r"'(\w+)':\s*(\d+)")  # one entry of counts={...}'s dict repr
_LEGACY_COUNT_RE = re.compile(r"\b([a-z_]+)=(\d+)")  # legacy census: `ransac=34 fiducial=2`


@dataclass
class HealthLine:
    """One accepted relocalize, parsed from a ``relocalize accepted`` log record.

    ``published_t_m`` is the VERBOSE trace's field and the key every fix is joined by,
    so a QUIET line parses to a positionless HealthLine: its fitness and winning prior
    are still real, there is simply nowhere to attach them (see label_fixes_from_log).
    ``margin`` is the mirror case -- quiet-only, because verbose carries the whole
    finalist table instead."""

    source: str
    fitness: float
    published_t_m: tuple[float, float, float] | None  # world_T_map translation (join key)
    margin: float | None = None  # winner's fitness - best other source's; None = no rival


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
    margin = _CONSOLE_RE["margin"].search(text)
    if margin is not None:
        fields["margin"] = float(margin.group(1))
    pub = _CONSOLE_RE["published_t_m"].search(text)
    if pub is not None:
        fields["published_t_m"] = _floats(pub.group(1))
    counts = _CONSOLE_RE["counts"].search(text)
    if counts is not None:
        fields["counts"] = {k: int(n) for k, n in _COUNT_RE.findall(counts.group(1))}
    elif _EVENT_CENSUS in text:
        # Legacy census: the counts ARE the kwargs -- `relocalize candidates:
        # ransac=34 fiducial=2`. Read only the tail, so nothing before the event
        # name (a timestamp, a pid) can be mistaken for a proposal count.
        legacy = _LEGACY_COUNT_RE.findall(text.split(_EVENT_CENSUS, 1)[1])
        if legacy:
            fields["counts"] = {k: int(n) for k, n in legacy}
    return fields


def _event_of(text: str, obj: dict[str, Any] | None) -> str | None:
    """Which record this line is, in either format and either rendering. jsonl
    states it in ``event``; a console line renders the event verbatim into the
    message, and a LEGACY jsonl line carries the whole f-string as its event."""
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
    """``(event, kwargs)`` for one relocalize log line in EITHER rendering and
    EITHER wire format, or None when the line is none of them (another module's
    log, a banner, a blank).

    Console matches and jsonl keys are merged, jsonl winning: a current jsonl
    record carries its kwargs as real typed values, while a legacy jsonl record
    hides them inside the ``event`` f-string, where the same key-anchored regexes
    find them (JSON escaping leaves `key=value` untouched)."""
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
    line carries one). ``source`` is absent on the single-source RANSAC path --
    module.py tags a winner only when the multi-prior judge ran -- so it defaults to
    ransac there.

    A QUIET accept has no ``published_t_m``; it still yields a HealthLine, positionless.
    Dropping it instead would make a quiet run's log indistinguishable from a log the
    parser cannot read at all -- both empty, no warning, and every count silently zero.
    ``fitness`` is the one required field: it is what makes a line an accept."""
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
        margin = fields.get("margin")
        out.append(
            HealthLine(
                source=str(fields.get("source", "ransac")),
                fitness=float(fit),
                published_t_m=(
                    (float(pub[0]), float(pub[1]), float(pub[2]))
                    if isinstance(pub, list) and len(pub) == 3
                    else None
                ),
                margin=None if margin is None else float(margin),
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
    the line named none. Its length is the reject count.

    Every live reject names it: module.py wires ``source=`` into BOTH branches (quiet
    and verbose) and onto the solo RANSAC path too, because the line prints that
    prior's own ``threshold=``. Only the legacy f-string format predates the field, so
    an archived capture buckets to ``unknown`` rather than fabricating a prior."""
    out: list[str] = []
    for raw in log_text.splitlines():
        record = _parse_line(raw)
        if record is None or record[0] != _EVENT_REJECT:
            continue
        out.append(str(record[1].get("source", "unknown")))
    return out


def parse_run_log(path: Path) -> tuple[list[HealthLine], list[dict[str, int]], list[str]]:
    """Every relocalize record in a run log FILE: ``(accepts, census, reject_sources)``.
    ``reject_sources`` is one entry per reject (its winning source, or ``unknown``); its
    length is the reject count.

    A log the parser reads NOTHING out of is announced, because downstream it is
    indistinguishable from a real measurement: no census reads as "the fiducial
    prior never proposed", and every fix falls back to ``unknown``. That is not
    hypothetical -- dropping the legacy f-string format silently turned a 24/28
    fiducial-proposal census into ``null`` on every archived capture, with no
    error anywhere. Zero records means the parser or the log is wrong; say so.
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
    world_T_map: np.ndarray, truth_map_B_T_world_B: np.ndarray, map_B_T_map_A: np.ndarray
) -> float:
    """Held-out translation error (m): the fix's world origin, carried from map_A
    into map_B by the alignment, versus B's PGO-truth world origin.

    ``est = map_B_T_map_A @ inv(world_T_map)`` gives ``map_B_T_world_B``; compare
    its translation to ``truth`` (also map_B_T_world_B, from score_replay)."""
    est_map_a = np.linalg.inv(world_T_map)  # map_A_T_world_B
    est_map_b = map_B_T_map_A @ est_map_a  # map_B_T_world_B
    return float(np.linalg.norm(est_map_b[:3, 3] - truth_map_B_T_world_B[:3, 3]))


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
    margin: float | None = None  # cross-source fitness margin, when the log carried one


@dataclass
class SourceRow:
    source: str
    won: int
    pct_traj: float
    med_err_m: float | None
    med_fit: float | None
    n_success: int | None
    n_judged: int | None
    # Per-source activity read straight from the run-log census + accept + reject
    # events -- independent of the /tf join, so these never degrade to ``unknown``
    # the way a join-labelled fix's source can.
    proposed: int = 0  # cycles this source put >=1 candidate forward (census PRESENCE)
    accepted: int = 0  # `relocalize accepted` log lines this source won
    rejected: int | None = None  # `relocalize rejected` lines; None = no source parsed
    n_false: int | None = None  # accepts beyond SUCCESS_T_M (a wrong accept); None = no truth


@dataclass
class Correction:
    """How much ONE accepted fix moved the robot's believed map-frame pose, and how
    far the robot drove to earn that correction."""

    ts: float
    source: str  # the winning prior of the NEW fix -- the one that did the correcting
    magnitude_m: float
    dyaw_deg: float  # |yaw| change between the two corrections
    dist_travelled_m: float
    rate_m_per_m: float | None  # None while stationary (see _MIN_TRAVEL_M)


@dataclass
class CorrectionSourceRow:
    source: str
    n: int
    med_magnitude_m: float | None
    med_rate_m_per_m: float | None


@dataclass
class CorrectionStats:
    """Run-level correction magnitude. ``first`` (acquisition -- correcting from no
    fix at all) is kept out of every aggregate below: it measures how far off the
    robot started, not the drift rate the run is removing."""

    first: Correction | None
    per_fix: list[Correction]  # drift corrections only; ``first`` is NOT in here
    med_magnitude_m: float | None
    p90_magnitude_m: float | None
    med_dyaw_deg: float | None
    med_rate_m_per_m: float | None
    total_correction_m: float
    total_distance_m: float  # driven between the first and the last accepted fix
    rows: list[CorrectionSourceRow]

    @property
    def rate_m_per_m(self) -> float | None:
        if self.total_distance_m < _MIN_TRAVEL_M:
            return None
        return self.total_correction_m / self.total_distance_m


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
    corrections: CorrectionStats | None = None


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
    Unmatched fixes keep source ``unknown`` -- the trajectory still stands.

    A positionless (QUIET) health line carries no join key, so it labels nothing and
    every fix in a quiet run reads ``unknown`` -- which is what ``--eval`` turning the
    verbose trace on exists to prevent."""
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
                margin=matched.margin if matched else None,
            )
        )
    return fixes


def _yaw_deg(rot: np.ndarray) -> float:
    """Yaw (deg) about the gravity axis of a gravity-aligned rotation, wrapped to
    (-180, 180] by atan2. Both frames here come from the LIO's gravity-aligned world,
    so roll/pitch of a correction are ~0 and yaw is the whole story."""
    return float(np.degrees(np.arctan2(rot[1, 0], rot[0, 0])))


def correction_at_robot(
    world_map_old: np.ndarray | None, world_map_new: np.ndarray, p_world_m: np.ndarray
) -> tuple[float, float]:
    """``(magnitude_m, |dyaw| deg)`` -- how far the ROBOT's believed map-frame
    position jumped when ``world_map_new`` replaced ``world_map_old``.

    FRAME, and why it is the load-bearing line here: a published fix is
    ``world_T_map`` (module.py inverts relocalize()'s map_T_world before publishing
    world->map on /tf), so the robot's map-frame belief is
    ``p_map = inv(fix) @ p_world`` -- the same ``map_T_world = inv(fix)`` mapping
    plot_trajectory and accepted_points_in_map use. Using the fix directly instead
    yields a plausible-looking number that is simply a different quantity.

    Evaluated AT THE ROBOT, never at the map origin: ``fix[:3, 3]`` is where the map
    origin sits in world, which two consecutive accepts barely move, so differencing
    it reports a near-zero correction no matter how large the real one was.

    ``world_map_old=None`` means the FIRST fix -- no world->map had been published, so
    the robot had no map-frame belief at all; identity is the convention for "no
    correction yet", which makes this the acquisition offset, not a drift correction.

    Args:
        world_map_old: 4x4 world_T_map of the previous accept, or None for the first.
        world_map_new: 4x4 world_T_map of this accept.
        p_world_m: (3,) robot position in the world (LIO/odom) frame at the new fix.
    """
    map_T_world_new = np.linalg.inv(world_map_new)
    map_T_world_old = np.eye(4) if world_map_old is None else np.linalg.inv(world_map_old)
    p_h = np.array([*np.asarray(p_world_m, dtype=float)[:3], 1.0])
    jump = (map_T_world_new @ p_h)[:3] - (map_T_world_old @ p_h)[:3]
    r_rel = map_T_world_old[:3, :3].T @ map_T_world_new[:3, :3]
    return float(np.linalg.norm(jump)), abs(_yaw_deg(r_rel))


def _path_distance_m(odom_xyz: np.ndarray, i_from: int, i_to: int) -> float:
    """Path length (m) integrated over odom samples ``i_from..i_to`` inclusive. The
    straight-line distance would undercount a curved or doubled-back leg, and it is
    driven distance -- not displacement -- that LIO drift accumulates over."""
    if i_to <= i_from:
        return 0.0
    steps = np.diff(odom_xyz[i_from : i_to + 1], axis=0)
    return float(np.linalg.norm(steps, axis=1).sum())


def _correction_rows(fixes_sorted: list[Fix], odom_txyz: np.ndarray) -> list[Correction]:
    """One Correction per accepted fix, in time order; row 0 is the acquisition fix.
    The robot position at an accept is the nearest odom sample (odom runs 10-50 Hz
    against a ~2 s accept cadence), and the same sample indices bound the leg the
    distance is integrated over, so magnitude and denominator share one convention."""
    usable: list[tuple[Fix, np.ndarray]] = [
        (f, f.world_map_fix) for f in fixes_sorted if f.world_map_fix is not None
    ]
    if not usable or odom_txyz.size == 0:
        return []
    odom_ts, odom_xyz = odom_txyz[:, 0], odom_txyz[:, 1:4]
    idx = [int(np.argmin(np.abs(odom_ts - f.ts))) for f, _ in usable]
    rows: list[Correction] = []
    for k, (f, mat) in enumerate(usable):
        prev = usable[k - 1][1] if k else None
        magnitude_m, dyaw_deg = correction_at_robot(prev, mat, odom_xyz[idx[k]])
        dist_m = _path_distance_m(odom_xyz, idx[k - 1], idx[k]) if k else 0.0
        rows.append(
            Correction(
                ts=f.ts,
                source=f.source,
                magnitude_m=magnitude_m,
                dyaw_deg=dyaw_deg,
                dist_travelled_m=dist_m,
                rate_m_per_m=(magnitude_m / dist_m) if dist_m >= _MIN_TRAVEL_M else None,
            )
        )
    return rows


def compute_correction_stats(
    fixes_sorted: list[Fix], odom_txyz: np.ndarray
) -> CorrectionStats | None:
    """Aggregate correction magnitude + correction-per-metre over a run's accepts.

    None when there is nothing honest to report (no accepted fix carrying a pose, or
    no odom -- with no robot position there is no place to evaluate the jump).

    Args:
        fixes_sorted: accepted fixes, ascending ts.
        odom_txyz: (N, 4) [ts, x, y, z] robot positions in world, same clock as fixes.
    """
    rows = _correction_rows(fixes_sorted, odom_txyz)
    if not rows:
        return None
    first, rest = rows[0], rows[1:]
    mags = [c.magnitude_m for c in rest]
    rates = [c.rate_m_per_m for c in rest if c.rate_m_per_m is not None]
    present = [s for s in SOURCES if any(c.source == s for c in rest)]
    present += sorted({c.source for c in rest} - set(SOURCES))
    return CorrectionStats(
        first=first,
        per_fix=rest,
        med_magnitude_m=_median(mags),
        p90_magnitude_m=float(np.percentile(mags, 90)) if mags else None,
        med_dyaw_deg=_median([c.dyaw_deg for c in rest]),
        med_rate_m_per_m=_median(rates),
        total_correction_m=float(sum(mags)),
        total_distance_m=float(sum(c.dist_travelled_m for c in rest)),
        rows=[
            CorrectionSourceRow(
                source=s,
                n=sum(1 for c in rest if c.source == s),
                med_magnitude_m=_median([c.magnitude_m for c in rest if c.source == s]),
                med_rate_m_per_m=_median(
                    [c.rate_m_per_m for c in rest if c.source == s and c.rate_m_per_m is not None]
                ),
            )
            for s in present
        ],
    )


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
            or an empty array. The POSITIONS are required, not just the timestamps:
            correction magnitude is evaluated at the robot. A wrong-width array raises
            rather than being indexed as if [ts, x, y] were [ts, x, y, z].
        accept_sources: the source of every ``relocalize accepted`` log line (the
            ACCEPTED-per-source count, straight from the log so a /tf join failure does
            not lose it). None -> fall back to the joined fixes' sources, which is what
            the held-out driver wants (its fixes ARE the pipeline's published accepts).
        reject_sources: the source of every reject (``parse_reject_lines``). None -> the
            run log was not parsed, so per-source ``rej`` is held out (``-``), not 0.
    """
    odom_txyz = np.asarray(odom_txyz, dtype=float)
    if odom_txyz.size and (odom_txyz.ndim != 2 or odom_txyz.shape[1] != 4):
        raise ValueError(
            f"compute_stats: odom must be (N,4) [ts,x,y,z], got {odom_txyz.shape}"
        )
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

    # PROPOSED is per-CYCLE PRESENCE, not candidate count: one source can offer many
    # candidates in a cycle, so counting candidates would reward a chatty prior. The
    # question the table answers is "in how many relocalize cycles did this source put
    # a candidate forward" -- the same convention as fiducial_proposed_cycles below.
    proposed_by = {
        s: sum(1 for c in census if c.get(s, 0) > 0) for s in {k for c in census for k in c}
    }
    acc_src = accept_sources if accept_sources is not None else [f.source for f in fixes]
    accepted_by = Counter(acc_src)
    rejected_by = Counter(reject_sources) if reject_sources is not None else None
    # SOURCES first (always shown, even at zero), then any other source that appears in
    # the census / accepts / rejects / joined fixes -- a new prior, or ``unknown`` from a
    # join failure or a source-less verbose reject. Same ordering as compute_correction_stats.
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
                # FALSE reuses the held-out accuracy gate already applied upstream
                # (Fix.success = err_t_m < SUCCESS_T_M): a judged accept that failed it.
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
        corrections=compute_correction_stats(fixes_sorted, odom_txyz),
    )


# --------------------------------------------------------------------------- #
# Rendering
# --------------------------------------------------------------------------- #
def _fmt(x: float | None, nd: int, suffix: str = "") -> str:
    return "-" if x is None else f"{x:.{nd}f}{suffix}"


def _format_corrections(cs: CorrectionStats | None) -> list[str]:
    """The correction block: one summary line, the acquisition fix on its own, then
    per-source. Three lines, not a per-fix dump -- the per-fix rows are in the json."""
    if cs is None:
        return ["correction: - (no accepted fix with odom to measure a jump at)"]
    first = (
        "  first fix: -"
        if cs.first is None
        else f"  first fix: {cs.first.magnitude_m:.3f}m {cs.first.dyaw_deg:.1f}deg "
        f"({cs.first.source}, acquisition -- excluded above)"
    )
    if not cs.per_fix:
        return ["correction: - (only the acquisition fix; no second accept to compare)", first]
    by_source = "  ".join(
        f"{r.source} {_fmt(r.med_magnitude_m, 3, 'm')} {_fmt(r.med_rate_m_per_m, 4, 'm/m')} "
        f"(n={r.n})"
        for r in cs.rows
    )
    return [
        f"correction (at the robot, n={len(cs.per_fix)}): med={_fmt(cs.med_magnitude_m, 3, 'm')} "
        f"p90={_fmt(cs.p90_magnitude_m, 3, 'm')} med_yaw={_fmt(cs.med_dyaw_deg, 1, 'deg')} "
        f"med_rate={_fmt(cs.med_rate_m_per_m, 4, 'm/m')}; total {cs.total_correction_m:.3f}m "
        f"over {cs.total_distance_m:.2f}m driven ({_fmt(cs.rate_m_per_m, 4, 'm/m')})",
        first,
        f"  by source: {by_source}",
    ]


def _count_cell(x: int | None) -> str:
    """A count cell: the number, or ``-`` when the datum was not derivable (no truth
    for FALSE, no parsed reject source for REJECTED). Never a fabricated 0."""
    return "-" if x is None else str(x)


def format_report(stats: EvalStats, *, title: str) -> str:
    """The per-source table [source | prop | acc | rej | (false) | %traj | (med_err) |
    med_fit], a TOTAL row, then the prior-activity line, overall, corrections, and the
    held-out note. ``false`` and ``med_err`` are dropped in live mode (no in-process
    truth). Columns:

      prop  -- cycles this source put >=1 candidate forward (run-log census presence)
      acc   -- accepts this source won and published (``relocalize accepted`` lines)
      rej   -- rejects this source's winning candidate took (``relocalize rejected``);
               ``-`` when the run log carried no source to attribute the reject by
      false -- accepts whose held-out error exceeded SUCCESS_T_M (a WRONG accept)

    ``won``/``success`` stay in the JSON (``stats_to_dict``) for downstream readers; the
    printed table shows ``acc``/``false``, the same quantities read straight from the log
    and the truth gate."""
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
    lines.extend(_format_corrections(stats.corrections))
    lines.append(stats.held_out_note)
    return "\n".join(lines)


def _correction_to_dict(c: Correction) -> dict[str, Any]:
    return {
        "ts": round(c.ts, 3),
        "source": c.source,
        "magnitude_m": round(c.magnitude_m, 4),
        "dyaw_deg": round(c.dyaw_deg, 3),
        "dist_travelled_m": round(c.dist_travelled_m, 4),
        "rate_m_per_m": None if c.rate_m_per_m is None else round(c.rate_m_per_m, 5),
    }


def _round(x: float | None, nd: int) -> float | None:
    return None if x is None else round(x, nd)


def corrections_to_dict(cs: CorrectionStats | None) -> dict[str, Any] | None:
    """The machine-readable form, per-fix rows included -- these are the numbers a
    trend tracker diffs across runs, so they ship in full even though the printed
    report shows only the summary."""
    if cs is None:
        return None
    return {
        "n": len(cs.per_fix),
        "med_magnitude_m": _round(cs.med_magnitude_m, 4),
        "p90_magnitude_m": _round(cs.p90_magnitude_m, 4),
        "med_dyaw_deg": _round(cs.med_dyaw_deg, 3),
        "med_rate_m_per_m": _round(cs.med_rate_m_per_m, 5),
        "total_correction_m": round(cs.total_correction_m, 4),
        "total_distance_m": round(cs.total_distance_m, 4),
        "rate_m_per_m": _round(cs.rate_m_per_m, 5),
        "first_fix": None if cs.first is None else _correction_to_dict(cs.first),
        "per_source": [
            {
                "source": r.source,
                "n": r.n,
                "med_magnitude_m": _round(r.med_magnitude_m, 4),
                "med_rate_m_per_m": _round(r.med_rate_m_per_m, 5),
            }
            for r in cs.rows
        ],
        "per_fix": [_correction_to_dict(c) for c in cs.per_fix],
    }


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
        "correction": corrections_to_dict(stats.corrections),
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
    """Top-down robot path in map_A frame, coloured by the winning prior of the fix
    active at each sample, with surveyed markers as stars. odom_txy: (N,3) [ts,x,y]
    in the world (odom) frame; each covered sample is mapped by inv(active fix)."""
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
            f"tag {tid}", (mx, my), textcoords="offset points",
            xytext=(6, 6), fontsize=9, weight="bold",
        )
    present = [s for s in SOURCES if any(f.source == s for f in fx)]
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


def _rgb(hex_color: str) -> tuple[int, int, int]:
    """'#rrggbb' -> (r, g, b) 0-255, the form rerun wants."""
    h = hex_color.lstrip("#")
    return (int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16))


@dataclass
class AcceptedPoints:
    """The rerun overlay for a run's accepted fixes: one coloured, labelled point
    each, in the MAP frame. Pure data -- built without rerun, logged by the Module."""

    positions_map_m: list[tuple[float, float, float]]
    colors_rgb: list[tuple[int, int, int]]
    labels: list[str]

    def __len__(self) -> int:
        return len(self.positions_map_m)


def accepted_points_in_map(fixes: list[Fix], odom_txyz: np.ndarray) -> AcceptedPoints:
    """One point per accepted fix, at the ROBOT's position when that fix landed,
    expressed in the premap's map frame, coloured by the prior that won.

    FRAME. A fix is ``world_T_map`` (module.py inverts relocalize()'s map_T_world and
    publishes world->map on /tf), so ``map_T_world = inv(fix)`` carries a world-frame
    (LIO/odom) point into the premap's frame. That is the frame the premap is drawn
    in -- module.py stamps the loaded premap ``frame_id = "map"`` and the rerun bridge
    parents that entity to ``tf#/map`` -- and it is the SAME mapping plot_trajectory
    uses, so the PNG and the viewer cannot drift apart.

    The robot's position at accept time is the odom sample nearest in the shared wall
    clock (odom runs at ~10-50 Hz against a ~2 s accept cadence). The fix's own
    translation is deliberately NOT used: that is the map ORIGIN in world, which
    barely moves between accepts and would draw every prior's win in one blob.

    Args:
        fixes: accepted fixes, ``dedup_tf_fixes`` -> ``label_fixes_from_log``.
        odom_txyz: (N, 4) [wall_ts, x, y, z] robot positions in the world frame.
    Returns:
        AcceptedPoints, 1:1 with ``fixes`` -- empty when no odom was captured (with
        no robot position there is nowhere honest to draw a fix).
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
    """Print the table + correction block, then write ``<key>.eval.json`` and
    ``<key>.trajectory.png``.

    Args:
        odom_txyz: (N, 4) [ts, x, y, z] robot positions in the world frame; the plot
            is top-down and takes [ts, x, y].
    """
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


def load_odom_txyz(recording_db: Path) -> np.ndarray:
    """(N,4) [recording_ts, x, y, z] from a recording's ``odom`` stream. z is carried
    because correction magnitude and driven distance are 3D -- a stairs traversal
    moves mostly in z, and a top-down projection would under-report both."""
    from dimos.memory2.store.sqlite import SqliteStore

    store = SqliteStore(path=str(recording_db), must_exist=True)
    rows: list[tuple[float, float, float, float]] = []
    with store:
        for o in store.stream("odom", PoseStamped).to_list():
            rows.append((float(o.ts), float(o.data.x), float(o.data.y), float(o.data.z)))
    return np.array(rows, dtype=float) if rows else np.empty((0, 4), dtype=float)


def _fixes_from_replay_json(fixes_json: list[dict[str, Any]]) -> list[Fix]:
    out: list[Fix] = []
    for f in fixes_json:
        mat = f.get("world_map_fix")
        margin = f.get("margin")  # absent in replay.json written before the judge reported one
        out.append(
            Fix(
                ts=float(f["ts"]),
                world_map_fix=np.asarray(mat, dtype=float) if mat is not None else None,
                source=str(f.get("source", "unknown")),
                fitness=float(f.get("fitness", float("nan"))),
                margin=None if margin is None else float(margin),
            )
        )
    return out


def run_offline_report(
    replay_json: Path,
    recording_db: Path,
    marker_map_a_file: Path,
    out_dir: Path,
    key: str,
    *,
    title: str,
    run_log: Path | None = None,
    score_json: Path | None = None,
    marker_map_b_file: Path | None = None,
) -> dict[str, Any]:
    """Held-out report from CAPTURED artifacts -- no replay launched. Consumes the
    real pipeline's published fixes (replay_bench's replay.json), B's odom, and A's
    marker survey; enriches per-source ``med_err`` via Umeyama IF run-B references
    (marker_map_b_file) and per-fix truth (score_json) are both available, else ``-``.

    The scoring itself is score_replay's job -- this only aligns published fixes and
    reports; it re-implements no dimos data-path step.
    """
    replay = json.loads(replay_json.read_text())
    fixes = _fixes_from_replay_json(replay["fixes"])
    meta = replay.get("meta", {})

    markers_a = load_markers_xyz(marker_map_a_file)
    markers_b = (
        load_markers_xyz(marker_map_b_file)
        if marker_map_b_file and marker_map_b_file.exists()
        else None
    )
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
    reject_sources: list[str] | None = None
    log_rejects: int | None = None
    if run_log is None:
        cand = replay_json.with_name(replay_json.name.replace(".replay.json", ".replay_run.log"))
        run_log = cand if cand.exists() else None
    if run_log is not None:
        _health, census, reject_sources = parse_run_log(run_log)
        log_rejects = len(reject_sources)
    n_rejects = meta.get("n_rejects")
    if n_rejects is None:
        n_rejects = log_rejects

    note = (
        "held-out: premap+markers from run A, replay run B of the same scene; map "
        f"frames are independent PGO runs. med_err alignment: {align_reason}."
    )
    odom = load_odom_txyz(recording_db)
    # accept_sources default (the replay fixes' own sources): held-out fixes ARE the
    # pipeline's published accepts, so no separate log-accept join is needed here.
    stats = compute_stats(
        fixes, odom, census, n_rejects, mode="held_out", held_out_note=note,
        reject_sources=reject_sources,
    )
    markers_xy = {i: (x, y) for i, (x, y, _z) in markers_a.items()}
    paths = write_report(stats, odom, fixes, markers_xy, out_dir, key, title=title)
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
    """In-process collector: subscribes to the real ``/tf`` + ``/odom`` streams,
    draws each accept into rerun as it lands (``world/eval/accepted``, coloured by
    the winning prior), and at stop() prints the per-source table + writes json + a
    top-down trajectory PNG. Always "live" -- no in-process PGO truth, so
    med_err/success are the offline driver's job. Best-effort: neither the overlay
    nor a finalize failure may crash the run.
    """

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
        # z is carried for the rerun overlay (a 3D point over the premap); the report
        # and the top-down PNG use columns 0:3 only.
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
        offset keeps the /tf callback's work bounded -- a run log grows all session,
        and re-reading it from the top on every accept would not. A trailing partial
        line stays unconsumed, so the accept it belongs to is not lost."""
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
        the premap into; False -- a no-op overlay -- whenever no viewer is up
        (``--viewer none``, headless, or a blueprint without RerunBridgeModule).

        dimos starts every module worker through multiprocessing (forkserver,
        core/coordination/python_worker.py) and rr.init derives its default recording
        id from multiprocessing's authkey, so bridge worker and eval worker share ONE
        recording and these points land beside the premap instead of in a second
        recording the viewer would list separately.
        https://rerun.io/docs/howto/logging/shared-recordings
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
        """Redraw ``world/eval/accepted`` when a NEW accept lands: every accepted fix
        so far, one point each, coloured by the prior that won it. The full set is
        re-logged (not appended) so a fix that was still ``unknown`` when its /tf
        arrived gets its colour the moment its log line shows up."""
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
                # rerun shows labels below its own instance-count threshold and on
                # hover above it -- exactly the right behaviour for a growing set.
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
        # its prior, so ACCEPTED per source is exact even where a join failure would
        # have left the fix's own source ``unknown``. n_rejects = len(reject_sources).
        n_rejects = None if reject_sources is None else len(reject_sources)
        stats = compute_stats(
            fixes, odom, census, n_rejects, mode="live", held_out_note=note,
            accept_sources=[h.source for h in health], reject_sources=reject_sources,
        )
        write_report(stats, odom, fixes, markers_xy, Path(self.config.out_dir),
                     self.config.tag, title="RelocEval live capture")
