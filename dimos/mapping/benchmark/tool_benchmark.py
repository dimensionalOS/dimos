#!/usr/bin/env python3

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

"""In-repo benchmark devtool for marker/lidar localization runs.

Ports the metric computations from the trial-week scripts
(../../../trial/scripts/report.py's ATE/matching helpers +
../../../trial/scripts/bench.py's loop-closure/correction stats), restructured
onto this repo's dimos/protocol/pubsub/benchmark/ shape: typed results in
type.py, embedded fixtures in testdata.py, this runner. Correctness against
known numbers is asserted in the colocated test_benchmark_metrics.py; this
file (like pubsub's tool_benchmark.py) prints a comparison table and is not
collected by CI -- the `tool_` prefix keeps it out of both pytest's default
`test_*.py` collection and mypy's strict-checked file set (pyproject.toml
`[tool.mypy] exclude`).

Usage (manual, not run in CI):
    uv run pytest dimos/mapping/benchmark/tool_benchmark.py -s
"""

from collections.abc import Generator, Sequence
import json
import math
from pathlib import Path
import re
import statistics
from typing import Any

import pytest

from dimos.mapping.benchmark.testdata import FixtureCase, testcases
from dimos.mapping.benchmark.type import BenchmarkResults, RunMetrics, RunRecord

# How far a corrected_pose sample is allowed to be, in time, from the run's
# start/end instant and still count as "the pose at that instant" -- ported
# from bench.py's LOOP_CLOSURE_MAX_DT_S. VisualRelocalizationModule only
# publishes on a fresh accept, not on a timer, so a correction can be many
# seconds old and still be "the current one."
LOOP_CLOSURE_MAX_DT_S = 30.0

_REJECT_RE = re.compile(r"gate rejected \((\d+) tags? seen\)")


def load_jsonl(path: Path) -> list[RunRecord]:
    """Ported from trial/scripts/report.py's load_jsonl(), typed to RunRecord."""
    records = []
    with path.open() as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                records.append(RunRecord.from_dict(json.loads(line)))
            except json.JSONDecodeError:
                continue
    return records


def by_type(records: Sequence[RunRecord], kind: str) -> list[RunRecord]:
    """Ported from trial/scripts/report.py's by_type()."""
    return [r for r in records if r.type == kind]


def reject_tag_counts(log_events: Sequence[RunRecord]) -> list[int]:
    """Ported from trial/scripts/report.py's reject_tag_counts()."""
    counts = []
    for r in log_events:
        m = _REJECT_RE.search(r.event or "")
        if m:
            counts.append(int(m.group(1)))
    return counts


def nearest_match(
    ts: float, sorted_ref: Sequence[tuple[float, float, float, float]], max_dt: float
) -> tuple[float, float, float] | None:
    """Ported from trial/scripts/report.py's nearest_match().

    sorted_ref: list of (ts, x, y, z) sorted by ts. Linear scan is fine here --
    these logs are, at most, tens of thousands of rows.
    """
    best = None
    best_dt = max_dt
    for rt, x, y, z in sorted_ref:
        dt = abs(rt - ts)
        if dt <= best_dt:
            best_dt = dt
            best = (x, y, z)
    return best


def compute_ate(
    primary: Sequence[RunRecord], reference: Sequence[RunRecord], max_dt: float = 1.0
) -> dict[str, Any]:
    """Ported from trial/scripts/report.py's compute_ate(). Generic RMSE-
    between-two-point-streams math -- kept as a math helper, but note the
    ONLY caller in this file (compute_run_metrics below) feeds it a run's own
    odom stream against its own corrected stream, i.e. self-consistency, not
    a ground-truth-referenced ATE (see RunMetrics.self_consistency_rmse_m's
    docstring for why that result is deliberately not named "ate" anywhere
    it's surfaced)."""
    if not primary:
        return {"ate_rmse_m": None, "n_matched": 0, "reason": "no samples in primary trajectory"}
    if not reference:
        return {
            "ate_rmse_m": None,
            "n_matched": 0,
            "reason": "no samples in reference trajectory",
        }
    ref_sorted = sorted((r.ts, *r.xyz) for r in reference)
    errors = []
    for r in primary:
        match = nearest_match(r.ts, ref_sorted, max_dt)
        if match is None:
            continue
        errors.append(math.dist(r.xyz, match))
    if not errors:
        return {
            "ate_rmse_m": None,
            "n_matched": 0,
            "reason": f"no primary/reference samples within {max_dt}s of each other",
        }
    rmse = math.sqrt(sum(e * e for e in errors) / len(errors))
    return {
        "ate_rmse_m": round(rmse, 4),
        "n_matched": len(errors),
        "max_error_m": round(max(errors), 4),
        "mean_error_m": round(statistics.mean(errors), 4),
    }


def detection_stats(records: Sequence[RunRecord]) -> dict[str, Any]:
    """Ported from trial/scripts/report.py's detection_stats()."""
    odom = by_type(records, "odom_pose")
    corrected = by_type(records, "corrected_pose")
    new = by_type(records, "correction_new")
    hold = by_type(records, "correction_hold")
    log_events = by_type(records, "log_event")
    rejects = reject_tag_counts(log_events)

    n_odom = len(odom)
    coverage = (len(corrected) / n_odom) if n_odom else None
    accepted = len(new)
    total_attempts = accepted + len(rejects)
    accept_rate = (accepted / total_attempts) if total_attempts else None

    return {
        "odom_ticks": n_odom,
        "corrected_ticks": len(corrected),
        "corrected_pose_coverage": round(coverage, 4) if coverage is not None else None,
        "corrections_accepted": accepted,
        "corrections_held_unchanged": len(hold),
        "corrections_rejected": len(rejects),
        "rejected_mean_tags_seen": round(statistics.mean(rejects), 2) if rejects else None,
        "acceptance_rate": round(accept_rate, 4) if accept_rate is not None else None,
        "acceptance_rate_note": (
            "accepted / (accepted + rejected); rejected count only available if "
            "metrics_logger.py was run with --dimos-log"
        ),
    }


def _loop_closure_errors(
    odom: Sequence[RunRecord], corrected: Sequence[RunRecord], max_dt: float = LOOP_CLOSURE_MAX_DT_S
) -> tuple[float | None, float | None]:
    """Ported from trial/scripts/bench.py's _loop_closure_errors().

    Start/end pose delta -- the cheapest real ground truth for a route driven
    as a loop back to a taped floor mark: if start == end physically, any
    measured delta between the pose estimate at drill-start and at drill-end
    IS the error.
    """
    if len(odom) < 2:
        return None, None
    odom_err = round(math.dist(odom[0].xyz, odom[-1].xyz), 4)

    corr_err = None
    if corrected:
        ref = sorted((r.ts, *r.xyz) for r in corrected)
        p_start = nearest_match(odom[0].ts, ref, max_dt)
        p_end = nearest_match(odom[-1].ts, ref, max_dt)
        if p_start is not None and p_end is not None:
            corr_err = round(math.dist(p_start, p_end), 4)

    return odom_err, corr_err


def _correction_magnitude_stats(
    new_records: Sequence[RunRecord],
) -> tuple[float | None, float | None]:
    """Ported from trial/scripts/bench.py's _correction_magnitude_stats()."""
    mags = [r.magnitude_m for r in new_records if r.magnitude_m is not None]
    if not mags:
        return None, None
    return round(statistics.mean(mags), 4), round(max(mags), 4)


def _source_breakdown(log_events: Sequence[RunRecord]) -> tuple[int, int, int]:
    """Ported from trial/scripts/bench.py's _source_breakdown().

    Correction TF (`world->map`) carries no source tag -- VisualRelocalizationModule
    and RelocalizationModule can't even run in the same process (see
    trial/day1-runbook.md Drill C), so this is the only place source is
    attributable at all, via the `logger`/`event` text metrics_logger.py
    already captures.

    "fiducial" (VisualRelocalizationModule's home, dimos/perception/fiducial/)
    is checked alongside "marker" -- since the module rename, its own
    setup_logger()-derived `logger` path (visual_relocalization_module.py) no
    longer contains "marker" but does contain "relocaliz", which would
    otherwise misclassify it into the lidar bucket below.
    """
    marker = lidar = other = 0
    for r in log_events:
        blob = f"{r.logger or ''} {r.event or ''}".lower()
        if "marker" in blob or "fiducial" in blob:
            marker += 1
        elif "relocaliz" in blob or "lidar" in blob:
            lidar += 1
        else:
            other += 1
    return marker, lidar, other


# -- start/end referee (ported from bench.py's holdout-tag referee) --------
#
# A tag id deliberately EXCLUDED from every localization map under test.
# metrics_logger.py's --holdout-tag runs a second, independent solvePnP for
# that one id and logs each sighting as a tag_sighting record (marker_<id> ->
# camera_optical -- the camera's pose IN the tag's own fixed frame). Because
# the tag never moves, comparing that pose at drill-start vs drill-end IS the
# true physical camera displacement, with no TF/odom chain and none of the
# drift the rest of this benchmark measures. Median of the first/last
# `window` sightings gives the measured start/end; the same median-of-a-
# window statistic applied to whichever pose stream the run actually
# produced (corrected_pose if any landed, else odom_pose) -- matched to the
# SAME sighting timestamps -- gives that mode's claim over the identical
# span. The closure error is |claim - measured|: a mode's own loop-closure
# error, checked against a real physical reference instead of an assumed
# taped start==end.

# Number of holdout-tag sightings to median at run-start and run-end for the
# referee's measured delta -- ported from bench.py's HOLDOUT_DEFAULT_WINDOW.
START_END_REFEREE_WINDOW = 10


def _quat_angle_deg(
    q1: tuple[float, float, float, float], q2: tuple[float, float, float, float]
) -> float:
    """Ported from trial/scripts/bench.py's _quat_angle_deg(). Angle (deg)
    between two orientations -- robust to the q/-q double cover (same
    rotation, opposite sign) via abs() on the dot product."""
    dot = sum(a * b for a, b in zip(q1, q2, strict=True))
    dot = max(-1.0, min(1.0, abs(dot)))
    return math.degrees(2.0 * math.acos(dot))


def _median_pose(
    records: Sequence[RunRecord],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Ported from trial/scripts/bench.py's _median_pose(). Component-wise
    median translation + a hemisphere-aligned, renormalized median
    quaternion across a small window of same-shape records (tag_sighting /
    odom_pose / corrected_pose all carry translation+rotation). Not a proper
    SLERP average -- a cheap, dependency-free, outlier-resistant central
    tendency, good enough for an N~=10 referee window, not a smoothing
    filter."""

    def _rot(r: RunRecord) -> tuple[float, float, float, float]:
        if r.rotation is None:
            raise ValueError(f"RunRecord of type={r.type!r} carries no rotation")
        return r.rotation

    xs, ys, zs = zip(*(r.xyz for r in records), strict=True)
    translation = (statistics.median(xs), statistics.median(ys), statistics.median(zs))

    ref = _rot(records[0])
    aligned = [
        q if sum(a * b for a, b in zip(q, ref, strict=True)) >= 0 else (-q[0], -q[1], -q[2], -q[3])
        for q in (_rot(r) for r in records)
    ]
    qx, qy, qz, qw = (statistics.median(c) for c in zip(*aligned, strict=True))
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    rotation = (qx / norm, qy / norm, qz / norm, qw / norm) if norm > 1e-9 else ref
    return translation, rotation


def _nearest_record(
    ts: float, sorted_records: Sequence[RunRecord], max_dt: float
) -> RunRecord | None:
    """Ported from trial/scripts/bench.py's _nearest_record() -- like
    nearest_match() above but returns the whole record (translation +
    rotation), needed for the start/end referee's median-pose orientation
    comparison, not just xyz."""
    best: RunRecord | None = None
    best_dt = max_dt
    for r in sorted_records:
        dt = abs(r.ts - ts)
        if dt <= best_dt:
            best_dt = dt
            best = r
    return best


def _start_end_referee_metrics(
    records: Sequence[RunRecord],
    odom: Sequence[RunRecord],
    corrected: Sequence[RunRecord],
    window: int = START_END_REFEREE_WINDOW,
    max_dt: float = LOOP_CLOSURE_MAX_DT_S,
) -> dict[str, Any]:
    """Ported from trial/scripts/bench.py's _holdout_referee_metrics().

    Shared-pipeline caveat (benchmark-spec.md §4): this referee's PnP runs
    the exact same detector/solvePnP code as the mode under test, on the
    same physical camera -- it is not an independent instrument. One
    tape-measure check per session certifies that shared pipeline; it does
    not certify the mode's own map-building/gating logic, which is what the
    rest of this benchmark measures.
    """
    null_result: dict[str, Any] = {
        "start_end_readings_start": 0,
        "start_end_readings_end": 0,
        "start_end_closure_error_m": None,
        "start_end_closure_error_deg": None,
    }

    sightings = sorted(by_type(records, "tag_sighting"), key=lambda r: r.ts)
    n = len(sightings)
    if n < 2:
        return {**null_result, "start_end_readings_start": n}

    eff_window = window if n >= 2 * window else max(1, n // 2)
    first, last = sightings[:eff_window], sightings[-eff_window:]
    readings = {"start_end_readings_start": len(first), "start_end_readings_end": len(last)}

    claim_source = corrected if corrected else odom
    if not claim_source:
        return {**null_result, **readings}

    claim_sorted = sorted(claim_source, key=lambda r: r.ts)
    claim_first = [
        m for s in first if (m := _nearest_record(s.ts, claim_sorted, max_dt)) is not None
    ]
    claim_last = [m for s in last if (m := _nearest_record(s.ts, claim_sorted, max_dt)) is not None]
    if not claim_first or not claim_last:
        return {**null_result, **readings}

    measured_start_t, measured_start_q = _median_pose(first)
    measured_end_t, measured_end_q = _median_pose(last)
    measured_delta_m = math.dist(measured_start_t, measured_end_t)
    measured_delta_deg = _quat_angle_deg(measured_start_q, measured_end_q)

    claimed_start_t, claimed_start_q = _median_pose(claim_first)
    claimed_end_t, claimed_end_q = _median_pose(claim_last)
    claimed_delta_m = math.dist(claimed_start_t, claimed_end_t)
    claimed_delta_deg = _quat_angle_deg(claimed_start_q, claimed_end_q)

    return {
        **readings,
        "start_end_closure_error_m": round(abs(claimed_delta_m - measured_delta_m), 4),
        "start_end_closure_error_deg": round(abs(claimed_delta_deg - measured_delta_deg), 2),
    }


# -- checkpoint series: max/RMS error across every GT-visible instant -------
#
# benchmark-metrics.md's ranked research (#3 max checkpoint error, #4 RMS
# over checkpoints) asks for both computed over every instant the start/end
# tag was visible, not just the first/last window _start_end_referee_metrics
# uses for the primary return-error number above. When the tag is only ever
# seen at drill-start and drill-end (the ordinary case), this degenerates to
# N=2 and RMS is correctly left None -- redundant with return-error/max-error
# per the research's own call.

CHECKPOINT_GAP_S = 5.0  # tag_sighting readings closer together than this are
# the same visit/checkpoint (one continuous dwell in front of the tag), not
# separate ones -- comfortably larger than one camera frame period, small
# enough that a route's single start-of-run and end-of-run dwells don't get
# merged into one.


def _cluster_sightings(
    sightings: Sequence[RunRecord], gap_s: float = CHECKPOINT_GAP_S
) -> list[list[RunRecord]]:
    """Group a tag_sighting stream into visits: consecutive readings within
    `gap_s` of each other are one visit/checkpoint."""
    if not sightings:
        return []
    ordered = sorted(sightings, key=lambda r: r.ts)
    clusters: list[list[RunRecord]] = [[ordered[0]]]
    for r in ordered[1:]:
        if r.ts - clusters[-1][-1].ts <= gap_s:
            clusters[-1].append(r)
        else:
            clusters.append([r])
    return clusters


def _checkpoint_position_errors(
    records: Sequence[RunRecord],
    odom: Sequence[RunRecord],
    corrected: Sequence[RunRecord],
    max_dt: float = LOOP_CLOSURE_MAX_DT_S,
) -> tuple[list[float], int]:
    """One magnitude-difference position error (same frame-mismatch
    reasoning as _start_end_referee_metrics -- the referee's poses live in
    the tag's own fixed frame, the claim lives in world/map frame, and only a
    distance stays honestly comparable across that uncalibrated-rotation
    mismatch) per visit to the start/end tag after the first, relative to the
    first visit as the zero-displacement anchor. Returns (errors,
    checkpoint_count) where checkpoint_count is the total number of visits
    found (including the anchor) -- callers gate RMS on checkpoint_count > 2.
    """
    sightings = by_type(records, "tag_sighting")
    clusters = _cluster_sightings(sightings)
    if len(clusters) < 2:
        return [], len(clusters)

    claim_source = corrected if corrected else odom
    if not claim_source:
        return [], len(clusters)
    claim_sorted = sorted(claim_source, key=lambda r: r.ts)

    def _claim_median(cluster: Sequence[RunRecord]) -> tuple[float, float, float] | None:
        matched = [
            m for s in cluster if (m := _nearest_record(s.ts, claim_sorted, max_dt)) is not None
        ]
        if not matched:
            return None
        t, _ = _median_pose(matched)
        return t

    anchor_measured_t, _ = _median_pose(clusters[0])
    anchor_claimed_t = _claim_median(clusters[0])
    if anchor_claimed_t is None:
        return [], len(clusters)

    errors: list[float] = []
    for cluster in clusters[1:]:
        measured_t, _ = _median_pose(cluster)
        claimed_t = _claim_median(cluster)
        if claimed_t is None:
            continue
        measured_delta = math.dist(measured_t, anchor_measured_t)
        claimed_delta = math.dist(claimed_t, anchor_claimed_t)
        errors.append(abs(claimed_delta - measured_delta))
    return errors, len(clusters)


# -- recovery time (kidnap variant) -----------------------------------------


def _recovery_time_s(
    odom: Sequence[RunRecord], new_corrections: Sequence[RunRecord], variant: str | None
) -> float | None:
    """Wall-clock delta from run-start to the first correction_new --
    automatable ONLY for a `kidnap` variant run structured as a cold-start-
    displaced boot (benchmarks.md's "cold-start it displaced" phrasing): the
    kidnap instant IS the run's first odom_pose sample in that case. A
    mid-run carry-and-drop kidnap (benchmarks.md's other phrasing) has no
    logged "carried at ts=X" marker anywhere in this schema -- this function
    can't see that instant and returns None rather than guess at it; that
    variant still needs a stopwatch mark per benchmark-spec.md #4."""
    if variant != "kidnap" or not odom or not new_corrections:
        return None
    first_correction = min(new_corrections, key=lambda r: r.ts)
    if first_correction.ts < odom[0].ts:
        return None
    return round(first_correction.ts - odom[0].ts, 3)


# -- bounded vs. unbounded drift classification (report-level) --------------
#
# benchmark-metrics.md research item #6: plot return-error vs path length
# ACROSS TRIALS -- flat reads as bounded (anchored to the marker/premap),
# growing reads as unbounded (dead-reckoning). This is inherently a
# multi-run comparison, not a single RunMetrics field -- classify_bounded_vs_unbounded
# takes (duration_s, return_error_m) pairs across every recorded run for one
# mode. duration_s stands in for path length under benchmark-spec.md #5's own
# fairness rule ("same route, same walking pace") -- fine within one route's
# repeated trials, and across benchmarks.md's routes of deliberately
# different lengths (Loop / Corridor) it's exactly the trend §6's pass bar
# describes ("odom/lidar-only shows unbounded drift by the far end").


def classify_bounded_vs_unbounded(points: Sequence[tuple[float, float]]) -> str:
    """Coarse ratio-of-extremes heuristic, not a regression -- good enough to
    flag the qualitative pattern, not a precision fit. Needs >=2 runs
    spanning a real duration spread; otherwise reports why it can't say."""
    if len(points) < 2:
        return "insufficient data (need >=2 runs)"
    durations = [d for d, _ in points]
    if max(durations) - min(durations) < 1e-6:
        return "insufficient data (all runs same duration)"
    shortest = min(points, key=lambda p: p[0])
    longest = max(points, key=lambda p: p[0])
    duration_ratio = longest[0] / shortest[0]
    if shortest[1] <= 1e-9:
        growth = longest[1] - shortest[1]
        return "unbounded (drift-like)" if growth > 0.05 else "bounded (anchored)"
    error_ratio = longest[1] / shortest[1]
    # Error growing roughly in step with duration reads as unbounded; error
    # staying far flatter than duration reads as bounded.
    return "unbounded (drift-like)" if error_ratio > 0.5 * duration_ratio else "bounded (anchored)"


def compute_run_metrics(
    records: Sequence[RunRecord],
    *,
    run_id: str,
    route: str,
    mode: str,
    duration_s: float,
    notes: str = "",
    log_path: str = "",
    start_end_tag: int | None = None,
    variant: str | None = None,
) -> RunMetrics:
    """Ported from trial/scripts/bench.py's compute_run_metrics().

    self_consistency_rmse_m = drift-vs-corrected divergence: report.py's own
    compute_ate, fed this run's raw odom stream as "primary" and its own
    corrected stream as "reference" -- an RMSE of how far dead-reckoning
    strays from the corrected estimate over the run, with no external ground
    truth required. Self-consistency, not ATE (needs an outside reference for
    that -- see the field's own docstring in type.py).

    start_end_tag: marker id deliberately excluded from every localization
    map under test (bench.py's --holdout-tag) -- omit to skip the referee, as
    before this port (RunMetrics.start_end_* fields stay None/0).

    variant: the run's route variant (e.g. "kidnap"), if any -- only used to
    gate recovery_time_s (see _recovery_time_s); omit for a clean run.
    """
    odom = by_type(records, "odom_pose")
    corrected = by_type(records, "corrected_pose")
    new = by_type(records, "correction_new")
    log_events = by_type(records, "log_event")

    det = detection_stats(records)

    self_consistency = (
        compute_ate(odom, corrected, max_dt=1.0)
        if corrected
        else {
            "ate_rmse_m": None,
            "n_matched": 0,
            "reason": "no corrected_pose samples -- zero corrections landed this run",
        }
    )

    mag_mean, mag_max = _correction_magnitude_stats(new)
    marker_events, lidar_events, _other = _source_breakdown(log_events)
    loop_odom_m, loop_corrected_m = _loop_closure_errors(odom, corrected)

    start_end = (
        _start_end_referee_metrics(records, odom, corrected)
        if start_end_tag is not None
        else {
            "start_end_readings_start": 0,
            "start_end_readings_end": 0,
            "start_end_closure_error_m": None,
            "start_end_closure_error_deg": None,
        }
    )

    checkpoint_errors, checkpoint_count = (
        _checkpoint_position_errors(records, odom, corrected)
        if start_end_tag is not None
        else ([], 0)
    )
    max_checkpoint_error_m = max(checkpoint_errors) if checkpoint_errors else None
    checkpoint_error_rms_m = (
        round(math.sqrt(sum(e * e for e in checkpoint_errors) / len(checkpoint_errors)), 4)
        if checkpoint_errors and checkpoint_count > 2
        else None
    )

    recovery_time_s = _recovery_time_s(odom, new, variant)

    return RunMetrics(
        run_id=run_id,
        route=route,
        mode=mode,
        duration_s=round(duration_s, 1),
        odom_ticks=det["odom_ticks"],
        corrected_ticks=det["corrected_ticks"],
        corrected_pose_coverage=det["corrected_pose_coverage"],
        corrections_accepted=det["corrections_accepted"],
        corrections_held=det["corrections_held_unchanged"],
        corrections_rejected=det["corrections_rejected"],
        correction_mag_mean_m=mag_mean,
        correction_mag_max_m=mag_max,
        loop_closure_error_odom_m=loop_odom_m,
        loop_closure_error_corrected_m=loop_corrected_m,
        self_consistency_rmse_m=self_consistency.get("ate_rmse_m"),
        self_consistency_n_matched=self_consistency.get("n_matched", 0),
        marker_log_events=marker_events,
        lidar_log_events=lidar_events,
        notes=notes,
        log_path=log_path,
        start_end_tag_id=start_end_tag,
        start_end_closure_error_m=start_end["start_end_closure_error_m"],
        start_end_closure_error_deg=start_end["start_end_closure_error_deg"],
        start_end_readings_start=start_end["start_end_readings_start"],
        start_end_readings_end=start_end["start_end_readings_end"],
        max_checkpoint_error_m=round(max_checkpoint_error_m, 4)
        if max_checkpoint_error_m is not None
        else None,
        checkpoint_error_rms_m=checkpoint_error_rms_m,
        checkpoint_count=checkpoint_count,
        recovery_time_s=recovery_time_s,
    )


@pytest.fixture(scope="module")
def benchmark_results() -> Generator[BenchmarkResults, None, None]:
    """Module-scoped fixture to collect benchmark results."""
    results = BenchmarkResults()
    yield results
    results.print_summary()


@pytest.mark.parametrize("case", testcases, ids=[c.name for c in testcases])
def test_run_metrics(case: FixtureCase, benchmark_results: BenchmarkResults) -> None:
    """Computes RunMetrics for one fixture run and adds it to the summary table.

    Correctness of the ported computations against exact known numbers is
    asserted separately in test_benchmark_metrics.py -- this test's job is the
    human-facing comparison table (mirrors pubsub's tool_benchmark.py
    test_throughput(), which measures and prints rather than asserts).
    """
    metrics = compute_run_metrics(
        case.records,
        run_id=case.name,
        route=case.route,
        mode=case.mode,
        duration_s=case.duration_s,
        start_end_tag=case.start_end_tag,
    )
    benchmark_results.add(metrics)
