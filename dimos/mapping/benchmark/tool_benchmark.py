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
# from bench.py's LOOP_CLOSURE_MAX_DT_S. MarkerLocalizationModule only
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
    """Ported from trial/scripts/report.py's compute_ate()."""
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

    Correction TF (`world->map`) carries no source tag -- MarkerLocalizationModule
    and RelocalizationModule can't even run in the same process (see
    trial/day1-runbook.md Drill C), so this is the only place source is
    attributable at all, via the `logger`/`event` text metrics_logger.py
    already captures.
    """
    marker = lidar = other = 0
    for r in log_events:
        blob = f"{r.logger or ''} {r.event or ''}".lower()
        if "marker" in blob:
            marker += 1
        elif "relocaliz" in blob or "lidar" in blob:
            lidar += 1
        else:
            other += 1
    return marker, lidar, other


def compute_run_metrics(
    records: Sequence[RunRecord],
    *,
    run_id: str,
    route: str,
    mode: str,
    duration_s: float,
    notes: str = "",
    log_path: str = "",
) -> RunMetrics:
    """Ported from trial/scripts/bench.py's compute_run_metrics().

    ATE-proxy = drift-vs-corrected divergence: report.py's own compute_ate,
    fed this run's raw odom stream as "primary" and its own corrected stream
    as "reference" -- an RMSE of how far dead-reckoning strays from the
    corrected estimate over the run, with no external ground truth required.
    Proxy, not real ATE (needs an outside reference for that).
    """
    odom = by_type(records, "odom_pose")
    corrected = by_type(records, "corrected_pose")
    new = by_type(records, "correction_new")
    log_events = by_type(records, "log_event")

    det = detection_stats(records)

    ate_proxy = (
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

    # Holdout-referee metrics (a marker deliberately excluded from the map,
    # scored against real ground truth) are not computed here -- neither
    # trial/scripts/bench.py nor metrics_logger.py has this logic as of this
    # port. RunMetrics.holdout_tag_id / holdout_error_m stay None.
    # EXTENSION POINT: holdout referee metrics land with the trial-scripts port.

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
        ate_proxy_rmse_m=ate_proxy.get("ate_rmse_m"),
        ate_proxy_n_matched=ate_proxy.get("n_matched", 0),
        marker_log_events=marker_events,
        lidar_log_events=lidar_events,
        notes=notes,
        log_path=log_path,
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
    )
    benchmark_results.add(metrics)
