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

"""Run case suites through the ray tracer and MLS planner and score them.

Auto cases plan twice: online on the incremental map at the case start time,
and final on the map fed the whole recording. Manual and infeasible cases plan
once on the final map. The final path is gated against full final occupancy,
the online path against the incremental map at plan time. Every path must stand
on final-map occupancy and stay within the climb envelope. The headline score
is validity-gated SPL on the incremental map.
"""

from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from dataclasses import asdict, dataclass, field, replace
import itertools
import threading
from time import perf_counter
from typing import TYPE_CHECKING

import numpy as np

from dimos.navigation.nav_3d.evaluator import metrics
from dimos.navigation.nav_3d.evaluator.config import EvalConfig
from dimos.navigation.nav_3d.evaluator.final_map import (
    load_or_build_checkpoints,
    load_or_build_final_map,
)
from dimos.navigation.nav_3d.evaluator.recording import load_trajectory
from dimos.navigation.nav_3d.evaluator.voxel_keys import key_centers
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterator

    from numpy.typing import NDArray

    from dimos.navigation.nav_3d.evaluator.cases import Case, Suite
    from dimos.navigation.nav_3d.mls_planner.mls_planner import MLSPlanner

logger = setup_logger()

MAX_COLLISIONS_KEPT = 50


@dataclass
class PlanOutcome:
    planned: bool
    reached: bool
    valid: bool
    # Every sample stands on final-map occupancy. Fabricated bridges fail.
    supported: bool
    # For an ordinary case: all of the above. For an expect_fail case: the
    # planner correctly refused the infeasible goal.
    success: bool
    length: float
    plan_ms: float
    spl: float
    # Gate margin along the path (see GateResult.min_clearance_m). None when
    # no path was planned.
    min_clearance: float | None
    waypoints: list[list[float]]
    # Indices of the colliding samples along the densified path, so a viewer
    # can redraw the exact body boxes the gate rejected.
    collision_indices: list[int]
    unsupported: list[list[float]]
    steep: list[list[float]]


@dataclass
class PlannerArtifacts:
    """Graph state of one planner after its map update. Not serialized to JSON."""

    surface_clearance: NDArray[np.float32]
    edges: NDArray[np.float32]


@dataclass
class CaseResult:
    id: str
    dataset: str
    start: tuple[float, float, float]
    goal: tuple[float, float, float]
    tags: list[str]
    l_ref: float
    online_voxels: int
    map_update_ms: float
    expect_fail: bool
    online: PlanOutcome
    final: PlanOutcome
    soft_progress: float
    # Scored on the final map only, so there is no distinct online score.
    final_only: bool = False
    # The online plan succeeded but a new obstacle in the final map blocks its
    # route, so the case looks like a dynamic obstacle rather than a bug.
    dynamic_candidate: bool = False
    # Where the online route is blocked by that newly-appeared occupancy.
    blocking_points: list[list[float]] = field(default_factory=list)
    # Planner graph on the incremental map, kept for the rerun recording.
    online_artifacts: PlannerArtifacts | None = None
    # Occupied voxel centers of the incremental map at plan time, kept for the
    # rerun recording.
    online_occupied: NDArray[np.float32] | None = None


@dataclass
class DatasetResult:
    dataset: str
    cases: list[CaseResult]
    final_voxels: int
    map_build_ms: float
    add_frame_ms: dict[str, float]
    frames: int
    final_artifacts: PlannerArtifacts | None = None


@dataclass
class TagStats:
    """Aggregate scores over every case carrying a given tag."""

    n: int
    # Cases with an online phase (excludes final-only manual/infeasible cases).
    n_online: int
    inc_score: float
    fin_score: float


@dataclass
class Report:
    score: float
    score_soft: float
    final_score: float
    n_cases: int
    # Cases with an online phase. The incremental score is over these only.
    n_online: int
    n_success: int
    n_success_final: int
    # The incremental and final runs are independent tests per case. These
    # count the four pass/fail combinations.
    outcome_counts: dict[str, int]
    # Score sliced by case tag (stairs, flat, up, down, ...), so a config's
    # effect on each terrain class is visible next to the aggregate.
    by_tag: dict[str, TagStats]
    plan_ms: dict[str, float]
    map_update_ms: dict[str, float]
    datasets: list[DatasetResult]
    # dataset/id of cases whose online route a new final obstacle blocks, the
    # candidates for an expect_final_fail label.
    dynamic_candidates: list[str] = field(default_factory=list)
    config: dict[str, object] = field(default_factory=dict)

    def to_dict(self) -> dict[str, object]:
        out = asdict(self)
        for dataset in out["datasets"]:
            dataset.pop("final_artifacts")
            for case in dataset["cases"]:
                case.pop("online_artifacts")
                case.pop("online_occupied")
        return out


def _run_plan(
    planner: MLSPlanner,
    case: Case,
    l_ref: float,
    obstacle_keys: NDArray[np.int64],
    support_keys: NDArray[np.int64],
    cfg: EvalConfig,
) -> tuple[PlanOutcome, NDArray[np.float32] | None]:
    t0 = perf_counter()
    waypoints = planner.plan(case.start, case.goal)
    plan_ms = (perf_counter() - t0) * 1000
    if waypoints is None or len(waypoints) == 0:
        return _no_plan(plan_ms), None

    reached = metrics.goal_reached(waypoints, case.goal, cfg.goal_tolerance)
    gate = metrics.check_path(waypoints, obstacle_keys, cfg)
    support = metrics.check_support(waypoints, support_keys, cfg)
    kinematics = metrics.check_kinematics(waypoints, cfg)
    length = metrics.path_length(waypoints)
    success = reached and gate.valid and support.valid and kinematics.valid
    outcome = PlanOutcome(
        planned=True,
        reached=reached,
        valid=gate.valid,
        supported=support.valid,
        success=success,
        length=length,
        plan_ms=plan_ms,
        spl=metrics.spl(success, l_ref, length),
        min_clearance=gate.min_clearance_m,
        waypoints=waypoints.tolist(),
        collision_indices=gate.collision_indices[:MAX_COLLISIONS_KEPT].tolist(),
        unsupported=support.unsupported_points[:MAX_COLLISIONS_KEPT].tolist(),
        steep=kinematics.violation_points[:MAX_COLLISIONS_KEPT].tolist(),
    )
    return outcome, waypoints


def _no_plan(plan_ms: float) -> PlanOutcome:
    return PlanOutcome(
        planned=False,
        reached=False,
        valid=False,
        supported=True,
        success=False,
        length=0.0,
        plan_ms=plan_ms,
        spl=0.0,
        min_clearance=None,
        waypoints=[],
        collision_indices=[],
        unsupported=[],
        steep=[],
    )


def score_negative(raw: PlanOutcome) -> PlanOutcome:
    """Invert an outcome for a human-certified infeasible case.

    The planner succeeds by refusing. Any goal-reaching path it returns is a
    false positive scored zero, whether or not the gates would have caught
    it, because the planner claimed a route that does not exist.
    """
    refused = not (raw.planned and raw.reached)
    return replace(raw, success=refused, spl=1.0 if refused else 0.0)


def _dynamic_candidate(
    online: PlanOutcome,
    final: PlanOutcome,
    online_wp: NDArray[np.float32] | None,
    online_keys: NDArray[np.int64],
    final_keys: NDArray[np.int64],
    cfg: EvalConfig,
) -> tuple[bool, list[list[float]]]:
    """Flag a case whose online route is blocked only by new final occupancy.

    An online success with a final failure is either a dynamic obstacle that
    appeared after the robot passed or a planner or mapping bug. Gating the
    online path against the voxels gained since plan time tells them apart. A
    human confirms before labeling the case.
    """
    if online_wp is None or not online.success or final.success:
        return False, []
    # Both come from np.unique, so the sort in setdiff1d is pure waste.
    new_keys = np.setdiff1d(final_keys, online_keys, assume_unique=True)
    if not len(new_keys):
        return False, []
    gate = metrics.check_path(online_wp, new_keys, cfg)
    if gate.valid:
        return False, []
    return True, gate.collision_points[:MAX_COLLISIONS_KEPT].tolist()


def _snapshot(planner: MLSPlanner) -> PlannerArtifacts:
    return PlannerArtifacts(
        surface_clearance=planner.surface_clearance_map(),
        edges=planner.node_edges(),
    )


def _final_only(case: Case) -> bool:
    """Whether a case is scored on the final map only, with no online phase.

    Manual and certified-infeasible cases have hand-placed endpoints that are
    not tied to the recording timeline, so there is no meaningful incremental
    map at plan time to replay against. They are pure final-map tests.
    """
    return case.expect_fail or "manual" in case.tags


def run_suite(
    suite: Suite, cfg: EvalConfig, threads: int = 1, keep_artifacts: bool = False
) -> DatasetResult:
    db_path = suite.db_path()
    trajectory = load_trajectory(db_path, suite.odom_stream, suite.end_ts_seconds())
    final = load_or_build_final_map(db_path, suite, cfg)
    obstacle_keys = final.occupied_keys

    final_only = np.array([_final_only(c) for c in suite.cases], dtype=bool)
    refs: list[metrics.Reference] = []
    for i, case in enumerate(suite.cases):
        if case.expect_fail:
            # Infeasible by certification: no demonstrated route, no plan time.
            miss = float(np.linalg.norm(np.asarray(case.goal) - np.asarray(case.start)))
            refs.append(metrics.Reference(miss, False, float("inf"), False))
            continue
        ref = metrics.reference_length(trajectory, case.start, case.goal, cfg)
        if not final_only[i]:
            # Only online-replayed cases need a causal snap onto the trajectory.
            if not ref.snapped:
                logger.warning(
                    "%s/%s: start or goal is off the walked trajectory; "
                    "using straight-line reference and the full map",
                    suite.dataset,
                    case.id,
                )
            elif not ref.causal:
                logger.warning(
                    "%s/%s: goal is never visited before the start; planning on the full map",
                    suite.dataset,
                    case.id,
                )
        refs.append(ref)

    # Final-only cases never replay online, so they take no checkpoint and their
    # plan time drops out of the schedule.
    start_ts = np.array(
        [float("inf") if final_only[i] else r.start_ts for i, r in enumerate(refs)],
        dtype=np.float64,
    )
    checkpoints = load_or_build_checkpoints(db_path, suite, cfg, start_ts)
    case_ckpt = np.searchsorted(checkpoints.times, start_ts)
    case_ckpt[final_only] = -1

    final_planner = cfg.make_planner()
    final_planner.update_global_map(final.occupied)

    results: list[CaseResult | None] = [None] * len(suite.cases)

    def _result(case: Case, ref: metrics.Reference, **rest: object) -> CaseResult:
        """Fill the fields every case copies straight from its case and reference."""
        return CaseResult(
            id=case.id,
            dataset=suite.dataset,
            start=case.start,
            goal=case.goal,
            tags=case.tags,
            l_ref=ref.length,
            **rest,  # type: ignore[arg-type]
        )

    for ci in np.flatnonzero(final_only):
        case, ref = suite.cases[ci], refs[ci]
        outcome, _ = _run_plan(final_planner, case, ref.length, obstacle_keys, obstacle_keys, cfg)
        if case.expect_fail:
            # An infeasible case is passed by refusing it.
            outcome = score_negative(outcome)
        results[ci] = _result(
            case,
            ref,
            online_voxels=len(final.occupied),
            map_update_ms=0.0,
            expect_fail=case.expect_fail,
            online=outcome,
            final=outcome,
            soft_progress=outcome.spl,
            final_only=True,
        )

    def process_checkpoint(
        k: int,
        keys: NDArray[np.int64],
        online_planner: MLSPlanner,
    ) -> None:
        online_points = key_centers(keys, cfg.voxel_size)
        t0 = perf_counter()
        if len(online_points):
            online_planner.update_global_map(online_points)
        map_update_ms = (perf_counter() - t0) * 1000
        for ci in np.flatnonzero(case_ckpt == k):
            case, ref = suite.cases[ci], refs[ci]
            final_out, _ = _run_plan(
                final_planner, case, ref.length, obstacle_keys, obstacle_keys, cfg
            )
            if case.expect_final_fail:
                # A dynamic obstacle blocked the route by the final map, so the
                # planner is right to refuse it there while the online plan,
                # made before the closure, is scored normally.
                final_out = score_negative(final_out)
            if len(online_points):
                # Collisions are checked against the incremental map the planner
                # actually had at plan time (keys), not the final map. Support
                # still uses the final map, since the ground exists whether or
                # not it was mapped yet.
                online_out, online_wp = _run_plan(
                    online_planner, case, ref.length, keys, obstacle_keys, cfg
                )
            else:
                online_out = _no_plan(0.0)
                online_wp = None
            end = online_wp[-1] if online_wp is not None and len(online_wp) else None
            dynamic_candidate, blocking = (
                (False, [])
                if case.expect_final_fail
                else _dynamic_candidate(online_out, final_out, online_wp, keys, obstacle_keys, cfg)
            )
            results[ci] = _result(
                case,
                ref,
                online_voxels=len(keys),
                map_update_ms=map_update_ms,
                expect_fail=False,
                online=online_out,
                final=final_out,
                soft_progress=metrics.soft_progress(end, case.start, case.goal),
                dynamic_candidate=dynamic_candidate,
                blocking_points=blocking,
                online_artifacts=_snapshot(online_planner)
                if keep_artifacts and len(online_points)
                else None,
                online_occupied=online_points if keep_artifacts and len(online_points) else None,
            )

    active = {int(k) for k in case_ckpt}
    tls = threading.local()

    def task(k: int, keys: NDArray[np.int64]) -> None:
        planner = getattr(tls, "planner", None)
        if planner is None:
            planner = tls.planner = cfg.make_planner()
        try:
            process_checkpoint(k, keys, planner)
        finally:
            in_flight.release()

    def snapshot_stream() -> Iterator[tuple[int, NDArray[np.int64]]]:
        """Walk the delta chain once, yielding the incremental occupancy at each
        case's plan time. Only voxels mapped by then are present, so obstacles
        the sensor never saw are naturally excluded from the online check."""
        for k, keys in enumerate(checkpoints.iter_snapshots()):
            if k in active:
                yield k, keys

    # The semaphore caps how many reconstructed snapshots are held in memory.
    in_flight = threading.BoundedSemaphore(max(1, threads) * 2)
    with ThreadPoolExecutor(max_workers=max(1, threads)) as pool:
        futures = []
        for item in snapshot_stream():
            in_flight.acquire()
            futures.append(pool.submit(task, *item))
        for future in futures:
            future.result()

    done = [r for r in results if r is not None]
    if len(done) != len(suite.cases):
        raise RuntimeError(f"{suite.dataset}: {len(suite.cases) - len(done)} cases not planned")
    return DatasetResult(
        dataset=suite.dataset,
        cases=done,
        final_voxels=len(final.occupied),
        map_build_ms=final.build_ms,
        add_frame_ms=final.add_frame_ms,
        frames=final.frames,
        final_artifacts=_snapshot(final_planner) if keep_artifacts else None,
    )


def evaluate(
    suites: list[Suite],
    cfg: EvalConfig | None = None,
    workers: int = 1,
    keep_artifacts: bool = False,
) -> Report:
    """Score every suite. workers is total parallelism: datasets spread over
    processes and each dataset's checkpoints over threads. keep_artifacts
    snapshots each planner graph for the rerun recording."""
    cfg = cfg or EvalConfig()
    if workers > 1 and len(suites) > 1:
        threads = max(1, workers // len(suites))
        with ProcessPoolExecutor(max_workers=min(workers, len(suites))) as pool:
            datasets = list(
                pool.map(
                    run_suite,
                    suites,
                    itertools.repeat(cfg),
                    itertools.repeat(threads),
                    itertools.repeat(keep_artifacts),
                )
            )
    else:
        datasets = [
            run_suite(suite, cfg, threads=workers, keep_artifacts=keep_artifacts)
            for suite in suites
        ]
    cases = [c for d in datasets for c in d.cases]
    if not cases:
        raise ValueError("no cases to evaluate")

    # Manual and infeasible cases have no online phase, so every incremental
    # aggregate is over the online cases only. Final aggregates cover them all.
    online = [c for c in cases if not c.final_only]

    def mean(values: list[float]) -> float:
        return float(np.mean(values)) if values else 0.0

    outcome_names = {
        (True, True): "both",
        (False, True): "final_only",
        (True, False): "incremental_only",
        (False, False): "neither",
    }
    outcome_counts = dict.fromkeys(outcome_names.values(), 0)
    for c in online:
        outcome_counts[outcome_names[c.online.success, c.final.success]] += 1

    by_tag: dict[str, TagStats] = {}
    for tag in sorted({t for c in cases for t in c.tags}):
        tc = [c for c in cases if tag in c.tags]
        oc = [c for c in tc if not c.final_only]
        by_tag[tag] = TagStats(
            n=len(tc),
            n_online=len(oc),
            inc_score=mean([c.online.spl for c in oc]),
            fin_score=mean([c.final.spl for c in tc]),
        )

    return Report(
        score=mean([c.online.spl for c in online]),
        score_soft=mean(
            [c.soft_progress if not c.online.success else c.online.spl for c in online]
        ),
        final_score=mean([c.final.spl for c in cases]),
        n_cases=len(cases),
        n_online=len(online),
        n_success=sum(c.online.success for c in online),
        n_success_final=sum(c.final.success for c in cases),
        outcome_counts=outcome_counts,
        by_tag=by_tag,
        plan_ms=metrics.timing_stats([c.online.plan_ms for c in online]),
        map_update_ms=metrics.timing_stats([c.map_update_ms for c in online]),
        datasets=datasets,
        dynamic_candidates=[f"{c.dataset}/{c.id}" for c in cases if c.dynamic_candidate],
        config=asdict(cfg),
    )
