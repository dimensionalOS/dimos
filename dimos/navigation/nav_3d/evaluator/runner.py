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

"""Replay case suites through a pipeline and score them, online then final."""

from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, dataclass, field, replace
import multiprocessing
from time import perf_counter
from typing import TYPE_CHECKING

import numpy as np

from dimos.navigation.nav_3d.evaluator import metrics
from dimos.navigation.nav_3d.evaluator.config import EvalConfig
from dimos.navigation.nav_3d.evaluator.pipeline import PipelineIntrospection, make_pipeline
from dimos.navigation.nav_3d.evaluator.progress import frame_progress, stage_progress
from dimos.navigation.nav_3d.evaluator.voxel_keys import voxel_keys
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Callable

    from numpy.typing import NDArray

    from dimos.navigation.nav_3d.evaluator.cases import Case, Suite
    from dimos.navigation.nav_3d.evaluator.pipeline import NavPipeline
    from dimos.navigation.nav_3d.evaluator.progress import ProgressFactory, Tick
    from dimos.navigation.nav_3d.evaluator.recording import Trajectory

logger = setup_logger()

# Cap on each list of violation points carried into the report.
MAX_POINTS_KEPT = 50


@dataclass
class PlanOutcome:
    planned: bool
    reached: bool
    # Cleared the body box against the map the pipeline itself built.
    valid: bool
    # Stood on mapped ground the whole way. Only judged on the final map, since
    # a partial map has holes that are not voids.
    supported: bool
    # Reached the goal, cleared both gates, and climbed no steeper than the
    # robot can, or for an infeasible case, that the planner refused.
    success: bool
    length: float
    # Search only. The map rebuild it would otherwise hide is map_sync_ms.
    plan_ms: float
    map_sync_ms: float
    spl: float
    # Body-surface distance to the nearest occupied voxel, None without a plan.
    min_clearance: float | None
    waypoints: list[list[float]]
    # Where the path crosses occupancy, floats over nothing, and climbs past
    # the robot's limit.
    collisions: list[metrics.BodyBox]
    unsupported: list[list[float]]
    steep: list[list[float]]


@dataclass
class PlannerArtifacts:
    """Map and graph state a pipeline chose to expose. Not serialized to JSON."""

    surface_clearance: NDArray[np.float32]
    edges: NDArray[np.float32]
    occupied: NDArray[np.float32]


@dataclass
class CaseResult:
    id: str
    dataset: str
    start: tuple[float, float, float]
    goal: tuple[float, float, float]
    tags: list[str]
    l_ref: float
    expect_fail: bool
    online: PlanOutcome
    final: PlanOutcome
    # Scored after the whole replay, so there is no distinct online score.
    final_only: bool = False
    # Pipeline state at plan time, kept for the rerun recording.
    online_artifacts: PlannerArtifacts | None = None


@dataclass
class DatasetResult:
    dataset: str
    cases: list[CaseResult]
    # Per-frame cost of feeding the pipeline, which is what map_update_ms
    # aggregates.
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
    # Rebuilding the planner's map from ingested frames, paid by the first case
    # to plan after any ingest. Zero for the rest, which reuse that rebuild.
    map_sync_ms: dict[str, float]
    map_update_ms: dict[str, float]
    datasets: list[DatasetResult]
    config: dict[str, object] = field(default_factory=dict)

    def to_dict(self) -> dict[str, object]:
        out = asdict(self)
        for dataset in out["datasets"]:
            dataset.pop("final_artifacts")
            for case in dataset["cases"]:
                case.pop("online_artifacts")
        return out


@dataclass(frozen=True)
class Occupancy:
    """The pipeline's own map, packed for the gates to probe."""

    keys: NDArray[np.int64]
    # Whether the pipeline has been fed the whole recording. A partial map is
    # missing floor it simply has not driven past yet, so absent ground is not
    # evidence of a void and the support check is held back until the end.
    whole_recording: bool


def _occupancy(pipeline: NavPipeline, cfg: EvalConfig, whole_recording: bool) -> Occupancy | None:
    """None from a pipeline that does not expose its map."""
    if not isinstance(pipeline, PipelineIntrospection):
        return None
    return Occupancy(np.unique(voxel_keys(pipeline.occupied(), cfg.voxel_size)), whole_recording)


def _run_plan(
    pipeline: NavPipeline,
    case: Case,
    l_ref: float,
    cfg: EvalConfig,
    occupancy: Occupancy | None = None,
) -> PlanOutcome:
    t0 = perf_counter()
    pipeline.sync_map()
    t1 = perf_counter()
    waypoints = pipeline.plan(case.start, case.goal)
    plan_ms = (perf_counter() - t1) * 1000
    sync_ms = (t1 - t0) * 1000
    if waypoints is None or len(waypoints) == 0:
        return _no_plan(plan_ms, sync_ms)

    reached = metrics.goal_reached(waypoints, case.goal, cfg.goal_tolerance)
    gate = metrics.check_path(waypoints, occupancy.keys, cfg) if occupancy else None
    support = (
        metrics.check_support(waypoints, occupancy.keys, cfg)
        if occupancy and occupancy.whole_recording
        else None
    )
    kinematics = metrics.check_kinematics(waypoints, cfg)
    length = metrics.path_length(waypoints)
    valid = gate.valid if gate else True
    supported = support.valid if support else True
    success = reached and valid and supported and kinematics.valid
    return PlanOutcome(
        planned=True,
        reached=reached,
        valid=valid,
        supported=supported,
        success=success,
        length=length,
        plan_ms=plan_ms,
        map_sync_ms=sync_ms,
        spl=metrics.spl(success, l_ref, length),
        min_clearance=gate.min_clearance_m if gate else None,
        waypoints=waypoints.tolist(),
        collisions=gate.collision_boxes[:MAX_POINTS_KEPT] if gate else [],
        unsupported=support.unsupported_points[:MAX_POINTS_KEPT].tolist() if support else [],
        steep=kinematics.violation_points[:MAX_POINTS_KEPT].tolist(),
    )


def _no_plan(plan_ms: float, sync_ms: float) -> PlanOutcome:
    return PlanOutcome(
        planned=False,
        reached=False,
        valid=False,
        supported=True,
        success=False,
        length=0.0,
        plan_ms=plan_ms,
        map_sync_ms=sync_ms,
        spl=0.0,
        min_clearance=None,
        waypoints=[],
        collisions=[],
        unsupported=[],
        steep=[],
    )


def score_negative(raw: PlanOutcome) -> PlanOutcome:
    """Invert an outcome for a certified-infeasible case, which passes by refusing."""
    refused = not (raw.planned and raw.reached)
    return replace(raw, success=refused, spl=1.0 if refused else 0.0)


def _snapshot(pipeline: NavPipeline) -> PlannerArtifacts | None:
    """Graph layers for the rerun recording, or None when a pipeline hides them."""
    if not isinstance(pipeline, PipelineIntrospection):
        return None
    return PlannerArtifacts(
        surface_clearance=pipeline.surface_clearance_map(),
        edges=pipeline.node_edges(),
        occupied=pipeline.occupied(),
    )


def _final_only(case: Case) -> bool:
    """Whether a case skips the online phase, having no place in the timeline."""
    return case.expect_fail or "auto" not in case.tags


def _references(
    suite: Suite,
    trajectory: Trajectory,
    final_only: NDArray[np.bool_],
    cfg: EvalConfig,
    progress: ProgressFactory | None = None,
) -> list[metrics.Reference]:
    """The demonstrated route length each case is scored against."""
    refs: list[metrics.Reference] = []
    label = f"{suite.dataset} references"
    with stage_progress(progress, len(suite.cases), label) as tick:
        for i, case in enumerate(suite.cases):
            tick()
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
                        "endpoint off the walked trajectory, using a straight-line reference",
                        dataset=suite.dataset,
                        case=case.id,
                    )
                elif not ref.causal:
                    logger.warning(
                        "goal never visited before the start, planning on the full map",
                        dataset=suite.dataset,
                        case=case.id,
                    )
            refs.append(ref)
    return refs


@dataclass
class _OnlineCase:
    """One case's plan, made partway through the replay."""

    outcome: PlanOutcome
    artifacts: PlannerArtifacts | None = None


@dataclass
class _OnlinePass:
    """Everything the single replay pass produced, keyed by case index."""

    cases: dict[int, _OnlineCase]
    add_ms: list[float]
    final_artifacts: PlannerArtifacts | None


def _replay_online(
    pipeline: NavPipeline,
    suite: Suite,
    cfg: EvalConfig,
    start_ts: NDArray[np.float64],
    refs: list[metrics.Reference],
    keep_artifacts: bool,
    progress: ProgressFactory | None,
) -> _OnlinePass:
    """Replay once, planning each case as the frame clock passes its start time."""
    out = _OnlinePass({}, [], None)
    # Cases in the order the replay reaches them. Final-only cases carry inf.
    schedule = [ci for ci in np.argsort(start_ts, kind="stable") if np.isfinite(start_ts[ci])]

    def plan_at(ci: int, tick: Tick) -> None:
        tick()
        # Gated against the map as it stands right now, which is what the
        # planner was working from.
        occupancy = _occupancy(pipeline, cfg, whole_recording=False)
        out.cases[ci] = _OnlineCase(
            _run_plan(pipeline, suite.cases[ci], refs[ci].length, cfg, occupancy),
            artifacts=_snapshot(pipeline) if keep_artifacts else None,
        )

    k = 0
    with (
        frame_progress(progress, suite, "replay") as frame_tick,
        stage_progress(progress, len(schedule), f"{suite.dataset} online plans") as plan_tick,
    ):
        for frame in suite.world_frames(cfg.align_tol):
            frame_tick()
            while k < len(schedule) and frame.ts > start_ts[schedule[k]]:
                plan_at(schedule[k], plan_tick)
                k += 1
            t0 = perf_counter()
            pipeline.add_frame(frame.points, frame.origin, frame.ts)
            out.add_ms.append((perf_counter() - t0) * 1000)
        # Anything whose start time falls past the last frame plans on it all.
        while k < len(schedule):
            plan_at(schedule[k], plan_tick)
            k += 1

    # The stream is exhausted, so the pipeline now holds the whole recording.
    out.final_artifacts = _snapshot(pipeline) if keep_artifacts else None
    return out


def _score_final(
    pipeline: NavPipeline,
    suite: Suite,
    cfg: EvalConfig,
    refs: list[metrics.Reference],
    final_only: NDArray[np.bool_],
    online: _OnlinePass,
    progress: ProgressFactory | None = None,
) -> list[CaseResult]:
    """Plan every case on the whole recording, and combine both phases."""
    results: list[CaseResult] = []
    label = f"{suite.dataset} final plans"
    # The map is settled now, so it is packed once for every case to probe.
    occupancy = _occupancy(pipeline, cfg, whole_recording=True)
    with stage_progress(progress, len(suite.cases), label) as tick:
        for ci, case in enumerate(suite.cases):
            ref = refs[ci]
            tick()
            final_out = _run_plan(pipeline, case, ref.length, cfg, occupancy)
            if case.expect_fail or case.expect_final_fail:
                # Both labels certify there is no route, so the planner passes
                # by refusing. Before the split, so a curated case scores.
                final_out = score_negative(final_out)
            online_out = final_out if final_only[ci] else online.cases[ci].outcome
            results.append(
                CaseResult(
                    id=case.id,
                    dataset=suite.dataset,
                    start=case.start,
                    goal=case.goal,
                    tags=case.tags,
                    l_ref=ref.length,
                    expect_fail=case.expect_fail and final_only[ci],
                    online=online_out,
                    final=final_out,
                    final_only=bool(final_only[ci]),
                    online_artifacts=None if final_only[ci] else online.cases[ci].artifacts,
                )
            )
    return results


def run_suite(
    suite: Suite,
    cfg: EvalConfig,
    keep_artifacts: bool = False,
    progress: ProgressFactory | None = None,
) -> DatasetResult:
    db_path = suite.db_path()
    if not db_path.exists():
        # sqlite would otherwise create an empty db here and the failure would
        # surface as a missing odometry stream.
        raise FileNotFoundError(f"{suite.dataset}: recording not found at {db_path}")
    trajectory = suite.trajectory()

    final_only = np.array([_final_only(c) for c in suite.cases], dtype=bool)
    refs = _references(suite, trajectory, final_only, cfg, progress)
    # Final-only cases have no place in the recording timeline, so they never
    # plan online and drop out of the schedule.
    start_ts = np.array(
        [float("inf") if final_only[i] else r.start_ts for i, r in enumerate(refs)],
        dtype=np.float64,
    )

    pipeline = make_pipeline(cfg.pipeline, cfg)
    online = _replay_online(pipeline, suite, cfg, start_ts, refs, keep_artifacts, progress)
    return DatasetResult(
        dataset=suite.dataset,
        cases=_score_final(pipeline, suite, cfg, refs, final_only, online, progress),
        add_frame_ms=metrics.timing_stats(online.add_ms),
        frames=len(online.add_ms),
        final_artifacts=online.final_artifacts,
    )


def _run_datasets(
    suites: list[Suite],
    cfg: EvalConfig,
    workers: int,
    keep_artifacts: bool,
    progress: ProgressFactory | None,
    on_dataset: Callable[[str], None] | None,
) -> list[DatasetResult]:
    """Replay every suite, in parallel processes when there is more than one."""
    if workers > 1 and len(suites) > 1:
        # Spawn, not fork: the mapper and planner carry a native thread pool, and
        # a forked child inherits its mutexes locked if the parent ever built one.
        with ProcessPoolExecutor(
            max_workers=min(workers, len(suites)),
            mp_context=multiprocessing.get_context("spawn"),
        ) as pool:
            pending = {
                pool.submit(run_suite, suite, cfg, keep_artifacts, progress): i
                for i, suite in enumerate(suites)
            }
            # Reported as they land, collected back into manifest order.
            landed: dict[int, DatasetResult] = {}
            for future in as_completed(pending):
                result = future.result()
                landed[pending[future]] = result
                if on_dataset is not None:
                    on_dataset(result.dataset)
            return [landed[i] for i in range(len(suites))]

    datasets = []
    for suite in suites:
        datasets.append(run_suite(suite, cfg, keep_artifacts=keep_artifacts, progress=progress))
        if on_dataset is not None:
            on_dataset(suite.dataset)
    return datasets


def _build_report(datasets: list[DatasetResult], cfg: EvalConfig) -> Report:
    """Aggregate replayed datasets into the headline scores and tag slices."""
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
        tagged = [c for c in cases if tag in c.tags]
        online_tagged = [c for c in tagged if not c.final_only]
        by_tag[tag] = TagStats(
            n=len(tagged),
            n_online=len(online_tagged),
            inc_score=mean([c.online.spl for c in online_tagged]),
            fin_score=mean([c.final.spl for c in tagged]),
        )

    return Report(
        score=mean([c.online.spl for c in online]),
        final_score=mean([c.final.spl for c in cases]),
        n_cases=len(cases),
        n_online=len(online),
        n_success=sum(c.online.success for c in online),
        n_success_final=sum(c.final.success for c in cases),
        outcome_counts=outcome_counts,
        by_tag=by_tag,
        plan_ms=metrics.timing_stats([c.online.plan_ms for c in online]),
        map_sync_ms=metrics.timing_stats(
            [c.online.map_sync_ms for c in online if c.online.map_sync_ms]
        ),
        # Worst dataset's per-frame ingest cost: the budget asks whether any
        # pipeline failed to keep up with the sensor, not what the average was.
        map_update_ms={
            k: max(d.add_frame_ms.get(k, 0.0) for d in datasets) for k in ("p50", "p95", "max")
        },
        datasets=datasets,
        config=asdict(cfg),
    )


def evaluate(
    suites: list[Suite],
    cfg: EvalConfig | None = None,
    workers: int = 1,
    keep_artifacts: bool = False,
    progress: ProgressFactory | None = None,
    on_dataset: Callable[[str], None] | None = None,
) -> Report:
    """Score every suite. A dataset is one pass, so workers spreads datasets."""
    cfg = cfg or EvalConfig()
    datasets = _run_datasets(suites, cfg, workers, keep_artifacts, progress, on_dataset)
    return _build_report(datasets, cfg)
