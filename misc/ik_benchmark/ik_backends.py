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

"""Benchmark manipulation IK backends on representative DimOS workloads.

Issue: https://github.com/dimensionalOS/dimos/issues/3232

Scenarios are one or more robots sharing a world (single arm or dual arm).
Targets are sampled by drawing random joint configurations for every robot,
keeping only scene-wide collision-free ones, and mapping them through the
world's forward kinematics, so every target is reachable by construction.
Every backend runs against its own freshly built world (no shared mutable
scene), receives identical targets and an equivalent seed, is timed with
``time.perf_counter``, and every successful solution is independently verified
by pushing it back through the world's FK and comparing against the target
with ``compute_pose_error``.

Usage:
    uv run python misc/ik_benchmark/ik_backends.py
    uv run python misc/ik_benchmark/ik_backends.py --scenario dual_xarm6 --solver pink
    uv run python misc/ik_benchmark/ik_backends.py --samples 200 --output /tmp/ik.json

Extend by adding entries to ``SCENARIOS`` / ``_solver_registry``.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import resource
import statistics
import time

import numpy as np
import typer

from dimos.manipulation.planning.factory import create_kinematics, create_world
from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.kinematics.config import (
    PinkKinematicsConfig,
    RoboPlanKinematicsConfig,
)
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.models import IKResult, WorldRobotID
from dimos.manipulation.planning.spec.protocols import KinematicsSpec, WorldSpec
from dimos.manipulation.planning.utils.kinematics_utils import compute_pose_error
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.xarm.config import (
    make_xarm6_model_config,
    make_xarm7_model_config,
)
from dimos.utils.transform_utils import pose_to_matrix

# Sampled targets whose FK pose is in collision are rejected; allow slack for rejection sampling.
MAX_TARGET_TRIES_FACTOR = 20


@dataclass(frozen=True)
class ScenarioSpec:
    """One benchmark scenario: one or more robots sharing a world."""

    name: str
    make_configs: Callable[[], list[RobotModelConfig]]


def _single(make_config: Callable[[], RobotModelConfig]) -> Callable[[], list[RobotModelConfig]]:
    return lambda: [make_config()]


def _dual_xarm6_configs() -> list[RobotModelConfig]:
    # Mirrors the dual_xarm6_planner blueprint: two xArm6 arms 1 m apart.
    return [
        make_xarm6_model_config(name="left_arm", y_offset=0.5),
        make_xarm6_model_config(name="right_arm", y_offset=-0.5),
    ]


SCENARIOS: dict[str, ScenarioSpec] = {
    "xarm6": ScenarioSpec("xarm6", _single(make_xarm6_model_config)),
    "xarm7": ScenarioSpec("xarm7", _single(make_xarm7_model_config)),
    "dual_xarm6": ScenarioSpec("dual_xarm6", _dual_xarm6_configs),
}


@dataclass(frozen=True)
class SolverSpec:
    """Constructs one IK backend instance for a freshly built world."""

    name: str
    create: Callable[[WorldSpec], KinematicsSpec]


def _solver_registry(pink_max_iterations: int) -> dict[str, SolverSpec]:
    """Available IK backends. Add new solvers here to include them in the benchmark."""

    def create_pink(world: WorldSpec) -> KinematicsSpec:
        del world  # Pink is world-agnostic; it builds its own Pinocchio model.
        return create_kinematics(config=PinkKinematicsConfig(max_iterations=pink_max_iterations))

    def create_roboplan_oink(world: WorldSpec) -> KinematicsSpec:
        return create_kinematics(
            config=RoboPlanKinematicsConfig(), world=world, world_backend="roboplan"
        )

    return {
        "pink": SolverSpec("pink", create_pink),
        "roboplan_oink": SolverSpec("roboplan_oink", create_roboplan_oink),
    }


@dataclass
class SolveRecord:
    """One timed IK solve."""

    scenario: str
    backend: str
    target_index: int
    status: str
    wall_time_ms: float
    position_error: float
    orientation_error: float
    iterations: int
    verified_position_error: float | None
    verified_orientation_error: float | None
    message: str


@dataclass
class BackendRun:
    """Records plus coarse resource usage for one backend on one scenario."""

    scenario: str
    backend: str
    records: list[SolveRecord]
    rss_delta_mb: float


@dataclass
class DistributionStats:
    """Summary statistics for one measured distribution."""

    mean: float | None
    p50: float | None
    p95: float | None
    max: float | None


@dataclass
class RunSummary:
    """Aggregate result for one (scenario, backend) run."""

    scenario: str
    backend: str
    samples: int
    success_rate: float
    status_counts: dict[str, int]
    latency_ms: DistributionStats
    verified_position_error_m: DistributionStats
    verified_orientation_error_rad: DistributionStats
    rss_delta_mb: float


@dataclass
class _Scene:
    """A finalized world plus its resolved planning groups."""

    world: WorldSpec
    groups: list[PlanningGroup]
    robot_ids: dict[str, WorldRobotID]  # keyed by robot (config) name


def _build_scene(configs: Sequence[RobotModelConfig]) -> _Scene:
    """Build a fresh finalized RoboPlan world for the given robot configs."""
    world = create_world("roboplan")
    robot_ids = {config.name: world.add_robot(config) for config in configs}
    world.finalize()
    registry = PlanningGroupRegistry(configs)
    groups = [group for config in configs for group in registry.groups_for_robot(config.name)]
    return _Scene(world=world, groups=groups, robot_ids=robot_ids)


def _combined_seed(scene: _Scene) -> JointState:
    """Current live state of every robot, expressed with global joint names."""
    names: list[str] = []
    positions: list[float] = []
    for robot_name, robot_id in scene.robot_ids.items():
        state = scene.world.get_joint_state(scene.world.get_live_context(), robot_id)
        names.extend(f"{robot_name}/{name}" for name in state.name)
        positions.extend(state.position)
    return JointState(name=names, position=positions)


def _sample_reachable_targets(
    scene: _Scene,
    count: int,
    rng: np.random.Generator,
) -> list[dict[PlanningGroup, PoseStamped]]:
    """Sample reachable, scene-wide collision-free multi-group targets via FK."""
    limits = {
        robot_id: scene.world.get_joint_limits(robot_id) for robot_id in scene.robot_ids.values()
    }
    references = {
        robot_id: scene.world.get_joint_state(scene.world.get_live_context(), robot_id)
        for robot_id in scene.robot_ids.values()
    }
    targets: list[dict[PlanningGroup, PoseStamped]] = []
    tries = 0
    with scene.world.scratch_context() as ctx:
        while len(targets) < count and tries < count * MAX_TARGET_TRIES_FACTOR:
            tries += 1
            for robot_id, reference in references.items():
                lower, upper = limits[robot_id]
                q = rng.uniform(lower, upper)
                scene.world.set_joint_state(
                    ctx,
                    robot_id,
                    JointState(
                        name=list(reference.name),
                        position=[float(v) for v in q],
                    ),
                )
            # Scene-wide collision check: covers self-, inter-robot, and obstacle collisions.
            if not scene.world.is_collision_free(ctx, next(iter(scene.robot_ids.values()))):
                continue
            # OInK requires world-frame targets; rebuild with an explicit frame_id.
            targets.append(
                {
                    group: PoseStamped(
                        position=pose.position,
                        orientation=pose.orientation,
                        frame_id="world",
                    )
                    for group in scene.groups
                    for pose in [scene.world.get_group_ee_pose(ctx, group.id)]
                }
            )
    if len(targets) < count:
        raise RuntimeError(
            f"Only sampled {len(targets)}/{count} collision-free targets after {tries} tries"
        )
    return targets


def _verify_solution(
    scene: _Scene,
    result: IKResult,
    target: Mapping[PlanningGroup, PoseStamped],
) -> tuple[float, float] | None:
    """Push a successful solution through the world's FK and score against the target.

    Solutions use global ``robot/joint`` names (both backends); errors are the
    worst across all pose-targeted groups.
    """
    if not result.is_success() or result.joint_state is None:
        return None
    positions_by_robot: dict[str, dict[str, float]] = {
        robot_name: {} for robot_name in scene.robot_ids
    }
    for name, position in zip(result.joint_state.name, result.joint_state.position, strict=True):
        robot_name, _, local_name = name.rpartition("/")
        if robot_name in positions_by_robot:
            positions_by_robot[robot_name][local_name] = position
    with scene.world.scratch_context() as ctx:
        for robot_name, robot_id in scene.robot_ids.items():
            reference = scene.world.get_joint_state(scene.world.get_live_context(), robot_id)
            solved = positions_by_robot[robot_name]
            scene.world.set_joint_state(
                ctx,
                robot_id,
                JointState(
                    name=list(reference.name),
                    position=[
                        solved.get(name, ref)
                        for name, ref in zip(reference.name, reference.position, strict=True)
                    ],
                ),
            )
        errors = [
            compute_pose_error(
                pose_to_matrix(scene.world.get_group_ee_pose(ctx, group.id)),
                pose_to_matrix(target_pose),
            )
            for group, target_pose in target.items()
        ]
    return (
        max(error[0] for error in errors),
        max(error[1] for error in errors),
    )


def _current_rss_mb() -> float:
    """Current resident set size of this process in MB.

    Reads /proc/self/statm so the per-backend delta is a real before/after
    difference; ru_maxrss is a process-lifetime high-water mark and would
    report zero growth for every backend after the first one peaks.
    """
    try:
        with open("/proc/self/statm") as fh:
            resident_pages = int(fh.read().split()[1])
        return resident_pages * os.sysconf("SC_PAGESIZE") / 1024.0**2
    except OSError:
        return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0


def _run_backend(
    scenario: ScenarioSpec,
    solver: SolverSpec,
    targets: Sequence[dict[PlanningGroup, PoseStamped]],
    warmup: int,
    max_attempts: int,
) -> BackendRun:
    """Run one backend against its own freshly built world."""
    scene = _build_scene(scenario.make_configs())
    kinematics = solver.create(scene.world)
    seed = _combined_seed(scene)

    def solve_once(target: Mapping[PlanningGroup, PoseStamped]) -> tuple[IKResult, float]:
        start = time.perf_counter()
        result = kinematics.solve_pose_targets(
            scene.world,
            target,
            seed=seed,
            check_collision=True,
            max_attempts=max_attempts,
        )
        return result, (time.perf_counter() - start) * 1000.0

    for target in targets[:warmup]:
        solve_once(target)

    rss_before = _current_rss_mb()
    records: list[SolveRecord] = []
    for index, target in enumerate(targets[warmup:]):
        result, wall_time_ms = solve_once(target)
        verified = _verify_solution(scene, result, target)
        records.append(
            SolveRecord(
                scenario=scenario.name,
                backend=solver.name,
                target_index=index,
                status=result.status.name,
                wall_time_ms=wall_time_ms,
                position_error=result.position_error,
                orientation_error=result.orientation_error,
                iterations=result.iterations,
                verified_position_error=verified[0] if verified else None,
                verified_orientation_error=verified[1] if verified else None,
                message=result.message,
            )
        )
    return BackendRun(
        scenario=scenario.name,
        backend=solver.name,
        records=records,
        rss_delta_mb=_current_rss_mb() - rss_before,
    )


def _distribution_stats(values: Sequence[float]) -> DistributionStats:
    if not values:
        return DistributionStats(mean=None, p50=None, p95=None, max=None)
    arr = np.asarray(values)
    return DistributionStats(
        mean=statistics.fmean(values),
        p50=float(np.percentile(arr, 50)),
        p95=float(np.percentile(arr, 95)),
        max=float(arr.max()),
    )


def _summarize(run: BackendRun) -> RunSummary:
    successes = [r for r in run.records if r.status == "SUCCESS"]
    status_counts: dict[str, int] = {}
    for r in run.records:
        status_counts[r.status] = status_counts.get(r.status, 0) + 1
    return RunSummary(
        scenario=run.scenario,
        backend=run.backend,
        samples=len(run.records),
        success_rate=len(successes) / len(run.records) if run.records else 0.0,
        status_counts=status_counts,
        latency_ms=_distribution_stats([r.wall_time_ms for r in run.records]),
        verified_position_error_m=_distribution_stats(
            [r.verified_position_error for r in successes if r.verified_position_error is not None]
        ),
        verified_orientation_error_rad=_distribution_stats(
            [
                r.verified_orientation_error
                for r in successes
                if r.verified_orientation_error is not None
            ]
        ),
        rss_delta_mb=run.rss_delta_mb,
    )


def _print_summary(summaries: Sequence[RunSummary]) -> None:
    header = (
        f"{'scenario':<12} {'backend':<14} {'n':>5} {'success':>8} "
        f"{'p50 ms':>9} {'p95 ms':>9} {'mean ms':>9} "
        f"{'pos mm':>8} {'ori mrad':>9} {'rss ΔMB':>8}"
    )
    print(header)
    print("-" * len(header))
    for s in summaries:
        pos_mm = (s.verified_position_error_m.mean or 0.0) * 1000.0
        ori_mrad = (s.verified_orientation_error_rad.mean or 0.0) * 1000.0
        print(
            f"{s.scenario:<12} {s.backend:<14} {s.samples:>5} "
            f"{s.success_rate:>7.1%} "
            f"{s.latency_ms.p50 or 0.0:>9.2f} {s.latency_ms.p95 or 0.0:>9.2f} "
            f"{s.latency_ms.mean or 0.0:>9.2f} "
            f"{pos_mm:>8.3f} {ori_mrad:>9.3f} {s.rss_delta_mb:>8.1f}"
        )


app = typer.Typer(help=__doc__, add_completion=False)


@app.command()
def main(
    scenarios: list[str] = typer.Option(
        list(SCENARIOS), "--scenario", "-s", help="Scenarios to benchmark (repeatable)."
    ),
    solvers: list[str] = typer.Option(
        [], "--solver", help="IK backends to run (repeatable, default: all)."
    ),
    samples: int = typer.Option(200, help="Timed solves per backend."),
    warmup: int = typer.Option(10, help="Warmup solves discarded per backend."),
    max_attempts: int = typer.Option(10, help="IK attempts per solve (all backends)."),
    pink_max_iterations: int = typer.Option(200, help="Pink iterations per attempt."),
    seed: int = typer.Option(0, help="RNG seed for target sampling."),
    output: Path | None = typer.Option(None, help="Optional JSON output path."),
) -> None:
    registry = _solver_registry(pink_max_iterations)
    for scenario in scenarios:
        if scenario not in SCENARIOS:
            raise typer.BadParameter(
                f"Unknown scenario '{scenario}'. Available: {sorted(SCENARIOS)}"
            )
    selected = solvers or list(registry)
    for solver in selected:
        if solver not in registry:
            raise typer.BadParameter(f"Unknown solver '{solver}'. Available: {sorted(registry)}")

    runs: list[BackendRun] = []
    for scenario_name in scenarios:
        scenario_spec = SCENARIOS[scenario_name]
        print(
            f"[setup] {scenario_name}: sampling {samples + warmup} reachable targets ...",
            flush=True,
        )
        sample_scene = _build_scene(scenario_spec.make_configs())
        rng = np.random.default_rng(seed)
        targets = _sample_reachable_targets(sample_scene, samples + warmup, rng)
        for solver_name in selected:
            print(f"[run] {scenario_name} / {solver_name} ...", flush=True)
            runs.append(
                _run_backend(scenario_spec, registry[solver_name], targets, warmup, max_attempts)
            )

    summaries = [_summarize(run) for run in runs]
    _print_summary(summaries)

    if output is not None:
        payload = {
            "seed": seed,
            "samples": samples,
            "warmup": warmup,
            "max_attempts": max_attempts,
            "pink_max_iterations": pink_max_iterations,
            "summaries": [asdict(s) for s in summaries],
            "records": [asdict(record) for run in runs for record in run.records],
        }
        output.write_text(json.dumps(payload, indent=2))
        print(f"wrote {output}")


if __name__ == "__main__":
    app()
