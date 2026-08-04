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

For each robot, targets are sampled by drawing random collision-free joint
configurations and mapping them through the world's forward kinematics, so every
target is reachable by construction. Every backend runs against its own freshly
built world (no shared mutable scene), receives identical targets and an
equivalent seed, is timed with ``time.perf_counter``, and every successful
solution is independently verified by pushing it back through the world's FK
and comparing against the target with ``compute_pose_error``.

Usage:
    uv run python benchmarks/ik_backends.py
    uv run python benchmarks/ik_backends.py --robot xarm7 --solver pink --max-attempts 3
    uv run python benchmarks/ik_backends.py --samples 200 --output /tmp/ik.json

Extend by adding entries to ``ROBOT_CONFIG_FACTORIES`` / ``_solver_registry``.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import resource
import statistics
import time

import numpy as np
import typer

from dimos.manipulation.planning.factory import create_kinematics, create_world
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

ROBOT_CONFIG_FACTORIES: dict[str, Callable[[], RobotModelConfig]] = {
    "xarm6": make_xarm6_model_config,
    "xarm7": make_xarm7_model_config,
}

# Sampled targets whose FK pose is in collision are rejected; allow slack for rejection sampling.
MAX_TARGET_TRIES_FACTOR = 20


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

    robot: str
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
    """Records plus coarse resource usage for one backend on one robot."""

    robot: str
    backend: str
    records: list[SolveRecord]
    peak_rss_delta_mb: float


@dataclass
class DistributionStats:
    """Summary statistics for one measured distribution."""

    mean: float | None
    p50: float | None
    p95: float | None
    max: float | None


@dataclass
class RunSummary:
    """Aggregate result for one (robot, backend) run."""

    robot: str
    backend: str
    samples: int
    success_rate: float
    status_counts: dict[str, int]
    latency_ms: DistributionStats
    verified_position_error_m: DistributionStats
    verified_orientation_error_rad: DistributionStats
    peak_rss_delta_mb: float


def _sample_reachable_targets(
    world: WorldSpec,
    robot_id: WorldRobotID,
    group_id: str,
    count: int,
    rng: np.random.Generator,
) -> list[PoseStamped]:
    """Sample reachable, collision-free targets via FK of random valid configurations."""
    lower, upper = world.get_joint_limits(robot_id)
    reference = world.get_joint_state(world.get_live_context(), robot_id)
    targets: list[PoseStamped] = []
    tries = 0
    with world.scratch_context() as ctx:
        while len(targets) < count and tries < count * MAX_TARGET_TRIES_FACTOR:
            tries += 1
            q = rng.uniform(lower, upper)
            joint_state = JointState(
                name=list(reference.name),
                position=[float(v) for v in q],
            )
            if not world.check_config_collision_free(robot_id, joint_state):
                continue
            world.set_joint_state(ctx, robot_id, joint_state)
            pose = world.get_group_ee_pose(ctx, group_id)
            # OInK requires world-frame targets; rebuild with an explicit frame_id.
            targets.append(
                PoseStamped(
                    position=pose.position,
                    orientation=pose.orientation,
                    frame_id="world",
                )
            )
    if len(targets) < count:
        raise RuntimeError(
            f"Only sampled {len(targets)}/{count} collision-free targets after {tries} tries"
        )
    return targets


def _local_solution_joint_state(
    reference: JointState,
    solution: JointState,
) -> JointState:
    """Re-express a solution in the robot's local joint-name order.

    Pink returns config (local) joint names, OInK returns global ``robot/joint``
    names; positions are matched by local name so FK verification is backend-agnostic.
    Joints absent from the solution keep their reference positions.
    """
    by_local_name: dict[str, float] = {}
    for name, position in zip(solution.name, solution.position, strict=True):
        by_local_name[name.rsplit("/", 1)[-1]] = position
    return JointState(
        name=list(reference.name),
        position=[
            by_local_name.get(name, ref)
            for name, ref in zip(reference.name, reference.position, strict=True)
        ],
    )


def _verify_solution(
    world: WorldSpec,
    robot_id: WorldRobotID,
    group_id: str,
    reference: JointState,
    result: IKResult,
    target: PoseStamped,
) -> tuple[float, float] | None:
    """Push a successful solution through the world's FK and score against the target."""
    if not result.is_success() or result.joint_state is None:
        return None
    solution = _local_solution_joint_state(reference, result.joint_state)
    with world.scratch_context() as ctx:
        world.set_joint_state(ctx, robot_id, solution)
        actual = world.get_group_ee_pose(ctx, group_id)
    return compute_pose_error(pose_to_matrix(actual), pose_to_matrix(target))


def _peak_rss_mb() -> float:
    """Peak RSS of this process in MB (Linux ru_maxrss is KiB)."""
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0


def _build_world(config: RobotModelConfig) -> tuple[WorldSpec, WorldRobotID, str]:
    """Build a fresh finalized RoboPlan world for one robot."""
    world = create_world("roboplan")
    robot_id = world.add_robot(config)
    world.finalize()
    return world, robot_id, f"{config.name}/manipulator"


def _run_backend(
    robot_name: str,
    solver: SolverSpec,
    targets: Sequence[PoseStamped],
    warmup: int,
    max_attempts: int,
) -> BackendRun:
    """Run one backend against its own freshly built world."""
    world, robot_id, group_id = _build_world(ROBOT_CONFIG_FACTORIES[robot_name]())
    kinematics = solver.create(world)
    reference = world.get_joint_state(world.get_live_context(), robot_id)
    seed = reference

    def solve_once(target: PoseStamped) -> tuple[IKResult, float]:
        start = time.perf_counter()
        result = kinematics.solve(
            world,
            robot_id,
            target,
            seed=seed,
            check_collision=True,
            max_attempts=max_attempts,
        )
        return result, (time.perf_counter() - start) * 1000.0

    for target in targets[:warmup]:
        solve_once(target)

    rss_before = _peak_rss_mb()
    records: list[SolveRecord] = []
    for index, target in enumerate(targets[warmup:]):
        result, wall_time_ms = solve_once(target)
        verified = _verify_solution(world, robot_id, group_id, reference, result, target)
        records.append(
            SolveRecord(
                robot=robot_name,
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
        robot=robot_name,
        backend=solver.name,
        records=records,
        peak_rss_delta_mb=_peak_rss_mb() - rss_before,
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
        robot=run.robot,
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
        peak_rss_delta_mb=run.peak_rss_delta_mb,
    )


def _print_summary(summaries: Sequence[RunSummary]) -> None:
    header = (
        f"{'robot':<8} {'backend':<14} {'n':>5} {'success':>8} "
        f"{'p50 ms':>9} {'p95 ms':>9} {'mean ms':>9} "
        f"{'pos mm':>8} {'ori mrad':>9} {'rss ΔMB':>8}"
    )
    print(header)
    print("-" * len(header))
    for s in summaries:
        pos_mm = (s.verified_position_error_m.mean or 0.0) * 1000.0
        ori_mrad = (s.verified_orientation_error_rad.mean or 0.0) * 1000.0
        print(
            f"{s.robot:<8} {s.backend:<14} {s.samples:>5} "
            f"{s.success_rate:>7.1%} "
            f"{s.latency_ms.p50 or 0.0:>9.2f} {s.latency_ms.p95 or 0.0:>9.2f} "
            f"{s.latency_ms.mean or 0.0:>9.2f} "
            f"{pos_mm:>8.3f} {ori_mrad:>9.3f} {s.peak_rss_delta_mb:>8.1f}"
        )


app = typer.Typer(help=__doc__, add_completion=False)


@app.command()
def main(
    robots: list[str] = typer.Option(
        list(ROBOT_CONFIG_FACTORIES), "--robot", "-r", help="Robots to benchmark (repeatable)."
    ),
    solvers: list[str] = typer.Option(
        [], "--solver", "-s", help="IK backends to run (repeatable, default: all)."
    ),
    samples: int = typer.Option(200, help="Timed solves per backend."),
    warmup: int = typer.Option(10, help="Warmup solves discarded per backend."),
    max_attempts: int = typer.Option(10, help="IK attempts per solve (all backends)."),
    pink_max_iterations: int = typer.Option(200, help="Pink iterations per attempt."),
    seed: int = typer.Option(0, help="RNG seed for target sampling."),
    output: Path | None = typer.Option(None, help="Optional JSON output path."),
) -> None:
    registry = _solver_registry(pink_max_iterations)
    for robot in robots:
        if robot not in ROBOT_CONFIG_FACTORIES:
            raise typer.BadParameter(
                f"Unknown robot '{robot}'. Available: {sorted(ROBOT_CONFIG_FACTORIES)}"
            )
    selected = solvers or list(registry)
    for solver in selected:
        if solver not in registry:
            raise typer.BadParameter(f"Unknown solver '{solver}'. Available: {sorted(registry)}")

    runs: list[BackendRun] = []
    for robot_name in robots:
        print(
            f"[setup] {robot_name}: sampling {samples + warmup} reachable targets ...", flush=True
        )
        sample_world, sample_robot_id, sample_group_id = _build_world(
            ROBOT_CONFIG_FACTORIES[robot_name]()
        )
        rng = np.random.default_rng(seed)
        targets = _sample_reachable_targets(
            sample_world, sample_robot_id, sample_group_id, samples + warmup, rng
        )
        for solver_name in selected:
            print(f"[run] {robot_name} / {solver_name} ...", flush=True)
            runs.append(
                _run_backend(robot_name, registry[solver_name], targets, warmup, max_attempts)
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
