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

"""Benchmark Pink vs RoboPlan OInK IK backends on representative DimOS manipulation workloads.

Issue: https://github.com/dimensionalOS/dimos/issues/3232

For each supported robot, one RoboPlanWorld instance is shared by both solvers
(Pink is world-agnostic; RoboPlan OInK is the world itself). Targets are sampled
by drawing random collision-free joint configurations and mapping them through
the world's forward kinematics, so every target is reachable by construction.
Both backends receive identical targets and the same seed, are timed with
``time.perf_counter``, and every successful solution is independently verified
by pushing it back through the world's FK and comparing against the target with
``compute_pose_error``.

Usage:
    uv run python dimos/manipulation/benchmark_ik_backends.py --samples 200
    uv run python dimos/manipulation/benchmark_ik_backends.py --robots xarm7 --output /tmp/ik.json
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import resource
import statistics
import time
from typing import Any

import numpy as np

from dimos.manipulation.planning.factory import create_kinematics, create_world
from dimos.manipulation.planning.kinematics.config import (
    PinkKinematicsConfig,
    RoboPlanKinematicsConfig,
)
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

ROBOT_CONFIG_FACTORIES = {
    "xarm6": make_xarm6_model_config,
    "xarm7": make_xarm7_model_config,
}

# Sampled targets whose FK pose is in collision are rejected; allow slack for rejection sampling.
MAX_TARGET_TRIES_FACTOR = 20


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


def _run_backend(
    robot_name: str,
    backend_name: str,
    solver: KinematicsSpec,
    world: WorldSpec,
    robot_id: WorldRobotID,
    group_id: str,
    targets: Sequence[PoseStamped],
    seed: JointState,
    warmup: int,
    max_attempts: int,
) -> BackendRun:
    reference = world.get_joint_state(world.get_live_context(), robot_id)

    def solve_once(target: PoseStamped) -> tuple[IKResult, float]:
        start = time.perf_counter()
        result = solver.solve(
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
                backend=backend_name,
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
        backend=backend_name,
        records=records,
        peak_rss_delta_mb=_peak_rss_mb() - rss_before,
    )


def _percentile(values: Sequence[float], pct: float) -> float:
    return float(np.percentile(np.asarray(values), pct))


def _summarize(run: BackendRun) -> dict[str, Any]:
    successes = [r for r in run.records if r.status == "SUCCESS"]
    latencies = [r.wall_time_ms for r in run.records]
    verified_pos = [
        r.verified_position_error for r in successes if r.verified_position_error is not None
    ]
    verified_ori = [
        r.verified_orientation_error for r in successes if r.verified_orientation_error is not None
    ]
    status_counts: dict[str, int] = {}
    for r in run.records:
        status_counts[r.status] = status_counts.get(r.status, 0) + 1
    return {
        "robot": run.robot,
        "backend": run.backend,
        "samples": len(run.records),
        "success_rate": len(successes) / len(run.records) if run.records else 0.0,
        "status_counts": status_counts,
        "latency_ms": {
            "mean": statistics.fmean(latencies) if latencies else 0.0,
            "p50": _percentile(latencies, 50) if latencies else 0.0,
            "p95": _percentile(latencies, 95) if latencies else 0.0,
            "max": max(latencies) if latencies else 0.0,
        },
        "verified_position_error_m": {
            "mean": statistics.fmean(verified_pos) if verified_pos else None,
            "max": max(verified_pos) if verified_pos else None,
        },
        "verified_orientation_error_rad": {
            "mean": statistics.fmean(verified_ori) if verified_ori else None,
            "max": max(verified_ori) if verified_ori else None,
        },
        "peak_rss_delta_mb": run.peak_rss_delta_mb,
    }


def _print_summary(summaries: Sequence[dict[str, Any]]) -> None:
    header = (
        f"{'robot':<8} {'backend':<14} {'n':>5} {'success':>8} "
        f"{'p50 ms':>9} {'p95 ms':>9} {'mean ms':>9} "
        f"{'pos mm':>8} {'ori mrad':>9} {'rss ΔMB':>8}"
    )
    print(header)
    print("-" * len(header))
    for s in summaries:
        pos_mm = (s["verified_position_error_m"]["mean"] or 0.0) * 1000.0
        ori_mrad = (s["verified_orientation_error_rad"]["mean"] or 0.0) * 1000.0
        print(
            f"{s['robot']:<8} {s['backend']:<14} {s['samples']:>5} "
            f"{s['success_rate']:>7.1%} "
            f"{s['latency_ms']['p50']:>9.2f} {s['latency_ms']['p95']:>9.2f} "
            f"{s['latency_ms']['mean']:>9.2f} "
            f"{pos_mm:>8.3f} {ori_mrad:>9.3f} {s['peak_rss_delta_mb']:>8.1f}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robots",
        nargs="+",
        choices=sorted(ROBOT_CONFIG_FACTORIES),
        default=sorted(ROBOT_CONFIG_FACTORIES),
        help="Robots to benchmark (default: all).",
    )
    parser.add_argument("--samples", type=int, default=200, help="Timed solves per backend.")
    parser.add_argument(
        "--warmup", type=int, default=5, help="Warmup solves discarded per backend."
    )
    parser.add_argument("--max-attempts", type=int, default=10, help="IK attempts per solve.")
    parser.add_argument("--seed", type=int, default=0, help="RNG seed for target sampling.")
    parser.add_argument("--output", type=Path, default=None, help="Optional JSON output path.")
    args = parser.parse_args()

    runs: list[BackendRun] = []
    for robot_name in args.robots:
        print(f"[setup] building RoboPlanWorld for {robot_name} ...", flush=True)
        config = ROBOT_CONFIG_FACTORIES[robot_name]()
        world = create_world("roboplan")
        robot_id = world.add_robot(config)
        world.finalize()
        group_id = f"{config.name}/manipulator"

        rng = np.random.default_rng(args.seed)
        targets = _sample_reachable_targets(
            world, robot_id, group_id, args.samples + args.warmup, rng
        )
        seed = world.get_joint_state(world.get_live_context(), robot_id)

        solvers: dict[str, KinematicsSpec] = {
            "pink": create_kinematics(config=PinkKinematicsConfig()),
            "roboplan_oink": create_kinematics(
                config=RoboPlanKinematicsConfig(), world=world, world_backend="roboplan"
            ),
        }
        for backend_name, solver in solvers.items():
            print(f"[run] {robot_name} / {backend_name} ...", flush=True)
            runs.append(
                _run_backend(
                    robot_name,
                    backend_name,
                    solver,
                    world,
                    robot_id,
                    group_id,
                    targets,
                    seed,
                    args.warmup,
                    args.max_attempts,
                )
            )

    summaries = [_summarize(run) for run in runs]
    _print_summary(summaries)

    if args.output is not None:
        payload = {
            "seed": args.seed,
            "samples": args.samples,
            "warmup": args.warmup,
            "max_attempts": args.max_attempts,
            "summaries": summaries,
            "records": [asdict(record) for run in runs for record in run.records],
        }
        args.output.write_text(json.dumps(payload, indent=2))
        print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
