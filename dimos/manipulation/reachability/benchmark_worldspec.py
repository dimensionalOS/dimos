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

"""Reachability sampler benchmarks across direct MuJoCo and WorldSpec backends.

CLI::

    python -m dimos.manipulation.reachability.benchmark_worldspec \\
        --robot g1-left --samples 20000 --repeats 3 \\
        --backend direct-mujoco --backend mujoco --backend drake
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, replace
import json
import time
from typing import Literal

import numpy as np

from dimos.manipulation.reachability.capability_map import MapParams
from dimos.manipulation.reachability.construct import (
    ConstructionSpec,
    _DirectMujocoArmSampler,
    _WorldSpecArmSampler,
    arm_spec,
)
from dimos.manipulation.reachability.robots import arm_model, list_robots

BenchmarkBackend = Literal["direct-mujoco", "mujoco", "drake"]


@dataclass(frozen=True)
class BenchmarkResult:
    backend: str
    robot: str
    samples: int
    repeats: int
    kept: int = 0
    rejected: int = 0
    init_s: float = 0.0
    sample_s: float = 0.0
    samples_per_s: float = 0.0
    kept_per_s: float = 0.0
    error: str | None = None


def benchmark_backends(
    robot: str = "g1-left",
    samples: int = 20_000,
    repeats: int = 3,
    chunk: int = 5_000,
    backends: tuple[str, ...] = ("direct-mujoco", "mujoco", "drake"),
    seed: int = 0,
) -> list[BenchmarkResult]:
    """Run reachability sampling benchmarks for one registered arm."""
    base = _benchmark_spec(robot)
    return [
        _benchmark_backend(
            replace(base, world_backend="mujoco" if backend == "direct-mujoco" else backend),
            backend=backend,
            samples=samples,
            repeats=repeats,
            chunk=chunk,
            seed=seed,
        )
        for backend in backends
    ]


def _benchmark_spec(robot: str) -> ConstructionSpec:
    model = arm_model(robot)
    params = model.params or MapParams.at_base_height(model.base_height)
    return arm_spec(robot, params=params, world_backend="mujoco")


def _benchmark_backend(
    spec: ConstructionSpec,
    *,
    backend: str,
    samples: int,
    repeats: int,
    chunk: int,
    seed: int,
) -> BenchmarkResult:
    try:
        t0 = time.perf_counter()
        sampler = (
            _DirectMujocoArmSampler(spec)
            if backend == "direct-mujoco"
            else _WorldSpecArmSampler(spec)
        )
        init_s = time.perf_counter() - t0
        kept = 0
        rejected = 0
        t_sample = 0.0
        for repeat_idx in range(repeats):
            rng = np.random.default_rng(seed + repeat_idx)
            remaining = samples
            t0 = time.perf_counter()
            while remaining > 0:
                n = min(chunk, remaining)
                positions, _, n_rejected = sampler.sample_chunk(n, rng)
                kept += len(positions)
                rejected += n_rejected
                remaining -= n
            t_sample += time.perf_counter() - t0
    except Exception as exc:
        return BenchmarkResult(
            backend=backend,
            robot=spec.robot,
            samples=samples,
            repeats=repeats,
            error=f"{type(exc).__name__}: {exc}",
        )

    total_samples = samples * repeats
    return BenchmarkResult(
        backend=backend,
        robot=spec.robot,
        samples=samples,
        repeats=repeats,
        kept=kept,
        rejected=rejected,
        init_s=init_s,
        sample_s=t_sample,
        samples_per_s=total_samples / max(t_sample, 1e-9),
        kept_per_s=kept / max(t_sample, 1e-9),
    )


def cli_main() -> None:
    parser = argparse.ArgumentParser(description="Benchmark reachability WorldSpec samplers.")
    parser.add_argument("--robot", choices=list_robots(), default="g1-left")
    parser.add_argument("--samples", type=int, default=20_000)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--chunk", type=int, default=5_000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--backend",
        action="append",
        choices=("direct-mujoco", "mujoco", "drake"),
        default=None,
        help="Backend to benchmark. Repeat to compare multiple backends.",
    )
    parser.add_argument("--json", action="store_true", help="Emit JSON instead of a text table.")
    args = parser.parse_args()

    results = benchmark_backends(
        robot=args.robot,
        samples=args.samples,
        repeats=args.repeats,
        chunk=args.chunk,
        backends=tuple(args.backend or ("direct-mujoco", "mujoco", "drake")),
        seed=args.seed,
    )
    if args.json:
        print(json.dumps([asdict(result) for result in results], indent=2))
    else:
        _print_table(results)


def _print_table(results: list[BenchmarkResult]) -> None:
    print(
        f"{'backend':<14} {'init_s':>8} {'sample_s':>9} {'samples/s':>12} "
        f"{'kept':>8} {'rejected':>9} error"
    )
    for result in results:
        if result.error:
            print(
                f"{result.backend:<14} {'-':>8} {'-':>9} {'-':>12} {'-':>8} {'-':>9} {result.error}"
            )
            continue
        print(
            f"{result.backend:<14} {result.init_s:8.3f} {result.sample_s:9.3f} "
            f"{result.samples_per_s:12.0f} {result.kept:8d} {result.rejected:9d}"
        )


if __name__ == "__main__":
    cli_main()


__all__ = ["BenchmarkResult", "benchmark_backends"]
