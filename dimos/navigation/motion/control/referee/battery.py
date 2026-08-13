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

"""Parallel episode batteries: N worker processes, one episode each.

Episodes are independent MuJoCo rollouts and nothing here scores CPU time
(plan_ms is a diagnostic), so process parallelism is free accuracy-wise —
unlike the planner referee, whose speed pillar demands a serial run. Each
(world, draw) pair gets its own deterministic rng stream, so results are
identical at any --jobs.

The OOD battery is the same generator under DIFFERENT rules and a disjoint
seed base: denser clutter, tighter gaps, longer routes. The optimizer sees
its score, but a candidate tuned to the in-distribution battery's quirks
meets structurally different worlds here.
"""

from __future__ import annotations

from concurrent import futures
from dataclasses import dataclass
import multiprocessing as mp
import os
from typing import Any
import zlib

import numpy as np

from dimos.navigation.motion.control.controller import load as load_controller
from dimos.navigation.motion.control.referee.episode import (
    DomainRandomization,
    EpisodeConfig,
    run_episode,
)
from dimos.navigation.motion.control.referee.judge import score_episode
from dimos.navigation.motion.embodiment import GO2
from dimos.navigation.motion.scenarios import GenRules, Scenario, generated

# Held-out generator: disjoint seed base, harder placement rules. go2-only —
# the physical robot in the sim is always the go2; judging its body against
# another embodiment's footprint would misread clearance.
OOD_SEED = 7700
OOD_RULES = GenRules(
    n_boxes=(10, 18),
    on_path=(3, 5),
    sprinkle=(12, 22),
    sprinkle_size=(0.05, 0.30),
    min_gap=0.75,
    spawn_clear=0.5,
    goal_dist=4.2,
)


def ood_worlds(count: int) -> list[Scenario]:
    worlds = generated(count, seed=OOD_SEED, rules=OOD_RULES, emb=GO2)
    return [
        Scenario(
            name=f"ood{i:03d}",
            boxes=s.boxes,
            goal=s.goal,
            start=s.start,
            expect=s.expect,
            emb=s.emb,
            note=s.note + " [ood]",
        )
        for i, s in enumerate(worlds)
    ]


@dataclass
class _Job:
    scenario: Scenario
    group: str  # "curated" | "gen" | "ood"
    draw: int


_WORKER: dict[str, Any] = {}


def _init_worker(policy_path: str, controller_name: str) -> None:
    # Keep numpy's pools out of the way of process-level parallelism.
    for v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ.setdefault(v, "1")
    from dimos.navigation.motion.simulation.policy import FreePolicy
    from dimos.utils.data import get_data

    _WORKER["policy"] = FreePolicy.load(get_data(policy_path))
    _WORKER["make"] = load_controller(controller_name)


def _run_job(
    job: _Job, cfg: EpisodeConfig, dr: DomainRandomization | None, seed: int
) -> dict[str, Any]:
    # One rng stream per (world, draw): identical results at any --jobs.
    # crc32, not hash() -- python string hashing is salted per process.
    name_key = zlib.crc32(job.scenario.name.encode())
    rng = np.random.default_rng([seed, name_key, job.draw])
    result = run_episode(job.scenario, _WORKER["make"](), _WORKER["policy"], cfg, dr=dr, rng=rng)
    row = score_episode(result)
    row["group"] = job.group
    row["draw"] = job.draw
    return row


def run_battery(
    scenarios: list[tuple[Scenario, str]],
    policy_path: str,
    controller_name: str,
    cfg: EpisodeConfig,
    dr: DomainRandomization | None = None,
    seed: int = 0,
    draws: int = 1,
    jobs: int = 1,
) -> list[dict[str, Any]]:
    """Score every (world, draw) pair; rows come back in submission order."""
    joblist = [_Job(sc, group, d) for sc, group in scenarios for d in range(draws if dr else 1)]
    if jobs <= 1:
        _init_worker(policy_path, controller_name)
        return [_run_job(j, cfg, dr, seed) for j in joblist]
    ctx = mp.get_context("spawn")
    with futures.ProcessPoolExecutor(
        max_workers=jobs,
        mp_context=ctx,
        initializer=_init_worker,
        initargs=(policy_path, controller_name),
    ) as pool:
        return list(
            pool.map(
                _run_job, joblist, [cfg] * len(joblist), [dr] * len(joblist), [seed] * len(joblist)
            )
        )


def group_summaries(rows: list[dict[str, Any]]) -> dict[str, Any]:
    """Per-group mean scores alongside the overall summary fields."""
    groups: dict[str, list[float]] = {}
    for r in rows:
        groups.setdefault(r.get("group", "curated"), []).append(r["total"])
    return {g: round(float(np.mean(v)), 2) for g, v in sorted(groups.items())}
