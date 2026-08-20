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

"""Canonical workload matrix for public and smoke campaigns."""

from __future__ import annotations

import random

from dimos.protocol.pubsub.benchmark.model import (
    Cohort,
    Environment,
    NetworkProfile,
    Stack,
    TopicWorkload,
    TrialSpec,
    Workload,
)

SYNTHETIC_WORKLOADS = tuple(
    Workload(
        name=f"bytes-{size}-{rate:g}hz",
        topics=(TopicWorkload("payload", size, rate, Cohort.FRESHNESS),),
    )
    for size, rate in (
        (64, 1000.0),
        (1024, 1000.0),
        (16 * 1024, 100.0),
        (60 * 1024, 100.0),
        (64 * 1024, 100.0),
        (65 * 1024, 100.0),
        (256 * 1024, 30.0),
        (1024 * 1024, 30.0),
        (4 * 1024 * 1024, 10.0),
    )
)

SATURATION_WORKLOADS = tuple(
    Workload(
        name=f"bytes-{size}-saturation",
        topics=(TopicWorkload("payload", size, 0.0, Cohort.FRESHNESS),),
        saturation=True,
        saturation_max_messages=100_000,
        saturation_max_bytes=4 * 1024 * 1024 * 1024,
    )
    for size in (64, 64 * 1024, 1024 * 1024, 4 * 1024 * 1024)
)

ROBOT_WORKLOAD = Workload(
    name="vision-lidar-stack",
    topics=(
        TopicWorkload("control", 64, 50.0, Cohort.RELIABLE, "control"),
        TopicWorkload("odometry", 256, 100.0, Cohort.FRESHNESS, "odometry"),
        TopicWorkload("image", 1280 * 720 * 3, 30.0, Cohort.FRESHNESS, "image"),
        TopicWorkload("pointcloud", int(1.6 * 1024 * 1024), 10.0, Cohort.FRESHNESS, "pointcloud"),
    ),
)

SMOKE_WORKLOAD = Workload(
    name="smoke-1kib-100hz",
    topics=(TopicWorkload("payload", 1024, 100.0, Cohort.FRESHNESS),),
)


def _stacks_for(cohort: Cohort) -> tuple[Stack, ...]:
    if cohort == Cohort.RELIABLE:
        return (Stack.ZENOH, Stack.ROS2_ZENOH)
    return (Stack.LCM, Stack.ZENOH, Stack.ROS2_ZENOH)


Cell = tuple[Workload, Cohort, Stack, Environment, NetworkProfile, int]


def _cells_for(
    workload: Workload,
    cohort: Cohort,
    environment: Environment,
    profile: NetworkProfile,
    *,
    subscribers: int = 1,
) -> list[Cell]:
    return [
        (workload, cohort, stack, environment, profile, subscribers)
        for stack in _stacks_for(cohort)
    ]


def _public_cells() -> list[Cell]:
    """Return the 146 curated cells; no dimension is implicitly cross-joined."""
    cells: list[Cell] = []
    reliable_names = {
        "bytes-64-1000hz",
        "bytes-1024-1000hz",
        "bytes-65536-100hz",
        "bytes-1048576-30hz",
        "bytes-4194304-10hz",
    }
    for environment in (Environment.LOCAL, Environment.EMULATED):
        for workload in SYNTHETIC_WORKLOADS:
            cells.extend(
                _cells_for(
                    workload,
                    Cohort.FRESHNESS,
                    environment,
                    NetworkProfile.CLEAN,
                )
            )
            if workload.name in reliable_names:
                cells.extend(
                    _cells_for(
                        workload,
                        Cohort.RELIABLE,
                        environment,
                        NetworkProfile.CLEAN,
                    )
                )

    impaired_names = {"bytes-1024-1000hz", "bytes-1048576-30hz"}
    for profile in (NetworkProfile.CONSTRAINED, NetworkProfile.DEGRADED):
        for workload in SYNTHETIC_WORKLOADS:
            if workload.name not in impaired_names:
                continue
            for cohort in (Cohort.FRESHNESS, Cohort.RELIABLE):
                cells.extend(_cells_for(workload, cohort, Environment.EMULATED, profile))

    for environment, profile in (
        (Environment.LOCAL, NetworkProfile.CLEAN),
        (Environment.EMULATED, NetworkProfile.CLEAN),
        (Environment.EMULATED, NetworkProfile.CONSTRAINED),
        (Environment.EMULATED, NetworkProfile.DEGRADED),
    ):
        cells.extend(_cells_for(ROBOT_WORKLOAD, Cohort.MIXED, environment, profile))

    reliable_saturation_names = {
        "bytes-65536-saturation",
        "bytes-1048576-saturation",
    }
    for workload in SATURATION_WORKLOADS:
        cells.extend(
            _cells_for(
                workload,
                Cohort.FRESHNESS,
                Environment.EMULATED,
                NetworkProfile.CLEAN,
            )
        )
        if workload.name in reliable_saturation_names:
            cells.extend(
                _cells_for(
                    workload,
                    Cohort.RELIABLE,
                    Environment.EMULATED,
                    NetworkProfile.CLEAN,
                )
            )

    fanout = {
        "bytes-65536-100hz": (Cohort.FRESHNESS,),
        "bytes-1048576-30hz": (Cohort.FRESHNESS, Cohort.RELIABLE),
    }
    for workload in SYNTHETIC_WORKLOADS:
        for cohort in fanout.get(workload.name, ()):
            for subscribers in (2, 4, 8):
                cells.extend(
                    _cells_for(
                        workload,
                        cohort,
                        Environment.EMULATED,
                        NetworkProfile.CLEAN,
                        subscribers=subscribers,
                    )
                )

    if len(cells) != 146 or len(set(cells)) != 146:
        raise AssertionError("The curated public benchmark must contain 146 unique cells")
    return cells


def build_matrix(
    suite: str,
    *,
    repetitions: int | None = None,
    warmup_s: float | None = None,
    duration_s: float | None = None,
    drain_s: float | None = None,
    seed: int = 7,
) -> list[TrialSpec]:
    """Build the explicit, deterministic benchmark campaign."""
    if suite not in {"smoke", "public"}:
        raise ValueError(f"Unknown benchmark suite: {suite}")

    if suite == "smoke":
        cells = [
            (
                SMOKE_WORKLOAD,
                Cohort.FRESHNESS,
                stack,
                Environment.LOCAL,
                NetworkProfile.CLEAN,
                1,
            )
            for stack in _stacks_for(Cohort.FRESHNESS)
        ]
        defaults = (1, 0.2, 1.0, 0.2)
    else:
        cells = _public_cells()
        defaults = (5, 2.0, 20.0, 1.0)

    default_repetitions, default_warmup, default_duration, default_drain = defaults
    reps = repetitions if repetitions is not None else default_repetitions
    warmup = warmup_s if warmup_s is not None else default_warmup
    duration = duration_s if duration_s is not None else default_duration
    drain = drain_s if drain_s is not None else default_drain
    specs = []
    ordinal = 0
    for workload, cohort, stack, environment, profile, subscribers in cells:
        for repetition in range(reps):
            specs.append(
                TrialSpec(
                    trial_id=f"t{ordinal:06d}",
                    stack=stack,
                    cohort=cohort,
                    environment=environment,
                    profile=profile,
                    workload=workload,
                    subscribers=subscribers,
                    repetition=repetition,
                    warmup_s=warmup,
                    duration_s=duration,
                    drain_s=drain,
                    seed=seed + repetition,
                )
            )
            ordinal += 1

    random.Random(seed).shuffle(specs)
    return specs
