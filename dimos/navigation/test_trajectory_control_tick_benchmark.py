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

"""P4-2: reproducible micro-benchmarks for trajectory control tick CPU (slow tests).

Median per-call time is checked against loose ceilings so default CI stays stable
across hosts; use ``pytest -sv`` locally to print medians and implied single-thread
headroom in Hz. See ``docs/development/921_trajectory_controller/docs/trajectory_control_tick_benchmark.md``.
"""

from __future__ import annotations

import os
import statistics
import time
from collections.abc import Callable

import numpy as np
import pytest

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.replanning_a_star.controllers import (
    HolonomicPathController,
    _pose_from_pose_stamped,
)
from dimos.navigation.trajectory_control_tick_log import trajectory_control_tick_from_samples
from dimos.navigation.trajectory_types import TrajectoryMeasuredSample, TrajectoryReferenceSample

_WARMUP = 2000
_ITERS = 30_000
# Loose ceilings (nanoseconds) so shared CI VMs do not flake; see benchmark doc.
_MAX_MEDIAN_HOLONOMIC_ADVANCE_NS = 15_000_000
_MAX_MEDIAN_TICK_FROM_SAMPLES_NS = 2_000_000


def _median_call_ns(warmup: int, iterations: int, fn: Callable[[], None]) -> float:
    for _ in range(warmup):
        fn()
    samples: list[int] = []
    for _ in range(iterations):
        t0 = time.perf_counter_ns()
        fn()
        samples.append(time.perf_counter_ns() - t0)
    return float(statistics.median(samples))


def _maybe_report(name: str, median_ns: float) -> None:
    if os.environ.get("DIMOS_TRAJECTORY_BENCH_VERBOSE", "").strip().lower() not in (
        "1",
        "true",
        "yes",
        "on",
    ):
        return
    us = median_ns / 1000.0
    hz = 1e9 / median_ns if median_ns > 0 else float("nan")
    print(f"{name}: median {median_ns:.0f} ns ({us:.1f} us), implied single-thread ~{hz:.0f} Hz")


@pytest.mark.slow
def test_holonomic_path_controller_advance_median_under_budget() -> None:
    g = GlobalConfig(local_planner_path_controller="holonomic", simulation=False)
    ctrl = HolonomicPathController(g, 0.55, 20.0, 2.0, 1.5)
    odom = PoseStamped(
        frame_id="map", position=[0.0, 0.0, 0.0], orientation=Quaternion(0, 0, 0, 1)
    )
    lookahead = np.array([0.4, 0.0], dtype=np.float64)

    def one() -> None:
        ctrl.advance(lookahead, odom)

    med = _median_call_ns(_WARMUP, _ITERS, one)
    _maybe_report("HolonomicPathController.advance", med)
    assert med < _MAX_MEDIAN_HOLONOMIC_ADVANCE_NS


@pytest.mark.slow
def test_trajectory_control_tick_from_samples_median_under_budget() -> None:
    odom = PoseStamped(
        frame_id="map", position=[0.1, 0.05, 0.0], orientation=Quaternion(0, 0, 0, 1)
    )
    ref_yaw = 0.2
    ref_pose = Pose(
        position=Vector3(0.4, 0.0, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, ref_yaw)),
    )
    ref = TrajectoryReferenceSample(
        0.0,
        ref_pose,
        Twist(linear=Vector3(0.55, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)),
    )
    meas = TrajectoryMeasuredSample(0.0, _pose_from_pose_stamped(odom), Twist())
    cmd = Twist(linear=Vector3(0.2, -0.05, 0.0), angular=Vector3(0.0, 0.0, 0.1))

    def one() -> None:
        trajectory_control_tick_from_samples(ref, meas, cmd, 0.05)

    med = _median_call_ns(_WARMUP, _ITERS, one)
    _maybe_report("trajectory_control_tick_from_samples", med)
    assert med < _MAX_MEDIAN_TICK_FROM_SAMPLES_NS
