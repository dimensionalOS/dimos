#!/usr/bin/env python3
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

"""Head-to-head benchmark: PurePursuit+PID vs Lyapunov reactive controller.

Tests both controllers on identical paths with realistic plant dynamics
(FOPDT-based Go2 simulation) and compares tracking metrics.

Includes perturbation scenarios (lateral offset, heading error) to test
recovery behaviour — the key weakness of the old precomputed approach.

Usage:
    .venv/bin/python -m dimos.control.tests.controller_comparison_benchmark
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np
from numpy.typing import NDArray

from dimos.control.tasks.lyapunov_path_controller import (
    LyapunovPathController,
    LyapunovPathControllerConfig,
)
from dimos.control.tasks.path_controllers import PIDCrossTrackController, PurePursuitController
from dimos.control.tasks.path_distancer import PathDistancer
from dimos.control.tasks.velocity_profiler import VelocityProfiler
from dimos.control.tests.trajectory_benchmark import Go2PlantSim, make_pose
from dimos.core.global_config import global_config
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.trigonometry import angle_diff

# ---------------------------------------------------------------------------
# Test paths
# ---------------------------------------------------------------------------


def _straight(length: float = 5.0, step: float = 0.1) -> Path:
    n = int(length / step) + 1
    return Path(
        poses=[
            PoseStamped(
                position=Vector3(i * step, 0, 0),
                orientation=Quaternion.from_euler(Vector3(0, 0, 0)),
            )
            for i in range(n)
        ]
    )


def _s_curve(amp: float = 1.0, length: float = 6.0, step: float = 0.1) -> Path:
    n = int(length / step) + 1
    poses = []
    for i in range(n):
        x = i * step
        y = amp * math.sin(2 * math.pi * x / length)
        yaw = math.atan2(amp * 2 * math.pi / length * math.cos(2 * math.pi * x / length), 1.0)
        poses.append(
            PoseStamped(
                position=Vector3(x, y, 0), orientation=Quaternion.from_euler(Vector3(0, 0, yaw))
            )
        )
    return Path(poses=poses)


def _right_angle(leg: float = 3.0, step: float = 0.1) -> Path:
    n = int(leg / step)
    poses = [
        PoseStamped(
            position=Vector3(i * step, 0, 0), orientation=Quaternion.from_euler(Vector3(0, 0, 0))
        )
        for i in range(n + 1)
    ]
    poses += [
        PoseStamped(
            position=Vector3(leg, i * step, 0),
            orientation=Quaternion.from_euler(Vector3(0, 0, math.pi / 2)),
        )
        for i in range(1, n + 1)
    ]
    return Path(poses=poses)


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------


@dataclass
class RunResult:
    name: str = ""
    reached_goal: bool = False
    total_time: float = 0.0
    path_length: float = 0.0

    cte_list: list[float] = field(default_factory=list, repr=False)
    he_list: list[float] = field(default_factory=list, repr=False)
    speed_list: list[float] = field(default_factory=list, repr=False)
    wz_list: list[float] = field(default_factory=list, repr=False)
    vy_list: list[float] = field(default_factory=list, repr=False)

    @property
    def mean_cte(self) -> float:
        return float(np.mean(np.abs(self.cte_list))) if self.cte_list else 0

    @property
    def max_cte(self) -> float:
        return float(np.max(np.abs(self.cte_list))) if self.cte_list else 0

    @property
    def rms_cte(self) -> float:
        return float(np.sqrt(np.mean(np.array(self.cte_list) ** 2))) if self.cte_list else 0

    @property
    def mean_speed(self) -> float:
        return float(np.mean(self.speed_list)) if self.speed_list else 0

    @property
    def max_speed_during_recovery(self) -> float:
        """Max speed in first 20 ticks (recovery phase)."""
        return float(np.max(self.speed_list[:20])) if len(self.speed_list) >= 20 else 0

    @property
    def angular_smoothness(self) -> float:
        w = self.wz_list
        return sum(abs(w[i] - w[i - 1]) for i in range(1, len(w))) if len(w) > 1 else 0

    @property
    def mean_abs_vy(self) -> float:
        return float(np.mean(np.abs(self.vy_list))) if self.vy_list else 0


# ---------------------------------------------------------------------------
# Simulation runners
# ---------------------------------------------------------------------------


def _tangent_yaw(path_arr: NDArray, idx: int) -> float:
    if idx < len(path_arr) - 1:
        d = path_arr[idx + 1] - path_arr[idx]
    elif idx > 0:
        d = path_arr[idx] - path_arr[idx - 1]
    else:
        return 0.0
    return float(np.arctan2(d[1], d[0]))


def run_pure_pursuit(
    path: Path,
    plant: Go2PlantSim,
    dt: float = 0.1,
    max_steps: int = 2000,
    start_x: float | None = None,
    start_y: float | None = None,
    start_yaw: float | None = None,
) -> RunResult:
    pp = PurePursuitController(global_config, control_frequency=10.0, max_linear_speed=0.8)
    pid = PIDCrossTrackController(control_frequency=10.0, k_p=1.5, k_i=0.1, k_d=0.2)
    profiler = VelocityProfiler(max_linear_speed=0.8)
    vel_profile = profiler.compute_profile(path)
    distancer = PathDistancer(path)
    path_arr = np.array([[p.position.x, p.position.y] for p in path.poses])

    sx = start_x if start_x is not None else path.poses[0].position.x
    sy = start_y if start_y is not None else path.poses[0].position.y
    syaw = start_yaw if start_yaw is not None else 0.0
    plant.reset(sx, sy, syaw, dt)

    result = RunResult(name="PurePursuit+PID")
    dist = 0.0
    px, py = plant.x, plant.y

    for _ in range(max_steps):
        pose = make_pose(plant.x, plant.y, plant.yaw)
        pos = np.array([plant.x, plant.y])
        d2g = distancer.distance_to_goal(pos)

        if d2g < 0.3 and abs(angle_diff(path.poses[-1].orientation.euler[2], plant.yaw)) < 0.35:
            result.reached_goal = True
            break

        idx = distancer.find_closest_point_index(pos)
        tv = float(vel_profile[min(idx, len(vel_profile) - 1)])
        la = distancer.find_adaptive_lookahead_point(idx, tv)
        curv = distancer.get_curvature_at_index(idx)
        twist = pp.advance(la, pose, current_speed=tv, path_curvature=curv)
        cte = distancer.get_signed_cross_track_error(pos)
        corr = pid.compute_correction(cte)
        wz = float(np.clip(twist.angular.z - corr, -1.2, 1.2))
        vx, vy = float(twist.linear.x), 0.0

        he = angle_diff(plant.yaw, _tangent_yaw(path_arr, idx))
        speed = math.sqrt(vx**2 + vy**2)
        result.cte_list.append(cte)
        result.he_list.append(he)
        result.speed_list.append(speed)
        result.wz_list.append(wz)
        result.vy_list.append(vy)

        plant.step(vx, vy, wz, dt)
        dist += math.sqrt((plant.x - px) ** 2 + (plant.y - py) ** 2)
        px, py = plant.x, plant.y

    result.total_time = len(result.cte_list) * dt
    result.path_length = dist
    return result


def run_lyapunov(
    path: Path,
    plant: Go2PlantSim,
    dt: float = 0.1,
    max_steps: int = 2000,
    start_x: float | None = None,
    start_y: float | None = None,
    start_yaw: float | None = None,
) -> RunResult:
    ctrl = LyapunovPathController(LyapunovPathControllerConfig(v_max=0.6))
    distancer = PathDistancer(path)
    path_arr = np.array([[p.position.x, p.position.y] for p in path.poses])

    sx = start_x if start_x is not None else path.poses[0].position.x
    sy = start_y if start_y is not None else path.poses[0].position.y
    syaw = start_yaw if start_yaw is not None else 0.0
    plant.reset(sx, sy, syaw, dt)

    result = RunResult(name="Lyapunov")
    dist = 0.0
    px, py = plant.x, plant.y

    for _ in range(max_steps):
        pose = make_pose(plant.x, plant.y, plant.yaw)
        pos = np.array([plant.x, plant.y])
        d2g = distancer.distance_to_goal(pos)

        if d2g < 0.3 and abs(angle_diff(path.poses[-1].orientation.euler[2], plant.yaw)) < 0.35:
            result.reached_goal = True
            break

        out = ctrl.compute(pose, distancer, distance_to_goal=d2g)
        idx = distancer.find_closest_point_index(pos)
        he = angle_diff(plant.yaw, _tangent_yaw(path_arr, idx))
        cte = distancer.get_signed_cross_track_error(pos)
        speed = math.sqrt(out.vx**2 + out.vy**2)

        result.cte_list.append(cte)
        result.he_list.append(he)
        result.speed_list.append(speed)
        result.wz_list.append(out.wz)
        result.vy_list.append(out.vy)

        plant.step(out.vx, out.vy, out.wz, dt)
        dist += math.sqrt((plant.x - px) ** 2 + (plant.y - py) ** 2)
        px, py = plant.x, plant.y

    result.total_time = len(result.cte_list) * dt
    result.path_length = dist
    return result


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------


def _print_comparison(pp: RunResult, ly: RunResult, scenario: str) -> None:
    w = 20
    print(f"\n{'=' * 70}")
    print(f"  {scenario}")
    print(f"{'=' * 70}")
    print(f"{'Metric':<30} {'PurePursuit+PID':>{w}} {'Lyapunov':>{w}}")
    print(f"{'-' * 70}")
    rows = [
        ("Reached goal", str(pp.reached_goal), str(ly.reached_goal)),
        ("Time (s)", f"{pp.total_time:.1f}", f"{ly.total_time:.1f}"),
        ("Mean |CTE| (m)", f"{pp.mean_cte:.4f}", f"{ly.mean_cte:.4f}"),
        ("Max |CTE| (m)", f"{pp.max_cte:.4f}", f"{ly.max_cte:.4f}"),
        ("RMS CTE (m)", f"{pp.rms_cte:.4f}", f"{ly.rms_cte:.4f}"),
        ("Mean speed (m/s)", f"{pp.mean_speed:.3f}", f"{ly.mean_speed:.3f}"),
        (
            "Max speed (recovery)",
            f"{pp.max_speed_during_recovery:.3f}",
            f"{ly.max_speed_during_recovery:.3f}",
        ),
        ("Angular smoothness", f"{pp.angular_smoothness:.2f}", f"{ly.angular_smoothness:.2f}"),
        ("Mean |vy| (m/s)", f"{pp.mean_abs_vy:.4f}", f"{ly.mean_abs_vy:.4f}"),
    ]
    for label, v1, v2 in rows:
        print(f"{label:<30} {v1:>{w}} {v2:>{w}}")
    print(f"{'=' * 70}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    plant = Go2PlantSim()

    scenarios: list[tuple[str, Path, dict]] = [
        ("Straight 5m (on path)", _straight(5.0), {}),
        ("S-curve (on path)", _s_curve(), {}),
        ("90° turn (on path)", _right_angle(), {}),
        ("Straight — 0.5m lateral offset", _straight(5.0), {"start_y": 0.5}),
        ("Straight — 45° heading error", _straight(5.0), {"start_yaw": math.pi / 4}),
        (
            "S-curve — 0.3m offset + 30° heading",
            _s_curve(),
            {"start_y": 0.3, "start_yaw": math.pi / 6},
        ),
    ]

    for name, path, kwargs in scenarios:
        pp = run_pure_pursuit(path, plant, **kwargs)
        ly = run_lyapunov(path, plant, **kwargs)
        _print_comparison(pp, ly, name)


if __name__ == "__main__":
    main()
