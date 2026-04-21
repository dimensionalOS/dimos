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

"""Trajectory controller benchmark using FOPDT-based kinematic simulation.

Compares old PController vs PathFollowerTask (PurePursuit + PID) on
synthetic test paths with a realistic plant model.

Usage:
    python -m dimos.control.tests.trajectory_benchmark
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math

import numpy as np
from numpy.typing import NDArray

from dimos.control.tasks.path_controllers import PIDCrossTrackController, PurePursuitController
from dimos.control.tasks.path_distancer import PathDistancer
from dimos.control.tasks.velocity_profiler import VelocityProfiler
from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.replanning_a_star.controllers import PController
from dimos.utils.trigonometry import angle_diff

# ---------------------------------------------------------------------------
# FOPDT Plant Simulator
# ---------------------------------------------------------------------------


class FOPDTChannel:
    """First-order lag + dead-time for one velocity channel."""

    def __init__(self, K: float = 1.0, tau: float = 0.1, theta: float = 0.03) -> None:
        self.K = K
        self.tau = tau
        self.theta = theta
        self._delay_buf: deque[float] = deque()
        self._delay_samples = 0
        self._y = 0.0

    def reset(self, dt: float) -> None:
        self._delay_samples = max(1, int(self.theta / dt))
        self._delay_buf = deque([0.0] * self._delay_samples, maxlen=self._delay_samples)
        self._y = 0.0

    def step(self, u: float, dt: float) -> float:
        self._delay_buf.append(u)
        u_delayed = self._delay_buf[0]
        alpha = dt / (self.tau + dt)
        self._y += alpha * (self.K * u_delayed - self._y)
        return self._y


class Go2PlantSim:
    """Unicycle kinematic sim with FOPDT velocity response per channel."""

    def __init__(
        self,
        K_vx: float = 1.0,
        tau_vx: float = 0.1,
        theta_vx: float = 0.03,
        K_vy: float = 1.0,
        tau_vy: float = 0.1,
        theta_vy: float = 0.03,
        K_wz: float = 1.0,
        tau_wz: float = 0.05,
        theta_wz: float = 0.02,
    ) -> None:
        self.ch_vx = FOPDTChannel(K_vx, tau_vx, theta_vx)
        self.ch_vy = FOPDTChannel(K_vy, tau_vy, theta_vy)
        self.ch_wz = FOPDTChannel(K_wz, tau_wz, theta_wz)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def reset(self, x: float, y: float, yaw: float, dt: float) -> None:
        self.x, self.y, self.yaw = x, y, yaw
        for ch in (self.ch_vx, self.ch_vy, self.ch_wz):
            ch.reset(dt)

    def step(self, cmd_vx: float, cmd_vy: float, cmd_wz: float, dt: float) -> None:
        vx = self.ch_vx.step(cmd_vx, dt)
        vy = self.ch_vy.step(cmd_vy, dt)
        wz = self.ch_wz.step(cmd_wz, dt)

        self.x += (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        self.y += (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
        self.yaw = (self.yaw + wz * dt + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------


@dataclass
class BenchmarkResult:
    controller_name: str = ""
    reached_goal: bool = False
    total_time: float = 0.0
    actual_path_length: float = 0.0
    ticks: int = 0

    mean_cte: float = 0.0
    max_cte: float = 0.0
    rms_cte: float = 0.0
    mean_heading_error: float = 0.0
    mean_speed: float = 0.0
    path_length_ratio: float = 0.0
    angular_smoothness: float = 0.0

    _cte_list: list[float] = field(default_factory=list, repr=False)
    _he_list: list[float] = field(default_factory=list, repr=False)
    _speed_list: list[float] = field(default_factory=list, repr=False)
    _wz_list: list[float] = field(default_factory=list, repr=False)

    def record(self, cte: float, heading_err: float, speed: float, wz: float) -> None:
        self._cte_list.append(abs(cte))
        self._he_list.append(abs(heading_err))
        self._speed_list.append(speed)
        self._wz_list.append(wz)

    def finalise(self, planned_length: float) -> None:
        if not self._cte_list:
            return
        self.ticks = len(self._cte_list)
        self.mean_cte = float(np.mean(self._cte_list))
        self.max_cte = float(np.max(self._cte_list))
        self.rms_cte = float(np.sqrt(np.mean(np.array(self._cte_list) ** 2)))
        self.mean_heading_error = float(np.mean(self._he_list))
        self.mean_speed = float(np.mean(self._speed_list))
        if planned_length > 0:
            self.path_length_ratio = self.actual_path_length / planned_length
        w = self._wz_list
        self.angular_smoothness = sum(abs(w[i] - w[i - 1]) for i in range(1, len(w)))


# ---------------------------------------------------------------------------
# Test paths
# ---------------------------------------------------------------------------


def make_straight_path(length: float = 5.0, step: float = 0.1) -> Path:
    n = int(length / step)
    poses = []
    for i in range(n + 1):
        x = i * step
        poses.append(
            PoseStamped(
                position=Vector3(x, 0.0, 0.0),
                orientation=Quaternion.from_euler(Vector3(0.0, 0.0, 0.0)),
            )
        )
    return Path(poses=poses)


def make_s_curve_path(amplitude: float = 1.0, length: float = 6.0, step: float = 0.1) -> Path:
    n = int(length / step)
    poses = []
    for i in range(n + 1):
        x = i * step
        y = amplitude * math.sin(2 * math.pi * x / length)
        yaw = math.atan2(amplitude * 2 * math.pi / length * math.cos(2 * math.pi * x / length), 1.0)
        poses.append(
            PoseStamped(
                position=Vector3(x, y, 0.0),
                orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
            )
        )
    return Path(poses=poses)


def make_right_angle_path(leg: float = 3.0, step: float = 0.1) -> Path:
    poses = []
    n = int(leg / step)
    for i in range(n + 1):
        poses.append(
            PoseStamped(
                position=Vector3(i * step, 0.0, 0.0),
                orientation=Quaternion.from_euler(Vector3(0.0, 0.0, 0.0)),
            )
        )
    for i in range(1, n + 1):
        poses.append(
            PoseStamped(
                position=Vector3(leg, i * step, 0.0),
                orientation=Quaternion.from_euler(Vector3(0.0, 0.0, math.pi / 2)),
            )
        )
    return Path(poses=poses)


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------


def make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    return PoseStamped(
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
    )


def get_path_tangent_yaw(path_array: NDArray, index: int) -> float:
    if index < len(path_array) - 1:
        dx = path_array[index + 1][0] - path_array[index][0]
        dy = path_array[index + 1][1] - path_array[index][1]
    elif index > 0:
        dx = path_array[index][0] - path_array[index - 1][0]
        dy = path_array[index][1] - path_array[index - 1][1]
    else:
        return 0.0
    return float(np.arctan2(dy, dx))


def simulate_old_controller(
    path: Path, plant: Go2PlantSim, dt: float = 0.1, max_steps: int = 2000
) -> BenchmarkResult:
    """Simulate old PController at 10 Hz."""
    gc = GlobalConfig()
    ctrl = PController(gc, speed=0.55, control_frequency=10)
    distancer = PathDistancer(path)
    path_arr = np.array([[p.position.x, p.position.y] for p in path.poses])
    planned_len = float(distancer._cumulative_dists[-1])

    plant.reset(path.poses[0].position.x, path.poses[0].position.y, 0.0, dt)
    result = BenchmarkResult(controller_name="PController@10Hz")
    actual_dist = 0.0
    prev_x, prev_y = plant.x, plant.y

    for _ in range(max_steps):
        pose = make_pose(plant.x, plant.y, plant.yaw)
        pos = np.array([plant.x, plant.y])
        dist_to_goal = distancer.distance_to_goal(pos)

        if dist_to_goal < 0.3:
            goal_yaw = path.poses[-1].orientation.euler[2]
            if abs(angle_diff(goal_yaw, plant.yaw)) < 0.35:
                result.reached_goal = True
                break

        idx = distancer.find_closest_point_index(pos)
        lookahead = distancer.find_lookahead_point(idx)
        twist = ctrl.advance(lookahead, pose)

        cte = distancer.get_signed_cross_track_error(pos)
        tangent = get_path_tangent_yaw(path_arr, idx)
        he = angle_diff(plant.yaw, tangent)
        speed = math.sqrt(twist.linear.x**2 + twist.linear.y**2)
        result.record(cte, he, speed, twist.angular.z)

        plant.step(twist.linear.x, twist.linear.y, twist.angular.z, dt)
        step_d = math.sqrt((plant.x - prev_x) ** 2 + (plant.y - prev_y) ** 2)
        actual_dist += step_d
        prev_x, prev_y = plant.x, plant.y

    result.total_time = result.ticks * dt if result._cte_list else 0
    result.actual_path_length = actual_dist
    result.finalise(planned_len)
    return result


def simulate_new_controller(
    path: Path,
    plant: Go2PlantSim,
    dt: float = 0.1,
    max_steps: int = 2000,
    ct_kp: float = 1.5,
    ct_ki: float = 0.1,
    ct_kd: float = 0.2,
) -> BenchmarkResult:
    """Simulate PurePursuit + PID cross-track at 10 Hz."""
    gc = GlobalConfig()
    pp = PurePursuitController(gc, control_frequency=10.0, max_linear_speed=0.8)
    pid = PIDCrossTrackController(control_frequency=10.0, k_p=ct_kp, k_i=ct_ki, k_d=ct_kd)
    profiler = VelocityProfiler(max_linear_speed=0.8)
    vel_profile = profiler.compute_profile(path)
    distancer = PathDistancer(path)
    path_arr = np.array([[p.position.x, p.position.y] for p in path.poses])
    planned_len = float(distancer._cumulative_dists[-1])

    plant.reset(path.poses[0].position.x, path.poses[0].position.y, 0.0, dt)
    result = BenchmarkResult(controller_name="PurePursuit+PID@10Hz")
    actual_dist = 0.0
    prev_x, prev_y = plant.x, plant.y

    for _ in range(max_steps):
        pose = make_pose(plant.x, plant.y, plant.yaw)
        pos = np.array([plant.x, plant.y])
        dist_to_goal = distancer.distance_to_goal(pos)

        if dist_to_goal < 0.3:
            goal_yaw = path.poses[-1].orientation.euler[2]
            if abs(angle_diff(goal_yaw, plant.yaw)) < 0.35:
                result.reached_goal = True
                break

        idx = distancer.find_closest_point_index(pos)
        target_v = float(vel_profile[min(idx, len(vel_profile) - 1)])
        curvature = distancer.get_curvature_at_index(idx)
        lookahead = distancer.find_adaptive_lookahead_point(idx, target_v)

        twist = pp.advance(lookahead, pose, current_speed=target_v, path_curvature=curvature)

        cte = distancer.get_signed_cross_track_error(pos)
        correction = pid.compute_correction(cte)
        wz = float(np.clip(twist.angular.z - correction, -1.2, 1.2))

        tangent = get_path_tangent_yaw(path_arr, idx)
        he = angle_diff(plant.yaw, tangent)
        speed = math.sqrt(twist.linear.x**2 + twist.linear.y**2)
        result.record(cte, he, speed, wz)

        plant.step(twist.linear.x, twist.linear.y, wz, dt)
        step_d = math.sqrt((plant.x - prev_x) ** 2 + (plant.y - prev_y) ** 2)
        actual_dist += step_d
        prev_x, prev_y = plant.x, plant.y

    result.total_time = result.ticks * dt if result._cte_list else 0
    result.actual_path_length = actual_dist
    result.finalise(planned_len)
    return result


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------


def print_comparison(results: list[BenchmarkResult], path_name: str) -> None:
    header = f"{'Metric':<30}"
    for r in results:
        header += f" {r.controller_name:>20}"
    print(f"\n{'=' * (30 + 21 * len(results))}")
    print(f"  Path: {path_name}")
    print(f"{'=' * (30 + 21 * len(results))}")
    print(header)
    print("-" * (30 + 21 * len(results)))

    rows = [
        ("Reached goal", [str(r.reached_goal) for r in results]),
        ("Time (s)", [f"{r.total_time:.2f}" for r in results]),
        ("Mean |CTE| (m)", [f"{r.mean_cte:.4f}" for r in results]),
        ("Max |CTE| (m)", [f"{r.max_cte:.4f}" for r in results]),
        ("RMS CTE (m)", [f"{r.rms_cte:.4f}" for r in results]),
        ("Mean heading err (rad)", [f"{r.mean_heading_error:.4f}" for r in results]),
        ("Mean speed (m/s)", [f"{r.mean_speed:.3f}" for r in results]),
        ("Path length ratio", [f"{r.path_length_ratio:.3f}" for r in results]),
        ("Angular smoothness", [f"{r.angular_smoothness:.2f}" for r in results]),
    ]
    for label, vals in rows:
        line = f"{label:<30}"
        for v in vals:
            line += f" {v:>20}"
        print(line)
    print("=" * (30 + 21 * len(results)))


def main() -> None:
    plant = Go2PlantSim()  # default FOPDT params — update after plant ID

    test_paths = [
        ("Straight 5m", make_straight_path(5.0)),
        ("S-curve", make_s_curve_path(1.0, 6.0)),
        ("90-degree turn", make_right_angle_path(3.0)),
    ]

    for name, path in test_paths:
        old = simulate_old_controller(path, plant)
        new = simulate_new_controller(path, plant)
        print_comparison([old, new], name)


if __name__ == "__main__":
    main()
