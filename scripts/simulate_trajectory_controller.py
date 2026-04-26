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

"""Run a reusable Level 1 trajectory controller plus plant simulation.

The runner emits the issue 921 artifact bundle:

- ``ticks.jsonl`` using the stable trajectory control tick schema.
- ``config.yaml`` with the complete effective run configuration.
- ``summary.json`` with metrics, gate results, and verdict.
- ``plot.png`` when plotting dependencies are available.
"""

from __future__ import annotations

import argparse
from collections import deque
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from datetime import UTC, datetime
import hashlib
from itertools import pairwise
import json
import math
from pathlib import Path
import random
import subprocess
import sys
import time
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.trajectory_command_limits import (
    HolonomicCommandLimits,
    clamp_holonomic_cmd_vel,
)
from dimos.navigation.trajectory_control_tick_export import write_trajectory_control_ticks_jsonl
from dimos.navigation.trajectory_control_tick_log import (
    TrajectoryControlTick,
    trajectory_control_tick_from_samples,
)
from dimos.navigation.trajectory_holonomic_tracking_controller import HolonomicTrackingController
from dimos.navigation.trajectory_response_curve_presets import (
    RESPONSE_CURVE_PRESETS,
    ResponseCurveAxisConfig,
    ResponseCurvePreset,
    get_response_curve_preset,
)
from dimos.navigation.trajectory_types import TrajectoryMeasuredSample, TrajectoryReferenceSample
from dimos.utils.trigonometry import angle_diff

CONFIG_SCHEMA_VERSION = 1
SUMMARY_SCHEMA_VERSION = 1
DEFAULT_GOAL_POSITION_TOLERANCE_M = 0.15
DEFAULT_GOAL_YAW_TOLERANCE_RAD = 0.2
DEFAULT_SETTLE_MARGIN_S = 5.0

PLANT_PRESET_ALIASES = {
    "nominal": "synthetic_nominal",
    "sluggish": "synthetic_sluggish",
    "asymmetric": "synthetic_asymmetric",
    "noisy": "synthetic_noisy",
}


@dataclass(frozen=True)
class PlanarPose:
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class Scenario:
    name: str
    target_speed_m_s: float
    duration_s: float
    initial_pose: PlanarPose
    goal_pose: PlanarPose
    path_config: dict[str, Any]
    sample: Callable[[float], TrajectoryReferenceSample]


@dataclass(frozen=True)
class GateConfig:
    max_planar_position_divergence_m: float
    max_final_position_error_m: float
    max_final_heading_error_rad: float
    max_settle_time_s: float | None
    require_arrival: bool
    require_no_timeout: bool
    require_finite_metrics: bool


@dataclass(frozen=True)
class PlotResult:
    produced: bool
    path: Path
    sha256: str | None
    command: list[str]
    returncode: int
    stderr: str | None


def _pose_xy_yaw(x_m: float, y_m: float, yaw_rad: float) -> Pose:
    return Pose(
        x_m,
        y_m,
        0.0,
        0.0,
        0.0,
        math.sin(yaw_rad / 2.0),
        math.cos(yaw_rad / 2.0),
    )


def _twist(vx_m_s: float, vy_m_s: float = 0.0, wz_rad_s: float = 0.0) -> Twist:
    return Twist(linear=Vector3(vx_m_s, vy_m_s, 0.0), angular=Vector3(0.0, 0.0, wz_rad_s))


def _reference_sample(time_s: float, pose: PlanarPose, twist: Twist) -> TrajectoryReferenceSample:
    return TrajectoryReferenceSample(time_s=time_s, pose_plan=_pose_xy_yaw(pose.x_m, pose.y_m, pose.yaw_rad), twist_body=twist)


def _measured_sample(time_s: float, pose: PlanarPose, twist: Twist) -> TrajectoryMeasuredSample:
    return TrajectoryMeasuredSample(time_s=time_s, pose_plan=_pose_xy_yaw(pose.x_m, pose.y_m, pose.yaw_rad), twist_body=twist)


def _polyline_lengths(points: Sequence[tuple[float, float]]) -> tuple[list[float], float]:
    lengths = [0.0]
    total = 0.0
    for a, b in pairwise(points):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        total += seg
        lengths.append(total)
    return lengths, total


def _sample_polyline_pose(
    points: Sequence[tuple[float, float]],
    cumulative_lengths_m: Sequence[float],
    distance_m: float,
) -> PlanarPose:
    distance_m = max(0.0, min(float(distance_m), cumulative_lengths_m[-1]))
    if distance_m >= cumulative_lengths_m[-1]:
        i = len(points) - 2
    else:
        i = 0
        while i + 1 < len(cumulative_lengths_m) and cumulative_lengths_m[i + 1] < distance_m:
            i += 1
    x0, y0 = points[i]
    x1, y1 = points[i + 1]
    seg_len = cumulative_lengths_m[i + 1] - cumulative_lengths_m[i]
    u = 0.0 if seg_len <= 1e-12 else (distance_m - cumulative_lengths_m[i]) / seg_len
    x = x0 + (x1 - x0) * u
    y = y0 + (y1 - y0) * u
    yaw = math.atan2(y1 - y0, x1 - x0)
    return PlanarPose(x, y, yaw)


def _polyline_scenario(
    *,
    name: str,
    speed_m_s: float,
    points: Sequence[tuple[float, float]],
    path_config: dict[str, Any],
    settle_margin_s: float = DEFAULT_SETTLE_MARGIN_S,
) -> Scenario:
    if len(points) < 2:
        raise ValueError("polyline scenarios require at least two points")
    cumulative, total = _polyline_lengths(points)
    travel_s = total / speed_m_s
    duration_s = travel_s + settle_margin_s
    initial = _sample_polyline_pose(points, cumulative, 0.0)
    goal = _sample_polyline_pose(points, cumulative, total)

    def sample(t: float) -> TrajectoryReferenceSample:
        distance = min(max(t, 0.0) * speed_m_s, total)
        pose = _sample_polyline_pose(points, cumulative, distance)
        v = speed_m_s if distance < total else 0.0
        return _reference_sample(t, pose, _twist(v))

    return Scenario(
        name=name,
        target_speed_m_s=speed_m_s,
        duration_s=duration_s,
        initial_pose=initial,
        goal_pose=goal,
        path_config=path_config | {"waypoints": [{"x_m": x, "y_m": y} for x, y in points]},
        sample=sample,
    )


def _make_s_curve_points(length_m: float, amplitude_m: float, samples: int) -> list[tuple[float, float]]:
    return [
        (
            length_m * i / (samples - 1),
            amplitude_m * math.sin(2.0 * math.pi * i / (samples - 1)),
        )
        for i in range(samples)
    ]


def build_scenario(name: str, speed_m_s: float) -> Scenario:
    if not (math.isfinite(speed_m_s) and speed_m_s > 0.0):
        raise ValueError("speed must be a positive finite float")

    if name == "line":
        length = 8.0
        return _polyline_scenario(
            name=name,
            speed_m_s=speed_m_s,
            points=[(0.0, 0.0), (length, 0.0)],
            path_config={"length_m": length},
        )
    if name == "s_curve":
        length = 8.0
        amplitude = 1.0
        samples = 121
        return _polyline_scenario(
            name=name,
            speed_m_s=speed_m_s,
            points=_make_s_curve_points(length, amplitude, samples),
            path_config={"length_m": length, "lateral_amplitude_m": amplitude, "samples": samples},
        )
    if name == "right_angle_turn":
        leg = 4.0
        return _polyline_scenario(
            name=name,
            speed_m_s=speed_m_s,
            points=[(0.0, 0.0), (leg, 0.0), (leg, leg)],
            path_config={"leg_length_m": leg, "turn_angle_rad": math.pi / 2.0},
        )
    if name == "stop_at_goal":
        length = 3.0
        return _polyline_scenario(
            name=name,
            speed_m_s=speed_m_s,
            points=[(0.0, 0.0), (length, 0.0)],
            path_config={"length_m": length, "dwell_at_goal_s": DEFAULT_SETTLE_MARGIN_S},
        )
    if name == "circle":
        radius = 1.5
        omega = speed_m_s / radius
        travel_s = 2.0 * math.pi / omega
        duration_s = travel_s + DEFAULT_SETTLE_MARGIN_S
        initial = PlanarPose(radius, 0.0, math.pi / 2.0)
        goal = initial

        def sample(t: float) -> TrajectoryReferenceSample:
            active_t = min(max(t, 0.0), travel_s)
            phi = omega * active_t
            pose = PlanarPose(
                radius * math.cos(phi),
                radius * math.sin(phi),
                math.atan2(math.cos(phi), -math.sin(phi)),
            )
            twist = _twist(speed_m_s, wz_rad_s=omega) if t < travel_s else _twist(0.0)
            return _reference_sample(t, pose, twist)

        return Scenario(
            name=name,
            target_speed_m_s=speed_m_s,
            duration_s=duration_s,
            initial_pose=initial,
            goal_pose=goal,
            path_config={"radius_m": radius, "turn_angle_rad": 2.0 * math.pi},
            sample=sample,
        )

    raise ValueError(f"unknown scenario {name!r}")


class ResponseCurvePlant:
    """Small response-curve plant that supports asymmetric curves and fixed latency."""

    def __init__(
        self,
        *,
        preset: ResponseCurvePreset,
        initial_pose: PlanarPose,
        noise_rng: random.Random | None,
    ) -> None:
        self._preset = preset
        self._x = initial_pose.x_m
        self._y = initial_pose.y_m
        self._yaw = initial_pose.yaw_rad
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._time_s = 0.0
        self._noise_rng = noise_rng
        self._delayed = {"linear_x": 0.0, "linear_y": 0.0, "yaw": 0.0}
        self._queues: dict[str, deque[tuple[float, float]]] = {
            "linear_x": deque(),
            "linear_y": deque(),
            "yaw": deque(),
        }

    @property
    def pose(self) -> PlanarPose:
        return PlanarPose(self._x, self._y, self._yaw)

    @property
    def twist_body_velocity(self) -> Twist:
        return _twist(self._vx, self._vy, self._wz)

    def measured_sample(self, time_s: float) -> TrajectoryMeasuredSample:
        return _measured_sample(time_s, self.pose, self.twist_body_velocity)

    def _select_curve(self, axis: str, command: float, current_velocity: float) -> ResponseCurveAxisConfig:
        curves = [c for c in self._preset.curves if c.axis == axis]
        selector = command if command != 0.0 else current_velocity
        if selector > 0.0:
            preferred = [c for c in curves if c.direction == "positive"]
        elif selector < 0.0:
            preferred = [c for c in curves if c.direction == "negative"]
        else:
            preferred = [c for c in curves if c.direction == "positive"]
        bidirectional = [c for c in curves if c.direction == "bidirectional"]
        matches = preferred or bidirectional
        if not matches:
            matches = curves
        if len(matches) != 1:
            raise ValueError(f"preset {self._preset.name!r} has ambiguous {axis} curves")
        return matches[0]

    def _condition_command(
        self, axis: str, command: float, current_velocity: float
    ) -> tuple[float, ResponseCurveAxisConfig]:
        curve = self._select_curve(axis, command, current_velocity)
        value = command
        if abs(value) < curve.deadband:
            value = 0.0
        else:
            value = math.copysign(abs(value) - curve.deadband, value)
        value *= curve.command_gain
        value = max(-curve.saturation, min(curve.saturation, value))
        if self._noise_rng is not None and curve.noise_max > 0.0:
            value += self._noise_rng.uniform(-curve.noise_max, curve.noise_max)
        value = max(-curve.saturation, min(curve.saturation, value))
        return value, curve

    def _delayed_command(self, axis: str, value: float, latency_s: float) -> float:
        q = self._queues[axis]
        q.append((self._time_s + latency_s, value))
        while q and q[0][0] <= self._time_s + 1e-12:
            _, self._delayed[axis] = q.popleft()
        return self._delayed[axis]

    @staticmethod
    def _advance_velocity(current: float, target: float, curve: ResponseCurveAxisConfig, dt_s: float) -> float:
        if curve.tau_s > 0.0:
            delta = (dt_s / curve.tau_s) * (target - current)
        else:
            delta = target - current
        max_delta = curve.max_accel * dt_s
        delta = max(-max_delta, min(max_delta, delta))
        return current + delta

    def step(self, command: Twist, dt_s: float) -> None:
        if not (math.isfinite(dt_s) and dt_s > 0.0):
            raise ValueError("dt_s must be positive and finite")

        ux, curve_x = self._condition_command("linear_x", float(command.linear.x), self._vx)
        uy, curve_y = self._condition_command("linear_y", float(command.linear.y), self._vy)
        uw, curve_w = self._condition_command("yaw", float(command.angular.z), self._wz)
        ux = self._delayed_command("linear_x", ux, curve_x.latency_s)
        uy = self._delayed_command("linear_y", uy, curve_y.latency_s)
        uw = self._delayed_command("yaw", uw, curve_w.latency_s)

        self._vx = self._advance_velocity(self._vx, ux, curve_x, dt_s)
        self._vy = self._advance_velocity(self._vy, uy, curve_y, dt_s)
        self._wz = self._advance_velocity(self._wz, uw, curve_w, dt_s)

        c = math.cos(self._yaw)
        s = math.sin(self._yaw)
        self._x += (c * self._vx - s * self._vy) * dt_s
        self._y += (s * self._vx + c * self._vy) * dt_s
        self._yaw += self._wz * dt_s
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))
        self._time_s += dt_s


def _resolve_preset(name: str) -> ResponseCurvePreset:
    canonical = PLANT_PRESET_ALIASES.get(name, name)
    return get_response_curve_preset(canonical)


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _git_value(args: Sequence[str], default: str) -> str:
    try:
        result = subprocess.run(
            ["git", *args],
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return default
    return result.stdout.strip() or default


def _git_dirty() -> bool:
    status = _git_value(["status", "--porcelain"], "")
    return bool(status)


def _json_safe_dump(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=False, allow_nan=False) + "\n", encoding="utf-8")


def _finite_tree(value: object) -> bool:
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, dict):
        return all(_finite_tree(v) for v in value.values())
    if isinstance(value, list | tuple):
        return all(_finite_tree(v) for v in value)
    return True


def _percentile(values: Sequence[float], percentile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    pos = (len(ordered) - 1) * percentile
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return ordered[int(pos)]
    frac = pos - lo
    return ordered[lo] * (1.0 - frac) + ordered[hi] * frac


def _first_settle_time(
    ticks: Sequence[TrajectoryControlTick],
    goal: PlanarPose,
    *,
    position_tolerance_m: float,
    yaw_tolerance_rad: float,
) -> float | None:
    ok_suffix = True
    first: float | None = None
    for tick in reversed(ticks):
        pos_err = math.hypot(tick.meas_x_m - goal.x_m, tick.meas_y_m - goal.y_m)
        yaw_err = abs(angle_diff(tick.meas_yaw_rad, goal.yaw_rad))
        ok_suffix = ok_suffix and pos_err <= position_tolerance_m and yaw_err <= yaw_tolerance_rad
        if ok_suffix:
            first = tick.sim_time_s
    return first


def _gate(threshold: float | bool | None, actual: float | bool | None, passed: bool) -> dict[str, Any]:
    return {"threshold": threshold, "actual": actual, "passed": passed}


def _compute_metrics(
    ticks: Sequence[TrajectoryControlTick],
    final_pose: PlanarPose,
    scenario: Scenario,
    gate_config: GateConfig,
) -> dict[str, Any]:
    if not ticks:
        raise ValueError("simulation produced no ticks")
    divergences = [t.planar_position_divergence_m for t in ticks]
    cross_track = [abs(t.e_cross_track_m) for t in ticks]
    heading = [abs(t.e_heading_rad) for t in ticks]
    speeds = [t.commanded_planar_speed_m_s for t in ticks]
    final_position_error = math.hypot(
        final_pose.x_m - scenario.goal_pose.x_m,
        final_pose.y_m - scenario.goal_pose.y_m,
    )
    final_heading_error = abs(angle_diff(final_pose.yaw_rad, scenario.goal_pose.yaw_rad))
    arrived = (
        final_position_error <= DEFAULT_GOAL_POSITION_TOLERANCE_M
        and final_heading_error <= DEFAULT_GOAL_YAW_TOLERANCE_RAD
    )
    settle_time_s = _first_settle_time(
        ticks,
        scenario.goal_pose,
        position_tolerance_m=DEFAULT_GOAL_POSITION_TOLERANCE_M,
        yaw_tolerance_rad=DEFAULT_GOAL_YAW_TOLERANCE_RAD,
    )
    return {
        "arrived": arrived,
        "timed_out": not arrived,
        "final_position_error_m": final_position_error,
        "final_heading_error_rad": final_heading_error,
        "max_planar_position_divergence_m": max(divergences),
        "mean_planar_position_divergence_m": sum(divergences) / len(divergences),
        "p95_planar_position_divergence_m": _percentile(divergences, 0.95),
        "max_cross_track_error_m": max(cross_track),
        "max_heading_error_rad": max(heading),
        "max_commanded_planar_speed_m_s": max(speeds),
        "settle_time_s": settle_time_s,
    }


def _build_gates(metrics: dict[str, Any], gate_config: GateConfig) -> dict[str, Any]:
    finite = _finite_tree(metrics)
    settle_threshold = gate_config.max_settle_time_s
    settle_actual = metrics["settle_time_s"]
    settle_passed = settle_threshold is None or (
        settle_actual is not None and settle_actual <= settle_threshold
    )
    return {
        "max_planar_position_divergence_m": _gate(
            gate_config.max_planar_position_divergence_m,
            metrics["max_planar_position_divergence_m"],
            metrics["max_planar_position_divergence_m"] <= gate_config.max_planar_position_divergence_m,
        ),
        "max_final_position_error_m": _gate(
            gate_config.max_final_position_error_m,
            metrics["final_position_error_m"],
            metrics["final_position_error_m"] <= gate_config.max_final_position_error_m,
        ),
        "max_final_heading_error_rad": _gate(
            gate_config.max_final_heading_error_rad,
            metrics["final_heading_error_rad"],
            metrics["final_heading_error_rad"] <= gate_config.max_final_heading_error_rad,
        ),
        "max_settle_time_s": _gate(settle_threshold, settle_actual, settle_passed),
        "require_arrival": _gate(True, metrics["arrived"], (not gate_config.require_arrival) or metrics["arrived"]),
        "require_no_timeout": _gate(
            True,
            not metrics["timed_out"],
            (not gate_config.require_no_timeout) or not metrics["timed_out"],
        ),
        "require_finite_metrics": _gate(True, finite, (not gate_config.require_finite_metrics) or finite),
    }


def _plot_ticks(ticks_path: Path, plot_path: Path, enabled: bool) -> PlotResult | None:
    if not enabled:
        return None
    script = REPO_ROOT / "scripts" / "plot_trajectory_control_ticks.py"
    command = [sys.executable, str(script), str(ticks_path), "-o", str(plot_path)]
    try:
        result = subprocess.run(command, cwd=REPO_ROOT, capture_output=True, text=True, check=False)
    except OSError as exc:
        return PlotResult(False, plot_path, None, command, 127, str(exc))
    produced = result.returncode == 0 and plot_path.is_file()
    return PlotResult(
        produced=produced,
        path=plot_path,
        sha256=_sha256(plot_path) if produced else None,
        command=command,
        returncode=result.returncode,
        stderr=(result.stderr.strip() or None),
    )


def _run_id(plant: str, scenario: str, speed: float, rate: float, seed: int | None) -> str:
    speed_token = str(speed).replace(".", "p")
    rate_token = str(rate).replace(".", "p")
    return f"{plant}_{scenario}_{speed_token}mps_{rate_token}hz_seed{seed}"


def _scenario_config(scenario: Scenario) -> dict[str, Any]:
    return {
        "name": scenario.name,
        "variant": None,
        "target_speed_m_s": scenario.target_speed_m_s,
        "duration_s": scenario.duration_s,
        "initial_pose": {
            "x_m": scenario.initial_pose.x_m,
            "y_m": scenario.initial_pose.y_m,
            "yaw_rad": scenario.initial_pose.yaw_rad,
        },
        "goal_tolerance": {
            "position_m": DEFAULT_GOAL_POSITION_TOLERANCE_M,
            "yaw_rad": DEFAULT_GOAL_YAW_TOLERANCE_RAD,
        },
        "path": scenario.path_config,
    }


def _limits_config(limits: HolonomicCommandLimits) -> dict[str, Any]:
    return {
        "max_linear_speed_m_s": limits.max_planar_speed_m_s,
        "max_angular_speed_rad_s": limits.max_yaw_rate_rad_s,
        "max_planar_accel_m_s2": limits.max_planar_linear_accel_m_s2,
        "max_yaw_accel_rad_s2": limits.max_yaw_accel_rad_s2,
        "max_normal_accel_m_s2": None,
        "goal_tolerance_position_m": DEFAULT_GOAL_POSITION_TOLERANCE_M,
        "goal_tolerance_yaw_rad": DEFAULT_GOAL_YAW_TOLERANCE_RAD,
    }


def _simulate(
    *,
    scenario: Scenario,
    preset: ResponseCurvePreset,
    limits: HolonomicCommandLimits,
    rate_hz: float,
    k_position: float,
    k_yaw: float,
    seed: int | None,
) -> tuple[list[TrajectoryControlTick], PlanarPose]:
    dt_s = 1.0 / rate_hz
    max_ticks = math.ceil(scenario.duration_s / dt_s)
    noise_enabled = any(c.noise_max > 0.0 for c in preset.curves)
    rng = random.Random(seed) if noise_enabled else None
    plant = ResponseCurvePlant(preset=preset, initial_pose=scenario.initial_pose, noise_rng=rng)
    controller = HolonomicTrackingController(k_position_per_s=k_position, k_yaw_per_s=k_yaw)
    controller.configure(limits)
    controller.reset()
    prev_cmd = _twist(0.0)
    ticks: list[TrajectoryControlTick] = []

    for i in range(max_ticks):
        t = i * dt_s
        ref = scenario.sample(t)
        meas = plant.measured_sample(t)
        raw_cmd = controller.control(ref, meas)
        cmd = clamp_holonomic_cmd_vel(prev_cmd, raw_cmd, limits, dt_s)
        ticks.append(
            trajectory_control_tick_from_samples(
                ref,
                meas,
                cmd,
                dt_s,
                wall_time_s=None,
                sim_time_s=t,
            )
        )
        plant.step(cmd, dt_s)
        prev_cmd = cmd

    return ticks, plant.pose


def _write_config(path: Path, config: dict[str, Any]) -> None:
    path.write_text(yaml.safe_dump(config, sort_keys=False, allow_unicode=False), encoding="utf-8")


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Simulate the holonomic trajectory controller against response-curve plant presets."
    )
    parser.add_argument("--scenario", choices=["line", "circle", "s_curve", "right_angle_turn", "stop_at_goal"], default="s_curve")
    parser.add_argument("--speed", type=float, default=1.5, help="Target planar path speed in m/s.")
    parser.add_argument("--rate", type=float, default=10.0, help="Control loop rate in Hz.")
    parser.add_argument("--k-position", type=float, default=2.2, help="Position gain in 1/s.")
    parser.add_argument("--k-yaw", type=float, default=2.5, help="Yaw gain in 1/s.")
    parser.add_argument(
        "--plant-preset",
        default="synthetic_nominal",
        choices=sorted(set(RESPONSE_CURVE_PRESETS) | set(PLANT_PRESET_ALIASES)),
        help="Response-curve preset or alias.",
    )
    parser.add_argument("--output-dir", type=Path, required=True, help="Directory for ticks, plot, config, and summary.")
    parser.add_argument("--seed", type=int, default=7, help="Random seed recorded for the run.")
    parser.add_argument("--max-linear-speed", type=float, default=None, help="Controller max planar speed in m/s.")
    parser.add_argument("--max-yaw-rate", type=float, default=1.5, help="Controller max yaw rate in rad/s.")
    parser.add_argument("--max-planar-accel", type=float, default=3.0, help="Controller planar accel cap in m/s^2.")
    parser.add_argument("--max-yaw-accel", type=float, default=4.0, help="Controller yaw accel cap in rad/s^2.")
    parser.add_argument("--gate-max-divergence", type=float, default=1.0, help="Gate max planar divergence in m.")
    parser.add_argument("--gate-final-position", type=float, default=0.2, help="Gate final position error in m.")
    parser.add_argument("--gate-final-heading", type=float, default=0.25, help="Gate final heading error in rad.")
    parser.add_argument("--gate-settle-time", type=float, default=None, help="Gate settle time in s; default is scenario duration.")
    parser.add_argument("--no-plot", action="store_true", help="Skip plot.png generation.")
    parser.add_argument("--list-presets", action="store_true", help="Print available plant presets and exit.")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_arg_parser()
    args = parser.parse_args(argv)
    if args.list_presets:
        for name in sorted(RESPONSE_CURVE_PRESETS):
            print(name)
        return 0
    if not (math.isfinite(args.rate) and args.rate > 0.0):
        parser.error("--rate must be positive and finite")

    started_wall = time.monotonic()
    started_at = datetime.now(UTC).isoformat()
    preset = _resolve_preset(args.plant_preset)
    scenario = build_scenario(args.scenario, args.speed)
    max_linear_speed = args.max_linear_speed if args.max_linear_speed is not None else args.speed
    limits = HolonomicCommandLimits(
        max_planar_speed_m_s=max_linear_speed,
        max_yaw_rate_rad_s=args.max_yaw_rate,
        max_planar_linear_accel_m_s2=args.max_planar_accel,
        max_yaw_accel_rad_s2=args.max_yaw_accel,
    )
    gate_config = GateConfig(
        max_planar_position_divergence_m=args.gate_max_divergence,
        max_final_position_error_m=args.gate_final_position,
        max_final_heading_error_rad=args.gate_final_heading,
        max_settle_time_s=args.gate_settle_time if args.gate_settle_time is not None else scenario.duration_s,
        require_arrival=True,
        require_no_timeout=True,
        require_finite_metrics=True,
    )

    out_dir = args.output_dir.expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    ticks_path = out_dir / "ticks.jsonl"
    config_path = out_dir / "config.yaml"
    summary_path = out_dir / "summary.json"
    plot_path = out_dir / "plot.png"
    run_id = _run_id(preset.name, scenario.name, args.speed, args.rate, args.seed)
    commit = _git_value(["rev-parse", "HEAD"], "unknown")
    dirty = _git_dirty()
    dt_s = 1.0 / args.rate
    max_ticks = math.ceil(scenario.duration_s / dt_s)
    noise_enabled = any(c.noise_max > 0.0 for c in preset.curves)

    config = {
        "schema_version": CONFIG_SCHEMA_VERSION,
        "run": {
            "id": run_id,
            "created_at": started_at,
            "commit": commit,
            "dirty": dirty,
            "description": f"{preset.name} {scenario.name} at {args.speed} m/s and {args.rate} Hz",
        },
        "scenario": _scenario_config(scenario),
        "plant": preset.to_plant_config(),
        "controller": {
            "name": "HolonomicTrackingController",
            "mode": "direct",
            "gains": {"k_position_per_s": args.k_position, "k_yaw_per_s": args.k_yaw},
            "calibration_source": None,
            "parameters": {},
        },
        "limits": _limits_config(limits),
        "rate": {
            "control_rate_hz": args.rate,
            "dt_s": dt_s,
            "jitter_s": 0.0,
            "max_ticks": max_ticks,
        },
        "random": {
            "seed": args.seed,
            "noise_enabled": noise_enabled,
            "rng": "python.random.Random" if noise_enabled else None,
        },
        "outputs": {
            "ticks_jsonl": "ticks.jsonl",
            "summary_json": "summary.json",
            "plot_png": None if args.no_plot else "plot.png",
        },
        "gates": {
            "max_planar_position_divergence_m": gate_config.max_planar_position_divergence_m,
            "max_final_position_error_m": gate_config.max_final_position_error_m,
            "max_final_heading_error_rad": gate_config.max_final_heading_error_rad,
            "max_settle_time_s": gate_config.max_settle_time_s,
            "require_arrival": gate_config.require_arrival,
            "require_no_timeout": gate_config.require_no_timeout,
            "require_finite_metrics": gate_config.require_finite_metrics,
        },
    }
    _write_config(config_path, config)

    invalid_reasons: list[str] = []
    try:
        preset.validate_for_pr_gate()
    except ValueError as exc:
        invalid_reasons.append(str(exc))

    ticks, final_pose = _simulate(
        scenario=scenario,
        preset=preset,
        limits=limits,
        rate_hz=args.rate,
        k_position=args.k_position,
        k_yaw=args.k_yaw,
        seed=args.seed,
    )
    write_trajectory_control_ticks_jsonl(ticks, ticks_path)
    plot_result = _plot_ticks(ticks_path, plot_path, enabled=not args.no_plot)
    metrics = _compute_metrics(ticks, final_pose, scenario, gate_config)
    gates = _build_gates(metrics, gate_config)
    failed_gates = [name for name, gate in gates.items() if not gate["passed"]]
    if not _finite_tree(metrics):
        invalid_reasons.append("non_finite_metrics")
    if not ticks_path.is_file():
        invalid_reasons.append("missing_ticks_jsonl")
    if not config_path.is_file():
        invalid_reasons.append("missing_config_yaml")
    status = "invalid" if invalid_reasons else ("fail" if failed_gates else "pass")
    reason = "invalid_run" if invalid_reasons else ("gate_failure" if failed_gates else "all_gates_passed")
    finished_at = datetime.now(UTC).isoformat()

    sim_duration_s = len(ticks) * dt_s
    summary = {
        "schema_version": SUMMARY_SCHEMA_VERSION,
        "run": {
            "id": run_id,
            "started_at": started_at,
            "finished_at": finished_at,
            "duration_wall_s": time.monotonic() - started_wall,
            "commit": commit,
            "dirty": dirty,
        },
        "config": {"path": "config.yaml", "sha256": _sha256(config_path)},
        "artifacts": {
            "ticks_jsonl": {
                "path": "ticks.jsonl",
                "sha256": _sha256(ticks_path),
                "line_count": len(ticks),
            },
            "summary_json": {"path": "summary.json"},
            "plot_png": None
            if plot_result is None
            else {
                "path": "plot.png",
                "sha256": plot_result.sha256,
                "produced": plot_result.produced,
                "command": plot_result.command,
                "returncode": plot_result.returncode,
                "stderr": plot_result.stderr,
            },
        },
        "scenario": {
            "name": scenario.name,
            "variant": None,
            "target_speed_m_s": scenario.target_speed_m_s,
        },
        "plant": {
            "model": preset.plant_model,
            "preset": preset.name,
            "response_curve_source": preset.source,
            "response_curve_id": preset.response_curve_id,
        },
        "controller": {
            "name": "HolonomicTrackingController",
            "mode": "direct",
            "calibration_source": None,
        },
        "limits": _limits_config(limits),
        "rate": {
            "control_rate_hz": args.rate,
            "dt_s": dt_s,
            "tick_count": len(ticks),
            "sim_duration_s": sim_duration_s,
            "achieved_rate_hz": len(ticks) / sim_duration_s if sim_duration_s > 0.0 else None,
            "max_dt_error_s": 0.0,
        },
        "seed": args.seed,
        "metrics": metrics,
        "gates": gates,
        "verdict": {
            "status": status,
            "reason": reason,
            "failed_gates": failed_gates,
            "invalid_reasons": invalid_reasons,
        },
    }
    _json_safe_dump(summary_path, summary)

    print(f"Wrote {ticks_path}")
    print(f"Wrote {config_path}")
    print(f"Wrote {summary_path}")
    if plot_result is None:
        print("Skipped plot.png")
    elif plot_result.produced:
        print(f"Wrote {plot_path}")
    else:
        print("Plot not produced; see summary.json artifacts.plot_png.stderr")
    print(f"Verdict: {status} ({reason})")
    return 2 if status == "invalid" else (1 if status == "fail" else 0)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except BrokenPipeError:
        raise SystemExit(0)
