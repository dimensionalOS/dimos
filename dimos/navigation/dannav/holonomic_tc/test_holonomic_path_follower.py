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

"""Closed-loop ``_HolonomicPathFollower`` integration tests.

Math for profiling, tracking, and command limits lives in sibling unit tests.
Here: path in -> cmd_vel out -> simulated plant -> arrival / caps / branches.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.dannav.geometry.path_speed_profile import (
    PathSpeedProfileLimits,
)
from dimos.navigation.dannav.holonomic_tc.module import (
    ActiveRunEnvelope,
    DanHolonomicTCConfig,
    _HolonomicPathFollower,
)
from dimos.navigation.dannav.holonomic_tc.run_profiles import RunProfile


def _planar_speed_m_s(cmd: Twist) -> float:
    return math.hypot(float(cmd.linear.x), float(cmd.linear.y))


def _yaw_quaternion(yaw_rad: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


def _pose_stamped(x: float, y: float, yaw_rad: float, *, ts: float = 1.0) -> PoseStamped:
    return PoseStamped(
        ts=ts,
        frame_id="map",
        position=[x, y, 0.0],
        orientation=_yaw_quaternion(yaw_rad),
    )


def _path_from_points(points: list[tuple[float, float]]) -> Path:
    poses: list[PoseStamped] = []
    for index, point in enumerate(points):
        if index + 1 < len(points):
            next_point = points[index + 1]
            yaw = math.atan2(next_point[1] - point[1], next_point[0] - point[0])
        else:
            prev_point = points[index - 1]
            yaw = math.atan2(point[1] - prev_point[1], point[0] - prev_point[0])
        poses.append(_pose_stamped(point[0], point[1], yaw))
    return Path(frame_id="map", poses=poses)


def _make_follower(**overrides: object) -> _HolonomicPathFollower:
    return _HolonomicPathFollower(DanHolonomicTCConfig(**overrides))


def _install_envelope(
    core: _HolonomicPathFollower,
    *,
    speed_m_s: float,
    max_tangent_accel_m_s2: float,
    max_normal_accel_m_s2: float,
    goal_decel_m_s2: float,
) -> None:
    core._apply_run_envelope(
        ActiveRunEnvelope(
            profile=RunProfile(
                name="test",
                requested_planner_speed_m_s=speed_m_s,
                max_tangent_accel_m_s2=max_tangent_accel_m_s2,
                max_normal_accel_m_s2=max_normal_accel_m_s2,
                goal_decel_m_s2=goal_decel_m_s2,
                max_planar_cmd_accel_m_s2=8.0,
                max_yaw_rate_rad_s=speed_m_s,
                max_yaw_accel_rad_s2=8.0,
            ),
            speed_m_s=speed_m_s,
            path_limits=PathSpeedProfileLimits(
                max_speed_m_s=speed_m_s,
                max_tangent_accel_m_s2=max_tangent_accel_m_s2,
                max_normal_accel_m_s2=max_normal_accel_m_s2,
            ),
            goal_decel_m_s2=goal_decel_m_s2,
        )
    )


@dataclass
class _RunResult:
    command_history: list[Twist]
    stop_messages: list[str]
    final_x_m: float
    final_y_m: float
    final_yaw_rad: float


def _run_follower(
    core: _HolonomicPathFollower,
    *,
    points: list[tuple[float, float]],
    initial_yaw_rad: float = 0.0,
    max_ticks: int = 300,
    rate_hz: float = 60.0,
) -> _RunResult:
    dt_s = 1.0 / rate_hz
    plant_x_m, plant_y_m, plant_yaw_rad = 0.0, 0.0, initial_yaw_rad
    latest_cmd = Twist()
    command_history: list[Twist] = []
    stop_messages: list[str] = []

    def _on_cmd_vel(cmd: Twist) -> None:
        nonlocal latest_cmd
        latest_cmd = Twist(cmd)
        command_history.append(Twist(cmd))

    cmd_sub = core.cmd_vel.subscribe(_on_cmd_vel)
    stop_sub = core.stopped_navigating.subscribe(stop_messages.append)
    sim_time_s = 1.0

    try:
        core.handle_odom(_pose_stamped(plant_x_m, plant_y_m, plant_yaw_rad, ts=sim_time_s))
        core.start_planning(_path_from_points(points))
        for _ in range(max_ticks):
            if "arrived" in stop_messages:
                break
            time.sleep(dt_s * 1.1)
            vx = float(latest_cmd.linear.x)
            vy = float(latest_cmd.linear.y)
            wz = float(latest_cmd.angular.z)
            c = math.cos(plant_yaw_rad)
            s = math.sin(plant_yaw_rad)
            plant_x_m += (c * vx - s * vy) * dt_s
            plant_y_m += (s * vx + c * vy) * dt_s
            plant_yaw_rad += wz * dt_s
            plant_yaw_rad = math.atan2(math.sin(plant_yaw_rad), math.cos(plant_yaw_rad))
            sim_time_s += dt_s
            core.handle_odom(_pose_stamped(plant_x_m, plant_y_m, plant_yaw_rad, ts=sim_time_s))
    finally:
        core.close()
        cmd_sub.dispose()
        stop_sub.dispose()

    return _RunResult(command_history, stop_messages, plant_x_m, plant_y_m, plant_yaw_rad)


def test_closed_loop_straight_line_arrives_with_goal_decel() -> None:
    cruise_speed_m_s = 1.2
    goal_tolerance_m = 0.08
    goal_x_m = 1.0
    result = _run_follower(
        _make_follower(
            speed_m_s=cruise_speed_m_s, control_frequency=60.0, goal_tolerance=goal_tolerance_m
        ),
        points=[(0.1, 0.0), (goal_x_m, 0.0)],
    )
    speeds = [
        _planar_speed_m_s(cmd) for cmd in result.command_history if _planar_speed_m_s(cmd) > 0.05
    ]

    assert "arrived" in result.stop_messages
    assert speeds
    peak = max(speeds)
    assert peak > 0.5 * cruise_speed_m_s
    assert min(speeds[-10:]) < 0.5 * peak
    assert math.hypot(result.final_x_m - goal_x_m, result.final_y_m) < goal_tolerance_m + 0.07


def test_closed_loop_right_angle_respects_curvature_speed_cap() -> None:
    core = _make_follower(control_frequency=60.0, goal_tolerance=0.08)
    _install_envelope(
        core,
        speed_m_s=2.0,
        max_tangent_accel_m_s2=0.5,
        max_normal_accel_m_s2=0.1,
        goal_decel_m_s2=0.5,
    )
    result = _run_follower(
        core,
        points=[(0.1, 0.0), (0.55, 0.0), (0.55, 0.55), (0.9, 0.55)],
        max_ticks=420,
    )
    speeds = [_planar_speed_m_s(cmd) for cmd in result.command_history]

    assert "arrived" in result.stop_messages
    assert speeds
    assert max(speeds) < 0.75


def test_closed_loop_rotate_first_then_arrives() -> None:
    result = _run_follower(
        _make_follower(
            speed_m_s=0.9,
            control_frequency=60.0,
            goal_tolerance=0.08,
            align_heading_before_move=True,
        ),
        points=[(0.1, 0.0), (1.0, 0.0)],
        initial_yaw_rad=0.8,
        max_ticks=320,
    )

    assert "arrived" in result.stop_messages
    assert any(
        abs(float(cmd.angular.z)) > 0.05 and _planar_speed_m_s(cmd) < 0.05
        for cmd in result.command_history[:20]
    )
    assert abs(result.final_yaw_rad) < 0.35
