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

"""Run-profile session envelope via closed-loop ``cmd_vel``.

Profile resolution and live ``set_run_profile`` - observable as commanded speed
and arrival, not ``_path_speed_at_position`` math.
"""

from __future__ import annotations

import math
import time

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.dannav.holonomic_tc.module import (
    DanHolonomicTCConfig,
    _HolonomicPathFollower,
)


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


def _planar_speed_m_s(cmd: Twist) -> float:
    return math.hypot(float(cmd.linear.x), float(cmd.linear.y))


def _closed_loop_max_planar_speed(
    core: _HolonomicPathFollower,
    *,
    rate_hz: float = 60.0,
    max_ticks: int = 420,
) -> float:
    dt_s = 1.0 / rate_hz
    plant_x_m, plant_y_m, plant_yaw_rad = 0.0, 0.0, 0.0
    latest_cmd = Twist()
    commanded_speeds: list[float] = []
    stops: list[str] = []

    def _on_cmd_vel(cmd: Twist) -> None:
        nonlocal latest_cmd
        latest_cmd = Twist(cmd)
        commanded_speeds.append(_planar_speed_m_s(cmd))

    cmd_sub = core.cmd_vel.subscribe(_on_cmd_vel)
    stop_sub = core.stopped_navigating.subscribe(stops.append)
    sim_time_s = 1.0

    try:
        core.handle_odom(_pose_stamped(plant_x_m, plant_y_m, plant_yaw_rad, ts=sim_time_s))
        core.start_planning(_path_from_points([(0.1, 0.0), (3.1, 0.0)]))
        for _ in range(max_ticks):
            if "arrived" in stops:
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

    assert "arrived" in stops
    assert commanded_speeds
    return max(commanded_speeds)


def test_run_conservative_commands_faster_than_walk_in_closed_loop() -> None:
    walk_max = _closed_loop_max_planar_speed(_make_follower(run_profile="walk"))
    run_max = _closed_loop_max_planar_speed(_make_follower(run_profile="run_conservative"))
    assert run_max > walk_max


def test_set_run_profile_rejects_unknown_profile() -> None:
    core = _make_follower()
    assert core.set_run_profile("does-not-exist") is False
    assert core.set_run_profile("trot") is True
