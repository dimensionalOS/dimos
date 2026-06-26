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

"""Run-profile speed application in the local planner."""

from __future__ import annotations

import math
from pathlib import Path as FsPath
import time

import numpy as np
import pytest

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.dannav.local_planner import LocalPlanner
from dimos.navigation.replanning_a_star.navigation_map import NavigationMap
from dimos.navigation.dannav.path_distancer import PathDistancer


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


def _free_navigation_map(global_config: GlobalConfig) -> NavigationMap:
    nav = NavigationMap(global_config, "gradient")
    nav.update(
        OccupancyGrid(
            grid=np.zeros((240, 240), dtype=np.int8),
            resolution=0.05,
            origin=Pose(-2.0, -2.0, 0.0),
            frame_id="map",
            ts=1.0,
        )
    )
    return nav


def _straight_points(length_m: float, spacing_m: float = 0.1) -> list[tuple[float, float]]:
    n = int(round(length_m / spacing_m))
    return [(i * spacing_m, 0.0) for i in range(n + 1)]


def _right_angle_points(
    approach_m: float, exit_m: float, spacing_m: float = 0.1
) -> list[tuple[float, float]]:
    points = _straight_points(approach_m, spacing_m)
    n_exit = int(round(exit_m / spacing_m))
    points.extend((approach_m, i * spacing_m) for i in range(1, n_exit + 1))
    return points


def _make_planner(g: GlobalConfig, *, goal_tolerance: float = 0.2) -> LocalPlanner:
    return LocalPlanner(g, _free_navigation_map(g), goal_tolerance=goal_tolerance)


def _integrate_holonomic_pose(
    x_m: float,
    y_m: float,
    yaw_rad: float,
    cmd_body: Twist,
    dt_s: float,
) -> tuple[float, float, float]:
    """Integrate body-frame cmd_vel in the world frame (test harness only)."""
    vx = float(cmd_body.linear.x)
    vy = float(cmd_body.linear.y)
    wz = float(cmd_body.angular.z)
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    x_m += (c * vx - s * vy) * dt_s
    y_m += (s * vx + c * vy) * dt_s
    yaw_rad += wz * dt_s
    yaw_rad = math.atan2(math.sin(yaw_rad), math.cos(yaw_rad))
    return x_m, y_m, yaw_rad


def _start_and_stop(lp: LocalPlanner, path: Path, run_profile_name: str | None = None) -> None:
    """Start a goal, then stop and wait for the planner thread to exit."""
    lp.start_planning(path, run_profile_name=run_profile_name)
    thread = lp._thread
    lp.stop_planning()
    if thread is not None:
        thread.join(2.0)


def test_walk_default_keeps_legacy_speed() -> None:
    """With the default profile, short sharp paths still plan, and the path speed
    comes from planner_robot_speed exactly as before."""
    g = GlobalConfig(
        local_planner_path_controller="holonomic",
        planner_robot_speed=1.0,
    )
    lp = _make_planner(g)
    lp.handle_odom(_pose_stamped(0.0, 0.0, 0.0, ts=1.0))
    stops: list[str] = []
    sub = lp.stopped_navigating.subscribe(stops.append)

    try:
        _start_and_stop(lp, _path_from_points(_right_angle_points(0.3, 0.2)))
    finally:
        sub.dispose()
        lp.stop()

    assert "run_envelope_rejected" not in stops

    distancer = PathDistancer(_path_from_points(_straight_points(6.0)))
    mid_pos = np.array([3.0, 0.0], dtype=np.float64)
    speed = lp._path_speed_for_index(distancer, distancer.find_closest_point_index(mid_pos), mid_pos)
    assert speed == pytest.approx(1.0)


def test_run_conservative_drives_planner_speed_and_command_caps() -> None:
    g = GlobalConfig(
        local_planner_path_controller="holonomic",
        go2_run_profile="run_conservative",
    )
    lp = _make_planner(g)
    lp.handle_odom(_pose_stamped(0.0, 0.0, 0.0, ts=time.time()))
    stops: list[str] = []
    sub = lp.stopped_navigating.subscribe(stops.append)
    path = _path_from_points(_straight_points(6.0))

    try:
        _start_and_stop(lp, path)
    finally:
        sub.dispose()
        lp.stop()

    assert "run_envelope_rejected" not in stops

    distancer = PathDistancer(path)
    mid_pos = np.array([3.0, 0.0], dtype=np.float64)
    near_goal_pos = np.array([5.7, 0.0], dtype=np.float64)
    mid_speed = lp._path_speed_for_index(
        distancer, distancer.find_closest_point_index(mid_pos), mid_pos
    )
    near_goal_speed = lp._path_speed_for_index(
        distancer, distancer.find_closest_point_index(near_goal_pos), near_goal_pos
    )

    assert mid_speed == pytest.approx(1.5)
    assert near_goal_speed == pytest.approx(math.sqrt(2.0 * 1.5 * 0.3))


def test_run_profile_speed_respects_global_nerf() -> None:
    g = GlobalConfig(
        local_planner_path_controller="holonomic",
        go2_run_profile="run_conservative",
        nerf_speed=0.5,
    )
    lp = _make_planner(g)
    lp.handle_odom(_pose_stamped(0.0, 0.0, 0.0, ts=time.time()))
    path = _path_from_points(_straight_points(6.0))

    try:
        _start_and_stop(lp, path)
    finally:
        lp.stop()

    distancer = PathDistancer(path)
    mid_pos = np.array([3.0, 0.0], dtype=np.float64)
    speed = lp._path_speed_for_index(distancer, distancer.find_closest_point_index(mid_pos), mid_pos)
    assert speed == pytest.approx(0.75)


def test_per_goal_override_applies_and_does_not_leak_to_next_goal() -> None:
    g = GlobalConfig(local_planner_path_controller="holonomic")
    lp = _make_planner(g)
    lp.handle_odom(_pose_stamped(0.0, 0.0, 0.0, ts=time.time()))
    path = _path_from_points(_straight_points(6.0))
    distancer = PathDistancer(path)
    mid_pos = np.array([3.0, 0.0], dtype=np.float64)
    mid_index = distancer.find_closest_point_index(mid_pos)

    try:
        _start_and_stop(lp, path, run_profile_name="trot")
        trot_speed = lp._path_speed_for_index(distancer, mid_index, mid_pos)

        _start_and_stop(lp, path)
        default_speed = lp._path_speed_for_index(distancer, mid_index, mid_pos)
    finally:
        lp.stop()

    assert trot_speed == pytest.approx(1.0)
    assert default_speed == pytest.approx(0.55)


def test_run_profile_commands_faster_than_walk_in_closed_loop() -> None:
    rate_hz = 60.0
    dt_s = 1.0 / rate_hz
    g = GlobalConfig(
        local_planner_path_controller="holonomic",
        local_planner_control_rate_hz=rate_hz,
        go2_run_profile="run_conservative",
    )
    planner = LocalPlanner(g, _free_navigation_map(g), goal_tolerance=0.1)
    plant_x_m, plant_y_m, plant_yaw_rad = 0.0, 0.0, 0.0
    latest_cmd = Twist()
    commanded_speeds: list[float] = []
    stops: list[str] = []

    def _on_cmd_vel(cmd: Twist) -> None:
        nonlocal latest_cmd
        latest_cmd = Twist(cmd)
        commanded_speeds.append(math.hypot(float(cmd.linear.x), float(cmd.linear.y)))

    cmd_sub = planner.cmd_vel.subscribe(_on_cmd_vel)
    stop_sub = planner.stopped_navigating.subscribe(stops.append)

    try:
        planner.handle_odom(
            _pose_stamped(plant_x_m, plant_y_m, plant_yaw_rad, ts=time.time())
        )
        planner.start_planning(_path_from_points([(0.1, 0.0), (3.1, 0.0)]))
        for _ in range(420):
            if stops:
                break
            time.sleep(dt_s * 1.1)
            plant_x_m, plant_y_m, plant_yaw_rad = _integrate_holonomic_pose(
                plant_x_m, plant_y_m, plant_yaw_rad, latest_cmd, dt_s
            )
            planner.handle_odom(
                _pose_stamped(plant_x_m, plant_y_m, plant_yaw_rad, ts=time.time())
            )
    finally:
        planner.stop()
        cmd_sub.dispose()
        stop_sub.dispose()

    assert "arrived" in stops
    assert commanded_speeds
    assert max(commanded_speeds) > 1.0
    assert max(commanded_speeds) <= 1.5 + 1e-6
