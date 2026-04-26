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

"""P3-3: ``LocalPlanner`` path controller switch and holonomic integration smoke."""

from __future__ import annotations

import math
from pathlib import Path as FsPath
import time

import numpy as np
from pydantic import ValidationError
import pytest

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.navigation.replanning_a_star.controllers import (
    HolonomicPathController,
    PController,
    make_local_path_controller,
)
from dimos.navigation.replanning_a_star.local_planner import LocalPlanner
from dimos.navigation.replanning_a_star.navigation_map import NavigationMap
from dimos.navigation.replanning_a_star.path_distancer import PathDistancer
from dimos.navigation.trajectory_control_tick_export import iter_trajectory_control_tick_jsonl
from dimos.navigation.trajectory_holonomic_plant import IntegratedHolonomicPlant


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
            grid=np.zeros((200, 200), dtype=np.int8),
            resolution=0.05,
            origin=Pose(-2.0, -2.0, 0.0),
            frame_id="map",
            ts=1.0,
        )
    )
    return nav


class _LocalPlannerHarnessResult:
    def __init__(
        self,
        *,
        rows: list[dict[str, object]],
        stop_messages: list[str],
        final_x_m: float,
        final_y_m: float,
        final_yaw_rad: float,
    ) -> None:
        self.rows = rows
        self.stop_messages = stop_messages
        self.final_x_m = final_x_m
        self.final_y_m = final_y_m
        self.final_yaw_rad = final_yaw_rad


def _run_local_planner_harness(
    tmp_path: FsPath,
    *,
    points: list[tuple[float, float]],
    initial_yaw_rad: float = 0.0,
    speed_m_s: float = 1.0,
    max_normal_accel_m_s2: float = 0.6,
    max_tangent_accel_m_s2: float = 1.0,
    max_ticks: int = 260,
) -> _LocalPlannerHarnessResult:
    rate_hz = 60.0
    dt_s = 1.0 / rate_hz
    log_path = tmp_path / "ticks.jsonl"
    global_config = GlobalConfig(
        local_planner_path_controller="holonomic",
        local_planner_control_rate_hz=rate_hz,
        planner_robot_speed=speed_m_s,
        local_planner_max_normal_accel_m_s2=max_normal_accel_m_s2,
        local_planner_max_tangent_accel_m_s2=max_tangent_accel_m_s2,
        local_planner_max_planar_cmd_accel_m_s2=8.0,
        local_planner_max_yaw_accel_rad_s2=8.0,
        local_planner_trajectory_tick_log_path=str(log_path),
    )
    planner = LocalPlanner(global_config, _free_navigation_map(global_config), goal_tolerance=0.08)
    plant = IntegratedHolonomicPlant(x=0.0, y=0.0, yaw_rad=initial_yaw_rad)
    latest_cmd = Twist()
    stop_messages: list[str] = []

    def _on_cmd_vel(cmd: Twist) -> None:
        nonlocal latest_cmd
        latest_cmd = Twist(cmd)

    cmd_sub = planner.cmd_vel.subscribe(_on_cmd_vel)
    stop_sub = planner.stopped_navigating.subscribe(stop_messages.append)
    sim_time_s = 1.0

    try:
        planner.handle_odom(_pose_stamped(plant.x, plant.y, plant.yaw_rad, ts=sim_time_s))
        planner.start_planning(_path_from_points(points))
        for _ in range(max_ticks):
            if "arrived" in stop_messages:
                break
            time.sleep(dt_s * 1.1)
            plant.step(latest_cmd, dt_s)
            sim_time_s += dt_s
            planner.handle_odom(_pose_stamped(plant.x, plant.y, plant.yaw_rad, ts=sim_time_s))
    finally:
        planner.stop()
        cmd_sub.dispose()
        stop_sub.dispose()

    return _LocalPlannerHarnessResult(
        rows=list(iter_trajectory_control_tick_jsonl(log_path)),
        stop_messages=stop_messages,
        final_x_m=plant.x,
        final_y_m=plant.y,
        final_yaw_rad=plant.yaw_rad,
    )


def test_make_local_path_controller_default_is_holonomic() -> None:
    g = GlobalConfig()
    c = make_local_path_controller(g, 0.5, 10.0)
    assert type(c) is HolonomicPathController


def test_make_local_path_controller_differential_when_configured() -> None:
    g = GlobalConfig(local_planner_path_controller="differential")
    c = make_local_path_controller(g, 0.5, 10.0)
    assert type(c) is PController


def test_make_local_path_controller_holonomic() -> None:
    g = GlobalConfig(local_planner_path_controller="holonomic")
    c = make_local_path_controller(g, 0.5, 10.0)
    assert type(c) is HolonomicPathController


def test_holonomic_path_controller_advance_on_straight_line() -> None:
    g = GlobalConfig(local_planner_path_controller="holonomic")
    ctrl = make_local_path_controller(g, 0.5, 10.0)
    assert isinstance(ctrl, HolonomicPathController)
    odom = PoseStamped(
        frame_id="map", position=[0.0, 0.0, 0.0], orientation=Quaternion(0, 0, 0, 1)
    )
    lookahead = np.array([0.4, 0.0], dtype=np.float64)
    out = ctrl.advance(lookahead, odom)
    # Along +x, expect forward body x and finite yaw rate
    assert math.hypot(float(out.linear.x), float(out.linear.y)) > 0.1
    assert abs(float(out.angular.z)) < 2.0


def test_holonomic_path_controller_rotate_uses_pose_when_passed() -> None:
    g = GlobalConfig(local_planner_path_controller="holonomic")
    ctrl = HolonomicPathController(
        g,
        speed=0.55,
        control_frequency=10.0,
        k_position_per_s=2.0,
        k_yaw_per_s=1.5,
    )
    odom = PoseStamped(
        frame_id="map", position=[0.0, 0.0, 0.0], orientation=Quaternion(0, 0, 0, 1)
    )
    yaw_e = 0.3
    t = ctrl.rotate(yaw_e, odom)
    assert math.isfinite(t.angular.z)
    assert math.hypot(float(t.linear.x), float(t.linear.y)) < 1.0


def test_holonomic_path_controller_slews_first_command_from_rest() -> None:
    g = GlobalConfig(local_planner_path_controller="holonomic")
    ctrl = HolonomicPathController(
        g,
        speed=0.55,
        control_frequency=10.0,
        k_position_per_s=2.0,
        k_yaw_per_s=1.5,
    )
    odom = PoseStamped(
        frame_id="map", position=[0.0, 0.0, 0.0], orientation=Quaternion(0, 0, 0, 1)
    )
    out = ctrl.advance(np.array([0.5, 0.0], dtype=np.float64), odom)
    assert math.hypot(float(out.linear.x), float(out.linear.y)) == pytest.approx(0.5)


def test_holonomic_path_controller_uses_configured_command_limits() -> None:
    g = GlobalConfig(
        local_planner_path_controller="holonomic",
        local_planner_max_planar_cmd_accel_m_s2=0.25,
        local_planner_max_yaw_accel_rad_s2=0.3,
        local_planner_max_yaw_rate_rad_s=0.4,
    )
    ctrl = HolonomicPathController(
        g,
        speed=1.2,
        control_frequency=10.0,
        k_position_per_s=2.0,
        k_yaw_per_s=1.5,
    )

    assert ctrl._limits.max_planar_speed_m_s == pytest.approx(1.2)
    assert ctrl._limits.max_planar_linear_accel_m_s2 == pytest.approx(0.25)
    assert ctrl._limits.max_yaw_accel_rad_s2 == pytest.approx(0.3)
    assert ctrl._limits.max_yaw_rate_rad_s == pytest.approx(0.4)

    ctrl.set_speed(0.8)

    assert ctrl._limits.max_planar_speed_m_s == pytest.approx(0.8)
    assert ctrl._limits.max_yaw_rate_rad_s == pytest.approx(0.4)


def test_local_planner_wires_holonomic_when_configured() -> None:
    g = GlobalConfig(local_planner_path_controller="holonomic")
    nav = NavigationMap(g, "gradient")
    lp = LocalPlanner(g, nav, goal_tolerance=0.2)
    assert isinstance(lp._controller, HolonomicPathController)


def test_local_planner_path_following_writes_speed_vs_divergence_jsonl(tmp_path: FsPath) -> None:
    log_path = tmp_path / "ticks.jsonl"
    g = GlobalConfig(
        local_planner_path_controller="holonomic",
        local_planner_trajectory_tick_log_path=str(log_path),
    )
    nav = NavigationMap(g, "gradient")
    lp = LocalPlanner(g, nav, goal_tolerance=0.01)
    path = Path(
        frame_id="map",
        poses=[
            PoseStamped(
                frame_id="map",
                position=[0.0, 0.0, 0.0],
                orientation=Quaternion(0, 0, 0, 1),
            ),
            PoseStamped(
                frame_id="map",
                position=[1.0, 0.0, 0.0],
                orientation=Quaternion(0, 0, 0, 1),
            ),
        ],
    )
    odom = PoseStamped(
        frame_id="map",
        position=[0.0, 0.1, 0.0],
        orientation=Quaternion(0, 0, 0, 1),
    )
    lp._path = path
    lp._path_distancer = PathDistancer(path)
    lp._current_odom = odom

    cmd = lp._compute_path_following()

    rows = list(iter_trajectory_control_tick_jsonl(log_path))
    assert len(rows) == 1
    row = rows[0]
    assert row["commanded_planar_speed_m_s"] == pytest.approx(math.hypot(cmd.linear.x, cmd.linear.y))
    assert row["planar_position_divergence_m"] > 0.0
    assert row["dt_s"] == pytest.approx(0.1)


def test_local_planner_live_tick_jsonl_includes_measured_body_twist(tmp_path: FsPath) -> None:
    log_path = tmp_path / "ticks.jsonl"
    g = GlobalConfig(
        local_planner_path_controller="holonomic",
        local_planner_trajectory_tick_log_path=str(log_path),
    )
    nav = NavigationMap(g, "gradient")
    lp = LocalPlanner(g, nav, goal_tolerance=0.01)
    path = Path(
        frame_id="map",
        poses=[
            PoseStamped(
                frame_id="map",
                position=[0.0, 0.0, 0.0],
                orientation=Quaternion(0, 0, 0, 1),
            ),
            PoseStamped(
                frame_id="map",
                position=[2.0, 0.0, 0.0],
                orientation=Quaternion(0, 0, 0, 1),
            ),
        ],
    )
    lp._path = path
    lp._path_distancer = PathDistancer(path)
    lp._current_odom = PoseStamped(
        ts=1.0,
        frame_id="map",
        position=[0.0, 0.0, 0.0],
        orientation=Quaternion(0, 0, 0, 1),
    )
    lp._compute_path_following()
    lp._current_odom = PoseStamped(
        ts=2.0,
        frame_id="map",
        position=[0.2, 0.0, 0.0],
        orientation=Quaternion(0, 0, 0, 1),
    )

    lp._compute_path_following()

    rows = list(iter_trajectory_control_tick_jsonl(log_path))
    assert len(rows) == 2
    assert rows[1]["meas_twist_linear_x_m_s"] == pytest.approx(0.2)
    assert rows[1]["meas_twist_linear_y_m_s"] == pytest.approx(0.0)
    assert rows[1]["meas_twist_angular_z_rad_s"] == pytest.approx(0.0)


def test_local_planner_uses_curvature_speed_cap_for_holonomic_path() -> None:
    g = GlobalConfig(
        local_planner_path_controller="holonomic",
        planner_robot_speed=1.2,
        local_planner_max_normal_accel_m_s2=0.6,
    )
    nav = NavigationMap(g, "gradient")
    lp = LocalPlanner(g, nav, goal_tolerance=0.01)
    path = Path(
        frame_id="map",
        poses=[
            PoseStamped(
                frame_id="map",
                position=[0.0, 0.0, 0.0],
                orientation=Quaternion(0, 0, 0, 1),
            ),
            PoseStamped(
                frame_id="map",
                position=[0.5, 0.0, 0.0],
                orientation=Quaternion(0, 0, 0, 1),
            ),
            PoseStamped(
                frame_id="map",
                position=[0.5, 0.5, 0.0],
                orientation=Quaternion(0, 0, 0, 1),
            ),
        ],
    )
    lp._path = path
    lp._path_distancer = PathDistancer(path)
    lp._current_odom = PoseStamped(
        frame_id="map",
        position=[0.5, 0.0, 0.0],
        orientation=Quaternion(0, 0, 0, 1),
    )

    lp._compute_path_following()

    assert isinstance(lp._controller, HolonomicPathController)
    assert lp._controller._speed < 1.2


def test_local_planner_in_the_loop_harness_reaches_arrival_on_straight_line(
    tmp_path: FsPath,
) -> None:
    result = _run_local_planner_harness(
        tmp_path,
        points=[(0.1, 0.0), (1.2, 0.0)],
        speed_m_s=1.0,
    )

    assert "arrived" in result.stop_messages
    assert result.rows
    assert math.hypot(result.final_x_m - 1.2, result.final_y_m) < 0.15
    assert max(float(row["planar_position_divergence_m"]) for row in result.rows) < 0.75


def test_local_planner_in_the_loop_harness_exercises_curvature_speed_cap(
    tmp_path: FsPath,
) -> None:
    result = _run_local_planner_harness(
        tmp_path,
        points=[(0.1, 0.0), (0.45, 0.0), (0.45, 0.45), (0.8, 0.45)],
        speed_m_s=1.2,
        max_normal_accel_m_s2=0.6,
        max_ticks=320,
    )
    positive_path_speeds = [
        float(row["ref_twist_linear_x_m_s"])
        for row in result.rows
        if float(row["ref_twist_linear_x_m_s"]) > 0.0
    ]

    assert "arrived" in result.stop_messages
    assert positive_path_speeds
    assert min(positive_path_speeds) < 0.6
    assert max(float(row["planar_position_divergence_m"]) for row in result.rows) < 0.9


def test_local_planner_in_the_loop_harness_decelerates_near_goal(
    tmp_path: FsPath,
) -> None:
    result = _run_local_planner_harness(
        tmp_path,
        points=[(0.1, 0.0), (1.0, 0.0)],
        speed_m_s=1.2,
        max_ticks=300,
    )
    positive_path_speeds = [
        float(row["ref_twist_linear_x_m_s"])
        for row in result.rows
        if float(row["ref_twist_linear_x_m_s"]) > 0.0
    ]

    assert "arrived" in result.stop_messages
    assert max(positive_path_speeds[: len(positive_path_speeds) // 2]) > 0.7
    assert min(positive_path_speeds[-10:]) < 0.55
    assert min(positive_path_speeds[-10:]) < 0.6 * max(positive_path_speeds)
    assert math.hypot(result.final_x_m - 1.0, result.final_y_m) < 0.15


def test_local_planner_in_the_loop_harness_exercises_initial_rotation(
    tmp_path: FsPath,
) -> None:
    result = _run_local_planner_harness(
        tmp_path,
        points=[(0.1, 0.0), (1.0, 0.0)],
        initial_yaw_rad=0.8,
        speed_m_s=0.9,
        max_ticks=320,
    )

    assert "arrived" in result.stop_messages
    assert any(
        abs(float(row["cmd_angular_z_rad_s"])) > 0.05
        and float(row["commanded_planar_speed_m_s"]) < 0.05
        for row in result.rows[:20]
    )
    assert abs(result.final_yaw_rad) < 0.35


def test_local_planner_control_rate_from_global_config() -> None:
    """P4-1: loop pacing and ``make_local_path_controller`` share ``GlobalConfig`` Hz."""
    g = GlobalConfig(local_planner_control_rate_hz=25.0, local_planner_path_controller="holonomic")
    nav = NavigationMap(g, "gradient")
    lp = LocalPlanner(g, nav, goal_tolerance=0.2)
    assert lp._control_frequency == 25.0
    assert lp._controller._control_frequency == 25.0


def test_local_planner_uses_configured_robot_speed_for_holonomic_runs() -> None:
    g = GlobalConfig(local_planner_path_controller="holonomic", planner_robot_speed=1.2)
    nav = NavigationMap(g, "gradient")
    lp = LocalPlanner(g, nav, goal_tolerance=0.2)
    assert lp._speed == pytest.approx(1.2)
    assert lp._controller._speed == pytest.approx(1.2)


def test_local_planner_control_rate_hz_validation() -> None:
    with pytest.raises(ValidationError):
        GlobalConfig(local_planner_control_rate_hz=0.0)
    with pytest.raises(ValidationError):
        GlobalConfig(local_planner_control_rate_hz=100.0)
    with pytest.raises(ValidationError):
        GlobalConfig(local_planner_control_rate_hz=300.0)
    with pytest.raises(ValidationError):
        GlobalConfig(local_planner_max_planar_cmd_accel_m_s2=0.0)
    with pytest.raises(ValidationError):
        GlobalConfig(local_planner_max_yaw_accel_rad_s2=0.0)
    with pytest.raises(ValidationError):
        GlobalConfig(local_planner_max_yaw_rate_rad_s=0.0)


def test_local_planner_control_rate_hz_update_validation() -> None:
    g = GlobalConfig()
    with pytest.raises(ValidationError):
        g.update(local_planner_control_rate_hz=100.0)
    assert g.local_planner_control_rate_hz == pytest.approx(10.0)


def test_replay_flag_does_not_change_holonomic_path_controller() -> None:
    """``replay=True`` selects replay sensor backends; the path follower is independent."""
    g = GlobalConfig(
        replay=True,
        local_planner_path_controller="holonomic",
    )
    nav = NavigationMap(g, "gradient")
    lp = LocalPlanner(g, nav, goal_tolerance=0.2)
    assert isinstance(lp._controller, HolonomicPathController)
