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
from typing import Any

import numpy as np
import pytest

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.dannav.controllers import (
    HolonomicPathController,
    make_local_path_controller,
)
from dimos.navigation.dannav.local_planner import LocalPlanner
from dimos.navigation.replanning_a_star.navigation_map import NavigationMap
from dimos.navigation.dannav.path_distancer import PathDistancer
from dimos.navigation.holonomic_trajectory_controller.trajectory_holonomic_tracking_controller import HolonomicTrackingController
from dimos.navigation.holonomic_trajectory_controller.trajectory_types import TrajectoryMeasuredSample, TrajectoryReferenceSample
from dimos.utils.trigonometry import angle_diff


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


class _LocalPlannerHarnessResult:
    def __init__(
        self,
        *,
        command_history: list[Twist],
        stop_messages: list[str],
        final_x_m: float,
        final_y_m: float,
        final_yaw_rad: float,
    ) -> None:
        self.command_history = command_history
        self.stop_messages = stop_messages
        self.final_x_m = final_x_m
        self.final_y_m = final_y_m
        self.final_yaw_rad = final_yaw_rad


class _RecordingHolonomicTrackingController(HolonomicTrackingController):
    def __init__(self) -> None:
        super().__init__(k_position_per_s=0.0, k_yaw_per_s=0.0)
        self.measurements: list[TrajectoryMeasuredSample] = []

    def control(
        self,
        reference: TrajectoryReferenceSample,
        measurement: TrajectoryMeasuredSample,
    ) -> Twist:
        self.measurements.append(measurement)
        return super().control(reference, measurement)


def _local_planner_with_recording_holonomic_controller() -> tuple[
    LocalPlanner, _RecordingHolonomicTrackingController
]:
    g = GlobalConfig(local_planner_path_controller="holonomic")
    nav = NavigationMap(g, "gradient")
    lp = LocalPlanner(g, nav, goal_tolerance=0.01)
    path = _path_from_points([(0.0, 0.0), (2.0, 0.0)])
    lp._path = path
    lp._path_distancer = PathDistancer(path)
    controller = lp._controller
    assert isinstance(controller, HolonomicPathController)
    recorder = _RecordingHolonomicTrackingController()
    controller._inner = recorder
    return lp, recorder


def _run_local_planner_harness(
    tmp_path: FsPath,
    *,
    points: list[tuple[float, float]],
    initial_yaw_rad: float = 0.0,
    speed_m_s: float = 1.0,
    max_normal_accel_m_s2: float = 0.6,
    max_tangent_accel_m_s2: float = 1.0,
    max_goal_decel_m_s2: float = 1.0,
    max_ticks: int = 260,
) -> _LocalPlannerHarnessResult:
    rate_hz = 60.0
    dt_s = 1.0 / rate_hz
    global_config = GlobalConfig(
        local_planner_path_controller="holonomic",
        local_planner_control_rate_hz=rate_hz,
        planner_robot_speed=speed_m_s,
        local_planner_max_normal_accel_m_s2=max_normal_accel_m_s2,
        local_planner_max_tangent_accel_m_s2=max_tangent_accel_m_s2,
        local_planner_goal_decel_m_s2=max_goal_decel_m_s2,
        local_planner_max_planar_cmd_accel_m_s2=8.0,
        local_planner_max_yaw_accel_rad_s2=8.0,
    )
    planner = LocalPlanner(global_config, _free_navigation_map(global_config), goal_tolerance=0.08)
    plant_x_m, plant_y_m, plant_yaw_rad = 0.0, 0.0, initial_yaw_rad
    latest_cmd = Twist()
    command_history: list[Twist] = []
    stop_messages: list[str] = []

    def _on_cmd_vel(cmd: Twist) -> None:
        nonlocal latest_cmd
        latest_cmd = Twist(cmd)
        command_history.append(Twist(cmd))

    cmd_sub = planner.cmd_vel.subscribe(_on_cmd_vel)
    stop_sub = planner.stopped_navigating.subscribe(stop_messages.append)
    sim_time_s = 1.0

    try:
        planner.handle_odom(_pose_stamped(plant_x_m, plant_y_m, plant_yaw_rad, ts=sim_time_s))
        planner.start_planning(_path_from_points(points))
        for _ in range(max_ticks):
            if "arrived" in stop_messages:
                break
            time.sleep(dt_s * 1.1)
            plant_x_m, plant_y_m, plant_yaw_rad = _integrate_holonomic_pose(
                plant_x_m, plant_y_m, plant_yaw_rad, latest_cmd, dt_s
            )
            sim_time_s += dt_s
            planner.handle_odom(
                _pose_stamped(plant_x_m, plant_y_m, plant_yaw_rad, ts=sim_time_s)
            )
    finally:
        planner.stop()
        cmd_sub.dispose()
        stop_sub.dispose()

    return _LocalPlannerHarnessResult(
        command_history=command_history,
        stop_messages=stop_messages,
        final_x_m=plant_x_m,
        final_y_m=plant_y_m,
        final_yaw_rad=plant_yaw_rad,
    )


def test_holonomic_path_controller_slews_first_command_from_rest() -> None:
    g = GlobalConfig(local_planner_path_controller="holonomic")
    ctrl = HolonomicPathController(
        g,
        speed=0.55,
        control_frequency=10.0,
        k_position_per_s=2.0,
        k_yaw_per_s=1.5,
    )
    odom = PoseStamped(frame_id="map", position=[0.0, 0.0, 0.0], orientation=Quaternion(0, 0, 0, 1))
    out = ctrl.advance(np.array([0.5, 0.0], dtype=np.float64), odom)
    assert math.hypot(float(out.linear.x), float(out.linear.y)) == pytest.approx(0.5)


def test_local_planner_odom_delta_passes_estimated_measured_body_twist() -> None:
    lp, recorder = _local_planner_with_recording_holonomic_controller()
    lp._current_odom = _pose_stamped(0.0, 0.0, 0.0, ts=1.0)
    lp._compute_path_following()
    lp._current_odom = _pose_stamped(0.3, -0.1, 0.2, ts=1.5)

    lp._compute_path_following()

    measured = recorder.measurements[-1].twist_body
    vx_w = 0.3 / 0.5
    vy_w = -0.1 / 0.5
    assert measured.linear.x == pytest.approx(math.cos(0.2) * vx_w + math.sin(0.2) * vy_w)
    assert measured.linear.y == pytest.approx(-math.sin(0.2) * vx_w + math.cos(0.2) * vy_w)
    assert measured.angular.z == pytest.approx(0.2 / 0.5)


def test_local_planner_rotation_passes_estimated_measured_body_twist() -> None:
    lp, recorder = _local_planner_with_recording_holonomic_controller()
    lp._current_odom = _pose_stamped(0.0, 0.0, 1.0, ts=1.0)
    lp._compute_initial_rotation()
    lp._current_odom = _pose_stamped(0.2, 0.1, 0.8, ts=1.4)

    lp._compute_initial_rotation()

    measured = recorder.measurements[-1].twist_body
    vx_w = 0.2 / 0.4
    vy_w = 0.1 / 0.4
    assert measured.linear.x == pytest.approx(math.cos(0.8) * vx_w + math.sin(0.8) * vy_w)
    assert measured.linear.y == pytest.approx(-math.sin(0.8) * vx_w + math.cos(0.8) * vy_w)
    assert measured.angular.z == pytest.approx(angle_diff(0.8, 1.0) / 0.4)


def test_local_planner_invalid_dt_passes_zero_measured_body_twist() -> None:
    lp, recorder = _local_planner_with_recording_holonomic_controller()
    lp._current_odom = _pose_stamped(0.0, 0.0, 0.0, ts=5.0)
    lp._compute_path_following()
    lp._current_odom = _pose_stamped(0.2, 0.0, 0.0, ts=5.0)

    lp._compute_path_following()

    measured = recorder.measurements[-1].twist_body
    assert measured.is_zero()


def test_local_planner_lookahead_reference_uses_path_end_progress() -> None:
    g = GlobalConfig(
        local_planner_path_controller="holonomic",
        planner_robot_speed=0.5,
    )
    nav = NavigationMap(g, "gradient")
    lp = LocalPlanner(g, nav, goal_tolerance=0.01)
    path = _path_from_points([(0.0, 0.0), (2.0, 0.0)])
    distancer = PathDistancer(path)
    odom = _pose_stamped(0.2, 0.1, 0.0, ts=10.0)
    current_pos = np.array([odom.position.x, odom.position.y], dtype=np.float64)
    path_speed = lp._path_speed_for_index(distancer, 0, current_pos)

    reference = lp._lookahead_reference_sample(
        distancer,
        odom,
        current_pos,
        path_speed,
    )

    assert reference.pose_plan.position.x == pytest.approx(0.7)
    assert reference.time_s == pytest.approx(11.0)


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


def test_local_planner_uses_configured_goal_decel_for_path_speed_cap() -> None:
    g = GlobalConfig(
        local_planner_path_controller="holonomic",
        planner_robot_speed=2.0,
        local_planner_max_tangent_accel_m_s2=4.0,
        local_planner_goal_decel_m_s2=0.5,
    )
    nav = NavigationMap(g, "gradient")
    lp = LocalPlanner(g, nav, goal_tolerance=0.01)
    path = _path_from_points([(0.0, 0.0), (1.0, 0.0)])
    distancer = PathDistancer(path)
    current_pos = np.array([0.75, 0.0], dtype=np.float64)

    speed = lp._path_speed_for_index(distancer, 0, current_pos)

    assert speed == pytest.approx(0.5, abs=1e-3)


def test_local_planner_path_speed_uses_geometry_and_near_goal_decel_caps() -> None:
    g = GlobalConfig(
        local_planner_path_controller="holonomic",
        planner_robot_speed=2.0,
        local_planner_max_normal_accel_m_s2=0.5,
        local_planner_goal_decel_m_s2=1.0,
    )
    nav = NavigationMap(g, "gradient")
    lp = LocalPlanner(g, nav, goal_tolerance=0.01)
    path = _path_from_points([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (2.0, 1.0)])
    distancer = PathDistancer(path)

    corner_pos = np.array([1.0, 0.0], dtype=np.float64)
    corner_index = distancer.find_closest_point_index(corner_pos)
    corner_speed = lp._path_speed_for_index(distancer, corner_index, corner_pos)

    near_goal_pos = np.array([1.95, 1.0], dtype=np.float64)
    near_goal_index = distancer.find_closest_point_index(near_goal_pos)
    near_goal_speed = lp._path_speed_for_index(distancer, near_goal_index, near_goal_pos)

    right_angle_radius_m = math.sqrt(0.5)
    geometry_cap_m_s = math.sqrt(g.local_planner_max_normal_accel_m_s2 * right_angle_radius_m)
    goal_decel_cap_m_s = math.sqrt(
        2.0 * g.local_planner_goal_decel_m_s2 * distancer.distance_to_goal(near_goal_pos)
    )
    assert corner_speed == pytest.approx(geometry_cap_m_s)
    assert near_goal_speed == pytest.approx(goal_decel_cap_m_s, abs=1e-3)
    assert near_goal_speed < corner_speed


def test_local_planner_path_speed_anticipates_corner_tangent_decel() -> None:
    g = GlobalConfig(
        local_planner_path_controller="holonomic",
        planner_robot_speed=2.0,
        local_planner_max_tangent_accel_m_s2=0.5,
        local_planner_max_normal_accel_m_s2=0.1,
    )
    nav = NavigationMap(g, "gradient")
    lp = LocalPlanner(g, nav, goal_tolerance=0.01)
    path = _path_from_points([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
    distancer = PathDistancer(path)

    before_corner_pos = np.array([0.85, 0.0], dtype=np.float64)
    before_corner_speed = lp._path_speed_for_index(
        distancer,
        distancer.find_closest_point_index(before_corner_pos),
        before_corner_pos,
    )

    assert before_corner_speed < g.planner_robot_speed
    assert before_corner_speed < 1.0


def test_local_planner_in_the_loop_harness_reaches_arrival_on_straight_line(
    tmp_path: FsPath,
) -> None:
    result = _run_local_planner_harness(
        tmp_path,
        points=[(0.1, 0.0), (1.2, 0.0)],
        speed_m_s=1.0,
    )

    assert "arrived" in result.stop_messages
    assert result.command_history
    assert math.hypot(result.final_x_m - 1.2, result.final_y_m) < 0.15


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

    assert "arrived" in result.stop_messages


def test_local_planner_in_the_loop_harness_supports_2mps_right_angle_with_conservative_profile(
    tmp_path: FsPath,
) -> None:
    result = _run_local_planner_harness(
        tmp_path,
        points=[(0.1, 0.0), (0.55, 0.0), (0.55, 0.55), (0.9, 0.55)],
        speed_m_s=2.0,
        max_tangent_accel_m_s2=0.5,
        max_normal_accel_m_s2=0.1,
        max_goal_decel_m_s2=0.5,
        max_ticks=420,
    )
    commanded_speeds = [_planar_speed_m_s(cmd) for cmd in result.command_history]

    assert "arrived" in result.stop_messages
    assert commanded_speeds
    assert max(commanded_speeds) < 0.75


def test_local_planner_in_the_loop_harness_decelerates_near_goal(
    tmp_path: FsPath,
) -> None:
    result = _run_local_planner_harness(
        tmp_path,
        points=[(0.1, 0.0), (1.0, 0.0)],
        speed_m_s=1.2,
        max_ticks=300,
    )
    commanded_speeds = [_planar_speed_m_s(cmd) for cmd in result.command_history]
    moving_speeds = [speed for speed in commanded_speeds if speed > 0.05]

    assert "arrived" in result.stop_messages
    assert moving_speeds
    assert max(moving_speeds[: len(moving_speeds) // 2]) > 0.7
    assert min(moving_speeds[-10:]) < 0.55
    assert min(moving_speeds[-10:]) < 0.6 * max(moving_speeds)
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
        abs(float(cmd.angular.z)) > 0.05 and _planar_speed_m_s(cmd) < 0.05
        for cmd in result.command_history[:20]
    )
    assert abs(result.final_yaw_rad) < 0.35


def test_make_local_path_controller_holonomic_velocity_damping_reduces_planar_command() -> None:
    # Aligned pose so kp/ky contribute nothing; the only difference is the velocity
    # damping gain wired from GlobalConfig through the factory into the inner law.
    reference = TrajectoryReferenceSample(
        time_s=1.0,
        pose_plan=Pose(position=[0.0, 0.0, 0.0], orientation=_yaw_quaternion(0.0)),
        twist_body=Twist(linear=[0.5, 0.2, 0.0]),
    )
    odom = _pose_stamped(0.0, 0.0, 0.0, ts=1.0)
    measured_body_twist = Twist(linear=[1.1, 0.6, 0.0])

    def _factory(k_velocity: float) -> Any:
        return make_local_path_controller(
            GlobalConfig(
                local_planner_path_controller="holonomic",
                local_planner_holonomic_kp=0.0,
                local_planner_holonomic_ky=0.0,
                local_planner_holonomic_kv=k_velocity,
                local_planner_holonomic_kw=0.0,
                # Generous slew headroom so the first-tick accel clamp does not mask the gain.
                local_planner_max_planar_cmd_accel_m_s2=8.0,
            ),
            speed=1.0,
            control_frequency=10.0,
        )

    undamped = _factory(0.0).advance_reference(reference, odom, measured_body_twist)
    damped = _factory(0.5).advance_reference(reference, odom, measured_body_twist)

    assert undamped.linear.x == pytest.approx(0.5)
    assert undamped.linear.y == pytest.approx(0.2)
    # 0.5 * (ref - measured) damping pulls the command down to (0.2, 0.0).
    assert damped.linear.x == pytest.approx(0.2)
    assert damped.linear.y == pytest.approx(0.0)
    assert math.hypot(damped.linear.x, damped.linear.y) < math.hypot(
        undamped.linear.x, undamped.linear.y
    )


def _narrow_corridor_navigation_map(global_config: GlobalConfig) -> NavigationMap:
    grid = np.zeros((80, 80), dtype=np.int8)
    for row in range(grid.shape[0]):
        world_y = -2.0 + row * 0.05
        if abs(world_y) > 0.15:
            grid[row, :] = CostValues.OCCUPIED
    nav = NavigationMap(global_config, "gradient")
    nav.update(
        OccupancyGrid(
            grid=grid,
            resolution=0.05,
            origin=Pose(-2.0, -2.0, 0.0),
            frame_id="map",
            ts=1.0,
        )
    )
    return nav


def test_holonomic_yaw_lock_reference_drives_along_backward_path() -> None:
    g = GlobalConfig(
        local_planner_path_controller="holonomic",
        local_planner_holonomic_kp=2.0,
        local_planner_holonomic_ky=1.0,
        robot_rotation_diameter=0.6,
    )
    nav = _narrow_corridor_navigation_map(g)
    lp = LocalPlanner(g, nav, goal_tolerance=0.01)
    path = _path_from_points([(0.0, 0.0), (-0.8, 0.0)])
    distancer = PathDistancer(path)
    odom = _pose_stamped(0.0, 0.0, 0.0, ts=1.0)
    current_pos = np.array([odom.position.x, odom.position.y], dtype=np.float64)
    path_speed = lp._path_speed_for_index(distancer, 0, current_pos)

    reference = lp._lookahead_reference_sample(
        distancer,
        odom,
        current_pos,
        path_speed,
        yaw_lock_rad=0.0,
    )
    cmd = lp._controller.advance_reference(
        reference,
        odom,
        Twist(),
    )

    assert reference.pose_plan.orientation.euler[2] == pytest.approx(0.0)
    assert float(cmd.linear.x) < -0.05
    assert abs(float(cmd.angular.z)) < 0.05


def test_local_planner_narrow_corridor_retreats_without_spinning(tmp_path: FsPath) -> None:
    rate_hz = 60.0
    dt_s = 1.0 / rate_hz
    global_config = GlobalConfig(
        local_planner_path_controller="holonomic",
        local_planner_control_rate_hz=rate_hz,
        planner_robot_speed=0.45,
        robot_width=0.25,
        robot_rotation_diameter=0.6,
        local_planner_holonomic_kp=2.0,
        local_planner_holonomic_ky=1.0,
        local_planner_max_planar_cmd_accel_m_s2=8.0,
        local_planner_max_yaw_accel_rad_s2=8.0,
    )
    planner = LocalPlanner(
        global_config,
        _narrow_corridor_navigation_map(global_config),
        goal_tolerance=0.08,
    )
    plant_x_m, plant_y_m, plant_yaw_rad = 0.0, 0.0, 0.0
    latest_cmd = Twist()
    command_history: list[Twist] = []
    stop_messages: list[str] = []

    def _on_cmd_vel(cmd: Twist) -> None:
        nonlocal latest_cmd
        latest_cmd = Twist(cmd)
        command_history.append(Twist(cmd))

    cmd_sub = planner.cmd_vel.subscribe(_on_cmd_vel)
    stop_sub = planner.stopped_navigating.subscribe(stop_messages.append)
    sim_time_s = 1.0

    try:
        planner.handle_odom(_pose_stamped(plant_x_m, plant_y_m, plant_yaw_rad, ts=sim_time_s))
        planner.start_planning(_path_from_points([(0.0, 0.0), (-0.8, 0.0)]))
        for _ in range(420):
            if "arrived" in stop_messages:
                break
            time.sleep(dt_s * 1.1)
            plant_x_m, plant_y_m, plant_yaw_rad = _integrate_holonomic_pose(
                plant_x_m, plant_y_m, plant_yaw_rad, latest_cmd, dt_s
            )
            sim_time_s += dt_s
            planner.handle_odom(
                _pose_stamped(plant_x_m, plant_y_m, plant_yaw_rad, ts=sim_time_s)
            )
    finally:
        planner.stop()
        cmd_sub.dispose()
        stop_sub.dispose()

    assert "arrived" in stop_messages
    assert plant_x_m < -0.55
    assert abs(plant_yaw_rad) < 0.2
    assert all(abs(float(cmd.angular.z)) < 0.12 for cmd in command_history[:40])


def test_make_local_path_controller_differential_ignores_measured_body_twist() -> None:
    # The differential branch returns a PController, which has no velocity-damping term;
    # large kv/kw in config must not change its output when measured twist is supplied.
    controller = make_local_path_controller(
        GlobalConfig(
            local_planner_path_controller="differential",
            local_planner_holonomic_kv=5.0,
            local_planner_holonomic_kw=5.0,
        ),
        speed=1.0,
        control_frequency=10.0,
    )
    reference = TrajectoryReferenceSample(
        time_s=1.0,
        pose_plan=Pose(position=[1.0, 0.0, 0.0], orientation=_yaw_quaternion(0.0)),
        twist_body=Twist(linear=[1.0, 0.0, 0.0]),
    )
    odom = _pose_stamped(0.0, 0.0, 0.0, ts=1.0)
    measured_body_twist = Twist(linear=[1.1, 0.6, 0.0], angular=[0.0, 0.0, 0.9])

    without_measurement = controller.advance_reference(reference, odom)
    with_measurement = controller.advance_reference(reference, odom, measured_body_twist)

    assert math.hypot(without_measurement.linear.x, without_measurement.linear.y) > 0.0
    assert with_measurement.linear.x == pytest.approx(without_measurement.linear.x)
    assert with_measurement.linear.y == pytest.approx(without_measurement.linear.y)
    assert with_measurement.angular.z == pytest.approx(without_measurement.angular.z)
