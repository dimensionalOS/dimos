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

from collections.abc import Generator
from dataclasses import dataclass, field
import math

import numpy as np
from pydantic import ValidationError
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.navigation.obstacle_avoidance.module import (
    ObstacleAvoidance,
    ObstacleAvoidanceConfig,
    _dynamic_stop_distance_m,
)


@dataclass
class Captured:
    commands: list[Twist] = field(default_factory=list)
    states: list[bool] = field(default_factory=list)


@pytest.fixture()
def guard_and_captured() -> Generator[tuple[ObstacleAvoidance, Captured], None, None]:
    guard = ObstacleAvoidance(
        enabled=True,
        stop_distance_m=1.2,
        resume_distance_m=1.4,
        reaction_time_s=0.5,
        braking_deceleration_m_s2=0.6,
        stop_safety_margin_m=0.2,
        lateral_margin_m=0.1,
        clear_costmap_count=2,
        costmap_timeout_s=1.0,
        odometry_timeout_s=0.5,
        g={"robot_width": 0.9, "robot_rotation_diameter": 0.9},
    )
    captured = Captured()
    command_unsub = guard.nav_cmd_vel.subscribe(captured.commands.append)
    state_unsub = guard.obstacle_avoidance_active.subscribe(
        lambda msg: captured.states.append(msg.data)
    )
    try:
        yield guard, captured
    finally:
        command_unsub()
        state_unsub()
        guard._close_module()


def _command(*, linear_x: float = 0.5, linear_y: float = 0.0, angular_z: float = 0.0) -> Twist:
    return Twist(
        linear=Vector3(linear_x, linear_y, 0.0),
        angular=Vector3(0.0, 0.0, angular_z),
    )


def _odometry(*, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> Odometry:
    return Odometry(
        frame_id="map",
        child_frame_id="base_link",
        pose=Pose(
            position=[x, y, 0.0],
            orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
        ),
    )


def _costmap(
    *,
    ts: float,
    occupied: tuple[float, float] | None = None,
    cell_value: int = CostValues.OCCUPIED,
) -> OccupancyGrid:
    resolution = 0.05
    origin = Pose(position=[-2.0, -2.0, 0.0])
    grid = np.zeros((80, 80), dtype=np.int8)
    if occupied is not None:
        col = int((occupied[0] - origin.position.x) / resolution)
        row = int((occupied[1] - origin.position.y) / resolution)
        grid[row, col] = cell_value
    return OccupancyGrid(
        grid=grid,
        resolution=resolution,
        origin=origin,
        frame_id="map",
        ts=ts,
    )


def _confirm_initial_clear(guard: ObstacleAvoidance) -> None:
    guard._on_odometry(_odometry())
    guard._on_costmap(_costmap(ts=1.0))
    guard._on_costmap(_costmap(ts=2.0))
    assert guard.blocked is False


def test_config_requires_spatial_hysteresis() -> None:
    with pytest.raises(ValidationError, match="resume_distance_m"):
        ObstacleAvoidanceConfig(stop_distance_m=1.2, resume_distance_m=1.2)


def test_dynamic_stop_distance_uses_speed_latency_braking_and_footprint() -> None:
    assert _dynamic_stop_distance_m(
        _command(linear_x=1.0),
        minimum_distance_m=1.2,
        reaction_time_s=0.5,
        braking_deceleration_m_s2=0.6,
        footprint_radius_m=0.55,
        safety_margin_m=0.2,
    ) == pytest.approx(2.083333)


def test_disabled_filter_passes_command_without_sensor_inputs() -> None:
    guard = ObstacleAvoidance(enabled=False)
    captured: list[Twist] = []
    unsubscribe = guard.nav_cmd_vel.subscribe(captured.append)
    command = _command(linear_x=0.4, angular_z=0.2)
    try:
        guard._on_raw_nav_cmd_vel(command)
        assert captured == [command]
    finally:
        unsubscribe()
        guard._close_module()


def test_clear_forward_command_passes_through(guard_and_captured) -> None:
    guard, captured = guard_and_captured
    _confirm_initial_clear(guard)
    command = _command()

    guard._on_raw_nav_cmd_vel(command)

    assert captured.commands[-1] == command


def test_forward_obstacle_immediately_outputs_zero(guard_and_captured) -> None:
    guard, captured = guard_and_captured
    _confirm_initial_clear(guard)
    guard._on_raw_nav_cmd_vel(_command())

    guard._on_costmap(_costmap(ts=3.0, occupied=(0.8, 0.0)))

    assert guard.blocked is True
    assert captured.commands[-1].is_zero()
    assert captured.states[-1] is True


def test_obstacle_behind_robot_is_ignored_for_forward_motion(guard_and_captured) -> None:
    guard, captured = guard_and_captured
    _confirm_initial_clear(guard)
    command = _command()
    guard._on_raw_nav_cmd_vel(command)

    guard._on_costmap(_costmap(ts=3.0, occupied=(-0.4, 0.0)))

    assert guard.blocked is False
    assert captured.commands[-1] == command


def test_obstacle_outside_lateral_corridor_is_ignored(guard_and_captured) -> None:
    guard, captured = guard_and_captured
    _confirm_initial_clear(guard)
    command = _command()
    guard._on_raw_nav_cmd_vel(command)

    guard._on_costmap(_costmap(ts=3.0, occupied=(0.8, 0.6)))

    assert guard.blocked is False
    assert captured.commands[-1] == command


def test_obstacle_beyond_stop_distance_is_ignored_while_clear(guard_and_captured) -> None:
    guard, captured = guard_and_captured
    _confirm_initial_clear(guard)
    command = _command()
    guard._on_raw_nav_cmd_vel(command)

    guard._on_costmap(_costmap(ts=3.0, occupied=(1.3, 0.0)))

    assert guard.blocked is False
    assert captured.commands[-1] == command


def test_unknown_cell_is_ignored(guard_and_captured) -> None:
    guard, captured = guard_and_captured
    _confirm_initial_clear(guard)
    command = _command()
    guard._on_raw_nav_cmd_vel(command)

    guard._on_costmap(_costmap(ts=3.0, occupied=(0.8, 0.0), cell_value=CostValues.UNKNOWN))

    assert guard.blocked is False
    assert captured.commands[-1] == command


def test_robot_yaw_rotates_forward_envelope(guard_and_captured) -> None:
    guard, captured = guard_and_captured
    guard._on_odometry(_odometry(yaw=math.pi / 2.0))
    guard._on_costmap(_costmap(ts=1.0))
    guard._on_costmap(_costmap(ts=2.0))
    guard._on_raw_nav_cmd_vel(_command())

    guard._on_costmap(_costmap(ts=3.0, occupied=(0.0, 0.8)))

    assert guard.blocked is True
    assert captured.commands[-1].is_zero()


def test_rotation_obstacle_outputs_zero(guard_and_captured) -> None:
    guard, captured = guard_and_captured
    _confirm_initial_clear(guard)
    guard._on_raw_nav_cmd_vel(_command(linear_x=0.0, angular_z=0.5))

    guard._on_costmap(_costmap(ts=3.0, occupied=(0.0, 0.5)))

    assert guard.blocked is True
    assert captured.commands[-1].is_zero()


def test_mixed_forward_rotation_checks_rotation_envelope(guard_and_captured) -> None:
    guard, captured = guard_and_captured
    _confirm_initial_clear(guard)
    guard._on_raw_nav_cmd_vel(_command(linear_x=0.5, angular_z=0.5))

    guard._on_costmap(_costmap(ts=3.0, occupied=(-0.3, 0.3)))

    assert guard.blocked is True
    assert captured.commands[-1].is_zero()


def test_curved_command_checks_swept_arc_not_straight_corridor(guard_and_captured) -> None:
    guard, captured = guard_and_captured
    _confirm_initial_clear(guard)
    guard._on_raw_nav_cmd_vel(_command(linear_x=1.5, angular_z=0.75))

    guard._on_costmap(_costmap(ts=3.0, occupied=(1.7, 0.9)))

    assert guard.blocked is True
    assert captured.commands[-1].is_zero()


def test_curved_command_ignores_obstacle_away_from_swept_arc(guard_and_captured) -> None:
    guard, captured = guard_and_captured
    _confirm_initial_clear(guard)
    command = _command(linear_x=1.5, angular_z=0.75)
    guard._on_raw_nav_cmd_vel(command)

    guard._on_costmap(_costmap(ts=3.0, occupied=(0.0, 1.5)))

    assert guard.blocked is False
    assert captured.commands[-1] == command


def test_recovery_requires_distinct_clear_costmaps(guard_and_captured) -> None:
    guard, captured = guard_and_captured
    _confirm_initial_clear(guard)
    command = _command()
    guard._on_raw_nav_cmd_vel(command)
    guard._on_costmap(_costmap(ts=3.0, occupied=(0.8, 0.0)))
    guard._on_costmap(_costmap(ts=4.0, occupied=(1.3, 0.0)))
    guard._on_costmap(_costmap(ts=5.0, occupied=(1.3, 0.0)))
    assert guard.blocked is True

    first_clear = _costmap(ts=6.0)

    guard._on_costmap(first_clear)
    guard._on_costmap(first_clear)
    assert guard.blocked is True

    guard._on_costmap(_costmap(ts=7.0))

    assert guard.blocked is False
    assert captured.commands[-1] == command
    assert captured.states[-1] is False


def test_obstacle_inside_resume_distance_keeps_filter_blocked(guard_and_captured) -> None:
    guard, _ = guard_and_captured
    _confirm_initial_clear(guard)
    guard._on_raw_nav_cmd_vel(_command())
    guard._on_costmap(_costmap(ts=3.0, occupied=(0.8, 0.0)))

    guard._on_costmap(_costmap(ts=4.0, occupied=(1.3, 0.0)))
    guard._on_costmap(_costmap(ts=5.0, occupied=(1.3, 0.0)))

    assert guard.blocked is True


def test_missing_costmap_fails_closed(guard_and_captured) -> None:
    guard, captured = guard_and_captured
    guard._on_odometry(_odometry())

    guard._on_raw_nav_cmd_vel(_command())

    assert guard.blocked is True
    assert captured.commands[-1].is_zero()


def test_missing_odometry_fails_closed(guard_and_captured) -> None:
    guard, captured = guard_and_captured
    guard._on_costmap(_costmap(ts=1.0))

    guard._on_raw_nav_cmd_vel(_command())

    assert guard.blocked is True
    assert captured.commands[-1].is_zero()


def test_stale_costmap_blocks_nonzero_command(guard_and_captured) -> None:
    guard, captured = guard_and_captured
    _confirm_initial_clear(guard)
    assert guard._costmap_rx_monotonic is not None
    guard._costmap_rx_monotonic -= guard.config.costmap_timeout_s + 0.1

    guard._on_raw_nav_cmd_vel(_command())

    assert guard.blocked is True
    assert captured.commands[-1].is_zero()


def test_stale_odometry_blocks_nonzero_command(guard_and_captured) -> None:
    guard, captured = guard_and_captured
    _confirm_initial_clear(guard)
    assert guard._odometry_rx_monotonic is not None
    guard._odometry_rx_monotonic -= guard.config.odometry_timeout_s + 0.1

    guard._on_raw_nav_cmd_vel(_command())

    assert guard.blocked is True
    assert captured.commands[-1].is_zero()


def test_odometry_outside_costmap_fails_closed(guard_and_captured) -> None:
    guard, captured = guard_and_captured
    _confirm_initial_clear(guard)
    guard._on_odometry(_odometry(x=2.1))

    guard._on_raw_nav_cmd_vel(_command())

    assert guard.blocked is True
    assert captured.commands[-1].is_zero()


def test_nonfinite_autonomous_command_fails_closed(guard_and_captured) -> None:
    guard, captured = guard_and_captured
    _confirm_initial_clear(guard)

    guard._on_raw_nav_cmd_vel(_command(linear_x=math.nan))

    assert guard.blocked is True
    assert captured.commands[-1].is_zero()


def test_reverse_autonomous_command_fails_closed(guard_and_captured) -> None:
    guard, captured = guard_and_captured
    _confirm_initial_clear(guard)

    guard._on_raw_nav_cmd_vel(_command(linear_x=-0.2))

    assert guard.blocked is True
    assert captured.commands[-1].is_zero()


def test_lateral_autonomous_command_fails_closed(guard_and_captured) -> None:
    guard, captured = guard_and_captured
    _confirm_initial_clear(guard)

    guard._on_raw_nav_cmd_vel(_command(linear_y=0.2))

    assert guard.blocked is True
    assert captured.commands[-1].is_zero()


def test_zero_command_remains_exactly_zero_while_blocked(guard_and_captured) -> None:
    guard, captured = guard_and_captured

    guard._on_raw_nav_cmd_vel(_command(linear_x=0.0))

    assert guard.blocked is True
    assert captured.commands[-1] == Twist()


def test_diagnostic_state_is_published_only_on_transitions(guard_and_captured) -> None:
    guard, captured = guard_and_captured
    _confirm_initial_clear(guard)
    command = _command()
    guard._on_raw_nav_cmd_vel(command)

    guard._on_costmap(_costmap(ts=3.0, occupied=(0.8, 0.0)))
    guard._on_costmap(_costmap(ts=4.0, occupied=(0.8, 0.0)))
    guard._on_raw_nav_cmd_vel(command)
    guard._on_costmap(_costmap(ts=5.0))
    guard._on_costmap(_costmap(ts=6.0))
    guard._on_costmap(_costmap(ts=7.0))

    assert captured.states == [False, True, False]
