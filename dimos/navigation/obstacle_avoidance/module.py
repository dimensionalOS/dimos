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

"""Near-field stop filter for autonomous navigation velocity commands."""

from __future__ import annotations

import math
from threading import RLock
import time

from dimos_lcm.std_msgs import Bool
import numpy as np
from pydantic import Field, model_validator
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_MOTION_EPSILON = 1e-6


class ObstacleAvoidanceConfig(ModuleConfig):
    enabled: bool = False
    stop_distance_m: float = Field(default=1.2, gt=0.0)
    resume_distance_m: float = Field(default=1.4, gt=0.0)
    reaction_time_s: float = Field(default=0.0, ge=0.0)
    braking_deceleration_m_s2: float = Field(default=1.0, gt=0.0)
    stop_safety_margin_m: float = Field(default=0.0, ge=0.0)
    lateral_margin_m: float = Field(default=0.1, ge=0.0)
    clear_costmap_count: int = Field(default=2, ge=1)
    costmap_timeout_s: float = Field(default=1.0, gt=0.0)
    odometry_timeout_s: float = Field(default=0.5, gt=0.0)

    @model_validator(mode="after")
    def validate_hysteresis(self) -> ObstacleAvoidanceConfig:
        if self.resume_distance_m <= self.stop_distance_m:
            raise ValueError("resume_distance_m must be greater than stop_distance_m")
        return self


def _is_supported_motion(command: Twist) -> bool:
    values = (
        command.linear.x,
        command.linear.y,
        command.linear.z,
        command.angular.x,
        command.angular.y,
        command.angular.z,
    )
    return (
        all(math.isfinite(value) for value in values)
        and command.linear.x >= -_MOTION_EPSILON
        and abs(command.linear.y) <= _MOTION_EPSILON
        and abs(command.linear.z) <= _MOTION_EPSILON
        and abs(command.angular.x) <= _MOTION_EPSILON
        and abs(command.angular.y) <= _MOTION_EPSILON
    )


def _dynamic_stop_distance_m(
    command: Twist,
    *,
    minimum_distance_m: float,
    reaction_time_s: float,
    braking_deceleration_m_s2: float,
    footprint_radius_m: float,
    safety_margin_m: float,
) -> float:
    speed_m_s = max(0.0, command.linear.x)
    stopping_distance_m = (
        speed_m_s * reaction_time_s
        + speed_m_s**2 / (2.0 * braking_deceleration_m_s2)
        + footprint_radius_m
        + safety_margin_m
    )
    return max(minimum_distance_m, stopping_distance_m)


def _trajectory_centers(
    command: Twist, travel_distance_m: float, sample_spacing_m: float
) -> tuple[np.ndarray, np.ndarray]:
    sample_count = max(2, math.ceil(travel_distance_m / sample_spacing_m) + 1)
    distance = np.linspace(0.0, travel_distance_m, sample_count)
    curvature = command.angular.z / command.linear.x
    if abs(curvature) <= _MOTION_EPSILON:
        return distance, np.zeros_like(distance)
    return (
        np.sin(curvature * distance) / curvature,
        (1.0 - np.cos(curvature * distance)) / curvature,
    )


def _has_obstacle_in_envelope(
    costmap: OccupancyGrid,
    odometry: Odometry,
    command: Twist,
    *,
    forward_distance_m: float,
    robot_width_m: float,
    robot_rotation_diameter_m: float,
    lateral_margin_m: float,
    rotation_hysteresis_m: float = 0.0,
) -> bool:
    """Check occupied cells in the command-dependent robot-frame envelope."""
    if costmap.grid.size == 0 or costmap.resolution <= 0:
        raise ValueError("costmap is empty or has invalid resolution")
    rotation_radius_m = robot_rotation_diameter_m / 2.0 + lateral_margin_m + rotation_hysteresis_m
    forward_motion = command.linear.x > _MOTION_EPSILON
    rotation_motion = abs(command.angular.z) > _MOTION_EPSILON
    idle = not forward_motion and not rotation_motion
    forward_radius_m = robot_width_m / 2.0 + lateral_margin_m
    trajectory_radius_m = (
        max(forward_radius_m, rotation_radius_m) if rotation_motion else forward_radius_m
    )
    search_radius_m = max(
        rotation_radius_m,
        forward_distance_m + trajectory_radius_m if forward_motion or idle else 0.0,
    )
    resolution = costmap.resolution
    origin_x = costmap.origin.position.x
    origin_y = costmap.origin.position.y
    map_max_x = origin_x + costmap.grid.shape[1] * resolution
    map_max_y = origin_y + costmap.grid.shape[0] * resolution
    if not (origin_x <= odometry.x < map_max_x and origin_y <= odometry.y < map_max_y):
        raise ValueError("odometry lies outside the current costmap")

    min_col = max(0, math.floor((odometry.x - search_radius_m - origin_x) / resolution))
    max_col = min(
        costmap.grid.shape[1],
        math.ceil((odometry.x + search_radius_m - origin_x) / resolution) + 1,
    )
    min_row = max(0, math.floor((odometry.y - search_radius_m - origin_y) / resolution))
    max_row = min(
        costmap.grid.shape[0],
        math.ceil((odometry.y + search_radius_m - origin_y) / resolution) + 1,
    )
    if min_col >= max_col or min_row >= max_row:
        raise ValueError("odometry lies outside the current costmap")

    local_grid = costmap.grid[min_row:max_row, min_col:max_col]
    occupied_rows, occupied_cols = np.nonzero(local_grid == CostValues.OCCUPIED)
    if occupied_rows.size == 0:
        return False

    world_x = origin_x + (occupied_cols + min_col + 0.5) * resolution
    world_y = origin_y + (occupied_rows + min_row + 0.5) * resolution
    dx = world_x - odometry.x
    dy = world_y - odometry.y
    cos_yaw = math.cos(odometry.yaw)
    sin_yaw = math.sin(odometry.yaw)
    robot_x = cos_yaw * dx + sin_yaw * dy
    robot_y = -sin_yaw * dx + cos_yaw * dy

    if forward_motion:
        centerline_distance_m = max(0.0, forward_distance_m - trajectory_radius_m)
        center_x, center_y = _trajectory_centers(
            command,
            centerline_distance_m,
            sample_spacing_m=max(resolution, 0.01),
        )
        relevant = np.ones_like(robot_x, dtype=np.bool_)
        if not rotation_motion:
            relevant = robot_x >= 0.0
        for x, y in zip(center_x, center_y, strict=True):
            in_footprint = relevant & (
                (robot_x - x) ** 2 + (robot_y - y) ** 2 <= trajectory_radius_m**2
            )
            if bool(np.any(in_footprint)):
                return True
    elif idle:
        in_forward_envelope = (
            (robot_x >= 0.0)
            & (robot_x <= forward_distance_m)
            & (np.abs(robot_y) <= forward_radius_m)
        )
        if bool(np.any(in_forward_envelope)):
            return True

    if not forward_motion and (rotation_motion or idle):
        in_rotation_envelope = dx**2 + dy**2 <= rotation_radius_m**2
        if bool(np.any(in_rotation_envelope)):
            return True

    return False


class ObstacleAvoidance(Module):
    """Pass autonomous velocity through unless the near-field envelope is blocked."""

    config: ObstacleAvoidanceConfig

    raw_nav_cmd_vel: In[Twist]
    global_costmap: In[OccupancyGrid]
    odometry: In[Odometry]

    nav_cmd_vel: Out[Twist]
    obstacle_avoidance_active: Out[Bool]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._latest_costmap: OccupancyGrid | None = None
        self._latest_odometry: Odometry | None = None
        self._costmap_rx_monotonic: float | None = None
        self._odometry_rx_monotonic: float | None = None
        self._last_raw_command = Twist()
        self._blocked = self.config.enabled
        self._clear_costmaps = 0

    @property
    def blocked(self) -> bool:
        with self._lock:
            return self._blocked

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.raw_nav_cmd_vel.subscribe(self._on_raw_nav_cmd_vel))
        )
        self.register_disposable(Disposable(self.global_costmap.subscribe(self._on_costmap)))
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self.obstacle_avoidance_active.publish(Bool(data=self._blocked))

    def _on_odometry(self, msg: Odometry) -> None:
        with self._lock:
            self._latest_odometry = msg
            self._odometry_rx_monotonic = time.monotonic()

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        now = time.monotonic()
        with self._lock:
            if self._latest_costmap is not None and msg.ts == self._latest_costmap.ts:
                return
            self._latest_costmap = msg
            self._costmap_rx_monotonic = now
            command = Twist(self._last_raw_command)

        if not self.config.enabled:
            return

        input_error = self._input_error(now)
        if input_error is not None:
            self._activate_stop(input_error)
            return
        if not _is_supported_motion(command):
            self._activate_stop("unsupported autonomous motion")
            return

        try:
            obstacle = self._detect_obstacle(command)
        except ValueError as error:
            self._activate_stop(str(error))
            return

        if obstacle:
            self._activate_stop("occupied cell in near-field envelope")
            return

        with self._lock:
            if not self._blocked:
                self._clear_costmaps = 0
                return
            self._clear_costmaps += 1
            clear = self._clear_costmaps >= self.config.clear_costmap_count

        if clear:
            self._set_blocked(False, "clear costmaps confirmed")
            self.nav_cmd_vel.publish(command)

    def _on_raw_nav_cmd_vel(self, msg: Twist) -> None:
        command = Twist(msg)
        with self._lock:
            self._last_raw_command = command

        if not self.config.enabled:
            self.nav_cmd_vel.publish(command)
            return
        if command.is_zero():
            self.nav_cmd_vel.publish(Twist())
            return
        if not _is_supported_motion(command):
            self._activate_stop("unsupported autonomous motion")
            return

        input_error = self._input_error(time.monotonic())
        if input_error is not None:
            self._activate_stop(input_error)
            return

        try:
            obstacle = self._detect_obstacle(command)
        except ValueError as error:
            self._activate_stop(str(error))
            return

        if obstacle:
            self._activate_stop("occupied cell in near-field envelope")
        elif self.blocked:
            self.nav_cmd_vel.publish(Twist())
        else:
            self.nav_cmd_vel.publish(command)

    def _input_error(self, now: float) -> str | None:
        with self._lock:
            costmap = self._latest_costmap
            odometry = self._latest_odometry
            costmap_rx = self._costmap_rx_monotonic
            odometry_rx = self._odometry_rx_monotonic

        if costmap is None or costmap_rx is None:
            return "missing costmap"
        if odometry is None or odometry_rx is None:
            return "missing odometry"
        if now - costmap_rx > self.config.costmap_timeout_s:
            return "stale costmap"
        if now - odometry_rx > self.config.odometry_timeout_s:
            return "stale odometry"
        return None

    def _detect_obstacle(self, command: Twist) -> bool:
        with self._lock:
            costmap = self._latest_costmap
            odometry = self._latest_odometry
            blocked = self._blocked

        assert costmap is not None
        assert odometry is not None

        rotation_motion = abs(command.angular.z) > _MOTION_EPSILON
        footprint_radius_m = (
            max(self.config.g.robot_width, self.config.g.robot_rotation_diameter) / 2.0
            if rotation_motion
            else self.config.g.robot_width / 2.0
        )
        stop_distance_m = _dynamic_stop_distance_m(
            command,
            minimum_distance_m=self.config.stop_distance_m,
            reaction_time_s=self.config.reaction_time_s,
            braking_deceleration_m_s2=self.config.braking_deceleration_m_s2,
            footprint_radius_m=footprint_radius_m + self.config.lateral_margin_m,
            safety_margin_m=self.config.stop_safety_margin_m,
        )
        hysteresis_m = self.config.resume_distance_m - self.config.stop_distance_m
        forward_distance_m = stop_distance_m + hysteresis_m if blocked else stop_distance_m
        rotation_hysteresis_m = hysteresis_m if blocked else 0.0
        return _has_obstacle_in_envelope(
            costmap,
            odometry,
            command,
            forward_distance_m=forward_distance_m,
            robot_width_m=self.config.g.robot_width,
            robot_rotation_diameter_m=self.config.g.robot_rotation_diameter,
            lateral_margin_m=self.config.lateral_margin_m,
            rotation_hysteresis_m=rotation_hysteresis_m,
        )

    def _activate_stop(self, reason: str) -> None:
        with self._lock:
            self._clear_costmaps = 0
        self._set_blocked(True, reason)
        self.nav_cmd_vel.publish(Twist())

    def _set_blocked(self, blocked: bool, reason: str) -> None:
        with self._lock:
            if self._blocked == blocked:
                return
            self._blocked = blocked

        logger.info("Obstacle avoidance state changed.", blocked=blocked, reason=reason)
        self.obstacle_avoidance_active.publish(Bool(data=blocked))
