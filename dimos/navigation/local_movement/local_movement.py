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

"""
Local Movement Controller

Safe short-range body-frame relative movement controller with
collision checking and PID control.

Accepts (dx, dy, dtheta) commands in the robot body frame, transforms
them to world-frame goals, and drives to them at 50 Hz while checking
terrain for obstacles on all axes.

State Machine:
    IDLE ──move──► MOVING ──reached──► COMPLETED
                     │                  timeout──► TIMEOUT
                     │
                  no progress──► STUCK ──progress──► MOVING

Topics (ROS-equivalent names from launch file):
    Subscribes: /state_estimation, /local_movement, /terrain_map
    Publishes:  /cmd_vel, /local_movement/status
"""

from dataclasses import dataclass
from enum import IntEnum
import math
import threading
import time
from typing import Any

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs import Pose, TwistStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.msgs.std_msgs.Int8 import Int8
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import normalize_angle

logger = setup_logger()


class Status(IntEnum):
    IDLE = 0
    MOVING = 1
    COMPLETED = 2
    TIMEOUT = 3
    STUCK = 4


@dataclass
class _PIDState:
    integral: float = 0.0
    prev_error: float = 0.0
    first: bool = True

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0
        self.first = True


@dataclass
class LocalMovementConfig(ModuleConfig):
    """Configuration for local movement controller."""

    # PID gains
    kp_linear: float = 1.0
    ki_linear: float = 0.0
    kd_linear: float = 0.1
    kp_angular: float = 2.0
    ki_angular: float = 0.0
    kd_angular: float = 0.1
    max_integral: float = 0.5

    # Speed limits
    max_linear_speed: float = 0.3
    max_angular_speed: float = 0.45

    # Tolerances
    pos_tolerance: float = 0.02
    yaw_tolerance: float = 0.04

    # Collision
    collision_stop: bool = True
    obstacle_height_thre: float = 0.05
    obstacle_count_thre: int = 20
    vehicle_lx: float = 0.5
    vehicle_ly: float = 0.5
    stop_margin: float = 0.1

    # Timeout / stuck detection
    move_timeout: float = 15.0
    stuck_time: float = 5.0
    stuck_progress_thre: float = 0.01

    # Control rate
    control_frequency: float = 50.0


class LocalMovement(Module):
    """
    Safe short-range body-frame movement controller.

    Accepts relative (dx, dy, dtheta) move commands in the robot body
    frame, transforms to world-frame goals, and drives to them with
    PID control while checking terrain for obstacles.

    State Machine:
        IDLE ──move──► MOVING ──reached──► COMPLETED
                         │                  timeout──► TIMEOUT
                         │
                      no progress──► STUCK ──progress──► MOVING
    """

    default_config = LocalMovementConfig
    config: LocalMovementConfig

    # Input topics
    odom: In[Odometry] = None  # type: ignore[assignment]           # /state_estimation
    move_command: In[Pose] = None  # type: ignore[assignment]       # /local_movement
    terrain: In[PointCloud2] = None  # type: ignore[assignment]     # /terrain_map

    # Output topics
    cmd_vel: Out[TwistStamped] = None  # type: ignore[assignment]   # /cmd_vel
    status: Out[Int8] = None  # type: ignore[assignment]            # /local_movement/status

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)

        # Vehicle state
        self._x = 0.0
        self._y = 0.0
        self._z = 0.0
        self._yaw = 0.0
        self._odom_time = 0.0
        self._has_odom = False

        # Goal
        self._goal_x = 0.0
        self._goal_y = 0.0
        self._goal_yaw = 0.0
        self._status = Status.IDLE

        # PID state
        self._pid_x = _PIDState()
        self._pid_y = _PIDState()
        self._pid_yaw = _PIDState()

        # Obstacle counts per direction
        self._front_stop = 0
        self._back_stop = 0
        self._left_stop = 0
        self._right_stop = 0
        self._cw_stop = 0
        self._ccw_stop = 0

        # Timing
        self._move_start_time = 0.0
        self._last_progress_time = 0.0
        self._last_progress_dist = 0.0

        # Thread control
        self._stop_event = threading.Event()
        self._control_thread: threading.Thread | None = None

        logger.info("LocalMovement initialized")

    @rpc
    def start(self) -> None:
        """Start the local movement controller and begin the control loop."""
        super().start()

        self.odom.subscribe(self._on_odom)
        self.move_command.subscribe(self._on_move_command)
        self.terrain.subscribe(self._on_terrain)

        self._stop_event.clear()
        self._control_thread = threading.Thread(
            target=self._control_loop, daemon=True, name="local_movement_control"
        )
        self._control_thread.start()

        logger.info("LocalMovement started")

    @rpc
    def stop(self) -> None:
        """Stop the local movement controller."""
        logger.info("Stopping LocalMovement...")

        self._stop_event.set()
        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout=2.0)

        super().stop()
        logger.info("LocalMovement stopped")

    @rpc
    def move(self, dx: float, dy: float, dtheta: float) -> bool:
        """
        Command a body-frame relative move.

        Args:
            dx: Forward distance (meters, body +x)
            dy: Left distance (meters, body +y)
            dtheta: Rotation (radians, CCW positive)

        Returns:
            True if accepted, False if no odometry available
        """
        if not self._has_odom:
            logger.warning("No odometry yet, ignoring move command")
            return False

        cy, sy = math.cos(self._yaw), math.sin(self._yaw)
        self._goal_x = self._x + cy * dx - sy * dy
        self._goal_y = self._y + sy * dx + cy * dy
        self._goal_yaw = normalize_angle(self._yaw + dtheta)

        self._pid_x.reset()
        self._pid_y.reset()
        self._pid_yaw.reset()
        self._front_stop = self._back_stop = 0
        self._left_stop = self._right_stop = 0
        self._cw_stop = self._ccw_stop = 0

        now = time.time()
        self._move_start_time = now
        self._last_progress_time = now
        self._last_progress_dist = math.hypot(dx, dy)
        self._status = Status.MOVING

        logger.info(f"Move started: dx={dx:.2f} dy={dy:.2f} dtheta={dtheta:.2f}")
        return True

    @rpc
    def get_status(self) -> int:
        """Get current movement status as integer (see Status enum)."""
        return int(self._status)

    def _on_odom(self, msg: Odometry) -> None:
        """Update vehicle state from odometry."""
        self._odom_time = msg.ts
        self._has_odom = True
        self._x = msg.x
        self._y = msg.y
        self._z = msg.z
        self._yaw = msg.yaw

    def _on_move_command(self, msg: Pose) -> None:
        """Handle body-frame move command received via topic (x, y, yaw)."""
        self.move(msg.x, msg.y, msg.yaw)

    def _on_terrain(self, msg: PointCloud2) -> None:
        """Count obstacle points in each directional zone around the vehicle."""
        self._front_stop = self._back_stop = 0
        self._left_stop = self._right_stop = 0
        self._cw_stop = self._ccw_stop = 0

        if not self.config.collision_stop:
            return

        half_x = self.config.vehicle_lx / 2.0
        half_y = self.config.vehicle_ly / 2.0
        margin = self.config.stop_margin
        large_x = half_x + margin
        large_y = half_y + margin
        r_large = math.hypot(large_x, large_y)
        h_thre = self.config.obstacle_height_thre

        points = np.asarray(msg.pointcloud.points)
        if len(points) == 0:
            return

        dx = points[:, 0] - self._x
        dy = points[:, 1] - self._y
        dz = points[:, 2] - self._z

        dist = np.hypot(dx, dy)
        keep = (dist <= r_large) & (np.abs(dz) >= h_thre)
        dx, dy = dx[keep], dy[keep]

        cy, sy = math.cos(self._yaw), math.sin(self._yaw)
        bx = cy * dx + sy * dy
        by = -sy * dx + cy * dy

        in_y = np.abs(by) < half_y
        in_x = np.abs(bx) < half_x

        front = in_y & (bx > 0) & (bx < large_x)
        back = in_y & (bx > -large_x) & (bx < 0)
        left = in_x & (by > 0) & (by < large_y)
        right = in_x & (by > -large_y) & (by < 0)

        self._front_stop = int(np.count_nonzero(front))
        self._back_stop = int(np.count_nonzero(back))
        self._left_stop = int(np.count_nonzero(left))
        self._right_stop = int(np.count_nonzero(right))

        # Rotation obstacle counts (which direction of rotation is blocked)
        self._cw_stop = int(
            np.count_nonzero(
                (front & (by > 0)) | (back & (by < 0)) | (right & (bx > 0)) | (left & (bx < 0))
            )
        )
        self._ccw_stop = int(
            np.count_nonzero(
                (front & (by < 0)) | (back & (by > 0)) | (right & (bx < 0)) | (left & (bx > 0))
            )
        )

    def _control_loop(self) -> None:
        """Main PID control loop. Runs at control_frequency Hz."""
        period = 1.0 / self.config.control_frequency
        dt = period

        while not self._stop_event.is_set():
            self._publish_status()

            if self._status not in (Status.MOVING, Status.STUCK) or not self._has_odom:
                time.sleep(period)
                continue

            t = time.time()
            obs_thre = self.config.obstacle_count_thre

            # Timeout
            if t - self._move_start_time > self.config.move_timeout:
                self._finish(Status.TIMEOUT, "Timed out")
                time.sleep(period)
                continue

            # World-frame error → body-frame
            ew_x = self._goal_x - self._x
            ew_y = self._goal_y - self._y
            cy, sy = math.cos(self._yaw), math.sin(self._yaw)
            ex = cy * ew_x + sy * ew_y
            ey = -sy * ew_x + cy * ew_y
            eyaw = normalize_angle(self._goal_yaw - self._yaw)
            pos_err = math.hypot(ex, ey)

            # Zero error on blocked axes
            if ex > 0 and self._front_stop > obs_thre:
                ex = 0.0
                self._pid_x.reset()
            elif ex < 0 and self._back_stop > obs_thre:
                ex = 0.0
                self._pid_x.reset()

            if ey > 0 and self._left_stop > obs_thre:
                ey = 0.0
                self._pid_y.reset()
            elif ey < 0 and self._right_stop > obs_thre:
                ey = 0.0
                self._pid_y.reset()

            if eyaw > 0 and self._ccw_stop > obs_thre:
                eyaw = 0.0
                self._pid_yaw.reset()
            elif eyaw < 0 and self._cw_stop > obs_thre:
                eyaw = 0.0
                self._pid_yaw.reset()

            # Stuck detection
            progress = self._last_progress_dist - pos_err
            if progress > self.config.stuck_progress_thre:
                self._last_progress_time = t
                self._last_progress_dist = pos_err
                if self._status == Status.STUCK:
                    self._status = Status.MOVING
                    logger.info("Unstuck — progress resumed")
            elif t - self._last_progress_time > self.config.stuck_time:
                if self._status != Status.STUCK:
                    self._status = Status.STUCK
                    logger.info("Stuck — no progress")

            # Goal reached
            yaw_err = abs(normalize_angle(self._goal_yaw - self._yaw))
            if pos_err < self.config.pos_tolerance and yaw_err < self.config.yaw_tolerance:
                self._finish(Status.COMPLETED, "Completed")
                time.sleep(period)
                continue

            # PID compute
            cfg = self.config
            vx = self._pid_compute(
                self._pid_x,
                ex,
                dt,
                cfg.kp_linear,
                cfg.ki_linear,
                cfg.kd_linear,
                cfg.max_integral,
                cfg.max_linear_speed,
            )
            vy = self._pid_compute(
                self._pid_y,
                ey,
                dt,
                cfg.kp_linear,
                cfg.ki_linear,
                cfg.kd_linear,
                cfg.max_integral,
                cfg.max_linear_speed,
            )
            wz = self._pid_compute(
                self._pid_yaw,
                eyaw,
                dt,
                cfg.kp_angular,
                cfg.ki_angular,
                cfg.kd_angular,
                cfg.max_integral,
                cfg.max_angular_speed,
            )

            self._publish_cmd_vel(vx, vy, wz)
            time.sleep(period)

    @staticmethod
    def _pid_compute(
        state: _PIDState,
        error: float,
        dt: float,
        kp: float,
        ki: float,
        kd: float,
        max_integral: float,
        max_output: float,
    ) -> float:
        """Single-axis PID with integral clamping and output saturation."""
        state.integral = max(-max_integral, min(max_integral, state.integral + error * dt))
        deriv = 0.0 if state.first else (error - state.prev_error) / dt
        state.first = False
        state.prev_error = error
        return max(-max_output, min(max_output, kp * error + ki * state.integral + kd * deriv))

    def _publish_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        msg = TwistStamped(
            ts=self._odom_time,
            frame_id="vehicle",
            linear=[vx, vy, 0.0],
            angular=[0.0, 0.0, wz],
        )
        self.cmd_vel.publish(msg)

    def _publish_status(self) -> None:
        self.status.publish(Int8(int(self._status)))

    def _finish(self, s: Status, reason: str) -> None:
        self._status = s
        self._publish_cmd_vel(0.0, 0.0, 0.0)
        logger.info(reason)


local_movement = LocalMovement.blueprint

__all__ = ["LocalMovement", "Status", "local_movement"]
