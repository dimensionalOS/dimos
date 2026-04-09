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
Trajectory Controller for Unitree Go2 (holonomic quadruped)
============================================================
Replaces the existing P/PD controllers with a proper trajectory-aware
controller that can run the Go2 at higher speeds without oscillation.

Architecture
------------
TrajectoryController
  ├── VelocityProfiler   – assigns a target speed to every waypoint on the
  │                        path, slowing for high-curvature segments and
  │                        respecting acceleration limits (forward + backward
  │                        pass, similar to a motion-profile smoother).
  └── HolonomicPIDController – closed-loop tracking using:
        • Along-track velocity  → from VelocityProfiler
        • Cross-track error     → corrected with lateral (vy) command
        • Heading error         → corrected with angular (wz) command
        • Feed-forward wz       → anticipates turns from path curvature

Key design decisions
--------------------
* No Pure Pursuit – the maintainer explicitly asked not to use it for
  holonomic robots (it restricts DOF unnecessarily).
* Holonomic exploitation – the Go2 can strafe, so cross-track error is
  corrected directly with vy instead of forcing a heading change first.
* Feed-forward curvature term – reduces lag when entering curves.
* Drop-in replacement – implements the same Controller Protocol as
  PController / PdController so nothing else needs to change.

Usage (in local_planner.py, replace PController with TrajectoryController)
--------------------------------------------------------------------------
    from dimos.navigation.replanning_a_star.trajectory_controller import (
        TrajectoryController,
    )

    self._controller = TrajectoryController(
        global_config=self._global_config,
        speed=self._speed,
        control_frequency=self._control_frequency,
    )

    # Whenever a new path arrives, call set_path() so the velocity
    # profiler can pre-compute the speed profile:
    self._controller.set_path(path)   # path: dimos.msgs.nav_msgs.Path
"""

from __future__ import annotations

import math
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs import Path
from dimos.utils.trigonometry import angle_diff


# ---------------------------------------------------------------------------
# Velocity Profiler
# ---------------------------------------------------------------------------

class VelocityProfiler:
    """
    Pre-computes a target longitudinal speed for every waypoint in a path.

    Algorithm
    ---------
    1. Estimate the curvature κ at each waypoint using the three-point
       circumscribed-circle formula.
    2. Compute curvature-limited speed:  v = min(v_max, sqrt(a_lat / κ))
    3. Forward pass  – enforce max acceleration from the start.
    4. Backward pass – enforce max deceleration toward the goal.

    The result is a smooth, physically feasible speed profile.
    """

    def __init__(
        self,
        max_speed: float = 0.55,
        min_speed: float = 0.15,
        max_lateral_accel: float = 0.8,   # m/s²  – tune per robot
        max_accel: float = 0.6,           # m/s²  longitudinal
        max_decel: float = 0.8,           # m/s²  longitudinal
    ) -> None:
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.max_lateral_accel = max_lateral_accel
        self.max_accel = max_accel
        self.max_decel = max_decel

        self._speeds: NDArray[np.float64] = np.array([max_speed])
        self._path_xy: NDArray[np.float64] = np.zeros((1, 2))

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def build(self, path: Path) -> None:
        """Call once when a new path is received."""
        pts = np.array([[p.position.x, p.position.y] for p in path.poses])
        self._path_xy = pts
        self._speeds = self._compute_profile(pts)

    def speed_at(self, index: int) -> float:
        """Return the target speed for waypoint *index*."""
        idx = int(np.clip(index, 0, len(self._speeds) - 1))
        return float(self._speeds[idx])

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _compute_profile(self, pts: NDArray[np.float64]) -> NDArray[np.float64]:
        n = len(pts)
        speeds = np.full(n, self.max_speed)

        # --- Step 1: curvature-limited speed at every interior point ----
        for i in range(1, n - 1):
            kappa = _curvature_at(pts, i)
            if kappa > 1e-6:
                v_curve = math.sqrt(self.max_lateral_accel / kappa)
                speeds[i] = min(speeds[i], v_curve)

        # Always stop smoothly at the last waypoint
        speeds[-1] = 0.0

        # Enforce minimum speed on non-terminal points
        speeds[:-1] = np.maximum(speeds[:-1], self.min_speed)

        # --- Step 2: segment lengths ------------------------------------
        seg_lens = np.linalg.norm(np.diff(pts, axis=0), axis=1)          # shape (n-1,)

        # --- Step 3: forward pass  (accel limit) ------------------------
        for i in range(1, n):
            ds = seg_lens[i - 1]
            v_max_fwd = math.sqrt(speeds[i - 1] ** 2 + 2 * self.max_accel * ds)
            speeds[i] = min(speeds[i], v_max_fwd)

        # --- Step 4: backward pass  (decel limit) -----------------------
        for i in range(n - 2, -1, -1):
            ds = seg_lens[i]
            v_max_bwd = math.sqrt(speeds[i + 1] ** 2 + 2 * self.max_decel * ds)
            speeds[i] = min(speeds[i], v_max_bwd)

        return speeds


# ---------------------------------------------------------------------------
# Holonomic PID trajectory controller
# ---------------------------------------------------------------------------

class HolonomicPIDController:
    """
    Cross-track + heading PID controller for a holonomic robot.

    Error decomposition (robot frame)
    ----------------------------------
    * cross_track_error  – signed perpendicular distance to the path tangent
                           (positive = robot is to the left of the path)
    * along_track_error  – signed distance ahead/behind the reference point
    * heading_error      – difference between desired and actual yaw

    Output
    ------
    Twist with:
      linear.x  = along-track speed   (from velocity profile + P term)
      linear.y  = lateral correction  (cross-track PID, holonomic advantage)
      angular.z = heading correction  (heading PD + curvature feed-forward)
    """

    # --- Gains (all tunable) --------------------------------------------
    kp_cross: float = 2.5        # lateral proportional gain
    kd_cross: float = 0.15       # lateral derivative gain
    kp_heading: float = 1.8      # heading proportional gain
    kd_heading: float = 0.20     # heading derivative gain
    kff_curvature: float = 0.8   # feed-forward: wz = kff * v * kappa
    kp_along: float = 0.2        # along-track speed boost/trim

    # --- Limits ---------------------------------------------------------
    max_vy: float = 0.8          # m/s  lateral
    max_wz: float = 1.6          # rad/s
    rotation_threshold: float = 90 * math.pi / 180  # rad
    cte_tolerance: float = 0.10  # m — set by ClearanceMonitor each tick

    def __init__(
        self,
        global_config: GlobalConfig,
        control_frequency: float,
    ) -> None:
        self._global_config = global_config
        self._dt = 1.0 / control_frequency

        self._prev_cross_error: float = 0.0
        self._prev_heading_error: float = 0.0
        self._prev_angular_velocity: float = 0.0
        self._max_angular_accel: float = 2.0      # rad/s²

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def compute(
        self,
        lookahead_point: NDArray[np.float64],
        current_odom: PoseStamped,
        target_speed: float,
        path_xy: NDArray[np.float64],
        closest_index: int,
    ) -> Twist:
        """
        Predictive cross-track error controller.

        At high speed, correcting current position error is too late — the
        robot has already passed the correction window. Instead:
          1. Project the robot one control step ahead using target_speed.
          2. Measure CTE at that predicted position.
          3. Steer toward the lookahead point (not the closest point).
        This gives the controller time to act before error accumulates.
        """
        pos = np.array([current_odom.position.x, current_odom.position.y])
        robot_yaw = current_odom.orientation.euler[2]
        cos_y, sin_y = math.cos(robot_yaw), math.sin(robot_yaw)

        # --- Heading toward lookahead point ----------------------------
        to_lookahead = lookahead_point - pos
        dist_to_lh = float(np.linalg.norm(to_lookahead))
        if dist_to_lh < 1e-6:
            return Twist()
        desired_yaw = math.atan2(to_lookahead[1], to_lookahead[0])
        heading_error = angle_diff(desired_yaw, robot_yaw)

        # --- Curvature feed-forward (slightly ahead of robot) ----------
        ff_idx = min(closest_index + 3, len(path_xy) - 1)
        kappa_signed = _signed_curvature_at(path_xy, ff_idx)

        # --- Current CTE (for derivative term) -------------------------
        cur_tang = _path_tangent(path_xy, closest_index)
        cur_norm = np.array([-cur_tang[1], cur_tang[0]])
        cur_cte  = float(np.dot(pos - path_xy[closest_index], cur_norm))

        # --- Predictive CTE: project one step ahead --------------------
        pred_pos = pos + self._dt * target_speed * np.array([cos_y, sin_y])
        s = max(0, closest_index - 1)
        e = min(len(path_xy), closest_index + 6)
        sub_dists = np.linalg.norm(path_xy[s:e] - pred_pos, axis=1)
        pred_idx  = s + int(np.argmin(sub_dists))
        pred_tang = _path_tangent(path_xy, pred_idx)
        pred_norm = np.array([-pred_tang[1], pred_tang[0]])
        pred_cte  = float(np.dot(pred_pos - path_xy[pred_idx], pred_norm))

        # --- Commands --------------------------------------------------
        vx = float(np.clip(target_speed, 0.0, target_speed))

        d_cross = (cur_cte - self._prev_cross_error) / self._dt
        # Only apply lateral correction when error exceeds tolerance
        # (set by ClearanceMonitor — loose in open space, tight near walls)
        correction_cte = pred_cte if abs(pred_cte) > self.cte_tolerance else 0.0
        vy = -(self.kp_cross * correction_cte + self.kd_cross * d_cross)
        vy = float(np.clip(vy, -self.max_vy, self.max_vy))

        d_heading = (heading_error - self._prev_heading_error) / self._dt
        wz = (self.kp_heading * heading_error
              + self.kd_heading * d_heading
              + self.kff_curvature * vx * kappa_signed)
        max_delta = self._max_angular_accel * self._dt
        wz = float(np.clip(wz,
                            self._prev_angular_velocity - max_delta,
                            self._prev_angular_velocity + max_delta))
        wz = float(np.clip(wz, -self.max_wz, self.max_wz))

        self._prev_cross_error    = cur_cte
        self._prev_heading_error  = heading_error
        self._prev_angular_velocity = wz

        return Twist(
            linear=Vector3(vx, vy, 0.0),
            angular=Vector3(0.0, 0.0, wz),
        )

    def rotate(self, yaw_error: float, speed: float) -> Twist:
        """Pure rotation, used during initial/final alignment."""
        kp = 0.5
        angular_velocity = float(np.clip(kp * yaw_error, -speed, speed))
        min_av = 0.2
        if angular_velocity != 0.0 and abs(angular_velocity) < min_av:
            angular_velocity = math.copysign(min_av, angular_velocity)
        # In simulation a small forward nudge helps locomotion policies
        linear_x = 0.18 if self._global_config.simulation else 0.0
        return Twist(
            linear=Vector3(linear_x, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )

    def reset(self) -> None:
        self._prev_cross_error = 0.0
        self._prev_heading_error = 0.0
        self._prev_angular_velocity = 0.0

    def reset_heading_error(self, value: float) -> None:
        self._prev_heading_error = value


# ---------------------------------------------------------------------------
# TrajectoryController  (drop-in replacement, satisfies Controller Protocol)
# ---------------------------------------------------------------------------

class TrajectoryController:
    """
    Full trajectory controller: velocity profiling + holonomic PID.

    Drop-in replacement for PController / PdController.
    After constructing, call set_path(path) every time a new path arrives.

    Example (local_planner.py)
    --------------------------
        self._controller = TrajectoryController(
            global_config=self._global_config,
            speed=self._speed,
            control_frequency=self._control_frequency,
        )
        # ... inside start_planning():
        self._controller.set_path(path)
    """

    def __init__(
        self,
        global_config: GlobalConfig,
        speed: float,
        control_frequency: float,
    ) -> None:
        self._speed = speed
        self._control_frequency = control_frequency

        self._profiler = VelocityProfiler(
            max_speed=speed,
            min_speed=max(0.15, speed * 0.25),
        )
        self._pid = HolonomicPIDController(
            global_config=global_config,
            control_frequency=control_frequency,
        )

        # Populated by set_path()
        self._path_xy: NDArray[np.float64] = np.zeros((1, 2))
        self._closest_index: int = 0

    # ------------------------------------------------------------------
    # Path update – call this whenever local_planner receives a new path
    # ------------------------------------------------------------------

    def set_path(self, path: Path) -> None:
        """Pre-compute velocity profile for the new path."""
        self._profiler.build(path)
        self._path_xy = np.array([[p.position.x, p.position.y] for p in path.poses])
        self._closest_index = 0

    # ------------------------------------------------------------------
    # Controller Protocol implementation
    # ------------------------------------------------------------------

    def advance(
        self,
        lookahead_point: NDArray[np.float64],
        current_odom: PoseStamped,
        speed_scale: float = 1.0,
        cte_tolerance: float = 0.10,
    ) -> Twist:
        """
        Called by LocalPlanner at every control tick during path following.

        Parameters
        ----------
        speed_scale    : float [0,1] from ClearanceMonitor — scales target speed
                         down in tight spaces (1.0 = full speed in open space)
        cte_tolerance  : acceptable cross-track error (m) from ClearanceMonitor
                         (larger = looser correction in open space)
        """
        pos = np.array([current_odom.position.x, current_odom.position.y])

        # Update closest index (search forward only to avoid back-tracking)
        self._closest_index = _advance_closest_index(
            self._path_xy, pos, self._closest_index
        )

        target_speed = self._profiler.speed_at(self._closest_index) * speed_scale

        # Pass tolerance to PID so it can gate the lateral correction
        self._pid.cte_tolerance = cte_tolerance

        return self._pid.compute(
            lookahead_point=lookahead_point,
            current_odom=current_odom,
            target_speed=target_speed,
            path_xy=self._path_xy,
            closest_index=self._closest_index,
        )

    def rotate(self, yaw_error: float) -> Twist:
        """Called during initial_rotation and final_rotation states."""
        return self._pid.rotate(yaw_error, self._speed)

    def reset_errors(self) -> None:
        self._pid.reset()
        self._closest_index = 0

    def reset_yaw_error(self, value: float) -> None:
        self._pid.reset_heading_error(value)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _curvature_at(pts: NDArray[np.float64], i: int) -> float:
    """
    Menger curvature of the arc through pts[i-1], pts[i], pts[i+1].
    Returns 0 for boundary points or degenerate configurations.
    """
    if i <= 0 or i >= len(pts) - 1:
        return 0.0
    a, b, c = pts[i - 1], pts[i], pts[i + 1]
    ab = float(np.linalg.norm(b - a))
    bc = float(np.linalg.norm(c - b))
    ac = float(np.linalg.norm(c - a))
    if ab < 1e-9 or bc < 1e-9 or ac < 1e-9:
        return 0.0
    # Area of triangle via cross product
    cross = float(abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])))
    denom = ab * bc * ac
    return cross / denom if denom > 1e-9 else 0.0


def _signed_curvature_at(pts: NDArray[np.float64], i: int) -> float:
    """
    Signed Menger curvature – positive = left turn, negative = right turn.
    """
    if i <= 0 or i >= len(pts) - 1:
        return 0.0
    a, b, c = pts[i - 1], pts[i], pts[i + 1]
    cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    ab = float(np.linalg.norm(b - a))
    bc = float(np.linalg.norm(c - b))
    ac = float(np.linalg.norm(c - a))
    denom = ab * bc * ac
    return float(cross / denom) if denom > 1e-9 else 0.0


def _path_tangent(pts: NDArray[np.float64], i: int) -> NDArray[np.float64]:
    """
    Unit tangent vector at waypoint i (central difference where possible).
    """
    n = len(pts)
    if n < 2:
        return np.array([1.0, 0.0])
    if i == 0:
        diff = pts[1] - pts[0]
    elif i >= n - 1:
        diff = pts[-1] - pts[-2]
    else:
        diff = pts[i + 1] - pts[i - 1]   # central difference
    norm = float(np.linalg.norm(diff))
    return diff / norm if norm > 1e-9 else np.array([1.0, 0.0])


def _advance_closest_index(
    path_xy: NDArray[np.float64],
    pos: NDArray[np.float64],
    current_index: int,
    search_window: int = 20,
) -> int:
    """
    Find the closest path point, but only search ahead of *current_index*
    to avoid the controller snapping backwards.
    """
    start = current_index
    end = min(current_index + search_window, len(path_xy))
    sub = path_xy[start:end]
    dists = np.linalg.norm(sub - pos, axis=1)
    return start + int(np.argmin(dists))