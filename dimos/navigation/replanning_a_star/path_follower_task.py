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
PathFollowerTask
================
Integrates TrajectoryController into the ControlCoordinator architecture
proposed by mustafab0 in issue #921.

Architecture
------------
Before (current):
    GlobalPlanner → LocalPlanner → cmd_vel (Subject) → Go2Connection

After (this file):
    GlobalPlanner → LocalPlanner → PathFollowerTask → ControlCoordinator
                                       ├── VelocityProfiler
                                       ├── TrajectoryController
                                       ├── ClearanceMonitor
                                       └── compute() → Twist → twist_command

The PathFollowerTask:
  - Implements the ControlTask protocol so it registers with ControlCoordinator
  - Runs at the coordinator's tick rate (default 10Hz, tunable)
  - Claims base joints [base_vx, base_vy, base_wz]
  - Outputs Twist via the coordinator's twist_command stream
  - Supports priority-based arbitration (nav can be preempted by teleop)

Usage
-----
    # In your robot blueprint / setup:
    from dimos.navigation.replanning_a_star.path_follower_task import (
        PathFollowerTask, PathFollowerTaskConfig
    )

    task = PathFollowerTask(
        config=PathFollowerTaskConfig(
            name="path_follower",
            joint_names=["base_vx", "base_vy", "base_wz"],
            priority=10,
            max_speed=1.2,
            control_frequency=10.0,
        ),
        global_config=global_config,
    )
    coordinator.add_task(task)

    # When navigation starts a new path:
    task.set_path(path)
    task.set_active(True)

    # Feed odom each tick:
    task.update_odom(pose_stamped)

    # Feed costmap for clearance:
    task.update_costmap(occupancy_grid)
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs import Path
from dimos.navigation.replanning_a_star.clearance_monitor import ClearanceMonitor
from dimos.navigation.replanning_a_star.path_distancer import PathDistancer
from dimos.navigation.replanning_a_star.performance_logger import PerformanceLogger
from dimos.navigation.replanning_a_star.trajectory_controller import (
    HolonomicPIDController,
    VelocityProfiler,
    _advance_closest_index,
    _path_tangent,
)
from dimos.utils.logging_config import setup_logger
from dimos.utils.trigonometry import angle_diff

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass
class PathFollowerTaskConfig:
    """Configuration for PathFollowerTask.

    Attributes
    ----------
    name            : task name registered with ControlCoordinator
    joint_names     : base joints to claim  (default: holonomic Go2)
    priority        : arbitration priority — higher wins
                      (teleop should be higher, e.g. 20)
    max_speed       : maximum forward speed (m/s)
    min_speed       : minimum speed (m/s) — prevents stalling
    control_frequency : Hz — should match ControlCoordinator tick rate
    max_lateral_accel : m/s² — limits curve speed in VelocityProfiler
    max_accel       : m/s² longitudinal acceleration limit
    max_decel       : m/s² longitudinal deceleration limit
    goal_tolerance  : metres — distance to goal to consider arrived
    """
    name: str = "path_follower"
    joint_names: list[str] = field(
        default_factory=lambda: ["base_vx", "base_vy", "base_wz"]
    )
    priority: int = 10
    max_speed: float = 1.2
    min_speed: float = 0.15
    control_frequency: float = 10.0
    max_lateral_accel: float = 0.5
    max_accel: float = 0.4
    max_decel: float = 0.6
    goal_tolerance: float = 0.20


# ---------------------------------------------------------------------------
# Claim / Arbitration helpers (mirrors ControlTask protocol)
# ---------------------------------------------------------------------------

@dataclass
class JointClaim:
    joints: list[str]
    priority: int


# ---------------------------------------------------------------------------
# PathFollowerTask
# ---------------------------------------------------------------------------

class PathFollowerTask:
    """
    Navigation path-following task for ControlCoordinator.

    Implements the ControlTask duck-type protocol:
      - name        : str
      - claim()     → JointClaim
      - is_active() → bool
      - compute(t_now) → dict[joint_name, float]

    The coordinator calls compute() every tick, merges the returned
    joint velocities with other tasks via priority arbitration, then
    routes to the hardware adapter.
    """

    def __init__(
        self,
        config: PathFollowerTaskConfig,
        global_config: GlobalConfig,
    ) -> None:
        self.name = config.name
        self._config = config
        self._global_config = global_config

        dt = 1.0 / config.control_frequency

        # Core controller components
        self._profiler = VelocityProfiler(
            max_speed=config.max_speed,
            min_speed=config.min_speed,
            max_lateral_accel=config.max_lateral_accel,
            max_accel=config.max_accel,
            max_decel=config.max_decel,
        )
        self._pid = HolonomicPIDController(global_config, config.control_frequency)
        self._pid.kp_cross = 4.0
        self._pid.kd_cross = 0.25
        self._pid.kp_heading = 2.0
        self._pid.kd_heading = 0.20
        self._pid.kff_curvature = 0.9
        self._pid.max_vy = 1.0
        self._pid.max_wz = 2.0

        self._clearance = ClearanceMonitor(
            robot_width=global_config.robot_width,
        )
        self._logger = PerformanceLogger(print_interval=10.0)

        # State (protected by lock for thread safety)
        self._lock = threading.Lock()
        self._active: bool = False
        self._path_xy: NDArray | None = None
        self._path_distancer: PathDistancer | None = None
        self._current_odom: PoseStamped | None = None
        self._costmap = None
        self._closest_index: int = 0

        # Arrival callback — set by LocalPlanner / GlobalPlanner
        self._on_arrived: "callable | None" = None
        self._on_obstacle: "callable | None" = None

    # ------------------------------------------------------------------
    # ControlTask Protocol
    # ------------------------------------------------------------------

    def claim(self) -> JointClaim:
        """Return the joints this task controls and its priority."""
        return JointClaim(
            joints=self._config.joint_names,
            priority=self._config.priority,
        )

    def is_active(self) -> bool:
        with self._lock:
            return self._active and self._path_xy is not None

    def compute(self, t_now: float) -> dict[str, float]:
        """
        Called every coordinator tick.
        Returns {joint_name: velocity} for arbitration.
        Returns empty dict if inactive.
        """
        with self._lock:
            if not self._active or self._path_xy is None:
                return {}
            odom = self._current_odom
            path_xy = self._path_xy
            distancer = self._path_distancer
            costmap = self._costmap
            cidx = self._closest_index

        if odom is None or distancer is None:
            return {}

        pos = np.array([odom.position.x, odom.position.y])

        # Check goal reached
        if distancer.distance_to_goal(pos) < self._config.goal_tolerance:
            self._handle_arrived()
            return {}

        # Advance closest index
        cidx = _advance_closest_index(path_xy, pos, cidx)
        with self._lock:
            self._closest_index = cidx

        # Speed-scaled lookahead
        target_speed = self._profiler.speed_at(cidx)

        # Clearance-aware scaling
        speed_scale, cte_tolerance = 1.0, 0.10
        if costmap is not None:
            try:
                speed_scale, cte_tolerance = self._clearance.update(pos, costmap)
            except Exception:
                pass

        target_speed *= speed_scale
        self._pid.cte_tolerance = cte_tolerance

        # Find lookahead point
        lh_dist = max(0.3, target_speed * 0.35)
        lh = distancer.find_lookahead_point(cidx)

        # Compute twist
        twist = self._pid.compute(lh, odom, target_speed, path_xy, cidx)
        vx, vy, wz = twist.linear.x, twist.linear.y, twist.angular.z

        # Log performance
        self._log_tick(pos, path_xy, cidx, vx, vy, wz, target_speed, odom)

        # Map to joint velocities
        return {
            "base_vx": float(vx),
            "base_vy": float(vy),
            "base_wz": float(wz),
        }

    # ------------------------------------------------------------------
    # Public API (called by LocalPlanner / GlobalPlanner)
    # ------------------------------------------------------------------

    def set_path(self, path: Path) -> None:
        """Load a new path. Resets controller state and builds velocity profile."""
        pts = np.array([[p.position.x, p.position.y] for p in path.poses])
        path_len = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1))) \
            if len(pts) > 1 else 0.0

        self._profiler.build(path)
        distancer = PathDistancer(path)

        with self._lock:
            self._path_xy = pts
            self._path_distancer = distancer
            self._closest_index = 0

        self._pid.reset()
        self._logger.start_episode(path_length=path_len)
        logger.info(f"PathFollowerTask: new path, {len(pts)} waypoints, {path_len:.2f}m")

    def set_active(self, active: bool) -> None:
        """Enable or disable this task."""
        with self._lock:
            self._active = active
        if not active:
            self._logger.end_episode(completed=False)
            self._pid.reset()

    def update_odom(self, odom: PoseStamped) -> None:
        """Feed latest odometry. Call from odom handler."""
        with self._lock:
            self._current_odom = odom

    def update_costmap(self, costmap) -> None:
        """Feed latest costmap for clearance monitoring."""
        with self._lock:
            self._costmap = costmap

    def on_arrived(self, callback: "callable") -> None:
        """Register callback for when the robot reaches the goal."""
        self._on_arrived = callback

    def on_obstacle(self, callback: "callable") -> None:
        """Register callback for obstacle detection."""
        self._on_obstacle = callback

    def get_performance_summary(self) -> dict:
        """Return latest performance stats."""
        return self._logger.summary()

    def save_performance_log(self, path: str = "/tmp/path_follower_perf.json") -> None:
        self._logger.save(path)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _handle_arrived(self) -> None:
        logger.info("PathFollowerTask: goal reached")
        with self._lock:
            self._active = False
        self._logger.end_episode(completed=True)
        if self._on_arrived:
            self._on_arrived()

    def _log_tick(
        self,
        pos: "NDArray",
        path_xy: "NDArray",
        cidx: int,
        vx: float,
        vy: float,
        wz: float,
        target_speed: float,
        odom: PoseStamped,
    ) -> None:
        try:
            speed = math.sqrt(vx**2 + vy**2)
            tang = _path_tangent(path_xy, cidx)
            norm = np.array([-tang[1], tang[0]])
            cte  = float(np.dot(pos - path_xy[cidx], norm))
            desired_yaw = math.atan2(tang[1], tang[0])
            heading_err = angle_diff(desired_yaw, odom.orientation.euler[2])
            self._logger.record(
                speed=speed,
                cte=cte,
                heading_error=heading_err,
                target_speed=target_speed,
                clearance=self._clearance.clearance,
                mode="path_following",
            )
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Convenience: build a PathFollowerTask and register with coordinator
# ---------------------------------------------------------------------------

def add_path_follower_to_coordinator(
    coordinator,
    global_config: GlobalConfig,
    max_speed: float = 1.2,
    priority: int = 10,
) -> PathFollowerTask:
    """
    Create a PathFollowerTask and register it with a ControlCoordinator.

    Parameters
    ----------
    coordinator  : ControlCoordinator instance
    global_config: GlobalConfig
    max_speed    : maximum navigation speed (m/s)
    priority     : task priority — should be below teleop (e.g. teleop=20, nav=10)

    Returns
    -------
    PathFollowerTask  — keep a reference to call set_path() / update_odom()

    Example
    -------
        task = add_path_follower_to_coordinator(coordinator, global_config)
        # When a new path arrives:
        task.set_path(path)
        task.set_active(True)
        # Each odom tick:
        task.update_odom(odom)
        # Each costmap update:
        task.update_costmap(costmap)
    """
    from dimos.control.components import make_twist_base_joints

    base_joints = make_twist_base_joints("base")

    task = PathFollowerTask(
        config=PathFollowerTaskConfig(
            name="path_follower",
            joint_names=base_joints,
            priority=priority,
            max_speed=max_speed,
        ),
        global_config=global_config,
    )
    coordinator.add_task(task)
    logger.info(f"PathFollowerTask registered with coordinator (priority={priority})")
    return task