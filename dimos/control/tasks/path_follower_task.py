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

"""Path-following ControlTask for mobile-base navigation via ControlCoordinator.

Runs inside the coordinator's tick loop (typically 100 Hz) but **decimates**
its control computation to ``control_frequency`` (default 10 Hz) to match the
Go2 WebRTC command rate. On intermediate ticks the last computed command is
re-emitted so the coordinator's arbitration and write phases still run at full
rate.

Control pipeline (executed at ``control_frequency``):
    1. Read current pose from ``CoordinatorState.joints`` (odometry).
    2. Find closest point on path, compute cross-track error.
    3. PurePursuit: compute steering from lookahead point curvature.
    4. PIDCrossTrack: correct lateral drift.
    5. VelocityProfiler: limit speed based on path curvature.
    6. Output ``JointCommandOutput`` with VELOCITY mode ``[vx, vy, wz]``.

State machine::

    IDLE ──start_path()──► FOLLOWING ──goal_reached──► COMPLETED
      ▲                        │                          │
      │                    cancel()                    reset()
      │                        ▼                          │
      └─────reset()───── ABORTED ◄──────────────────────┘

CRITICAL: Uses ``state.t_now`` from CoordinatorState, never ``time.time()``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Literal

import numpy as np

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.control.tasks.path_controllers import (
    PIDCrossTrackController,
    PurePursuitController,
)
from dimos.control.tasks.path_distancer import PathDistancer
from dimos.control.tasks.velocity_profiler import VelocityProfiler
from dimos.utils.logging_config import setup_logger
from dimos.utils.trigonometry import angle_diff

if TYPE_CHECKING:
    from dimos.core.global_config import GlobalConfig
    from dimos.msgs.geometry_msgs import PoseStamped
    from dimos.msgs.nav_msgs import Path

logger = setup_logger()

# Type alias for task state
PathFollowerState = Literal["idle", "following", "completed", "aborted"]


@dataclass
class PathFollowerTaskConfig:
    """Configuration for PathFollowerTask.

    PID gains (``ct_kp``, ``ct_ki``, ``ct_kd``) should come from the plant
    identification / tuning pipeline — do **not** guess these values.
    """

    joint_names: list[str] = field(default_factory=lambda: ["go2_vx", "go2_vy", "go2_wz"])
    priority: int = 20
    control_frequency: float = 10.0  # effective output rate (Hz) — matches WebRTC limit

    # PurePursuit
    min_lookahead: float = 0.3
    max_lookahead: float = 1.0
    lookahead_speed_gain: float = 0.8
    k_angular: float = 0.6
    max_angular_velocity: float = 1.2
    max_linear_speed: float = 0.6

    # Cross-track PID (populate from tuning!)
    ct_kp: float = 0.0
    ct_ki: float = 0.0
    ct_kd: float = 0.0
    ct_max_correction: float = 0.6
    ct_max_integral: float = 0.3

    # Goal tolerance
    goal_tolerance: float = 0.2
    orientation_tolerance: float = 0.15


class PathFollowerTask(BaseControlTask):
    """ControlTask that follows a nav_msgs/Path using PurePursuit + PID."""

    def __init__(
        self,
        name: str,
        config: PathFollowerTaskConfig,
        global_config: GlobalConfig,
    ) -> None:
        if len(config.joint_names) != 3:
            raise ValueError(
                f"PathFollowerTask '{name}' requires exactly 3 joints "
                f"(vx, vy, wz), got {len(config.joint_names)}"
            )

        self._name = name
        self._config = config
        self._global_config = global_config
        self._joint_names = frozenset(config.joint_names)
        self._joint_names_list = list(config.joint_names)

        # State machine
        self._state: PathFollowerState = "idle"

        # Path data (set via start_path)
        self._path: Path | None = None
        self._path_distancer: PathDistancer | None = None
        self._velocity_profiler: VelocityProfiler | None = None
        self._current_odom: PoseStamped | None = None
        self._closest_index: int = 0

        # Controllers
        self._controller = PurePursuitController(
            global_config,
            control_frequency=config.control_frequency,
            min_lookahead=config.min_lookahead,
            max_lookahead=config.max_lookahead,
            lookahead_gain=config.lookahead_speed_gain,
            max_linear_speed=config.max_linear_speed,
            k_angular=config.k_angular,
            max_angular_velocity=config.max_angular_velocity,
        )
        self._cross_track_pid = PIDCrossTrackController(
            control_frequency=config.control_frequency,
            k_p=config.ct_kp,
            k_i=config.ct_ki,
            k_d=config.ct_kd,
            max_correction=config.ct_max_correction,
            max_integral=config.ct_max_integral,
        )

        # 10 Hz decimation state
        self._control_period = 1.0 / config.control_frequency
        self._last_compute_time: float = 0.0
        self._cached_output: JointCommandOutput | None = None

        logger.info(
            f"PathFollowerTask '{name}' initialised "
            f"(effective {config.control_frequency} Hz, joints={config.joint_names})"
        )

    # ------------------------------------------------------------------
    # ControlTask protocol
    # ------------------------------------------------------------------

    @property
    def name(self) -> str:
        return self._name

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            joints=self._joint_names,
            priority=self._config.priority,
            mode=ControlMode.VELOCITY,
        )

    def is_active(self) -> bool:
        return self._state == "following"

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Compute path-following velocity command.

        Called every coordinator tick (~100 Hz). The actual control computation
        only runs every ``_control_period`` seconds; intermediate ticks return
        the cached output so the coordinator still writes to hardware.
        """
        if self._state != "following" or self._path is None or self._path_distancer is None:
            return None
        if self._current_odom is None:
            return None

        # ---- Decimation: only recompute at control_frequency ----
        elapsed = state.t_now - self._last_compute_time
        if elapsed < self._control_period and self._cached_output is not None:
            return self._cached_output

        self._last_compute_time = state.t_now
        output = self._compute_control()
        self._cached_output = output
        return output

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        if joints & self._joint_names:
            logger.warning(f"PathFollowerTask '{self._name}' preempted by {by_task}")
            if self._state == "following":
                self._state = "aborted"

    # ------------------------------------------------------------------
    # Control computation (runs at control_frequency)
    # ------------------------------------------------------------------

    def _compute_control(self) -> JointCommandOutput:
        """Core control pipeline — PurePursuit + PID + VelocityProfiler."""
        assert self._path is not None
        assert self._path_distancer is not None
        assert self._velocity_profiler is not None
        assert self._current_odom is not None

        odom = self._current_odom
        distancer = self._path_distancer
        current_pos = np.array([odom.position.x, odom.position.y])
        distance_to_goal = distancer.distance_to_goal(current_pos)

        # ---- Final rotation mode ----
        if distance_to_goal < self._config.goal_tolerance and len(self._path.poses) > 0:
            goal_yaw = self._path.poses[-1].orientation.euler[2]
            robot_yaw = odom.orientation.euler[2]
            yaw_err = angle_diff(goal_yaw, robot_yaw)

            if abs(yaw_err) < self._config.orientation_tolerance:
                self._state = "completed"
                logger.info(f"PathFollowerTask '{self._name}' completed — goal reached")
                return self._zero_output()

            twist = self._controller.rotate(yaw_err)
            max_wz = self._config.max_angular_velocity
            wz = float(np.clip(twist.angular.z, -max_wz, max_wz))
            return JointCommandOutput(
                joint_names=self._joint_names_list,
                velocities=[float(twist.linear.x), float(twist.linear.y), wz],
                mode=ControlMode.VELOCITY,
            )

        # ---- Normal path following ----
        closest_idx = distancer.find_closest_point_index(current_pos)
        self._closest_index = closest_idx

        target_speed = self._velocity_profiler.get_velocity_at_index(self._path, closest_idx)
        curvature = distancer.get_curvature_at_index(closest_idx)

        lookahead = distancer.find_adaptive_lookahead_point(
            closest_idx,
            target_speed,
            min_lookahead=self._config.min_lookahead,
            max_lookahead=self._config.max_lookahead,
        )

        twist = self._controller.advance(
            lookahead, odom, current_speed=target_speed, path_curvature=curvature
        )

        # Cross-track PID correction
        cte = distancer.get_signed_cross_track_error(current_pos)
        ct_correction = self._cross_track_pid.compute_correction(cte)
        max_wz = self._config.max_angular_velocity
        wz = float(np.clip(twist.angular.z - ct_correction, -max_wz, max_wz))

        return JointCommandOutput(
            joint_names=self._joint_names_list,
            velocities=[float(twist.linear.x), float(twist.linear.y), wz],
            mode=ControlMode.VELOCITY,
        )

    def _zero_output(self) -> JointCommandOutput:
        return JointCommandOutput(
            joint_names=self._joint_names_list,
            velocities=[0.0, 0.0, 0.0],
            mode=ControlMode.VELOCITY,
        )

    # ------------------------------------------------------------------
    # Public API (called by navigation module)
    # ------------------------------------------------------------------

    def start_path(self, path: Path, current_odom: PoseStamped) -> bool:
        """Begin following a new path.

        Returns True if accepted.
        """
        if path is None or len(path.poses) < 2:
            logger.warning(f"PathFollowerTask '{self._name}': invalid path (need >= 2 poses)")
            return False

        self._path = path
        self._path_distancer = PathDistancer(path)
        self._velocity_profiler = VelocityProfiler(
            max_linear_speed=self._config.max_linear_speed,
            max_angular_speed=2.0,
            max_linear_accel=1.0,
            max_linear_decel=2.0,
            max_centripetal_accel=1.5,
            min_speed=0.1,
        )

        self._cross_track_pid.reset()
        self._current_odom = current_odom
        self._closest_index = self._path_distancer.find_closest_point_index(
            np.array([current_odom.position.x, current_odom.position.y])
        )
        self._state = "following"
        self._cached_output = None
        self._last_compute_time = 0.0

        logger.info(f"PathFollowerTask '{self._name}' started ({len(path.poses)} poses)")
        return True

    def update_odom(self, odom: PoseStamped) -> None:
        """Update current robot pose (called externally, e.g. from odom stream)."""
        self._current_odom = odom

    def cancel(self) -> bool:
        if self._state != "following":
            return False
        self._state = "aborted"
        self._cached_output = None
        logger.info(f"PathFollowerTask '{self._name}' cancelled")
        return True

    def reset(self) -> bool:
        if self._state == "following":
            logger.warning(f"Cannot reset '{self._name}' while following")
            return False
        self._state = "idle"
        self._path = None
        self._path_distancer = None
        self._velocity_profiler = None
        self._current_odom = None
        self._cached_output = None
        logger.info(f"PathFollowerTask '{self._name}' reset to IDLE")
        return True

    def get_state(self) -> PathFollowerState:
        return self._state


__all__ = [
    "PathFollowerTask",
    "PathFollowerTaskConfig",
]
