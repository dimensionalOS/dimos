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

"""Baseline path-follower ControlTask: production LocalPlanner algorithm,
unwrapped from its daemon thread and rebuilt as a passive ControlTask.

Algorithm is a faithful port of
:class:`dimos.navigation.replanning_a_star.local_planner.LocalPlanner`:
PController + 0.5 m fixed lookahead + rotate-then-drive heuristic +
state machine (initial_rotation → path_following → final_rotation → arrived).

Costmap / obstacle-clearance plumbing is intentionally omitted - the
benchmark battery is obstacle-free.
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
from dimos.control.tasks.feedforward_gain_compensator import (
    FeedforwardGainCompensator,
    FeedforwardGainConfig,
)
from dimos.control.tasks.velocity_tracking_pid import (
    VelocityTrackingConfig,
    VelocityTrackingPID,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.replanning_a_star.controllers import PController
from dimos.navigation.replanning_a_star.path_distancer import PathDistancer
from dimos.utils.benchmarking.velocity_profile import PathSpeedCap, VelocityProfileConfig
from dimos.utils.logging_config import setup_logger
from dimos.utils.trigonometry import angle_diff

if TYPE_CHECKING:
    from dimos.core.global_config import GlobalConfig
    from dimos.msgs.nav_msgs.Path import Path

logger = setup_logger()

BaselineState = Literal[
    "idle", "initial_rotation", "path_following", "final_rotation", "arrived", "aborted"
]

# Sentinel so callers can pass ``None`` to configure() to explicitly
# clear ff/profile_config, distinct from "don't touch this field".
_UNSET: object = object()


@dataclass
class BaselinePathFollowerTaskConfig:
    joint_names: list[str] = field(default_factory=lambda: ["base/vx", "base/vy", "base/wz"])
    priority: int = 20
    speed: float = 0.55
    control_frequency: float = 10.0
    goal_tolerance: float = 0.2
    orientation_tolerance: float = 0.35
    # PController outer-loop angular gain. Default 0.5 matches production
    # LocalPlanner; sweep on circle_R1.0 found 1.0 gives ~9x lower CTE.
    k_angular: float = 0.5
    # Optional inner-loop velocity-tracking PID. None ⟹ no closed loop.
    # Mutually exclusive with ff_config (PI takes precedence if both set).
    pid_config: VelocityTrackingConfig | None = None
    # Optional static feedforward plant-gain compensator (Strategy B).
    # cmd_to_robot = controller_cmd / K_plant. No actual feedback needed.
    ff_config: FeedforwardGainConfig | None = None
    # Optional curvature velocity-profile cap. None ⟹ off
    velocity_profile_config: VelocityProfileConfig | None = None


class BaselinePathFollowerTask(BaseControlTask):
    """Production LocalPlanner algorithm as a passive ControlTask."""

    def __init__(
        self,
        name: str,
        config: BaselinePathFollowerTaskConfig,
        global_config: GlobalConfig,
    ) -> None:
        if len(config.joint_names) != 3:
            raise ValueError(
                f"BaselinePathFollowerTask '{name}' needs 3 joints (vx, vy, wz), "
                f"got {len(config.joint_names)}"
            )

        self._name = name
        self._config = config
        self._joint_names_list = list(config.joint_names)
        self._joint_names = frozenset(config.joint_names)

        self._controller = PController(global_config, config.speed, config.control_frequency)
        # Override the class-level _k_angular for this instance only.
        self._controller._k_angular = config.k_angular
        self._pid: VelocityTrackingPID | None = (
            VelocityTrackingPID(config.pid_config) if config.pid_config else None
        )
        self._ff: FeedforwardGainCompensator | None = (
            FeedforwardGainCompensator(config.ff_config) if config.ff_config else None
        )
        self._profile_cap: PathSpeedCap | None = (
            PathSpeedCap(config.velocity_profile_config)
            if config.velocity_profile_config is not None
            else None
        )

        self._state: BaselineState = "idle"
        self._path: Path | None = None
        self._distancer: PathDistancer | None = None
        self._current_odom: PoseStamped | None = None
        # Closed-path gate: track the furthest-along path index reached so
        # that closed paths (where goal==start) don't trip arrival on tick 1.
        self._max_progress_idx: int = 0

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
        return self._state in ("initial_rotation", "path_following", "final_rotation")

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        if not self.is_active():
            return None
        if self._path is None or self._distancer is None:
            return None

        # Pull pose from CoordinatorState. The twist-base ConnectedHardware
        # routes adapter.read_odometry() -> [x, y, yaw]
        pos = state.joints.joint_positions
        x = pos.get(self._joint_names_list[0])
        y = pos.get(self._joint_names_list[1])
        yaw = pos.get(self._joint_names_list[2])
        if x is not None and y is not None and yaw is not None:
            self._current_odom = PoseStamped(
                ts=state.t_now,
                position=Vector3(float(x), float(y), 0.0),
                orientation=Quaternion.from_euler(Vector3(0.0, 0.0, float(yaw))),
            )
        if self._current_odom is None:
            return None

        match self._state:
            case "initial_rotation":
                vx, vy, wz = self._step_initial_rotation()
            case "path_following":
                vx, vy, wz = self._step_path_following()
            case "final_rotation":
                vx, vy, wz = self._step_final_rotation()
            case _:
                return None

        # Inner-loop options (mutually exclusive - PI wins if both set).
        if self._pid is not None:
            actual_vx = state.joints.joint_velocities.get(self._joint_names_list[0], 0.0)
            actual_vy = state.joints.joint_velocities.get(self._joint_names_list[1], 0.0)
            actual_wz = state.joints.joint_velocities.get(self._joint_names_list[2], 0.0)
            vx, vy, wz = self._pid.compute(vx, vy, wz, actual_vx, actual_vy, actual_wz)
        elif self._ff is not None:
            # Static gain compensation: cmd_to_robot = controller_cmd / K_plant
            vx, vy, wz = self._ff.compute(vx, vy, wz)

        # Curvature velocity-profile cap (preserves commanded turn radius).
        if self._profile_cap is not None:
            vx, vy, wz = self._profile_cap.cap(
                self._current_odom.position.x, self._current_odom.position.y, vx, vy, wz
            )

        return JointCommandOutput(
            joint_names=self._joint_names_list,
            velocities=[vx, vy, wz],
            mode=ControlMode.VELOCITY,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        if joints & self._joint_names and self.is_active():
            logger.warning(f"BaselinePathFollowerTask '{self._name}' preempted by {by_task}")
            self._state = "aborted"

    # ------------------------------------------------------------------
    # State-machine bodies (mirrors LocalPlanner._compute_*)
    # ------------------------------------------------------------------

    def _step_initial_rotation(self) -> tuple[float, float, float]:
        assert self._path is not None and self._current_odom is not None
        first_yaw = self._path.poses[0].orientation.euler[2]
        robot_yaw = self._current_odom.orientation.euler[2]
        yaw_err = angle_diff(first_yaw, robot_yaw)

        if abs(yaw_err) < self._config.orientation_tolerance:
            self._state = "path_following"
            return self._step_path_following()

        twist = self._controller.rotate(yaw_err)
        return float(twist.linear.x), float(twist.linear.y), float(twist.angular.z)

    def _windowed_closest(self, pos: np.ndarray, window: int = 20) -> int:
        """Closest path index searched only in a forward window from
        ``_max_progress_idx``. Prevents wrap-around matches on closed paths
        (e.g. circle where path[0] == path[-1] would otherwise let argmin
        return the last index on tick 1 → spurious 'arrived').
        """
        assert self._path is not None
        n = len(self._path.poses)
        lo = self._max_progress_idx
        hi = min(n, lo + window + 1)
        best_idx = lo
        best_d_sq = float("inf")
        for i in range(lo, hi):
            p = self._path.poses[i].position
            d_sq = (p.x - pos[0]) ** 2 + (p.y - pos[1]) ** 2
            if d_sq < best_d_sq:
                best_d_sq = d_sq
                best_idx = i
        return best_idx

    def _step_path_following(self) -> tuple[float, float, float]:
        assert self._path is not None
        assert self._distancer is not None
        assert self._current_odom is not None

        pos = np.array([self._current_odom.position.x, self._current_odom.position.y])

        closest = self._windowed_closest(pos)
        if closest > self._max_progress_idx:
            self._max_progress_idx = closest

        # Arrival is only valid AFTER we've traversed enough of the path.
        # Otherwise closed paths (goal==start) would arrive on tick 1.
        progress_threshold = max(1, int(0.7 * (len(self._path.poses) - 1)))
        if (
            self._max_progress_idx >= progress_threshold
            and self._distancer.distance_to_goal(pos) < self._config.goal_tolerance
        ):
            self._state = "final_rotation"
            return self._step_final_rotation()

        lookahead = self._distancer.find_lookahead_point(closest)
        twist = self._controller.advance(lookahead, self._current_odom)
        return float(twist.linear.x), float(twist.linear.y), float(twist.angular.z)

    def _step_final_rotation(self) -> tuple[float, float, float]:
        assert self._path is not None and self._current_odom is not None
        goal_yaw = self._path.poses[-1].orientation.euler[2]
        robot_yaw = self._current_odom.orientation.euler[2]
        yaw_err = angle_diff(goal_yaw, robot_yaw)

        if abs(yaw_err) < self._config.orientation_tolerance:
            self._state = "arrived"
            logger.info(f"BaselinePathFollowerTask '{self._name}' arrived")
            return 0.0, 0.0, 0.0

        twist = self._controller.rotate(yaw_err)
        return float(twist.linear.x), float(twist.linear.y), float(twist.angular.z)

    # ------------------------------------------------------------------
    # Public API (called by runner — typically over RPC from a tool)
    # ------------------------------------------------------------------

    def configure(
        self,
        speed: float | None = None,
        k_angular: float | None = None,
        ff_config: FeedforwardGainConfig | None | object = _UNSET,
        velocity_profile_config: VelocityProfileConfig | None | object = _UNSET,
    ) -> bool:
        """Override per-run knobs before start_path. ``ff_config`` and
        ``velocity_profile_config`` use a sentinel so callers can
        explicitly clear them by passing ``None`` (distinct from "don't
        touch"). Only valid while idle/arrived/aborted — refuses while
        the task is actively driving the robot.
        """
        if self.is_active():
            logger.warning(
                f"BaselinePathFollowerTask '{self._name}': cannot configure while active"
            )
            return False
        if speed is not None:
            self._config.speed = speed
            self._controller._speed = speed  # PController exposes _speed
        if k_angular is not None:
            self._config.k_angular = k_angular
            self._controller._k_angular = k_angular
        if ff_config is not _UNSET:
            self._config.ff_config = ff_config  # type: ignore[assignment]
            self._ff = FeedforwardGainCompensator(ff_config) if ff_config is not None else None
        if velocity_profile_config is not _UNSET:
            self._config.velocity_profile_config = velocity_profile_config  # type: ignore[assignment]
            self._profile_cap = (
                PathSpeedCap(velocity_profile_config)
                if velocity_profile_config is not None
                else None
            )
        return True

    def start_path(self, path: Path, current_odom: PoseStamped) -> bool:
        if path is None or len(path.poses) < 2:
            logger.warning(f"BaselinePathFollowerTask '{self._name}': invalid path")
            return False
        self._path = path
        self._distancer = PathDistancer(path)
        self._current_odom = current_odom
        self._max_progress_idx = 0
        self._controller.reset_errors()
        if self._pid is not None:
            self._pid.reset()
        if self._ff is not None:
            self._ff.reset()
        if self._profile_cap is not None:
            self._profile_cap.for_path(path)

        first_yaw = path.poses[0].orientation.euler[2]
        robot_yaw = current_odom.orientation.euler[2]
        yaw_err = angle_diff(first_yaw, robot_yaw)
        self._controller.reset_yaw_error(yaw_err)

        if abs(yaw_err) < self._config.orientation_tolerance:
            # Note: production LocalPlanner transitions to "final_rotation" when
            # the robot is exactly at path[0] (pos_d < 0.01). That's broken for
            # open paths - we'd snap to "arrived" immediately. Always start in
            # path_following when aligned; arrival is detected by distance_to_goal.
            self._state = "path_following"
        else:
            self._state = "initial_rotation"

        logger.info(
            f"BaselinePathFollowerTask '{self._name}' started "
            f"({len(path.poses)} poses, initial state={self._state})"
        )
        return True

    def update_odom(self, odom: PoseStamped) -> None:
        # Pose now flows in through compute()'s CoordinatorState (sourced
        # from the twist-base adapter's read_odometry → joint positions).
        # This setter is kept as a no-op-or-override seam so out-of-tree
        # callers that still pump odom externally don't break.
        self._current_odom = odom

    def cancel(self) -> bool:
        if not self.is_active():
            return False
        self._state = "aborted"
        return True

    def reset(self) -> bool:
        if self.is_active():
            return False
        self._state = "idle"
        self._path = None
        self._distancer = None
        self._current_odom = None
        return True

    def get_state(self) -> BaselineState:
        return self._state


__all__ = [
    "BaselinePathFollowerTask",
    "BaselinePathFollowerTaskConfig",
]
