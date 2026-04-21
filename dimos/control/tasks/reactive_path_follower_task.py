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

"""Reactive path-following ControlTask using the Lyapunov controller.

Same integration contract as PathFollowerTask (state machine, 10 Hz
decimation, ControlTask protocol) but with a fully reactive control law:
ALL velocity components (vx, vy, wz) are computed from the robot's current
error state relative to the path.  No precomputed velocity profiles.

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
from dimos.control.tasks.lyapunov_path_controller import (
    LyapunovPathController,
    LyapunovPathControllerConfig,
)
from dimos.control.tasks.path_distancer import PathDistancer
from dimos.utils.logging_config import setup_logger
from dimos.utils.trigonometry import angle_diff

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.nav_msgs.Path import Path

logger = setup_logger()

ReactivePathFollowerState = Literal["idle", "following", "completed", "aborted"]


@dataclass
class ReactivePathFollowerTaskConfig:
    """Configuration for the reactive (Lyapunov) path follower task."""

    joint_names: list[str] = field(default_factory=lambda: ["go2_vx", "go2_vy", "go2_wz"])
    priority: int = 20
    control_frequency: float = 10.0  # effective output rate (Hz)

    # Goal tolerance
    goal_tolerance: float = 0.2
    orientation_tolerance: float = 0.15

    # Lyapunov controller config (all reactive gains)
    controller: LyapunovPathControllerConfig = field(default_factory=LyapunovPathControllerConfig)


class ReactivePathFollowerTask(BaseControlTask):
    """ControlTask that follows a path using the Lyapunov reactive controller.

    All velocity components (vx, vy, wz) are computed from the robot's
    instantaneous error state — no precomputed velocity profiles.
    """

    def __init__(
        self,
        name: str,
        config: ReactivePathFollowerTaskConfig,
    ) -> None:
        if len(config.joint_names) != 3:
            raise ValueError(
                f"ReactivePathFollowerTask '{name}' requires exactly 3 joints "
                f"(vx, vy, wz), got {len(config.joint_names)}"
            )

        self._name = name
        self._config = config
        self._joint_names = frozenset(config.joint_names)
        self._joint_names_list = list(config.joint_names)

        # State machine
        self._state: ReactivePathFollowerState = "idle"

        # Path data
        self._path: Path | None = None
        self._path_distancer: PathDistancer | None = None
        self._current_odom: PoseStamped | None = None

        # Controller
        self._controller = LyapunovPathController(config.controller)

        # 10 Hz decimation
        self._control_period = 1.0 / config.control_frequency
        self._last_compute_time: float = 0.0
        self._cached_output: JointCommandOutput | None = None

        logger.info(
            f"ReactivePathFollowerTask '{name}' initialised "
            f"(Lyapunov, {config.control_frequency} Hz, joints={config.joint_names})"
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
        if self._state != "following" or self._path is None or self._path_distancer is None:
            return None
        if self._current_odom is None:
            return None

        # Decimation
        elapsed = state.t_now - self._last_compute_time
        if elapsed < self._control_period and self._cached_output is not None:
            return self._cached_output

        self._last_compute_time = state.t_now
        output = self._compute_control()
        self._cached_output = output
        return output

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        if joints & self._joint_names:
            logger.warning(f"ReactivePathFollowerTask '{self._name}' preempted by {by_task}")
            if self._state == "following":
                self._state = "aborted"

    # ------------------------------------------------------------------
    # Control computation
    # ------------------------------------------------------------------

    def _compute_control(self) -> JointCommandOutput:
        assert self._path is not None
        assert self._path_distancer is not None
        assert self._current_odom is not None

        odom = self._current_odom
        distancer = self._path_distancer
        current_pos = np.array([odom.position.x, odom.position.y])
        distance_to_goal = distancer.distance_to_goal(current_pos)

        # ---- Goal reached check ----
        if distance_to_goal < self._config.goal_tolerance and len(self._path.poses) > 0:
            goal_yaw = self._path.poses[-1].orientation.euler[2]
            robot_yaw = odom.orientation.euler[2]
            yaw_err = angle_diff(goal_yaw, robot_yaw)

            if abs(yaw_err) < self._config.orientation_tolerance:
                self._state = "completed"
                logger.info(f"ReactivePathFollowerTask '{self._name}' completed — goal reached")
                return self._zero_output()

            # Final rotation: use heading-only control
            cfg = self._config.controller
            wz = float(
                np.clip(
                    cfg.k_theta * np.sin(yaw_err),
                    -cfg.wz_max,
                    cfg.wz_max,
                )
            )
            return JointCommandOutput(
                joint_names=self._joint_names_list,
                velocities=[0.0, 0.0, wz],
                mode=ControlMode.VELOCITY,
            )

        # ---- Reactive Lyapunov control ----
        out = self._controller.compute(odom, distancer, distance_to_goal)

        return JointCommandOutput(
            joint_names=self._joint_names_list,
            velocities=[out.vx, out.vy, out.wz],
            mode=ControlMode.VELOCITY,
        )

    def _zero_output(self) -> JointCommandOutput:
        return JointCommandOutput(
            joint_names=self._joint_names_list,
            velocities=[0.0, 0.0, 0.0],
            mode=ControlMode.VELOCITY,
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start_path(self, path: Path, current_odom: PoseStamped) -> bool:
        if path is None or len(path.poses) < 2:
            logger.warning(
                f"ReactivePathFollowerTask '{self._name}': invalid path (need >= 2 poses)"
            )
            return False

        self._path = path
        self._path_distancer = PathDistancer(path)
        self._current_odom = current_odom
        self._state = "following"
        self._cached_output = None
        self._last_compute_time = 0.0

        logger.info(f"ReactivePathFollowerTask '{self._name}' started ({len(path.poses)} poses)")
        return True

    def update_odom(self, odom: PoseStamped) -> None:
        self._current_odom = odom

    def cancel(self) -> bool:
        if self._state != "following":
            return False
        self._state = "aborted"
        self._cached_output = None
        logger.info(f"ReactivePathFollowerTask '{self._name}' cancelled")
        return True

    def reset(self) -> bool:
        if self._state == "following":
            logger.warning(f"Cannot reset '{self._name}' while following")
            return False
        self._state = "idle"
        self._path = None
        self._path_distancer = None
        self._current_odom = None
        self._cached_output = None
        logger.info(f"ReactivePathFollowerTask '{self._name}' reset to IDLE")
        return True

    def get_state(self) -> ReactivePathFollowerState:
        return self._state


__all__ = [
    "ReactivePathFollowerTask",
    "ReactivePathFollowerTaskConfig",
]
