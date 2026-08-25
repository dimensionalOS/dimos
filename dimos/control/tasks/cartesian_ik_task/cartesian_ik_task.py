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

"""Absolute Cartesian pose leaf over the shared bounded Pink IK task."""

from __future__ import annotations

from collections.abc import Mapping
import threading
from typing import TYPE_CHECKING

import attrs

from dimos.control.task import CoordinatorState
from dimos.control.tasks.pose_target_ik import (
    FrameTargetSnapshot,
    PinkPoseTargetSolver,
    PoseTargetIKTask,
    PoseTargetIKTaskConfig,
    PoseTargetIKTaskParams,
    string_tuple_converter,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

if TYPE_CHECKING:
    from dimos.control.coordinator import TaskConfig
    from dimos.control.hardware_interface import ConnectedHardware, ConnectedWholeBody


@attrs.frozen(slots=False)
class CartesianIKTaskConfig(PoseTargetIKTaskConfig):
    """Configuration for one absolute Cartesian target frame."""

    target_frames: tuple[str, ...] = attrs.field(
        default=(),
        converter=string_tuple_converter,
        validator=[attrs.validators.min_len(1), attrs.validators.max_len(1)],
    )


class CartesianIKTask(PoseTargetIKTask):
    """Track one stream of absolute poses with the shared Pink control core."""

    def __init__(
        self,
        name: str,
        config: CartesianIKTaskConfig,
        *,
        solver: PinkPoseTargetSolver | None = None,
    ) -> None:
        self._lock = threading.Lock()
        self._target_pose: PoseStamped | None = None
        self._last_update_time = 0.0
        self._active = False
        super().__init__(name, config, solver=solver)

    def is_active(self) -> bool:
        with self._lock:
            return self._active and self._target_pose is not None

    def on_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        """Accept an absolute target pose and activate tracking."""
        target = PoseStamped(
            ts=pose.ts if isinstance(pose, PoseStamped) else 0.0,
            frame_id=pose.frame_id if isinstance(pose, PoseStamped) else "",
            position=pose.position,
            orientation=pose.orientation,
        )
        with self._lock:
            self._target_pose = target
            self._last_update_time = t_now
            self._active = True
        return True

    def start(self) -> None:
        with self._lock:
            self._active = True

    def stop(self) -> None:
        with self._lock:
            self._active = False
        self._reset_command_state()

    def clear(self) -> None:
        with self._lock:
            self._target_pose = None
            self._active = False
        self._reset_command_state()

    def is_tracking(self) -> bool:
        return self.is_active()

    def _frame_target_snapshot(self, state: CoordinatorState) -> FrameTargetSnapshot | None:
        with self._lock:
            if not self._active or self._target_pose is None:
                return None
            return FrameTargetSnapshot(
                targets={self._config.target_frames[0]: self._target_pose},
                last_update_time=self._last_update_time,
            )

    def _on_target_timeout(self) -> None:
        self.clear()

    def _on_pose_target_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        self.clear()


class CartesianIKTaskParams(PoseTargetIKTaskParams):
    """Task-owned parameters carried inside the generic task envelope."""

    target_frame: str


def create_task(
    cfg: TaskConfig,
    hardware: Mapping[str, ConnectedHardware | ConnectedWholeBody],
) -> CartesianIKTask:
    """Create an absolute Cartesian Pink task from a registry configuration."""
    params = CartesianIKTaskParams.model_validate(cfg.params)
    return CartesianIKTask(
        cfg.name,
        CartesianIKTaskConfig(
            joint_names=tuple(cfg.joint_names),
            robot_model=params.robot_model,
            target_frames=(params.target_frame,),
            pink=params.pink,
            priority=cfg.priority,
            timeout=params.timeout,
            max_joint_velocity_rad_s=params.max_joint_velocity_rad_s,
            joint_velocity_limits_rad_s=params.joint_velocity_limits_rad_s,
            joint_command_filter_cutoff_hz=params.joint_command_filter_cutoff_hz,
            max_command_tracking_error_deg=params.max_command_tracking_error_deg,
            feedback_limit_tolerance=params.feedback_limit_tolerance,
            command_limit_margin=params.command_limit_margin,
        ),
    )
