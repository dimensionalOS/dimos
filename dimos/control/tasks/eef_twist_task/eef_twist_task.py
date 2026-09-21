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

"""Command-integrating end-effector twist control over shared Pink IK."""

from __future__ import annotations

from collections.abc import Mapping
import threading
from typing import TYPE_CHECKING

import attrs
import numpy as np
import pinocchio

from dimos.control.task import CoordinatorState
from dimos.control.tasks.pose_target_ik import (
    FrameTargetSnapshot,
    PinkPoseTargetSolver,
    PoseTargetIKTask,
    PoseTargetIKTaskConfig,
    PoseTargetIKTaskParams,
    string_tuple_converter,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import matrix_to_pose, pose_to_matrix, twist_to_numpy

if TYPE_CHECKING:
    from dimos.control.coordinator import TaskConfig
    from dimos.control.hardware_interface import ConnectedHardware, ConnectedWholeBody
    from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped

logger = setup_logger()


@attrs.frozen(slots=False)
class EEFTwistTaskConfig(PoseTargetIKTaskConfig):
    """Configuration for command-relative end-effector twist control."""

    target_frames: tuple[str, ...] = attrs.field(
        default=(),
        converter=string_tuple_converter,
        validator=[attrs.validators.min_len(1), attrs.validators.max_len(1)],
    )
    command_timeout: float = attrs.field(default=0.3, converter=float)


class EEFTwistTask(PoseTargetIKTask):
    """Integrate twists into a Cartesian target solved by the shared IK core."""

    _config: EEFTwistTaskConfig

    def __init__(
        self,
        name: str,
        config: EEFTwistTaskConfig,
        *,
        solver: PinkPoseTargetSolver | None = None,
    ) -> None:
        self._input_lock = threading.Lock()
        self._latest_twist: TwistStamped | None = None
        self._last_twist_time = 0.0
        self._target_pose: PoseStamped | None = None
        self._estopped = False
        super().__init__(name, config, solver=solver)

    def is_active(self) -> bool:
        with self._input_lock:
            return not self._estopped

    def is_tracking(self) -> bool:
        return self.is_active()

    def on_cartesian_command(self, pose: object, t_now: float) -> bool:
        """Reject Cartesian commands because this task accepts only twists."""
        logger.warning("EEFTwistTask rejects Cartesian commands", task=self._name)
        return False

    def on_ee_twist_command(self, twist: TwistStamped, t_now: float) -> bool:
        values = twist_to_numpy(twist)
        if values.shape != (6,) or not np.all(np.isfinite(values)):
            logger.warning("EEFTwistTask rejecting invalid twist", task=self._name)
            return False
        with self._input_lock:
            if self._estopped:
                return False
            self._latest_twist = None if np.allclose(values, 0.0) else twist
            self._last_twist_time = t_now
        return True

    def set_estop(self, estopped: bool) -> None:
        with self._input_lock:
            self._estopped = estopped
            if estopped:
                self._latest_twist = None
                self._target_pose = None
        self._reset_command_state()

    def start(self) -> None:
        return None

    def stop(self) -> None:
        self.clear()

    def clear(self) -> None:
        with self._input_lock:
            self._latest_twist = None
            self._target_pose = None
        self._reset_command_state()

    def _frame_target_snapshot(self, state: CoordinatorState) -> FrameTargetSnapshot | None:
        with self._input_lock:
            if self._estopped:
                return None
            if (
                self._latest_twist is not None
                and self._config.command_timeout > 0.0
                and state.t_now - self._last_twist_time > self._config.command_timeout
            ):
                self._latest_twist = None
            twist = self._latest_twist
            target = self._target_pose

        if target is None:
            poses = self.current_frame_poses(state, self._config.target_frames)
            if poses is None:
                return None
            target = poses[self._config.target_frames[0]]

        if twist is not None:
            target = _integrate_twist(target, twist, max(state.dt, 0.0))

        with self._input_lock:
            if self._estopped:
                return None
            self._target_pose = target

        return FrameTargetSnapshot(
            targets={self._config.target_frames[0]: target},
            last_update_time=state.t_now,
        )

    def _on_target_timeout(self) -> None:
        self.clear()

    def _on_pose_target_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        with self._input_lock:
            self._latest_twist = None
            self._target_pose = None


class EEFTwistTaskParams(PoseTargetIKTaskParams):
    target_frame: str
    timeout: float = 0.3


def create_task(
    cfg: TaskConfig,
    hardware: Mapping[str, ConnectedHardware | ConnectedWholeBody],
) -> EEFTwistTask:
    del hardware
    params = EEFTwistTaskParams.model_validate(cfg.params)
    return EEFTwistTask(
        cfg.name,
        EEFTwistTaskConfig(
            joint_names=tuple(cfg.joint_names),
            robot_model=params.robot_model,
            target_frames=(params.target_frame,),
            pink=params.pink,
            priority=cfg.priority,
            timeout=0.0,
            command_timeout=params.timeout,
            max_joint_velocity_rad_s=params.max_joint_velocity_rad_s,
            joint_velocity_limits_rad_s=params.joint_velocity_limits_rad_s,
            joint_command_filter_cutoff_hz=params.joint_command_filter_cutoff_hz,
            max_command_tracking_error_deg=params.max_command_tracking_error_deg,
            feedback_limit_tolerance=params.feedback_limit_tolerance,
            command_limit_margin=params.command_limit_margin,
        ),
    )


def _integrate_twist(pose: PoseStamped, twist: TwistStamped, dt: float) -> PoseStamped:
    matrix = np.asarray(pose_to_matrix(pose), dtype=np.float64)
    values = twist_to_numpy(twist)
    matrix[:3, 3] += values[:3] * dt
    angular_step = values[3:] * dt
    if np.linalg.norm(angular_step) > 0.0:
        matrix[:3, :3] = pinocchio.exp3(angular_step) @ matrix[:3, :3]
    integrated = matrix_to_pose(matrix)
    return PoseStamped(
        ts=pose.ts,
        frame_id=pose.frame_id,
        position=integrated.position,
        orientation=integrated.orientation,
    )
