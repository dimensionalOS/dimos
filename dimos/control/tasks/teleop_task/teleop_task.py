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

"""Engagement-relative teleop control through command-integrating Pink IK."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import TYPE_CHECKING, Literal

import numpy as np
import pinocchio
from pydantic import Field, FiniteFloat

from dimos.control.coordinator import TaskConfig
from dimos.control.task import CoordinatorState, JointCommandOutput, ResourceClaim
from dimos.control.tasks.cartesian_ik_task.cartesian_ik_task import (
    CartesianIKTask,
    CartesianIKTaskConfig,
    CartesianIKTaskParams,
    append_gripper_position,
    claim_with_gripper,
)
from dimos.control.tasks.cartesian_ik_task.pink_control_ik import PinkControlIKConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.teleop.quest.quest_types import Buttons

logger = setup_logger()


class _EngagementState(Enum):
    DISENGAGED = auto()
    ENGAGED = auto()
    WAITING_FOR_RELEASE = auto()


class TeleopControlIKConfig(PinkControlIKConfig):
    """Pink control policy for engagement-relative arm teleoperation."""

    max_velocity: FiniteFloat = Field(1.0, gt=0.0)
    position_cost: FiniteFloat = Field(1.0, ge=0.0)
    orientation_cost: FiniteFloat = Field(1.0, ge=0.0)
    posture_cost: FiniteFloat = Field(0.0, ge=0.0)
    joint_centering_cost: FiniteFloat = Field(1e-3, ge=0.0)
    damping_cost: FiniteFloat = Field(1e-3, ge=0.0)


@dataclass
class TeleopIKTaskConfig(CartesianIKTaskConfig):
    """Configuration for engagement-relative teleop IK."""

    max_joint_delta_deg: float = 5.0
    hand: Literal["left", "right"] | None = None
    gripper_joint: str | None = None
    gripper_open_pos: float = 0.0
    gripper_closed_pos: float = 0.0


class TeleopIKTask(CartesianIKTask):
    """Cartesian IK specialization for engagement-relative teleoperation."""

    _config: TeleopIKTaskConfig

    def __init__(self, name: str, config: TeleopIKTaskConfig) -> None:
        if config.hand not in ("left", "right"):
            raise ValueError(f"TeleopIKTask '{name}' requires hand='left' or 'right'")
        super().__init__(name, config)
        self._initial_ee_pose: pinocchio.SE3 | None = None
        self._engagement = _EngagementState.DISENGAGED
        self._primary_down = False
        self._estopped = False
        self._gripper_target = config.gripper_open_pos
        self._gripper_active = config.gripper_joint is not None

    def claim(self) -> ResourceClaim:
        """Claim arm joints and the optional gripper joint."""
        return claim_with_gripper(super().claim(), self._config.gripper_joint)

    def is_active(self) -> bool:
        """Run only when a non-E-STOPped pose target is active."""
        with self._lock:
            return not self._estopped and self._active and self._target_pose is not None

    def is_tracking(self) -> bool:
        """Report whether teleop currently participates in control."""
        return self.is_active()

    def set_estop(self, estopped: bool) -> None:
        """Latch or clear E-STOP without retaining replayable commands."""
        with self._lock:
            self._estopped = estopped
            if estopped:
                self._engagement = _EngagementState.WAITING_FOR_RELEASE
                self._active = False
                self._target_pose = None
                self._initial_ee_pose = None
                self._last_commanded_joints = None
                self._gripper_active = False
            else:
                self._engagement = (
                    _EngagementState.WAITING_FOR_RELEASE
                    if self._primary_down
                    else _EngagementState.DISENGAGED
                )

    def _prepare_target(
        self,
        state: CoordinatorState,
        q_current: NDArray[np.float64],
        dt: float,
    ) -> pinocchio.SE3 | None:
        """Compose the controller delta with the measured engagement baseline."""
        delta = super()._prepare_target(state, q_current, dt)
        if delta is None:
            return None

        with self._lock:
            if self._estopped or self._target_pose is None:
                return None
            baseline = self._initial_ee_pose

        if baseline is None:
            captured = self.forward_kinematics(q_current)
            values = np.concatenate((captured.translation, captured.rotation.reshape(-1)))
            if not np.all(np.isfinite(values)):
                return None
            with self._lock:
                if self._estopped or self._target_pose is None:
                    return None
                if self._initial_ee_pose is None:
                    self._initial_ee_pose = captured.copy()
                baseline = self._initial_ee_pose

        target = pinocchio.SE3(
            delta.rotation @ baseline.rotation,
            baseline.translation + delta.translation,
        )
        values = np.concatenate((target.translation, target.rotation.reshape(-1)))
        if not np.all(np.isfinite(values)):
            return None
        return target

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Run the inherited Pink solve and append the optional gripper target."""
        output = super().compute(state)
        with self._lock:
            if self._estopped:
                return None
            gripper_target = self._gripper_target
            gripper_joint = self._config.gripper_joint if self._gripper_active else None
        return append_gripper_position(output, gripper_joint, gripper_target)

    def on_buttons(self, msg: Buttons) -> bool:
        """Use the configured primary button as press-and-hold engagement."""
        is_left = self._config.hand == "left"
        primary = msg.left_primary if is_left else msg.right_primary
        trigger = msg.left_trigger_analog if is_left else msg.right_trigger_analog

        with self._lock:
            was_primary_down = self._primary_down
            self._primary_down = primary
            if self._estopped:
                return False
            if self._engagement is _EngagementState.WAITING_FOR_RELEASE:
                if not primary:
                    self._engagement = _EngagementState.DISENGAGED
            elif (
                self._engagement is _EngagementState.DISENGAGED and primary and not was_primary_down
            ):
                self._engagement = _EngagementState.ENGAGED
                self._active = False
                self._target_pose = None
                self._initial_ee_pose = None
                self._last_commanded_joints = None
            elif self._engagement is _EngagementState.ENGAGED and not primary:
                self._engagement = _EngagementState.DISENGAGED
                self._active = False
                self._target_pose = None
                self._initial_ee_pose = None
                self._last_commanded_joints = None

        if self._config.gripper_joint is not None:
            self.on_gripper_trigger(trigger)
        return True

    def on_teleop_buttons(self, msg: Buttons, t_now: float) -> bool:
        """Uniform stream handler for broadcast controller buttons."""
        return self.on_buttons(msg)

    def on_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        """Accept an engagement-relative pose delta only while its button is held."""
        with self._lock:
            if self._estopped or self._engagement is not _EngagementState.ENGAGED:
                return False
            if not self._active:
                self._last_commanded_joints = None
            self._target_pose = pose
            self._last_update_time = t_now
            self._active = True
        return True

    def on_gripper_trigger(self, value: float, _t_now: float = 0.0) -> bool:
        """Map an analog trigger value onto the configured gripper range."""
        if self._config.gripper_joint is None or not np.isfinite(value):
            return False
        clamped = max(0.0, min(1.0, value))
        position = (
            self._config.gripper_open_pos
            + (self._config.gripper_closed_pos - self._config.gripper_open_pos) * clamped
        )
        with self._lock:
            if self._estopped:
                return False
            self._gripper_target = position
            self._gripper_active = True
        return True

    def _on_timeout(self) -> None:
        """Discard the baseline while the parent holds the task lock."""
        self._initial_ee_pose = None
        self._engagement = (
            _EngagementState.WAITING_FOR_RELEASE
            if self._primary_down
            else _EngagementState.DISENGAGED
        )

    def stop(self) -> None:
        """Stop output and discard engagement-relative state."""
        super().stop()
        self._reset_engagement_state()

    def _reset_engagement_state(self) -> None:
        """Discard state owned specifically by engagement-relative teleop."""
        with self._lock:
            self._initial_ee_pose = None
            self._engagement = _EngagementState.DISENGAGED
            self._primary_down = False
            self._gripper_active = False

    def clear(self) -> None:
        """Clear output and discard engagement-relative state."""
        super().clear()
        self._reset_engagement_state()


class TeleopIKTaskParams(CartesianIKTaskParams):
    control_ik: TeleopControlIKConfig
    hand: Literal["left", "right"] | None = None
    max_joint_delta_deg: float = 5.0
    gripper_joint: str | None = None
    gripper_open_pos: float = 0.0
    gripper_closed_pos: float = 0.0


def create_task(cfg: TaskConfig, hardware: object) -> TeleopIKTask:
    """Create a Pink-backed teleop task from declarative configuration."""
    params = TeleopIKTaskParams.model_validate(cfg.params)
    return TeleopIKTask(
        cfg.name,
        TeleopIKTaskConfig(
            joint_names=cfg.joint_names,
            control_ik=params.control_ik,
            priority=cfg.priority,
            timeout=params.timeout,
            max_joint_delta_deg=params.max_joint_delta_deg,
            max_tracking_error_deg=params.max_tracking_error_deg,
            min_dt=params.min_dt,
            max_dt=params.max_dt,
            hand=params.hand,
            gripper_joint=params.gripper_joint,
            gripper_open_pos=params.gripper_open_pos,
            gripper_closed_pos=params.gripper_closed_pos,
        ),
    )
