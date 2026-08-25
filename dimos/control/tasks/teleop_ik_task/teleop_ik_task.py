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

"""One- or two-hand device-independent teleoperation over the Pink IK core."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from enum import Enum
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
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.protocol.service.spec import BaseConfig
from dimos.teleop.quest.quest_types import Buttons

if TYPE_CHECKING:
    from dimos.control.coordinator import TaskConfig
    from dimos.control.hardware_interface import ConnectedHardware, ConnectedWholeBody


class OperatorHand(str, Enum):
    """Operator input channels supported by pose-target teleoperation."""

    LEFT = "left"
    RIGHT = "right"


def _binding_tuple_converter(
    values: Iterable[TeleopHandBinding],
) -> tuple[TeleopHandBinding, ...]:
    return tuple(values)


@attrs.frozen(slots=False)
class TeleopHandBinding:
    """Bind one operator hand to one robot frame."""

    hand: OperatorHand = attrs.field(converter=OperatorHand)
    target_frame: str


@attrs.frozen(slots=False)
class TeleopIKTaskConfig:
    """Configuration for single-arm or bimanual pose control."""

    joint_names: tuple[str, ...] = attrs.field(converter=string_tuple_converter)
    robot_model: RobotModelConfig
    bindings: tuple[TeleopHandBinding, ...] = attrs.field(converter=_binding_tuple_converter)
    pink: PinkKinematicsConfig = attrs.field(factory=PinkKinematicsConfig)
    priority: int = 10
    timeout: float = 0.5
    max_joint_velocity_rad_s: float = 5.0
    joint_velocity_limits_rad_s: dict[str, float] = attrs.field(factory=dict)
    joint_command_filter_cutoff_hz: float | None = 5.0
    max_command_tracking_error_deg: float = 10.0
    feedback_limit_tolerance: float = 1e-3
    command_limit_margin: float = 1e-4

    def __attrs_post_init__(self) -> None:
        if not 1 <= len(self.bindings) <= 2:
            raise ValueError("TeleopIKTask requires exactly one or two hand bindings")
        hands = [binding.hand for binding in self.bindings]
        frames = [binding.target_frame for binding in self.bindings]
        if len(set(hands)) != len(hands):
            raise ValueError("TeleopIKTask requires unique operator hands")
        if any(not frame for frame in frames) or len(set(frames)) != len(frames):
            raise ValueError("TeleopIKTask requires unique target frames")


@dataclass
class _HandState:
    latest_pose: PoseStamped | None = None
    last_update_time: float = 0.0
    controller_reference: PoseStamped | None = None
    robot_reference: PoseStamped | None = None


class _SessionState(Enum):
    DISENGAGED = "disengaged"
    ENGAGED = "engaged"
    ESTOPPED = "estopped"


class TeleopIKTask(PoseTargetIKTask):
    """Map one or two controller streams into absolute robot frame targets."""

    def __init__(
        self,
        name: str,
        config: TeleopIKTaskConfig,
        *,
        solver: PinkPoseTargetSolver | None = None,
        solver_type: type[PinkPoseTargetSolver] | None = None,
    ) -> None:
        self._teleop_config = config
        self._bindings = {binding.hand: binding for binding in config.bindings}
        self._lock = threading.Lock()
        self._hands = {binding.hand: _HandState() for binding in config.bindings}
        self._session_state = _SessionState.DISENGAGED
        self._session_epoch = 0
        self._last_button_update_time = 0.0
        super().__init__(
            name,
            PoseTargetIKTaskConfig(
                joint_names=config.joint_names,
                robot_model=config.robot_model,
                target_frames=tuple(binding.target_frame for binding in config.bindings),
                pink=config.pink,
                priority=config.priority,
                timeout=config.timeout,
                max_joint_velocity_rad_s=config.max_joint_velocity_rad_s,
                joint_velocity_limits_rad_s=config.joint_velocity_limits_rad_s,
                joint_command_filter_cutoff_hz=config.joint_command_filter_cutoff_hz,
                max_command_tracking_error_deg=config.max_command_tracking_error_deg,
                feedback_limit_tolerance=config.feedback_limit_tolerance,
                command_limit_margin=config.command_limit_margin,
            ),
            solver=solver,
            solver_type=solver_type,
        )

    def is_active(self) -> bool:
        with self._lock:
            return self._session_state is _SessionState.ENGAGED and all(
                state.latest_pose is not None for state in self._hands.values()
            )

    def set_estop(self, estopped: bool) -> None:
        """Latch or clear E-STOP; latching clears the complete session."""
        with self._lock:
            if estopped:
                self._end_session_locked(_SessionState.ESTOPPED)
            elif self._session_state is _SessionState.ESTOPPED:
                self._session_state = _SessionState.DISENGAGED

    def on_left_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        """Store the latest absolute left-controller pose."""
        return self._on_controller_pose(OperatorHand.LEFT, pose, t_now)

    def on_right_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        """Store the latest absolute right-controller pose."""
        return self._on_controller_pose(OperatorHand.RIGHT, pose, t_now)

    def _on_controller_pose(
        self, hand: OperatorHand, pose: Pose | PoseStamped, t_now: float
    ) -> bool:
        if hand not in self._bindings:
            return False
        sample = PoseStamped(
            ts=pose.ts if isinstance(pose, PoseStamped) else 0.0,
            frame_id=pose.frame_id if isinstance(pose, PoseStamped) else "",
            position=pose.position,
            orientation=pose.orientation,
        )
        with self._lock:
            if self._session_state is _SessionState.ESTOPPED:
                return False
            state = self._hands[hand]
            state.latest_pose = sample
            state.last_update_time = t_now
        return True

    def on_teleop_buttons(self, msg: Buttons, t_now: float) -> bool:
        """Update the all-bound-hands deadman condition."""
        primary_by_hand = {
            OperatorHand.LEFT: msg.left_primary,
            OperatorHand.RIGHT: msg.right_primary,
        }
        with self._lock:
            self._last_button_update_time = t_now
            condition = all(primary_by_hand[hand] for hand in self._bindings)
            if self._session_state is _SessionState.ESTOPPED:
                return True
            if condition and self._session_state is _SessionState.DISENGAGED:
                self._engage_locked()
            elif not condition and self._session_state is _SessionState.ENGAGED:
                self._end_session_locked(_SessionState.DISENGAGED)
        return True

    def _engage_locked(self) -> None:
        self._session_state = _SessionState.ENGAGED
        self._session_epoch += 1
        self._clear_hand_session_locked()

    def _end_session_locked(self, state: _SessionState) -> None:
        self._reset_command_state()
        self._session_state = state
        self._session_epoch += 1
        self._last_button_update_time = 0.0
        self._clear_hand_session_locked()

    def _clear_hand_session_locked(self) -> None:
        for hand_state in self._hands.values():
            hand_state.latest_pose = None
            hand_state.last_update_time = 0.0
            hand_state.controller_reference = None
            hand_state.robot_reference = None

    def _frame_target_snapshot(self, state: CoordinatorState) -> FrameTargetSnapshot | None:
        with self._lock:
            if self._session_state is not _SessionState.ENGAGED or any(
                hand.latest_pose is None for hand in self._hands.values()
            ):
                return None
            session_epoch = self._session_epoch
            needs_capture = any(
                hand.controller_reference is None or hand.robot_reference is None
                for hand in self._hands.values()
            )

        if needs_capture:
            frame_names = [binding.target_frame for binding in self._teleop_config.bindings]
            robot_poses = self.current_frame_poses(state, frame_names)
            if robot_poses is None:
                return None
            with self._lock:
                if (
                    self._session_state is not _SessionState.ENGAGED
                    or session_epoch != self._session_epoch
                    or any(hand.latest_pose is None for hand in self._hands.values())
                ):
                    return None
                for hand, binding in self._bindings.items():
                    hand_state = self._hands[hand]
                    hand_state.controller_reference = hand_state.latest_pose
                    hand_state.robot_reference = robot_poses[binding.target_frame]

        with self._lock:
            if (
                self._session_state is not _SessionState.ENGAGED
                or session_epoch != self._session_epoch
            ):
                return None
            targets: dict[str, PoseStamped] = {}
            update_times = [self._last_button_update_time]
            for hand, binding in self._bindings.items():
                hand_state = self._hands[hand]
                current = hand_state.latest_pose
                controller_reference = hand_state.controller_reference
                robot_reference = hand_state.robot_reference
                if current is None or controller_reference is None or robot_reference is None:
                    return None
                delta = current - controller_reference
                targets[binding.target_frame] = PoseStamped(
                    frame_id=self._teleop_config.robot_model.base_pose.frame_id,
                    position=robot_reference.position + delta.position,
                    orientation=delta.orientation * robot_reference.orientation,
                )
                update_times.append(hand_state.last_update_time)
            return FrameTargetSnapshot(
                targets=targets,
                last_update_time=min(update_times),
            )

    def _on_target_timeout(self) -> None:
        with self._lock:
            self._end_session_locked(_SessionState.DISENGAGED)

    def _on_pose_target_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        with self._lock:
            self._end_session_locked(_SessionState.DISENGAGED)

    def start(self) -> None:
        """Teleop tasks remain inert until their deadman condition is met."""

    def stop(self) -> None:
        with self._lock:
            self._end_session_locked(_SessionState.DISENGAGED)


class TeleopHandBindingParams(BaseConfig):
    """Serialized hand binding carried by `TaskConfig.params`."""

    hand: OperatorHand
    target_frame: str


class TeleopIKTaskParams(PoseTargetIKTaskParams):
    """Task-owned parameters carried inside the generic task envelope."""

    bindings: list[TeleopHandBindingParams]
    solver_type: type[PinkPoseTargetSolver] = PinkPoseTargetSolver


def create_task(
    cfg: TaskConfig,
    hardware: Mapping[str, ConnectedHardware | ConnectedWholeBody],
) -> TeleopIKTask:
    """Create and validate a pose-target teleop task from registry configuration."""
    params = TeleopIKTaskParams.model_validate(cfg.params)
    bindings = tuple(
        TeleopHandBinding(
            hand=binding.hand,
            target_frame=binding.target_frame,
        )
        for binding in params.bindings
    )
    del hardware
    task_config = TeleopIKTaskConfig(
        joint_names=tuple(cfg.joint_names),
        robot_model=params.robot_model,
        bindings=bindings,
        pink=params.pink,
        priority=cfg.priority,
        timeout=params.timeout,
        max_joint_velocity_rad_s=params.max_joint_velocity_rad_s,
        joint_velocity_limits_rad_s=params.joint_velocity_limits_rad_s,
        joint_command_filter_cutoff_hz=params.joint_command_filter_cutoff_hz,
        max_command_tracking_error_deg=params.max_command_tracking_error_deg,
        feedback_limit_tolerance=params.feedback_limit_tolerance,
        command_limit_margin=params.command_limit_margin,
    )
    return TeleopIKTask(
        cfg.name,
        task_config,
        solver_type=params.solver_type,
    )
