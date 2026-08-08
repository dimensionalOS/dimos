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

from collections.abc import Mapping
from dataclasses import dataclass, field
import threading
from typing import Any, Literal

from pydantic import Field

from dimos.control.task import CoordinatorState
from dimos.control.tasks.pose_target_ik import (
    FrameTargetSnapshot,
    PinkPoseTargetSolver,
    PoseTargetIKTask,
    PoseTargetIKTaskConfig,
)
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.protocol.service.spec import BaseConfig
from dimos.teleop.quest.quest_types import Buttons

OperatorHand = Literal["left", "right"]


@dataclass(frozen=True)
class TeleopHandBinding:
    """Bind one operator hand to one robot frame and optional gripper."""

    hand: OperatorHand
    target_frame: str
    gripper_joint: str | None = None
    gripper_open_position: float = 0.0
    gripper_closed_position: float = 0.0


@dataclass(frozen=True)
class TeleopIKTaskConfig:
    """Configuration for single-arm or bimanual pose control."""

    joint_names: tuple[str, ...]
    robot_model: RobotModelConfig
    bindings: tuple[TeleopHandBinding, ...]
    pink: PinkKinematicsConfig = field(default_factory=PinkKinematicsConfig)
    priority: int = 10
    timeout: float = 0.5
    max_joint_velocity_rad_s: float = 5.0
    joint_velocity_limits_rad_s: dict[str, float] = field(default_factory=dict)
    joint_command_filter_cutoff_hz: float | None = 5.0
    max_command_tracking_error_deg: float = 10.0
    feedback_limit_tolerance: float = 1e-3
    command_limit_margin: float = 1e-4


@dataclass
class _HandState:
    latest_pose: PoseStamped | None = None
    last_update_time: float = 0.0
    controller_reference: PoseStamped | None = None
    robot_reference: PoseStamped | None = None
    gripper_target: float = 0.0


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
        self._validate_bindings(name, config)
        self._teleop_config = config
        self._bindings = {binding.hand: binding for binding in config.bindings}
        self._lock = threading.Lock()
        self._hands = {
            binding.hand: _HandState(gripper_target=binding.gripper_open_position)
            for binding in config.bindings
        }
        self._engagement_condition = False
        self._engaged = False
        self._estopped = False
        self._engagement_generation = 0
        gripper_joints = tuple(
            binding.gripper_joint
            for binding in config.bindings
            if binding.gripper_joint is not None
        )
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
            additional_claimed_joints=gripper_joints,
            solver=solver,
            solver_type=solver_type,
        )

    @staticmethod
    def _validate_bindings(name: str, config: TeleopIKTaskConfig) -> None:
        if not 1 <= len(config.bindings) <= 2:
            raise ValueError(f"TeleopIKTask '{name}' requires exactly one or two hand bindings")
        hands = [binding.hand for binding in config.bindings]
        frames = [binding.target_frame for binding in config.bindings]
        grippers = [
            binding.gripper_joint
            for binding in config.bindings
            if binding.gripper_joint is not None
        ]
        if any(hand not in ("left", "right") for hand in hands):
            raise ValueError(f"TeleopIKTask '{name}' has an unknown operator hand")
        if len(set(hands)) != len(hands):
            raise ValueError(f"TeleopIKTask '{name}' requires unique operator hands")
        if any(not frame for frame in frames) or len(set(frames)) != len(frames):
            raise ValueError(f"TeleopIKTask '{name}' requires unique target frames")
        if len(set(grippers)) != len(grippers):
            raise ValueError(f"TeleopIKTask '{name}' requires unique gripper joints")

    def is_active(self) -> bool:
        with self._lock:
            return (
                not self._estopped
                and self._engaged
                and all(state.latest_pose is not None for state in self._hands.values())
            )

    def set_estop(self, estopped: bool) -> None:
        """Latch or clear E-STOP; latching clears the complete session."""
        with self._lock:
            self._estopped = estopped
            if estopped:
                self._disengage_locked()

    def on_left_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        """Store the latest absolute left-controller pose."""
        return self._on_controller_pose("left", pose, t_now)

    def on_right_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        """Store the latest absolute right-controller pose."""
        return self._on_controller_pose("right", pose, t_now)

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
            if self._estopped:
                return False
            state = self._hands[hand]
            state.latest_pose = sample
            state.last_update_time = t_now
        return True

    def on_teleop_buttons(self, msg: Buttons, t_now: float) -> bool:
        """Update the all-bound-hands deadman condition and gripper targets."""
        del t_now
        primary_by_hand = {
            "left": msg.left_primary,
            "right": msg.right_primary,
        }
        trigger_by_hand = {
            "left": msg.left_trigger_analog,
            "right": msg.right_trigger_analog,
        }
        with self._lock:
            for hand, binding in self._bindings.items():
                if binding.gripper_joint is None:
                    continue
                trigger = max(0.0, min(1.0, trigger_by_hand[hand]))
                self._hands[hand].gripper_target = (
                    binding.gripper_open_position
                    + (binding.gripper_closed_position - binding.gripper_open_position) * trigger
                )

            condition = all(primary_by_hand[hand] for hand in self._bindings)
            if self._estopped:
                condition = False
            if condition and not self._engagement_condition:
                self._engage_locked()
            elif not condition and self._engagement_condition:
                self._disengage_locked()
            self._engagement_condition = condition
        return True

    def _engage_locked(self) -> None:
        self._engaged = True
        self._engagement_generation += 1
        for state in self._hands.values():
            state.latest_pose = None
            state.last_update_time = 0.0
            state.controller_reference = None
            state.robot_reference = None

    def _disengage_locked(self) -> None:
        self._reset_command_state()
        self._engaged = False
        self._engagement_condition = False
        self._engagement_generation += 1
        for state in self._hands.values():
            state.latest_pose = None
            state.last_update_time = 0.0
            state.controller_reference = None
            state.robot_reference = None

    def _frame_target_snapshot(self, state: CoordinatorState) -> FrameTargetSnapshot | None:
        with self._lock:
            if (
                self._estopped
                or not self._engaged
                or any(hand.latest_pose is None for hand in self._hands.values())
            ):
                return None
            generation = self._engagement_generation
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
                    not self._engaged
                    or generation != self._engagement_generation
                    or any(hand.latest_pose is None for hand in self._hands.values())
                ):
                    return None
                for hand, binding in self._bindings.items():
                    hand_state = self._hands[hand]
                    hand_state.controller_reference = hand_state.latest_pose
                    hand_state.robot_reference = robot_poses[binding.target_frame]

        with self._lock:
            if not self._engaged or generation != self._engagement_generation:
                return None
            targets: dict[str, PoseStamped] = {}
            extras: dict[str, float] = {}
            update_times: list[float] = []
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
                if binding.gripper_joint is not None:
                    extras[binding.gripper_joint] = hand_state.gripper_target
            return FrameTargetSnapshot(
                targets=targets,
                last_update_time=min(update_times),
                extra_joint_positions=extras,
            )

    def _on_target_timeout(self) -> None:
        with self._lock:
            self._disengage_locked()

    def _on_pose_target_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        with self._lock:
            self._disengage_locked()

    def start(self) -> None:
        """Teleop tasks remain inert until their deadman condition is met."""

    def stop(self) -> None:
        with self._lock:
            self._disengage_locked()


class TeleopHandBindingParams(BaseConfig):
    """Serialized hand binding carried by `TaskConfig.params`."""

    hand: OperatorHand
    target_frame: str
    gripper_joint: str | None = None
    gripper_open_position: float = 0.0
    gripper_closed_position: float = 0.0


class TeleopIKTaskParams(BaseConfig):
    """Task-owned parameters carried inside the generic task envelope."""

    robot_model: RobotModelConfig
    bindings: list[TeleopHandBindingParams]
    pink: PinkKinematicsConfig = Field(default_factory=PinkKinematicsConfig)
    solver_type: type[PinkPoseTargetSolver] = PinkPoseTargetSolver
    timeout: float = 0.5
    max_joint_velocity_rad_s: float = 5.0
    joint_velocity_limits_rad_s: dict[str, float] = Field(default_factory=dict)
    joint_command_filter_cutoff_hz: float | None = 5.0
    max_command_tracking_error_deg: float = 10.0
    feedback_limit_tolerance: float = 1e-3
    command_limit_margin: float = 1e-4


def create_task(
    cfg: Any,
    hardware: Mapping[str, Any],
) -> TeleopIKTask:
    """Create and validate a pose-target teleop task from registry configuration."""
    params = TeleopIKTaskParams.model_validate(cfg.params)
    bindings = tuple(
        TeleopHandBinding(
            hand=binding.hand,
            target_frame=binding.target_frame,
            gripper_joint=binding.gripper_joint,
            gripper_open_position=binding.gripper_open_position,
            gripper_closed_position=binding.gripper_closed_position,
        )
        for binding in params.bindings
    )
    available_joints = {
        joint_name for connected in hardware.values() for joint_name in connected.joint_names
    }
    unknown_grippers = {
        binding.gripper_joint
        for binding in bindings
        if binding.gripper_joint is not None and binding.gripper_joint not in available_joints
    }
    if unknown_grippers:
        raise ValueError(f"Teleop task references unknown gripper joints: {unknown_grippers}")
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
