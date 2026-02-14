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

"""Teleop cartesian control task with internal Pinocchio IK solver.

Accepts streaming cartesian delta poses from teleoperation and computes
inverse kinematics internally to output joint commands. Deltas are applied
relative to the EE pose captured at engage time.

Participates in joint-level arbitration.

CRITICAL: Uses t_now from CoordinatorState, never calls time.time()
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import TYPE_CHECKING, Any

import numpy as np
import pinocchio  # type: ignore[import-untyped]

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.manipulation.planning.kinematics.pinocchio_ik import (
    PinocchioIK,
    check_joint_delta,
    pose_to_se3,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from pathlib import Path

    from numpy.typing import NDArray

    from dimos.msgs.geometry_msgs import Pose, PoseStamped
    from dimos.teleop.quest.quest_types import QuestButtons

logger = setup_logger()


@dataclass
class TeleopIKTaskConfig:
    """Configuration for teleop IK task.

    Attributes:
        joint_names: List of joint names this task controls (must match model DOF)
        model_path: Path to URDF or MJCF file for IK solver
        ee_joint_id: End-effector joint ID in the kinematic chain
        priority: Priority for arbitration (higher wins)
        timeout: If no command received for this many seconds, go inactive (0 = never)
        max_joint_delta_deg: Maximum allowed joint change per tick (safety limit)
        hand: "left" or "right" — which controller's primary button to listen to
    """

    joint_names: list[str]
    model_path: str | Path
    ee_joint_id: int
    priority: int = 10
    timeout: float = 0.5
    max_joint_delta_deg: float = 5.0  # ~500°/s at 100Hz
    hand: str = ""


class TeleopIKTask(BaseControlTask):
    """Teleop cartesian control task with internal Pinocchio IK solver.

    Accepts streaming cartesian delta poses via on_cartesian_command() and computes IK
    internally to output joint commands. Deltas are applied relative to the EE pose
    captured at engage time (first compute).

    Uses current joint state from CoordinatorState as IK warm-start for fast convergence.
    Outputs JointCommandOutput and participates in joint-level arbitration.

    Example:
        >>> from dimos.utils.data import get_data
        >>> piper_path = get_data("piper_description")
        >>> task = TeleopIKTask(
        ...     name="teleop_arm",
        ...     config=TeleopIKTaskConfig(
        ...         joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        ...         model_path=piper_path / "mujoco_model" / "piper_no_gripper_description.xml",
        ...         ee_joint_id=6,
        ...         priority=10,
        ...         timeout=0.5,
        ...     ),
        ... )
        >>> coordinator.add_task(task)
        >>> task.start()
        >>>
        >>> # From teleop callback:
        >>> task.on_cartesian_command(delta_pose, t_now=time.perf_counter())
    """

    def __init__(self, name: str, config: TeleopIKTaskConfig) -> None:
        """Initialize teleop IK task.

        Args:
            name: Unique task name
            config: Task configuration
        """
        if not config.joint_names:
            raise ValueError(f"TeleopIKTask '{name}' requires at least one joint")
        if not config.model_path:
            raise ValueError(f"TeleopIKTask '{name}' requires model_path for IK solver")

        self._name = name
        self._config = config
        self._joint_names = frozenset(config.joint_names)
        self._joint_names_list = list(config.joint_names)
        self._num_joints = len(config.joint_names)

        # Create IK solver from model
        self._ik = PinocchioIK.from_model_path(config.model_path, config.ee_joint_id)

        # Validate DOF matches joint names
        if self._ik.nq != self._num_joints:
            logger.warning(
                f"TeleopIKTask {name}: model DOF ({self._ik.nq}) != "
                f"joint_names count ({self._num_joints})"
            )

        # Thread-safe target state
        self._lock = threading.Lock()
        self._target_pose: Pose | PoseStamped | None = None
        self._last_update_time: float = 0.0
        self._active = False

        # Initial EE pose for delta application
        self._initial_ee_pose: pinocchio.SE3 | None = None
        self._prev_primary: bool = False

        logger.info(
            f"TeleopIKTask {name} initialized with model: {config.model_path}, "
            f"ee_joint_id={config.ee_joint_id}, joints={config.joint_names}"
        )

    @property
    def name(self) -> str:
        """Unique task identifier."""
        return self._name

    def claim(self) -> ResourceClaim:
        """Declare resource requirements."""
        return ResourceClaim(
            joints=self._joint_names,
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        """Check if task should run this tick."""
        with self._lock:
            return self._active and self._target_pose is not None

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Compute IK and output joint positions.

        Args:
            state: Current coordinator state (contains joint positions for IK warm-start)

        Returns:
            JointCommandOutput with positions, or None if inactive/timed out/IK failed
        """
        with self._lock:
            if not self._active or self._target_pose is None:
                return None

            # Timeout safety: stop if teleop stream drops
            if self._config.timeout > 0:
                time_since_update = state.t_now - self._last_update_time
                if time_since_update > self._config.timeout:
                    logger.warning(
                        f"TeleopIKTask {self._name} timed out "
                        f"(no update for {time_since_update:.3f}s)"
                    )
                    self._target_pose = None
                    self._active = False
                    return None
            raw_pose = self._target_pose

        # Convert to SE3 right before use
        delta_se3 = pose_to_se3(raw_pose)
        # Capture initial EE pose if not set (first command after engage)
        with self._lock:
            need_capture = self._initial_ee_pose is None

        if need_capture:
            q_current = self._get_current_joints(state)
            if q_current is None:
                logger.debug(
                    f"TeleopIKTask {self._name}: cannot capture initial pose, joint state unavailable"
                )
                return None
            initial_pose = self._ik.forward_kinematics(q_current)
            with self._lock:
                self._initial_ee_pose = initial_pose

        # Apply delta to initial pose: target = initial + delta
        with self._lock:
            if self._initial_ee_pose is None:
                return None
            target_pose = pinocchio.SE3(
                delta_se3.rotation @ self._initial_ee_pose.rotation,
                self._initial_ee_pose.translation + delta_se3.translation,
            )

        # Get current joint positions for IK warm-start
        q_current = self._get_current_joints(state)
        if q_current is None:
            logger.debug(f"TeleopIKTask {self._name}: missing joint state for IK warm-start")
            return None

        # Compute IK
        q_solution, converged, final_error = self._ik.solve(target_pose, q_current)
        # Use the solution even if it didn't fully converge
        if not converged:
            logger.debug(
                f"TeleopIKTask {self._name}: IK did not converge "
                f"(error={final_error:.4f}), using partial solution"
            )
        # Safety: reject if any joint would jump too far in one tick
        if not check_joint_delta(q_solution, q_current, self._config.max_joint_delta_deg):
            logger.warning(
                f"TeleopIKTask {self._name}: joint delta exceeds "
                f"{self._config.max_joint_delta_deg}°, rejecting solution"
            )
            return None

        positions = q_solution.flatten().tolist()
        return JointCommandOutput(
            joint_names=self._joint_names_list,
            positions=positions,
            mode=ControlMode.SERVO_POSITION,
        )

    def _get_current_joints(self, state: CoordinatorState) -> NDArray[np.floating[Any]] | None:
        """Get current joint positions from coordinator state."""
        positions = []
        for joint_name in self._joint_names_list:
            pos = state.joints.get_position(joint_name)
            if pos is None:
                return None
            positions.append(pos)
        return np.array(positions)

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        """Handle preemption by higher-priority task.

        Args:
            by_task: Name of preempting task
            joints: Joints that were preempted
        """
        if joints & self._joint_names:
            logger.warning(f"TeleopIKTask {self._name} preempted by {by_task} on joints {joints}")

    # =========================================================================
    # Task-specific methods
    # =========================================================================

    def on_buttons(self, msg: QuestButtons) -> bool:
        """Press-and-hold engage: hold primary button to track, release to stop.

        Checks only the button matching self._config.hand (left_x or right_a).
        If hand is not set, listens to both.
        """
        hand = self._config.hand
        if hand == "left":
            primary = msg.left_x
        elif hand == "right":
            primary = msg.right_a
        else:
            primary = msg.left_x or msg.right_a

        if primary and not self._prev_primary:
            # Rising edge: reset initial pose so compute() recaptures
            logger.info(f"TeleopIKTask {self._name}: engage")
            with self._lock:
                self._initial_ee_pose = None
        elif not primary and self._prev_primary:
            # Falling edge: stop tracking
            logger.info(f"TeleopIKTask {self._name}: disengage")
            with self._lock:
                self._target_pose = None
                self._initial_ee_pose = None
        self._prev_primary = primary
        return True

    def on_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        """Handle incoming cartesian command (delta pose from teleop)"""
        with self._lock:
            self._target_pose = pose  # Store raw, convert to SE3 in compute()
            self._last_update_time = t_now
            self._active = True

        return True

    def start(self) -> None:
        """Activate the task (start accepting and outputting commands)."""
        with self._lock:
            self._active = True
        logger.info(f"TeleopIKTask {self._name} started")

    def stop(self) -> None:
        """Deactivate the task (stop outputting commands)."""
        with self._lock:
            self._active = False
        logger.info(f"TeleopIKTask {self._name} stopped")


__all__ = [
    "TeleopIKTask",
    "TeleopIKTaskConfig",
]
