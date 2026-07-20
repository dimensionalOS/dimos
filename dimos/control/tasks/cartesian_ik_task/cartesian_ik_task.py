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

"""Cartesian control task with Pink differential IK by default.

Accepts streaming cartesian poses (e.g., from teleoperation, visual servoing)
and computes inverse kinematics internally to output joint commands.
Participates in joint-level arbitration.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import TYPE_CHECKING

import numpy as np
import pinocchio
from pydantic import FiniteFloat

from dimos.control.coordinator import TaskConfig
from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.control.tasks.cartesian_ik_task.pink_control_ik import (
    PinkControlIK,
    PinkControlIKConfig,
)
from dimos.manipulation.planning.kinematics.pinocchio_ik import (
    check_joint_delta,
    get_worst_joint_delta,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

logger = setup_logger()


@dataclass
class CartesianIKTaskConfig:
    """Configuration for cartesian IK task.

    Attributes:
        joint_names: List of joint names this task controls (must match model DOF)
        priority: Priority for arbitration (higher wins)
        timeout: If no command received for this many seconds, go inactive (0 = never)
        max_joint_delta_deg: Maximum allowed joint change per tick (safety limit)
    """

    joint_names: list[str]
    control_ik: PinkControlIKConfig
    priority: int = 10
    timeout: float = 0.5
    max_joint_delta_deg: float = 15.0  # ~1500°/s at 100Hz
    min_dt: FiniteFloat = 1e-4
    max_dt: FiniteFloat = 0.05

    def __post_init__(self) -> None:
        if (
            not np.isfinite(self.min_dt)
            or not np.isfinite(self.max_dt)
            or self.min_dt <= 0.0
            or self.max_dt <= 0.0
        ):
            raise ValueError("CartesianIKTask dt bounds must be finite and positive")
        if self.max_dt < self.min_dt:
            raise ValueError("CartesianIKTask dt bounds must be ordered")


class CartesianIKTask(BaseControlTask):
    """Cartesian control task with Pink differential IK.

    Accepts streaming cartesian poses via on_cartesian_command() and computes IK
    internally to output joint commands. Pink re-anchors each solve to the
    current joint state from CoordinatorState.

    Unlike CartesianServoTask (which bypasses joint arbitration), this task
    outputs JointCommandOutput and participates in joint-level arbitration.

    Example:
        >>> from dimos.robot.manipulators.piper.config import make_piper_model_config
        >>> task = CartesianIKTask(
        ...     name="cartesian_arm",
        ...     config=CartesianIKTaskConfig(
        ...         joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        ...         control_ik=PinkControlIKConfig(
        ...             robot_model=make_piper_model_config(),
        ...         ),
        ...         priority=10,
        ...         timeout=0.5,
        ...     ),
        ... )
        >>> coordinator.add_task(task)
        >>> task.start()
        >>>
        >>> # From teleop callback or other source:
        >>> task.on_cartesian_command(pose_stamped, t_now=time.perf_counter())
    """

    def __init__(self, name: str, config: CartesianIKTaskConfig) -> None:
        """Initialize cartesian IK task.

        Args:
            name: Unique task name
            config: Task configuration
        """
        if not config.joint_names or len(set(config.joint_names)) != len(config.joint_names):
            raise ValueError(f"CartesianIKTask '{name}' requires at least one joint")
        if not np.isfinite(config.timeout) or config.timeout < 0.0:
            raise ValueError("CartesianIKTask timeout must be finite and non-negative")
        if not np.isfinite(config.max_joint_delta_deg) or config.max_joint_delta_deg <= 0.0:
            raise ValueError("CartesianIKTask max_joint_delta_deg must be positive and finite")

        self._name = name
        self._config = config
        self._joint_names = frozenset(config.joint_names)
        self._joint_names_list = list(config.joint_names)
        self._num_joints = len(config.joint_names)
        expected_joints = config.control_ik.robot_model.get_coordinator_joint_names()
        if config.joint_names != expected_joints:
            raise ValueError(
                f"CartesianIKTask {name}: task joints must match RobotModelConfig coordinator joints"
            )

        # Create IK solver from model
        self._ik = PinkControlIK(config.control_ik)

        # Validate DOF matches joint names
        if self._ik.nq != self._num_joints:
            raise ValueError(
                f"CartesianIKTask {name}: model DOF ({self._ik.nq}) != "
                f"joint_names count ({self._num_joints})"
            )

        # Thread-safe target state
        self._lock = threading.Lock()
        self._target_pose: Pose | PoseStamped | None = None
        self._last_update_time: float = 0.0
        self._active = False

        logger.info(
            f"CartesianIKTask {name} initialized with model: "
            f"{config.control_ik.robot_model.model_path}, "
            f"joints={config.joint_names}"
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
            state: Current coordinator state (contains measured joint positions)

        Returns:
            JointCommandOutput with positions or a measured-state hold after an
            expected runtime failure; None if inactive or timed out.
        """
        with self._lock:
            if not self._active or (self._target_pose is None and not self._uses_prepared_target()):
                return None
            # Check timeout
            if self._config.timeout > 0:
                time_since_update = state.t_now - self._last_update_time
                if time_since_update > self._config.timeout:
                    logger.warning(
                        f"CartesianIKTask {self._name} timed out "
                        f"(no update for {time_since_update:.3f}s)"
                    )
                    self._active = False
                    self._target_pose = None
                    self._on_timeout()
                    return None

        q_current = self._get_current_joints(state)
        if q_current is None:
            logger.debug(f"CartesianIKTask {self._name}: missing joint state for IK warm-start")
            return None
        if not np.all(np.isfinite(q_current)):
            logger.error("CartesianIKTask %s: measured joint state is non-finite", self._name)
            return None
        raw_dt = state.dt
        if not np.isfinite(raw_dt) or raw_dt <= 0.0:
            return self._hold(q_current)
        dt = min(max(raw_dt, self._config.min_dt), self._config.max_dt)
        try:
            target_pose = self._prepare_target(state, q_current, dt)
        except (FloatingPointError, RuntimeError, ValueError) as exc:
            logger.warning("CartesianIKTask %s: target preparation failed: %s", self._name, exc)
            return self._hold(q_current)
        if target_pose is None:
            return self._hold(q_current)

        # Compute IK
        try:
            result = self._ik.solve(target_pose, q_current, dt)
        except (FloatingPointError, RuntimeError, ValueError) as exc:
            logger.warning("CartesianIKTask %s: IK solve failed: %s", self._name, exc)
            return self._hold(q_current)
        q_solution = np.asarray(result.positions, dtype=np.float64).reshape(-1)
        if not np.all(np.isfinite(q_solution)) or q_solution.shape != q_current.shape:
            logger.warning("CartesianIKTask %s: rejecting invalid IK output", self._name)
            return self._hold(q_current)

        # Safety check: reject if any joint delta exceeds limit
        if not check_joint_delta(q_solution, q_current, self._config.max_joint_delta_deg):
            worst_idx, worst_deg = get_worst_joint_delta(q_solution, q_current)
            logger.warning(
                f"CartesianIKTask {self._name}: rejecting motion - "
                f"joint {self._joint_names_list[worst_idx]} delta "
                f"{worst_deg:.1f}° exceeds limit {self._config.max_joint_delta_deg}°"
            )
            return self._hold(q_current)

        return JointCommandOutput(
            joint_names=self._joint_names_list,
            positions=q_solution.flatten().tolist(),
            mode=ControlMode.SERVO_POSITION,
        )

    def _hold(self, q_current: NDArray[np.float64]) -> JointCommandOutput:
        """Keep the measured configuration under the task's servo contract."""
        return JointCommandOutput(
            joint_names=self._joint_names_list,
            positions=q_current.tolist(),
            mode=ControlMode.SERVO_POSITION,
        )

    def _get_current_joints(self, state: CoordinatorState) -> NDArray[np.float64] | None:
        """Get the measured coordinator joint snapshot (never a command cache)."""
        positions = []
        for joint_name in self._joint_names_list:
            pos = state.joints.get_position(joint_name)
            if pos is None:
                return None
            positions.append(pos)
        return np.array(positions, dtype=np.float64)

    def _prepare_target(
        self,
        state: CoordinatorState,
        q_current: NDArray[np.float64],
        dt: float,
    ) -> pinocchio.SE3 | None:
        """Prepare one normalized target for the measured-state solve."""
        with self._lock:
            pose = self._target_pose
        if pose is None:
            return None
        quaternion = np.array(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            dtype=np.float64,
        )
        quaternion_norm = float(np.linalg.norm(quaternion))
        if not np.isfinite(quaternion_norm) or quaternion_norm <= 1e-12:
            return None
        normalized = quaternion / quaternion_norm
        target = pinocchio.SE3(
            pinocchio.Quaternion(
                normalized[3], normalized[0], normalized[1], normalized[2]
            ).toRotationMatrix(),
            np.array([pose.x, pose.y, pose.z], dtype=np.float64),
        )
        values = np.concatenate((target.translation, target.rotation.reshape(-1)))
        if not np.all(np.isfinite(values)):
            return None
        return target

    def _on_timeout(self) -> None:
        """Hook for target sources with state outside the Cartesian pose cache."""

    def _uses_prepared_target(self) -> bool:
        return False

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        """Handle preemption by higher-priority task.

        Args:
            by_task: Name of preempting task
            joints: Joints that were preempted
        """
        if joints & self._joint_names:
            logger.warning(
                f"CartesianIKTask {self._name} preempted by {by_task} on joints {joints}"
            )

    def on_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        """Handle incoming cartesian command (target EE pose).

        Args:
            pose: Target end-effector pose (Pose or PoseStamped)
            t_now: Current time (from coordinator or time.perf_counter())

        Returns:
            True if accepted
        """
        with self._lock:
            self._target_pose = pose  # Store raw, convert to SE3 in compute()
            self._last_update_time = t_now
            self._active = True

        return True

    def start(self) -> None:
        """Activate the task (start accepting and outputting commands)."""
        with self._lock:
            self._active = True
        logger.info(f"CartesianIKTask {self._name} started")

    def stop(self) -> None:
        """Deactivate the task (stop outputting commands)."""
        with self._lock:
            self._active = False
            self._target_pose = None
        logger.info(f"CartesianIKTask {self._name} stopped")

    def clear(self) -> None:
        """Clear current target and deactivate."""
        with self._lock:
            self._target_pose = None
            self._active = False
        logger.info(f"CartesianIKTask {self._name} cleared")

    def is_tracking(self) -> bool:
        """Check if actively receiving and outputting commands."""
        with self._lock:
            return self._active and self._target_pose is not None

    def get_current_ee_pose(self, state: CoordinatorState) -> pinocchio.SE3 | None:
        """Get current end-effector pose via forward kinematics.

        Useful for getting initial pose before starting tracking.

        Args:
            state: Current coordinator state

        Returns:
            Current EE pose as SE3, or None if joint state unavailable
        """
        q_current = self._get_current_joints(state)
        if q_current is None:
            return None

        return self._ik.forward_kinematics(q_current)

    def forward_kinematics(self, joint_positions: NDArray[np.float64]) -> pinocchio.SE3:
        """Compute end-effector pose from joint positions.

        Args:
            joint_positions: Joint angles in radians

        Returns:
            End-effector pose as SE3
        """
        return self._ik.forward_kinematics(joint_positions)


class CartesianIKTaskParams(BaseConfig):
    control_ik: PinkControlIKConfig
    min_dt: FiniteFloat = 1e-4
    max_dt: FiniteFloat = 0.05


def create_task(cfg: TaskConfig, hardware: object) -> CartesianIKTask:
    params = CartesianIKTaskParams.model_validate(cfg.params)
    return CartesianIKTask(
        cfg.name,
        CartesianIKTaskConfig(
            joint_names=cfg.joint_names,
            priority=cfg.priority,
            min_dt=params.min_dt,
            max_dt=params.max_dt,
            control_ik=params.control_ik,
        ),
    )
