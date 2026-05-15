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

"""Pink-backed teleoperation IK control tasks.

These tasks preserve the passive ControlCoordinator contract while using Pink
to solve differential IK from Quest controller pose deltas.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import threading
from typing import TYPE_CHECKING, Any, Literal

import numpy as np
import pink
from pink import solve_ik
from pink.limits import AccelerationLimit
from pink.tasks import DampingTask, FrameTask, ManipulabilityTask, PostureTask
import pinocchio
import qpsolvers

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.manipulation.planning.kinematics.pinocchio_ik import check_joint_delta, pose_to_se3
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.catalog.piper import PIPER_FK_MODEL
from dimos.robot.catalog.ufactory import XARM7_FK_MODEL
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.teleop.quest.quest_types import Buttons

logger = setup_logger()


def _load_pinocchio_model(model_path: str | Path) -> pinocchio.Model:
    path = Path(str(model_path))
    if not path.exists():
        raise FileNotFoundError(f"Model file not found: {path}")
    if path.suffix == ".xml":
        return pinocchio.buildModelFromMJCF(str(path))
    return pinocchio.buildModelFromUrdf(str(path))


def _default_solver() -> str:
    if "daqp" in qpsolvers.available_solvers:
        return "daqp"
    if not qpsolvers.available_solvers:
        raise RuntimeError("Pink IK requires at least one qpsolvers backend")
    return str(qpsolvers.available_solvers[0])


def _model_joint_names(model: pinocchio.Model) -> list[str]:
    return [str(name) for name in model.names[1:]]


def _unqualified_joint_name(joint_name: str) -> str:
    return joint_name.rsplit("/", maxsplit=1)[-1]


def _validate(
    name: str,
    value: float,
    *,
    ge: float | None = None,
    le: float | None = None,
    gt: float | None = None,
) -> None:
    if not np.isfinite(value):
        raise ValueError(f"{name} must be finite")
    if ge is not None and value < ge:
        raise ValueError(f"{name} must be >= {ge}")
    if le is not None and value > le:
        raise ValueError(f"{name} must be <= {le}")
    if gt is not None and value <= gt:
        raise ValueError(f"{name} must be > {gt}")


@dataclass
class PinkIKTaskConfig:
    """Configuration shared by Pink teleop IK tasks."""

    joint_names: list[str]
    model_path: str | Path
    priority: int = 10
    timeout: float = 0.5
    max_joint_delta_deg: float = 5.0
    joint_limit_margin: float = 1e-3
    hand: Literal["left", "right"] | None = "right"
    solver: str | None = None
    damping: float = 1e-12
    end_effector_frame: str = ""
    position_cost: float = 1.0
    orientation_cost: float = 1.0
    lm_damping: float = 1.0
    gain: float = 1.0
    gripper_joint: str | None = None
    gripper_open_pos: float = 0.0
    gripper_closed_pos: float = 0.0
    posture_cost: float = 0.0
    posture_reference: dict[str, float] = field(default_factory=dict)
    posture_lm_damping: float = 0.0
    posture_gain: float = 1.0
    # Velocity smoothing
    max_joint_acceleration: float = 0.0  # rad/s^2; 0.0 disables AccelerationLimit
    velocity_smoothing_alpha: float = 1.0  # EMA blend; 1.0 = passthrough, <1.0 smooths
    # Manipulability cost (singularity avoidance)
    manipulability_cost: float = 0.0  # 0.0 disables ManipulabilityTask
    manipulability_lm_damping: float = 1e-3
    manipulability_gain: float = 1.0
    manipulability_rate: float = 1.5


@dataclass
class SingleArmPinkIKTaskConfig(PinkIKTaskConfig):
    """Configuration for one-arm, one-frame Pink teleop IK tasks."""

    damping_task_cost: float = 0.0


class BasePinkIKTask(BaseControlTask):
    """Base control task for Pink-backed teleoperation IK."""

    def __init__(self, name: str, config: PinkIKTaskConfig) -> None:
        if not config.joint_names:
            raise ValueError(f"{type(self).__name__} '{name}' requires at least one joint")
        if config.hand is not None and config.hand not in ("left", "right"):
            raise ValueError(f"{type(self).__name__} '{name}' requires hand='left' or 'right'")

        self._name = name
        self._config = config
        self._joint_names = frozenset(config.joint_names)
        self._joint_names_list = list(config.joint_names)
        self._solver = config.solver or _default_solver()

        self._model = self._prepare_model(_load_pinocchio_model(config.model_path))
        self._relax_position_limits(config.joint_limit_margin)
        self._data = self._model.createData()
        self._validate_model()

        q0 = pinocchio.neutral(self._model)
        self._configuration = pink.Configuration(self._model, self._data, q0)
        self._frame_tasks, self._pink_tasks = self._build_pink_tasks()
        self._acceleration_limit = self._build_acceleration_limit()

        self._lock = threading.Lock()
        self._active = False
        self._gripper_target = config.gripper_open_pos
        self._logged_first_output = False
        self._last_solution: NDArray[np.floating[Any]] | None = None
        self._prev_velocity: NDArray[np.floating[Any]] | None = None

    @property
    def name(self) -> str:
        """Unique task identifier."""
        return self._name

    def claim(self) -> ResourceClaim:
        """Declare resource requirements for coordinator arbitration."""
        joints = self._joint_names
        if self._config.gripper_joint:
            joints = joints | frozenset([self._config.gripper_joint])
        return ResourceClaim(
            joints=joints, priority=self._config.priority, mode=ControlMode.SERVO_POSITION
        )

    def is_active(self) -> bool:
        """Return true when this task should be considered for compute."""
        with self._lock:
            return self._active

    def start(self) -> None:
        """Activate the task so incoming targets can be consumed."""
        with self._lock:
            self._active = True
        logger.info(f"{type(self).__name__} {self._name} started")

    def stop(self) -> None:
        """Deactivate the task and clear captured target state."""
        with self._lock:
            self._active = False
            self._clear_target_state()
        logger.info(f"{type(self).__name__} {self._name} stopped")

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Run one Pink differential IK tick and return joint positions."""
        if not self._prepare_compute(state):
            return None

        q_current = self._get_current_joints(state)
        if q_current is None:
            logger.debug(
                f"{type(self).__name__} {self._name}: missing joint state for IK warm-start"
            )
            return None

        self._configuration.update(q_current)
        if not self._update_frame_targets(state, q_current):
            return None
        if not self._update_extra_task_targets(q_current):
            return None

        dt = max(state.dt, 1e-9)
        q_solution = self._solve_ik(dt)
        if q_solution is None:
            return None

        if not check_joint_delta(q_solution, q_current, self._config.max_joint_delta_deg):
            logger.warning(
                f"{type(self).__name__} {self._name}: joint delta exceeds "
                f"{self._config.max_joint_delta_deg}°, rejecting solution"
            )
            return None
        self._last_solution = q_solution

        joint_names = list(self._joint_names_list)
        positions = q_solution.flatten().tolist()
        if self._config.gripper_joint:
            with self._lock:
                gripper_pos = self._gripper_target
            joint_names.append(self._config.gripper_joint)
            positions.append(gripper_pos)

        if not self._logged_first_output:
            logger.info(
                f"{type(self).__name__} {self._name}: Pink IK output active "
                f"for {len(self._joint_names_list)} joints using solver={self._solver}"
            )
            self._logged_first_output = True

        return JointCommandOutput(
            joint_names=joint_names,
            positions=positions,
            mode=ControlMode.SERVO_POSITION,
        )

    def _solve_ik(self, dt: float) -> NDArray[np.floating[Any]] | None:
        solve_kwargs: dict[str, Any] = {
            "solver": self._solver,
            "damping": self._config.damping,
        }
        if self._acceleration_limit is not None:
            solve_kwargs["limits"] = (
                self._configuration.model.configuration_limit,
                self._configuration.model.velocity_limit,
                self._acceleration_limit,
            )
        try:
            velocity = solve_ik(
                self._configuration,
                self._pink_tasks,
                dt,
                **solve_kwargs,
            )
        except Exception as exc:
            logger.warning(f"{type(self).__name__} {self._name}: Pink IK failed: {exc}")
            return None

        velocity_array = np.asarray(velocity, dtype=float)
        if not np.isfinite(velocity_array).all():
            logger.warning(
                f"{type(self).__name__} {self._name}: Pink IK returned non-finite velocity"
            )
            return None

        alpha = self._config.velocity_smoothing_alpha
        if (
            alpha < 1.0
            and self._prev_velocity is not None
            and self._prev_velocity.shape == velocity_array.shape
        ):
            velocity_array = alpha * velocity_array + (1.0 - alpha) * self._prev_velocity
        self._prev_velocity = velocity_array.copy()

        self._configuration.integrate_inplace(velocity_array, dt)
        q_solution = np.asarray(self._configuration.q, dtype=float)
        if not np.isfinite(q_solution).all():
            logger.warning(
                f"{type(self).__name__} {self._name}: Pink IK returned non-finite output"
            )
            return None
        return q_solution

    def on_gripper_trigger(self, value: float, _t_now: float = 0.0) -> bool:
        """Map analog trigger value to configured gripper position."""
        if not self._config.gripper_joint:
            return False
        clamped = max(0.0, min(1.0, value))
        position = (
            self._config.gripper_open_pos
            + (self._config.gripper_closed_pos - self._config.gripper_open_pos) * clamped
        )
        with self._lock:
            self._gripper_target = position
        return True

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        """Clear active target state when a higher-priority task preempts us."""
        if joints & self.claim().joints:
            logger.warning(
                f"{type(self).__name__} {self._name} preempted by {by_task} on joints {joints}"
            )
            with self._lock:
                self._active = False
                self._clear_target_state()

    def _get_current_joints(self, state: CoordinatorState) -> NDArray[np.floating[Any]] | None:
        positions = []
        for joint_name in self._joint_names_list:
            position = state.joints.get_position(joint_name)
            if position is None:
                return None
            positions.append(position)
        return np.array(positions, dtype=float)

    def _prepare_model(self, model: pinocchio.Model) -> pinocchio.Model:
        return model

    def _relax_position_limits(self, margin: float) -> None:
        _validate("joint_limit_margin", margin, ge=0.0)
        if margin == 0.0:
            return
        lower = self._model.lowerPositionLimit
        upper = self._model.upperPositionLimit
        finite = np.isfinite(lower) & np.isfinite(upper)
        lower[finite] -= margin
        upper[finite] += margin

    def _validate_model(self) -> None:
        if self._model.nq != len(self._joint_names_list):
            raise ValueError(
                f"{type(self).__name__} '{self._name}' model DOF ({self._model.nq}) "
                f"does not match joint count ({len(self._joint_names_list)})"
            )

    def _build_pink_tasks(self) -> tuple[list[FrameTask], list[Any]]:
        frame_tasks = self._create_frame_tasks()
        pink_tasks = [*frame_tasks, *self._create_auxiliary_tasks()]
        return frame_tasks, pink_tasks

    def _create_frame_tasks(self) -> list[FrameTask]:
        raise NotImplementedError

    def _create_auxiliary_tasks(self) -> list[Any]:
        return []

    def _update_extra_task_targets(self, _q_current: NDArray[np.floating[Any]]) -> bool:
        return True

    def _clear_target_state(self) -> None:
        """Clear concrete target state while holding ``self._lock``."""
        self._prev_velocity = None

    def _build_acceleration_limit(self) -> AccelerationLimit | None:
        if self._config.max_joint_acceleration <= 0.0:
            return None
        bound = np.full(self._model.nv, float(self._config.max_joint_acceleration))
        return AccelerationLimit(self._model, bound)

    def _prepare_compute(self, _state: CoordinatorState) -> bool:
        return True

    def _update_frame_targets(
        self, state: CoordinatorState, q_current: NDArray[np.floating[Any]]
    ) -> bool:
        raise NotImplementedError


class SingleFramePinkIKTask(BasePinkIKTask):
    """Pink teleop IK base for concrete tasks with one controlled frame target."""

    def __init__(self, name: str, config: PinkIKTaskConfig) -> None:
        super().__init__(name, config)
        self._target_pose: Pose | PoseStamped | None = None
        self._last_update_time = 0.0
        self._initial_ee_pose: pinocchio.SE3 | None = None
        self._prev_primary = False
        self._logged_first_target = False

    def is_active(self) -> bool:
        """Return true when a live single-frame target can produce IK output."""
        with self._lock:
            return self._active and self._target_pose is not None

    def on_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        """Store the latest robot-frame controller delta pose."""
        with self._lock:
            self._target_pose = pose
            self._last_update_time = t_now
            self._active = True
        if not self._logged_first_target:
            logger.info(f"{type(self).__name__} {self._name}: received first cartesian target")
            self._logged_first_target = True
        return True

    def on_buttons(self, msg: Buttons) -> bool:
        """Press-and-hold engage with optional gripper trigger mapping."""
        is_left = self._config.hand == "left"
        primary = msg.left_primary if is_left else msg.right_primary

        if primary and not self._prev_primary:
            logger.info(f"{type(self).__name__} {self._name}: engage")
            with self._lock:
                self._initial_ee_pose = None
        elif not primary and self._prev_primary:
            logger.info(f"{type(self).__name__} {self._name}: disengage")
            with self._lock:
                self._active = False
                self._clear_target_state()
        self._prev_primary = primary

        if self._config.gripper_joint:
            trigger = msg.left_trigger_analog if is_left else msg.right_trigger_analog
            self.on_gripper_trigger(trigger)
        return True

    def _prepare_compute(self, state: CoordinatorState) -> bool:
        return self._get_live_target(state.t_now) is not None

    def _update_frame_targets(
        self, state: CoordinatorState, q_current: NDArray[np.floating[Any]]
    ) -> bool:
        raw_pose = self._get_live_target(state.t_now)
        if raw_pose is None:
            return False
        if not self._ensure_initial_ee_pose(q_current):
            return False

        with self._lock:
            initial_ee_pose = self._initial_ee_pose
        if initial_ee_pose is None:
            return False

        delta_se3 = pose_to_se3(raw_pose)
        target_pose = pinocchio.SE3(
            delta_se3.rotation @ initial_ee_pose.rotation,
            initial_ee_pose.translation + delta_se3.translation,
        )
        self._single_frame_task().set_target(target_pose)
        return True

    def _get_live_target(self, t_now: float) -> Pose | PoseStamped | None:
        with self._lock:
            if not self._active or self._target_pose is None:
                return None
            if self._config.timeout > 0:
                time_since_update = t_now - self._last_update_time
                if time_since_update > self._config.timeout:
                    logger.warning(
                        f"{type(self).__name__} {self._name} timed out "
                        f"(no update for {time_since_update:.3f}s)"
                    )
                    self._active = False
                    self._clear_target_state()
                    return None
            return self._target_pose

    def _ensure_initial_ee_pose(self, q_current: NDArray[np.floating[Any]]) -> bool:
        with self._lock:
            if self._initial_ee_pose is not None:
                return True
        initial_pose = self._capture_end_effector_pose(q_current)
        with self._lock:
            self._initial_ee_pose = initial_pose
        return True

    def _capture_end_effector_pose(self, _q_current: NDArray[np.floating[Any]]) -> pinocchio.SE3:
        return self._configuration.get_transform_frame_to_world(self._single_frame_name()).copy()

    def _single_frame_name(self) -> str:
        return str(self._single_frame_task().frame)

    def _single_frame_task(self) -> FrameTask:
        if len(self._frame_tasks) != 1:
            raise RuntimeError(
                f"{type(self).__name__} {self._name} requires exactly one frame task"
            )
        return self._frame_tasks[0]

    def _clear_target_state(self) -> None:
        super()._clear_target_state()
        self._target_pose = None
        self._initial_ee_pose = None


class SingleArmPinkIKTask(SingleFramePinkIKTask):
    """Configurable Pink teleop IK task for one arm and one end-effector frame."""

    _config: SingleArmPinkIKTaskConfig

    def __init__(self, name: str, config: SingleArmPinkIKTaskConfig) -> None:
        self._posture_task: PostureTask | None = None
        super().__init__(name, config)

    def _prepare_model(self, model: pinocchio.Model) -> pinocchio.Model:
        configured = [_unqualified_joint_name(name) for name in self._joint_names_list]
        model_joints = _model_joint_names(model)
        if model_joints == configured:
            return model
        if any(joint_name not in model_joints for joint_name in configured):
            return model

        locked_joint_ids = [
            model.getJointId(joint_name)
            for joint_name in model_joints
            if joint_name not in configured
        ]
        if not locked_joint_ids:
            return model

        return pinocchio.buildReducedModel(model, locked_joint_ids, pinocchio.neutral(model))

    def _validate_model(self) -> None:
        configured = [_unqualified_joint_name(name) for name in self._joint_names_list]
        model_joints = _model_joint_names(self._model)
        if configured != model_joints:
            raise ValueError(
                f"{type(self).__name__} '{self._name}' joint names {configured} do not match "
                f"model joints {model_joints}"
            )
        super()._validate_model()
        if not self._model.existFrame(self._config.end_effector_frame):
            raise ValueError(
                f"{type(self).__name__} '{self._name}' model has no frame "
                f"'{self._config.end_effector_frame}'"
            )
        self._validate_numeric_config()
        self._validate_posture_config()

    def _create_frame_tasks(self) -> list[FrameTask]:
        return [
            FrameTask(
                self._config.end_effector_frame,
                position_cost=self._config.position_cost,
                orientation_cost=self._config.orientation_cost,
                lm_damping=self._config.lm_damping,
                gain=self._config.gain,
            )
        ]

    def _create_auxiliary_tasks(self) -> list[Any]:
        extra_tasks: list[Any] = []

        if self._config.posture_cost > 0.0:
            self._posture_task = PostureTask(
                cost=self._config.posture_cost,
                lm_damping=self._config.posture_lm_damping,
                gain=self._config.posture_gain,
            )
            extra_tasks.append(self._posture_task)

        if self._config.damping_task_cost > 0.0:
            extra_tasks.append(DampingTask(cost=self._config.damping_task_cost))

        if self._config.manipulability_cost > 0.0:
            extra_tasks.append(
                ManipulabilityTask(
                    frame=self._config.end_effector_frame,
                    model=self._model,
                    cost=self._config.manipulability_cost,
                    lm_damping=self._config.manipulability_lm_damping,
                    gain=self._config.manipulability_gain,
                    manipulability_rate=self._config.manipulability_rate,
                )
            )

        return extra_tasks

    def _update_extra_task_targets(self, q_current: NDArray[np.floating[Any]]) -> bool:
        if self._posture_task is None:
            return True

        target = np.asarray(q_current, dtype=float).copy()
        for joint_name, position in self._config.posture_reference.items():
            target[self._posture_joint_index(joint_name)] = position
        self._posture_task.set_target(target)
        return True

    def _validate_numeric_config(self) -> None:
        _validate("damping", self._config.damping, ge=0.0)
        _validate("damping_task_cost", self._config.damping_task_cost, ge=0.0)
        _validate("position_cost", self._config.position_cost, ge=0.0)
        _validate("orientation_cost", self._config.orientation_cost, ge=0.0)
        _validate("lm_damping", self._config.lm_damping, ge=0.0)
        _validate("gain", self._config.gain, ge=0.0, le=1.0)
        _validate("timeout", self._config.timeout, ge=0.0)
        _validate("max_joint_delta_deg", self._config.max_joint_delta_deg, gt=0.0)
        _validate("max_joint_acceleration", self._config.max_joint_acceleration, ge=0.0)
        _validate("velocity_smoothing_alpha", self._config.velocity_smoothing_alpha, ge=0.0, le=1.0)
        _validate("manipulability_cost", self._config.manipulability_cost, ge=0.0)
        _validate("manipulability_lm_damping", self._config.manipulability_lm_damping, ge=0.0)
        _validate("manipulability_gain", self._config.manipulability_gain, ge=0.0, le=1.0)
        _validate("manipulability_rate", self._config.manipulability_rate)

    def _validate_posture_config(self) -> None:
        _validate("posture_cost", self._config.posture_cost, ge=0.0)
        _validate("posture_lm_damping", self._config.posture_lm_damping, ge=0.0)
        _validate("posture_gain", self._config.posture_gain, ge=0.0, le=1.0)
        for joint_name, position in self._config.posture_reference.items():
            _validate(f"posture_reference[{joint_name!r}]", position)
            index = self._posture_joint_index(joint_name)
            lower = float(self._model.lowerPositionLimit[index])
            upper = float(self._model.upperPositionLimit[index])
            if position < lower or position > upper:
                raise ValueError(
                    f"posture_reference[{joint_name!r}]={position} is outside "
                    f"model limits [{lower}, {upper}]"
                )

    def _posture_joint_index(self, joint_name: str) -> int:
        names = {n: i for i, n in enumerate(self._joint_names_list)}
        names.update({_unqualified_joint_name(n): i for n, i in names.items()})
        if joint_name not in names:
            raise ValueError(
                f"{type(self).__name__} '{self._name}' posture joint '{joint_name}' "
                f"is not in configured joints {self._joint_names_list}"
            )
        return names[joint_name]


@dataclass
class XArm7IKTaskConfig(SingleArmPinkIKTaskConfig):
    """XArm7-specific Pink teleop IK configuration."""

    model_path: str | Path = XARM7_FK_MODEL
    end_effector_frame: str = "link7"
    hand: Literal["left", "right"] | None = "right"
    posture_cost: float = 1e-3
    damping_task_cost: float = 1e-3
    max_joint_acceleration: float = 20.0
    velocity_smoothing_alpha: float = 0.6


XArm7IKTask = SingleArmPinkIKTask


@dataclass
class PiperPinkIKTaskConfig(SingleArmPinkIKTaskConfig):
    """Piper-specific Pink teleop IK configuration."""

    model_path: str | Path = PIPER_FK_MODEL
    end_effector_frame: str = "gripper_base"
    hand: Literal["left", "right"] | None = "right"
    damping_task_cost: float = 1e-3
    posture_cost: float = 1e-3
    max_joint_acceleration: float = 0.0
    velocity_smoothing_alpha: float = 0.8
    manipulability_cost: float = 0.01


PiperPinkIKTask = SingleArmPinkIKTask


__all__ = [
    "BasePinkIKTask",
    "PinkIKTaskConfig",
    "SingleFramePinkIKTask",
    "XArm7IKTask",
    "XArm7IKTaskConfig",
]
