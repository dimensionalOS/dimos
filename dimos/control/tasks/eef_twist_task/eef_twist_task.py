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

"""Command-integrating end-effector twist control."""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import TYPE_CHECKING

import numpy as np
import pinocchio

from dimos.control.coordinator import TaskConfig
from dimos.control.task import CoordinatorState, JointCommandOutput, ResourceClaim
from dimos.control.tasks.cartesian_ik_task.cartesian_ik_task import (
    CartesianIKTask,
    CartesianIKTaskConfig,
    CartesianIKTaskParams,
    append_gripper_position,
    claim_with_gripper,
)
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import twist_to_numpy

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
    from dimos.msgs.std_msgs.Bool import Bool

logger = setup_logger()


@dataclass
class EEFTwistTaskConfig(CartesianIKTaskConfig):
    """Configuration for command-relative EEF twist control."""

    gripper_joint: str | None = None
    gripper_open_pos: float = 0.0
    gripper_closed_pos: float = 0.0
    workspace_min: tuple[float, float, float] | None = None
    workspace_max: tuple[float, float, float] | None = None
    max_reach_m: float | None = None
    max_orientation_delta_rad: float | None = None
    home_orientation_rpy: tuple[float, float, float] | None = None
    max_home_orientation_delta_rpy: tuple[float, float, float] | None = None

    def __post_init__(self) -> None:
        super().__post_init__()
        if (self.workspace_min is None) != (self.workspace_max is None):
            raise ValueError("EEF twist workspace bounds must be configured together")
        if self.workspace_min is not None and self.workspace_max is not None:
            lower = np.asarray(self.workspace_min, dtype=np.float64)
            upper = np.asarray(self.workspace_max, dtype=np.float64)
            if (
                not np.all(np.isfinite(lower))
                or not np.all(np.isfinite(upper))
                or np.any(lower > upper)
            ):
                raise ValueError("EEF twist workspace bounds must be finite and ordered")
        if self.max_reach_m is not None and (
            not np.isfinite(self.max_reach_m) or self.max_reach_m <= 0.0
        ):
            raise ValueError("EEF twist maximum reach must be positive and finite")
        if self.max_orientation_delta_rad is not None and (
            not np.isfinite(self.max_orientation_delta_rad) or self.max_orientation_delta_rad <= 0.0
        ):
            raise ValueError("EEF twist maximum orientation delta must be positive and finite")
        if (self.home_orientation_rpy is None) != (self.max_home_orientation_delta_rpy is None):
            raise ValueError("EEF twist home orientation limits must be configured together")
        if (
            self.home_orientation_rpy is not None
            and self.max_home_orientation_delta_rpy is not None
        ):
            home = np.asarray(self.home_orientation_rpy, dtype=np.float64)
            maximum = np.asarray(self.max_home_orientation_delta_rpy, dtype=np.float64)
            if (
                not np.all(np.isfinite(home))
                or not np.all(np.isfinite(maximum))
                or np.any(maximum <= 0.0)
            ):
                raise ValueError("EEF twist home orientation limits must be finite and positive")


class EEFTwistTask(CartesianIKTask):
    """Integrate twists from the last accepted command while the stream is active."""

    _config: EEFTwistTaskConfig

    def __init__(self, name: str, config: EEFTwistTaskConfig) -> None:
        super().__init__(name, config)
        self._twist_lock = threading.Lock()
        self._latest_twist: TwistStamped | None = None
        self._orientation_baseline: np.ndarray | None = None
        self._estopped = False
        self._gripper_target = config.gripper_open_pos
        self._gripper_active = config.gripper_joint is not None

    def claim(self) -> ResourceClaim:
        return claim_with_gripper(super().claim(), self._config.gripper_joint)

    def is_active(self) -> bool:
        with self._twist_lock:
            has_twist = self._latest_twist is not None
            estopped = self._estopped
        with self._lock:
            has_gripper_hold = self._config.gripper_joint is not None and self._active
            return not estopped and (has_twist or has_gripper_hold) and self._active

    def is_tracking(self) -> bool:
        return self.is_active()

    def _uses_prepared_target(self) -> bool:
        return True

    def on_cartesian_command(self, pose: object, t_now: float) -> bool:
        """Reject Cartesian stream commands; twist is this task's only input."""
        logger.warning("EEFTwistTask rejects Cartesian commands", task=self.name)
        return False

    def on_ee_twist_command(self, twist: TwistStamped, t_now: float) -> bool:
        values = twist_to_numpy(twist)
        if values.shape != (6,) or not np.all(np.isfinite(values)):
            logger.warning("EEFTwistTask rejecting invalid twist", task=self.name)
            return False
        with self._twist_lock:
            if self._estopped:
                return False
            if np.allclose(values, 0.0):
                self._latest_twist = None
                self._orientation_baseline = None
                cleared = True
            else:
                if self._latest_twist is None:
                    self._orientation_baseline = None
                self._latest_twist = twist
                cleared = False
        if cleared:
            self._reset_command_state()
        if cleared and self._config.gripper_joint is None:
            super().clear()
            return True
        with self._lock:
            if not self._active:
                self._last_commanded_joints = None
            self._last_update_time = t_now
            self._active = True
        return True

    def on_gripper_command(self, msg: Bool, t_now: float) -> bool:
        if self._config.gripper_joint is None:
            return False
        with self._twist_lock:
            if self._estopped:
                return False
            self._gripper_target = (
                self._config.gripper_closed_pos if msg.data else self._config.gripper_open_pos
            )
            self._gripper_active = True
        with self._lock:
            if not self._active:
                self._last_commanded_joints = None
            self._last_update_time = t_now
            self._active = True
        return True

    def set_estop(self, estopped: bool) -> None:
        with self._twist_lock:
            self._estopped = estopped
            if estopped:
                self._latest_twist = None
                self._orientation_baseline = None
                self._gripper_active = False
        if estopped:
            super().clear()

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        output = super().compute(state)
        with self._twist_lock:
            gripper_target = self._gripper_target
            gripper_joint = self._config.gripper_joint if self._gripper_active else None
        return append_gripper_position(
            output,
            gripper_joint,
            gripper_target,
        )

    def _prepare_target(
        self,
        state: CoordinatorState,
        q_current: np.ndarray,
        dt: float,
    ) -> pinocchio.SE3 | None:
        with self._twist_lock:
            twist = self._latest_twist
        if twist is None:
            return None
        pose = self.forward_kinematics(q_current)
        orientation_baseline: np.ndarray | None = None
        if self._config.max_orientation_delta_rad is not None:
            with self._twist_lock:
                if self._orientation_baseline is None:
                    self._orientation_baseline = pose.rotation.copy()
                orientation_baseline = self._orientation_baseline
        values = twist_to_numpy(twist)
        pose.translation = pose.translation + values[:3] * dt
        angular_step = values[3:] * dt
        if np.linalg.norm(angular_step) > 0.0:
            pose.rotation = pinocchio.exp3(angular_step) @ pose.rotation
        if self._config.workspace_min is not None:
            pose.translation = np.clip(
                pose.translation,
                np.asarray(self._config.workspace_min, dtype=np.float64),
                np.asarray(self._config.workspace_max, dtype=np.float64),
            )
        if self._config.max_reach_m is not None:
            reach = np.linalg.norm(pose.translation)
            if reach > self._config.max_reach_m:
                pose.translation *= self._config.max_reach_m / reach
        if self._config.max_orientation_delta_rad is not None:
            assert orientation_baseline is not None
            rotation_vector = pinocchio.log3(pose.rotation @ orientation_baseline.T)
            angle = np.linalg.norm(rotation_vector)
            if angle > self._config.max_orientation_delta_rad:
                pose.rotation = (
                    pinocchio.exp3(
                        rotation_vector * (self._config.max_orientation_delta_rad / angle)
                    )
                    @ orientation_baseline
                )
        if self._config.home_orientation_rpy is not None:
            home_rotation = pinocchio.rpy.rpyToMatrix(
                np.asarray(self._config.home_orientation_rpy, dtype=np.float64)
            )
            relative_rpy = pinocchio.rpy.matrixToRpy(pose.rotation @ home_rotation.T)
            bounded_rpy = np.clip(
                relative_rpy,
                -np.asarray(self._config.max_home_orientation_delta_rpy, dtype=np.float64),
                np.asarray(self._config.max_home_orientation_delta_rpy, dtype=np.float64),
            )
            pose.rotation = pinocchio.rpy.rpyToMatrix(bounded_rpy) @ home_rotation
        if not np.all(np.isfinite(pose.translation)) or not np.all(np.isfinite(pose.rotation)):
            return None
        return pose

    def stop(self) -> None:
        self._clear_inputs()
        super().stop()

    def _clear_inputs(self) -> None:
        """Discard twist and gripper commands owned by this specialization."""
        with self._twist_lock:
            self._latest_twist = None
            self._orientation_baseline = None
            self._gripper_active = False

    def _on_timeout(self) -> None:
        with self._twist_lock:
            self._latest_twist = None
            self._orientation_baseline = None

    def clear(self) -> None:
        self._clear_inputs()
        super().clear()


class EEFTwistTaskParams(CartesianIKTaskParams):
    timeout: float = 0.3
    gripper_joint: str | None = None
    gripper_open_pos: float = 0.0
    gripper_closed_pos: float = 0.0
    workspace_min: tuple[float, float, float] | None = None
    workspace_max: tuple[float, float, float] | None = None
    max_reach_m: float | None = None
    max_orientation_delta_rad: float | None = None
    home_orientation_rpy: tuple[float, float, float] | None = None
    max_home_orientation_delta_rpy: tuple[float, float, float] | None = None


def create_task(cfg: TaskConfig, hardware: object) -> EEFTwistTask:
    params = EEFTwistTaskParams.model_validate(cfg.params)
    return EEFTwistTask(
        cfg.name,
        EEFTwistTaskConfig(
            joint_names=cfg.joint_names,
            priority=cfg.priority,
            timeout=params.timeout,
            max_joint_delta_deg=params.max_joint_delta_deg,
            max_tracking_error_deg=params.max_tracking_error_deg,
            min_dt=params.min_dt,
            max_dt=params.max_dt,
            control_ik=params.control_ik,
            gripper_joint=params.gripper_joint,
            gripper_open_pos=params.gripper_open_pos,
            gripper_closed_pos=params.gripper_closed_pos,
            workspace_min=params.workspace_min,
            workspace_max=params.workspace_max,
            max_reach_m=params.max_reach_m,
            max_orientation_delta_rad=params.max_orientation_delta_rad,
            home_orientation_rpy=params.home_orientation_rpy,
            max_home_orientation_delta_rpy=params.max_home_orientation_delta_rpy,
        ),
    )
