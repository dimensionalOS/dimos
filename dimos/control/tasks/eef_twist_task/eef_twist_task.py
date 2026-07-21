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

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import threading
from typing import TYPE_CHECKING, Any

import numpy as np
from numpy.typing import NDArray
import pinocchio

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
    get_worst_joint_delta,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import twist_to_numpy

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped

logger = setup_logger()

_MAX_DT = 0.05


@dataclass
class EEFTwistTaskConfig:
    joint_names: list[str]
    model_path: str | Path
    ee_joint_id: int
    timeout: float
    max_joint_delta_deg: float
    priority: int = 10


class EEFTwistTask(BaseControlTask):
    """Spatial EEF twist task using twist-integrated pose IK."""

    def __init__(self, name: str, config: EEFTwistTaskConfig) -> None:
        if not config.joint_names:
            raise ValueError(f"EEFTwistTask '{name}' requires at least one joint")
        if not config.model_path:
            raise ValueError(f"EEFTwistTask '{name}' requires model_path for IK solver")
        self._name = name
        self._config = config
        self._joint_names = frozenset(config.joint_names)
        self._joint_names_list = list(config.joint_names)
        self._ik = PinocchioIK.from_model_path(config.model_path, config.ee_joint_id)
        if self._ik.nq != len(config.joint_names):
            raise ValueError(
                f"EEFTwistTask {name}: model DOF ({self._ik.nq}) != "
                f"joint_names count ({len(config.joint_names)})"
            )
        self._lock = threading.Lock()
        self._latest_twist: TwistStamped | None = None
        self._last_update_time = 0.0

    def claim(self) -> ResourceClaim:
        return ResourceClaim(self._joint_names, self._config.priority, ControlMode.SERVO_POSITION)

    def is_active(self) -> bool:
        with self._lock:
            return self._latest_twist is not None

    def on_ee_twist_command(self, twist: TwistStamped, t_now: float) -> bool:
        values = twist_to_numpy(twist)
        if not np.all(np.isfinite(values)):
            logger.warning("EEFTwistTask rejecting non-finite twist", task=self._name)
            return False
        with self._lock:
            if np.allclose(values, 0.0):
                self._clear_locked()
                self._last_update_time = t_now
                return True
            self._latest_twist = twist
            self._last_update_time = t_now
        return True

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        with self._lock:
            twist = self._latest_twist
            if twist is None:
                return None
            if (
                self._config.timeout > 0
                and state.t_now - self._last_update_time > self._config.timeout
            ):
                self._clear_locked()
                return None

        q_current = self._get_current_joints(state)
        if q_current is None or not np.all(np.isfinite(q_current)):
            return None
        target_pose = self._ik.forward_kinematics(q_current)
        dt = min(max(state.dt, 0.0), _MAX_DT)
        candidate = self._integrate_twist(target_pose, twist, dt)

        q_solution, converged, final_error = self._ik.solve(candidate, q_current)
        if not np.all(np.isfinite(q_solution)):
            return None
        if not converged:
            logger.debug(
                "EEFTwistTask IK did not converge, using partial solution",
                task=self._name,
                error=final_error,
            )
        if not check_joint_delta(q_solution, q_current, self._config.max_joint_delta_deg):
            worst_idx, worst_deg = get_worst_joint_delta(q_solution, q_current)
            logger.warning(
                "EEFTwistTask rejecting solution: joint delta exceeds limit",
                task=self._name,
                joint=self._joint_names_list[worst_idx],
                delta_deg=worst_deg,
                max_delta_deg=self._config.max_joint_delta_deg,
            )
            return None

        return JointCommandOutput(
            joint_names=self._joint_names_list,
            positions=q_solution.flatten().tolist(),
            mode=ControlMode.SERVO_POSITION,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        if joints & self._joint_names:
            logger.warning(
                "EEFTwistTask preempted", task=self._name, by_task=by_task, joints=joints
            )

    def _get_current_joints(self, state: CoordinatorState) -> NDArray[np.floating[Any]] | None:
        positions = []
        for joint_name in self._joint_names_list:
            pos = state.joints.get_position(joint_name)
            if pos is None:
                return None
            positions.append(pos)
        return np.array(positions, dtype=np.float64)

    def _clear_locked(self) -> None:
        self._latest_twist = None

    def _integrate_twist(
        self, pose: pinocchio.SE3, twist: TwistStamped, dt: float
    ) -> pinocchio.SE3:
        candidate = pose.copy()
        values = twist_to_numpy(twist)
        candidate.translation = candidate.translation + values[:3] * dt
        angular_step = values[3:] * dt
        if np.linalg.norm(angular_step) > 0.0:
            candidate.rotation = pinocchio.exp3(angular_step) @ candidate.rotation
        return candidate


class EEFTwistTaskParams(BaseConfig):
    model_path: str | Path
    ee_joint_id: int = 6
    timeout: float = 0.3
    max_joint_delta_deg: float = 15.0


def create_task(cfg: Any, hardware: Any) -> EEFTwistTask:
    params = EEFTwistTaskParams.model_validate(cfg.params)
    return EEFTwistTask(
        cfg.name,
        EEFTwistTaskConfig(
            joint_names=cfg.joint_names,
            model_path=params.model_path,
            ee_joint_id=params.ee_joint_id,
            priority=cfg.priority,
            timeout=params.timeout,
            max_joint_delta_deg=params.max_joint_delta_deg,
        ),
    )
