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

"""Measured-state end-effector twist control."""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import TYPE_CHECKING

import numpy as np
import pinocchio
from pydantic import FiniteFloat

from dimos.control.coordinator import TaskConfig
from dimos.control.task import CoordinatorState
from dimos.control.tasks.cartesian_ik_task.cartesian_ik_task import (
    CartesianIKTask,
    CartesianIKTaskConfig,
)
from dimos.control.tasks.cartesian_ik_task.pink_control_ik import PinkControlIKConfig
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import twist_to_numpy

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped

logger = setup_logger()


@dataclass
class EEFTwistTaskConfig(CartesianIKTaskConfig):
    """Configuration for measured-FK-relative EEF twist control."""


class EEFTwistTask(CartesianIKTask):
    """Cartesian task specialization whose target is prepared from a twist."""

    def __init__(self, name: str, config: EEFTwistTaskConfig) -> None:
        super().__init__(name, config)
        self._twist_lock = threading.Lock()
        self._latest_twist: TwistStamped | None = None

    def is_active(self) -> bool:
        with self._twist_lock:
            has_twist = self._latest_twist is not None
        with self._lock:
            return has_twist and self._active

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
            if np.allclose(values, 0.0):
                self._latest_twist = None
                cleared = True
            else:
                self._latest_twist = twist
                cleared = False
        if cleared:
            super().clear()
            return True
        with self._lock:
            self._last_update_time = t_now
            self._active = True
        return True

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
        values = twist_to_numpy(twist)
        pose.translation = pose.translation + values[:3] * dt
        angular_step = values[3:] * dt
        if np.linalg.norm(angular_step) > 0.0:
            pose.rotation = pinocchio.exp3(angular_step) @ pose.rotation
        if not np.all(np.isfinite(pose.translation)) or not np.all(np.isfinite(pose.rotation)):
            return None
        return pose

    def stop(self) -> None:
        with self._twist_lock:
            self._latest_twist = None
        super().stop()

    def _on_timeout(self) -> None:
        with self._twist_lock:
            self._latest_twist = None

    def clear(self) -> None:
        with self._twist_lock:
            self._latest_twist = None
        super().clear()


class EEFTwistTaskParams(BaseConfig):
    timeout: float = 0.3
    max_joint_delta_deg: float = 15.0
    min_dt: FiniteFloat = 1e-4
    max_dt: FiniteFloat = 0.05
    control_ik: PinkControlIKConfig


def create_task(cfg: TaskConfig, hardware: object) -> EEFTwistTask:
    params = EEFTwistTaskParams.model_validate(cfg.params)
    return EEFTwistTask(
        cfg.name,
        EEFTwistTaskConfig(
            joint_names=cfg.joint_names,
            priority=cfg.priority,
            timeout=params.timeout,
            max_joint_delta_deg=params.max_joint_delta_deg,
            min_dt=params.min_dt,
            max_dt=params.max_dt,
            control_ik=params.control_ik,
        ),
    )
