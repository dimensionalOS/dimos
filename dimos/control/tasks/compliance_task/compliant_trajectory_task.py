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

"""Composed compliant joint trajectory task."""

from __future__ import annotations

from typing import Any

from dimos.control.composition import ComposedControlTask
from dimos.control.task import CoordinatorState, JointCommandOutput
from dimos.control.tasks.compliance_task.compliance_task import (
    JointComplianceDiagnostics,
    JointComplianceTask,
    JointComplianceTaskConfig,
    JointComplianceTaskParams,
)
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
)
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState


class CompliantJointTrajectoryTask(ComposedControlTask):
    """Joint trajectory source followed by a joint compliance transform."""

    def __init__(
        self,
        name: str,
        trajectory: JointTrajectoryTask,
        compliance: JointComplianceTask,
    ) -> None:
        super().__init__(name=name, source=trajectory, transforms=[compliance])
        self._trajectory = trajectory
        self._compliance = compliance

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        reference = self._trajectory.compute(state)
        if reference is None:
            return None
        if not self._trajectory.is_active():
            self._compliance.reset()
            return reference
        return self._compliance.compute_from_reference(state, reference)

    def execute(self, trajectory: JointTrajectory) -> bool:
        """Start executing a trajectory and reset prior compliance offsets."""
        accepted = self._trajectory.execute(trajectory)
        if accepted:
            self._compliance.reset()
        return accepted

    def cancel(self) -> bool:
        """Cancel trajectory execution and reset compliance offsets."""
        cancelled = self._trajectory.cancel()
        if cancelled:
            self._compliance.reset()
        return cancelled

    def reset(self) -> bool:
        """Reset trajectory and compliance state."""
        reset = self._trajectory.reset()
        if reset:
            self._compliance.reset()
        return reset

    def get_state(self) -> TrajectoryState:
        """Get the underlying trajectory state."""
        return self._trajectory.get_state()

    def get_progress(self, t_now: float) -> float:
        """Get trajectory execution progress."""
        return self._trajectory.get_progress(t_now)

    def get_compliance_diagnostics(self) -> JointComplianceDiagnostics:
        """Get current compliance diagnostics."""
        return self._compliance.get_diagnostics()


def create_compliant_joint_trajectory_task(
    name: str,
    joint_names: list[str],
    priority: int = 10,
    compliance_config: JointComplianceTaskConfig | None = None,
) -> CompliantJointTrajectoryTask:
    """Create a compliant trajectory task from joint names and optional gains."""
    trajectory = JointTrajectoryTask(
        name=f"{name}.trajectory",
        config=JointTrajectoryTaskConfig(joint_names=joint_names, priority=priority),
    )
    compliance = JointComplianceTask(
        name=f"{name}.compliance",
        config=compliance_config
        or JointComplianceTaskConfig(joint_names=joint_names, priority=priority),
    )
    return CompliantJointTrajectoryTask(name=name, trajectory=trajectory, compliance=compliance)


def create_task(cfg: Any, hardware: Any) -> CompliantJointTrajectoryTask:
    params = JointComplianceTaskParams.model_validate(cfg.params)
    kwargs: dict[str, object] = {
        "joint_names": cfg.joint_names,
        "priority": cfg.priority,
        "use_effort_feedback": params.use_effort_feedback,
    }
    for key in (
        "mass",
        "damping",
        "stiffness",
        "max_offset",
        "max_offset_velocity",
        "deadband",
        "effort_gain",
    ):
        value = getattr(params, key)
        if value is not None:
            kwargs[key] = value
    return create_compliant_joint_trajectory_task(
        name=cfg.name,
        joint_names=cfg.joint_names,
        priority=cfg.priority,
        compliance_config=JointComplianceTaskConfig(**kwargs),  # type: ignore[arg-type]
    )
