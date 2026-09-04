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

"""Profile-driven host contract for isolated policy rollout."""

from __future__ import annotations

from pathlib import Path
from typing import ClassVar, Protocol, TypedDict

from pydantic import Field, field_validator

from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryExecutionResult,
)
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.experimental.isolated_python.module import (
    IsolatedPythonModule,
    IsolatedPythonModuleConfig,
)
from dimos.imitation.profile import ImageSource, PolicyIOProfile
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.spec.utils import Spec
from dimos.teleop.quest.quest_types import BUTTON_ALIASES, Buttons

POLICY_ROLLOUT_TASK_NAME = "policy_rollout"
POLICY_ROLLOUT_INSTANCE_NAME = "PolicyRolloutModule"


class PolicyControlSpec(Spec, Protocol):
    """Coordinator operations used by policy rollout."""

    def execute_trajectory(
        self,
        trajectory: JointTrajectory,
        task_name: str,
    ) -> TrajectoryExecutionResult: ...

    def cancel_trajectory(self, task_name: str) -> TrajectoryCancellationResult: ...

    def list_tasks(self) -> list[str]: ...


class RolloutStatus(TypedDict):
    """Operator-facing state of the configured policy rollout."""

    active: bool
    artifact: str
    backend: str | None
    task: str
    device: str | None
    policy_ready: bool
    observations_ready: bool
    chunks_accepted: int
    last_error: str | None


class PolicyRolloutConfig(IsolatedPythonModuleConfig):
    """Backend-neutral rollout and safety configuration."""

    artifact: str = Field(min_length=1)
    task: str = Field(min_length=1)
    device: str | None = None
    max_observation_age_s: float = Field(default=0.5, gt=0)
    max_execution_horizon_s: float = Field(default=0.5, gt=0)
    trajectory_task_name: str = POLICY_ROLLOUT_TASK_NAME
    rollout_button: str = "A"

    @field_validator("artifact")
    @classmethod
    def artifact_must_not_be_blank(cls, artifact: str) -> str:
        if not artifact.strip():
            raise ValueError("artifact must not be blank")
        path = Path(artifact).expanduser()
        return str(path.resolve()) if path.exists() else artifact

    @field_validator("trajectory_task_name")
    @classmethod
    def trajectory_task_name_must_not_be_blank(cls, name: str) -> str:
        if not name.strip():
            raise ValueError("trajectory_task_name must not be blank")
        return name

    @field_validator("rollout_button")
    @classmethod
    def rollout_button_must_be_digital(cls, name: str) -> str:
        if BUTTON_ALIASES.get(name, name) not in Buttons.BITS:
            raise ValueError(f"unknown Quest button {name!r}")
        return name


class _PolicyModule(IsolatedPythonModule):
    """RPC surface shared by all generated policy module declarations."""

    config: PolicyRolloutConfig
    profile: ClassVar[PolicyIOProfile]
    button_pressed: In[Buttons]
    _control: PolicyControlSpec

    @rpc
    def preflight_rollout(self) -> RolloutStatus:
        """Load and validate the policy and live inputs without moving the robot."""
        raise NotImplementedError

    @rpc
    def start_rollout(self) -> RolloutStatus:
        """Start the configured policy until explicitly stopped or it fails."""
        raise NotImplementedError

    @rpc
    def stop_rollout(self) -> RolloutStatus:
        """Stop rollout publication and clear the policy action queue."""
        raise NotImplementedError

    @rpc
    def rollout_status(self) -> RolloutStatus:
        """Return the lifecycle and observation state of the configured policy."""
        raise NotImplementedError


def declare_policy_module(
    name: str,
    module_name: str,
    profile: PolicyIOProfile,
    config_type: type[PolicyRolloutConfig],
    implementation: str,
) -> type[_PolicyModule]:
    """Declare a stable importable policy module with profile-shaped ports."""
    annotations: dict[str, object] = {"config": config_type}
    for source in profile.observations.values():
        annotations[source.stream] = (
            In[Image] if isinstance(source, ImageSource) else In[JointState]
        )

    return type(
        name,
        (_PolicyModule,),
        {
            "__annotations__": annotations,
            "__doc__": f"Policy rollout module for the {profile.name!r} profile.",
            "__module__": module_name,
            "__qualname__": name,
            "implementation": implementation,
            "profile": profile,
        },
    )
