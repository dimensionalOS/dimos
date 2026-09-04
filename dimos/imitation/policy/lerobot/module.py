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

"""Host contract for isolated LeRobot policy rollout."""

from __future__ import annotations

from pathlib import Path
from typing import Protocol, TypedDict

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
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.spec.utils import Spec
from dimos.teleop.quest.quest_types import BUTTON_ALIASES, Buttons

POLICY_ROLLOUT_TASK_NAME = "policy_rollout"


class PolicyControlSpec(Spec, Protocol):
    """Coordinator operations used by policy rollout."""

    def execute_trajectory(
        self,
        trajectory: JointTrajectory,
        task_name: str,
    ) -> TrajectoryExecutionResult: ...

    def cancel_trajectory(self, task_name: str) -> TrajectoryCancellationResult: ...


class RolloutStatus(TypedDict):
    """Operator-facing state of the configured policy rollout."""

    active: bool
    policy_path: str
    task: str
    device: str | None
    observations_ready: bool
    chunks_accepted: int
    last_error: str | None


class LeRobotPolicyModuleConfig(IsolatedPythonModuleConfig):
    """Configuration for one checkpoint shared with the isolated runtime."""

    policy_path: str = Field(min_length=1)
    task: str = ""
    device: str | None = None
    joint_names: list[str] = Field(min_length=1)
    fps: float = Field(default=30.0, gt=0)
    robot_type: str = ""
    max_observation_age_s: float = Field(default=0.5, gt=0)
    trajectory_task_name: str = POLICY_ROLLOUT_TASK_NAME
    rollout_button: str = "A"

    @field_validator("policy_path")
    @classmethod
    def policy_path_must_not_be_blank(cls, policy_path: str) -> str:
        if not policy_path.strip():
            raise ValueError("policy_path must not be blank")
        path = Path(policy_path).expanduser()
        return str(path.resolve()) if path.exists() else policy_path

    @field_validator("joint_names")
    @classmethod
    def joint_names_must_be_unique(cls, joint_names: list[str]) -> list[str]:
        if len(set(joint_names)) != len(joint_names):
            raise ValueError("joint_names must not contain duplicates")
        return joint_names

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


class LeRobotPolicyModule(IsolatedPythonModule):
    """Convert live image and joint-state observations into joint targets."""

    implementation = "dimos_lerobot.runtime:LeRobotPolicyRuntime"
    config: LeRobotPolicyModuleConfig

    color_image: In[Image]
    coordinator_joint_state: In[JointState]
    button_pressed: In[Buttons]

    _control: PolicyControlSpec

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
