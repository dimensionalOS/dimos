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
from typing import TypedDict

from pydantic import Field, field_validator, model_validator

from dimos.core.core import rpc
from dimos.core.isolated_python_module import (
    IsolatedPythonModule,
    IsolatedPythonModuleConfig,
)
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32


class RolloutStatus(TypedDict):
    """Operator-facing state of the configured policy rollout."""

    active: bool
    policy_path: str
    task: str
    device: str | None
    observations_ready: bool
    commands_published: int
    last_error: str | None


class LeRobotPolicyModuleConfig(IsolatedPythonModuleConfig):
    """Configuration for one checkpoint shared with the isolated runtime."""

    policy_path: str = Field(min_length=1)
    task: str = ""
    device: str | None = None
    joint_names: list[str] = Field(min_length=1)
    gripper_joint_name: str | None = None
    fps: float = Field(default=30.0, gt=0)
    robot_type: str = ""
    max_observation_age_s: float = Field(default=0.5, gt=0)

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

    @model_validator(mode="after")
    def gripper_joint_must_be_a_policy_joint(self) -> LeRobotPolicyModuleConfig:
        if self.gripper_joint_name is not None and self.gripper_joint_name not in self.joint_names:
            raise ValueError("gripper_joint_name must be present in joint_names")
        return self


class LeRobotPolicyModule(IsolatedPythonModule):
    """Convert live image and joint-state observations into joint targets."""

    implementation = "dimos_lerobot.runtime:LeRobotPolicyRuntime"
    config: LeRobotPolicyModuleConfig

    color_image: In[Image]
    coordinator_joint_state: In[JointState]
    joint_command: Out[JointState]
    gripper_command: Out[Float32]

    @rpc
    def start_rollout(
        self,
        duration: float | None = None,
    ) -> RolloutStatus:
        """Start the configured policy until stopped, preempted, or duration expires."""
        raise NotImplementedError

    @rpc
    def stop_rollout(self) -> RolloutStatus:
        """Stop rollout publication and clear the policy action queue."""
        raise NotImplementedError

    @rpc
    def rollout_status(self) -> RolloutStatus:
        """Return the lifecycle and observation state of the configured policy."""
        raise NotImplementedError
