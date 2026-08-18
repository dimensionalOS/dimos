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

"""Host contract for isolated LeRobot policy inference."""

from typing import TypedDict

from pydantic import Field, field_validator

from dimos.core.core import rpc
from dimos.core.python_native_module import PythonNativeModule, PythonNativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.service.spec import BaseConfig


class PolicyStatus(TypedDict):
    running: bool
    observations_ready: bool
    observation_error: str | None
    active_policy: str | None
    policy_path: str | None
    available_policies: list[str]
    task: str
    commands_sent: int
    last_error: str | None


class LeRobotPolicyConfig(BaseConfig):
    """Configuration for one named learned policy."""

    policy_path: str
    task: str = ""
    device: str | None = None
    default_duration: float = Field(default=10.0, gt=0)


class LeRobotPolicyModuleConfig(PythonNativeModuleConfig):
    """Configuration shared by the host contract and isolated runtime."""

    policies: dict[str, LeRobotPolicyConfig] = Field(min_length=1)
    joint_names: list[str] = Field(min_length=1)
    fps: float = Field(default=15.0, gt=0)
    robot_type: str = ""
    max_observation_age_s: float = Field(default=0.5, gt=0)

    @field_validator("policies")
    @classmethod
    def policy_names_must_not_be_empty(
        cls, policies: dict[str, LeRobotPolicyConfig]
    ) -> dict[str, LeRobotPolicyConfig]:
        if any(not name.strip() for name in policies):
            raise ValueError("policy names must not be empty")
        return policies

    @field_validator("joint_names")
    @classmethod
    def joint_names_must_be_unique(cls, joint_names: list[str]) -> list[str]:
        if len(set(joint_names)) != len(joint_names):
            raise ValueError("joint_names must not contain duplicates")
        return joint_names


class LeRobotPolicyModule(PythonNativeModule):
    """Convert live image and joint-state observations into joint targets."""

    implementation = "dimos_lerobot.runtime:LeRobotPolicyRuntime"
    config: LeRobotPolicyModuleConfig

    color_image: In[Image]
    coordinator_joint_state: In[JointState]
    joint_command: Out[JointState]

    @rpc
    def execute_learned_policy(
        self,
        policy_name: str,
        duration: float | None = None,
    ) -> str:
        """Execute a configured learned policy against live camera and robot state."""
        raise NotImplementedError

    @rpc
    def stop_learned_policy(self) -> str:
        """Stop the running learned policy and hold the last commanded pose."""
        raise NotImplementedError

    @rpc
    def policy_status(self) -> PolicyStatus:
        """Return live execution status for CLIs and monitoring."""
        raise NotImplementedError
