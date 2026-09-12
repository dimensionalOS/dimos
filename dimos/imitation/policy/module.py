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

"""Host contract for isolated, configurable policy rollout."""

from __future__ import annotations

from collections.abc import Callable
from functools import cache
import json
import keyword
from pathlib import Path
from typing import Any, Literal, Protocol, TypedDict

from pydantic import Field, field_validator

from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryExecutionResult,
)
from dimos.core.coordination.blueprints import Blueprint
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
from dimos.utils.generic import classproperty

POLICY_ROLLOUT_TASK_NAME = "policy_rollout"


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

    backend: str
    active: bool
    policy_path: str
    task: str
    device: str | None
    policy_ready: bool
    observations_ready: bool
    chunks_accepted: int
    last_error: str | None


class RolloutControlSpec(Spec, Protocol):
    """The existing policy's operator controls, discoverable by attached clients."""

    def preflight_rollout(self) -> RolloutStatus: ...
    def start_rollout(self) -> RolloutStatus: ...
    def stop_rollout(self) -> RolloutStatus: ...
    def rollout_status(self) -> RolloutStatus: ...


class PolicyModuleConfig(IsolatedPythonModuleConfig):
    """Configuration for one checkpoint shared with the isolated runtime."""

    backend: Literal["lerobot", "abc"] = "lerobot"
    image_mapping: dict[str, str] = Field(
        default_factory=lambda: {"color_image": "observation.images.wrist"}, min_length=1
    )
    policy_joint_names: list[str] | None = None
    fast_inference: bool = True
    diffusion_steps: int = Field(default=10, gt=0)
    execution_steps: int | None = Field(default=None, gt=0)
    norm_stats_path: str | None = None
    clip_cache_dir: str | None = None
    policy_path: str = Field(min_length=1)
    task: str = Field(min_length=1)
    device: str | None = None
    joint_names: list[str] = Field(min_length=1)
    fps: float | None = Field(default=None, gt=0)
    robot_type: str = ""
    max_observation_age_s: float = Field(default=0.5, gt=0)
    trajectory_task_name: str = POLICY_ROLLOUT_TASK_NAME
    rollout_button: str = "A"

    @field_validator("image_mapping")
    @classmethod
    def image_mapping_must_be_valid(cls, mapping: dict[str, str]) -> dict[str, str]:
        return validate_image_mapping(mapping)

    @field_validator("norm_stats_path", "clip_cache_dir")
    @classmethod
    def norm_stats_path_is_absolute(cls, path: str | None) -> str | None:
        return str(Path(path).expanduser().resolve()) if path is not None else None

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


class PolicyModule(IsolatedPythonModule):
    """Convert live image and joint-state observations into joint targets."""

    implementation = "dimos.imitation.policy.runtime:_PolicyRuntime"
    config: PolicyModuleConfig

    coordinator_joint_state: In[JointState]
    button_pressed: In[Buttons]

    _control: PolicyControlSpec

    @classproperty
    def blueprint(self) -> Callable[..., Blueprint]:
        return policy_module

    @property
    def runtime_project(self) -> Path:
        return backend_project(self.config.backend)

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


def backend_project(backend: str) -> Path:
    """Locate the independently locked dependency project for a backend."""
    if backend not in {"abc", "lerobot"}:
        raise ValueError(f"Unknown policy backend {backend!r}")
    return Path(__file__).resolve().parent / backend / "python"


def validate_image_mapping(mapping: dict[str, str]) -> dict[str, str]:
    for port, feature in mapping.items():
        if (
            not port.isidentifier()
            or keyword.iskeyword(port)
            or port.startswith("_")
            or port in dir(PolicyModule)
            or port in {"coordinator_joint_state", "button_pressed"}
        ):
            raise ValueError(f"Invalid or reserved policy image port {port!r}")
        if not feature.strip():
            raise ValueError("Policy image feature names must not be blank")
    if not mapping or len(set(mapping.values())) != len(mapping):
        raise ValueError("Policy image features must be nonempty and unique")
    return mapping


@cache
def policy_class(ports: tuple[str, ...]) -> type[PolicyModule]:
    """Encode ports in the import name so ordinary pickle works in fresh workers."""
    validate_image_mapping({port: port for port in ports})
    suffix = json.dumps(list(ports), separators=(",", ":")).encode().hex()
    name = "PolicyModule_" + suffix
    cls = type(
        name,
        (PolicyModule,),
        {
            "__module__": __name__,
            "__annotations__": {port: In[Image] for port in ports},
            "implementation": "dimos.imitation.policy.runtime:PolicyRuntime_" + suffix,
        },
    )
    globals()[name] = cls
    return cls


def __getattr__(name: str) -> Any:
    if name.startswith("PolicyModule_"):
        ports = tuple(json.loads(bytes.fromhex(name.removeprefix("PolicyModule_")).decode()))
        return policy_class(ports)
    raise AttributeError(name)


def policy_module(
    *, image_mapping: dict[str, str] | None = None, instance_name: str = "policy", **kwargs: Any
) -> Blueprint:
    """Declare typed camera ports before blueprint autoconnection."""
    mapping = validate_image_mapping(
        image_mapping if image_mapping is not None else {"color_image": "observation.images.wrist"}
    )
    return Blueprint.create(
        policy_class(tuple(sorted(mapping))),
        image_mapping=mapping,
        instance_name=instance_name,
        **kwargs,
    )
