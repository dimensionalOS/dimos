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

"""Configuration for the policy node module."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import Field, model_validator

from dimos.core.module import ModuleConfig

# Command families the node may publish.
CommandMode = Literal["joint_position"]


class PolicyModuleConfig(ModuleConfig):
    """Configuration for `PolicyModule`.

    Attributes:
        backend: Registered backend name (e.g., ``"test"``, ``"lerobot"``).
        backend_config: Free-form dict passed to the backend factory.
        policy_rate: Inference rate in Hz. Inference is run independently of
            the `ControlCoordinator` tick loop.
        joint_names: Coordinator-native joint names the node publishes
            commands for. The backend's `JointPositionCommand` joint names
            MUST match this list (after the optional `joint_name_map`).
        joint_name_map: Optional mapping from backend-emitted joint names
            to coordinator-native joint names. Empty by default — backend
            joint names are used as-is.
        camera_key: Key the node uses for the camera entry in
            `PolicyObservation.images`. The single `image: In[Image]` slot
            on `PolicyModule` is published under this key in the observation
            dict the backend receives. Defaults to ``"main"``; the Piper
            policy deployment overrides this to ``"usb"`` to match
            `PiperRobotContract.cameras`.
        enabled_command_modes: Command families the node will publish.
            Backend output in any other family is rejected.
        default_task: Default task description used when no upstream value
            has been seen on the `task_description` input.
        teleop_engage_buttons: `Buttons` field names whose `True` value is
            treated as "teleop engaged on this node's joints". When any of
            these is high, command publication is suspended and
            `backend.reset()` is called on the engage edge. When non-empty,
            the `buttons` subscription is a hard requirement of `start()`
            (subscription failure aborts module startup) and publication is
            additionally gated on having received at least one `Buttons`
            message.
        observation_max_age: Max age (seconds) of the latest joint_state
            tolerated before the node skips a step. ``0.0`` disables the
            check.
        buttons_grace_period: Seconds to wait after `start()` for the first
            `Buttons` message before logging a warning each tick. Applies
            only when `teleop_engage_buttons` is non-empty. Publication
            remains suppressed until the first message arrives regardless
            of this value.
    """

    backend: str = "test"
    backend_config: dict[str, Any] = Field(default_factory=lambda: {})
    policy_rate: float = 10.0
    joint_names: list[str] = Field(default_factory=lambda: [])
    joint_name_map: dict[str, str] = Field(default_factory=lambda: {})
    camera_key: str = "main"
    enabled_command_modes: list[CommandMode] = Field(
        default_factory=lambda: ["joint_position"]  # type: ignore[arg-type]
    )
    default_task: str = ""
    teleop_engage_buttons: list[str] = Field(
        default_factory=lambda: ["left_primary", "right_primary"]
    )
    observation_max_age: float = 0.0
    buttons_grace_period: float = 2.0

    @model_validator(mode="after")
    def _validate_command_modes(self) -> PolicyModuleConfig:
        bad_modes = [m for m in self.enabled_command_modes if m not in ("joint_position",)]
        if bad_modes:
            raise ValueError(f"PolicyModuleConfig.enabled_command_modes: unsupported {bad_modes}")
        return self


__all__ = ["CommandMode", "PolicyModuleConfig"]
