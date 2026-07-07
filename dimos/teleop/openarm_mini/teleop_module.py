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

"""OpenArm Mini teleop module with CLI-configurable leader settings."""

from __future__ import annotations

from dataclasses import asdict
from typing import Any

from pydantic import Field, model_validator

from dimos.teleop.openarm_mini.adapter import OpenArmMiniTeleopAdapter
from dimos.teleop.openarm_mini.config import OpenArmMiniTeleopConfig
from dimos.teleop.runtime.teleop_module import TeleopModule, TeleopModuleConfig


class OpenArmMiniTeleopModuleConfig(TeleopModuleConfig):
    """Config for OpenArm Mini leader teleoperation."""

    openarm_mini: OpenArmMiniTeleopConfig = Field(
        default_factory=lambda: OpenArmMiniTeleopConfig(enabled_sides=("left",))
    )
    openarm_mini_defaults: OpenArmMiniTeleopConfig = Field(
        default_factory=lambda: OpenArmMiniTeleopConfig(enabled_sides=("left",)),
        exclude=True,
    )

    @model_validator(mode="before")
    @classmethod
    def _merge_partial_openarm_mini_config(cls, value: object) -> object:
        if not isinstance(value, dict):
            return value

        openarm_mini = value.get("openarm_mini")
        if not isinstance(openarm_mini, dict):
            return value

        defaults = value.get("openarm_mini_defaults")
        default_config = (
            defaults
            if isinstance(defaults, OpenArmMiniTeleopConfig)
            else OpenArmMiniTeleopConfig(enabled_sides=("left",))
        )
        return {
            **value,
            "openarm_mini": {
                **asdict(default_config),
                **openarm_mini,
            },
        }


class OpenArmMiniTeleopModule(TeleopModule):
    """TeleopModule specialized for OpenArm Mini leader adapters."""

    config: OpenArmMiniTeleopModuleConfig  # type: ignore[assignment]

    def __init__(self, **kwargs: Any) -> None:
        config = OpenArmMiniTeleopModuleConfig(**kwargs)
        super().__init__(adapter=OpenArmMiniTeleopAdapter(config.openarm_mini), **kwargs)
