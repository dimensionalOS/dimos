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

"""Teleop adapter backend configuration and factory helpers."""

from __future__ import annotations

from typing import Annotated

from pydantic import Field

from dimos.teleop.openarm_mini.config import OpenArmMiniTeleopConfig
from dimos.teleop.runtime.types import TeleopAdapter

TeleopAdapterConfig = Annotated[
    OpenArmMiniTeleopConfig,
    Field(discriminator="backend"),
]


def create_teleop_adapter(config: TeleopAdapterConfig) -> TeleopAdapter:
    """Create a teleop adapter from its backend config."""
    if isinstance(config, OpenArmMiniTeleopConfig):
        from dimos.teleop.openarm_mini.adapter import OpenArmMiniTeleopAdapter

        return OpenArmMiniTeleopAdapter(config)

    raise AssertionError(f"Unhandled teleop adapter config: {config!r}")
