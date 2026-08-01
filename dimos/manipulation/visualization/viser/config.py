# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

from typing import Literal

from pydantic import AliasChoices, BaseModel, ConfigDict, Field


class ViserVisualizationConfig(BaseModel):
    """Runtime options for the in-process Viser manipulation visualizer."""

    model_config = ConfigDict(extra="forbid", frozen=True, populate_by_name=True)

    backend: Literal["viser"] = "viser"
    host: str = Field(
        default="127.0.0.1", validation_alias=AliasChoices("host", "visualization_host")
    )
    port: int = Field(default=8095, validation_alias=AliasChoices("port", "visualization_port"))
    open_browser: bool = Field(
        default=False, validation_alias=AliasChoices("open_browser", "open_visualization")
    )
    panel_enabled: bool = Field(
        default=True, validation_alias=AliasChoices("panel_enabled", "viser_panel_enabled")
    )
    poll_hz: float = Field(default=5.0, validation_alias=AliasChoices("poll_hz", "viser_poll_hz"))
    preview_duration: float = Field(
        default=3.0, validation_alias=AliasChoices("preview_duration", "viser_preview_duration")
    )
    preview_fps: float = Field(
        default=30.0, validation_alias=AliasChoices("preview_fps", "viser_preview_fps")
    )
    preview_debounce_seconds: float = Field(
        default=0.05,
        validation_alias=AliasChoices("preview_debounce_seconds", "viser_preview_debounce_seconds"),
    )
    preview_request_timeout: float = Field(
        default=5.0,
        validation_alias=AliasChoices("preview_request_timeout", "viser_preview_request_timeout"),
    )
    target_evaluation_timeout: float = Field(
        default=5.0,
        validation_alias=AliasChoices(
            "target_evaluation_timeout", "viser_target_evaluation_timeout"
        ),
    )
    allow_plan_execute: bool = Field(
        default=False,
        validation_alias=AliasChoices("allow_plan_execute", "viser_allow_plan_execute"),
    )
    current_match_tolerance: float = Field(
        default=0.02,
        validation_alias=AliasChoices("current_match_tolerance", "viser_current_match_tolerance"),
    )

    @property
    def requires_world_visualization(self) -> bool:
        """Whether the planning world must create an embedded visualizer."""
        return False
