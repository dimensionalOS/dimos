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

from dataclasses import dataclass
from typing import Protocol


class _ViserModuleConfig(Protocol):
    visualization_host: str
    visualization_port: int
    open_visualization: bool
    viser_panel_enabled: bool
    viser_poll_hz: float
    viser_preview_duration: float
    viser_preview_fps: float
    viser_preview_debounce_seconds: float
    viser_preview_request_timeout: float
    viser_current_match_tolerance: float
    allow_plan_execute: bool


@dataclass(frozen=True)
class ViserVisualizationConfig:
    """Runtime options for the in-process Viser manipulation visualizer."""

    host: str = "127.0.0.1"
    port: int = 8095
    open_browser: bool = False
    panel_enabled: bool = True
    poll_hz: float = 5.0
    preview_duration: float = 3.0
    preview_fps: float = 30.0
    preview_debounce_seconds: float = 0.05
    preview_request_timeout: float = 5.0
    current_match_tolerance: float = 0.02
    allow_plan_execute: bool = False

    @classmethod
    def from_module_config(cls, config: _ViserModuleConfig | None) -> ViserVisualizationConfig:
        """Build Viser config from a ManipulationModuleConfig-like object."""
        if config is None:
            return cls()
        return cls(
            host=str(config.visualization_host),
            port=int(config.visualization_port),
            open_browser=bool(config.open_visualization),
            panel_enabled=bool(config.viser_panel_enabled),
            poll_hz=float(config.viser_poll_hz),
            preview_duration=float(config.viser_preview_duration),
            preview_fps=float(config.viser_preview_fps),
            preview_debounce_seconds=float(config.viser_preview_debounce_seconds),
            preview_request_timeout=float(config.viser_preview_request_timeout),
            current_match_tolerance=float(config.viser_current_match_tolerance),
            allow_plan_execute=bool(config.allow_plan_execute),
        )
