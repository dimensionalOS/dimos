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

from collections.abc import Mapping
from dataclasses import dataclass


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
    def from_options(cls, options: Mapping[str, object] | None) -> ViserVisualizationConfig:
        """Build Viser config from backend-specific visualization options."""
        if not options:
            return cls()
        defaults = cls()
        return cls(
            host=_str_option(options, "host", defaults.host, legacy_key="visualization_host"),
            port=_int_option(options, "port", defaults.port, legacy_key="visualization_port"),
            open_browser=_bool_option(
                options, "open_browser", defaults.open_browser, legacy_key="open_visualization"
            ),
            panel_enabled=_bool_option(
                options, "panel_enabled", defaults.panel_enabled, legacy_key="viser_panel_enabled"
            ),
            poll_hz=_float_option(options, "poll_hz", defaults.poll_hz, legacy_key="viser_poll_hz"),
            preview_duration=_float_option(
                options,
                "preview_duration",
                defaults.preview_duration,
                legacy_key="viser_preview_duration",
            ),
            preview_fps=_float_option(
                options, "preview_fps", defaults.preview_fps, legacy_key="viser_preview_fps"
            ),
            preview_debounce_seconds=_float_option(
                options,
                "preview_debounce_seconds",
                defaults.preview_debounce_seconds,
                legacy_key="viser_preview_debounce_seconds",
            ),
            preview_request_timeout=_float_option(
                options,
                "preview_request_timeout",
                defaults.preview_request_timeout,
                legacy_key="viser_preview_request_timeout",
            ),
            current_match_tolerance=_float_option(
                options,
                "current_match_tolerance",
                defaults.current_match_tolerance,
                legacy_key="viser_current_match_tolerance",
            ),
            allow_plan_execute=_bool_option(
                options,
                "allow_plan_execute",
                defaults.allow_plan_execute,
            ),
        )


def _option(
    options: Mapping[str, object], key: str, default: object, *, legacy_key: str | None = None
) -> object:
    if key in options:
        return options[key]
    if legacy_key is not None and legacy_key in options:
        return options[legacy_key]
    return default


def _str_option(
    options: Mapping[str, object], key: str, default: str, *, legacy_key: str | None = None
) -> str:
    return str(_option(options, key, default, legacy_key=legacy_key))


def _int_option(
    options: Mapping[str, object], key: str, default: int, *, legacy_key: str | None = None
) -> int:
    return int(str(_option(options, key, default, legacy_key=legacy_key)))


def _float_option(
    options: Mapping[str, object], key: str, default: float, *, legacy_key: str | None = None
) -> float:
    return float(str(_option(options, key, default, legacy_key=legacy_key)))


def _bool_option(
    options: Mapping[str, object], key: str, default: bool, *, legacy_key: str | None = None
) -> bool:
    value = _option(options, key, default, legacy_key=legacy_key)
    if isinstance(value, str):
        return value.lower() in {"1", "true", "yes", "on"}
    return bool(value)
