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

from typing import Any


class PanelController:
    def __init__(self, panel: Any) -> None:
        self._panel = panel

    def refresh_panel_state(self) -> dict[str, Any]:
        return self._panel._refresh_panel_state_impl()

    def select_robot(self, robot_name: str) -> None:
        self._panel._select_robot_impl(robot_name)

    def apply_preset(self, preset: str) -> None:
        self._panel._apply_preset_impl(preset)

    def target_pose_changed(self, target: Any) -> None:
        self._panel._target_pose_changed_impl(target)

    def joint_slider_changed(self, joint_name: str) -> None:
        self._panel._joint_slider_changed_impl(joint_name)

    def plan(self) -> None:
        self._panel._plan_impl()

    def preview(self) -> None:
        self._panel._preview_impl()

    def execute(self) -> None:
        self._panel._execute_impl()

    def cancel(self) -> None:
        self._panel._cancel_impl()

    def clear_plan(self) -> None:
        self._panel._clear_plan_impl()
