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

from collections.abc import Callable, Sequence
from typing import Any

from dimos.manipulation.viser_panel.state import ActionStatus, FeasibilityStatus, PanelSession


class PanelGui:
    def __init__(
        self,
        server: Any | None,
        session: PanelSession,
        handles: dict[str, Any],
        joint_sliders: dict[str, Any],
    ) -> None:
        self.server = server
        self.session = session
        self.handles = handles
        self.joint_sliders = joint_sliders

    def build(
        self,
        *,
        on_robot: Callable[[str], None],
        on_preset: Callable[[str], None],
        on_plan: Callable[[], None],
        on_preview: Callable[[], None],
        on_execute: Callable[[], None],
        on_cancel: Callable[[], None],
        on_clear_plan: Callable[[], None],
    ) -> None:
        if self.server is None:
            return
        gui = self.server.gui
        self.handles["status"] = gui.add_text("Status", initial_value="Disconnected")
        self.handles["error"] = gui.add_text("Error", initial_value="")
        self.handles["feasibility"] = gui.add_text("Feasibility", initial_value="unknown")
        self.handles["robot"] = gui.add_dropdown(
            "Robot", options=["No robots"], initial_value="No robots"
        )
        self.handles["preset"] = gui.add_dropdown(
            "Target Preset",
            options=["Select preset...", "Current"],
            initial_value="Select preset...",
        )
        self.handles["plan"] = gui.add_button("Plan", disabled=True)
        self.handles["preview"] = gui.add_button("Preview", disabled=True)
        self.handles["execute"] = gui.add_button("Execute", disabled=True)
        self.handles["cancel"] = gui.add_button("Cancel")
        self.handles["clear_plan"] = gui.add_button("Clear Plan")
        self.handles["robot"].on_update(lambda event: on_robot(str(event.target.value)))
        self.handles["preset"].on_update(lambda event: on_preset(str(event.target.value)))
        self.handles["plan"].on_click(lambda _: on_plan())
        self.handles["preview"].on_click(lambda _: on_preview())
        self.handles["execute"].on_click(lambda _: on_execute())
        self.handles["cancel"].on_click(lambda _: on_cancel())
        self.handles["clear_plan"].on_click(lambda _: on_clear_plan())

    def update_robot_dropdown(self, robots: Sequence[str]) -> None:
        handle = self.handles.get("robot")
        if handle is None:
            return
        options = list(robots) or ["No robots"]
        handle.options = options
        handle.value = self.session.selected_robot or options[0]

    def build_joint_sliders(
        self, robot_name: str, info: dict[str, Any], on_change: Callable[[str], None]
    ) -> None:
        del robot_name
        if self.server is None:
            return
        names = list(info.get("joint_names") or [])
        limits = info.get("joint_limits") or []
        values = self.session.current_joints or [0.0] * len(names)
        for index, name in enumerate(names):
            lower, upper = (-3.14, 3.14)
            if index < len(limits) and limits[index] is not None:
                lower, upper = limits[index]
            initial = values[index] if index < len(values) else 0.0
            slider = self.server.gui.add_slider(
                name,
                min=float(lower),
                max=float(upper),
                step=0.001,
                initial_value=float(initial),
            )
            slider.on_update(lambda _event, joint_name=name: on_change(joint_name))
            self.joint_sliders[name] = slider

    def update_preset_options(self, info: dict[str, Any]) -> None:
        preset = self.handles.get("preset")
        if preset is None:
            return
        options = ["Select preset...", "Current"]
        if info.get("init_joints") is not None:
            options.append("Init")
        if info.get("home_joints") is not None:
            options.append("Home")
        preset.options = options

    def sync_controls_from_targets(
        self,
        *,
        set_ghost_joints: Callable[[Sequence[float]], object],
    ) -> None:
        if self.session.joint_target is not None:
            self.session.sync_source = "cartesian"
            try:
                info = self.session.robot_info or {}
                for name, value in zip(
                    info.get("joint_names") or [], self.session.joint_target, strict=False
                ):
                    if name in self.joint_sliders:
                        self.joint_sliders[name].value = float(value)
                if self.session.action_status != ActionStatus.PREVIEWING:
                    set_ghost_joints(self.session.joint_target)
            finally:
                self.session.sync_source = None
        pose = self.session.cartesian_target
        ee_control = self.handles.get("ee_control")
        if pose is not None and ee_control is not None:
            self.session.sync_source = "joints"
            try:
                ee_control.position = (pose.position.x, pose.position.y, pose.position.z)
                ee_control.wxyz = (
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                )
            finally:
                self.session.sync_source = None

    def update_gui_state(
        self,
        *,
        can_execute: bool,
        set_target_visual_state: Callable[[bool], None],
    ) -> None:
        if not self.handles:
            return
        self.handles["status"].value = self.session.module_state
        self.handles["error"].value = self.session.error
        feasibility = self.session.feasibility.status.value
        if self.session.feasibility.message:
            feasibility = f"{feasibility}: {self.session.feasibility.message}"
        self.handles["feasibility"].value = feasibility
        set_target_visual_state(self.session.feasibility.status == FeasibilityStatus.FEASIBLE)
        self.handles["plan"].disabled = not self.session.can_plan()
        self.handles["preview"].disabled = not self.session.can_preview()
        self.handles["execute"].disabled = not can_execute
