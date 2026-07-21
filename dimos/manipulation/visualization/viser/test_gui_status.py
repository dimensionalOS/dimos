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

from collections.abc import Callable
from types import SimpleNamespace
from typing import cast

import pytest

pytest.importorskip("viser", reason="Viser optional dependency is not installed")

from dimos.manipulation.visualization.types import TargetEvaluation
from dimos.manipulation.visualization.viser.adapter import InProcessViserAdapter
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.gui import ROBOT_DISPLAY_LABELS, ViserPanelGui
from dimos.manipulation.visualization.viser.scene import RobotDisplayMode, ViserManipulationScene
from dimos.manipulation.visualization.viser.state import FeasibilityStatus


class StatusOnlyServer:
    pass


class StatusOnlyAdapter(InProcessViserAdapter):
    def __init__(self) -> None:
        pass


class DisplayFolder:
    def __enter__(self) -> DisplayFolder:
        return self

    def __exit__(self, *args: object) -> bool:
        return False


class DisplayDropdown:
    def __init__(self, label: str, options: list[str], value: str) -> None:
        self.label = label
        self.options = options
        self.value = value
        self.update_callback: Callable[[object], None] | None = None

    def on_update(self, callback: Callable[[object], None]) -> None:
        self.update_callback = callback


class DisplayMarkdown:
    def __init__(self, value: str, visible: bool = True) -> None:
        self.value = value
        self.visible = visible


class DisplayGui:
    def __init__(self) -> None:
        self.folder_label = ""
        self.dropdown: DisplayDropdown | None = None
        self.warning: DisplayMarkdown | None = None

    def add_folder(self, label: str, **_: bool) -> DisplayFolder:
        self.folder_label = label
        return DisplayFolder()

    def add_dropdown(
        self, label: str, *, options: tuple[str, ...], initial_value: str, hint: str | None = None
    ) -> DisplayDropdown:
        self.dropdown = DisplayDropdown(label, list(options), initial_value)
        return self.dropdown

    def add_markdown(self, value: str, *, visible: bool = True) -> DisplayMarkdown:
        self.warning = DisplayMarkdown(value, visible)
        return self.warning


class DisplayScene:
    def __init__(self, mode: RobotDisplayMode | str = "visual", has_collision: bool = True) -> None:
        self._robot_display_mode = RobotDisplayMode(mode)
        self.collision_geometry_available = has_collision
        self.set_modes: list[str] = []

    @property
    def robot_display_mode(self) -> RobotDisplayMode:
        return self._robot_display_mode

    @robot_display_mode.setter
    def robot_display_mode(self, mode: RobotDisplayMode | str) -> None:
        normalized_mode = RobotDisplayMode(mode)
        self.set_modes.append(normalized_mode)
        self._robot_display_mode = normalized_mode

    def has_reference_grid(self) -> bool:
        return False


@pytest.mark.parametrize(
    ("result", "success", "collision_free", "expected"),
    [
        ({"status": "FEASIBLE"}, True, True, FeasibilityStatus.FEASIBLE),
        ({"status": "COLLISION"}, False, False, FeasibilityStatus.COLLISION),
        ({"status": "COLLISION_AT_START"}, False, False, FeasibilityStatus.COLLISION),
        ({"status": "COLLISION_AT_GOAL"}, False, False, FeasibilityStatus.COLLISION),
        ({"status": "NO_SOLUTION"}, False, False, FeasibilityStatus.IK_FAILED),
        ({"status": "SINGULARITY"}, False, False, FeasibilityStatus.IK_FAILED),
        ({"status": "JOINT_LIMITS"}, False, False, FeasibilityStatus.IK_FAILED),
        ({"status": "TIMEOUT"}, False, False, FeasibilityStatus.IK_FAILED),
        ({"status": "IK_SUCCEEDED"}, False, False, FeasibilityStatus.INVALID),
    ],
)
def test_gui_feasibility_status_uses_exact_status_mapping(
    result: TargetEvaluation,
    success: bool,
    collision_free: bool,
    expected: FeasibilityStatus,
) -> None:
    gui = ViserPanelGui(
        StatusOnlyServer(),
        StatusOnlyAdapter(),
        ViserVisualizationConfig(),
    )

    assert gui._feasibility_status(result, success, collision_free) == expected


def test_robot_display_control_is_labelled_and_initialized_from_scene() -> None:
    scene = DisplayScene("collision")
    display_gui = DisplayGui()
    panel = ViserPanelGui(
        StatusOnlyServer(),
        StatusOnlyAdapter(),
        ViserVisualizationConfig(),
        cast("ViserManipulationScene", scene),
    )

    panel._build_scene_controls(display_gui)

    assert display_gui.folder_label == "Robot display"
    assert display_gui.dropdown is not None
    assert display_gui.dropdown.label == "Robot display"
    assert display_gui.dropdown.options == list(ROBOT_DISPLAY_LABELS)
    assert display_gui.dropdown.value == "Collision"


def test_robot_display_control_applies_immediately_and_syncs_on_refresh() -> None:
    scene = DisplayScene()
    display_gui = DisplayGui()
    panel = ViserPanelGui(
        StatusOnlyServer(),
        StatusOnlyAdapter(),
        ViserVisualizationConfig(),
        cast("ViserManipulationScene", scene),
    )
    panel._build_scene_controls(display_gui)
    assert display_gui.dropdown is not None
    callback = display_gui.dropdown.update_callback
    assert callback is not None

    callback(SimpleNamespace(target=SimpleNamespace(value="Both")))

    assert scene.set_modes == ["both"]
    assert display_gui.dropdown.value == "Both"
    scene.robot_display_mode = "visual"
    panel._sync_robot_display_dropdown()
    assert display_gui.dropdown.value == "Visual"


def test_robot_display_warning_is_conditional_and_updates_with_mode() -> None:
    scene = DisplayScene("collision", has_collision=False)
    display_gui = DisplayGui()
    panel = ViserPanelGui(
        StatusOnlyServer(),
        StatusOnlyAdapter(),
        ViserVisualizationConfig(),
        cast("ViserManipulationScene", scene),
    )

    panel._build_scene_controls(display_gui)
    assert display_gui.warning is not None
    assert display_gui.warning.value == (
        "**Collision meshes unavailable.** Showing visual geometry with collision styling."
    )
    assert display_gui.warning.visible is True
    assert display_gui.dropdown is not None
    callback = display_gui.dropdown.update_callback
    assert callback is not None

    callback(SimpleNamespace(target=SimpleNamespace(value="Visual")))
    assert display_gui.warning.visible is False

    scene.collision_geometry_available = True
    callback(SimpleNamespace(target=SimpleNamespace(value="Both")))
    assert display_gui.warning.visible is False


def test_robot_display_callback_is_noop_without_scene_or_after_close() -> None:
    panel = ViserPanelGui(StatusOnlyServer(), StatusOnlyAdapter(), ViserVisualizationConfig())
    panel._set_robot_display_mode("Collision")

    scene = DisplayScene()
    panel.scene = cast("ViserManipulationScene", scene)
    panel.close()
    panel._set_robot_display_mode("Collision")

    assert scene.set_modes == []
