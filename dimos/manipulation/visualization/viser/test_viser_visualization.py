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

from collections.abc import Callable, Iterator, Sequence
from dataclasses import dataclass
import sys
import threading
from types import ModuleType, SimpleNamespace, TracebackType

import numpy as np
import pytest

pytest.importorskip("viser", reason="Viser optional dependency is not installed")

from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.manipulation.planning.spec.models import (
    CollisionCheckResult,
    ForwardKinematicsResult,
    IKResult,
)
from dimos.manipulation.visualization.types import RobotInfo
from dimos.manipulation.visualization.viser import scene as scene_module
from dimos.manipulation.visualization.viser.animation import (
    GroupPreviewAnimation,
    PreviewAnimator,
    PreviewTrack,
    interpolate_joint_path,
    sampled_joint_path_frames,
)
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.gui import (
    ACTIVE_GROUP_COLOR,
    INACTIVE_GROUP_COLOR,
    PRIMARY_ACTION_COLOR,
    ViserPanelGui,
)
from dimos.manipulation.visualization.viser.panel_backend import pose_from_transform_values
from dimos.manipulation.visualization.viser.scene import ViserManipulationScene
from dimos.manipulation.visualization.viser.state import (
    ActionStatus,
    FeasibilityStatus,
    OperationWorker,
    PanelPlanState,
    PlanStatus,
    TargetEvaluationRequest,
    TargetEvaluationWorker,
    TargetStatus,
)
from dimos.manipulation.visualization.viser.theme import _dimos_logo_data_url, apply_dimos_theme
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState

GuiCallback = Callable[[SimpleNamespace], None]
ThemeValue = str | bool | tuple[int, int, int] | dict[str, str | dict[str, str]] | None
RobotConfigOverride = str | list[str] | list[float] | None
DEFAULT_GROUP_ID = "arm:manipulator"


@dataclass
class RobotConfigStub:
    name: str = "arm"
    joint_names: list[str] | None = None
    end_effector_link: str = "ee_link"
    base_link: str = "base_link"
    home_joints: list[float] | None = None
    joint_limits_lower: list[float] | None = None
    joint_limits_upper: list[float] | None = None

    def __post_init__(self) -> None:
        if self.joint_names is None:
            self.joint_names = ["j1", "j2"]


@dataclass
class SceneRobotConfigStub:
    name: str = "arm"
    model_path: str = "/tmp/arm.urdf"
    package_paths: dict[str, str] | None = None
    xacro_args: dict[str, str] | None = None
    auto_convert_meshes: bool = False
    joint_names: list[str] | None = None

    def __post_init__(self) -> None:
        if self.package_paths is None:
            self.package_paths = {}
        if self.xacro_args is None:
            self.xacro_args = {}
        if self.joint_names is None:
            self.joint_names = ["joint1"]


@dataclass
class NamedState:
    name: str


@dataclass
class GuiMarkdownHandle:
    value: str
    removed: bool = False

    def remove(self) -> None:
        self.removed = True


@dataclass
class GuiDropdownHandle:
    label: str
    options: list[str]
    value: str
    update_callback: GuiCallback | None = None
    removed: bool = False

    def on_update(self, callback: GuiCallback) -> None:
        self.update_callback = callback

    def remove(self) -> None:
        self.removed = True


@dataclass
class GuiButtonHandle:
    label: str
    disabled: bool = False
    color: tuple[int, int, int] | None = None
    click_callback: GuiCallback | None = None
    removed: bool = False

    def on_click(self, callback: GuiCallback) -> None:
        self.click_callback = callback

    def remove(self) -> None:
        self.removed = True


@dataclass
class GuiCheckboxHandle:
    label: str
    value: bool
    update_callback: GuiCallback | None = None
    removed: bool = False

    def on_update(self, callback: GuiCallback) -> None:
        self.update_callback = callback

    def remove(self) -> None:
        self.removed = True


@dataclass
class GuiSliderHandle:
    label: str
    min: float
    max: float
    step: float
    value: float
    removed: bool = False
    update_callback: GuiCallback | None = None

    def on_update(self, callback: GuiCallback) -> None:
        self.update_callback = callback

    def remove(self) -> None:
        self.removed = True


class FakeHandle:
    def __init__(self) -> None:
        self.visible: object | None = None
        self.removed = False
        self.name = ""
        self.kwargs: dict[str, object] = {}

    def remove(self) -> None:
        self.removed = True


class FakeUrdf:
    def __init__(self, names: tuple[str, ...]) -> None:
        self._urdf = SimpleNamespace(actuated_joint_names=names)
        self._meshes = []
        self.cfg = None
        self.removed = False

    def update_cfg(self, cfg: Sequence[float]) -> None:
        self.cfg = list(cfg)

    def remove(self) -> None:
        self.removed = True


class FakeJointState(JointState):
    def __init__(
        self,
        name: Sequence[str],
        position: Sequence[float] | None = None,
        velocity: Sequence[float] | None = None,
        effort: Sequence[float] | None = None,
    ) -> None:
        self.ts = 0.0
        self.frame_id = ""
        self.name = list(name)
        self.position = list(position or [])
        self.velocity = list(velocity or [])
        self.effort = list(effort or [])


class FakeServer:
    def __init__(self) -> None:
        self.scene = SimpleNamespace()
        self.scene.add_transform_controls = self.add_transform_controls
        self.scene.add_frame = self.add_frame
        self.frames = []

    def add_transform_controls(self, path: str, *, scale: float) -> FakeTransformHandle:
        handle = FakeTransformHandle()
        handle.path = path
        handle.scale = scale
        return handle

    def add_frame(self, name: str, **kwargs: object) -> FakeHandle:
        handle = FakeHandle()
        handle.name = name
        handle.kwargs = kwargs
        self.frames.append(handle)
        return handle


class FakeGridServer(FakeServer):
    def __init__(self) -> None:
        super().__init__()
        self.grids = []
        self.scene.add_grid = self.add_grid

    def add_grid(self, name: str, **kwargs: float | bool) -> FakeHandle:
        handle = FakeHandle()
        handle.name = name
        handle.kwargs = dict(kwargs)
        handle.visible = kwargs.get("visible")
        self.grids.append(handle)
        return handle


class FakeTransformHandle(FakeHandle):
    def __init__(self) -> None:
        super().__init__()
        self.position = (0.0, 0.0, 0.0)
        self.wxyz = (1.0, 0.0, 0.0, 0.0)
        self.color = None
        self.material_color = None
        self.update_callback = None
        self.path = ""
        self.scale = 0.0

    def on_update(self, callback: GuiCallback) -> None:
        self.update_callback = callback


class FakeTransformServer(FakeServer):
    def __init__(self) -> None:
        super().__init__()
        self.transform_controls = []
        self.scene.add_transform_controls = self.add_transform_controls

    def add_transform_controls(self, path: str, *, scale: float) -> FakeTransformHandle:
        handle = FakeTransformHandle()
        handle.path = path
        handle.scale = scale
        self.transform_controls.append(handle)
        return handle


class FakeFolder:
    def __init__(self, label: str, kwargs: dict[str, bool]) -> None:
        self.label = label
        self.kwargs = kwargs
        self.entered = False
        self.exited = False
        self.removed = False

    def __enter__(self) -> FakeFolder:
        self.entered = True
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        traceback: TracebackType | None,
    ) -> bool:
        self.exited = True
        return False

    def remove(self) -> None:
        self.removed = True


class FakeGuiServer:
    def __init__(self) -> None:
        self.theme_kwargs: dict[str, ThemeValue] | None = None
        self.folders = []
        self.gui = SimpleNamespace(
            add_markdown=lambda value: GuiMarkdownHandle(value=value),
            add_dropdown=self.add_dropdown,
            add_button=self.add_button,
            add_checkbox=self.add_checkbox,
            add_slider=self.add_slider,
            add_folder=self.add_folder,
            configure_theme=self.configure_theme,
        )
        self.buttons: dict[str, GuiButtonHandle] = {}
        self.checkboxes: dict[str, GuiCheckboxHandle] = {}
        self.sliders: list[GuiSliderHandle] = []

    def configure_theme(self, **kwargs: ThemeValue) -> None:
        self.theme_kwargs = kwargs

    def add_folder(self, label: str, **kwargs: bool) -> FakeFolder:
        handle = FakeFolder(label, kwargs)
        self.folders.append(handle)
        return handle

    def add_dropdown(
        self, label: str, *, options: Sequence[str], initial_value: str
    ) -> GuiDropdownHandle:
        handle = GuiDropdownHandle(label=label, options=list(options), value=initial_value)
        return handle

    def add_button(
        self,
        label: str,
        *,
        disabled: bool = False,
        color: tuple[int, int, int] | None = None,
        hint: str | None = None,
    ) -> GuiButtonHandle:
        _ = hint
        handle = GuiButtonHandle(label=label, disabled=disabled, color=color)
        self.buttons[label] = handle
        return handle

    def add_checkbox(self, label: str, *, initial_value: bool) -> GuiCheckboxHandle:
        handle = GuiCheckboxHandle(label=label, value=initial_value)
        self.checkboxes[label] = handle
        return handle

    def add_slider(
        self,
        label: str,
        *,
        min: float,
        max: float,
        step: float,
        initial_value: float,
    ) -> GuiSliderHandle:
        handle = GuiSliderHandle(label=label, min=min, max=max, step=step, value=initial_value)
        self.sliders.append(handle)
        return handle


def make_robot_config(**overrides: RobotConfigOverride) -> RobotConfigStub:
    """Build a faithful RobotModelConfig stand-in with the fields the panel reads."""
    config = RobotConfigStub()
    for name, value in overrides.items():
        setattr(config, name, value)
    return config


def make_planning_group_info(
    robot_name: str,
    config: RobotConfigStub | SimpleNamespace,
    *,
    group_name: str = "manipulator",
    has_pose_target: bool = True,
) -> PlanningGroup:
    joint_names = [str(name) for name in config.joint_names]
    return PlanningGroup(
        id=f"{robot_name}:{group_name}",
        group_name=group_name,
        robot_name=robot_name,
        joint_names=tuple(f"{robot_name}/{name}" for name in joint_names),
        local_joint_names=tuple(joint_names),
        base_link=str(config.base_link),
        tip_link=str(config.end_effector_link) if has_pose_target else None,
        source="fallback",
    )


class FakeManipulationModule(SimpleNamespace):
    """Public ManipulationModule surface used by the in-process Viser panel tests."""

    def list_robots(self) -> list[str]:
        return list(getattr(self, "_robots", {}).keys())

    def robot_items(self) -> list[tuple[str, str, RobotConfigStub | SimpleNamespace]]:
        return [
            (name, robot_id, config)
            for name, (robot_id, config, _) in getattr(self, "_robots", {}).items()
        ]

    def robot_id_for_name(self, robot_name: str) -> str | None:
        entry = getattr(self, "_robots", {}).get(robot_name)
        return entry[0] if entry is not None else None

    def robot_name_for_id(self, robot_id: str) -> str | None:
        for robot_name, (candidate_id, _, _) in getattr(self, "_robots", {}).items():
            if candidate_id == robot_id:
                return robot_name
        return None

    def get_robot_config(self, robot_name: str) -> RobotConfigStub | SimpleNamespace | None:
        entry = getattr(self, "_robots", {}).get(robot_name)
        return entry[1] if entry is not None else None

    def get_robot_info(self, robot_name: str) -> RobotInfo | None:
        config = self.get_robot_config(robot_name)
        if config is None:
            return None
        init = self.get_init_joints(robot_name)
        home_joints = config.home_joints if hasattr(config, "home_joints") else None
        planning_groups = getattr(self, "_planning_groups", None)
        if planning_groups is None:
            planning_groups = [make_planning_group_info(robot_name, config)]
        else:
            planning_groups = [group for group in planning_groups if group.robot_name == robot_name]
        return {
            "name": config.name,
            "world_robot_id": self.robot_id_for_name(robot_name) or robot_name,
            "joint_names": list(config.joint_names),
            "planning_groups": planning_groups,
            "end_effector_link": config.end_effector_link,
            "base_link": config.base_link,
            "max_velocity": 1.0,
            "max_acceleration": 1.0,
            "has_joint_name_mapping": False,
            "coordinator_task_name": None,
            "home_joints": list(home_joints) if home_joints is not None else None,
            "pre_grasp_offset": 0.0,
            "init_joints": list(init.position) if init is not None else None,
        }

    def list_planning_groups(self) -> list[PlanningGroup]:
        groups: list[PlanningGroup] = []
        for robot_name in self.list_robots():
            info = self.get_robot_info(robot_name)
            if info is not None:
                groups.extend(info.get("planning_groups", []))
        return groups

    def get_init_joints(self, robot_name: str) -> JointState | None:
        return getattr(self, "_init_joints", {}).get(robot_name)

    def get_state(self) -> str:
        state = getattr(self, "_state", "IDLE")
        return str(getattr(state, "name", state))

    def get_error(self) -> str:
        return str(getattr(self, "_error_message", ""))

    def get_current_joint_state(self, robot_name: str) -> JointState | None:
        robot_id = self.robot_id_for_name(robot_name)
        world_monitor = getattr(self, "_world_monitor", None)
        if robot_id is None or world_monitor is None:
            return None
        return world_monitor.get_current_joint_state(robot_id)

    def check_collision(
        self, target_joints: JointState, max_age: float = 1.0
    ) -> CollisionCheckResult:
        del max_age
        world_monitor = getattr(self, "_world_monitor", None)
        if world_monitor is not None and hasattr(world_monitor, "check_collision"):
            return world_monitor.check_collision(target_joints)
        collision_free = True
        if world_monitor is not None:
            for robot_name in self.list_robots():
                robot_id = self.robot_id_for_name(robot_name)
                if robot_id is not None:
                    collision_free = collision_free and world_monitor.is_state_valid(
                        robot_id, target_joints
                    )
        return CollisionCheckResult(
            status="VALID" if collision_free else "COLLISION",
            collision_free=collision_free,
            message="Target is collision-free" if collision_free else "Target is in collision",
        )

    def forward_kinematics(
        self,
        group_id: str,
        target_joints: JointState | None = None,
        max_age: float = 1.0,
    ) -> ForwardKinematicsResult:
        del max_age
        robot_name = group_id.split(":", 1)[0].split("/", 1)[0]
        robot_id = self.robot_id_for_name(robot_name)
        world_monitor = getattr(self, "_world_monitor", None)
        pose = None
        if world_monitor is not None and robot_id is not None:
            if hasattr(world_monitor, "get_group_ee_pose"):
                pose = world_monitor.get_group_ee_pose(group_id, target_joints)
            else:
                pose = world_monitor.get_ee_pose(robot_id, target_joints)
        return ForwardKinematicsResult(
            status="VALID", pose=pose, message="Forward kinematics solved"
        )

    def _current_values_by_name(self, robot_name: str) -> dict[str, float]:
        current = self.get_current_joint_state(robot_name)
        if current is None:
            return {}
        values: dict[str, float] = {}
        for name, position in zip(current.name, current.position, strict=False):
            name_str = str(name)
            values[name_str] = float(position)
            if "/" in name_str:
                values[name_str.rsplit("/", 1)[1]] = float(position)
            else:
                values[f"{robot_name}/{name_str}"] = float(position)
        return values

    def inverse_kinematics(
        self,
        pose_targets: dict[str, PoseStamped],
        auxiliary_group_ids: Sequence[str] = (),
        seed: JointState | None = None,
    ) -> IKResult:
        del seed
        group_ids = tuple(pose_targets) + tuple(auxiliary_group_ids)
        names: list[str] = []
        positions: list[float] = []
        for group_id in group_ids or (DEFAULT_GROUP_ID,):
            robot_name = group_id.split(":", 1)[0].split("/", 1)[0]
            config = self.get_robot_config(robot_name)
            if config is None:
                continue
            for joint_name in config.joint_names:
                names.append(f"{robot_name}/{joint_name}")
                positions.append(0.1 + 0.1 * len(positions))
        return IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState({"name": names, "position": positions}),
            message="Target is collision-free",
        )

    def plan_to_joint_targets(self, _joint_targets: dict[str, JointState]) -> bool:
        return True

    def reset(self) -> bool:
        return True


def make_module_with_robot() -> tuple[SimpleNamespace, FakeManipulationModule]:
    current = FakeJointState(["j1", "j2"], position=[0.3, 0.4])
    config = make_robot_config(
        name="arm",
        joint_names=["j1", "j2"],
        joint_limits_lower=[-1.0, -2.0],
        joint_limits_upper=[1.0, 2.0],
        home_joints=[0.0, 0.0],
    )
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda _robot_id: current,
        is_state_stale=lambda _robot_id, max_age=1.0: False,
        is_state_valid=lambda _robot_id, _joint_state: True,
        get_ee_pose=lambda _robot_id, joint_state=None: None,
    )
    module = FakeManipulationModule(
        _robots={"arm": ("robot-1", config, None)},
        _init_joints={"arm": FakeJointState(["j1", "j2"], position=[0.1, 0.2])},
        _state=NamedState(name="IDLE"),
        _error_message="",
        _world_monitor=world_monitor,
    )
    return world_monitor, module


def joints_from_values(joint_names: Sequence[str], values: Sequence[float]) -> JointState:
    return JointState({"name": list(joint_names), "position": [float(value) for value in values]})


@pytest.fixture
def make_panel() -> Iterator[Callable[..., ViserPanelGui]]:
    """Build and start a ViserPanelGui, closing it (and its worker threads) on teardown."""
    panels: list[ViserPanelGui] = []

    def _make(
        server: FakeGuiServer | FakeServer,
        module_context: tuple[SimpleNamespace, FakeManipulationModule],
        config: ViserVisualizationConfig | None = None,
        scene: ViserManipulationScene | None = None,
    ) -> ViserPanelGui:
        gui = ViserPanelGui(
            server,
            module_context[0],
            module_context[1],
            config or ViserVisualizationConfig(panel_enabled=True),
            scene,
        )
        gui.start()
        panels.append(gui)
        return gui

    yield _make
    for gui in panels:
        gui.close()


def test_viser_config_enables_panel_by_default() -> None:
    assert ViserVisualizationConfig().panel_enabled is True


def test_gui_builds_controls_in_manipulation_panel_folder(
    make_panel: Callable[..., ViserPanelGui],
) -> None:
    server = FakeGuiServer()
    module_context = make_module_with_robot()
    gui = make_panel(server, module_context, ViserVisualizationConfig())
    assert server.folders
    assert server.folders[0].label == "Manipulation Panel"
    assert server.folders[0].kwargs == {"expand_by_default": True}
    assert "status" in gui._handles
    assert "robot" not in gui._handles
    assert "planning_groups_heading" in gui._handles
    assert "target_heading" in gui._handles
    assert "target_summary" in gui._handles
    assert "actions_heading" in gui._handles
    assert "plan" in gui._handles
    assert "select_all_manipulators" not in gui._handles
    assert "clear_group_selection" not in gui._handles
    assert "plan_controls_heading" in gui._handles
    assert "actions_folder" not in gui._handles
    assert "joint_control_folder" in gui._handles
    handle_order = list(gui._handles)
    assert handle_order.index(f"group:{DEFAULT_GROUP_ID}") < handle_order.index("plan")
    assert handle_order.index("target_summary") < handle_order.index("plan")
    assert handle_order.index("plan") < handle_order.index("plan_controls_heading")
    assert handle_order.index("plan_controls_heading") < handle_order.index("preview")
    assert handle_order.index("preview") < handle_order.index("execute")
    assert handle_order.index("clear") < handle_order.index("joint_control_folder")
    assert isinstance(gui._handles["status"], GuiMarkdownHandle)
    assert "Starting" not in gui._handles["status"].value
    assert isinstance(gui._handles["target_summary"], GuiMarkdownHandle)
    assert "Feasibility:" in gui._handles["target_summary"].value
    assert "Primary:" not in gui._handles["target_summary"].value
    assert "Auxiliary:" not in gui._handles["target_summary"].value
    assert "Ghosts:" not in gui._handles["target_summary"].value
    assert isinstance(gui._handles["plan_controls_heading"], GuiMarkdownHandle)
    assert "Plan controls" in gui._handles["plan_controls_heading"].value
    plan_button = gui._handles["plan"]
    assert isinstance(plan_button, GuiButtonHandle)
    assert plan_button.color == PRIMARY_ACTION_COLOR
    group_button = gui._handles[f"group:{DEFAULT_GROUP_ID}"]
    assert isinstance(group_button, GuiButtonHandle)
    assert group_button.label == "arm"
    assert group_button.color == ACTIVE_GROUP_COLOR
    joint_folder = gui._handles["joint_control_folder"]
    assert isinstance(joint_folder, FakeFolder)
    assert joint_folder.label == "Joint Control"
    assert joint_folder.kwargs == {"expand_by_default": False}
    assert gui._operation_worker._timeout_seconds is None


def test_gui_scene_grid_checkbox_toggles_reference_grid(
    make_panel: Callable[..., ViserPanelGui],
) -> None:
    grid_server = FakeGridServer()
    scene = ViserManipulationScene(
        grid_server, lambda *args, **kwargs: FakeUrdf(("joint1",)), preview_fps=10.0
    )
    server = FakeGuiServer()
    module_context = make_module_with_robot()
    make_panel(server, module_context, ViserVisualizationConfig(), scene)
    assert grid_server.grids
    assert server.checkboxes["Scene grid"].value is True
    server.checkboxes["Scene grid"].update_callback(
        SimpleNamespace(target=SimpleNamespace(value=False))
    )
    assert grid_server.grids[0].visible is False
    server.checkboxes["Scene grid"].update_callback(
        SimpleNamespace(target=SimpleNamespace(value=True))
    )
    assert grid_server.grids[0].visible is True


def test_gui_close_removes_handles_and_late_callbacks_are_noops(
    make_panel: Callable[..., ViserPanelGui],
) -> None:
    server = FakeGuiServer()
    grid_server = FakeGridServer()
    scene = ViserManipulationScene(
        grid_server, lambda *args, **kwargs: FakeUrdf(("joint1",)), preview_fps=10.0
    )
    module_context = make_module_with_robot()
    gui = make_panel(server, module_context, ViserVisualizationConfig(), scene)
    plan_button = server.buttons["Plan"]
    grid = grid_server.grids[0]
    handles = list(gui._handles.values())

    gui.close()
    if plan_button.click_callback is not None:
        plan_button.click_callback(SimpleNamespace())
    gui._set_scene_grid_visible(False)

    assert all(getattr(handle, "removed", False) for handle in handles)
    assert gui._handles == {}
    assert grid.visible is True


def test_gui_ignores_target_evaluation_after_close(
    make_panel: Callable[..., ViserPanelGui],
) -> None:
    module_context = make_module_with_robot()
    gui = make_panel(FakeGuiServer(), module_context)
    gui.state.selected_robot = "arm"
    sequence_id = gui.state.next_sequence_id()
    request = TargetEvaluationRequest(
        sequence_id=sequence_id,
        source="joints",
        group_ids=(DEFAULT_GROUP_ID,),
        joint_targets={DEFAULT_GROUP_ID: FakeJointState(["arm/j1", "arm/j2"], position=[0.1, 0.2])},
    )
    gui.close()

    gui._apply_target_evaluation_result(
        request,
        {
            "success": True,
            "collision_free": True,
            "status": "FEASIBLE",
            "joint_state": FakeJointState(["j1", "j2"], position=[0.8, 0.9]),
        },
    )

    assert gui.state.target_status == TargetStatus.CHECKING
    assert gui.state.joint_target is None


def test_dimos_theme_configures_supported_viser_chrome() -> None:
    server = FakeGuiServer()

    assert apply_dimos_theme(server) is True
    assert server.theme_kwargs is not None
    assert server.theme_kwargs["brand_color"] == (0, 153, 255)
    assert server.theme_kwargs["dark_mode"] is True
    assert server.theme_kwargs["show_logo"] is False
    assert server.theme_kwargs["show_share_button"] is False
    assert server.theme_kwargs["control_layout"] == "fixed"
    assert server.theme_kwargs["control_width"] == "large"


def test_dimos_theme_configures_titlebar_when_supported(monkeypatch: pytest.MonkeyPatch) -> None:
    fake_viser = ModuleType("viser")
    fake_theme = ModuleType("viser.theme")
    fake_theme.TitlebarImage = lambda **kwargs: kwargs
    fake_theme.TitlebarButton = lambda **kwargs: kwargs
    fake_theme.TitlebarConfig = lambda **kwargs: kwargs
    monkeypatch.setitem(sys.modules, "viser", fake_viser)
    monkeypatch.setitem(sys.modules, "viser.theme", fake_theme)
    server = FakeGuiServer()

    assert apply_dimos_theme(server) is True
    assert server.theme_kwargs is not None
    titlebar_content = server.theme_kwargs["titlebar_content"]
    assert isinstance(titlebar_content, dict)
    image = titlebar_content["image"]
    assert isinstance(image, dict)
    assert image["image_alt"] == "Dimensional"
    assert image["image_url_light"].startswith("data:image/svg+xml;base64,")


def test_dimos_logo_asset_loads_as_data_url() -> None:
    logo_url = _dimos_logo_data_url()

    assert logo_url is not None
    assert logo_url.startswith("data:image/svg+xml;base64,")


def test_dimos_theme_is_non_blocking_when_theme_api_fails() -> None:
    class BrokenGui:
        @staticmethod
        def configure_theme(**_kwargs: ThemeValue) -> None:
            raise TypeError("theme API changed")

    server = SimpleNamespace(gui=BrokenGui())

    assert apply_dimos_theme(server) is False


def test_dimos_theme_retries_without_titlebar_when_titlebar_content_fails(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fake_viser = ModuleType("viser")
    fake_theme = ModuleType("viser.theme")
    fake_theme.TitlebarImage = lambda **kwargs: kwargs
    fake_theme.TitlebarButton = lambda **kwargs: kwargs
    fake_theme.TitlebarConfig = lambda **kwargs: kwargs
    monkeypatch.setitem(sys.modules, "viser", fake_viser)
    monkeypatch.setitem(sys.modules, "viser.theme", fake_theme)
    titlebar_values: list[ThemeValue] = []

    class FallbackGui:
        @staticmethod
        def configure_theme(**kwargs: ThemeValue) -> None:
            titlebar_values.append(kwargs["titlebar_content"])
            if kwargs["titlebar_content"] is not None:
                raise TypeError("titlebar unsupported")

    server = SimpleNamespace(gui=FallbackGui())

    assert apply_dimos_theme(server) is True
    assert titlebar_values[0] is not None
    assert titlebar_values[1] is None


class FakeMesh:
    def __init__(self) -> None:
        self.visible = None
        self.color = None
        self.material_color = None
        self.opacity = None


class FakeViserUrdfWithMeshes:
    def __init__(self, names: tuple[str, ...] = ("joint1", "joint2", "joint3")) -> None:
        self._urdf = SimpleNamespace(actuated_joint_names=names)
        self._meshes = [FakeMesh(), FakeMesh()]
        self.cfg = None

    def update_cfg(self, cfg: Sequence[float]) -> None:
        self.cfg = list(cfg)


def test_viser_joint_configuration_maps_names_to_urdf_order() -> None:
    server = FakeServer()
    urdf = FakeUrdf(("shoulder", "elbow", "wrist"))
    scene = ViserManipulationScene(server, lambda *args, **kwargs: urdf, preview_fps=10.0)
    scene.prepared_urdf_path = lambda config: "dummy.urdf"

    cfg = SimpleNamespace(
        name="arm",
        model_path="/tmp/arm.urdf",
        package_paths={},
        xacro_args={},
        auto_convert_meshes=False,
        joint_names=["arm/shoulder", "elbow"],
    )
    scene.register_robot("robot1", cfg)
    scene.set_urdf_joints(urdf, cfg.joint_names, [1.5, 2.5])
    assert urdf.cfg == [1.5, 2.5, 0.0]


def test_scene_adds_reference_grid_when_supported() -> None:
    server = FakeGridServer()
    scene = ViserManipulationScene(
        server, lambda *args, **kwargs: FakeUrdf(("j1",)), preview_fps=10.0
    )

    assert scene.has_reference_grid() is True
    assert len(server.grids) == 1
    grid = server.grids[0]
    assert grid.name == "/reference_grid"
    assert grid.kwargs["plane"] == "xy"
    assert grid.kwargs["infinite_grid"] is True
    assert grid.kwargs["visible"] is True

    scene.set_reference_grid_visible(False)
    assert grid.visible is False
    scene.set_reference_grid_visible(True)
    assert grid.visible is True


def test_preview_visibility_only_affects_preview_ghost_and_close_removes_handles() -> None:
    server = FakeServer()
    urdfs = [FakeViserUrdfWithMeshes(("joint1",)) for _ in range(3)]
    scene = ViserManipulationScene(server, lambda *args, **kwargs: urdfs.pop(0), preview_fps=10.0)
    scene.prepared_urdf_path = lambda config: "dummy.urdf"
    config = SimpleNamespace(
        name="arm",
        model_path="/tmp/arm.urdf",
        package_paths={},
        xacro_args={},
        auto_convert_meshes=False,
        joint_names=["joint1"],
    )
    scene.register_robot("robot1", config)
    target = scene._urdfs["robot1:target"]
    preview = scene._urdfs["robot1:preview"]
    assert all(mesh.visible is False for mesh in target._meshes)
    assert all(mesh.visible is False for mesh in preview._meshes)
    scene.set_target_active("robot1", True)
    assert all(mesh.visible is True for mesh in target._meshes)
    scene.show_preview("robot1")
    assert all(mesh.visible is True for mesh in preview._meshes)
    assert all(mesh.visible is True for mesh in target._meshes)
    scene.hide_preview("robot1")
    assert all(mesh.visible is False for mesh in preview._meshes)
    assert all(mesh.visible is True for mesh in target._meshes)
    scene.close()
    assert scene._handles == {}
    assert all(mesh.visible is False for mesh in preview._meshes)


def test_target_ghost_tracks_current_but_is_visible_only_when_active() -> None:
    server = FakeServer()
    urdfs = [FakeViserUrdfWithMeshes(("joint1",)) for _ in range(3)]
    scene = ViserManipulationScene(server, lambda *args, **kwargs: urdfs.pop(0), preview_fps=10.0)
    scene.prepared_urdf_path = lambda config: "dummy.urdf"
    config = SimpleNamespace(
        name="arm",
        model_path="/tmp/arm.urdf",
        package_paths={},
        xacro_args={},
        auto_convert_meshes=False,
        joint_names=["joint1"],
    )
    scene.register_robot("robot1", config)
    current = scene._urdfs["robot1:current"]
    target = scene._urdfs["robot1:target"]
    preview = scene._urdfs["robot1:preview"]

    assert all(mesh.visible is False for mesh in target._meshes)
    assert all(mesh.visible is False for mesh in preview._meshes)
    scene.update_current_robot("robot1", FakeJointState(["joint1"], position=[0.25]))
    assert current.cfg == [0.25]
    assert target.cfg == [0.25]
    assert preview.cfg is None
    assert all(mesh.visible is False for mesh in target._meshes)

    scene.set_target_active("robot1", True)
    assert all(mesh.visible is True for mesh in target._meshes)

    scene.set_target_joints("robot1", ["joint1"], [0.8])
    scene.update_current_robot("robot1", FakeJointState(["joint1"], position=[0.1]))
    assert current.cfg == [0.1]
    assert target.cfg == [0.8]
    assert preview.cfg is None

    scene.set_target_active("robot1", False)
    assert all(mesh.visible is False for mesh in target._meshes)


def test_scene_parents_urdfs_under_base_pose_frame() -> None:
    server = FakeServer()
    root_node_names: list[str] = []

    def make_urdf(*_: object, **kwargs: object) -> FakeViserUrdfWithMeshes:
        root_node_names.append(str(kwargs["root_node_name"]))
        return FakeViserUrdfWithMeshes(("joint1",))

    scene = ViserManipulationScene(server, make_urdf, preview_fps=10.0)
    scene.prepared_urdf_path = lambda config: "dummy.urdf"
    config = SimpleNamespace(
        name="arm",
        model_path="/tmp/arm.urdf",
        package_paths={},
        xacro_args={},
        auto_convert_meshes=False,
        joint_names=["joint1"],
        base_pose=PoseStamped(
            position=Vector3(1.0, 2.0, 3.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
    )

    scene.register_robot("robot1", config)

    assert [frame.name for frame in server.frames] == [
        "/robots/robot1/current/base_pose",
        "/targets/robot1/target/base_pose",
        "/previews/robot1/ghost/base_pose",
    ]
    assert [frame.kwargs["position"] for frame in server.frames] == [
        (1.0, 2.0, 3.0),
        (1.0, 2.0, 3.0),
        (1.0, 2.0, 3.0),
    ]
    assert root_node_names == [
        "/robots/robot1/current/base_pose/urdf",
        "/targets/robot1/target/base_pose/urdf",
        "/previews/robot1/ghost/base_pose/urdf",
    ]


def test_preview_animation_uses_separate_colored_ghost_and_hides_after_playback() -> None:
    server = FakeServer()
    urdfs = [FakeViserUrdfWithMeshes(("joint1",)) for _ in range(3)]
    scene = ViserManipulationScene(server, lambda *args, **kwargs: urdfs.pop(0), preview_fps=10.0)
    scene.prepared_urdf_path = lambda config: "dummy.urdf"
    config = SimpleNamespace(
        name="arm",
        model_path="/tmp/arm.urdf",
        package_paths={},
        xacro_args={},
        auto_convert_meshes=False,
        joint_names=["joint1"],
    )
    scene.register_robot("robot1", config)
    target = scene._urdfs["robot1:target"]
    preview = scene._urdfs["robot1:preview"]

    assert all(mesh.color == (255, 122, 0) for mesh in target._meshes)
    assert all(mesh.color == (80, 180, 255) for mesh in preview._meshes)
    assert all(mesh.opacity == 0.55 for mesh in preview._meshes)

    ok = scene.animate_path(
        "robot1",
        [
            FakeJointState(["joint1"], position=[0.0]),
            FakeJointState(["joint1"], position=[1.0]),
        ],
        duration=0.0,
    )

    assert ok is True
    assert preview.cfg == [1.0]
    assert all(mesh.visible is False for mesh in preview._meshes)
    assert all(mesh.visible is False for mesh in target._meshes)


def test_group_preview_animation_updates_all_tracks_on_shared_frame_clock(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    server = FakeServer()
    scene = ViserManipulationScene(
        server, lambda *args, **kwargs: FakeUrdf(("joint1",)), preview_fps=10.0
    )
    scene.prepared_urdf_path = lambda config: "dummy.urdf"
    config = SimpleNamespace(
        name="arm",
        model_path="/tmp/arm.urdf",
        package_paths={},
        xacro_args={},
        auto_convert_meshes=False,
        joint_names=["joint1"],
    )
    scene.register_robot("left", config)
    scene.register_robot("right", config)
    updates: list[tuple[str, tuple[str, ...], tuple[float, ...]]] = []
    sleep_calls: list[float] = []

    def record_preview_joints(
        robot_id: str, joint_names: Sequence[str], joints: Sequence[float]
    ) -> None:
        updates.append((robot_id, tuple(joint_names), tuple(joints)))

    monkeypatch.setattr(scene, "_set_preview_ghost_joints", record_preview_joints)
    monkeypatch.setattr(scene_module.time, "sleep", sleep_calls.append)

    ok = scene.animate_preview(
        GroupPreviewAnimation(
            group_ids=("left/arm", "right/arm"),
            tracks=(
                PreviewTrack(
                    robot_id="left",
                    group_ids=("left/arm",),
                    joint_names=("joint1",),
                    path=(
                        FakeJointState(["joint1"], position=[0.0]),
                        FakeJointState(["joint1"], position=[1.0]),
                    ),
                ),
                PreviewTrack(
                    robot_id="right",
                    group_ids=("right/arm",),
                    joint_names=("joint1",),
                    path=(
                        FakeJointState(["joint1"], position=[10.0]),
                        FakeJointState(["joint1"], position=[11.0]),
                    ),
                ),
            ),
        ),
        duration=0.0,
    )

    assert ok is True
    assert updates == [
        ("left", ("joint1",), (0.0,)),
        ("right", ("joint1",), (10.0,)),
        ("left", ("joint1",), (1.0,)),
        ("right", ("joint1",), (11.0,)),
    ]
    assert sleep_calls == [0.0]
    assert scene._preview_visible == {"left": False, "right": False}


def test_scene_target_helpers_handle_missing_robot_and_pose() -> None:
    server = FakeTransformServer()
    scene = ViserManipulationScene(
        server, lambda *args, **kwargs: FakeUrdf(("joint1",)), preview_fps=10.0
    )

    assert scene.animate_path("missing", [], duration=0.0) is False
    assert scene.set_target_joints("missing", ["joint1"], [1.0]) is False
    scene.set_target_pose("missing", Pose())
    handle = scene.ensure_target_controls("robot1", lambda _target: None)
    scene.set_target_pose("robot1", None)

    assert handle is not None
    assert handle.position == (0.0, 0.0, 0.0)


def test_scene_close_removes_grid_transform_and_urdf_handles() -> None:
    server = FakeGridServer()
    current = FakeUrdf(("joint1",))
    target = FakeUrdf(("joint1",))
    scene = ViserManipulationScene(
        server, lambda *args, **kwargs: FakeUrdf(("joint1",)), preview_fps=10.0
    )
    handle = scene.ensure_target_controls("robot1", lambda _target: None)
    scene._urdfs["robot1:current"] = current
    scene._urdfs["robot1:target"] = target

    scene.close()

    assert handle is not None and handle.removed is True
    assert current.removed is True
    assert target.removed is True
    assert server.grids[0].removed is True
    assert scene.has_reference_grid() is False


def test_sampled_joint_path_frames_preserves_dense_trajectory_samples() -> None:
    dense_path = [FakeJointState(["j1"], position=[float(index)]) for index in range(32)]

    frames = sampled_joint_path_frames(dense_path, duration=1.0, fps=30.0)

    assert frames == [[float(index)] for index in range(32)]


def test_sampled_joint_path_frames_interpolates_sparse_paths() -> None:
    sparse_path = [
        FakeJointState(["j1"], position=[0.0]),
        FakeJointState(["j1"], position=[1.0]),
    ]

    frames = sampled_joint_path_frames(sparse_path, duration=1.0, fps=4.0)

    assert frames == [[0.0], [0.25], [0.5], [0.75], [1.0]]


def test_joint_path_frame_edge_cases_and_empty_animation() -> None:
    empty_position = FakeJointState(["j1"], position=[])
    single = FakeJointState(["j1"], position=[0.7])
    start = FakeJointState(["j1"], position=[0.0])
    middle = FakeJointState(["j1"], position=[1.0])
    mismatched_final = FakeJointState(["j1", "j2"], position=[2.0, 3.0])
    set_calls: list[list[float]] = []
    sleep_calls: list[float] = []

    assert interpolate_joint_path([empty_position], duration=1.0, fps=10.0) == []
    assert interpolate_joint_path([single], duration=1.0, fps=10.0) == [[0.7]]
    assert interpolate_joint_path([start, middle, mismatched_final], duration=1.0, fps=2.0) == [
        [0.0],
        [2.0, 3.0],
    ]
    assert sampled_joint_path_frames([empty_position], duration=1.0, fps=10.0) == []
    assert (
        PreviewAnimator(set_calls.append, sleep=sleep_calls.append).animate(
            [empty_position], duration=1.0, fps=10.0
        )
        is False
    )
    assert set_calls == []
    assert sleep_calls == []


def test_scene_registers_goal_robot_coloring_and_updates_visibility() -> None:
    server = FakeServer()
    scene = ViserManipulationScene(
        server,
        lambda *args, **kwargs: FakeViserUrdfWithMeshes(("joint1", "joint2")),
        preview_fps=10.0,
    )
    scene.prepared_urdf_path = lambda config: "dummy.urdf"
    config = SimpleNamespace(
        name="arm",
        model_path="/tmp/arm.urdf",
        package_paths={},
        xacro_args={},
        auto_convert_meshes=False,
        joint_names=["joint1", "joint2"],
    )

    scene.register_robot("robot1", config)
    target = scene._urdfs["robot1:target"]
    preview = scene._urdfs["robot1:preview"]

    assert all(mesh.color == (255, 122, 0) for mesh in target._meshes)
    assert all(mesh.opacity == 0.7 for mesh in target._meshes)
    assert all(mesh.color == (80, 180, 255) for mesh in preview._meshes)
    assert all(mesh.opacity == 0.55 for mesh in preview._meshes)

    scene.show_preview("robot1")
    assert all(mesh.visible is True for mesh in preview._meshes)
    scene.hide_preview("robot1")
    assert all(mesh.visible is False for mesh in preview._meshes)
    assert all(mesh.visible is False for mesh in target._meshes)


def test_scene_transform_controls_update_pose_callback_and_visual_state() -> None:
    server = FakeTransformServer()
    scene = ViserManipulationScene(
        server,
        lambda *args, **kwargs: FakeViserUrdfWithMeshes(("joint1", "joint2")),
        preview_fps=10.0,
    )
    scene.prepared_urdf_path = lambda config: "dummy.urdf"
    config = SimpleNamespace(
        name="arm",
        model_path="/tmp/arm.urdf",
        package_paths={},
        xacro_args={},
        auto_convert_meshes=False,
        joint_names=["joint1", "joint2"],
    )
    scene.register_robot("robot1", config)
    updates = []

    control = scene.ensure_target_controls("robot1", updates.append)
    assert control is not None
    assert server.transform_controls[0].path == "/targets/robot1/ee_control"
    assert control.update_callback is not None
    moved = SimpleNamespace(position=(1.0, 2.0, 3.0), wxyz=(1.0, 0.0, 0.0, 0.0))
    control.update_callback(SimpleNamespace(target=moved))
    assert updates == [moved]

    pose = Pose({"position": [0.1, 0.2, 0.3], "orientation": [0.0, 0.0, 0.0, 1.0]})
    scene.set_target_pose("robot1", pose)
    assert control.position == (0.1, 0.2, 0.3)
    assert control.wxyz == (1.0, 0.0, 0.0, 0.0)

    scene.set_target_visual_state("robot1", feasible=False)
    target = scene._urdfs["robot1:target"]
    preview = scene._urdfs["robot1:preview"]
    assert control.color == (255, 40, 40)
    assert all(mesh.color == (255, 30, 30) for mesh in target._meshes)
    assert all(mesh.opacity == 0.75 for mesh in target._meshes)
    assert all(mesh.color == (80, 180, 255) for mesh in preview._meshes)


def test_scene_target_controls_update_target_ghost_pose_and_feasibility() -> None:
    server = FakeTransformServer()
    scene = ViserManipulationScene(
        server,
        lambda *args, **kwargs: FakeViserUrdfWithMeshes(("joint1", "joint2")),
        preview_fps=10.0,
    )
    scene.prepared_urdf_path = lambda config: "dummy.urdf"
    config = SimpleNamespace(
        name="arm",
        model_path="/tmp/arm.urdf",
        package_paths={},
        xacro_args={},
        auto_convert_meshes=False,
        joint_names=["joint1", "joint2"],
    )
    scene.register_robot("robot1", config)
    scene.ensure_target_controls("robot1", lambda target: None)

    pose = Pose({"position": [0.1, 0.2, 0.3], "orientation": [0.0, 0.0, 0.0, 1.0]})
    assert scene.set_target_joints("robot1", ["joint1", "joint2"], [0.7, 0.9]) is True
    assert scene.set_target_pose("robot1", pose) is None
    assert scene.set_target_visual_state("robot1", feasible=False) is None

    target = scene._urdfs["robot1:target"]
    handle = scene._handles["robot1:ee_control"]
    assert target.cfg == [0.7, 0.9]
    assert handle.position == (0.1, 0.2, 0.3)
    assert handle.color == (255, 40, 40)


def test_gui_initializes_pose_selector_to_current_ee_pose(
    make_panel: Callable[..., ViserPanelGui],
) -> None:
    current = FakeJointState(["j1"], position=[0.25])
    current_pose = SimpleNamespace(
        position=SimpleNamespace(x=0.1, y=0.2, z=0.3),
        orientation=SimpleNamespace(w=0.9, x=0.1, y=0.2, z=0.3),
    )
    config = make_robot_config(joint_names=["j1"], home_joints=[0.0])
    module = FakeManipulationModule(_robots={"arm": ("robot-1", config, None)})
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: current,
        is_state_stale=lambda robot_id, max_age=1.0: False,
        get_ee_pose=lambda robot_id, joint_state=None: current_pose,
    )
    module_context = (world_monitor, module)
    scene = ViserManipulationScene(
        FakeTransformServer(), lambda *args, **kwargs: FakeViserUrdfWithMeshes(), preview_fps=10.0
    )
    gui = make_panel(
        FakeGuiServer(), module_context, ViserVisualizationConfig(panel_enabled=True), scene
    )
    control = scene._handles[f"{DEFAULT_GROUP_ID}:ee_control"]
    assert control.position == (0.1, 0.2, 0.3)
    assert control.wxyz == (0.9, 0.1, 0.2, 0.3)
    assert gui.state.cartesian_target is current_pose


def test_gui_removes_pose_selector_when_group_is_deselected(
    make_panel: Callable[..., ViserPanelGui],
) -> None:
    current = FakeJointState(["j1"], position=[0.25])
    current_pose = SimpleNamespace(
        position=SimpleNamespace(x=0.1, y=0.2, z=0.3),
        orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0),
    )
    config = make_robot_config(joint_names=["j1"], home_joints=[0.0])
    module = FakeManipulationModule(_robots={"arm": ("robot-1", config, None)})
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: current,
        is_state_stale=lambda robot_id, max_age=1.0: False,
        get_ee_pose=lambda robot_id, joint_state=None: current_pose,
    )
    module_context = (world_monitor, module)
    scene = ViserManipulationScene(
        FakeTransformServer(), lambda *args, **kwargs: FakeViserUrdfWithMeshes(), preview_fps=10.0
    )
    gui = make_panel(
        FakeGuiServer(), module_context, ViserVisualizationConfig(panel_enabled=True), scene
    )
    control = scene._handles[f"{DEFAULT_GROUP_ID}:ee_control"]

    gui._set_group_selected(DEFAULT_GROUP_ID, False)

    assert f"{DEFAULT_GROUP_ID}:ee_control" not in scene._handles
    assert control.removed is True


def test_gui_group_selector_derives_primary_and_auxiliary_groups(
    make_panel: Callable[..., ViserPanelGui],
) -> None:
    current = FakeJointState(["j1", "grip"], position=[0.25, 0.5])
    config = make_robot_config(joint_names=["j1", "grip"], home_joints=[0.0, 0.0])
    pose_group = make_planning_group_info("arm", config)
    auxiliary_group = make_planning_group_info(
        "arm", config, group_name="gripper", has_pose_target=False
    )
    module = FakeManipulationModule(
        _robots={"arm": ("robot-1", config, None)},
        _planning_groups=[pose_group, auxiliary_group],
    )
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: current,
        is_state_stale=lambda robot_id, max_age=1.0: False,
        is_state_valid=lambda robot_id, joint_state: True,
        get_ee_pose=lambda robot_id, joint_state=None: Pose(
            {"position": [0.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]}
        ),
    )
    module_context = (world_monitor, module)
    target_controls = []
    scene = SimpleNamespace(
        has_reference_grid=lambda: False,
        ensure_target_controls=lambda *args: target_controls.append(args) or object(),
        remove_target_controls=lambda *args: None,
        set_target_active=lambda *args: None,
        set_target_joints=lambda *args: True,
        set_target_pose=lambda *args: None,
        set_target_visual_state=lambda *args: None,
    )
    server = FakeGuiServer()

    gui = make_panel(server, module_context, ViserVisualizationConfig(panel_enabled=True), scene)
    assert "robot" not in gui._handles
    pose_button = gui._handles["group:arm:manipulator"]
    aux_button = gui._handles["group:arm:gripper"]
    assert isinstance(pose_button, GuiButtonHandle)
    assert isinstance(aux_button, GuiButtonHandle)
    assert pose_button.label == "arm"
    assert pose_button.color == ACTIVE_GROUP_COLOR
    assert aux_button.label == "arm gripper"
    assert aux_button.color == INACTIVE_GROUP_COLOR

    assert aux_button.click_callback is not None
    aux_button.click_callback(SimpleNamespace(target=aux_button))

    assert gui.state.selected_group_ids == ("arm:manipulator", "arm:gripper")
    assert gui.state.auxiliary_group_ids == ("arm:gripper",)
    assert aux_button.label == "arm gripper"
    assert aux_button.color == ACTIVE_GROUP_COLOR
    assert [call[0] for call in target_controls] == ["arm:manipulator"]


def test_gui_target_ghost_visibility_follows_active_selected_groups(
    make_panel: Callable[..., ViserPanelGui],
) -> None:
    left_config = make_robot_config(name="left", joint_names=["j1"], home_joints=[0.0])
    right_config = make_robot_config(name="right", joint_names=["j1"], home_joints=[0.0])
    module = FakeManipulationModule(
        _robots={
            "left": ("left-id", left_config, None),
            "right": ("right-id", right_config, None),
        }
    )
    current = FakeJointState(["j1"], position=[0.0])
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: current,
        is_state_stale=lambda robot_id, max_age=1.0: False,
        is_state_valid=lambda robot_id, joint_state: True,
        get_ee_pose=lambda robot_id, joint_state=None: Pose(
            {"position": [0.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]}
        ),
    )
    module_context = (world_monitor, module)
    active_updates = []
    scene = SimpleNamespace(
        has_reference_grid=lambda: False,
        ensure_target_controls=lambda *args: object(),
        remove_target_controls=lambda *args: None,
        set_target_active=lambda *args: active_updates.append(args),
        set_target_joints=lambda *args: True,
        set_target_pose=lambda *args: None,
        set_target_visual_state=lambda *args: None,
    )

    gui = make_panel(
        FakeGuiServer(), module_context, ViserVisualizationConfig(panel_enabled=True), scene
    )

    assert active_updates[-2:] == [("left-id", True), ("right-id", False)]
    gui._set_group_selected("right:manipulator", True)
    assert active_updates[-2:] == [("left-id", True), ("right-id", True)]
    gui._set_group_selected("left:manipulator", False)
    assert active_updates[-2:] == [("left-id", False), ("right-id", True)]


def test_gui_preset_dropdown_and_controls_include_init_home_current_and_callbacks(
    make_panel: Callable[..., ViserPanelGui],
) -> None:
    current = FakeJointState(["arm/j1", "arm/j2"], position=[0.25, 0.5])
    config = make_robot_config(joint_names=["j1", "j2"], home_joints=[1.0, 2.0])
    module = FakeManipulationModule(
        _robots={"arm": ("robot-1", config, None)},
        _init_joints={"arm": FakeJointState(["j1", "j2"], position=[-1.0, -2.0])},
    )
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: current,
        is_state_stale=lambda robot_id, max_age=1.0: False,
        is_state_valid=lambda robot_id, joint_state: True,
        get_ee_pose=lambda robot_id, joint_state=None: None,
    )
    module_context = (world_monitor, module)
    gui = make_panel(FakeGuiServer(), module_context)
    assert gui._handles["preset"].options == ["Select preset...", "Init", "Current", "Home"]
    assert list(gui._joint_sliders) == ["arm/j1", "arm/j2"]
    gui._apply_preset("Home")
    assert [gui._joint_sliders[name].value for name in ("arm/j1", "arm/j2")] == [1.0, 2.0]
    gui._apply_preset("Current")
    assert [gui._joint_sliders[name].value for name in ("arm/j1", "arm/j2")] == [0.25, 0.5]
    gui._submit_execute()
    assert gui.state.error == "Panel execution disabled; set allow_plan_execute=True to enable"


def test_gui_rebuilding_joint_sliders_removes_stale_viser_handles(
    make_panel: Callable[..., ViserPanelGui],
) -> None:
    current = FakeJointState(["j1", "j2"], position=[0.0, 0.0])
    config = make_robot_config(joint_names=["j1", "j2"], home_joints=[1.0, 2.0])
    module = FakeManipulationModule(_robots={"arm": ("robot-1", config, None)})
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: current,
        is_state_stale=lambda robot_id, max_age=1.0: False,
        is_state_valid=lambda robot_id, joint_state: True,
        get_ee_pose=lambda robot_id, joint_state=None: None,
    )
    module_context = (world_monitor, module)
    server = FakeGuiServer()
    gui = make_panel(server, module_context)
    stale_sliders = list(server.sliders)
    assert [slider.value for slider in stale_sliders] == [0.0, 0.0]

    current.position = [-0.738, -0.2826151825863572]
    gui.state.target_joints = None
    gui.state.group_joint_targets.clear()
    gui._build_joint_sliders()

    assert all(slider.removed is True for slider in stale_sliders)
    assert [gui._joint_sliders[name].value for name in ("arm/j1", "arm/j2")] == [
        -0.738,
        -0.2826151825863572,
    ]


def test_gui_parses_numpy_transform_control_arrays() -> None:
    pose = pose_from_transform_values(
        np.array([1.0, 2.0, 3.0]),
        np.array([0.5, 0.1, 0.2, 0.3]),
    )

    assert list(pose.position) == [1.0, 2.0, 3.0]
    assert list(pose.orientation) == [0.1, 0.2, 0.3, 0.5]


def test_panel_execution_is_gated_by_default_and_refresh_updates_robot_controls(
    make_panel: Callable[..., ViserPanelGui],
) -> None:
    current = FakeJointState(["j1"], position=[1.2])
    config = make_robot_config(joint_names=["j1"], home_joints=[0.5])
    module = FakeManipulationModule(
        _robots={"arm": ("robot-1", config, None)},
        execute=lambda robot_name=None: False,
    )
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: current,
        is_state_stale=lambda robot_id, max_age=1.0: False,
        is_state_valid=lambda robot_id, joint_state: True,
        get_ee_pose=lambda robot_id, joint_state=None: None,
    )
    module_context = (world_monitor, module)
    gui = make_panel(FakeGuiServer(), module_context)
    gui.refresh()
    assert gui.state.selected_robot == "arm"
    assert list(gui._joint_sliders) == ["arm/j1"]
    gui._apply_preset("Home")
    assert gui._joint_sliders["arm/j1"].value == 0.5

    gui._submit_execute()
    assert "Panel execution disabled" in gui.state.error


def test_gui_moves_joint_target_immediately_and_stores_evaluated_joint_solution(
    make_panel: Callable[..., ViserPanelGui],
) -> None:
    current = FakeJointState(["j1", "j2"], position=[0.0, 0.0])
    target_pose = SimpleNamespace(position=SimpleNamespace(x=0.2, y=0.3, z=0.4))
    config = make_robot_config(joint_names=["j1", "j2"], home_joints=[0.5, 0.6])
    module = FakeManipulationModule(_robots={"arm": ("robot-1", config, None)})
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: current,
        is_state_stale=lambda robot_id, max_age=1.0: False,
        is_state_valid=lambda robot_id, joint_state: True,
        get_ee_pose=lambda robot_id, joint_state=None: target_pose,
    )
    module_context = (world_monitor, module)
    target_updates = []
    target_pose_updates = []
    scene = SimpleNamespace(
        has_reference_grid=lambda: False,
        ensure_target_controls=lambda *args: None,
        set_target_joints=lambda *args: target_updates.append(args) or True,
        set_target_pose=lambda *args: target_pose_updates.append(args),
        set_target_visual_state=lambda *args: None,
    )
    gui = make_panel(
        FakeGuiServer(), module_context, ViserVisualizationConfig(panel_enabled=True), scene
    )
    requests = []
    gui._worker.stop()
    gui._worker = SimpleNamespace(
        submit=lambda request: requests.append(request), stop=lambda: None
    )
    gui._joint_sliders["arm/j1"].value = 0.25
    gui._joint_sliders["arm/j2"].value = 0.75
    gui._submit_joint_target_evaluation()
    assert target_updates[-1] == ("robot-1", ["j1", "j2"], [0.25, 0.75])
    assert target_pose_updates[-1] == (DEFAULT_GROUP_ID, target_pose)
    assert requests[-1].source == "joints"

    stale_request = TargetEvaluationRequest(
        sequence_id=1, source="joints", group_ids=(DEFAULT_GROUP_ID,)
    )
    fresh_request = TargetEvaluationRequest(
        sequence_id=2, source="joints", group_ids=(DEFAULT_GROUP_ID,)
    )
    gui.state.latest_sequence_id = 2
    gui._apply_target_evaluation_result(
        stale_request,
        {
            "success": True,
            "collision_free": True,
            "target_joints": joints_from_values(["arm/j1", "arm/j2"], [9.0, 9.0]),
        },
    )
    assert gui.state.target_joints is not None
    assert list(gui.state.target_joints.position) == [0.25, 0.75]

    joint_bar_pose = Pose({"position": [0.4, 0.5, 0.6], "orientation": [0.0, 0.0, 0.0, 1.0]})
    gui._apply_target_evaluation_result(
        fresh_request,
        {
            "success": True,
            "collision_free": True,
            "target_joints": joints_from_values(["arm/j1", "arm/j2"], [1.0, 2.0]),
            "group_poses": {DEFAULT_GROUP_ID: joint_bar_pose},
        },
    )
    assert gui.state.target_status == TargetStatus.FEASIBLE
    assert gui.state.feasibility.status == FeasibilityStatus.FEASIBLE
    assert gui.state.target_joints is not None
    assert list(gui.state.target_joints.position) == [1.0, 2.0]
    assert [gui._joint_sliders[name].value for name in ("arm/j1", "arm/j2")] == [0.25, 0.75]
    assert target_updates[-1] == ("robot-1", ["j1", "j2"], [0.25, 0.75])
    assert target_pose_updates[-1] == (DEFAULT_GROUP_ID, joint_bar_pose)


def test_gui_cartesian_ik_result_does_not_rewrite_active_gizmo(
    make_panel: Callable[..., ViserPanelGui],
) -> None:
    current = FakeJointState(["j1", "j2"], position=[0.0, 0.0])
    config = make_robot_config(joint_names=["j1", "j2"], home_joints=[0.5, 0.6])
    module = FakeManipulationModule(_robots={"arm": ("robot-1", config, None)})
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: current,
        is_state_stale=lambda robot_id, max_age=1.0: False,
        is_state_valid=lambda robot_id, joint_state: True,
        get_ee_pose=lambda robot_id, joint_state=None: None,
    )
    module_context = (world_monitor, module)
    target_joint_updates = []
    target_pose_updates = []
    scene = SimpleNamespace(
        has_reference_grid=lambda: False,
        ensure_target_controls=lambda *args: object(),
        set_target_joints=lambda *args: target_joint_updates.append(args) or True,
        set_target_pose=lambda *args: target_pose_updates.append(args),
        set_target_visual_state=lambda *args: None,
    )
    gui = make_panel(
        FakeGuiServer(), module_context, ViserVisualizationConfig(panel_enabled=True), scene
    )
    gui._handles[f"ee_control:{DEFAULT_GROUP_ID}"] = object()
    dragged_pose = Pose({"position": [0.1, 0.2, 0.3], "orientation": [0.0, 0.0, 0.0, 1.0]})
    solved_pose = Pose({"position": [0.4, 0.5, 0.6], "orientation": [0.0, 0.0, 0.0, 1.0]})
    gui.state.cartesian_target = dragged_pose
    gui.state.pose_targets[DEFAULT_GROUP_ID] = dragged_pose
    target_pose_updates.clear()
    request = TargetEvaluationRequest(
        sequence_id=1, source="cartesian", group_ids=(DEFAULT_GROUP_ID,)
    )
    gui.state.latest_sequence_id = 1

    gui._apply_target_evaluation_result(
        request,
        {
            "success": True,
            "collision_free": True,
            "target_joints": joints_from_values(["arm/j1", "arm/j2"], [1.0, 2.0]),
            "group_poses": {DEFAULT_GROUP_ID: solved_pose},
        },
    )

    assert gui.state.target_status == TargetStatus.FEASIBLE
    assert [gui._joint_sliders[name].value for name in ("arm/j1", "arm/j2")] == [1.0, 2.0]
    assert target_joint_updates[-1] == ("robot-1", ["j1", "j2"], [1.0, 2.0])
    assert target_pose_updates == []
    assert gui.state.pose_targets[DEFAULT_GROUP_ID] is dragged_pose
    assert gui.state.group_poses[DEFAULT_GROUP_ID] is solved_pose


def test_gui_cartesian_collision_still_updates_target_ghost_red(
    make_panel: Callable[..., ViserPanelGui],
) -> None:
    current = FakeJointState(["j1", "j2"], position=[0.0, 0.0])
    config = make_robot_config(joint_names=["j1", "j2"], home_joints=[0.5, 0.6])
    module = FakeManipulationModule(_robots={"arm": ("robot-1", config, None)})
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: current,
        is_state_stale=lambda robot_id, max_age=1.0: False,
        is_state_valid=lambda robot_id, joint_state: False,
        get_ee_pose=lambda robot_id, joint_state=None: None,
    )
    module_context = (world_monitor, module)
    target_joint_updates = []
    target_pose_updates = []
    visual_states = []
    scene = SimpleNamespace(
        has_reference_grid=lambda: False,
        ensure_target_controls=lambda *args: object(),
        set_target_joints=lambda *args: target_joint_updates.append(args) or True,
        set_target_pose=lambda *args: target_pose_updates.append(args),
        set_target_visual_state=lambda *args: visual_states.append(args),
    )
    gui = make_panel(
        FakeGuiServer(), module_context, ViserVisualizationConfig(panel_enabled=True), scene
    )
    dragged_pose = Pose({"position": [0.1, 0.2, 0.3], "orientation": [0.0, 0.0, 0.0, 1.0]})
    solved_pose = Pose({"position": [0.4, 0.5, 0.6], "orientation": [0.0, 0.0, 0.0, 1.0]})
    gui.state.cartesian_target = dragged_pose
    gui.state.pose_targets[DEFAULT_GROUP_ID] = dragged_pose
    target_joint_updates.clear()
    target_pose_updates.clear()
    visual_states.clear()
    request = TargetEvaluationRequest(
        sequence_id=1, source="cartesian", group_ids=(DEFAULT_GROUP_ID,)
    )
    gui.state.latest_sequence_id = 1

    gui._apply_target_evaluation_result(
        request,
        {
            "success": False,
            "status": "COLLISION",
            "message": "Target is in collision",
            "collision_free": False,
            "target_joints": joints_from_values(["arm/j1", "arm/j2"], [1.0, 2.0]),
            "group_poses": {DEFAULT_GROUP_ID: solved_pose},
        },
    )

    assert gui.state.target_status == TargetStatus.INFEASIBLE
    assert gui.state.feasibility.status == FeasibilityStatus.COLLISION
    assert gui.state.target_joints is not None
    assert list(gui.state.target_joints.position) == [1.0, 2.0]
    assert gui.state.last_valid_target_joints is None
    assert [gui._joint_sliders[name].value for name in ("arm/j1", "arm/j2")] == [1.0, 2.0]
    assert target_joint_updates[-1] == ("robot-1", ["j1", "j2"], [1.0, 2.0])
    assert (DEFAULT_GROUP_ID, False) in visual_states
    assert ("robot-1", False) in visual_states
    assert target_pose_updates == []
    assert gui.state.pose_targets[DEFAULT_GROUP_ID] is dragged_pose
    assert gui.state.group_poses[DEFAULT_GROUP_ID] is solved_pose


def test_gui_can_disable_collision_check_for_cartesian_target_evaluation(
    make_panel: Callable[..., ViserPanelGui],
) -> None:
    current = FakeJointState(["j1", "j2"], position=[0.0, 0.0])
    config = make_robot_config(joint_names=["j1", "j2"], home_joints=[0.0, 0.0])
    module = FakeManipulationModule(_robots={"arm": ("robot-1", config, None)})
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: current,
        is_state_stale=lambda robot_id, max_age=1.0: False,
        is_state_valid=lambda robot_id, joint_state: False,
        get_ee_pose=lambda robot_id, joint_state=None: Pose(
            {"position": [0.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]}
        ),
    )
    module_context = (world_monitor, module)
    scene = SimpleNamespace(
        has_reference_grid=lambda: False,
        ensure_target_controls=lambda *args: None,
        set_target_joints=lambda *args: True,
        set_target_pose=lambda *args: None,
        set_target_visual_state=lambda *args: None,
    )
    gui = make_panel(
        FakeGuiServer(),
        module_context,
        ViserVisualizationConfig(panel_enabled=True),
        scene,
    )
    request = TargetEvaluationRequest(
        sequence_id=1,
        source="cartesian",
        group_ids=(DEFAULT_GROUP_ID,),
        pose_targets={
            DEFAULT_GROUP_ID: Pose(
                {"position": [0.1, 0.2, 0.3], "orientation": [0.0, 0.0, 0.0, 1.0]}
            )
        },
        check_collision=False,
    )

    result = gui._handle_target_evaluation_request(request)

    assert result["success"] is True
    assert result["collision_free"] is True
    assert result["message"] == "Target collision check skipped"


def test_gui_collision_evaluation_marks_target_infeasible_and_colors_scene(
    make_panel: Callable[..., ViserPanelGui],
) -> None:
    current = FakeJointState(["j1"], position=[0.0])
    config = make_robot_config(joint_names=["j1"], home_joints=[0.0])
    module = FakeManipulationModule(_robots={"arm": ("robot-1", config, None)})
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: current,
        is_state_stale=lambda robot_id, max_age=1.0: False,
        is_state_valid=lambda robot_id, joint_state: False,
        get_ee_pose=lambda robot_id, joint_state=None: SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0)
        ),
    )
    module._world_monitor = world_monitor
    module_context = (world_monitor, module)
    visual_states = []
    scene = SimpleNamespace(
        has_reference_grid=lambda: False,
        ensure_target_controls=lambda *args: None,
        set_target_joints=lambda *args: True,
        set_target_pose=lambda *args: None,
        set_target_visual_state=lambda *args: visual_states.append(args),
    )
    gui = make_panel(
        FakeGuiServer(), module_context, ViserVisualizationConfig(panel_enabled=True), scene
    )
    request = TargetEvaluationRequest(sequence_id=1, source="joints", group_ids=(DEFAULT_GROUP_ID,))
    gui.state.latest_sequence_id = 1
    result = gui._evaluate_joint_target_set(
        {DEFAULT_GROUP_ID: FakeJointState(["arm/j1"], position=[1.0])}
    )

    gui._apply_target_evaluation_result(request, result)

    assert result["status"] == "COLLISION"
    assert gui.state.target_status == TargetStatus.INFEASIBLE
    assert gui.state.feasibility.status == FeasibilityStatus.COLLISION
    assert gui.state.error == "Target is in collision"
    assert (DEFAULT_GROUP_ID, False) in visual_states
    assert ("robot-1", False) in visual_states


def test_gui_safe_execute_requires_fresh_matching_plan_and_clear_resets_path(
    make_panel: Callable[..., ViserPanelGui], monkeypatch: pytest.MonkeyPatch
) -> None:
    current = FakeJointState(["j1"], position=[1.0])
    planned = [FakeJointState(["j1"], position=[1.0]), FakeJointState(["j1"], position=[2.0])]
    executed = []
    cleared = []
    module = FakeManipulationModule(
        _robots={
            "arm": ("robot-1", make_robot_config(joint_names=["j1"], home_joints=[1.0]), None)
        },
        _state=NamedState(name="IDLE"),
        execute=lambda robot_name=None: executed.append(robot_name) or True,
        clear_planned_path=lambda: cleared.append(True) or True,
    )
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: current,
        is_state_stale=lambda robot_id, max_age=1.0: False,
        is_state_valid=lambda robot_id, joint_state: True,
        get_ee_pose=lambda robot_id, joint_state=None: SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0)
        ),
    )
    module_context = (world_monitor, module)
    gui = make_panel(
        FakeGuiServer(),
        module_context,
        ViserVisualizationConfig(
            panel_enabled=True, allow_plan_execute=True, current_match_tolerance=0.05
        ),
    )
    gui._operation_worker.stop()
    monkeypatch.setattr(
        gui,
        "_operation_worker",
        SimpleNamespace(
            submit=lambda operation, **_kwargs: operation(), stop=lambda timeout=2.0: None
        ),
    )
    gui.state.target_status = TargetStatus.FEASIBLE
    gui.state.plan_state = PanelPlanState(
        status=PlanStatus.FRESH,
        group_ids=(DEFAULT_GROUP_ID,),
        start_joints_snapshot={DEFAULT_GROUP_ID: [1.2]},
        planned_path=planned,
    )
    gui.state.target_joints = FakeJointState(["arm/j1"], position=[2.0])
    gui._submit_execute()
    assert executed == []
    assert "Cannot execute" in gui.state.error

    gui.state.action_status = ActionStatus.IDLE
    gui.state.error = ""
    gui.state.plan_state.start_joints_snapshot = {DEFAULT_GROUP_ID: [1.0]}
    gui._submit_execute()
    assert executed == [None]

    gui._submit_clear()
    assert cleared == [True]
    assert gui.state.plan_state.status == PlanStatus.NONE


def test_gui_plan_target_failure_recovers_action_state(
    make_panel: Callable[..., ViserPanelGui],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module_context = make_module_with_robot()
    gui = make_panel(FakeGuiServer(), module_context)
    gui._operation_worker.stop()
    monkeypatch.setattr(
        gui,
        "_operation_worker",
        SimpleNamespace(
            submit=lambda operation, **_kwargs: operation(), stop=lambda timeout=2.0: None
        ),
    )
    gui.state.selected_group_ids = ("missing",)
    gui.state.target_joints = JointState({"name": ["missing/j1"], "position": [1.0]})
    gui.state.target_status = TargetStatus.FEASIBLE
    gui.state.manipulation_state = "IDLE"

    gui._submit_plan()

    assert gui.state.action_status == ActionStatus.IDLE
    assert gui.state.plan_state.status == PlanStatus.FAILED
    assert gui.state.error == "Unknown planning group: missing"
    assert gui.state.last_result == "plan_to_joints=False"


def test_gui_resets_fault_before_replanning(
    make_panel: Callable[..., ViserPanelGui],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls = []
    module_context = make_module_with_robot()
    gui = make_panel(FakeGuiServer(), module_context)
    gui._operation_worker.stop()
    monkeypatch.setattr(
        gui,
        "_operation_worker",
        SimpleNamespace(
            submit=lambda operation, **_kwargs: operation(), stop=lambda timeout=2.0: None
        ),
    )

    def reset() -> bool:
        calls.append("reset")
        return True

    def plan_target_set(_joint_targets: dict[str, JointState]) -> bool:
        calls.append("plan")
        return True

    monkeypatch.setattr(module_context[1], "reset", reset)
    monkeypatch.setattr(module_context[1], "plan_to_joint_targets", plan_target_set)
    gui.state.target_status = TargetStatus.FEASIBLE
    gui.state.manipulation_state = "FAULT"

    gui._submit_plan()

    assert calls == ["reset", "plan"]
    assert gui.state.plan_state.status == PlanStatus.FRESH
    assert gui.state.last_result == "plan_to_joints=True"


def test_operation_worker_coalesces_pending_requests() -> None:
    errors = []
    calls = []
    worker = OperationWorker(errors.append)
    worker.submit(lambda: calls.append("old"))
    worker.submit(lambda: calls.append("new"))

    operation = worker._requests.get_nowait()
    operation.operation()

    assert calls == ["new"]
    assert errors == []


def test_operation_worker_stop_can_wait_for_in_flight_operation() -> None:
    errors = []
    worker = OperationWorker(errors.append)
    started = threading.Event()
    release = threading.Event()
    finished = threading.Event()
    stopped = threading.Event()

    def operation() -> None:
        started.set()
        release.wait(timeout=1.0)
        finished.set()

    worker.start()
    worker.submit(operation)
    assert started.wait(timeout=1.0)

    stopper = threading.Thread(
        target=lambda: (worker.stop(timeout=None), stopped.set()),
        name="StopViserOperationTest",
    )
    stopper.start()
    assert not stopped.wait(timeout=0.05)
    release.set()
    assert stopped.wait(timeout=1.0)
    stopper.join(timeout=1.0)

    assert finished.is_set()
    assert worker._thread is None
    assert errors == []


def test_target_evaluation_worker_coalesces_pending_requests() -> None:
    worker = TargetEvaluationWorker(lambda request: {}, lambda request, result: None)
    old_request = TargetEvaluationRequest(
        sequence_id=1, source="joints", group_ids=(DEFAULT_GROUP_ID,)
    )
    new_request = TargetEvaluationRequest(
        sequence_id=2, source="joints", group_ids=(DEFAULT_GROUP_ID,)
    )

    worker.submit(old_request)
    worker.submit(new_request)

    assert worker._requests.get_nowait() is new_request


def test_operation_worker_reports_timeout() -> None:
    errors = []
    release = threading.Event()
    finished = threading.Event()
    worker = OperationWorker(errors.append, timeout_seconds=0.01)

    def operation() -> None:
        release.wait(timeout=1.0)
        finished.set()

    worker.submit(operation, timeout_seconds=0.01)
    worker._run_operation(worker._requests.get_nowait())
    release.set()

    assert errors == ["Operation timed out after 0.0s"]
    assert finished.wait(timeout=1.0)
