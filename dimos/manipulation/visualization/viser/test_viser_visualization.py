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

"""Hermetic contract tests for the group-aware Viser manipulation panel."""

from __future__ import annotations

from collections.abc import Callable, Iterator, Sequence
from dataclasses import dataclass
import threading
from types import SimpleNamespace

import pytest

pytest.importorskip("viser", reason="Viser optional dependency is not installed")

from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.visualization.viser.animation import (
    GroupPreviewAnimation,
    PreviewTrack,
    interpolate_joint_path,
    sampled_joint_path_frames,
)
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.gui import (
    ACTIVE_GROUP_COLOR,
    INACTIVE_GROUP_COLOR,
    ViserPanelGui,
)
from dimos.manipulation.visualization.viser.panel_backend import (
    complete_states_for_targets,
    evaluate_joint_target_set,
    evaluate_pose_target_set,
    group_display_name,
)
from dimos.manipulation.visualization.viser.scene import (
    GOAL_ROBOT_FEASIBLE_COLOR,
    GOAL_ROBOT_INFEASIBLE_COLOR,
    TARGET_CONTROL_FEASIBLE_COLOR,
    TARGET_CONTROL_INFEASIBLE_COLOR,
    ViserManipulationScene,
)
from dimos.manipulation.visualization.viser.state import (
    PanelPlanState,
    PlanStatus,
    TargetEvaluationRequest,
    TargetStatus,
)
from dimos.manipulation.visualization.viser.theme import apply_dimos_theme
from dimos.manipulation.visualization.viser.visualizer import ViserManipulationVisualizer
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.sensor_msgs.JointState import JointState


@dataclass
class Handle:
    label: str = ""
    value: object = None
    options: list[str] | None = None
    disabled: bool = False
    color: tuple[int, int, int] | None = None
    min: float = 0.0
    max: float = 0.0
    step: float = 0.0
    visible: bool = True
    callback: Callable[[object], None] | None = None
    removed: bool = False

    def on_update(self, callback: Callable[[object], None]) -> None:
        self.callback = callback

    def on_click(self, callback: Callable[[object], None]) -> None:
        self.callback = callback

    def remove(self) -> None:
        self.removed = True


class Folder(Handle):
    def __init__(self, label: str, **kwargs: bool) -> None:
        super().__init__(label=label)
        self.kwargs = kwargs

    def __enter__(self) -> Folder:
        return self

    def __exit__(self, *_: object) -> bool:
        return False


class Gui:
    def __init__(self) -> None:
        self.folders: list[Folder] = []
        self.buttons: list[Handle] = []
        self.dropdowns: list[Handle] = []
        self.sliders: list[Handle] = []
        self.markdown: list[Handle] = []
        self.theme_kwargs: dict[str, object] | None = None

    def add_folder(self, label: str, **kwargs: bool) -> Folder:
        folder = Folder(label, **kwargs)
        self.folders.append(folder)
        return folder

    def add_markdown(self, value: str) -> Handle:
        handle = Handle(value=value)
        self.markdown.append(handle)
        return handle

    def add_button(self, label: str, **kwargs: object) -> Handle:
        color = kwargs.get("color")
        handle = Handle(
            label=label,
            disabled=bool(kwargs.get("disabled", False)),
            color=color if isinstance(color, tuple) else None,
        )
        self.buttons.append(handle)
        return handle

    def add_dropdown(self, label: str, *, options: Sequence[str], initial_value: str) -> Handle:
        handle = Handle(label=label, options=list(options), value=initial_value)
        self.dropdowns.append(handle)
        return handle

    def add_checkbox(self, label: str, *, initial_value: bool) -> Handle:
        return Handle(label=label, value=initial_value)

    def add_slider(self, label: str, **kwargs: float) -> Handle:
        handle = Handle(label=label, value=kwargs["initial_value"])
        handle.min, handle.max, handle.step = kwargs["min"], kwargs["max"], kwargs["step"]
        self.sliders.append(handle)
        return handle

    def configure_theme(self, **kwargs: object) -> None:
        self.theme_kwargs = kwargs


class Server:
    def __init__(self) -> None:
        self.gui = Gui()
        self.scene = SimpleNamespace()


@dataclass
class Config:
    name: str
    joint_names: list[str]
    joint_limits_lower: list[float]
    joint_limits_upper: list[float]
    home_joints: list[float] | None
    base_link: str = "base"
    end_effector_link: str = "tool"
    model_path: str = "robot.urdf"
    package_paths: dict[str, str] | None = None
    xacro_args: dict[str, str] | None = None
    auto_convert_meshes: bool = False


def group(robot: str, name: str, joints: tuple[str, ...], *, pose: bool = False) -> PlanningGroup:
    return PlanningGroup(
        f"{robot}/{name}",
        robot,
        name,
        tuple(f"{robot}/{joint}" for joint in joints),
        joints,
        "base",
        "tool" if pose else None,
    )


class Module:
    def __init__(self, groups: list[PlanningGroup], states: dict[str, JointState]) -> None:
        self.groups = groups
        self.states = states
        robots = {item.robot_name for item in groups}
        self.configs = {
            robot_name: Config(robot_name, ["j1", "j2"], [-1.0, -2.0], [1.0, 2.0], [0.0, 0.0])
            for robot_name in robots
        }
        self.plans: list[tuple[tuple[str, ...], dict[str, JointState]]] = []
        self.executions = 0
        self.execution_receipts: list[str] = []
        self.cancelled = 0
        self.cleared = 0

    def list_robots(self) -> list[str]:
        return list(self.configs)

    def list_planning_groups(self) -> list[PlanningGroup]:
        return self.groups

    def robot_items(self) -> list[tuple[str, str, Config]]:
        return [(name, f"id-{name}", config) for name, config in self.configs.items()]

    def robot_id_for_name(self, name: str) -> str:
        return f"id-{name}"

    def get_robot_config(self, name: str) -> Config:
        return self.configs[name]

    def get_robot_info(self, name: str) -> dict[str, object]:
        config = self.configs[name]
        return {
            "name": name,
            "world_robot_id": self.robot_id_for_name(name),
            "joint_names": config.joint_names,
            "end_effector_link": "tool",
            "base_link": "base",
            "max_velocity": 1.0,
            "max_acceleration": 1.0,
            "has_joint_name_mapping": False,
            "coordinator_task_name": None,
            "home_joints": config.home_joints,
            "pre_grasp_offset": 0.0,
            "init_joints": [0.0, 0.0],
        }

    def get_init_joints(self, name: str) -> JointState:
        return JointState({"name": self.configs[name].joint_names, "position": [-0.5, -1.0]})

    def get_state(self) -> str:
        return "IDLE"

    def get_error(self) -> str:
        return ""

    def reset(self) -> SimpleNamespace:
        return SimpleNamespace(is_success=lambda: True)

    def plan_to_joint_targets_with_receipt(self, targets: dict[str, JointState]) -> str:
        self.plans.append((tuple(targets), targets))
        return f"receipt-{len(self.plans)}"

    def preview_plan(self) -> bool:
        return True

    def execute_plan_receipt(self, receipt: str) -> bool:
        self.executions += 1
        self.execution_receipts.append(receipt)
        return True

    def cancel(self) -> bool:
        self.cancelled += 1
        return True

    def clear_planned_path(self) -> bool:
        self.cleared += 1
        return True


class Monitor:
    def __init__(self, module: Module) -> None:
        self.module = module
        self.invalid: set[str] = set()
        self.stale: set[str] = set()

    def get_current_joint_state(self, robot_id: str) -> JointState:
        return JointState(self.module.states[robot_id.removeprefix("id-")])

    def is_state_stale(self, robot_id: str, max_age: float = 1.0) -> bool:
        return robot_id in self.stale

    def is_state_valid(self, robot_id: str, _state: JointState) -> bool:
        return robot_id not in self.invalid

    def get_group_ee_pose(self, group_id: str, _state: JointState | None = None) -> Pose:
        return Pose({"position": [0.1, 0.2, 0.3], "orientation": [0.0, 0.0, 0.0, 1.0]})


@pytest.fixture
def panel() -> Iterator[
    Callable[[list[PlanningGroup], dict[str, JointState]], tuple[ViserPanelGui, Module, Server]]
]:
    panels: list[ViserPanelGui] = []

    def make(
        groups: list[PlanningGroup], states: dict[str, JointState]
    ) -> tuple[ViserPanelGui, Module, Server]:
        module = Module(groups, states)
        server = Server()
        gui = ViserPanelGui(
            server, Monitor(module), module, ViserVisualizationConfig(panel_enabled=True)
        )
        gui.start()
        panels.append(gui)
        return gui, module, server

    yield make
    for gui in panels:
        gui.close()


def states(*robots: str) -> dict[str, JointState]:
    return {robot: JointState({"name": ["j1", "j2"], "position": [0.1, 0.2]}) for robot in robots}


def test_panel_contract_group_order_defaults_and_controls(
    panel: Callable[..., tuple[ViserPanelGui, Module, Server]],
) -> None:
    pose = group("arm", "manipulator", ("j1",), pose=True)
    auxiliary = group("arm", "gripper", ("j2",))
    gui, _module, server = panel([auxiliary, pose], states("arm"))

    assert [(folder.label, folder.kwargs) for folder in server.gui.folders] == [
        ("Manipulation Panel", {"expand_by_default": True}),
        ("Joint Control", {"expand_by_default": False}),
    ]
    assert [button.label for button in server.gui.buttons] == [
        "arm",
        "arm gripper",
        "Plan",
        "Preview",
        "Execute",
        "Cancel",
        "Clear plan",
    ]
    assert "robot" not in gui._handles
    assert (
        server.gui.markdown[1].value
        == "### Planning Groups\nActive MoveIt group for pose goal, planning, and joint edits."
    )
    assert [button.color for button in server.gui.buttons[:2]] == [
        ACTIVE_GROUP_COLOR,
        INACTIVE_GROUP_COLOR,
    ]
    assert gui.state.selected_group_ids == ("arm/manipulator",)
    assert server.gui.dropdowns[0].options == ["Select preset...", "Init", "Current", "Home"]
    assert [
        (slider.label, slider.min, slider.max, slider.value) for slider in server.gui.sliders
    ] == [("arm/manipulator/j1", -1.0, 1.0, 0.1)]
    server.gui.buttons[1].callback(SimpleNamespace())
    assert gui.state.selected_group_ids == ("arm/manipulator", "arm/gripper")
    assert [slider.label for slider in server.gui.sliders if not slider.removed] == [
        "arm/manipulator/j1",
        "arm/gripper/j2",
    ]


def test_backend_composes_complete_states_with_duplicate_local_names() -> None:
    left, right = group("left", "manipulator", ("j1",)), group("right", "manipulator", ("j1",))
    module = Module([left, right], states("left", "right"))
    monitor = Monitor(module)
    targets = {
        left.id: JointState({"name": ["left/j1"], "position": [0.7]}),
        right.id: JointState({"name": ["right/j1"], "position": [0.8]}),
    }
    complete = complete_states_for_targets(monitor, module, (left.id, right.id), targets)
    assert complete is not None
    assert complete["left"].position == [0.7, 0.2]
    assert complete["right"].position == [0.8, 0.2]
    assert (
        evaluate_joint_target_set(monitor, module, (left.id, right.id), targets)["status"]
        == "FEASIBLE"
    )


def test_target_callbacks_require_current_sequence_and_selection_epoch(
    panel: Callable[..., tuple[ViserPanelGui, Module, Server]],
) -> None:
    first, second = (
        group("arm", "manipulator", ("j1",), pose=True),
        group("arm", "gripper", ("j2",)),
    )
    gui, _module, _server = panel([first, second], states("arm"))
    request = TargetEvaluationRequest(
        1, "joints", selection_epoch=gui.state.selection_epoch, group_ids=(first.id,)
    )
    gui.state.latest_sequence_id = 2
    gui._apply_target_evaluation_result(
        request, {"success": True, "collision_free": True, "status": "FEASIBLE"}
    )
    assert gui.state.target_status == TargetStatus.EMPTY
    gui.state.latest_sequence_id = 1
    gui.state.advance_selection_epoch()
    gui._apply_target_evaluation_result(
        request, {"success": True, "collision_free": True, "status": "FEASIBLE"}
    )
    assert gui.state.target_status == TargetStatus.CHECKING


def test_plan_target_sequence_invalidation_and_unfiltered_all_robot_execute(
    panel: Callable[..., tuple[ViserPanelGui, Module, Server]], monkeypatch: pytest.MonkeyPatch
) -> None:
    left, right = (
        group("left", "manipulator", ("j1",), pose=True),
        group("right", "manipulator", ("j1",), pose=True),
    )
    gui, module, _server = panel([left, right], states("left", "right"))
    gui._toggle_group_selected(right.id)
    gui.state.target_status = TargetStatus.FEASIBLE
    monkeypatch.setattr(
        gui,
        "_operation_worker",
        SimpleNamespace(submit=lambda operation, **_: operation(), stop=lambda **_: None),
    )
    gui._submit_plan()
    assert module.plans[-1][0] == (left.id, right.id)
    assert set(gui.state.plan_state.robot_snapshots) == {"left", "right"}
    assert gui.state.plan_state.plan_receipt == "receipt-1"
    gui.state.next_sequence_id()
    assert gui.state.plan_state.status == PlanStatus.STALE
    gui.state.plan_state = PanelPlanState(
        status=PlanStatus.FRESH,
        group_ids=(left.id, right.id),
        target_sequence_id=gui.state.latest_sequence_id,
        robot_snapshots={"left": module.states["left"], "right": module.states["right"]},
        plan_receipt="receipt-1",
    )
    gui.state.target_status = TargetStatus.FEASIBLE
    gui._submit_execute()
    assert module.executions == 1
    assert module.execution_receipts == ["receipt-1"]


def test_plan_execute_uses_exact_plan_receipt(
    panel: Callable[..., tuple[ViserPanelGui, Module, Server]], monkeypatch: pytest.MonkeyPatch
) -> None:
    selected = group("arm", "manipulator", ("j1",), pose=True)
    gui, module, _server = panel([selected], states("arm"))
    gui.state.target_status = TargetStatus.FEASIBLE
    monkeypatch.setattr(
        gui,
        "_operation_worker",
        SimpleNamespace(submit=lambda operation, **_: operation(), stop=lambda **_: None),
    )

    gui._submit_plan()
    receipt = gui.state.plan_state.plan_receipt
    gui._submit_execute()

    assert receipt == "receipt-1"
    assert module.execution_receipts == [receipt]


def test_initialization_waits_for_complete_fresh_telemetry(
    panel: Callable[..., tuple[ViserPanelGui, Module, Server]],
) -> None:
    selected = group("arm", "manipulator", ("j1", "j2"), pose=True)
    gui, module, _server = panel(
        [selected], {"arm": JointState({"name": ["j1"], "position": [0.4]})}
    )

    assert selected.id not in gui.state.group_joint_targets
    gui._world_monitor.stale.add("id-arm")
    module.states["arm"] = JointState({"name": ["j1", "j2"], "position": [0.4, 0.5]})
    gui.refresh()
    assert selected.id not in gui.state.group_joint_targets

    gui._world_monitor.stale.clear()
    gui.refresh()
    assert gui.state.group_joint_targets[selected.id].position == [0.4, 0.5]


def test_incomplete_preset_preserves_existing_group_targets(
    panel: Callable[..., tuple[ViserPanelGui, Module, Server]],
) -> None:
    selected = group("arm", "manipulator", ("j1", "j2"), pose=True)
    gui, module, _server = panel([selected], states("arm"))
    before = JointState(gui.state.group_joint_targets[selected.id])
    module.get_init_joints = lambda _name: JointState({"name": ["j1"], "position": [-0.5]})

    gui._apply_preset("Init")

    assert gui.state.group_joint_targets[selected.id] == before
    assert "missing joints" in gui.state.error


def test_valid_init_preset_builds_sliders_after_incomplete_initial_telemetry(
    panel: Callable[..., tuple[ViserPanelGui, Module, Server]],
) -> None:
    selected = group("arm", "manipulator", ("j1", "j2"), pose=True)
    gui, _module, server = panel(
        [selected], {"arm": JointState({"name": ["j1"], "position": [0.4]})}
    )

    assert gui.state.group_joint_targets == {}
    assert server.gui.sliders == []

    gui._apply_preset("Init")

    assert gui.state.group_joint_targets[selected.id].position == [-0.5, -1.0]
    assert [slider.label for slider in server.gui.sliders if not slider.removed] == [
        "arm/manipulator/j1",
        "arm/manipulator/j2",
    ]


def test_incomplete_multi_group_preset_does_not_change_any_targets(
    panel: Callable[..., tuple[ViserPanelGui, Module, Server]],
) -> None:
    left, right = (
        group("left", "manipulator", ("j1",), pose=True),
        group("right", "manipulator", ("j1",), pose=True),
    )
    gui, module, _server = panel([left, right], states("left", "right"))
    gui._toggle_group_selected(right.id)
    before = {
        group_id: JointState(target) for group_id, target in gui.state.group_joint_targets.items()
    }
    module.get_init_joints = lambda robot_name: JointState(
        {
            "name": ["j1"] if robot_name == "left" else [],
            "position": [-0.5] if robot_name == "left" else [],
        }
    )

    gui._apply_preset("Init")

    assert gui.state.group_joint_targets == before
    assert "right/manipulator" in gui.state.error


def test_cancel_clear_and_close_invalidate_operations_and_preview_generation(
    panel: Callable[..., tuple[ViserPanelGui, Module, Server]], monkeypatch: pytest.MonkeyPatch
) -> None:
    selected = group("arm", "manipulator", ("j1",), pose=True)
    gui, module, _server = panel([selected], states("arm"))
    submitted: list[Callable[[], None]] = []
    gui._operation_worker.stop()
    monkeypatch.setattr(
        gui,
        "_operation_worker",
        SimpleNamespace(
            submit=lambda operation, **_: submitted.append(operation),
            stop=lambda **_: None,
            start=lambda: None,
        ),
    )
    gui.state.target_status = TargetStatus.FEASIBLE
    gui._submit_plan()
    gui._submit_clear()
    submitted[0]()
    assert module.plans == []
    submitted[1]()
    assert module.cleared == 1 and gui.state.plan_state.status == PlanStatus.NONE
    gui.close()
    status_before_callback = gui.state.target_status
    gui._apply_target_evaluation_result(
        TargetEvaluationRequest(0, "joints"), {"success": True, "collision_free": True}
    )
    assert gui.state.target_status is status_before_callback


class Mesh:
    def __init__(self) -> None:
        self.visible = False
        self.color: tuple[int, int, int] | None = None
        self.opacity: float | None = None


class Urdf:
    def __init__(self, *_: object, **__: object) -> None:
        self._urdf = SimpleNamespace(actuated_joint_names=("j1", "j2"))
        self._meshes = [Mesh()]
        self.cfg: list[float] | None = None

    def update_cfg(self, cfg: Sequence[float]) -> None:
        self.cfg = list(cfg)

    def remove(self) -> None:
        pass


def test_scene_active_only_ghosts_group_gizmos_feasibility_and_shared_ticks(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    updates: list[tuple[str, list[float]]] = []
    server = Server()
    server.scene.add_grid = lambda *_args, **_kwargs: Handle()
    server.scene.add_transform_controls = lambda *_args, **_kwargs: Handle()
    scene = ViserManipulationScene(server, Urdf, preview_fps=2.0)
    scene.prepared_urdf_path = lambda _config: "robot.urdf"  # type: ignore[method-assign]
    config = Config("arm", ["j1", "j2"], [-1.0, -2.0], [1.0, 2.0], [0.0, 0.0])
    scene.register_robot("id-arm", config)
    scene.set_target_active("id-arm", False)
    assert scene._urdfs["id-arm:target"]._meshes[0].visible is False
    scene.set_target_joints("id-arm", ["j1", "j2"], [0.8, 0.2])
    assert scene._urdfs["id-arm:target"].cfg == [0.8, 0.2]
    scene.set_target_visual_state("id-arm", False)
    assert scene._urdfs["id-arm:target"]._meshes[0].color == (255, 30, 30)
    monkeypatch.setattr(
        "dimos.manipulation.visualization.viser.scene.time.sleep", lambda _delay: None
    )
    original = scene._set_preview_ghost_joints
    scene._set_preview_ghost_joints = lambda robot, names, values: (
        updates.append((robot, list(values))),
        original(robot, names, values),
    )  # type: ignore[method-assign]
    preview = GroupPreviewAnimation(
        (
            PreviewTrack(
                "id-arm",
                ("j1", "j2"),
                (
                    JointState({"name": ["j1", "j2"], "position": [0.0, 0.2]}),
                    JointState({"name": ["j1", "j2"], "position": [1.0, 0.2]}),
                ),
            ),
        )
    )
    assert scene.animate_preview(preview, 1.0) is True
    assert updates[-1] == ("id-arm", [1.0, 0.2])


def test_theme_and_reference_scene_contract() -> None:
    server = Server()
    assert apply_dimos_theme(server) is True
    assert server.gui.theme_kwargs is not None
    assert server.gui.theme_kwargs["brand_color"] == (0, 153, 255)
    assert server.gui.theme_kwargs["dark_mode"] is True
    assert server.gui.theme_kwargs["control_layout"] == "fixed"
    assert ViserVisualizationConfig().panel_enabled is True


def test_preview_selection_rejects_malformed_before_visibility() -> None:
    selection = PlanningGroupSelection.from_groups((group("arm", "manipulator", ("j1",)),))
    assert selection.group_ids == ("arm/manipulator",)
    # The scene transaction itself rejects missing tracks before revealing ghosts.
    server = Server()
    server.scene.add_grid = lambda *_args, **_kwargs: Handle()
    scene = ViserManipulationScene(server, Urdf, preview_fps=10.0)
    assert scene.animate_preview(GroupPreviewAnimation(()), 1.0) is False


def test_group_controls_use_source_labels_and_active_colors(
    panel: Callable[..., tuple[ViserPanelGui, Module, Server]],
) -> None:
    pose = group("arm", "manipulator", ("j1",), pose=True)
    auxiliary = group("arm", "gripper", ("j2",))
    gui, _module, server = panel([auxiliary, pose], states("arm"))

    assert group_display_name(pose) == "arm"
    assert group_display_name(auxiliary) == "arm gripper"
    assert [button.label for button in server.gui.buttons[:2]] == ["arm", "arm gripper"]
    assert [button.color for button in server.gui.buttons[:2]] == [
        ACTIVE_GROUP_COLOR,
        INACTIVE_GROUP_COLOR,
    ]
    assert server.gui.buttons[1].callback is not None
    server.gui.buttons[1].callback(SimpleNamespace())
    assert gui._handles[f"group:{auxiliary.id}"].color == ACTIVE_GROUP_COLOR


def test_panel_preset_defaults_and_joint_slider_limits(
    panel: Callable[..., tuple[ViserPanelGui, Module, Server]],
) -> None:
    selected = group("arm", "manipulator", ("j1", "j2"), pose=True)
    _gui, _module, server = panel([selected], states("arm"))

    assert server.gui.dropdowns[0].options == ["Select preset...", "Init", "Current", "Home"]
    assert [
        (slider.label, slider.min, slider.max, slider.step, slider.value)
        for slider in server.gui.sliders
    ] == [
        ("arm/manipulator/j1", -1.0, 1.0, 0.001, 0.1),
        ("arm/manipulator/j2", -2.0, 2.0, 0.001, 0.2),
    ]


def test_panel_action_controls_are_present_in_source_order(
    panel: Callable[..., tuple[ViserPanelGui, Module, Server]],
) -> None:
    selected = group("arm", "manipulator", ("j1",), pose=True)
    _gui, _module, server = panel([selected], states("arm"))

    assert [button.label for button in server.gui.buttons[1:]] == [
        "Plan",
        "Preview",
        "Execute",
        "Cancel",
        "Clear plan",
    ]
    assert [folder.label for folder in server.gui.folders] == [
        "Manipulation Panel",
        "Joint Control",
    ]


def test_target_callbacks_require_current_target_identity(
    panel: Callable[..., tuple[ViserPanelGui, Module, Server]],
) -> None:
    first, second = (
        group("arm", "manipulator", ("j1",), pose=True),
        group("arm", "gripper", ("j2",)),
    )
    gui, _module, _server = panel([first, second], states("arm"))
    request = TargetEvaluationRequest(
        gui.state.next_sequence_id(),
        "joints",
        selection_epoch=gui.state.selection_epoch,
        group_ids=(second.id,),
    )

    gui._apply_target_evaluation_result(
        request, {"success": True, "collision_free": True, "status": "FEASIBLE"}
    )

    assert gui.state.target_status == TargetStatus.CHECKING


def test_backend_rejects_unknown_or_malformed_group_targets() -> None:
    selected = group("arm", "manipulator", ("j1",))
    module = Module([selected], states("arm"))
    monitor = Monitor(module)
    malformed = {selected.id: JointState({"name": ["j1"], "position": [0.7]})}

    assert complete_states_for_targets(monitor, module, ("missing",), {}) is None
    assert complete_states_for_targets(monitor, module, (selected.id,), malformed) is None
    assert (
        evaluate_joint_target_set(monitor, module, (selected.id,), malformed).get("status")
        == "INVALID"
    )


def test_backend_marks_complete_target_infeasible_when_any_robot_is_invalid() -> None:
    left, right = group("left", "manipulator", ("j1",)), group("right", "manipulator", ("j1",))
    module = Module([left, right], states("left", "right"))
    monitor = Monitor(module)
    monitor.invalid.add("id-right")
    targets = {
        left.id: JointState({"name": ["left/j1"], "position": [0.7]}),
        right.id: JointState({"name": ["right/j1"], "position": [0.8]}),
    }

    result = evaluate_joint_target_set(monitor, module, (left.id, right.id), targets)

    assert result.get("status") == "COLLISION"
    assert result.get("collision_free") is False
    assert set(result.get("group_diagnostics", {})) == {left.id, right.id}


def test_backend_joint_evaluation_returns_forward_kinematics_for_each_group() -> None:
    selected = group("arm", "manipulator", ("j1",), pose=True)
    module = Module([selected], states("arm"))
    result = evaluate_joint_target_set(
        Monitor(module),
        module,
        (selected.id,),
        {selected.id: JointState({"name": ["arm/j1"], "position": [0.7]})},
    )

    pose = result.get("group_poses", {}).get(selected.id)
    assert pose is not None
    assert list(pose.position) == [0.1, 0.2, 0.3]


def test_backend_cartesian_evaluation_uses_group_ik_and_complete_state_validation() -> None:
    selected = group("arm", "manipulator", ("j1",), pose=True)
    module = Module([selected], states("arm"))
    calls: list[dict[str, object]] = []

    def inverse_kinematics(
        *,
        pose_targets: object,
        auxiliary_group_ids: Sequence[str] = (),
        seed: JointState | None = None,
        check_collision: bool = True,
    ) -> SimpleNamespace:
        calls.append(
            {
                "pose_targets": pose_targets,
                "auxiliary_group_ids": auxiliary_group_ids,
                "seed": seed,
                "check_collision": check_collision,
            }
        )
        return SimpleNamespace(
            is_success=lambda: True,
            joint_state=JointState({"name": ["arm/j1"], "position": [0.7]}),
        )

    module.inverse_kinematics = inverse_kinematics
    result = evaluate_pose_target_set(
        Monitor(module),
        module,
        {selected.id: Pose({"position": [0.1, 0.2, 0.3], "orientation": [0.0, 0.0, 0.0, 1.0]})},
    )

    assert calls[0]["check_collision"] is True
    assert result.get("status") == "FEASIBLE"


def test_scene_target_ghost_tracks_current_only_until_explicit_target() -> None:
    server = Server()
    server.scene.add_grid = lambda *_args, **_kwargs: Handle()
    scene = ViserManipulationScene(server, Urdf, preview_fps=2.0)
    scene.prepared_urdf_path = lambda _config: "robot.urdf"  # type: ignore[method-assign]
    config = Config("arm", ["j1", "j2"], [-1.0, -2.0], [1.0, 2.0], [0.0, 0.0])
    scene.register_robot("id-arm", config)

    scene.update_current_robot("id-arm", JointState({"name": ["j1", "j2"], "position": [0.1, 0.2]}))
    assert scene._urdfs["id-arm:target"].cfg == [0.1, 0.2]
    scene.set_target_joints("id-arm", ["j1", "j2"], [0.8, 0.9])
    scene.update_current_robot("id-arm", JointState({"name": ["j1", "j2"], "position": [0.2, 0.3]}))

    assert scene._urdfs["id-arm:target"].cfg == [0.8, 0.9]


def test_scene_target_feasibility_colors_ghost_and_gizmo() -> None:
    server = Server()
    server.scene.add_grid = lambda *_args, **_kwargs: Handle()
    server.scene.add_transform_controls = lambda *_args, **_kwargs: Handle()
    scene = ViserManipulationScene(server, Urdf, preview_fps=2.0)
    scene.prepared_urdf_path = lambda _config: "robot.urdf"  # type: ignore[method-assign]
    config = Config("arm", ["j1", "j2"], [-1.0, -2.0], [1.0, 2.0], [0.0, 0.0])
    scene.register_robot("id-arm", config)
    scene.ensure_target_controls("id-arm", lambda _target: None)

    scene.set_target_visual_state("id-arm", False)

    assert scene._urdfs["id-arm:target"]._meshes[0].color == (255, 30, 30)
    assert scene._handles["id-arm:ee_control"].color == (255, 40, 40)


def test_panel_feasibility_colors_group_controls_and_deduplicated_robot_ghosts() -> None:
    arm_primary, arm_secondary, other = (
        group("arm", "primary", ("j1",), pose=True),
        group("arm", "secondary", ("j2",), pose=True),
        group("other", "manipulator", ("j1",), pose=True),
    )
    module = Module([arm_primary, arm_secondary, other], states("arm", "other"))
    server = Server()
    server.scene.add_grid = lambda *_args, **_kwargs: Handle()
    server.scene.add_transform_controls = lambda *_args, **_kwargs: Handle()
    scene = ViserManipulationScene(server, Urdf, preview_fps=2.0)
    scene.prepared_urdf_path = lambda _config: "robot.urdf"  # type: ignore[method-assign]
    scene.register_robot("id-arm", module.configs["arm"])
    scene.register_robot("id-other", module.configs["other"])
    gui = ViserPanelGui(server, Monitor(module), module, ViserVisualizationConfig(), scene)
    gui.start()
    gui._worker.stop()
    gui._operation_worker.stop()
    gui._toggle_group_selected(arm_secondary.id)
    gui._toggle_group_selected(other.id)

    control_calls: list[str] = []
    robot_calls: list[str] = []
    original_control = scene.set_target_control_visual_state
    original_robot = scene.set_target_robot_visual_state
    scene.set_target_control_visual_state = lambda group_id, feasible: (
        control_calls.append(group_id),
        original_control(group_id, feasible),
    )  # type: ignore[method-assign]
    scene.set_target_robot_visual_state = lambda robot_id, feasible: (
        robot_calls.append(robot_id),
        original_robot(robot_id, feasible),
    )  # type: ignore[method-assign]
    request = TargetEvaluationRequest(
        gui.state.next_sequence_id(),
        "joints",
        selection_epoch=gui.state.selection_epoch,
        group_ids=gui.state.selected_group_ids,
    )

    gui._apply_target_evaluation_result(
        request, {"success": True, "collision_free": True, "status": "FEASIBLE"}
    )

    assert control_calls == [arm_primary.id, arm_secondary.id, other.id] * 2
    assert robot_calls == ["id-arm", "id-other"] * 2
    assert all(
        scene._handles[f"{item.id}:ee_control"].color == TARGET_CONTROL_FEASIBLE_COLOR
        for item in (arm_primary, arm_secondary, other)
    )
    assert scene._urdfs["id-arm:target"]._meshes[0].color == GOAL_ROBOT_FEASIBLE_COLOR
    assert scene._urdfs["id-other:target"]._meshes[0].color == GOAL_ROBOT_FEASIBLE_COLOR

    control_calls.clear()
    robot_calls.clear()
    request = TargetEvaluationRequest(
        gui.state.next_sequence_id(),
        "joints",
        selection_epoch=gui.state.selection_epoch,
        group_ids=gui.state.selected_group_ids,
    )
    gui._apply_target_evaluation_result(
        request, {"success": True, "collision_free": False, "status": "COLLISION"}
    )

    assert control_calls == [arm_primary.id, arm_secondary.id, other.id] * 2
    assert robot_calls == ["id-arm", "id-other"] * 2
    assert all(
        scene._handles[f"{item.id}:ee_control"].color == TARGET_CONTROL_INFEASIBLE_COLOR
        for item in (arm_primary, arm_secondary, other)
    )
    assert scene._urdfs["id-arm:target"]._meshes[0].color == GOAL_ROBOT_INFEASIBLE_COLOR
    assert scene._urdfs["id-other:target"]._meshes[0].color == GOAL_ROBOT_INFEASIBLE_COLOR
    gui.close()


def test_scene_shared_clock_resamples_unequal_robot_frames(monkeypatch: pytest.MonkeyPatch) -> None:
    server = Server()
    server.scene.add_grid = lambda *_args, **_kwargs: Handle()
    scene = ViserManipulationScene(server, Urdf, preview_fps=2.0)
    scene.prepared_urdf_path = lambda _config: "robot.urdf"  # type: ignore[method-assign]
    config = Config("arm", ["j1", "j2"], [-1.0, -2.0], [1.0, 2.0], [0.0, 0.0])
    scene.register_robot("left", config)
    scene.register_robot("right", config)
    updates: list[tuple[str, list[float]]] = []
    monkeypatch.setattr(
        scene,
        "_set_preview_ghost_joints",
        lambda robot, _names, values: updates.append((robot, list(values))),
    )
    monkeypatch.setattr(
        "dimos.manipulation.visualization.viser.scene.time.sleep", lambda _delay: None
    )

    assert scene.animate_preview(
        GroupPreviewAnimation(
            (
                PreviewTrack("left", ("j1",), (JointState({"name": ["j1"], "position": [0.0]}),)),
                PreviewTrack(
                    "right",
                    ("j1",),
                    (
                        JointState({"name": ["j1"], "position": [10.0]}),
                        JointState({"name": ["j1"], "position": [11.0]}),
                    ),
                ),
            )
        ),
        1.0,
    )
    assert updates == [
        ("left", [0.0]),
        ("right", [10.0]),
        ("left", [0.0]),
        ("right", [10.5]),
        ("left", [0.0]),
        ("right", [11.0]),
    ]


def test_animation_frame_helpers_preserve_dense_paths_and_interpolate_sparse_paths() -> None:
    dense = [JointState({"name": ["j1"], "position": [float(index)]}) for index in range(4)]
    sparse = [
        JointState({"name": ["j1"], "position": [0.0]}),
        JointState({"name": ["j1"], "position": [1.0]}),
    ]

    assert sampled_joint_path_frames(dense, 1.0, 2.0) == [[0.0], [1.0], [2.0], [3.0]]
    assert interpolate_joint_path(sparse, 1.0, 2.0) == [[0.0], [0.5], [1.0]]


def test_panel_disables_plan_preview_and_execute_until_a_feasible_target(
    panel: Callable[..., tuple[ViserPanelGui, Module, Server]],
) -> None:
    selected = group("arm", "manipulator", ("j1",), pose=True)
    _gui, _module, server = panel([selected], states("arm"))

    assert [button.disabled for button in server.gui.buttons[1:4]] == [True, True, True]


def test_panel_status_reports_target_and_plan_defaults(
    panel: Callable[..., tuple[ViserPanelGui, Module, Server]],
) -> None:
    selected = group("arm", "manipulator", ("j1",), pose=True)
    gui, _module, server = panel([selected], states("arm"))

    assert server.gui.markdown[0].value == "### Status\n**State:** Ready"
    assert gui.state.error == ""


def test_scene_reference_grid_has_expected_defaults_and_toggle() -> None:
    server = Server()
    grids: list[Handle] = []
    server.scene.add_grid = lambda *_args, **_kwargs: grids.append(Handle()) or grids[-1]
    scene = ViserManipulationScene(server, Urdf, preview_fps=2.0)

    assert scene.has_reference_grid() is True
    scene.set_reference_grid_visible(False)
    assert grids[0].visible is False
    scene.set_reference_grid_visible(True)
    assert grids[0].visible is True


def test_scene_returns_false_for_missing_robot_target_updates() -> None:
    server = Server()
    server.scene.add_grid = lambda *_args, **_kwargs: Handle()
    scene = ViserManipulationScene(server, Urdf, preview_fps=2.0)

    assert scene.set_target_joints("missing", ["j1"], [0.1]) is False
    assert scene.animate_path("missing", [], duration=0.0) is False


def test_scene_cancel_generation_hides_preview_and_rejects_old_animation() -> None:
    server = Server()
    server.scene.add_grid = lambda *_args, **_kwargs: Handle()
    scene = ViserManipulationScene(server, Urdf, preview_fps=2.0)
    scene.prepared_urdf_path = lambda _config: "robot.urdf"  # type: ignore[method-assign]
    config = Config("arm", ["j1", "j2"], [-1.0, -2.0], [1.0, 2.0], [0.0, 0.0])
    scene.register_robot("id-arm", config)
    scene.show_preview("id-arm")

    scene.cancel_preview_animation()

    assert scene._preview_visible == {"id-arm": False}
    assert scene._animation_generation == 1


def test_scene_base_pose_requires_urdf_root_to_match(monkeypatch: pytest.MonkeyPatch) -> None:
    server = Server()
    server.scene.add_grid = lambda *_args, **_kwargs: Handle()
    scene = ViserManipulationScene(server, Urdf, preview_fps=2.0)
    monkeypatch.setattr(
        "dimos.manipulation.visualization.viser.scene.parse_model",
        lambda _path: SimpleNamespace(root_link="world"),
    )

    with pytest.raises(ValueError, match="base_link 'base'.*URDF root 'world'"):
        scene._assert_base_link_is_urdf_root(SimpleNamespace(base_link="base"), "robot.urdf")


def test_scene_detects_non_identity_base_pose() -> None:
    identity = SimpleNamespace(
        base_pose=Pose({"position": [0.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]})
    )
    translated = SimpleNamespace(
        base_pose=Pose({"position": [1.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]})
    )

    assert ViserManipulationScene._has_non_identity_base_pose(identity) is False
    assert ViserManipulationScene._has_non_identity_base_pose(translated) is True


@pytest.mark.parametrize("rejection", ["changed", "stale", "missing_snapshot"])
def test_all_robot_execute_rejects_secondary_robot_preconditions(
    panel: Callable[..., tuple[ViserPanelGui, Module, Server]],
    monkeypatch: pytest.MonkeyPatch,
    rejection: str,
) -> None:
    left, right = (
        group("left", "manipulator", ("j1",), pose=True),
        group("right", "manipulator", ("j1",), pose=True),
    )
    gui, module, _server = panel([left, right], states("left", "right"))
    gui._toggle_group_selected(right.id)
    gui.state.target_status = TargetStatus.FEASIBLE
    gui.state.plan_state = PanelPlanState(
        status=PlanStatus.FRESH,
        group_ids=(left.id, right.id),
        target_sequence_id=gui.state.latest_sequence_id,
        robot_snapshots={
            "left": JointState(module.states["left"]),
            "right": JointState(module.states["right"]),
        },
        plan_receipt="receipt-1",
    )
    monkeypatch.setattr(
        gui,
        "_operation_worker",
        SimpleNamespace(submit=lambda operation, **_: operation(), stop=lambda **_: None),
    )
    # Enter the operation so each rejection is asserted at its authoritative
    # all-robot preflight, rather than only through a disabled GUI button.
    monkeypatch.setattr(gui, "_can_execute", lambda: True)
    if rejection == "changed":
        module.states["right"] = JointState({"name": ["j1", "j2"], "position": [0.9, 0.2]})
    elif rejection == "stale":
        gui._world_monitor.stale.add("id-right")
    else:
        gui.state.plan_state.robot_snapshots.pop("right")

    gui._submit_execute()

    assert module.executions == 0
    assert gui.state.plan_state.status is PlanStatus.STALE
    if rejection == "stale":
        assert "right" in gui.state.error


def test_visualizer_rejects_invalid_group_preview_before_scene_animation() -> None:
    selected = group("arm", "manipulator", ("j1",))
    module = Module([selected], states("arm"))
    monitor = Monitor(module)
    monitor.planning_groups = SimpleNamespace(
        select=lambda ids: (
            (_ for _ in ()).throw(KeyError("unknown"))
            if tuple(ids) == ("missing",)
            else PlanningGroupSelection.from_groups((selected,))
        )
    )
    visualizer = ViserManipulationVisualizer(
        world_monitor=monitor,
        manipulation_module=module,
        config=ViserVisualizationConfig(panel_enabled=False),
    )
    calls: list[object] = []
    visualizer._ensure_started = lambda: None  # type: ignore[method-assign]
    visualizer._scene = SimpleNamespace(animate_preview=lambda *args: calls.append(args))
    plans = (
        (
            SimpleNamespace(
                group_ids=("missing",), path=(JointState({"name": ["arm/j1"], "position": [0.1]}),)
            ),
            None,
        ),
        (
            SimpleNamespace(
                group_ids=(selected.id,), path=(JointState({"name": ["arm/j1"], "position": []}),)
            ),
            None,
        ),
        (
            SimpleNamespace(
                group_ids=(selected.id,),
                path=(JointState({"name": ["arm/j2"], "position": [0.1]}),),
            ),
            None,
        ),
        (
            SimpleNamespace(
                group_ids=(selected.id,),
                path=(JointState({"name": ["arm/j1"], "position": [0.1]}),),
            ),
            None,
        ),
        (
            SimpleNamespace(
                group_ids=(selected.id,),
                path=(JointState({"name": ["arm/j1"], "position": [0.1]}),),
            ),
            JointState({"name": ["j1"], "position": [0.1]}),
        ),
    )
    for plan, current in plans:
        monitor.get_current_joint_state = lambda _robot_id, state=current: state  # type: ignore[method-assign]
        visualizer.animate_plan(plan)

    assert calls == []


@pytest.mark.parametrize("interruption", ["cancel", "replacement", "close"])
def test_scene_inflight_preview_never_updates_after_generation_replacement(
    monkeypatch: pytest.MonkeyPatch, interruption: str
) -> None:
    server = Server()
    server.scene.add_grid = lambda *_args, **_kwargs: Handle()
    scene = ViserManipulationScene(server, Urdf, preview_fps=2.0)
    scene.prepared_urdf_path = lambda _config: "robot.urdf"  # type: ignore[method-assign]
    config = Config("arm", ["j1", "j2"], [-1.0, -2.0], [1.0, 2.0], [0.0, 0.0])
    scene.register_robot("id-arm", config)
    first_tick, release = threading.Event(), threading.Event()
    updates: list[float] = []
    original = scene._set_preview_ghost_joints

    def record(robot_id: str, names: Sequence[str], values: Sequence[float]) -> None:
        updates.append(float(values[0]))
        original(robot_id, names, values)

    scene._set_preview_ghost_joints = record  # type: ignore[method-assign]
    sleep_calls = 0

    def block_after_first_tick(_delay: float) -> None:
        nonlocal sleep_calls
        sleep_calls += 1
        if sleep_calls == 1:
            first_tick.set()
            assert release.wait(timeout=2.0)

    monkeypatch.setattr(
        "dimos.manipulation.visualization.viser.scene.time.sleep", block_after_first_tick
    )
    old = GroupPreviewAnimation(
        (
            PreviewTrack(
                "id-arm",
                ("j1",),
                (
                    JointState({"name": ["j1"], "position": [0.0]}),
                    JointState({"name": ["j1"], "position": [1.0]}),
                ),
            ),
        )
    )
    worker = threading.Thread(target=lambda: scene.animate_preview(old, 1.0))
    worker.start()
    assert first_tick.wait(timeout=2.0)
    if interruption == "cancel":
        visualizer = ViserManipulationVisualizer(
            world_monitor=SimpleNamespace(),
            manipulation_module=SimpleNamespace(),
            config=ViserVisualizationConfig(panel_enabled=False),
        )
        visualizer._scene = scene
        visualizer.cancel_preview_animation()
        stable_updates = list(updates)
    elif interruption == "replacement":
        updates.clear()
        assert scene.animate_preview(
            GroupPreviewAnimation(
                (
                    PreviewTrack(
                        "id-arm", ("j1",), (JointState({"name": ["j1"], "position": [10.0]}),)
                    ),
                )
            ),
            0.0,
        )
        stable_updates = list(updates)
    else:
        scene.close()
        stable_updates = list(updates)
    release.set()
    worker.join(timeout=2.0)

    assert not worker.is_alive()
    assert updates == stable_updates


def test_transform_control_callback_preserves_pose_through_gui_and_backend(
    panel: Callable[..., tuple[ViserPanelGui, Module, Server]],
) -> None:
    selected = group("arm", "manipulator", ("j1",), pose=True)
    module = Module([selected], states("arm"))
    server = Server()
    server.scene.add_grid = lambda *_args, **_kwargs: Handle()
    server.scene.add_transform_controls = lambda *_args, **_kwargs: Handle()
    scene = ViserManipulationScene(server, Urdf, preview_fps=2.0)
    gui = ViserPanelGui(server, Monitor(module), module, ViserVisualizationConfig(), scene)
    submitted: list[TargetEvaluationRequest] = []
    gui._worker.submit = submitted.append  # type: ignore[method-assign]
    gui.start()
    control = scene._handles[f"{selected.id}:ee_control"]
    control.position = (1.0, 2.0, 3.0)
    control.wxyz = (0.4, 0.1, 0.2, 0.3)
    assert control.callback is not None
    control.callback(SimpleNamespace(target=control))

    request = submitted[-1]
    assert list(gui.state.pose_targets[selected.id].position) == [1.0, 2.0, 3.0]
    assert list(gui.state.pose_targets[selected.id].orientation) == [0.1, 0.2, 0.3, 0.4]
    assert control.position == (1.0, 2.0, 3.0)
    assert control.wxyz == (0.4, 0.1, 0.2, 0.3)
    assert request.pose_targets[selected.id] == gui.state.pose_targets[selected.id]
    gui.close()


def test_joint_evaluation_updates_active_gizmo_from_computed_group_pose() -> None:
    selected = group("arm", "manipulator", ("j1",), pose=True)
    module = Module([selected], states("arm"))
    server = Server()
    server.scene.add_grid = lambda *_args, **_kwargs: Handle()
    server.scene.add_transform_controls = lambda *_args, **_kwargs: Handle()
    scene = ViserManipulationScene(server, Urdf, preview_fps=2.0)
    gui = ViserPanelGui(server, Monitor(module), module, ViserVisualizationConfig(), scene)
    gui.start()
    control = scene._handles[f"{selected.id}:ee_control"]
    request = TargetEvaluationRequest(
        gui.state.next_sequence_id(),
        "joints",
        selection_epoch=gui.state.selection_epoch,
        group_ids=gui.state.selected_group_ids,
    )
    computed_pose = Pose({"position": [0.7, 0.8, 0.9], "orientation": [0.1, 0.2, 0.3, 0.4]})

    gui._apply_target_evaluation_result(
        request,
        {
            "success": True,
            "collision_free": True,
            "status": "FEASIBLE",
            "group_poses": {selected.id: computed_pose},
        },
    )

    assert control.position == (0.7, 0.8, 0.9)
    assert control.wxyz == (0.4, 0.1, 0.2, 0.3)
    gui.close()


def test_delayed_cartesian_evaluation_updates_joints_without_rewriting_dragged_gizmo() -> None:
    selected = group("arm", "manipulator", ("j1",), pose=True)
    module = Module([selected], states("arm"))
    server = Server()
    server.scene.add_grid = lambda *_args, **_kwargs: Handle()
    server.scene.add_transform_controls = lambda *_args, **_kwargs: Handle()
    scene = ViserManipulationScene(server, Urdf, preview_fps=2.0)
    scene.prepared_urdf_path = lambda _config: "robot.urdf"  # type: ignore[method-assign]
    scene.register_robot("id-arm", module.configs["arm"])
    gui = ViserPanelGui(server, Monitor(module), module, ViserVisualizationConfig(), scene)
    gui.start()
    try:
        requests: list[TargetEvaluationRequest] = []
        gui._worker.submit = requests.append  # type: ignore[method-assign]
        control = scene._handles[f"{selected.id}:ee_control"]
        control.position = (1.0, 2.0, 3.0)
        control.wxyz = (0.4, 0.1, 0.2, 0.3)
        assert control.callback is not None
        control.callback(SimpleNamespace(target=control))
        request = requests[-1]

        gui._apply_target_evaluation_result(
            request,
            {
                "success": True,
                "collision_free": True,
                "status": "FEASIBLE",
                "target_joints": JointState({"name": ["arm/j1"], "position": [0.7]}),
            },
        )

        assert server.gui.sliders[0].value == 0.7
        assert scene._urdfs["id-arm:target"].cfg == [0.7, 0.2]
        assert control.position == (1.0, 2.0, 3.0)
        assert control.wxyz == (0.4, 0.1, 0.2, 0.3)
    finally:
        gui.close()


def test_cartesian_evaluation_forwards_auxiliary_groups_and_rejects_composed_invalid_state() -> (
    None
):
    pose_group, auxiliary = (
        group("arm", "manipulator", ("j1",), pose=True),
        group("arm", "gripper", ("j2",)),
    )
    module = Module([pose_group, auxiliary], states("arm"))
    monitor = Monitor(module)
    calls: list[Sequence[str]] = []
    validated_states: list[dict[str, float]] = []

    def is_state_valid(robot_id: str, state: JointState) -> bool:
        values = dict(zip(state.name, state.position, strict=True))
        validated_states.append(values)
        assert robot_id == "id-arm"
        assert values == {"j1": 0.7, "j2": 0.8}
        return values != {"j1": 0.7, "j2": 0.8}

    monitor.is_state_valid = is_state_valid  # type: ignore[method-assign]

    def inverse_kinematics(**kwargs: object) -> SimpleNamespace:
        calls.append(kwargs["auxiliary_group_ids"])  # type: ignore[arg-type]
        return SimpleNamespace(
            is_success=lambda: True,
            joint_state=JointState({"name": ["arm/j1", "arm/j2"], "position": [0.7, 0.8]}),
        )

    module.inverse_kinematics = inverse_kinematics
    result = evaluate_pose_target_set(
        monitor,
        module,
        {pose_group.id: Pose({"position": [0.1, 0.2, 0.3], "orientation": [0.0, 0.0, 0.0, 1.0]})},
        (auxiliary.id,),
    )

    assert calls == [(auxiliary.id,)]
    assert validated_states == [{"j1": 0.7, "j2": 0.8}] * 2
    assert result["status"] == "COLLISION"
    assert result["collision_free"] is False
    assert result["group_ids"] == (pose_group.id, auxiliary.id)
    assert result["group_diagnostics"] == {
        pose_group.id: "Target is in collision or violates limits",
        auxiliary.id: "Target is in collision or violates limits",
    }


def test_urdf_projection_reorders_global_selected_joints_with_current_baseline() -> None:
    config = Config("arm", ["j2", "j1", "j3"], [-2.0, -1.0, -3.0], [2.0, 1.0, 3.0], None)
    plan = SimpleNamespace(
        path=(JointState({"name": ["arm/j1"], "position": [0.7]}),),
    )
    projected = ViserManipulationVisualizer._project_plan_path(
        "arm",
        config,
        JointState({"name": ["j1", "j2", "j3"], "position": [0.1, 0.2, 0.3]}),
        plan,
    )
    assert projected is not None
    server = Server()
    server.scene.add_grid = lambda *_args, **_kwargs: Handle()
    scene = ViserManipulationScene(server, Urdf, preview_fps=2.0)
    urdf = Urdf()
    urdf._urdf.actuated_joint_names = ("j3", "j1", "j2")
    scene.set_urdf_joints(urdf, projected[0].name, projected[0].position)

    assert projected[0].position == [0.2, 0.7, 0.3]
    assert urdf.cfg == [0.3, 0.7, 0.2]
