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
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest

pytest.importorskip("viser", reason="Viser optional dependency is not installed")

from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan, PlanningSceneInfo
from dimos.manipulation.visualization.operator import TargetEvaluationResult
from dimos.manipulation.visualization.viser.animation import (
    GroupPreviewAnimation,
    PreviewFrame,
    PreviewTrack,
    scaled_frame_delays,
)
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.gui import (
    ACTIVE_GROUP_COLOR,
    INACTIVE_GROUP_COLOR,
    ViserPanelGui,
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
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


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
    model_path: Path | str = "robot.urdf"
    package_paths: dict[str, str] | None = None
    xacro_args: dict[str, str] | None = None
    auto_convert_meshes: bool = False
    max_velocity: float = 1.0
    max_acceleration: float = 1.0
    joint_name_mapping: dict[str, str] | None = None
    coordinator_task_name: str | None = None
    pre_grasp_offset: float = 0.0

    def __post_init__(self) -> None:
        if isinstance(self.model_path, str):
            self.model_path = Path(self.model_path)


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
        self.cancelled = 0
        self.cleared = 0
        self.last_plan: GeneratedPlan | None = None

    def make_plan(self, group_ids: tuple[str, ...]) -> GeneratedPlan:
        names = [
            name for group in self.groups for name in group.joint_names if group.id in group_ids
        ]
        if not names:
            names = ["robot/j1", "robot/j2"]
        plan = GeneratedPlan(
            group_ids=group_ids,
            trajectory=JointTrajectory(
                joint_names=names,
                points=[
                    TrajectoryPoint(0.0, [0.0] * len(names)),
                    TrajectoryPoint(1.0, [1.0] * len(names)),
                ],
            ),
            path=[JointState({"name": names, "position": [0.0] * len(names)})],
            status=PlanningStatus.SUCCESS,
        )
        self.last_plan = plan
        return plan

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

    def get_init_joints(self, name: str) -> JointState:
        return JointState({"name": self.configs[name].joint_names, "position": [-0.5, -1.0]})

    def get_state(self) -> str:
        return "IDLE"

    def get_error(self) -> str:
        return ""

    def reset(self) -> SimpleNamespace:
        return SimpleNamespace(is_success=lambda: True)

    def plan_to_joint_targets(self, targets: dict[str, JointState]) -> bool:
        self.plans.append((tuple(targets), targets))
        return True

    def preview_plan(self) -> bool:
        return True

    def execute(self) -> bool:
        self.executions += 1
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
        self.poses: dict[str, Pose] = {}

    def get_current_joint_state(self, robot_id: str) -> JointState:
        return JointState(self.module.states[robot_id.removeprefix("id-")])

    def is_state_stale(self, robot_id: str, max_age: float = 1.0) -> bool:
        return robot_id in self.stale

    def is_state_valid(self, robot_id: str, _state: JointState) -> bool:
        return robot_id not in self.invalid

    def get_group_ee_pose(self, group_id: str, _state: JointState | None = None) -> Pose:
        return self.poses.get(
            group_id,
            Pose({"position": [0.1, 0.2, 0.3], "orientation": [0.0, 0.0, 0.0, 1.0]}),
        )


class Operator:
    def __init__(self, module: Module, monitor: Monitor) -> None:
        self.module = module
        self.monitor = monitor

    def status(self) -> SimpleNamespace:
        return SimpleNamespace(
            state=self.module.get_state(),
            error=self.module.get_error(),
            has_plan=True,
        )

    def get_init_joints(self, robot_name: str) -> JointState | None:
        return self.module.get_init_joints(robot_name)

    def evaluate_joint_target(self, request: object) -> TargetEvaluationResult:
        target = request.target  # type: ignore[attr-defined]
        group_ids = request.group_ids  # type: ignore[attr-defined]
        diagnostics = {
            group_id: "Target is collision-free for this robot" for group_id in group_ids
        }
        poses = {group_id: self.monitor.get_group_ee_pose(group_id) for group_id in group_ids}
        return TargetEvaluationResult(
            True,
            "FEASIBLE",
            "Target is collision-free for each robot",
            True,
            tuple(group_ids),
            target,
            diagnostics,
            poses,
        )

    def evaluate_pose_target(self, request: object) -> TargetEvaluationResult:
        group_ids = tuple(
            dict.fromkeys((*request.pose_targets.keys(), *request.auxiliary_group_ids))
        )  # type: ignore[attr-defined]
        js = JointState(
            {
                "name": [
                    name
                    for group in self.module.groups
                    for name in group.joint_names
                    if group.id in group_ids
                ],
                "position": [
                    0.7
                    for group in self.module.groups
                    for _ in group.joint_names
                    if group.id in group_ids
                ],
            }
        )
        return TargetEvaluationResult(
            True,
            "FEASIBLE",
            "ok",
            True,
            group_ids,
            js,
            {},
            {group_id: self.monitor.get_group_ee_pose(group_id) for group_id in group_ids},
        )

    def plan_to_joints(self, request: object) -> GeneratedPlan:
        self.module.plan_to_joint_targets(
            {group_id: JointState({"name": [], "position": []}) for group_id in request.group_ids}
        )  # type: ignore[attr-defined]
        return self.module.make_plan(tuple(request.group_ids))  # type: ignore[attr-defined]

    def plan_to_pose(self, request: object) -> GeneratedPlan:
        return self.module.make_plan(tuple(request.pose_targets))  # type: ignore[attr-defined]

    def preview(self, plan: GeneratedPlan, duration: float | None = None) -> bool:
        _ = plan, duration
        return self.module.preview_plan()

    def execute(self, plan: GeneratedPlan) -> bool:
        _ = plan
        return self.module.execute()

    def cancel(self) -> bool:
        return self.module.cancel()

    def clear_plan(self) -> bool:
        return self.module.clear_planned_path()

    def reset(self) -> bool:
        result = self.module.reset()
        return result.is_success()


def session_inputs(module: Module) -> tuple[PlanningSceneInfo, Operator, dict[str, JointState]]:
    monitor = Monitor(module)
    robots = {f"id-{name}": config for name, config in module.configs.items()}
    current = {f"id-{name}": JointState(state) for name, state in module.states.items()}
    return (
        PlanningSceneInfo(robots=robots, planning_groups=tuple(module.groups)),
        Operator(module, monitor),
        current,
    )


def scene_gui(module: Module, server: Server, scene: ViserManipulationScene) -> ViserPanelGui:
    scene_info, operator, current = session_inputs(module)
    return ViserPanelGui(server, scene_info, operator, current, ViserVisualizationConfig(), scene)


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
        scene_info, operator, current = session_inputs(module)
        gui = ViserPanelGui(
            server, scene_info, operator, current, ViserVisualizationConfig(panel_enabled=True)
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
        == "### Planning Groups\nActive planning groups for pose goals, planning, and joint edits."
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


def test_gui_target_ghost_states_use_exact_group_names() -> None:
    left, right = group("left", "manipulator", ("j1",)), group("right", "manipulator", ("j1",))
    module = Module([left, right], states("left", "right"))
    scene_info, operator, current = session_inputs(module)
    gui = ViserPanelGui(Server(), scene_info, operator, current, ViserVisualizationConfig())
    gui.state.selected_group_ids = (left.id, right.id)
    targets = {
        left.id: JointState({"name": ["left/j1"], "position": [0.7]}),
        right.id: JointState({"name": ["right/j1"], "position": [0.8]}),
    }
    ghost_states = gui._target_ghost_states(targets)
    assert ghost_states["left"].position == [0.7, 0.2]
    assert ghost_states["right"].position == [0.8, 0.2]
    assert gui.evaluate_joint_target_set((left.id, right.id), targets).status == "FEASIBLE"


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
    gui._apply_target_evaluation_result(request, TargetEvaluationResult(True, "FEASIBLE", "", True))
    assert gui.state.target_status == TargetStatus.EMPTY
    gui.state.latest_sequence_id = 1
    gui.state.advance_selection_epoch()
    gui._apply_target_evaluation_result(request, TargetEvaluationResult(True, "FEASIBLE", "", True))
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
    assert gui.state.plan_state.group_ids == (left.id, right.id)
    gui.state.next_sequence_id()
    assert gui.state.plan_state.status == PlanStatus.STALE
    gui.state.plan_state = PanelPlanState(
        status=PlanStatus.FRESH,
        group_ids=(left.id, right.id),
        target_sequence_id=gui.state.latest_sequence_id,
        plan=module.last_plan,
    )
    gui.state.target_status = TargetStatus.FEASIBLE
    gui._submit_execute()
    assert module.executions == 1


def test_initialization_waits_for_complete_fresh_telemetry(
    panel: Callable[..., tuple[ViserPanelGui, Module, Server]],
) -> None:
    selected = group("arm", "manipulator", ("j1", "j2"), pose=True)
    gui, module, _server = panel(
        [selected], {"arm": JointState({"name": ["j1"], "position": [0.4]})}
    )

    assert selected.id not in gui.state.group_joint_targets
    module.states["arm"] = JointState({"name": ["j1", "j2"], "position": [0.4, 0.5]})
    gui.refresh()
    assert selected.id not in gui.state.group_joint_targets

    gui.current_states["id-arm"] = module.states["arm"]
    gui.refresh()
    assert gui.state.group_joint_targets[selected.id].position == [0.4, 0.5]


def test_incomplete_preset_preserves_existing_group_targets(
    panel: Callable[..., tuple[ViserPanelGui, Module, Server]],
) -> None:
    selected = group("arm", "manipulator", ("j1", "j2"), pose=True)
    gui, module, _server = panel([selected], states("arm"))
    before = JointState(gui.state.group_joint_targets[selected.id])
    module.get_init_joints = lambda name: JointState({"name": ["j1"], "position": [-0.5]})

    gui._apply_preset("Init")

    assert gui.state.group_joint_targets[selected.id] == before
    assert "missing joints" in gui.state.error


def test_valid_init_preset_builds_sliders_after_incomplete_initial_telemetry(
    panel: Callable[..., tuple[ViserPanelGui, Module, Server]],
) -> None:
    selected = group("arm", "manipulator", ("j1", "j2"), pose=True)
    gui, module, server = panel(
        [selected], {"arm": JointState({"name": ["j1"], "position": [0.4]})}
    )

    assert gui.state.group_joint_targets == {}
    assert server.gui.sliders == []

    module.configs["arm"].home_joints = [-0.5, -1.0]
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
    module.get_init_joints = lambda name: JointState(
        {"name": ["j1"] if name == "left" else [], "position": [-0.5] if name == "left" else []}
    )

    gui._apply_preset("Init")

    assert gui.state.group_joint_targets == before
    assert "missing joints" in gui.state.error


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
        TargetEvaluationRequest(0, "joints"), TargetEvaluationResult(True, "FEASIBLE", "", True)
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
    scene = ViserManipulationScene(server, Urdf)
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
                    PreviewFrame(0.0, (0.0, 0.2)),
                    PreviewFrame(1.0, (1.0, 0.2)),
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
    scene = ViserManipulationScene(server, Urdf)
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


def test_init_and_home_presets_use_operator_init_and_config_home(
    panel: Callable[..., tuple[ViserPanelGui, Module, Server]],
) -> None:
    selected = group("arm", "manipulator", ("j1", "j2"), pose=True)
    gui, module, _server = panel([selected], states("arm"))
    module.configs["arm"].home_joints = [0.9, 0.8]

    gui._apply_preset("Init")
    assert gui.state.group_joint_targets[selected.id].position == [-0.5, -1.0]

    gui._apply_preset("Home")
    assert gui.state.group_joint_targets[selected.id].position == [0.9, 0.8]


def test_initial_pose_targets_are_group_id_keyed_for_same_robot_groups() -> None:
    first = group("arm", "wrist", ("j1",), pose=True)
    second = group("arm", "tool", ("j2",), pose=True)
    module = Module([first, second], states("arm"))
    monitor = Monitor(module)
    monitor.poses[first.id] = Pose(
        {"position": [1.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]}
    )
    monitor.poses[second.id] = Pose(
        {"position": [2.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]}
    )
    server = Server()
    scene_info = PlanningSceneInfo(
        robots={"id-arm": module.configs["arm"]}, planning_groups=tuple(module.groups)
    )
    current = {"id-arm": JointState(module.states["arm"])}
    gui = ViserPanelGui(
        server,
        scene_info,
        Operator(module, monitor),
        current,
        ViserVisualizationConfig(panel_enabled=True),
    )
    try:
        gui.start()
        gui._toggle_group_selected(second.id)

        assert list(gui.state.pose_targets[first.id].position) == [1.0, 0.0, 0.0]
        assert list(gui.state.pose_targets[second.id].position) == [2.0, 0.0, 0.0]
    finally:
        gui.close()


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

    gui._apply_target_evaluation_result(request, TargetEvaluationResult(True, "FEASIBLE", "", True))

    assert gui.state.target_status == TargetStatus.CHECKING


def test_scene_target_ghost_tracks_current_only_until_explicit_target() -> None:
    server = Server()
    server.scene.add_grid = lambda *_args, **_kwargs: Handle()
    scene = ViserManipulationScene(server, Urdf)
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
    scene = ViserManipulationScene(server, Urdf)
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
    scene = ViserManipulationScene(server, Urdf)
    scene.prepared_urdf_path = lambda _config: "robot.urdf"  # type: ignore[method-assign]
    scene.register_robot("id-arm", module.configs["arm"])
    scene.register_robot("id-other", module.configs["other"])
    gui = scene_gui(module, server, scene)
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

    gui._apply_target_evaluation_result(request, TargetEvaluationResult(True, "FEASIBLE", "", True))

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
        request, TargetEvaluationResult(True, "COLLISION", "", False)
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


def test_scene_shared_clock_uses_stored_unequal_robot_frames(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    server = Server()
    server.scene.add_grid = lambda *_args, **_kwargs: Handle()
    scene = ViserManipulationScene(server, Urdf)
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
                PreviewTrack("left", ("j1",), (PreviewFrame(0.0, (0.0,)),)),
                PreviewTrack(
                    "right",
                    ("j1",),
                    (
                        PreviewFrame(0.0, (10.0,)),
                        PreviewFrame(1.0, (11.0,)),
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
        ("right", [11.0]),
    ]


def test_animation_frame_helpers_scale_stored_timestamps() -> None:
    frames = (
        PreviewFrame(0.0, (0.0,)),
        PreviewFrame(0.25, (1.0,)),
        PreviewFrame(1.0, (2.0,)),
    )

    assert scaled_frame_delays(frames, 2.0) == (0.5, 1.5)


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
    scene = ViserManipulationScene(server, Urdf)

    assert scene.has_reference_grid() is True
    scene.set_reference_grid_visible(False)
    assert grids[0].visible is False
    scene.set_reference_grid_visible(True)
    assert grids[0].visible is True


def test_scene_returns_false_for_missing_robot_target_updates() -> None:
    server = Server()
    server.scene.add_grid = lambda *_args, **_kwargs: Handle()
    scene = ViserManipulationScene(server, Urdf)

    assert scene.set_target_joints("missing", ["j1"], [0.1]) is False
    assert (
        scene.animate_preview(
            GroupPreviewAnimation(
                (PreviewTrack("missing", ("j1",), (PreviewFrame(0.0, (0.0,)),)),)
            ),
            duration=0.0,
        )
        is False
    )


def test_scene_cancel_generation_hides_preview_and_rejects_old_animation() -> None:
    server = Server()
    server.scene.add_grid = lambda *_args, **_kwargs: Handle()
    scene = ViserManipulationScene(server, Urdf)
    scene.prepared_urdf_path = lambda _config: "robot.urdf"  # type: ignore[method-assign]
    config = Config("arm", ["j1", "j2"], [-1.0, -2.0], [1.0, 2.0], [0.0, 0.0])
    scene.register_robot("id-arm", config)
    scene._preview_visible["id-arm"] = True
    scene._set_preview_visibility("id-arm", True)

    scene.cancel_preview_animation()

    assert scene._preview_visible == {"id-arm": False}
    assert scene._animation_generation == 1


def test_scene_base_pose_requires_urdf_root_to_match(monkeypatch: pytest.MonkeyPatch) -> None:
    server = Server()
    server.scene.add_grid = lambda *_args, **_kwargs: Handle()
    scene = ViserManipulationScene(server, Urdf)
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


@pytest.mark.parametrize("interruption", ["cancel", "replacement", "close"])
def test_scene_inflight_preview_never_updates_after_generation_replacement(
    monkeypatch: pytest.MonkeyPatch, interruption: str
) -> None:
    server = Server()
    server.scene.add_grid = lambda *_args, **_kwargs: Handle()
    scene = ViserManipulationScene(server, Urdf)
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
                    PreviewFrame(0.0, (0.0,)),
                    PreviewFrame(1.0, (1.0,)),
                ),
            ),
        )
    )
    worker = threading.Thread(target=lambda: scene.animate_preview(old, 1.0))
    worker.start()
    assert first_tick.wait(timeout=2.0)
    if interruption == "cancel":
        visualizer = ViserManipulationVisualizer(
            config=ViserVisualizationConfig(panel_enabled=False),
        )
        visualizer._scene = scene
        visualizer.cancel_preview_animation()
        stable_updates = list(updates)
    elif interruption == "replacement":
        updates.clear()
        assert scene.animate_preview(
            GroupPreviewAnimation(
                (PreviewTrack("id-arm", ("j1",), (PreviewFrame(0.0, (10.0,)),)),)
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
    scene = ViserManipulationScene(server, Urdf)
    gui = scene_gui(module, server, scene)
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
    scene = ViserManipulationScene(server, Urdf)
    gui = scene_gui(module, server, scene)
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
        TargetEvaluationResult(
            True, "FEASIBLE", "", True, group_poses={selected.id: computed_pose}
        ),
    )

    assert control.position == (0.7, 0.8, 0.9)
    assert control.wxyz == (0.4, 0.1, 0.2, 0.3)
    gui.close()
