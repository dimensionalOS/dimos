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

from collections.abc import Iterator
import threading
import time
from types import SimpleNamespace
from typing import Any, cast

import numpy as np
import pytest

from dimos.manipulation.planning.spec.models import PlanningSceneInfo
from dimos.manipulation.visualization.viser import (
    theme as theme_module,
    visualizer as visualizer_module,
)
from dimos.manipulation.visualization.viser.adapter import InProcessViserAdapter
from dimos.manipulation.visualization.viser.animation import sampled_joint_path_frames
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.gui import ViserPanelGui
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
from dimos.manipulation.visualization.viser.visualizer import ViserManipulationVisualizer
from dimos.msgs.geometry_msgs.Pose import Pose


class FakeHandle:
    def __init__(self) -> None:
        self.visible = None
        self.removed = False

    def remove(self) -> None:
        self.removed = True


class FakeUrdf:
    def __init__(self, names: tuple[str, ...]) -> None:
        self._urdf = SimpleNamespace(actuated_joint_names=names)
        self._meshes = []
        self.cfg = None

    def update_cfg(self, cfg) -> None:
        self.cfg = list(cfg)


class FakeJointState:
    def __init__(self, name, position=None, velocity=None, effort=None) -> None:
        self.name = list(name)
        self.position = list(position or [])
        self.velocity = list(velocity or [])
        self.effort = list(effort or [])


class FakeServer:
    def __init__(self) -> None:
        self.scene = SimpleNamespace()


class FakeGridServer(FakeServer):
    def __init__(self) -> None:
        super().__init__()
        self.grids = []
        self.scene.add_grid = self.add_grid

    def add_grid(self, name, **kwargs):
        handle = SimpleNamespace(name=name, kwargs=kwargs, visible=kwargs.get("visible"))
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

    def on_update(self, callback) -> None:
        self.update_callback = callback


class FakeTransformServer(FakeServer):
    def __init__(self) -> None:
        super().__init__()
        self.transform_controls = []
        self.scene.add_transform_controls = self.add_transform_controls

    def add_transform_controls(self, path, *, scale):
        handle = FakeTransformHandle()
        handle.path = path
        handle.scale = scale
        self.transform_controls.append(handle)
        return handle


class FakeFolder:
    def __init__(self, label, kwargs) -> None:
        self.label = label
        self.kwargs = kwargs
        self.entered = False
        self.exited = False

    def __enter__(self):
        self.entered = True
        return self

    def __exit__(self, *_args):
        self.exited = True
        return False


class FakeGuiServer:
    def __init__(self) -> None:
        self.theme_kwargs = None
        self.folders = []
        self.gui = SimpleNamespace(
            add_markdown=lambda value: SimpleNamespace(value=value),
            add_dropdown=self.add_dropdown,
            add_button=self.add_button,
            add_checkbox=self.add_checkbox,
            add_slider=self.add_slider,
            add_folder=self.add_folder,
            configure_theme=self.configure_theme,
        )
        self.buttons = {}
        self.checkboxes = {}
        self.sliders = []

    def configure_theme(self, **kwargs: Any) -> None:
        self.theme_kwargs = kwargs

    def add_folder(self, label, **kwargs):
        handle = FakeFolder(label, kwargs)
        self.folders.append(handle)
        return handle

    def add_dropdown(self, label, *, options, initial_value):
        handle = SimpleNamespace(
            label=label,
            options=options,
            value=initial_value,
            on_update=lambda callback: setattr(handle, "update_callback", callback),
        )
        return handle

    def add_button(self, label):
        handle = SimpleNamespace(
            label=label,
            on_click=lambda callback: setattr(handle, "click_callback", callback),
        )
        self.buttons[label] = handle
        return handle

    def add_checkbox(self, label, *, initial_value):
        handle = SimpleNamespace(
            label=label,
            value=initial_value,
            on_update=lambda callback: setattr(handle, "update_callback", callback),
        )
        self.checkboxes[label] = handle
        return handle

    def add_slider(self, label, *, min, max, step, initial_value):
        handle = SimpleNamespace(
            label=label,
            min=min,
            max=max,
            step=step,
            value=initial_value,
            removed=False,
        )
        handle.remove = lambda: setattr(handle, "removed", True)
        self.sliders.append(handle)
        return handle


def make_robot_config(**overrides: Any) -> SimpleNamespace:
    """Build a faithful RobotModelConfig stand-in with the fields the panel reads."""
    fields: dict[str, Any] = {
        "name": "arm",
        "joint_names": ["j1", "j2"],
        "end_effector_link": "ee_link",
        "base_link": "base_link",
        "home_joints": None,
        "joint_limits_lower": None,
        "joint_limits_upper": None,
    }
    fields.update(overrides)
    return SimpleNamespace(**fields)


def make_adapter_with_robot() -> InProcessViserAdapter:
    current = FakeJointState(["j1", "j2"], position=[0.3, 0.4])
    config = make_robot_config(
        name="arm",
        joint_names=["j1", "j2"],
        joint_limits_lower=[-1.0, -2.0],
        joint_limits_upper=[1.0, 2.0],
        home_joints=[0.0, 0.0],
    )
    module = SimpleNamespace(
        _robots={"arm": ("robot-1", config, None)},
        _init_joints={"arm": FakeJointState(["j1", "j2"], position=[0.1, 0.2])},
        _state=SimpleNamespace(name="IDLE"),
        _error_message="",
    )
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda _robot_id: current,
        is_state_stale=lambda _robot_id, max_age=1.0: False,
        get_ee_pose=lambda _robot_id, joint_state=None: None,
    )
    return InProcessViserAdapter(
        world_monitor=cast("Any", world_monitor),
        manipulation_module=cast("Any", module),
    )


@pytest.fixture
def make_panel() -> Iterator[Any]:
    """Build and start a ViserPanelGui, closing it (and its worker threads) on teardown."""
    panels: list[ViserPanelGui] = []

    def _make(
        server: Any,
        adapter: InProcessViserAdapter,
        config: ViserVisualizationConfig | None = None,
        scene: Any = None,
    ) -> ViserPanelGui:
        gui = ViserPanelGui(
            server, adapter, config or ViserVisualizationConfig(panel_enabled=True), scene
        )
        gui.start()
        panels.append(gui)
        return gui

    yield _make
    for gui in panels:
        gui.close()


def test_viser_config_enables_panel_by_default() -> None:
    assert ViserVisualizationConfig().panel_enabled is True


def test_gui_builds_controls_in_manipulation_panel_folder(make_panel: Any) -> None:
    server = FakeGuiServer()
    adapter = make_adapter_with_robot()
    gui = make_panel(server, adapter, ViserVisualizationConfig())
    assert server.folders
    assert server.folders[0].label == "Manipulation Panel"
    assert server.folders[0].kwargs == {"expand_by_default": True}
    assert "status" in gui._handles
    assert "robot" in gui._handles
    assert "plan" in gui._handles
    assert cast("Any", gui._operation_worker)._timeout_seconds is None


def test_gui_scene_grid_checkbox_toggles_reference_grid(make_panel: Any) -> None:
    grid_server = FakeGridServer()
    scene = ViserManipulationScene(
        grid_server, lambda *args, **kwargs: FakeUrdf(("joint1",)), preview_fps=10.0
    )
    server = FakeGuiServer()
    adapter = make_adapter_with_robot()
    make_panel(server, adapter, ViserVisualizationConfig(), scene)
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


def test_dimos_theme_configures_supported_viser_chrome() -> None:
    server = FakeGuiServer()

    assert apply_dimos_theme(server) is True
    assert server.theme_kwargs is not None
    assert server.theme_kwargs["brand_color"] == (22, 130, 163)
    assert server.theme_kwargs["dark_mode"] is True
    assert server.theme_kwargs["show_logo"] is False
    assert server.theme_kwargs["show_share_button"] is False
    assert server.theme_kwargs["control_layout"] == "collapsible"
    assert server.theme_kwargs["control_width"] == "medium"


def test_dimos_theme_configures_titlebar_when_supported(monkeypatch: Any) -> None:
    class FakeThemeModule:
        @staticmethod
        def TitlebarImage(**kwargs: Any) -> dict[str, Any]:
            return kwargs

        @staticmethod
        def TitlebarButton(**kwargs: Any) -> dict[str, Any]:
            return kwargs

        @staticmethod
        def TitlebarConfig(**kwargs: Any) -> dict[str, Any]:
            return kwargs

    real_import_module = theme_module.importlib.import_module

    def import_module(name: str) -> object:
        if name == "viser.theme":
            return FakeThemeModule
        return real_import_module(name)

    monkeypatch.setattr(theme_module.importlib, "import_module", import_module)
    server = FakeGuiServer()

    assert apply_dimos_theme(server) is True
    assert server.theme_kwargs is not None
    titlebar_content = cast("dict[str, Any]", server.theme_kwargs["titlebar_content"])
    image = cast("dict[str, str]", titlebar_content["image"])
    assert image["image_alt"] == "Dimensional"
    assert image["image_url_light"].startswith("data:image/svg+xml;base64,")


def test_dimos_logo_asset_loads_as_data_url() -> None:
    logo_url = _dimos_logo_data_url()

    assert logo_url is not None
    assert logo_url.startswith("data:image/svg+xml;base64,")


def test_dimos_theme_is_non_blocking_when_theme_api_fails() -> None:
    class BrokenGui:
        @staticmethod
        def configure_theme(**_kwargs: Any) -> None:
            raise TypeError("theme API changed")

    server = SimpleNamespace(gui=BrokenGui())

    assert apply_dimos_theme(server) is False


def test_visualizer_initializes_all_scene_robots_from_planning_scene() -> None:
    calls = []
    fake_scene = SimpleNamespace(
        register_robot=lambda robot_id, config: calls.append((robot_id, config.name))
    )
    fake_gui = SimpleNamespace(refresh=lambda: calls.append(("refresh", "gui")))
    visualizer = ViserManipulationVisualizer.__new__(ViserManipulationVisualizer)
    cast("Any", visualizer)._closed = False
    cast("Any", visualizer)._scene = fake_scene
    cast("Any", visualizer)._gui = fake_gui
    scene = PlanningSceneInfo(
        robots={
            "robot-1": SimpleNamespace(name="arm1"),
            "robot-2": SimpleNamespace(name="arm2"),
        }
    )

    visualizer.initialize_scene(scene)

    assert calls == [("robot-1", "arm1"), ("robot-2", "arm2"), ("refresh", "gui")]


def test_visualizer_closes_runtime_when_construction_fails() -> None:
    closed = []

    class FakeRuntime:
        url = "http://localhost:8095"

        def __init__(self, config) -> None:
            self.config = config

        def start(self) -> FakeServer:
            return FakeServer()

        def close(self) -> None:
            closed.append("runtime")

    def fail_import_viser_urdf():
        raise RuntimeError("missing viser_urdf")

    original_runtime = visualizer_module.ViserRuntime
    original_import = visualizer_module.import_viser_urdf
    cast("Any", visualizer_module).ViserRuntime = FakeRuntime
    cast("Any", visualizer_module).import_viser_urdf = fail_import_viser_urdf
    try:
        try:
            ViserManipulationVisualizer(
                world_monitor=cast("Any", object()),
                manipulation_module=None,
                config=ViserVisualizationConfig(panel_enabled=False),
            )
        except RuntimeError as e:
            assert str(e) == "missing viser_urdf"
        else:
            raise AssertionError("expected Viser construction failure")
        assert closed == ["runtime"]
    finally:
        cast("Any", visualizer_module).ViserRuntime = original_runtime
        cast("Any", visualizer_module).import_viser_urdf = original_import


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

    def update_cfg(self, cfg) -> None:
        self.cfg = list(cfg)


def test_viser_joint_configuration_maps_names_to_urdf_order() -> None:
    server = FakeServer()
    urdf = FakeUrdf(("shoulder", "elbow", "wrist"))
    scene = ViserManipulationScene(server, lambda *args, **kwargs: urdf, preview_fps=10.0)
    scene.prepared_urdf_path = lambda config: cast("Any", "dummy.urdf")

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
    scene.prepared_urdf_path = lambda config: cast("Any", "dummy.urdf")
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
    assert all(mesh.visible is True for mesh in target._meshes)
    assert all(mesh.visible is False for mesh in preview._meshes)
    scene.show_preview("robot1")
    assert all(mesh.visible is True for mesh in preview._meshes)
    assert all(mesh.visible is True for mesh in target._meshes)
    scene.hide_preview("robot1")
    assert all(mesh.visible is False for mesh in preview._meshes)
    assert all(mesh.visible is True for mesh in target._meshes)
    scene.close()
    assert scene._handles == {}
    assert all(mesh.visible is False for mesh in preview._meshes)


def test_target_ghost_is_visible_and_tracks_current_until_target_moves_it() -> None:
    server = FakeServer()
    urdfs = [FakeViserUrdfWithMeshes(("joint1",)) for _ in range(3)]
    scene = ViserManipulationScene(server, lambda *args, **kwargs: urdfs.pop(0), preview_fps=10.0)
    scene.prepared_urdf_path = lambda config: cast("Any", "dummy.urdf")
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

    assert all(mesh.visible is True for mesh in target._meshes)
    assert all(mesh.visible is False for mesh in preview._meshes)
    scene.update_current_robot("robot1", cast("Any", FakeJointState(["joint1"], position=[0.25])))
    assert current.cfg == [0.25]
    assert target.cfg == [0.25]
    assert preview.cfg is None

    scene.set_target_joints("robot1", ["joint1"], [0.8])
    scene.update_current_robot("robot1", cast("Any", FakeJointState(["joint1"], position=[0.1])))
    assert current.cfg == [0.1]
    assert target.cfg == [0.8]
    assert preview.cfg is None


def test_preview_animation_uses_separate_colored_ghost_and_hides_after_playback() -> None:
    server = FakeServer()
    urdfs = [FakeViserUrdfWithMeshes(("joint1",)) for _ in range(3)]
    scene = ViserManipulationScene(server, lambda *args, **kwargs: urdfs.pop(0), preview_fps=10.0)
    scene.prepared_urdf_path = lambda config: cast("Any", "dummy.urdf")
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
        cast(
            "Any",
            [
                FakeJointState(["joint1"], position=[0.0]),
                FakeJointState(["joint1"], position=[1.0]),
            ],
        ),
        duration=0.0,
    )

    assert ok is True
    assert preview.cfg == [1.0]
    assert all(mesh.visible is False for mesh in preview._meshes)
    assert all(mesh.visible is True for mesh in target._meshes)


def test_sampled_joint_path_frames_preserves_dense_trajectory_samples() -> None:
    dense_path = [FakeJointState(["j1"], position=[float(index)]) for index in range(32)]

    frames = sampled_joint_path_frames(cast("Any", dense_path), duration=1.0, fps=30.0)

    assert frames == [[float(index)] for index in range(32)]


def test_sampled_joint_path_frames_interpolates_sparse_paths() -> None:
    sparse_path = [
        FakeJointState(["j1"], position=[0.0]),
        FakeJointState(["j1"], position=[1.0]),
    ]

    frames = sampled_joint_path_frames(cast("Any", sparse_path), duration=1.0, fps=4.0)

    assert frames == [[0.0], [0.25], [0.5], [0.75], [1.0]]


def test_adapter_copies_joint_state_and_delegates_to_module() -> None:
    copied = FakeJointState(["j1"], position=[1.0], velocity=[2.0], effort=[3.0])
    module = SimpleNamespace(
        _robots={"arm": ("robot-1", SimpleNamespace(), None)},
        _planned_paths={"arm": [copied]},
        _planned_trajectories={},
        plan_to_pose=lambda pose, robot_name=None: (pose, robot_name),
        plan_to_joints=lambda joints, robot_name=None: (joints, robot_name),
        preview_path=lambda robot_name=None: robot_name,
        execute=lambda robot_name=None: robot_name,
        cancel=lambda: True,
        clear_planned_path=lambda: True,
    )
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: copied,
        is_state_stale=lambda robot_id, max_age=1.0: False,
        is_state_valid=lambda robot_id, joint_state: True,
        get_ee_pose=lambda robot_id, joint_state=None: (robot_id, joint_state),
    )
    adapter = InProcessViserAdapter(
        world_monitor=cast("Any", world_monitor), manipulation_module=cast("Any", module)
    )

    planned = cast("Any", adapter.get_planned_path("arm"))
    assert planned is not None
    assert planned[0] is not copied
    assert planned[0].name is not copied.name
    assert planned[0].position is not copied.position

    current = cast("Any", adapter.get_current_joint_state("arm"))
    assert current is not copied
    assert current.name is not copied.name

    assert adapter.plan_to_pose(cast("Any", "pose"), "arm") == ("pose", "arm")
    assert adapter.preview_path("arm") == "arm"
    assert adapter.evaluate_joint_target(planned[0], "arm")["status"] == "FEASIBLE"


def test_adapter_evaluate_joint_target_uses_world_monitor_and_copies_input() -> None:
    original = FakeJointState(["arm/j1", "j2"], position=[1.0, 2.0])
    seen = {}

    def is_state_valid(robot_id, joint_state) -> bool:
        seen["robot_id"] = robot_id
        seen["joint_state"] = joint_state
        return True

    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: None,
        is_state_stale=lambda robot_id, max_age=1.0: False,
        is_state_valid=is_state_valid,
        get_ee_pose=lambda robot_id, joint_state=None: (robot_id, joint_state),
    )
    module = SimpleNamespace(_robots={"arm": ("robot-1", SimpleNamespace(), None)})
    adapter = InProcessViserAdapter(
        world_monitor=cast("Any", world_monitor), manipulation_module=cast("Any", module)
    )

    result = adapter.evaluate_joint_target(cast("Any", original), "arm")

    assert result["success"] is True
    assert result["status"] == "FEASIBLE"
    assert seen["robot_id"] == "robot-1"
    assert seen["joint_state"] is not original
    assert seen["joint_state"].name == ["arm/j1", "j2"]
    assert seen["joint_state"].position == [1.0, 2.0]


def test_obstacle_collision_marks_joint_target_infeasible() -> None:
    obstacle = SimpleNamespace(name="blocking_box", blocked_joint_min=0.5)

    def is_state_valid(robot_id, joint_state) -> bool:
        return bool(joint_state.position[0] < obstacle.blocked_joint_min)

    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: FakeJointState(["j1"], position=[0.0]),
        is_state_stale=lambda robot_id, max_age=1.0: False,
        is_state_valid=is_state_valid,
        get_ee_pose=lambda robot_id, joint_state=None: SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0)
        ),
    )
    module = SimpleNamespace(
        _robots={"arm": ("robot-1", SimpleNamespace(joint_names=["j1"]), None)}
    )
    adapter = InProcessViserAdapter(
        world_monitor=cast("Any", world_monitor), manipulation_module=cast("Any", module)
    )

    free = adapter.evaluate_joint_target(FakeJointState(["j1"], position=[0.25]), "arm")
    colliding = adapter.evaluate_joint_target(FakeJointState(["j1"], position=[0.75]), "arm")

    assert free["success"] is True
    assert free["status"] == "FEASIBLE"
    assert free["collision_free"] is True
    assert colliding["success"] is True
    assert colliding["status"] == "COLLISION"
    assert colliding["collision_free"] is False


def test_scene_registers_goal_robot_coloring_and_updates_visibility() -> None:
    server = FakeServer()
    scene = ViserManipulationScene(
        server,
        lambda *args, **kwargs: FakeViserUrdfWithMeshes(("joint1", "joint2")),
        preview_fps=10.0,
    )
    scene.prepared_urdf_path = lambda config: cast("Any", "dummy.urdf")
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
    assert all(mesh.visible is True for mesh in target._meshes)


def test_scene_transform_controls_update_pose_callback_and_visual_state() -> None:
    server = FakeTransformServer()
    scene = ViserManipulationScene(
        server,
        lambda *args, **kwargs: FakeViserUrdfWithMeshes(("joint1", "joint2")),
        preview_fps=10.0,
    )
    scene.prepared_urdf_path = lambda config: cast("Any", "dummy.urdf")
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
    scene.prepared_urdf_path = lambda config: cast("Any", "dummy.urdf")
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


def test_gui_initializes_pose_selector_to_current_ee_pose(make_panel: Any) -> None:
    current = FakeJointState(["j1"], position=[0.25])
    current_pose = SimpleNamespace(
        position=SimpleNamespace(x=0.1, y=0.2, z=0.3),
        orientation=SimpleNamespace(w=0.9, x=0.1, y=0.2, z=0.3),
    )
    config = make_robot_config(joint_names=["j1"], home_joints=[0.0])
    module = SimpleNamespace(
        _robots={"arm": ("robot-1", config, None)}, _planned_paths={}, _planned_trajectories={}
    )
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: current,
        is_state_stale=lambda robot_id, max_age=1.0: False,
        get_ee_pose=lambda robot_id, joint_state=None: current_pose,
    )
    adapter = InProcessViserAdapter(
        world_monitor=cast("Any", world_monitor), manipulation_module=cast("Any", module)
    )
    scene = ViserManipulationScene(
        FakeTransformServer(), lambda *args, **kwargs: FakeViserUrdfWithMeshes(), preview_fps=10.0
    )
    gui = make_panel(FakeGuiServer(), adapter, ViserVisualizationConfig(panel_enabled=True), scene)
    control = scene._handles["robot-1:ee_control"]
    assert control.position == (0.1, 0.2, 0.3)
    assert control.wxyz == (0.9, 0.1, 0.2, 0.3)
    assert gui.state.cartesian_target is current_pose


def test_gui_preset_dropdown_and_controls_include_init_home_current_and_callbacks(
    make_panel: Any,
) -> None:
    current = FakeJointState(["arm/j1", "arm/j2"], position=[0.25, 0.5])
    config = make_robot_config(joint_names=["j1", "j2"], home_joints=[1.0, 2.0])
    module = SimpleNamespace(
        _robots={"arm": ("robot-1", config, None)},
        _init_joints={"arm": FakeJointState(["j1", "j2"], position=[-1.0, -2.0])},
        _planned_paths={},
        _planned_trajectories={},
    )
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: current,
        is_state_stale=lambda robot_id, max_age=1.0: False,
        is_state_valid=lambda robot_id, joint_state: True,
        get_ee_pose=lambda robot_id, joint_state=None: None,
    )
    adapter = InProcessViserAdapter(
        world_monitor=cast("Any", world_monitor), manipulation_module=cast("Any", module)
    )
    gui = make_panel(FakeGuiServer(), adapter)
    assert gui._handles["preset"].options == ["Select preset...", "Init", "Current", "Home"]
    assert list(gui._joint_sliders) == ["j1", "j2"]
    gui._apply_preset("Home")
    assert [gui._joint_sliders[name].value for name in ("j1", "j2")] == [1.0, 2.0]
    gui._apply_preset("Current")
    assert [gui._joint_sliders[name].value for name in ("j1", "j2")] == [0.25, 0.5]
    gui._submit_execute()
    assert gui.state.error == "Panel execution disabled; set allow_plan_execute=True to enable"


def test_gui_rebuilding_joint_sliders_removes_stale_viser_handles(make_panel: Any) -> None:
    current = FakeJointState(["j1", "j2"], position=[0.0, 0.0])
    config = make_robot_config(joint_names=["j1", "j2"], home_joints=[1.0, 2.0])
    module = SimpleNamespace(
        _robots={"arm": ("robot-1", config, None)}, _planned_paths={}, _planned_trajectories={}
    )
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: current,
        is_state_stale=lambda robot_id, max_age=1.0: False,
        is_state_valid=lambda robot_id, joint_state: True,
        get_ee_pose=lambda robot_id, joint_state=None: None,
    )
    adapter = InProcessViserAdapter(
        world_monitor=cast("Any", world_monitor), manipulation_module=cast("Any", module)
    )
    server = FakeGuiServer()
    gui = make_panel(server, adapter)
    stale_sliders = list(server.sliders)
    assert [slider.value for slider in stale_sliders] == [0.0, 0.0]

    current.position = [-0.738, -0.2826151825863572]
    gui._build_joint_sliders()

    assert all(slider.removed is True for slider in stale_sliders)
    assert [gui._joint_sliders[name].value for name in ("j1", "j2")] == [
        -0.738,
        -0.2826151825863572,
    ]


def test_gui_parses_numpy_transform_control_arrays() -> None:
    gui = ViserPanelGui(cast("Any", object()), cast("Any", object()), ViserVisualizationConfig())

    pose = gui._pose_from_transform_target(
        SimpleNamespace(
            position=np.array([1.0, 2.0, 3.0]),
            wxyz=np.array([0.5, 0.1, 0.2, 0.3]),
        )
    )

    assert pose is not None
    assert list(pose.position) == [1.0, 2.0, 3.0]
    assert list(pose.orientation) == [0.1, 0.2, 0.3, 0.5]


def test_panel_execution_is_gated_by_default_and_refresh_updates_robot_controls(
    make_panel: Any,
) -> None:
    current = FakeJointState(["j1"], position=[1.2])
    config = make_robot_config(joint_names=["j1"], home_joints=[0.5])
    module = SimpleNamespace(
        _robots={"arm": ("robot-1", config, None)},
        _planned_paths={},
        _planned_trajectories={},
        execute=lambda robot_name=None: False,
    )
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: current,
        is_state_stale=lambda robot_id, max_age=1.0: False,
        is_state_valid=lambda robot_id, joint_state: True,
        get_ee_pose=lambda robot_id, joint_state=None: None,
    )
    adapter = InProcessViserAdapter(
        world_monitor=cast("Any", world_monitor),
        manipulation_module=cast("Any", module),
    )
    gui = make_panel(FakeGuiServer(), adapter)
    gui.refresh()
    assert gui.state.selected_robot == "arm"
    assert list(gui._joint_sliders) == ["j1"]
    gui._apply_preset("Home")
    assert gui._joint_sliders["j1"].value == 0.5

    gui._submit_execute()
    assert "Panel execution disabled" in gui.state.error


def test_gui_moves_joint_target_immediately_and_stores_evaluated_joint_solution(
    make_panel: Any,
) -> None:
    current = FakeJointState(["j1", "j2"], position=[0.0, 0.0])
    target_pose = SimpleNamespace(position=SimpleNamespace(x=0.2, y=0.3, z=0.4))
    config = make_robot_config(joint_names=["j1", "j2"], home_joints=[0.5, 0.6])
    module = SimpleNamespace(
        _robots={"arm": ("robot-1", config, None)}, _planned_paths={}, _planned_trajectories={}
    )
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: current,
        is_state_stale=lambda robot_id, max_age=1.0: False,
        is_state_valid=lambda robot_id, joint_state: True,
        get_ee_pose=lambda robot_id, joint_state=None: target_pose,
    )
    adapter = InProcessViserAdapter(
        world_monitor=cast("Any", world_monitor), manipulation_module=cast("Any", module)
    )
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
        FakeGuiServer(), adapter, ViserVisualizationConfig(panel_enabled=True), cast("Any", scene)
    )
    requests = []
    gui._worker.stop()
    object.__setattr__(
        gui,
        "_worker",
        SimpleNamespace(submit=lambda request: requests.append(request), stop=lambda: None),
    )
    gui._joint_sliders["j1"].value = 0.25
    gui._joint_sliders["j2"].value = 0.75
    gui._submit_joint_target_evaluation()
    assert target_updates[-1] == ("robot-1", ["j1", "j2"], [0.25, 0.75])
    assert target_pose_updates[-1] == ("robot-1", target_pose)
    assert requests[-1].source == "joints"

    stale_request = TargetEvaluationRequest(sequence_id=1, source="joints", robot_name="arm")
    fresh_request = TargetEvaluationRequest(sequence_id=2, source="joints", robot_name="arm")
    gui.state.latest_sequence_id = 2
    gui._apply_target_evaluation_result(
        stale_request,
        {
            "success": True,
            "collision_free": True,
            "joint_state": adapter.joints_from_values(["j1", "j2"], [9.0, 9.0]),
        },
    )
    assert gui.state.joint_target == [0.25, 0.75]

    gui._apply_target_evaluation_result(
        fresh_request,
        {
            "success": True,
            "collision_free": True,
            "joint_state": adapter.joints_from_values(["j1", "j2"], [1.0, 2.0]),
        },
    )
    assert gui.state.target_status == TargetStatus.FEASIBLE
    assert gui.state.feasibility.status == FeasibilityStatus.FEASIBLE
    assert gui.state.joint_target == [1.0, 2.0]
    assert [gui._joint_sliders[name].value for name in ("j1", "j2")] == [0.25, 0.75]
    assert target_updates[-1] == ("robot-1", ["j1", "j2"], [0.25, 0.75])


def test_gui_collision_evaluation_marks_target_infeasible_and_colors_scene(make_panel: Any) -> None:
    current = FakeJointState(["j1"], position=[0.0])
    config = make_robot_config(joint_names=["j1"], home_joints=[0.0])
    module = SimpleNamespace(
        _robots={"arm": ("robot-1", config, None)}, _planned_paths={}, _planned_trajectories={}
    )
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: current,
        is_state_stale=lambda robot_id, max_age=1.0: False,
        is_state_valid=lambda robot_id, joint_state: False,
        get_ee_pose=lambda robot_id, joint_state=None: SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0)
        ),
    )
    adapter = InProcessViserAdapter(
        world_monitor=cast("Any", world_monitor), manipulation_module=cast("Any", module)
    )
    visual_states = []
    scene = SimpleNamespace(
        has_reference_grid=lambda: False,
        ensure_target_controls=lambda *args: None,
        set_target_joints=lambda *args: True,
        set_target_pose=lambda *args: None,
        set_target_visual_state=lambda *args: visual_states.append(args),
    )
    gui = make_panel(
        FakeGuiServer(), adapter, ViserVisualizationConfig(panel_enabled=True), cast("Any", scene)
    )
    request = TargetEvaluationRequest(sequence_id=1, source="joints", robot_name="arm")
    gui.state.latest_sequence_id = 1
    result = adapter.evaluate_joint_target(FakeJointState(["j1"], position=[1.0]), "arm")

    gui._apply_target_evaluation_result(request, result)

    assert result["status"] == "COLLISION"
    assert gui.state.target_status == TargetStatus.INFEASIBLE
    assert gui.state.feasibility.status == FeasibilityStatus.COLLISION
    assert gui.state.error == "Target is in collision"
    assert visual_states[-1] == ("robot-1", False)


def test_gui_safe_execute_requires_fresh_matching_plan_and_clear_resets_path(
    make_panel: Any,
) -> None:
    current = FakeJointState(["j1"], position=[1.0])
    planned = [FakeJointState(["j1"], position=[1.0]), FakeJointState(["j1"], position=[2.0])]
    executed = []
    cleared = []
    module = SimpleNamespace(
        _robots={
            "arm": ("robot-1", make_robot_config(joint_names=["j1"], home_joints=[1.0]), None)
        },
        _planned_paths={"arm": planned},
        _planned_trajectories={},
        _state=SimpleNamespace(name="IDLE"),
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
    adapter = InProcessViserAdapter(
        world_monitor=cast("Any", world_monitor), manipulation_module=cast("Any", module)
    )
    gui = make_panel(
        FakeGuiServer(),
        adapter,
        ViserVisualizationConfig(
            panel_enabled=True, allow_plan_execute=True, current_match_tolerance=0.05
        ),
    )
    gui.state.target_status = TargetStatus.FEASIBLE
    gui.state.plan_state = PanelPlanState(
        status=PlanStatus.FRESH,
        robot="arm",
        start_joints_snapshot=[1.2],
        planned_path=cast("Any", planned),
    )
    gui._submit_execute()
    assert executed == []
    assert "Cannot execute" in gui.state.error

    gui.state.action_status = ActionStatus.IDLE
    gui.state.error = ""
    gui.state.plan_state.start_joints_snapshot = [1.0]
    gui._submit_execute()
    for _ in range(20):
        if executed:
            break
        time.sleep(0.01)
    assert executed == ["arm"]

    gui._submit_clear()
    for _ in range(20):
        if cleared:
            break
        time.sleep(0.01)
    assert cleared == [True]
    assert gui.state.plan_state.status == PlanStatus.NONE


def test_gui_plan_target_failure_recovers_action_state(make_panel: Any) -> None:
    adapter = make_adapter_with_robot()
    gui = make_panel(FakeGuiServer(), adapter)
    gui._operation_worker.stop()
    object.__setattr__(
        gui,
        "_operation_worker",
        SimpleNamespace(submit=lambda operation: operation(), stop=lambda timeout=2.0: None),
    )
    gui.state.selected_robot = "missing"
    gui.state.target_status = TargetStatus.FEASIBLE
    gui.state.manipulation_state = "IDLE"

    gui._submit_plan()

    assert gui.state.action_status == ActionStatus.IDLE
    assert gui.state.plan_state.status == PlanStatus.FAILED
    assert gui.state.error == "No robot config"
    assert gui.state.last_result == "plan_to_joints=False"


def test_operation_worker_coalesces_pending_requests() -> None:
    errors = []
    calls = []
    worker = OperationWorker(errors.append)
    worker.submit(lambda: calls.append("old"))
    worker.submit(lambda: calls.append("new"))

    operation = worker._requests.get_nowait()
    operation()

    assert calls == ["new"]
    assert errors == []


def test_operation_worker_stop_can_wait_for_in_flight_operation() -> None:
    errors = []
    worker = OperationWorker(errors.append)
    started = threading.Event()
    release = threading.Event()
    finished = threading.Event()

    def operation() -> None:
        started.set()
        release.wait(timeout=1.0)
        finished.set()

    worker.start()
    worker.submit(operation)
    assert started.wait(timeout=1.0)

    releaser = threading.Thread(
        target=lambda: (time.sleep(0.05), release.set()),
        name="ReleaseViserOperationTest",
    )
    releaser.start()
    worker.stop(timeout=None)
    releaser.join(timeout=1.0)

    assert finished.is_set()
    assert cast("Any", worker)._thread is None
    assert errors == []


def test_target_evaluation_worker_coalesces_pending_requests() -> None:
    worker = TargetEvaluationWorker(lambda request: {}, lambda request, result: None)
    old_request = TargetEvaluationRequest(sequence_id=1, source="joints", robot_name="arm")
    new_request = TargetEvaluationRequest(sequence_id=2, source="joints", robot_name="arm")

    worker.submit(old_request)
    worker.submit(new_request)

    assert worker._requests.get_nowait() is new_request


def test_operation_worker_reports_timeout() -> None:
    errors = []
    worker = OperationWorker(errors.append, timeout_seconds=0.01)
    worker._run_operation(lambda: time.sleep(0.1))

    assert errors == ["Operation timed out after 0.0s"]
    time.sleep(0.12)
