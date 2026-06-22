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

from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("viser", reason="Viser optional dependency is not installed")

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan, PlanningSceneInfo
from dimos.manipulation.visualization.viser import (
    runtime as runtime_module,
    visualizer as visualizer_module,
)
from dimos.manipulation.visualization.viser.animation import GroupPreviewAnimation
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.runtime import ViserRuntime
from dimos.manipulation.visualization.viser.visualizer import ViserManipulationVisualizer
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState


class FakeDependency:
    pass


class FakeViserUrdf:
    pass


class FakeServer:
    def __init__(self) -> None:
        self.scene = SimpleNamespace()


class FakeRuntimeServer(FakeServer):
    def __init__(self) -> None:
        super().__init__()
        self.stopped = False

    def stop(self) -> None:
        self.stopped = True


def fake_robot_config(name: str) -> RobotModelConfig:
    return RobotModelConfig(
        name=name,
        model_path=Path(f"{name}.urdf"),
        base_pose=PoseStamped(),
        joint_names=["joint1"],
        end_effector_link="ee_link",
    )


def test_visualizer_construction_is_lazy(monkeypatch: pytest.MonkeyPatch) -> None:
    def fail_runtime(_config: ViserVisualizationConfig) -> FakeServer:
        raise AssertionError("runtime should not start during construction")

    monkeypatch.setattr(visualizer_module, "ViserRuntime", fail_runtime)

    visualizer = ViserManipulationVisualizer(
        world_monitor=FakeDependency(),
        manipulation_module=FakeDependency(),
        config=ViserVisualizationConfig(panel_enabled=False),
    )

    assert visualizer.get_visualization_url() is None
    visualizer.close()


def test_visualizer_initializes_all_scene_robots_from_planning_scene(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls = []

    class FakeRuntime:
        url = "http://localhost:8095"

        def __init__(self, config: ViserVisualizationConfig) -> None:
            self.config = config

        def start(self) -> FakeServer:
            calls.append(("start", "runtime"))
            return FakeServer()

        def close(self) -> None:
            calls.append(("close", "runtime"))

    class FakeScene:
        def __init__(
            self,
            server: FakeServer,
            viser_urdf: type[FakeViserUrdf],
            *,
            preview_fps: float,
        ) -> None:
            calls.append(("create", "scene"))

        def register_robot(self, robot_id: str, config: RobotModelConfig) -> None:
            calls.append((robot_id, config.name))

        def close(self) -> None:
            calls.append(("close", "scene"))

    class FakeGui:
        def __init__(
            self,
            server: FakeServer,
            world_monitor: object,
            manipulation_module: object,
            config: ViserVisualizationConfig,
            scene: FakeScene,
        ) -> None:
            del world_monitor, manipulation_module, config, scene
            calls.append(("create", "gui"))

        def start(self) -> None:
            calls.append(("start", "gui"))

        def refresh(self) -> None:
            calls.append(("refresh", "gui"))

        def close(self) -> None:
            calls.append(("close", "gui"))

    monkeypatch.setattr(visualizer_module, "ViserRuntime", FakeRuntime)
    monkeypatch.setattr(visualizer_module, "ViserUrdf", FakeViserUrdf)
    monkeypatch.setattr(visualizer_module, "ViserManipulationScene", FakeScene)
    monkeypatch.setattr(visualizer_module, "ViserPanelGui", FakeGui)
    visualizer = ViserManipulationVisualizer(
        world_monitor=FakeDependency(),
        manipulation_module=FakeDependency(),
        config=ViserVisualizationConfig(panel_enabled=True),
    )
    scene = PlanningSceneInfo(
        robots={
            "robot-1": fake_robot_config("arm1"),
            "robot-2": fake_robot_config("arm2"),
        }
    )

    visualizer.initialize_scene(scene)

    assert calls == [
        ("start", "runtime"),
        ("create", "scene"),
        ("create", "gui"),
        ("start", "gui"),
        ("robot-1", "arm1"),
        ("robot-2", "arm2"),
        ("refresh", "gui"),
    ]


def test_visualizer_closes_partial_startup_when_gui_start_fails(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    closed = []

    class FakeRuntime:
        url = "http://localhost:8095"

        def __init__(self, config: ViserVisualizationConfig) -> None:
            self.config = config

        def start(self) -> FakeServer:
            return FakeServer()

        def close(self) -> None:
            closed.append("runtime")

    class FakeScene:
        def __init__(
            self,
            server: FakeServer,
            viser_urdf: type[FakeViserUrdf],
            *,
            preview_fps: float,
        ) -> None:
            pass

        def close(self) -> None:
            closed.append("scene")

    class FakeGui:
        def __init__(
            self,
            server: FakeServer,
            world_monitor: object,
            manipulation_module: object,
            config: ViserVisualizationConfig,
            scene: FakeScene,
        ) -> None:
            del world_monitor, manipulation_module, config, scene
            pass

        def start(self) -> None:
            raise RuntimeError("gui failed")

        def close(self) -> None:
            closed.append("gui")

    monkeypatch.setattr(visualizer_module, "ViserRuntime", FakeRuntime)
    monkeypatch.setattr(visualizer_module, "ViserUrdf", FakeViserUrdf)
    monkeypatch.setattr(visualizer_module, "ViserManipulationScene", FakeScene)
    monkeypatch.setattr(visualizer_module, "ViserPanelGui", FakeGui)
    visualizer = ViserManipulationVisualizer(
        world_monitor=FakeDependency(),
        manipulation_module=FakeDependency(),
        config=ViserVisualizationConfig(panel_enabled=True),
    )

    with pytest.raises(RuntimeError, match="gui failed"):
        visualizer.initialize_scene(PlanningSceneInfo(robots={}))

    assert closed == ["gui", "scene", "runtime"]
    assert visualizer.get_visualization_url() is None


def test_visualizer_closes_runtime_when_scene_creation_fails(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    closed = []

    class FakeRuntime:
        url = "http://localhost:8095"

        def __init__(self, config: ViserVisualizationConfig) -> None:
            self.config = config

        def start(self) -> FakeServer:
            return FakeServer()

        def close(self) -> None:
            closed.append("runtime")

    class FailingScene:
        def __init__(
            self,
            server: FakeServer,
            viser_urdf: type[FakeViserUrdf],
            *,
            preview_fps: float,
        ) -> None:
            raise RuntimeError("scene failed")

    monkeypatch.setattr(visualizer_module, "ViserRuntime", FakeRuntime)
    monkeypatch.setattr(visualizer_module, "ViserUrdf", FakeViserUrdf)
    monkeypatch.setattr(visualizer_module, "ViserManipulationScene", FailingScene)
    visualizer = ViserManipulationVisualizer(
        world_monitor=FakeDependency(),
        manipulation_module=FakeDependency(),
        config=ViserVisualizationConfig(panel_enabled=False),
    )

    with pytest.raises(RuntimeError, match="scene failed"):
        visualizer.initialize_scene(PlanningSceneInfo(robots={}))

    assert closed == ["runtime"]
    assert visualizer.get_visualization_url() is None


def test_visualizer_close_is_best_effort_when_gui_raises(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    closed = []

    class FakeRuntime:
        url = "http://localhost:8095"

        def __init__(self, config: ViserVisualizationConfig) -> None:
            self.config = config

        def start(self) -> FakeServer:
            return FakeServer()

        def close(self) -> None:
            closed.append("runtime")

    class FakeScene:
        def __init__(
            self,
            server: FakeServer,
            viser_urdf: type[FakeViserUrdf],
            *,
            preview_fps: float,
        ) -> None:
            pass

        def close(self) -> None:
            closed.append("scene")

    class FailingGui:
        def __init__(
            self,
            server: FakeServer,
            world_monitor: object,
            manipulation_module: object,
            config: ViserVisualizationConfig,
            scene: FakeScene,
        ) -> None:
            del world_monitor, manipulation_module, config, scene
            pass

        def start(self) -> None:
            pass

        def refresh(self) -> None:
            pass

        def close(self) -> None:
            closed.append("gui")
            raise RuntimeError("gui close failed")

    monkeypatch.setattr(visualizer_module, "ViserRuntime", FakeRuntime)
    monkeypatch.setattr(visualizer_module, "ViserUrdf", FakeViserUrdf)
    monkeypatch.setattr(visualizer_module, "ViserManipulationScene", FakeScene)
    monkeypatch.setattr(visualizer_module, "ViserPanelGui", FailingGui)
    visualizer = ViserManipulationVisualizer(
        world_monitor=FakeDependency(),
        manipulation_module=FakeDependency(),
        config=ViserVisualizationConfig(panel_enabled=True),
    )
    visualizer.initialize_scene(PlanningSceneInfo(robots={}))

    with pytest.raises(RuntimeError, match="gui close failed"):
        visualizer.close()

    assert closed == ["gui", "scene", "runtime"]
    assert visualizer.get_visualization_url() is None


def test_runtime_starts_once_opens_browser_and_closes(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    servers: list[FakeRuntimeServer] = []
    opened_urls: list[str] = []

    def fake_server(*, host: str, port: int) -> FakeRuntimeServer:
        assert host == "127.0.0.1"
        assert port == 8123
        server = FakeRuntimeServer()
        servers.append(server)
        return server

    monkeypatch.setattr(runtime_module, "ViserServer", fake_server)
    monkeypatch.setattr(runtime_module.webbrowser, "open_new_tab", opened_urls.append)
    runtime = ViserRuntime(ViserVisualizationConfig(host="127.0.0.1", port=8123, open_browser=True))

    first = runtime.start()
    second = runtime.start()

    assert first is second
    assert runtime.url == "http://127.0.0.1:8123"
    assert opened_urls == ["http://127.0.0.1:8123"]
    runtime.close()
    assert runtime.url is None
    assert servers[0].stopped is True
    runtime.close()


def test_visualizer_publish_preview_and_close_paths(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[tuple[str, str]] = []
    current = JointState({"name": ["joint1"], "position": [0.5]})

    class FakeRuntime:
        url = "http://localhost:8095"

        def __init__(self, config: ViserVisualizationConfig) -> None:
            self.config = config

        def start(self) -> FakeServer:
            calls.append(("runtime", "start"))
            return FakeServer()

        def close(self) -> None:
            calls.append(("runtime", "close"))

    class FakeScene:
        def __init__(
            self,
            server: FakeServer,
            viser_urdf: type[FakeViserUrdf],
            *,
            preview_fps: float,
        ) -> None:
            calls.append(("scene", "create"))

        def update_current_robot(self, robot_id: str, joint_state: JointState | None) -> None:
            assert joint_state == current
            calls.append(("update", robot_id))

        def show_preview(self, robot_id: str) -> None:
            calls.append(("show", robot_id))

        def hide_preview(self, robot_id: str) -> None:
            calls.append(("hide", robot_id))

        def animate_preview(self, preview: GroupPreviewAnimation, duration: float) -> None:
            assert preview.group_ids == ("arm/manipulator",)
            assert len(preview.tracks) == 1
            assert preview.tracks[0].path == (current,)
            assert duration == 1.5
            calls.append(("animate", preview.tracks[0].robot_id))

        def close(self) -> None:
            calls.append(("scene", "close"))

    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda _robot_id: current,
        planning_groups=SimpleNamespace(
            select=lambda _group_ids: SimpleNamespace(
                groups=(SimpleNamespace(id="arm/manipulator", robot_name="arm"),),
                robot_names=("arm",),
            )
        ),
    )
    robot_config = fake_robot_config("arm")
    manipulation_module = SimpleNamespace(
        robot_items=lambda: [("arm", "robot-1", robot_config)],
        robot_id_for_name=lambda robot_name: "robot-1" if robot_name == "arm" else None,
        get_robot_config=lambda robot_name: robot_config if robot_name == "arm" else None,
    )
    monkeypatch.setattr(visualizer_module, "ViserRuntime", FakeRuntime)
    monkeypatch.setattr(visualizer_module, "ViserUrdf", FakeViserUrdf)
    monkeypatch.setattr(visualizer_module, "ViserManipulationScene", FakeScene)
    visualizer = ViserManipulationVisualizer(
        world_monitor=world_monitor,
        manipulation_module=manipulation_module,
        config=ViserVisualizationConfig(panel_enabled=False),
    )

    visualizer.publish_visualization()
    visualizer.show_preview(("arm/manipulator",))
    visualizer.hide_preview(("arm/manipulator",))
    plan = GeneratedPlan(
        group_ids=("arm/manipulator",),
        path=[JointState(name=["arm/joint1"], position=[0.5])],
        status=PlanningStatus.SUCCESS,
    )
    visualizer.animate_plan(plan, duration=1.5)
    visualizer.close()
    visualizer.publish_visualization()

    assert calls == [
        ("runtime", "start"),
        ("scene", "create"),
        ("update", "robot-1"),
        ("show", "robot-1"),
        ("hide", "robot-1"),
        ("animate", "robot-1"),
        ("scene", "close"),
        ("runtime", "close"),
    ]


def test_visualizer_animates_multi_robot_plan_as_one_group_preview(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    previews: list[GroupPreviewAnimation] = []

    class FakeRuntime:
        url = "http://localhost:8095"

        def __init__(self, config: ViserVisualizationConfig) -> None:
            self.config = config

        def start(self) -> FakeServer:
            return FakeServer()

        def close(self) -> None:
            pass

    class FakeScene:
        def __init__(
            self,
            server: FakeServer,
            viser_urdf: type[FakeViserUrdf],
            *,
            preview_fps: float,
        ) -> None:
            pass

        def animate_preview(self, preview: GroupPreviewAnimation, duration: float) -> None:
            assert duration == 2.0
            previews.append(preview)

        def close(self) -> None:
            pass

    groups = (
        SimpleNamespace(id="left/arm", robot_name="left"),
        SimpleNamespace(id="right/arm", robot_name="right"),
    )
    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda robot_id: JointState(
            {"name": ["joint1"], "position": [0.0 if robot_id == "left-id" else 10.0]}
        ),
        planning_groups=SimpleNamespace(
            select=lambda _group_ids: SimpleNamespace(groups=groups, robot_names=("left", "right"))
        ),
    )
    configs = {"left": fake_robot_config("left"), "right": fake_robot_config("right")}
    manipulation_module = SimpleNamespace(
        robot_id_for_name=lambda robot_name: f"{robot_name}-id" if robot_name in configs else None,
        get_robot_config=lambda robot_name: configs.get(robot_name),
    )
    monkeypatch.setattr(visualizer_module, "ViserRuntime", FakeRuntime)
    monkeypatch.setattr(visualizer_module, "ViserUrdf", FakeViserUrdf)
    monkeypatch.setattr(visualizer_module, "ViserManipulationScene", FakeScene)
    visualizer = ViserManipulationVisualizer(
        world_monitor=world_monitor,
        manipulation_module=manipulation_module,
        config=ViserVisualizationConfig(panel_enabled=False),
    )

    visualizer.animate_plan(
        GeneratedPlan(
            group_ids=("left/arm", "right/arm"),
            path=[
                JointState(name=["left/joint1", "right/joint1"], position=[0.0, 10.0]),
                JointState(name=["left/joint1", "right/joint1"], position=[1.0, 11.0]),
            ],
            status=PlanningStatus.SUCCESS,
        ),
        duration=2.0,
    )

    assert len(previews) == 1
    preview = previews[0]
    assert preview.group_ids == ("left/arm", "right/arm")
    assert [(track.robot_id, track.group_ids) for track in preview.tracks] == [
        ("left-id", ("left/arm",)),
        ("right-id", ("right/arm",)),
    ]
    assert [tuple(point.position) for point in preview.tracks[0].path] == [(0.0,), (1.0,)]
    assert [tuple(point.position) for point in preview.tracks[1].path] == [(10.0,), (11.0,)]
