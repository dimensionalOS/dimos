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

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.models import GeneratedPlan, PlanningSceneInfo
from dimos.manipulation.visualization.viser import (
    runtime as runtime_module,
    visualizer as visualizer_module,
)
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.runtime import ViserRuntime
from dimos.manipulation.visualization.viser.scene import ViserManipulationScene
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
        joint_names=[],
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator", joint_names=(), base_link="base_link", tip_link="ee_link"
            )
        ],
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
            world_monitor: FakeDependency,
            manipulation_module: FakeDependency,
            config: ViserVisualizationConfig,
            scene: FakeScene,
        ) -> None:
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
            world_monitor: FakeDependency,
            manipulation_module: FakeDependency,
            config: ViserVisualizationConfig,
            scene: FakeScene,
        ) -> None:
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
            world_monitor: FakeDependency,
            manipulation_module: FakeDependency,
            config: ViserVisualizationConfig,
            scene: FakeScene,
        ) -> None:
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

        def cancel_preview_animation(self) -> None:
            calls.append(("cancel", "preview"))

        def animate_preview(self, preview: object, duration: float) -> None:
            assert duration == 1.5
            calls.append(("animate", "groups"))

        def close(self) -> None:
            calls.append(("scene", "close"))

    world_monitor = SimpleNamespace(
        get_current_joint_state=lambda _robot_id: current,
        planning_groups=SimpleNamespace(
            select=lambda _group_ids: SimpleNamespace(
                robot_names=("arm",), joint_names=("arm/joint1",)
            )
        ),
    )
    manipulation_module = SimpleNamespace(
        robot_items=lambda: [("arm", "robot-1", fake_robot_config("arm"))],
        robot_id_for_name=lambda robot_name: "robot-1" if robot_name == "arm" else None,
        get_robot_config=lambda robot_name: fake_robot_config("arm")
        if robot_name == "arm"
        else None,
    )
    monkeypatch.setattr(visualizer_module, "ViserRuntime", FakeRuntime)
    monkeypatch.setattr(visualizer_module, "ViserUrdf", FakeViserUrdf)
    monkeypatch.setattr(visualizer_module, "ViserManipulationScene", FakeScene)
    visualizer = ViserManipulationVisualizer(
        world_monitor=world_monitor,
        manipulation_module=manipulation_module,
        config=ViserVisualizationConfig(panel_enabled=False),
    )

    assert hasattr(ViserManipulationVisualizer, "cancel_preview_animation")
    visualizer.cancel_preview_animation()
    visualizer.publish_visualization()
    visualizer.cancel_preview_animation()
    visualizer.animate_plan(
        GeneratedPlan(
            group_ids=("arm/manipulator",),
            path=[JointState({"name": ["arm/joint1"], "position": [0.5]})],
        ),
        duration=1.5,
    )
    visualizer.hide_preview(("arm/manipulator",))
    visualizer.close()
    visualizer.publish_visualization()

    assert calls == [
        ("runtime", "start"),
        ("scene", "create"),
        ("update", "robot-1"),
        ("cancel", "preview"),
        ("animate", "groups"),
        ("cancel", "preview"),
        ("hide", "robot-1"),
        ("scene", "close"),
        ("runtime", "close"),
    ]


def test_scene_prepares_urdf_applies_base_pose_and_rejects_wrong_root(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    created: list[tuple[Path, str]] = []
    prepared: list[dict[str, object]] = []
    frames: list[dict[str, object]] = []
    fixed_world_root = tmp_path / "fixed-world.urdf"
    fixed_world_root.write_text(
        """<robot name="arm">
<link name="world"/><link name="base_link"/>
<joint name="world_to_base" type="fixed"><parent link="world"/><child link="base_link"/></joint>
</robot>"""
    )
    non_fixed_world_root = tmp_path / "non-fixed-world.urdf"
    non_fixed_world_root.write_text(
        """<robot name="arm">
<link name="world"/><link name="base_link"/>
<joint name="world_to_base" type="revolute"><parent link="world"/><child link="base_link"/></joint>
</robot>"""
    )

    class Handle:
        def remove(self) -> None:
            return None

    class SceneApi:
        def add_frame(self, name: str, **kwargs: object) -> Handle:
            frames.append({"name": name, **kwargs})
            return Handle()

    class Server:
        scene = SceneApi()

    class Urdf:
        def __init__(
            self, _server: Server, path: Path, *, root_node_name: str, **_kwargs: object
        ) -> None:
            created.append((path, root_node_name))
            self._meshes: list[object] = []

    config = fake_robot_config("arm")
    config.base_pose.position.x = 1.0

    def prepare(path: Path, **kwargs: object) -> Path:
        prepared.append(kwargs)
        return fixed_world_root if path.name == "arm.urdf" else non_fixed_world_root

    monkeypatch.setattr(
        "dimos.manipulation.visualization.viser.scene.prepare_urdf_for_drake",
        prepare,
    )

    def parse_prepared_model(path: Path) -> SimpleNamespace:
        content = path.read_text()
        return SimpleNamespace(root_link="world" if 'name="world"' in content else "base_link")

    monkeypatch.setattr(
        "dimos.manipulation.visualization.viser.scene.parse_model", parse_prepared_model
    )
    scene = ViserManipulationScene(Server(), Urdf, preview_fps=30.0)

    scene.register_robot("robot-1", config)

    assert [root for _, root in created] == [
        "/robots/robot-1/current/base_pose/urdf",
        "/targets/robot-1/target/base_pose/urdf",
        "/previews/robot-1/ghost/base_pose/urdf",
    ]
    assert prepared == [{"package_paths": {}, "xacro_args": {}, "convert_meshes": False}] * 3
    assert all('name="world"' not in path.read_text() for path, _ in created)
    assert all(frame["position"] == (1.0, 0.0, 0.0) for frame in frames)
    wrong_root_config = fake_robot_config("wrong")
    with pytest.raises(ValueError, match="prepared URDF root 'world'"):
        scene.prepared_urdf_path(wrong_root_config)
