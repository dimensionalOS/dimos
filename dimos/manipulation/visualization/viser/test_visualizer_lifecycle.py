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
from typing import Any, cast

import pytest

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.models import PlanningSceneInfo
from dimos.manipulation.visualization.viser import visualizer as visualizer_module
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.visualizer import ViserManipulationVisualizer
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


class FakeServer:
    def __init__(self) -> None:
        self.scene = SimpleNamespace()


def fake_robot_config(name: str) -> RobotModelConfig:
    return RobotModelConfig(
        name=name,
        model_path=Path(f"{name}.urdf"),
        base_pose=PoseStamped(),
        joint_names=[],
        end_effector_link="ee_link",
    )


def test_visualizer_construction_is_lazy(monkeypatch: pytest.MonkeyPatch) -> None:
    def fail_runtime(_config: ViserVisualizationConfig) -> object:
        raise AssertionError("runtime should not start during construction")

    monkeypatch.setattr(visualizer_module, "ViserRuntime", fail_runtime)

    visualizer = ViserManipulationVisualizer(
        world_monitor=cast("Any", object()),
        manipulation_module=cast("Any", object()),
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
        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            calls.append(("create", "scene"))

        def register_robot(self, robot_id: str, config: Any) -> None:
            calls.append((robot_id, config.name))

        def close(self) -> None:
            calls.append(("close", "scene"))

    class FakeGui:
        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            calls.append(("create", "gui"))

        def start(self) -> None:
            calls.append(("start", "gui"))

        def refresh(self) -> None:
            calls.append(("refresh", "gui"))

        def close(self) -> None:
            calls.append(("close", "gui"))

    monkeypatch.setattr(visualizer_module, "ViserRuntime", FakeRuntime)
    monkeypatch.setattr(visualizer_module, "ViserUrdf", object)
    monkeypatch.setattr(visualizer_module, "ViserManipulationScene", FakeScene)
    monkeypatch.setattr(visualizer_module, "ViserPanelGui", FakeGui)
    visualizer = ViserManipulationVisualizer(
        world_monitor=cast("Any", object()),
        manipulation_module=cast("Any", object()),
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
        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            pass

        def close(self) -> None:
            closed.append("scene")

    class FakeGui:
        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            pass

        def start(self) -> None:
            raise RuntimeError("gui failed")

        def close(self) -> None:
            closed.append("gui")

    monkeypatch.setattr(visualizer_module, "ViserRuntime", FakeRuntime)
    monkeypatch.setattr(visualizer_module, "ViserUrdf", object)
    monkeypatch.setattr(visualizer_module, "ViserManipulationScene", FakeScene)
    monkeypatch.setattr(visualizer_module, "ViserPanelGui", FakeGui)
    visualizer = ViserManipulationVisualizer(
        world_monitor=cast("Any", object()),
        manipulation_module=cast("Any", object()),
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
        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            raise RuntimeError("scene failed")

    monkeypatch.setattr(visualizer_module, "ViserRuntime", FakeRuntime)
    monkeypatch.setattr(visualizer_module, "ViserUrdf", object)
    monkeypatch.setattr(visualizer_module, "ViserManipulationScene", FailingScene)
    visualizer = ViserManipulationVisualizer(
        world_monitor=cast("Any", object()),
        manipulation_module=cast("Any", object()),
        config=ViserVisualizationConfig(panel_enabled=False),
    )

    with pytest.raises(RuntimeError, match="scene failed"):
        visualizer.initialize_scene(PlanningSceneInfo(robots={}))

    assert closed == ["runtime"]
    assert visualizer.get_visualization_url() is None
