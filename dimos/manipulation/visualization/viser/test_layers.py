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

"""Hermetic Viser tests for generic visualization layers."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from dataclasses import dataclass, field
from threading import Event
from types import SimpleNamespace

import numpy as np
import pytest
from pytest_mock import MockerFixture

pytest.importorskip("viser", reason="Viser optional dependency is not installed")

from dimos.manipulation.visualization.layers import (
    LineSetElement,
    MeshElement,
    PointCloudElement,
    VisualizationLayer,
)
from dimos.manipulation.visualization.viser.layers import ViserLayerManager
from dimos.manipulation.visualization.viser.scene import ViserManipulationScene


@dataclass
class Handle:
    name: str
    visible: bool = True
    value: bool = True
    callback: Callable[[object], None] | None = None
    removed: bool = False
    callback_on_assignment: bool = False
    _initialized: bool = field(default=False, init=False)

    def __post_init__(self) -> None:
        self._initialized = True

    def __setattr__(self, name: str, value: object) -> None:
        previous = getattr(self, name, None)
        object.__setattr__(self, name, value)
        if (
            name == "value"
            and getattr(self, "_initialized", False)
            and getattr(self, "callback_on_assignment", False)
            and previous != value
            and self.callback is not None
        ):
            self.callback(SimpleNamespace(target=self))

    def on_update(self, callback: Callable[[object], None]) -> None:
        self.callback = callback

    def remove(self) -> None:
        self.removed = True

    def trigger(self, value: bool) -> None:
        self.value = value
        if not self.callback_on_assignment:
            assert self.callback is not None
            self.callback(SimpleNamespace(target=self))


class Folder(Handle):
    def __enter__(self) -> Folder:
        return self

    def __exit__(self, *_args: object) -> bool:
        return False


class Gui:
    def __init__(self, *, callback_on_assignment: bool = False) -> None:
        self.folders: list[Folder] = []
        self.checkboxes: list[Handle] = []
        self.callback_on_assignment = callback_on_assignment

    def add_folder(self, label: str, **_kwargs: object) -> Folder:
        handle = Folder(label)
        self.folders.append(handle)
        return handle

    def add_checkbox(self, label: str, *, initial_value: bool) -> Handle:
        handle = Handle(
            label,
            value=initial_value,
            callback_on_assignment=self.callback_on_assignment,
        )
        self.checkboxes.append(handle)
        return handle


class SceneApi:
    def __init__(self) -> None:
        self.handles: list[Handle] = []
        self.calls: list[tuple[str, str, dict[str, object]]] = []
        self.fail_on_name: str | None = None

    def add_grid(self, name: str, **_kwargs: object) -> Handle:
        return self._add("grid", name, {})

    def add_point_cloud(self, name: str, **kwargs: object) -> Handle:
        return self._add("point_cloud", name, kwargs)

    def add_line_segments(self, name: str, **kwargs: object) -> Handle:
        return self._add("line_segments", name, kwargs)

    def add_mesh_simple(
        self, name: str, vertices: np.ndarray, faces: np.ndarray, **kwargs: object
    ) -> Handle:
        return self._add("mesh", name, {"vertices": vertices, "faces": faces, **kwargs})

    def _add(self, kind: str, name: str, kwargs: dict[str, object]) -> Handle:
        if self.fail_on_name is not None and self.fail_on_name in name:
            raise RuntimeError("injected render failure")
        handle = Handle(name, visible=bool(kwargs.get("visible", True)))
        for key, value in kwargs.items():
            setattr(handle, key, value)
        self.handles.append(handle)
        self.calls.append((kind, name, kwargs))
        return handle


class Server:
    def __init__(self, *, callback_on_assignment: bool = False) -> None:
        self.gui = Gui(callback_on_assignment=callback_on_assignment)
        self.scene = SceneApi()


class Urdf:
    pass


@pytest.fixture
def scene() -> Iterator[ViserManipulationScene]:
    value = ViserManipulationScene(Server(), Urdf)  # type: ignore[arg-type]
    try:
        yield value
    finally:
        value.close()


@pytest.fixture
def manager(
    scene: ViserManipulationScene,
) -> Iterator[ViserLayerManager]:
    value = ViserLayerManager(scene.server, scene)
    try:
        yield value
    finally:
        value.close()


def point_layer(
    value: float = 0.0,
    *,
    visible: bool = True,
    count: int = 1,
    colors: np.ndarray | None = None,
) -> VisualizationLayer:
    points = np.full((count, 3), value, dtype=np.float32)
    return VisualizationLayer(
        "grasp/object-cloud",
        "world",
        (PointCloudElement("object", points, colors),),
        default_visible=visible,
    )


def test_scene_renders_cloud_with_paired_cap_and_fallback_color(
    scene: ViserManipulationScene, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr("dimos.manipulation.visualization.viser.scene.VISUALIZATION_POINT_CAP", 2)
    colors = np.asarray(
        [[0, 1, 2], [3, 4, 5], [6, 7, 8], [9, 10, 11], [12, 13, 14]],
        dtype=np.uint8,
    )
    layer = point_layer(count=5, colors=colors)

    scene.replace_visualization_layer(layer, generation=1, visible=True)
    cloud_call = next(call for call in scene.server.scene.calls if call[0] == "point_cloud")

    np.testing.assert_array_equal(
        cloud_call[2]["points"],
        np.asarray([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=np.float32),
    )
    np.testing.assert_array_equal(cloud_call[2]["colors"], colors[::3])
    assert cloud_call[2]["point_size"] == pytest.approx(0.005)
    assert scene.server.scene.handles[-1].visible is True

    scene.replace_visualization_layer(point_layer(value=1.0), generation=2, visible=True)
    fallback = scene.server.scene.calls[-1][2]
    assert fallback["colors"] == (0, 204, 204)


def test_scene_renders_indexed_line_set_and_encodes_logical_ids(
    scene: ViserManipulationScene,
) -> None:
    element = LineSetElement(
        "rank-1",
        np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]]),
        np.asarray([[0, 1], [1, 2]]),
        colors=np.asarray([[0, 255, 0], [255, 128, 0]]),
        line_width=2.5,
    )
    layer = VisualizationLayer("grasp/proposals", "world", (element,))

    scene.replace_visualization_layer(layer, generation=7, visible=True)
    kind, name, kwargs = scene.server.scene.calls[-1]

    assert kind == "line_segments"
    assert "grasp/proposals" not in name
    assert "rank-1" not in name
    assert "generation-7" in name
    np.testing.assert_array_equal(
        kwargs["points"],
        np.asarray(
            [
                [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
                [[1.0, 0.0, 0.0], [1.0, 1.0, 0.0]],
            ]
        ),
    )
    np.testing.assert_array_equal(
        kwargs["colors"],
        np.asarray(
            [
                [[0, 255, 0], [0, 255, 0]],
                [[255, 128, 0], [255, 128, 0]],
            ]
        ),
    )
    assert kwargs["line_width"] == pytest.approx(2.5)


def test_scene_renders_filled_mesh(scene: ViserManipulationScene) -> None:
    element = MeshElement(
        "tabletop-fill",
        np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]]),
        np.asarray([[0, 1, 2], [0, 2, 3]]),
        np.asarray([80, 180, 255]),
        opacity=0.65,
    )

    scene.replace_visualization_layer(
        VisualizationLayer("pick/table", "world", (element,)), generation=1, visible=True
    )

    kind, _name, kwargs = scene.server.scene.calls[-1]
    assert kind == "mesh"
    np.testing.assert_array_equal(kwargs["faces"], [[0, 1, 2], [0, 2, 3]])
    assert kwargs["color"] == (80, 180, 255)
    assert kwargs["opacity"] == pytest.approx(0.65)


def test_scene_failed_replacement_retains_previous_generation(
    scene: ViserManipulationScene,
) -> None:
    scene.replace_visualization_layer(point_layer(), generation=1, visible=True)
    previous = scene.server.scene.handles[-1]
    first = PointCloudElement("first", np.asarray([[1.0, 0.0, 0.0]]))
    failing = PointCloudElement("fail", np.asarray([[2.0, 0.0, 0.0]]))
    replacement = VisualizationLayer("grasp/object-cloud", "world", (first, failing))
    scene.server.scene.fail_on_name = "6661696c"  # "fail" in hexadecimal

    with pytest.raises(RuntimeError, match="injected"):
        scene.replace_visualization_layer(replacement, generation=2, visible=True)

    assert previous.removed is False
    partial = next(handle for handle in scene.server.scene.handles if "generation-2" in handle.name)
    assert partial.removed is True


def test_manager_registers_hierarchy_and_preserves_hidden_state(
    manager: ViserLayerManager,
    scene: ViserManipulationScene,
) -> None:
    manager.set_layer(point_layer(visible=False))
    manager.set_layer(
        VisualizationLayer(
            "grasp/proposals",
            "world",
            (
                LineSetElement(
                    "rank-1",
                    np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]),
                    np.asarray([[0, 1]]),
                ),
            ),
        )
    )
    assert manager.wait_idle()

    assert manager.layer_ids == ("grasp/object-cloud", "grasp/proposals")
    assert manager.visibility("grasp/object-cloud") is False
    assert [folder.name for folder in scene.server.gui.folders if not folder.removed].count(
        "Grasp"
    ) == 1
    object_handle = next(
        handle
        for handle in scene.server.gui.checkboxes
        if handle.name == "Object Cloud" and not handle.removed
    )
    object_handle.trigger(True)
    assert manager.visibility("grasp/object-cloud") is True

    manager.set_visible("grasp/object-cloud", False)
    manager.set_layer(point_layer(value=2.0, visible=True))
    assert manager.wait_idle()
    assert manager.visibility("grasp/object-cloud") is False
    assert scene.server.scene.handles[-1].visible is False

    manager.clear_layer("grasp/object-cloud")
    assert manager.wait_idle()
    assert "grasp/object-cloud" in manager.layer_ids
    assert manager.visibility("grasp/object-cloud") is False


def test_manager_parent_toggle_updates_all_descendants(
    manager: ViserLayerManager,
    scene: ViserManipulationScene,
) -> None:
    manager.set_layer(point_layer())
    manager.set_layer(VisualizationLayer("grasp/proposals", "world", ()))
    assert manager.wait_idle()
    parent = next(
        handle
        for handle in scene.server.gui.checkboxes
        if handle.name == "All" and not handle.removed
    )

    parent.trigger(False)

    assert manager.visibility("grasp/object-cloud") is False
    assert manager.visibility("grasp/proposals") is False


def test_manager_leaf_toggle_does_not_cascade_through_reactive_parent() -> None:
    scene = ViserManipulationScene(Server(callback_on_assignment=True), Urdf)  # type: ignore[arg-type]
    manager = ViserLayerManager(scene.server, scene)
    try:
        manager.set_layer(point_layer())
        manager.set_layer(VisualizationLayer("grasp/proposals", "world", ()))
        assert manager.wait_idle()
        object_handle = next(
            handle
            for handle in scene.server.gui.checkboxes
            if handle.name == "Object Cloud" and not handle.removed
        )

        object_handle.trigger(False)

        assert manager.visibility("grasp/object-cloud") is False
        assert manager.visibility("grasp/proposals") is True
    finally:
        manager.close()
        scene.close()


def test_manager_latest_pending_operation_wins(
    manager: ViserLayerManager,
    scene: ViserManipulationScene,
    mocker: MockerFixture,
) -> None:
    entered = Event()
    release = Event()
    original = scene.replace_visualization_layer

    def block_first(layer: VisualizationLayer, *, generation: int, visible: bool) -> None:
        if not entered.is_set():
            entered.set()
            assert release.wait(2.0)
        original(layer, generation=generation, visible=visible)

    mocker.patch.object(scene, "replace_visualization_layer", side_effect=block_first)
    manager.set_layer(point_layer(value=1.0))
    assert entered.wait(2.0)
    manager.set_layer(point_layer(value=2.0))
    manager.clear_layer("grasp/object-cloud")
    release.set()

    assert manager.wait_idle()
    assert not any(
        not handle.removed and "generation-" in handle.name for handle in scene.server.scene.handles
    )


def test_manager_failure_is_contained_and_cross_layer_remains_independent(
    manager: ViserLayerManager,
    scene: ViserManipulationScene,
) -> None:
    scene.server.scene.fail_on_name = "6661696c"
    manager.set_layer(
        VisualizationLayer(
            "debug/failing",
            "world",
            (PointCloudElement("fail", np.asarray([[0.0, 0.0, 0.0]])),),
        )
    )
    manager.set_layer(point_layer())

    assert manager.wait_idle()
    assert "injected render failure" in (manager.warning("debug/failing") or "")
    assert manager.warning("grasp/object-cloud") is None
    assert any(
        not handle.removed and "generation-" in handle.name for handle in scene.server.scene.handles
    )


def test_scene_rejects_unsupported_frame_without_replacing_current(
    scene: ViserManipulationScene,
) -> None:
    scene.replace_visualization_layer(point_layer(), generation=1, visible=True)
    previous = scene.server.scene.handles[-1]

    with pytest.raises(ValueError, match="requires frame 'world'"):
        scene.replace_visualization_layer(
            VisualizationLayer(
                "grasp/object-cloud",
                "camera",
                (PointCloudElement("object", np.asarray([[0.0, 0.0, 0.0]])),),
            ),
            generation=2,
            visible=True,
        )

    assert previous.removed is False
