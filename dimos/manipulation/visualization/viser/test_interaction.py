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

"""Tests for the R3 interactive layer: scan parsing, overlay, panel controls."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
from types import SimpleNamespace
from typing import Any

import pytest

pytest.importorskip("viser", reason="Viser optional dependency is not installed")

from dimos.manipulation.visualization.viser.adapter import parse_scan_objects
from dimos.manipulation.visualization.viser.config import (
    GroundTruthObject,
    ViserVisualizationConfig,
)
from dimos.manipulation.visualization.viser.scene import ViserManipulationScene


def _worldbelief_response(objects: list[dict]) -> dict:
    inner = json.dumps({"success": True, "message": "ok", "metadata": {"objects": objects}})
    return {"content": [{"type": "text", "text": inner}]}


def _scan_objects_response(text: str) -> dict:
    return {"content": [{"type": "text", "text": text}]}


# --- parse_scan_objects -----------------------------------------------------


def test_parse_worldbelief_scan_shape() -> None:
    resp = _worldbelief_response(
        [{"name": "red ball", "xyz": [0.4, 0.08, 0.19]}, {"name": "cup", "xyz": [0.5, 0.0, 0.19]}]
    )
    objs = parse_scan_objects(resp)
    assert [o["name"] for o in objs] == ["red ball", "cup"]
    assert objs[0]["x"] == pytest.approx(0.4)
    assert objs[1]["z"] == pytest.approx(0.19)


def test_parse_scan_objects_text_shape() -> None:
    text = "Detected 2 object(s):\n  - orange: (0.451, -0.079, 0.181) [7 views]\n  - sphere: (0.40, 0.08, 0.17) [3 views]"
    objs = parse_scan_objects(_scan_objects_response(text))
    assert [o["name"] for o in objs] == ["orange", "sphere"]
    assert objs[0]["y"] == pytest.approx(-0.079)


def test_parse_scan_empty_and_garbage() -> None:
    assert parse_scan_objects(_worldbelief_response([])) == []
    assert parse_scan_objects({"content": []}) == []
    assert parse_scan_objects("not a dict") == []
    assert parse_scan_objects(_scan_objects_response("No objects detected in scene")) == []


# --- scene overlay ----------------------------------------------------------


@dataclass
class _FakeMarker:
    name: str
    kwargs: dict[str, Any]
    removed: bool = False

    def remove(self) -> None:
        self.removed = True


@dataclass
class _FakeSceneApi:
    added: list[_FakeMarker] = field(default_factory=list)

    def add_grid(self, name: str, **kwargs: Any) -> _FakeMarker:
        return self._add(name, kwargs)

    def add_icosphere(self, name: str, **kwargs: Any) -> _FakeMarker:
        return self._add(name, kwargs)

    def add_label(self, name: str, text: str, **kwargs: Any) -> _FakeMarker:
        return self._add(name, {"text": text, **kwargs})

    def add_line_segments(self, name: str, **kwargs: Any) -> _FakeMarker:
        return self._add(name, kwargs)

    def _add(self, name: str, kwargs: dict[str, Any]) -> _FakeMarker:
        marker = _FakeMarker(name=name, kwargs=kwargs)
        self.added.append(marker)
        return marker


class _FakeServer:
    def __init__(self) -> None:
        self.scene = _FakeSceneApi()


def _make_scene() -> ViserManipulationScene:
    return ViserManipulationScene(_FakeServer(), object, preview_fps=30.0)


def test_render_ground_truth_adds_markers_and_labels() -> None:
    scene = _make_scene()
    scene.render_ground_truth([("apple", 0.4, 0.08, 0.17), ("cup", 0.5, 0.0, 0.19)])
    names = [m.name for m in scene.server.scene.added]
    assert "/ground_truth/0" in names
    assert "/ground_truth/1" in names
    assert "/ground_truth/0/label" in names


def test_render_detections_draws_pose_error_line_to_nearest_truth() -> None:
    scene = _make_scene()
    truth = [("apple", 0.4, 0.08, 0.17), ("orange", 0.45, -0.08, 0.175)]
    scene.render_detections([("red ball", 0.41, 0.07, 0.20)], truth)
    added = {m.name for m in scene.server.scene.added}
    assert "/detected/0" in added
    assert "/pose_error/0" in added


def test_render_detections_clears_previous_markers() -> None:
    scene = _make_scene()
    scene.render_detections([("a", 0.1, 0.1, 0.1)], [])
    first_markers = [m for m in scene.server.scene.added if m.name.startswith("/detected/")]
    scene.render_detections([("b", 0.2, 0.2, 0.2)], [])
    # The first detection's marker handle must have been removed on re-render.
    assert all(m.removed for m in first_markers)


# --- config -----------------------------------------------------------------


def test_config_accepts_scan_and_ground_truth() -> None:
    config = ViserVisualizationConfig(
        scan_tool="scan",
        scan_prompt="red ball, orange ball",
        ground_truth_objects=[{"name": "apple", "x": 0.4, "y": 0.08, "z": 0.17}],
    )
    assert config.scan_tool == "scan"
    assert config.ground_truth_objects[0] == GroundTruthObject(name="apple", x=0.4, y=0.08, z=0.17)


def test_config_defaults_disable_interaction() -> None:
    config = ViserVisualizationConfig()
    assert config.scan_tool is None
    assert config.ground_truth_objects == ()


# --- runtime URL reports the actually-bound port ----------------------------


def test_runtime_url_reports_bound_port_not_configured_port() -> None:
    from dimos.manipulation.visualization.viser.runtime import ViserRuntime

    runtime = ViserRuntime(ViserVisualizationConfig(host="127.0.0.1", port=8095))
    # Simulate Viser auto-incrementing off a busy 8095 to 8098.
    runtime.server = SimpleNamespace(get_host=lambda: "127.0.0.1", get_port=lambda: 8098)
    assert runtime.url == "http://127.0.0.1:8098"


def test_runtime_url_falls_back_to_config_without_accessors() -> None:
    from dimos.manipulation.visualization.viser.runtime import ViserRuntime

    runtime = ViserRuntime(ViserVisualizationConfig(host="127.0.0.1", port=8095))
    runtime.server = object()  # no get_host/get_port
    assert runtime.url == "http://127.0.0.1:8095"
