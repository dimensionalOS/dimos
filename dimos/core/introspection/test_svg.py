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

from pathlib import Path
import re
import subprocess

import pytest
from pytest_mock import MockerFixture

from dimos.core.coordination.blueprints import Blueprint, MetaBlueprint, autoconnect
from dimos.core.introspection.blueprint.dot import LayoutAlgo, render
from dimos.core.introspection.module.info import ModuleInfo
from dimos.core.introspection.svg import to_svg
from dimos.core.module import Module
from dimos.core.stream import In, Out


@pytest.mark.parametrize(
    ("target", "layout", "engine"),
    [
        pytest.param(ModuleInfo(name="Example"), None, "dot", id="module"),
        pytest.param(Blueprint(()), None, "dot", id="blueprint"),
        pytest.param(Blueprint(()), {LayoutAlgo.FDP}, "fdp", id="blueprint-fdp"),
        pytest.param(autoconnect(), None, "dot", id="composition"),
    ],
)
def test_svg_missing_graphviz_reports_install_commands(
    target: ModuleInfo | Blueprint | MetaBlueprint,
    layout: set[LayoutAlgo] | None,
    engine: str,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("PATH", str(tmp_path))

    with pytest.raises(RuntimeError) as exc_info:
        to_svg(target, str(tmp_path / "graph.svg"), layout=layout)

    assert str(exc_info.value) == (
        f"Graphviz executable '{engine}' was not found on PATH. "
        "Install Graphviz with 'sudo apt install graphviz' (Ubuntu/Debian) "
        "or 'brew install graphviz' (macOS)."
    )


def test_svg_reports_graphviz_rendering_errors(tmp_path: Path, mocker: MockerFixture) -> None:
    mocker.patch.object(
        subprocess,
        "run",
        return_value=subprocess.CompletedProcess(args=[], returncode=1, stderr="invalid graph"),
    )

    with pytest.raises(RuntimeError, match="^graphviz failed: invalid graph$"):
        to_svg(ModuleInfo(name="Example"), str(tmp_path / "graph.svg"))


class Camera(Module):
    image: Out[bytes]


class Display(Module):
    image: In[bytes]
    camera: Camera


def test_blueprint_graph_keeps_each_instances_stream_and_rpc() -> None:
    camera_display = autoconnect(Camera.blueprint(), Display.blueprint()).solidify()
    blueprint = autoconnect(camera_display.namespace("left"), camera_display.namespace("right"))
    graph = render(blueprint.solidify())

    for side in ("left", "right"):
        camera = f'"module:{side}/camera"'
        display = f'"module:{side}/display"'
        stream = re.search(rf"{camera} -> (\w+) ", graph)
        contract = re.search(rf"{display} -> (\w+) ", graph)
        assert stream and contract
        assert f"{stream[1]} -> {display} " in graph
        assert f"{contract[1]} -> {camera} " in graph
        other = "right" if side == "left" else "left"
        assert f'{stream[1]} -> "module:{other}/display"' not in graph
        assert f'{contract[1]} -> "module:{other}/camera"' not in graph


def test_disabled_modules_do_not_add_stream_nodes_or_edges() -> None:
    blueprint = autoconnect(Camera.blueprint(), Display.blueprint()).disabled_modules(Display)
    graph = render(blueprint.solidify())
    assert '"module:camera"' in graph
    assert '"module:display"' not in graph
    assert "image:bytes" not in graph
