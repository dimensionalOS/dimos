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
import subprocess

import pytest
from pytest_mock import MockerFixture

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.introspection.blueprint.dot import LayoutAlgo
from dimos.core.introspection.module.info import ModuleInfo
from dimos.core.introspection.svg import to_svg


@pytest.mark.parametrize(
    ("target", "layout", "engine"),
    [
        pytest.param(ModuleInfo(name="Example"), None, "dot", id="module"),
        pytest.param(Blueprint(()), None, "dot", id="blueprint"),
        pytest.param(Blueprint(()), {LayoutAlgo.FDP}, "fdp", id="blueprint-fdp"),
    ],
)
def test_svg_missing_graphviz_reports_install_commands(
    target: ModuleInfo | Blueprint,
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
