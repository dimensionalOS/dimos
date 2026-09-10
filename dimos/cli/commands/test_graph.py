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
import sys

import pytest
from pytest_mock import MockerFixture
from typer.testing import CliRunner

import dimos.cli.commands.graph as graph_command
from dimos.cli.dimos import main


def test_render_graph_resolves_blueprint_and_calls_existing_renderer(
    tmp_path: Path,
    mocker: MockerFixture,
) -> None:
    blueprint = mocker.sentinel.blueprint
    output = tmp_path / "graph.svg"
    resolve = mocker.patch.object(
        graph_command.get_all_blueprints,
        "get_by_name",
        return_value=blueprint,
    )
    render = mocker.patch.object(graph_command, "render_svg")

    graph_command._render_graph("alpha", output, include_rpc=False)

    resolve.assert_called_once_with("alpha")
    render.assert_called_once_with(blueprint, str(output), include_rpc=False)


def test_graph_command_uses_default_svg_path(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    mocker: MockerFixture,
) -> None:
    monkeypatch.chdir(tmp_path)
    render = mocker.patch.object(graph_command, "_render_in_subprocess")

    result = CliRunner().invoke(main, ["graph", "alpha"])

    expected = (tmp_path / "alpha.svg").resolve()
    assert result.exit_code == 0, result.output
    render.assert_called_once_with("alpha", expected, include_rpc=False)
    assert result.output == f"Blueprint graph written to {expected}\n"


def test_graph_command_includes_rpc_when_requested(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    mocker: MockerFixture,
) -> None:
    monkeypatch.chdir(tmp_path)
    render = mocker.patch.object(graph_command, "_render_in_subprocess")

    result = CliRunner().invoke(main, ["graph", "alpha", "--rpc"])

    expected = (tmp_path / "alpha.svg").resolve()
    assert result.exit_code == 0, result.output
    render.assert_called_once_with("alpha", expected, include_rpc=True)


def test_graph_command_creates_output_parent(
    tmp_path: Path,
    mocker: MockerFixture,
) -> None:
    render = mocker.patch.object(graph_command, "_render_in_subprocess")
    output = tmp_path / "nested" / "alpha.svg"

    result = CliRunner().invoke(main, ["graph", "alpha", "--output", str(output)])

    assert result.exit_code == 0, result.output
    assert output.parent.is_dir()
    render.assert_called_once_with("alpha", output.resolve(), include_rpc=False)


def test_graph_command_rejects_non_svg_output(
    tmp_path: Path,
    mocker: MockerFixture,
) -> None:
    render = mocker.patch.object(graph_command, "_render_in_subprocess")

    result = CliRunner().invoke(
        main,
        ["graph", "alpha", "--output", str(tmp_path / "alpha.png")],
    )

    assert result.exit_code == 2
    assert "output path must end in .svg" in result.output
    render.assert_not_called()


def test_graph_command_reports_renderer_error(mocker: MockerFixture) -> None:
    mocker.patch.object(
        graph_command,
        "_render_in_subprocess",
        side_effect=graph_command.GraphRenderError("graphviz failed"),
    )

    result = CliRunner().invoke(main, ["graph", "alpha"])

    assert result.exit_code == 1
    assert "graphviz failed" in result.output


def test_graph_subprocess_uses_static_transport_environment(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    mocker: MockerFixture,
) -> None:
    monkeypatch.setenv("LCM_DEFAULT_URL", "udpm://239.255.76.67:7667?ttl=0")
    monkeypatch.setenv("viewer", "rerun")
    run = mocker.patch.object(
        graph_command.subprocess,
        "run",
        return_value=subprocess.CompletedProcess(args=[], returncode=0),
    )
    output = tmp_path / "alpha.svg"

    graph_command._render_in_subprocess("alpha", output, include_rpc=False)

    run.assert_called_once()
    args, kwargs = run.call_args
    assert args[0] == [
        sys.executable,
        "-m",
        "dimos.cli.commands.graph",
        "alpha",
        str(output),
    ]
    assert kwargs["env"]["LCM_DEFAULT_URL"] == "memq://"
    assert kwargs["env"]["viewer"] == "none"
    assert kwargs["text"] is True
    assert kwargs["capture_output"] is True


def test_graph_subprocess_forwards_rpc_flag(
    tmp_path: Path,
    mocker: MockerFixture,
) -> None:
    run = mocker.patch.object(
        graph_command.subprocess,
        "run",
        return_value=subprocess.CompletedProcess(args=[], returncode=0),
    )
    output = tmp_path / "alpha.svg"

    graph_command._render_in_subprocess("alpha", output, include_rpc=True)

    args, _kwargs = run.call_args
    assert args[0] == [
        sys.executable,
        "-m",
        "dimos.cli.commands.graph",
        "alpha",
        str(output),
        "--rpc",
    ]


@pytest.mark.parametrize(
    ("extra_args", "include_rpc"),
    [
        ([], False),
        (["--rpc"], True),
    ],
)
def test_graph_worker_forwards_rpc_choice_to_renderer(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    mocker: MockerFixture,
    extra_args: list[str],
    include_rpc: bool,
) -> None:
    output = tmp_path / "alpha.svg"
    monkeypatch.setattr(
        graph_command.sys,
        "argv",
        ["dimos.cli.commands.graph", "alpha", str(output), *extra_args],
    )
    render = mocker.patch.object(graph_command, "_render_graph")

    graph_command._worker_main()

    render.assert_called_once_with("alpha", output, include_rpc=include_rpc)
