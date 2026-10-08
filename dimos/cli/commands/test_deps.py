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

import pytest
from pytest_mock import MockerFixture
import typer
from typer.testing import CliRunner

from dimos.cli.commands import deps as deps_commands
from dimos.cli.commands.deps import deps, prepare

app = typer.Typer()
app.command()(deps)
app.command()(prepare)
runner = CliRunner()


@pytest.fixture
def cpu_host(mocker: MockerFixture) -> None:
    mocker.patch.object(deps_commands, "resolve_backend", return_value=("cpu", "requested"))
    mocker.patch.object(deps_commands, "check_supported")
    mocker.patch.object(deps_commands, "limitations", return_value=["note one"])


def test_deps_shows_bundles_and_the_prepare_command(cpu_host: None, mocker: MockerFixture) -> None:
    run = mocker.patch("subprocess.run")

    result = runner.invoke(app, ["deps", "unitree-go2", "coordinator-xarm7", "--backend", "cpu"])

    assert result.exit_code == 0, result.output
    assert "unitree-go2        runtime-unitree" in result.output
    assert "coordinator-xarm7  runtime-manipulation" in result.output
    assert "Bundles:      runtime-manipulation, runtime-unitree" in result.output
    assert (
        "Prepare:      dimos prepare unitree-go2 coordinator-xarm7 --backend cpu" in result.output
    )
    assert "  - note one" in result.output
    run.assert_not_called()


def test_deps_unknown_name_exits_nonzero_with_suggestions(cpu_host: None) -> None:
    result = runner.invoke(app, ["deps", "unitree-go3"])

    assert result.exit_code == 1
    assert "Did you mean: unitree-go2" in result.output


def test_deps_reports_unsupported_combinations_and_exits_2(mocker: MockerFixture) -> None:
    mocker.patch.object(deps_commands, "resolve_backend", return_value=("cpu", "requested"))
    mocker.patch.object(
        deps_commands,
        "check_supported",
        side_effect=deps_commands.UnsupportedError("needs the CycloneDDS C library"),
    )
    mocker.patch.object(deps_commands, "limitations", return_value=[])

    result = runner.invoke(app, ["deps", "unitree-g1-teleop"])

    assert result.exit_code == 2
    assert "Unsupported:  needs the CycloneDDS C library" in result.output


def test_deps_rejects_an_invalid_backend_choice() -> None:
    result = runner.invoke(app, ["deps", "unitree-go2", "--backend", "tpu"])

    assert result.exit_code == 2
    assert "--backend must be one of auto, cpu, cuda" in result.output


def test_prepare_rejects_external_names_before_installing(mocker: MockerFixture) -> None:
    install = mocker.patch.object(deps_commands.install, "prepare")

    result = runner.invoke(app, ["prepare", "unitree-go2", "my-pkg.my-blueprint"])

    assert result.exit_code == 2
    assert "my-pkg.my-blueprint" in result.output
    install.assert_not_called()


def test_prepare_installs_the_union_and_reports_python_dependencies_only(
    cpu_host: None, mocker: MockerFixture
) -> None:
    install = mocker.patch.object(deps_commands.install, "prepare")

    result = runner.invoke(app, ["prepare", "drone-basic", "unitree-go2", "--offline"])

    assert result.exit_code == 0, result.output
    install.assert_called_once_with(["runtime-drone", "runtime-unitree"], "cpu", True)
    assert "Installed Python dependencies for runtime-drone, runtime-unitree (cpu)" in result.output
    assert "not verified" in result.output


def test_prepare_surfaces_installer_errors(cpu_host: None, mocker: MockerFixture) -> None:
    mocker.patch.object(
        deps_commands.install,
        "prepare",
        side_effect=deps_commands.install.PrepareError("installer exited with status 1"),
    )

    result = runner.invoke(app, ["prepare", "unitree-go2"])

    assert result.exit_code == 1
    assert "installer exited with status 1" in result.output
