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

import sys
from unittest.mock import call

import pytest
from typer.testing import CliRunner

from dimos.cli.commands import deps as deps_commands, lifecycle
from dimos.cli.dimos import main
from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.global_config import global_config
import dimos.utils.cache as cache_utils


@pytest.fixture
def prepared_run(tmp_path, monkeypatch, mocker):
    """Exercise preparation without installing packages or replacing the test process."""
    original_config = global_config.model_dump()
    monkeypatch.setattr(cache_utils, "_CACHE_LOCK_DIR", tmp_path / "cache-users")
    monkeypatch.setattr(cache_utils, "_CACHE_GATE_PATH", tmp_path / "cache-clean.lock")
    monkeypatch.setattr(lifecycle, "CONFIG_DIR", tmp_path / "config")
    mocker.patch.object(deps_commands, "resolve_backend", return_value=("cpu", "auto"))
    mocker.patch.object(deps_commands, "check_supported")

    calls = mocker.Mock()
    calls.attach_mock(mocker.patch.object(deps_commands.install, "prepare"), "install")
    calls.attach_mock(mocker.patch.object(lifecycle.os, "execv"), "launch")

    # These imports require runtime dependencies that may not exist until preparation.
    for module in (
        "dimos.core.coordination.blueprint_config.parser",
        "dimos.core.coordination.module_coordinator",
        "dimos.robot.get_all_blueprints",
    ):
        monkeypatch.setitem(sys.modules, module, None)

    try:
        yield calls
    finally:
        global_config.update(**original_config)


@pytest.mark.parametrize(
    ("argv", "bundles"),
    [
        (["run", "--prepare", "unitree-go2"], ["runtime-unitree"]),
        (
            [
                "run",
                "drone-basic",
                "unitree-go2",
                "--daemon",
                "--entity-prefix",
                "space value",
                "--prepare",
                "--disable",
                "OsmSkill",
            ],
            ["runtime-drone", "runtime-unitree"],
        ),
        (["--replay", "run", "sim-go2-world", "--prepare"], ["runtime-unitree"]),
    ],
)
def test_run_prepares_before_runtime_imports_and_relaunches(
    argv, bundles, prepared_run, monkeypatch
):
    original_argv = ["/venv/bin/dimos", *argv]
    monkeypatch.setattr(sys, "argv", original_argv)

    result = CliRunner().invoke(main, argv)

    assert result.exit_code == 0, result.output
    assert prepared_run.mock_calls == [
        call.install(bundles, "cpu", False),
        call.launch(
            sys.executable,
            [sys.executable, *[arg for arg in original_argv if arg != "--prepare"]],
        ),
    ]


def test_run_prepare_failure_prevents_launch(prepared_run):
    prepared_run.install.side_effect = deps_commands.install.PrepareError("uv failed")

    result = CliRunner().invoke(main, ["run", "--prepare", "unitree-go2"])

    assert result.exit_code == 1
    assert "Error: uv failed" in result.output
    prepared_run.launch.assert_not_called()


@pytest.mark.parametrize(
    ("name", "exit_code", "message"),
    [
        ("unitree-go3", 1, "Did you mean: unitree-go2"),
        ("my-stack.go2", 2, "external blueprints get their dependencies from their own package"),
    ],
)
def test_run_prepare_rejects_names_before_installing(name, exit_code, message, prepared_run):
    result = CliRunner().invoke(main, ["run", "--prepare", name])

    assert result.exit_code == exit_code
    assert message in result.output
    prepared_run.install.assert_not_called()
    prepared_run.launch.assert_not_called()


def test_run_prepare_requires_names_before_config_flags(prepared_run):
    result = CliRunner().invoke(
        main, ["run", "--prepare", "--robot-ip", "192.0.2.1", "unitree-go2"]
    )

    assert result.exit_code == 2
    assert "At least one blueprint name must precede configuration options" in result.output
    prepared_run.install.assert_not_called()
    prepared_run.launch.assert_not_called()


def test_run_prepare_reports_launch_failure(prepared_run, monkeypatch):
    argv = ["run", "--prepare", "unitree-go2"]
    monkeypatch.setattr(sys, "argv", ["/venv/bin/dimos", *argv])
    prepared_run.launch.side_effect = OSError("interpreter unavailable")

    result = CliRunner().invoke(main, argv)

    assert result.exit_code == 1
    assert "failed to start after preparation: interpreter unavailable" in result.output


def test_split_run_arguments_requires_leading_blueprint_names():
    assert lifecycle.split_run_arguments(
        ("first-blueprint", "second-blueprint", "--map-file", "map")
    ) == (
        ("first-blueprint", "second-blueprint"),
        ("--map-file", "map"),
    )
    with pytest.raises(BlueprintConfigError, match="must precede"):
        lifecycle.split_run_arguments(("--map-file", "map"))
