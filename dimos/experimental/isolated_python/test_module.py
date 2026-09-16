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

from dimos.core.core import rpc
from dimos.experimental.isolated_python.module import (
    IsolatedPythonModule,
    IsolatedPythonModuleConfig,
    isolated_python_environment,
    isolated_python_run_command,
    prepare_isolated_python,
)
from dimos.utils import data


class Contract(IsolatedPythonModule):
    project_dir = "native/python/test"
    implementation = "runtime:Runtime"
    config: IsolatedPythonModuleConfig

    @rpc
    def value(self) -> int:
        raise NotImplementedError


@pytest.fixture
def project(tmp_path, monkeypatch):
    runtime = tmp_path / Contract.project_dir
    runtime.mkdir(parents=True)
    (runtime / "pyproject.toml").touch()
    (runtime / "uv.lock").touch()
    monkeypatch.setattr(
        "dimos.experimental.isolated_python.module.get_project_root", lambda: tmp_path
    )
    return runtime


def test_checkout_resolution_is_lazy(mocker):
    root = mocker.patch("dimos.experimental.isolated_python.module.get_project_root")
    module = Contract()
    try:
        Contract.blueprint()
        root.assert_not_called()
    finally:
        module.stop()


@pytest.mark.parametrize("manifest", [False, True])
def test_missing_runtime_reports_expected_path(tmp_path, monkeypatch, manifest):
    project = tmp_path / Contract.project_dir
    if manifest:
        project.mkdir(parents=True)
    monkeypatch.setattr(
        "dimos.experimental.isolated_python.module.get_project_root", lambda: tmp_path
    )
    module = Contract()
    try:
        with pytest.raises(FileNotFoundError, match=str(project)):
            module.runtime_project  # noqa: B018
    finally:
        module.stop()


@pytest.mark.parametrize("installed", [False, True])
def test_runtime_uses_shared_checkout(tmp_path, monkeypatch, installed):
    checkout = tmp_path / "repo"
    (checkout / ".git").mkdir(parents=True)
    project = checkout / Contract.project_dir
    project.mkdir(parents=True)
    (project / "pyproject.toml").touch()
    monkeypatch.setattr(
        data, "DIMOS_PROJECT_ROOT", tmp_path / "site-packages" if installed else checkout
    )
    monkeypatch.setattr(data, "_get_user_data_dir", lambda: tmp_path)
    data.get_project_root.cache_clear()
    module = Contract()
    try:
        assert module.runtime_project == project
        assert isolated_python_run_command(project, "python") == [
            "uv",
            "run",
            "--no-sync",
            "python",
        ]
    finally:
        module.stop()
        data.get_project_root.cache_clear()


def test_prepare_installs_host_code_without_changing_runtime_dependencies(tmp_path, mocker):
    checkout = tmp_path / "checkout"
    checkout.mkdir()
    (checkout / "pyproject.toml").touch()
    mocker.patch(
        "dimos.experimental.isolated_python.module.get_project_root", return_value=checkout
    )
    run = mocker.patch("dimos.experimental.isolated_python.module.subprocess.run")
    run.return_value.returncode = 0

    prepare_isolated_python(tmp_path, isolated_python_environment(tmp_path))

    assert [call.args[0] for call in run.call_args_list] == [
        ["uv", "sync", "--frozen"],
        [
            "uv",
            "pip",
            "install",
            "--python",
            str(
                Path(isolated_python_environment(tmp_path)["UV_PROJECT_ENVIRONMENT"]) / "bin/python"
            ),
            "--no-deps",
            "--editable",
            str(checkout),
        ],
    ]
    assert isolated_python_run_command(tmp_path, "python", "script.py") == [
        "uv",
        "run",
        "--no-sync",
        "python",
        "script.py",
    ]


def test_failed_sync_does_not_install_host(tmp_path, mocker):
    run = mocker.patch("dimos.experimental.isolated_python.module.subprocess.run")
    run.return_value.returncode = 1
    run.return_value.stdout = ""
    run.return_value.stderr = "unsatisfiable dependencies"

    with pytest.raises(RuntimeError, match="unsatisfiable dependencies"):
        prepare_isolated_python(tmp_path, isolated_python_environment(tmp_path))

    assert run.call_count == 1


def test_pixi_wraps_preparation_and_launch(tmp_path, mocker):
    (tmp_path / "pixi.toml").touch()
    run = mocker.patch("dimos.experimental.isolated_python.module.subprocess.run")
    run.return_value.returncode = 0

    prepare_isolated_python(tmp_path, isolated_python_environment(tmp_path))

    assert all(
        call.args[0][:4] == ["pixi", "run", "--executable", "uv"] for call in run.call_args_list
    )
    assert isolated_python_run_command(tmp_path, "python") == [
        "pixi",
        "run",
        "--executable",
        "uv",
        "run",
        "--no-sync",
        "python",
    ]


def test_runtime_environment_uses_project_specific_cache(project, tmp_path, monkeypatch):
    monkeypatch.setenv("VIRTUAL_ENV", "/parent/.venv")
    monkeypatch.setenv("UV_PYTHON", "3.10")
    monkeypatch.setenv("UV_PROJECT_ENVIRONMENT", "/parent/.venv")
    monkeypatch.setattr("dimos.experimental.isolated_python.module.CACHE_DIR", tmp_path / "cache")
    module = Contract(extra_env={"EXAMPLE_SETTING": "configured"})
    try:
        env = module._runtime_env()
        assert "VIRTUAL_ENV" not in env
        assert "UV_PYTHON" not in env
        assert Path(env["UV_PROJECT_ENVIRONMENT"]).is_relative_to(tmp_path / "cache")
        assert module._runtime_env()["UV_PROJECT_ENVIRONMENT"] == env["UV_PROJECT_ENVIRONMENT"]
        other = tmp_path / "native/python/other"
        other.mkdir()
        (other / "pyproject.toml").touch()
        monkeypatch.setattr(Contract, "project_dir", "native/python/other")
        assert module._runtime_env()["UV_PROJECT_ENVIRONMENT"] != env["UV_PROJECT_ENVIRONMENT"]
        assert env["EXAMPLE_SETTING"] == "configured"
    finally:
        module.stop()


def test_host_build_prepares_and_builds_runtime(mocker: MockerFixture) -> None:
    module = Contract()
    prepare = mocker.patch.object(module, "_run_prepare")
    spawn = mocker.patch.object(module, "_spawn_runtime")
    runtime_client = mocker.Mock()
    connect = mocker.patch.object(
        module,
        "_connect_runtime",
        side_effect=lambda: setattr(module, "_runtime_client", runtime_client),
    )
    try:
        module.build()

        prepare.assert_called_once_with()
        spawn.assert_called_once_with()
        connect.assert_called_once_with()
        runtime_client.build.assert_called_once_with()
    finally:
        module.stop()


def test_runtime_build_skips_environment_preparation(mocker: MockerFixture) -> None:
    module = Contract(_isolated_python_runtime=True)
    prepare = mocker.patch.object(module, "_run_prepare")
    spawn = mocker.patch.object(module, "_spawn_runtime")
    try:
        module.build()

        prepare.assert_not_called()
        spawn.assert_not_called()
    finally:
        module.stop()


def test_preparation_warms_the_launch_environment(project: Path, mocker: MockerFixture) -> None:
    run = mocker.patch(
        "dimos.experimental.isolated_python.module.subprocess.run",
        return_value=subprocess.CompletedProcess([], 0, "", ""),
    )
    module = Contract()
    try:
        module._run_prepare()

        assert [call.args[0] for call in run.call_args_list] == [
            isolated_python_run_command(project, "python", "-c", "pass"),
        ]
        assert module._launch_command(7)[:5] == run.call_args.args[0][:5]
        for call in run.call_args_list:
            assert call.kwargs["cwd"] == project
            assert call.kwargs["env"] == module._runtime_env()
    finally:
        module.stop()


def test_preparation_failure_prevents_launch(project: Path, mocker: MockerFixture) -> None:
    mocker.patch(
        "dimos.experimental.isolated_python.module.subprocess.run",
        return_value=subprocess.CompletedProcess([], 1, "", "dependency unavailable"),
    )
    module = Contract()
    spawn = mocker.patch.object(module, "_spawn_runtime")
    try:
        with pytest.raises(RuntimeError, match="dependency unavailable"):
            module.build()
        spawn.assert_not_called()
    finally:
        module.stop()
