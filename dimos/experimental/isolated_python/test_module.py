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

import pytest
from pytest_mock import MockerFixture

from dimos.core.core import rpc
from dimos.experimental.isolated_python.module import (
    IsolatedPythonModule,
    IsolatedPythonModuleConfig,
)


class Contract(IsolatedPythonModule):
    implementation = "runtime:Runtime"
    config: IsolatedPythonModuleConfig

    @rpc
    def value(self) -> int:
        raise NotImplementedError


def test_sibling_project_is_required(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    source = tmp_path / "contract.py"
    source.touch()
    monkeypatch.setattr(
        "dimos.experimental.isolated_python.module.inspect.getfile", lambda _: str(source)
    )
    module = Contract()
    try:
        with pytest.raises(FileNotFoundError, match="sibling 'python/'"):
            module.runtime_project  # noqa: B018
    finally:
        module.stop()


def test_uv_lock_enables_frozen_commands(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    source = tmp_path / "contract.py"
    source.touch()
    project = tmp_path / "python"
    project.mkdir()
    (project / "pyproject.toml").touch()
    (project / "uv.lock").touch()
    checkout = tmp_path / "checkout"
    checkout.mkdir()
    (checkout / "pyproject.toml").touch()
    monkeypatch.setattr(
        "dimos.experimental.isolated_python.module.inspect.getfile", lambda _: str(source)
    )
    monkeypatch.setattr("dimos.experimental.isolated_python.module.DIMOS_PROJECT_ROOT", checkout)
    module = Contract()
    try:
        command = module._launch_command(7)

        assert module._prepare_command() == ["uv", "sync", "--frozen"]
        assert command[:5] == [
            "uv",
            "run",
            "--frozen",
            "--with-editable",
            str(checkout),
        ]
        assert "--python" not in command
    finally:
        module.stop()


def test_installed_host_uses_unversioned_dimos(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "contract.py"
    source.touch()
    project = tmp_path / "python"
    project.mkdir()
    (project / "pyproject.toml").touch()
    installed_root = tmp_path / "site-packages"
    installed_root.mkdir()
    monkeypatch.setattr(
        "dimos.experimental.isolated_python.module.inspect.getfile", lambda _: str(source)
    )
    monkeypatch.setattr(
        "dimos.experimental.isolated_python.module.DIMOS_PROJECT_ROOT", installed_root
    )
    module = Contract()
    try:
        command = module._launch_command(7)

        assert command[:4] == ["uv", "run", "--with", "dimos"]
        assert "--python" not in command
    finally:
        module.stop()


def test_pixi_supplies_uv_when_manifest_exists(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "contract.py"
    source.touch()
    project = tmp_path / "python"
    project.mkdir()
    (project / "pyproject.toml").touch()
    (project / "pixi.toml").touch()
    monkeypatch.setattr(
        "dimos.experimental.isolated_python.module.inspect.getfile", lambda _: str(source)
    )
    module = Contract()
    try:
        assert module._prepare_command() == ["pixi", "run", "--executable", "uv", "sync"]
    finally:
        module.stop()


def test_runtime_environment_uses_sibling_virtualenv(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("VIRTUAL_ENV", "/parent/.venv")
    module = Contract(extra_env={"EXAMPLE_SETTING": "configured"})
    try:
        env = module._runtime_env()

        assert "VIRTUAL_ENV" not in env
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
