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

import pytest

from dimos.cli.entry import main
from dimos.package.dispatch import dispatch
from dimos.package.project import consumer_project


@pytest.mark.parametrize("command", [["list"], ["--viewer", "none", "run", "acme-probe.probe"]])
def test_selected_project_dispatches_before_runtime_imports(tmp_path, mocker, monkeypatch, command):
    project = consumer_project(tmp_path, create=True)
    (project / "uv.lock").write_text("locked")
    monkeypatch.setenv("PYTHONPATH", "/wrong/checkout")
    monkeypatch.setenv("UV_PROJECT_ENVIRONMENT", "/wrong/env")
    execute = mocker.patch("dimos.package.dispatch.os.execvpe", side_effect=SystemExit(0))
    with pytest.raises(SystemExit):
        dispatch(["--project", str(project), *command])
    executable, arguments, environment = execute.call_args.args
    assert executable == "uv"
    assert arguments[:7] == [
        "uv",
        "run",
        "--project",
        str(project),
        "--locked",
        "--no-default-groups",
        "python",
    ]
    assert arguments[7] == "-I"
    assert arguments[-len(command) :] == command
    assert "PYTHONPATH" not in environment
    assert "UV_PROJECT_ENVIRONMENT" not in environment
    assert environment["DIMOS_CONSUMER_PROJECT"] == str(project)


def test_missing_lock_does_not_run_global_dimos(tmp_path, mocker):
    consumer_project(tmp_path, create=True)
    execute = mocker.patch("dimos.package.dispatch.os.execvpe")
    with pytest.raises(ValueError, match="lock is missing"):
        dispatch(["--project", str(tmp_path), "run", "acme-probe.probe"])
    execute.assert_not_called()


def test_dispatched_child_does_not_recurse(monkeypatch, mocker):
    monkeypatch.setenv("DIMOS_CONSUMER_PROJECT", "/consumer")
    execute = mocker.patch("dimos.package.dispatch.os.execvpe")
    assert dispatch(["list"]) == ["list"]
    execute.assert_not_called()


def test_package_help_never_imports_robot_cli(mocker, monkeypatch):
    monkeypatch.setattr(sys, "argv", ["dimos", "package", "--help"])
    mocker.patch.dict(sys.modules, {"dimos.cli.dimos": None})

    with pytest.raises(SystemExit) as error:
        main()
    assert error.value.code == 0
