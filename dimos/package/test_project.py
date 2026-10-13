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

import subprocess

import pytest

from dimos.package.project import add, consumer_project, remove, sync, update


def test_add_delegates_revision_extras_and_restores_failed_resolution(tmp_path, mocker):
    project = consumer_project(tmp_path, create=True)
    manifest = (project / "pyproject.toml").read_bytes()

    def fail(project, *args):
        (project / "pyproject.toml").write_text("broken")
        (project / "uv.lock").write_text("partial")
        raise subprocess.CalledProcessError(7, args)

    call = mocker.patch("dimos.package.project.uv", side_effect=fail)
    with pytest.raises(subprocess.CalledProcessError):
        add(project, "git+https://example.com/repo", tag="v1", extras=("camera",))
    call.assert_called_once_with(
        project,
        "add",
        "--no-workspace",
        "git+https://example.com/repo",
        "--tag",
        "v1",
        "--extra",
        "camera",
    )
    assert (project / "pyproject.toml").read_bytes() == manifest
    assert not (project / "uv.lock").exists()


def test_implicit_project_does_not_edit_dimos_checkout(tmp_path, monkeypatch):
    (tmp_path / "pyproject.toml").write_text('[project]\nname="dimos"\n')
    monkeypatch.chdir(tmp_path)
    with pytest.raises(ValueError, match="Select a consumer"):
        consumer_project(None, create=True)


def test_sync_requires_lock_and_never_relocks(tmp_path, mocker):
    project = consumer_project(tmp_path, create=True)
    run = mocker.patch("dimos.package.project.uv")
    with pytest.raises(ValueError, match="missing"):
        sync(project)
    run.assert_not_called()
    (project / "uv.lock").write_text("locked")
    sync(project)
    run.assert_called_once_with(project, "sync", "--locked", "--no-default-groups")


def test_update_and_remove_delegate_explicit_changes(tmp_path, mocker):
    project = consumer_project(tmp_path, create=True)
    run = mocker.patch("dimos.package.project.uv")
    update(project, ("acme-probe",))
    assert [call.args[1:] for call in run.call_args_list] == [
        ("lock", "--upgrade-package", "acme-probe"),
        ("sync", "--locked", "--no-default-groups"),
    ]
    run.reset_mock()
    remove(project, ("acme-probe",))
    run.assert_called_once_with(project, "remove", "acme-probe")
