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

import json
from pathlib import Path
import subprocess
import sys

import pytest

from dimos.cli.doctor import diagnose, main, probe, project_root


def test_doctor_import_does_not_import_runtime():
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            "import dimos.cli.doctor, sys; assert 'dimos.core.module' not in sys.modules",
        ],
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr


def test_missing_application_reports_action(tmp_path, capsys):
    assert main(["--project-dir", str(tmp_path)]) == 1
    assert "dimup init" in capsys.readouterr().out


def test_find_application_from_nested_directory(tmp_path):
    (tmp_path / "pyproject.toml").write_text(
        '[tool.uv.sources]\ndimos = {git="https://example.com/sdk", rev="abc"}\n'
    )
    nested = tmp_path / "src" / "application"
    nested.mkdir(parents=True)
    assert project_root(nested) == tmp_path


def test_missing_venv_reports_sync_without_writing_files(tmp_path):
    manifest = tmp_path / "pyproject.toml"
    manifest.write_text(
        '[project]\nname="app"\n[tool.uv.sources]\ndimos = {git="https://example.com/sdk", rev="abc"}\n'
    )
    results = diagnose(tmp_path)
    assert (False, "Application environment", "Run uv sync --locked") in results
    assert list(tmp_path.iterdir()) == [manifest]


@pytest.fixture
def application(tmp_path, monkeypatch):
    (tmp_path / "pyproject.toml").write_text(
        '[project]\nname="app"\n[tool.uv.sources]\ndimos = {git="https://example.com/sdk", rev="abc"}\n'
    )
    python = tmp_path / ".venv/bin/python"
    python.parent.mkdir(parents=True)
    python.touch()
    monkeypatch.setattr(sys, "prefix", str(tmp_path / ".venv"))
    monkeypatch.setattr("dimos.cli.doctor.shutil.which", lambda name: f"/usr/bin/{name}")
    return tmp_path


@pytest.mark.parametrize("installed_commit, expected_ok", [("abc", True), ("different", False)])
def test_doctor_checks_pinned_sdk_and_editable_app(
    application, monkeypatch, installed_commit, expected_ok
):
    metadata = {
        "editable": {"dir_info": {"editable": True}, "url": application.as_uri()},
        "sdk": {"vcs_info": {"commit_id": installed_commit}},
        "entries": ["demo"],
    }

    def successful_probe(python, script, *args, cwd):
        assert cwd == application
        return True, json.dumps(metadata) if args else ""

    monkeypatch.setattr("dimos.cli.doctor.probe", successful_probe)
    results = {label: ok for ok, label, detail in diagnose(application)}
    assert results["SDK revision"] is expected_ok
    assert results["Editable application"]
    assert results["Blueprint registration"]
    assert results["Active environment"]
    assert results["Native libraries and image codec"]


def test_doctor_reports_missing_metadata_tools_and_libraries(application, monkeypatch):
    monkeypatch.setattr(
        "dimos.cli.doctor.probe", lambda *args, **kwargs: (False, "dependency absent")
    )
    monkeypatch.setattr("dimos.cli.doctor.shutil.which", lambda name: None)
    results = {label: (ok, detail) for ok, label, detail in diagnose(application)}
    assert not results["Installed metadata"][0]
    assert "uv sync --locked" in results["Installed metadata"][1]
    assert not results["cargo"][0]
    assert "dimup setup" in results["cargo"][1]
    assert results["Native libraries and image codec"] == (False, "dependency absent")


def test_probe_reports_subprocess_failure_and_uses_application_directory(tmp_path):
    ok, detail = probe(
        Path(sys.executable),
        "from pathlib import Path; import sys; print(Path.cwd()); sys.exit(3)",
        cwd=tmp_path,
    )
    assert not ok
    assert detail == str(tmp_path)


def test_probe_reports_missing_interpreter(tmp_path):
    ok, detail = probe(tmp_path / "missing-python", "", cwd=tmp_path)
    assert not ok
    assert "missing-python" in detail


def test_probe_reports_timeout(tmp_path, monkeypatch):
    def timeout(*args, **kwargs):
        raise subprocess.TimeoutExpired("python", 45)

    monkeypatch.setattr("dimos.cli.doctor.subprocess.run", timeout)
    ok, detail = probe(Path(sys.executable), "", cwd=tmp_path)
    assert not ok
    assert "45" in detail
