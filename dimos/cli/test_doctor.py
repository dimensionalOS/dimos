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
import sys

from dimos.cli.doctor import diagnose, main, project_root


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
