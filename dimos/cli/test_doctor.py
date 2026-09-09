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

from dimos.cli import doctor


def test_doctor_dispatch_does_not_import_runtime_dependencies(tmp_path):
    code = "from dimos.cli.entrypoint import cli_main; cli_main()"
    result = subprocess.run(
        [sys.executable, "-S", "-c", code, "doctor", "--project-dir", str(tmp_path)],
        cwd=Path(__file__).resolve().parents[2],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 1
    assert "No workspace configuration found" in result.stdout
    assert result.stderr == ""


def test_doctor_reports_multiple_failures_and_optional_direnv(tmp_path, monkeypatch):
    state = tmp_path / ".dimos"
    state.mkdir()
    (tmp_path / "pyproject.toml").write_text('[project]\nname = "my-robot"\n')
    (state / "project.toml").write_text(
        'mode = "sdk"\nprofile = "navigation"\npackage = "my-robot"\n'
    )
    (state / "tools.json").write_text(json.dumps({"uv": "/uv", "pixi": "/pixi", "nix": "/nix"}))
    monkeypatch.setattr(doctor, "probe", lambda command: "broken dependency")
    monkeypatch.setattr(doctor.shutil, "which", lambda name: None)
    results = doctor.diagnose(tmp_path)
    failed = {label for status, label, _ in results if status == "FAIL"}
    assert {
        "uv",
        "Native libraries",
        "Profile dependencies",
        "Editable project and blueprints",
    } <= failed
    assert ("INFO", "Optional direnv") in [(status, label) for status, label, _ in results]
    assert sorted(path.name for path in state.iterdir()) == ["project.toml", "tools.json"]


def test_probe_reports_crashes_and_timeouts():
    assert "exit 7" in doctor.probe([sys.executable, "-c", "raise SystemExit(7)"])
    assert "timed out" in doctor.probe([sys.executable, "-c", "while True: pass"], timeout=0.1)


def test_corrupt_configuration_is_reported(tmp_path):
    state = tmp_path / ".dimos"
    state.mkdir()
    (state / "project.toml").write_text("broken = [")
    results = doctor.diagnose(tmp_path)
    assert results[0][:2] == ("FAIL", "Workspace configuration")
