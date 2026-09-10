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
import os
import shutil
import subprocess
import sys
import venv

import pytest
import tomllib

from dimup.process import SetupError
from dimup.project import create, manifest, package_name, write_project


def test_generated_application_pins_sdk_and_registers_blueprint(tmp_path):
    write_project(
        tmp_path,
        "my-robot",
        "a" * 40,
        {"project": {"optional-dependencies": {"all": [], "spot": [], "dds": []}}},
    )
    config = tomllib.loads((tmp_path / "pyproject.toml").read_text())
    assert config["project"]["entry-points"]["dimos.blueprints"] == {
        "demo": "my_robot.demo:blueprint"
    }
    assert config["tool"]["uv"]["sources"]["dimos"]["rev"] == "a" * 40
    assert config["project"]["dependencies"] == ["dimos[all,spot]"]
    assert "APP_MODULE" not in (tmp_path / "tests/test_demo.py").read_text()


@pytest.mark.parametrize("name", ["123", "dimos", "dimup"])
def test_rejects_invalid_package_name(name):
    with pytest.raises(SetupError):
        package_name(Path(name))


def test_nonempty_destination_is_untouched(tmp_path, monkeypatch):
    monkeypatch.setattr("dimup.project.supported_platform", lambda: "ubuntu")
    existing = tmp_path / "notes"
    existing.write_text("my work")
    with pytest.raises(SetupError, match="new or empty"):
        create(tmp_path, "main")
    assert list(tmp_path.iterdir()) == [existing]
    assert existing.read_text() == "my work"


def test_init_requires_directory():
    result = subprocess.run(
        [sys.executable, "-m", "dimup.cli", "init"], capture_output=True, check=False
    )
    assert result.returncode == 2


def test_source_backed_dependencies_are_direct_requirements():
    sdk = {
        "project": {"optional-dependencies": {"all": [], "graspgenx": []}},
        "tool": {
            "uv": {"sources": {"graspgenx": {"git": "https://example.com/grasp", "rev": "abc"}}}
        },
    }
    result = manifest("my-robot", "a" * 40, sdk)
    assert result["project"]["dependencies"] == ["dimos[all,graspgenx]", "graspgenx"]


@pytest.mark.parametrize("shell", ["bash", "zsh"])
def test_activation_restores_environment_and_handles_spaces(tmp_path, shell):
    root = tmp_path / "my robot"
    root.mkdir()
    write_project(root, "my-robot", "a" * 40, {"project": {"optional-dependencies": {"all": []}}})
    venv.EnvBuilder(with_pip=False).create(root / ".venv")
    script = """set -e
before=$PATH
unset VIRTUAL_ENV NIX_CONFIG
source "$1/.dimos/activate.sh"
test "$VIRTUAL_ENV" = "$1/.venv"
test "$(command -v python)" = "$1/.venv/bin/python"
test -n "$NIX_CONFIG"
deactivate
test "$PATH" = "$before"
test -z "${VIRTUAL_ENV+x}"
test -z "${NIX_CONFIG+x}"
"""
    result = subprocess.run(
        [shell, "-f", "-c", script, "test", str(root)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr


def test_direnv_selects_application_python(tmp_path):
    if shutil.which("direnv") is None:
        pytest.skip("direnv integration is exercised by CI with direnv installed")
    write_project(tmp_path, "my-robot", "a" * 40, {"project": {"optional-dependencies": {"all": []}}})
    venv.EnvBuilder(with_pip=False).create(tmp_path / ".venv")
    env = {**os.environ, "XDG_CONFIG_HOME": str(tmp_path / "config")}
    subprocess.run(["direnv", "allow", str(tmp_path)], env=env, check=True, capture_output=True)
    result = subprocess.run(["direnv", "exec", str(tmp_path), "python", "-c", "import sys; print(sys.prefix)"],
                            env=env, check=True, capture_output=True, text=True)
    assert result.stdout.strip() == str(tmp_path / ".venv")
