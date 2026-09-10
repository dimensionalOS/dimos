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

import os
from pathlib import Path
import shutil
import subprocess
import sys
import venv

from dimup.process import Runner, SetupError
from dimup.project import create, manifest, package_name, resolve_sdk, write_project
import pytest
import tomllib


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
    assert config["tool"]["uv"]["python-preference"] == "only-managed"
    assert "APP_MODULE" not in (tmp_path / "tests/test_demo.py").read_text()


@pytest.mark.parametrize("name", ["123", "dimos", "dimup"])
def test_rejects_invalid_package_name(name):
    with pytest.raises(SetupError):
        package_name(Path(name))


def test_nonempty_destination_is_untouched(tmp_path):
    existing = tmp_path / "notes"
    existing.write_text("my work")
    with pytest.raises(SetupError, match="new or empty"):
        create(tmp_path, "main")
    assert list(tmp_path.iterdir()) == [existing]
    assert existing.read_text() == "my work"


def test_init_on_arch_installs_and_verifies_application(tmp_path, monkeypatch):
    monkeypatch.setattr("platform.system", lambda: "Linux")
    monkeypatch.setattr("platform.machine", lambda: "x86_64")
    monkeypatch.setattr("platform.freedesktop_os_release", lambda: {"ID": "arch"})
    monkeypatch.setattr(
        "dimup.project.executable",
        lambda name: f"/usr/bin/{name}" if name != "deno" else pytest.fail("Deno is optional"),
    )
    monkeypatch.setattr(
        "dimup.project.resolve_sdk",
        lambda ref, runner: ("a" * 40, {"project": {"optional-dependencies": {"all": []}}}),
    )
    commands = []

    def run(self, stage, command, **kwargs):
        commands.append(command)
        return ""

    monkeypatch.setattr("dimup.project.Runner.run", run)
    create(tmp_path / "my-robot", "main")
    assert (tmp_path / "my-robot/pyproject.toml").is_file()
    assert commands[0] == ["/usr/bin/uv", "sync", "--python", "3.12"]
    assert commands[1][0] == str(tmp_path / "my-robot/.venv/bin/python")


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
    write_project(
        tmp_path, "my-robot", "a" * 40, {"project": {"optional-dependencies": {"all": []}}}
    )
    venv.EnvBuilder(with_pip=False).create(tmp_path / ".venv")
    env = {**os.environ, "XDG_CONFIG_HOME": str(tmp_path / "config")}
    subprocess.run(["direnv", "allow", str(tmp_path)], env=env, check=True, capture_output=True)
    result = subprocess.run(
        ["direnv", "exec", str(tmp_path), "python", "-c", "import sys; print(sys.prefix)"],
        env=env,
        check=True,
        capture_output=True,
        text=True,
    )
    assert result.stdout.strip() == str(tmp_path / ".venv")


@pytest.mark.parametrize("exit_code", [0, 7])
def test_init_presents_live_installation_and_preserves_failures(
    tmp_path, monkeypatch, capsys, exit_code
):
    root = tmp_path / "my [robot]"
    sha = "a" * 40
    monkeypatch.setenv("COLUMNS", "40")
    monkeypatch.setenv("NO_COLOR", "1")
    monkeypatch.setattr("dimup.project.executable", lambda name: name)
    monkeypatch.setattr(
        "dimup.project.resolve_sdk",
        lambda ref, runner: (sha, {"project": {"optional-dependencies": {"all": []}}}),
    )
    popen = subprocess.Popen

    def run_tool(command, **kwargs):
        script = (
            f"import sys; print('Installed 2 packages', file=sys.stderr); sys.exit({exit_code})"
            if command[0] == "uv"
            else "print('application registered')"
        )
        return popen([sys.executable, "-c", script], **kwargs)

    monkeypatch.setattr("dimup.process.subprocess.Popen", run_tool)
    if exit_code:
        with pytest.raises(SetupError, match="exit 7") as error:
            create(root, sha)
        assert "directory has been kept" in str(error.value)
        assert "Ready" not in capsys.readouterr().out
    else:
        create(root, sha)
        output = capsys.readouterr().out
        assert "Installed 2 packages" in output
        assert "Ready · my-robot" in output
        assert f"cd '{root}'" in output
        assert "dimos run my-robot.demo" in output
        assert "direnv allow" in output
        assert "→" not in output
        assert "\x1b" not in output
    assert (root / "pyproject.toml").is_file()
    assert "Installed 2 packages" in (root / ".dimos/setup.log").read_text()


def test_sdk_resolution_keeps_git_metadata_out_of_console(tmp_path, monkeypatch, capsys):
    sdk = tmp_path / "sdk"
    sdk.mkdir()
    (sdk / "pyproject.toml").write_text('[project]\nname = "metadata-only"\n')
    subprocess.run(["git", "init", "--quiet", str(sdk)], check=True)
    subprocess.run(["git", "-C", str(sdk), "add", "pyproject.toml"], check=True)
    subprocess.run(
        [
            "git",
            "-C",
            str(sdk),
            "-c",
            "user.name=Test",
            "-c",
            "user.email=test@example.com",
            "-c",
            "commit.gpgsign=false",
            "-c",
            "core.hooksPath=/dev/null",
            "commit",
            "--quiet",
            "-m",
            "SDK",
        ],
        check=True,
    )
    monkeypatch.setattr("dimup.project.SDK_URL", str(sdk))
    runner = Runner(tmp_path / "setup.log")
    with runner.stage("Resolve SDK"):
        sha, metadata = resolve_sdk("HEAD", runner)
    assert len(sha) == 40
    assert metadata == {"project": {"name": "metadata-only"}}
    output = capsys.readouterr().out
    assert "Read SDK dependencies" not in output
    assert "metadata-only" not in output
    assert "metadata-only" in runner.log.read_text()
