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
from types import SimpleNamespace

import pytest

from dimup.process import Runner, SetupError
from dimup.shell import configure_lfs


@pytest.fixture
def home(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.delenv("ZDOTDIR", raising=False)
    monkeypatch.delenv("XDG_CONFIG_HOME", raising=False)
    monkeypatch.delenv("GIT_LFS_SKIP_SMUDGE", raising=False)
    return tmp_path


@pytest.mark.parametrize(
    ("shell", "relative"),
    [("bash", ".bashrc"), ("zsh", ".zshrc"), ("fish", ".config/fish/config.fish")],
)
def test_shell_setting_preserves_content_and_is_idempotent(home, monkeypatch, shell, relative):
    monkeypatch.setenv("SHELL", f"/bin/{shell}")
    path = home / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("# user's configuration")
    runner = Runner(home / "setup.log")
    configure_lfs(runner)
    first = path.read_text()
    configure_lfs(runner)
    assert path.read_text() == first
    assert first.startswith("# user's configuration\n")
    assert first.count("# >>> dimup Git LFS >>>") == 1
    binary = shutil.which(shell)
    if binary is None:
        pytest.skip(f"{shell} is not installed")
    result = subprocess.run(
        [binary, "-c", 'source "$DIMUP_TEST_CONFIG"; printenv GIT_LFS_SKIP_SMUDGE'],
        env={**os.environ, "DIMUP_TEST_CONFIG": str(path)},
        capture_output=True,
        text=True,
        check=True,
    )
    assert result.stdout == "1\n"


@pytest.mark.parametrize("profile", [".bash_profile", ".bash_login", ".profile"])
def test_bash_uses_existing_login_profile(home, monkeypatch, profile):
    monkeypatch.setenv("SHELL", "/bin/bash")
    (home / profile).touch()
    configure_lfs(Runner(home / "setup.log"))
    assert "export GIT_LFS_SKIP_SMUDGE=1" in (home / profile).read_text()
    assert sorted(path.name for path in home.iterdir()) == sorted([".bashrc", profile])


@pytest.mark.parametrize(
    ("shell", "variable", "relative"),
    [("zsh", "ZDOTDIR", ".zshrc"), ("fish", "XDG_CONFIG_HOME", "fish/config.fish")],
)
def test_custom_shell_config_directory(home, monkeypatch, shell, variable, relative):
    monkeypatch.setenv("SHELL", f"/bin/{shell}")
    monkeypatch.setenv(variable, str(home / "custom"))
    configure_lfs(Runner(home / "setup.log"))
    assert "GIT_LFS_SKIP_SMUDGE" in (home / "custom" / relative).read_text()


def test_login_shell_fallback_and_default_bash_profile(home, monkeypatch):
    monkeypatch.delenv("SHELL", raising=False)
    monkeypatch.setattr(
        "dimup.shell.pwd.getpwuid", lambda uid: SimpleNamespace(pw_shell="/bin/bash")
    )
    configure_lfs(Runner(home / "setup.log"))
    assert "export GIT_LFS_SKIP_SMUDGE=1" in (home / ".profile").read_text()


def test_unknown_shell_prints_manual_command_without_writing(home, monkeypatch, capsys):
    monkeypatch.setenv("SHELL", "/bin/nu")
    configure_lfs(Runner(home / "setup.log"))
    assert "not configured automatically" in capsys.readouterr().out
    assert list(home.iterdir()) == []


def test_config_write_error_names_file(home, monkeypatch):
    monkeypatch.setenv("SHELL", "/bin/bash")
    (home / ".bashrc").mkdir()
    with pytest.raises(SetupError, match=r"Cannot configure Git LFS in .*\.bashrc"):
        configure_lfs(Runner(home / "setup.log"))
