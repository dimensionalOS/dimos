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
import shlex
import shutil
import subprocess
import sys

import pytest

RESOURCES = Path(__file__).resolve().parents[1] / "resources"


@pytest.fixture
def project(tmp_path):
    root = tmp_path / "a project's workspace"
    state = root / ".dimos"
    state.mkdir(parents=True)
    venv = root / ".venv/bin"
    venv.mkdir(parents=True)
    (venv / "python").symlink_to(sys.executable)
    (venv / "activate").write_text(
        f"export VIRTUAL_ENV={shlex.quote(str(root / '.venv'))}\n"
        f'export PATH={shlex.quote(str(venv))}:"$PATH"\n'
    )
    pixi = root / "pixi"
    hook = (
        f"export CONDA_PREFIX={shlex.quote(str(state / '.pixi/envs/default'))}\nexport CC=pixi-cc\n"
    )
    pixi.write_text(f"#!/bin/sh\nprintf '%s' {shlex.quote(hook)}\n")
    pixi.chmod(0o755)
    (state / "tools.json").write_text(
        json.dumps({"pixi": str(pixi), "uv": "/tools/uv", "nix": "/nix/bin/nix"})
    )
    for name in ("activate.sh", "environment.py"):
        shutil.copyfile(RESOURCES / name, state / name)
    return root


@pytest.mark.parametrize("shell", ["bash", "zsh"])
def test_activation_is_repeatable_and_deactivation_restores_environment(project, shell):
    executable = shutil.which(shell)
    if executable is None:
        pytest.skip(f"{shell} is unavailable")
    dump = shlex.quote("import os,json; print(json.dumps(dict(os.environ)))")
    python = shlex.quote(sys.executable)
    activate = shlex.quote(str(project / ".dimos/activate.sh"))
    script = f"""set -e
export CPPFLAGS=original LD_LIBRARY_PATH=/original
{python} -c {dump}
source {activate}
{python} -c {dump}
source {activate}
{python} -c {dump}
deactivate
{python} -c {dump}
"""
    result = subprocess.run(
        [executable, "-f", "-c", script],
        cwd=project.parent,
        capture_output=True,
        text=True,
        check=True,
    )
    before, active, repeated, after = [json.loads(line) for line in result.stdout.splitlines()]
    assert active["VIRTUAL_ENV"] == str(project / ".venv")
    assert active["CC"] == "pixi-cc"
    assert active["CPPFLAGS"].count("-idirafter /usr/include") == 1
    for environment in (before, active, repeated, after):
        for key in ("_", "SHLVL", "PWD", "OLDPWD"):
            environment.pop(key, None)
    assert active == repeated
    assert after == before


def test_missing_environment_fails_without_installing(tmp_path):
    state = tmp_path / ".dimos"
    state.mkdir()
    shutil.copyfile(RESOURCES / "activate.sh", state / "activate.sh")
    result = subprocess.run(
        ["bash", "-c", 'source "$1"', "test", str(state / "activate.sh")],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 1
    assert "--restore" in result.stderr
    assert not (tmp_path / ".venv").exists()


def test_direnv_loads_and_unloads_the_same_activation(project, monkeypatch):
    executable = shutil.which("direnv")
    if executable is None:
        pytest.skip("direnv is unavailable")
    for variable in ("XDG_CONFIG_HOME", "XDG_DATA_HOME", "XDG_CACHE_HOME"):
        monkeypatch.setenv(variable, str(project.parent / variable))
    shutil.copyfile(RESOURCES / "envrc.example", project / ".envrc")
    subprocess.run([executable, "allow", str(project)], check=True)
    result = subprocess.run(
        [
            executable,
            "exec",
            str(project),
            sys.executable,
            "-c",
            "import os,json; print(json.dumps(dict(os.environ)))",
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    environment = json.loads(result.stdout)
    assert environment["VIRTUAL_ENV"] == str(project / ".venv")
    assert environment["CC"] == "pixi-cc"

    dump = shlex.quote(
        "import os,json; print(json.dumps({k:os.environ.get(k) for k in ('PATH','VIRTUAL_ENV','CONDA_PREFIX','CPPFLAGS','LD_LIBRARY_PATH')}))"
    )
    python = shlex.quote(sys.executable)
    script = f"""set -e
{python} -c {dump}
cd {shlex.quote(str(project))}
eval \"$({shlex.quote(executable)} export bash)\"
{python} -c {dump}
cd ..
eval \"$({shlex.quote(executable)} export bash)\"
{python} -c {dump}
"""
    loaded = subprocess.run(
        ["bash", "--noprofile", "--norc", "-c", script],
        cwd=project.parent,
        capture_output=True,
        text=True,
        check=True,
    )
    before, active, after = [json.loads(line) for line in loaded.stdout.splitlines()]
    assert active["VIRTUAL_ENV"] == str(project / ".venv")
    assert after == before
