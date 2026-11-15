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
import sys

import pytest

from dimos.utils.executable import find_executable


@pytest.fixture
def tool_dirs(monkeypatch, tmp_path):
    venv = tmp_path / "venv" / "bin"
    venv.mkdir(parents=True)
    path_dir = tmp_path / "path"
    path_dir.mkdir()
    (venv / "python").symlink_to(sys.executable)
    monkeypatch.setattr(sys, "executable", str(venv / "python"))
    monkeypatch.setenv("PATH", str(path_dir))
    return venv, path_dir


@pytest.mark.parametrize("on_path", [True, False])
def test_prefers_tool_beside_interpreter(tool_dirs, on_path):
    venv, path_dir = tool_dirs
    sibling = venv / "rerun"
    sibling.touch(mode=0o755)
    if on_path:
        (path_dir / "rerun").touch(mode=0o755)

    assert find_executable("rerun") == str(sibling)
    assert os.environ["PATH"] == str(path_dir)


@pytest.mark.parametrize("sibling_mode", [None, 0o644])
def test_falls_back_to_path_when_sibling_is_unavailable(tool_dirs, sibling_mode):
    venv, path_dir = tool_dirs
    if sibling_mode is not None:
        (venv / "rerun").touch(mode=sibling_mode)
    on_path = path_dir / "rerun"
    on_path.touch(mode=0o755)

    assert find_executable("rerun") == str(on_path)


def test_missing_tool_returns_none(tool_dirs):
    assert find_executable("rerun") is None


@pytest.mark.parametrize("executable", ["", None])
def test_missing_interpreter_uses_path(tool_dirs, monkeypatch, executable):
    _, path_dir = tool_dirs
    on_path = path_dir / "rerun"
    on_path.touch(mode=0o755)
    monkeypatch.setattr(sys, "executable", executable)

    assert find_executable("rerun") == str(on_path)
