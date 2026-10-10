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

"""Normal CLI startup works after the robot image excludes private packages."""

import os
from pathlib import Path
import shutil
import subprocess
import sys


def test_robot_cli_starts_without_private_eval_or_simulator_package(tmp_path):
    project = Path(__file__).resolve().parents[3]
    shutil.copytree(
        project / "dimos",
        tmp_path / "dimos",
        ignore=shutil.ignore_patterns("__pycache__", "test_*.py", "conftest.py"),
    )
    for relative in ("evals", "simulation", "experimental/scene_cooking"):
        shutil.rmtree(tmp_path / "dimos" / relative)
    dependency_paths = [p for p in sys.path if p and not Path(p).resolve().is_relative_to(project)]
    env = dict(
        os.environ,
        PYTHONPATH=os.pathsep.join([str(tmp_path), *dependency_paths]),
        XDG_STATE_HOME=str(tmp_path / "state"),
        XDG_CACHE_HOME=str(tmp_path / "cache"),
    )
    result = subprocess.run(
        [sys.executable, "-c", "from dimos.cli.entry import main; main()", "--help"],
        cwd=tmp_path,
        env=env,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr
    assert "evals" not in result.stdout
    assert "run" in result.stdout
