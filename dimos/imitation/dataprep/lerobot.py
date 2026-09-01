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

"""Launch native LeRobot conversion in the policy runtime environment."""

from __future__ import annotations

import inspect
import os
from pathlib import Path
import subprocess
import tempfile

from dimos.core.isolated_python_module import isolated_python_run_command
from dimos.imitation.dataprep.core import DataPrepConfig
from dimos.imitation.policy.lerobot.module import LeRobotPolicyModule
from dimos.utils.cache import cache_usage_guard


def lerobot_project() -> Path:
    """Locate the packaged LeRobot project beside its host contract."""
    source = Path(inspect.getfile(LeRobotPolicyModule)).resolve()
    return source.parent / "python"


def run_lerobot_dataprep(config: DataPrepConfig) -> Path:
    """Run conversion under the locked LeRobot dependency stack."""
    project = lerobot_project()
    command: list[str]
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", encoding="utf-8") as config_file:
        config_file.write(config.model_dump_json())
        config_file.flush()
        command = isolated_python_run_command(
            project,
            "python",
            "-m",
            "dimos_lerobot.dataprep",
            config_file.name,
        )
        env = dict(os.environ)
        env.pop("VIRTUAL_ENV", None)
        try:
            with cache_usage_guard():
                result = subprocess.run(
                    command,
                    cwd=project,
                    env=env,
                    capture_output=True,
                    text=True,
                )
        except FileNotFoundError as error:
            raise RuntimeError(
                "uv is required for LeRobot conversion; install uv and ensure it is on PATH"
            ) from error
    if result.returncode:
        output = (result.stdout + "\n" + result.stderr).strip()
        raise RuntimeError(f"LeRobot conversion exited with status {result.returncode}: {output}")
    return config.output.path
