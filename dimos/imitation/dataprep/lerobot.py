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
from typing import Any

from dimos.experimental.isolated_python.module import isolated_python_run_command
from dimos.imitation.dataprep._lerobot_protocol import (
    RESULT_ADAPTER,
    BuildRequest,
    BuildResult,
    InspectRequest,
    InspectResult,
    Request,
    Result,
)
from dimos.imitation.dataprep.core import DataPrepConfig
from dimos.imitation.policy.lerobot.module import LeRobotPolicyModule
from dimos.utils.cache import cache_usage_guard


def lerobot_project() -> Path:
    """Locate the packaged LeRobot project beside its host contract."""
    source = Path(inspect.getfile(LeRobotPolicyModule)).resolve()
    return source.parent / "python"


def _run(request: Request) -> Result:
    """Run one typed request under the locked LeRobot dependency stack."""
    project = lerobot_project()
    command = isolated_python_run_command(
        project,
        "python",
        "-m",
        "dimos_lerobot.dataprep",
    )
    env = dict(os.environ)
    env.pop("VIRTUAL_ENV", None)
    try:
        with cache_usage_guard():
            result = subprocess.run(
                command,
                cwd=project,
                env=env,
                input=request.model_dump_json(),
                capture_output=True,
                text=True,
            )
    except FileNotFoundError as error:
        raise RuntimeError(
            "uv is required for LeRobot dataprep; install uv and ensure it is on PATH"
        ) from error
    if result.returncode:
        output = result.stderr.strip() or result.stdout.strip()
        raise RuntimeError(f"LeRobot dataprep exited with status {result.returncode}: {output}")
    try:
        return RESULT_ADAPTER.validate_json(result.stdout)
    except ValueError as error:
        raise RuntimeError(
            f"LeRobot dataprep returned an invalid result: {result.stdout!r}"
        ) from error


def run_lerobot_dataprep(config: DataPrepConfig) -> Path:
    """Build a dataset in the isolated LeRobot environment."""
    result = _run(BuildRequest(config=config))
    if not isinstance(result, BuildResult):
        raise RuntimeError(f"LeRobot dataprep returned {result.command!r} for a build request")
    return result.path


def inspect_lerobot_dataset(path: Path) -> dict[str, Any]:
    """Inspect a dataset in the isolated LeRobot environment."""
    result = _run(InspectRequest(path=path))
    if not isinstance(result, InspectResult):
        raise RuntimeError(f"LeRobot dataprep returned {result.command!r} for an inspect request")
    return result.info
