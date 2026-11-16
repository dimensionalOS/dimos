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

from pathlib import Path
import subprocess
import sys
from typing import Any

from dimos.experimental.isolated_python.module import (
    isolated_python_environment,
    isolated_python_run_command,
)
from dimos.imitation.dataprep.formats.lerobot.protocol import (
    RESULT_ADAPTER,
    BuildRequest,
    BuildResult,
    ErrorResult,
    InspectRequest,
    InspectResult,
    Request,
    Result,
)
from dimos.imitation.dataprep.schema import DataPrepConfig
from dimos.imitation.policy.lerobot.module import LeRobotPolicyModule
from dimos.utils.cache import cache_usage_guard
from dimos.utils.data import get_project_root


def lerobot_project() -> Path:
    """Locate the LeRobot project in the shared checkout."""
    return get_project_root() / LeRobotPolicyModule.project_dir


def _failure_detail(output: str) -> str:
    """Keep the error and its continuation lines, without preceding install logs."""
    traceback_header = "Traceback (most recent call last):\n"
    if traceback_header in output:
        lines = output.rsplit(traceback_header, 1)[-1].splitlines()
        for index, line in enumerate(lines):
            # Python indents stack frames; the exception starts at column zero.
            if line and not line[0].isspace():
                return " ".join("\n".join(lines[index:]).split())

    lines = output.strip().splitlines()
    for index, line in enumerate(lines):
        # uv errors can wrap across lines and end with a multi-line hint.
        if line.lstrip().startswith(("error:", "\u00d7 ")):
            return " ".join("\n".join(lines[index:]).split())
    return lines[-1].strip() if lines else "no diagnostic output"


def _run(request: Request) -> Result:
    """Run one typed request under the locked LeRobot dependency stack."""
    project = lerobot_project()
    command = isolated_python_run_command(
        project,
        "python",
        "-m",
        "dimos_lerobot.dataprep",
    )
    env = isolated_python_environment(project)
    try:
        with cache_usage_guard():
            print(
                "Preparing LeRobot environment and running dataprep "
                "(first use may download several GB and take minutes)...",
                file=sys.stderr,
                flush=True,
            )
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
        try:
            failure = ErrorResult.model_validate_json(result.stdout.rstrip().rsplit("\n", 1)[-1])
            detail = " ".join(failure.error.split())
        except ValueError:
            # Installer failures and unexpected exceptions have no protocol result.
            output = result.stderr.strip() or result.stdout.strip()
            detail = _failure_detail(output)
        raise RuntimeError(f"LeRobot dataprep exited with status {result.returncode}: {detail}")
    try:
        return RESULT_ADAPTER.validate_json(result.stdout.rstrip().rsplit("\n", 1)[-1])
    except ValueError as error:
        raise RuntimeError(
            f"LeRobot dataprep returned an invalid result: {result.stdout!r}"
        ) from error


def run_lerobot_dataprep(config: DataPrepConfig) -> Path:
    """Build a dataset in the isolated LeRobot environment."""
    config = config.model_copy(
        update={
            "source": str(Path(config.source).resolve()),
            "output": config.output.model_copy(update={"path": config.output.path.resolve()}),
        }
    )
    result = _run(BuildRequest(config=config))
    if not isinstance(result, BuildResult):
        raise RuntimeError(f"LeRobot dataprep returned {result.command!r} for a build request")
    return result.path


def inspect_lerobot_dataset(path: Path) -> dict[str, Any]:
    """Inspect a dataset in the isolated LeRobot environment."""
    result = _run(InspectRequest(path=path.resolve()))
    if not isinstance(result, InspectResult):
        raise RuntimeError(f"LeRobot dataprep returned {result.command!r} for an inspect request")
    return result.info
