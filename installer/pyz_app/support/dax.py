#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import shlex
import shutil
import subprocess
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Iterable, Sequence


@dataclass
class CommandResult:
    code: int
    _stdout: str | None
    _stderr: str | None
    _captured: bool = True

    @property
    def stdout(self) -> str:
        if not self._captured:
            raise RuntimeError(
                "stdout not captured; run_command was invoked with capture_output=False"
            )
        return self._stdout or ""

    @property
    def stderr(self) -> str:
        if not self._captured:
            raise RuntimeError(
                "stderr not captured; run_command was invoked with capture_output=False"
            )
        return self._stderr or ""


def _normalize_cmd(cmd: str | Sequence[str] | Iterable[str]) -> list[str]:
    if isinstance(cmd, (list, tuple)):
        return [str(part) for part in cmd]
    if isinstance(cmd, Path):
        return [str(cmd)]
    if isinstance(cmd, str):
        # Match shell-like splitting for convenience.
        return shlex.split(cmd)
    return [str(c) for c in cmd]


def command_exists(name: str) -> bool:
    return shutil.which(name) is not None


def run_command(
    cmd: str | Sequence[str] | Iterable[str],
    *,
    check: bool = False,
    capture_output: bool = False,
    cwd: str | Path | None = None,
    env: dict[str, str] | None = None,
    print_command: bool = False,
    dry_run: bool = False,
) -> CommandResult:
    cmd_list = _normalize_cmd(cmd)

    if dry_run:
        if print_command:
            print(f"$ {' '.join(cmd_list)}")
        else:
            print(f"> {' '.join(cmd_list)}")
        return CommandResult(code=0, stdout="", stderr="")

    if print_command:
        print(f"$ {' '.join(cmd_list)}")

    completed = subprocess.run(
        cmd_list,
        cwd=str(cwd) if cwd is not None else None,
        env=env,
        stdout=subprocess.PIPE if capture_output else None,
        stderr=subprocess.PIPE if capture_output else None,
        text=True,
        check=check,
    )
    return CommandResult(
        code=completed.returncode,
        _stdout=completed.stdout if capture_output else None,
        _stderr=completed.stderr if capture_output else None,
        _captured=capture_output,
    )


def run_quiet(
    cmd: str | Sequence[str] | Iterable[str],
    *,
    cwd: str | Path | None = None,
    env: dict[str, str] | None = None,
) -> CommandResult:
    return run_command(cmd, capture_output=True, cwd=cwd, env=env, check=False)


__all__ = ["CommandResult", "command_exists", "run_command", "run_quiet"]
