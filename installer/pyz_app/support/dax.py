#!/usr/bin/env python3
from __future__ import annotations

import shlex
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


@dataclass
class CommandResult:
    code: int
    stdout: str
    stderr: str


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
    capture_output: bool = True,
    cwd: str | Path | None = None,
    env: dict[str, str] | None = None,
    print_command: bool = False,
) -> CommandResult:
    cmd_list = _normalize_cmd(cmd)
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
        stdout=completed.stdout or "",
        stderr=completed.stderr or "",
    )


def run_quiet(
    cmd: str | Sequence[str] | Iterable[str],
    *,
    cwd: str | Path | None = None,
    env: dict[str, str] | None = None,
) -> CommandResult:
    return run_command(cmd, capture_output=True, cwd=cwd, env=env, check=False)


__all__ = ["CommandResult", "command_exists", "run_command", "run_quiet"]
