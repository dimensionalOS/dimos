#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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
import sys
from typing import (
    TYPE_CHECKING,
    Optional,
)

if TYPE_CHECKING:
    from collections.abc import Callable, Iterable, Sequence


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
    if isinstance(cmd, list | tuple):
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
    output_preview: bool = False,
    stream_callback: Callable[[str], None] | None = None,
    combine_streams: bool = True,
) -> CommandResult:
    cmd_list = _normalize_cmd(cmd)

    # If running as root, strip leading sudo to avoid nested privilege escalation issues (e.g., inside Docker).
    if cmd_list and cmd_list[0] == "sudo":
        try:
            import os

            if os.geteuid() == 0:
                cmd_list = cmd_list[1:]
        except Exception:
            # If we can't determine UID, fall through and run as-is.
            pass

    if dry_run:
        if print_command:
            print(f"DRY: $ {' '.join(cmd_list)}")
        else:
            print(f"DRY: > {' '.join(cmd_list)}")
        return CommandResult(
            code=0,
            _stdout="" if capture_output else None,
            _stderr="" if capture_output else None,
            _captured=capture_output,
        )

    if print_command:
        print(f"$ {' '.join(cmd_list)}")

    if stream_callback and output_preview:
        raise ValueError("stream_callback and output_preview cannot be used together")

    if stream_callback:
        process = subprocess.Popen(
            cmd_list,
            cwd=str(cwd) if cwd is not None else None,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT if combine_streams else subprocess.PIPE,
            text=True,
        )
        collected: list[str] = []

        while True:
            line = process.stdout.readline() if process.stdout else ""
            if not line and process.poll() is not None:
                break
            if line:
                for _each_line in line.rstrip("\n").split("\n"):
                    stream_callback(line)
                if capture_output:
                    collected.append(line)

        code = process.wait()
        if check and code != 0:
            raise subprocess.CalledProcessError(code, cmd_list)
        return CommandResult(
            code=code,
            _stdout="".join(collected) if capture_output else None,
            _stderr=None,
            _captured=capture_output,
        )

    if output_preview and capture_output:
        # stream line-by-line with \r, while capturing
        process = subprocess.Popen(
            cmd_list,
            cwd=str(cwd) if cwd is not None else None,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        combined_chunks: list[str] = []
        term_width = shutil.get_terminal_size(fallback=(80, 24)).columns

        def _emit(line: str):
            clipped = (line.rstrip("\n")).replace("\r", "")
            if len(clipped) > term_width - 1:
                clipped = clipped[: term_width - 1]
            sys.stdout.write("\r" + clipped + " " * max(0, term_width - len(clipped) - 1))
            sys.stdout.flush()
            combined_chunks.append(line)

        # read stdout/stderr interleaved-ish
        while True:
            out_line = process.stdout.readline() if process.stdout else ""
            err_line = process.stderr.readline() if process.stderr else ""
            if not out_line and not err_line and process.poll() is not None:
                break
            if out_line:
                _emit(out_line)
            if err_line:
                _emit(err_line)

        code = process.wait()
        sys.stdout.write("\n")
        stdout_combined = "".join(combined_chunks)
        if code != 0:
            print("---- command output ----")
            print(stdout_combined)
            print("------------------------")
        return CommandResult(
            code=code,
            _stdout=stdout_combined,
            _stderr="",
            _captured=True,
        )

    completed = subprocess.run(
        cmd_list,
        cwd=str(cwd) if cwd is not None else None,
        env=env,
        stdout=subprocess.PIPE if capture_output else sys.stdout,
        stderr=subprocess.PIPE if capture_output else sys.stderr,
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
