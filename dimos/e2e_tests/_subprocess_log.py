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

"""Subprocess launcher that streams stdout/stderr line-by-line to a logfile.

The naive `Popen(stdout=open(path, "w"))` pattern relies on the child's stdio
to flush on exit, which Deno (and Python with block-buffered stdout) do not
reliably do under SIGTERM. The result is empty CI logs.

This helper reads `subprocess.PIPE` in a background daemon thread and writes
each line with explicit `flush()` so SIGTERM-killed children still leave their
last few hundred lines on disk.
"""

from __future__ import annotations

import os
import re
import subprocess
import tempfile
import threading
import time
from pathlib import Path


def launch_with_streaming_log(
    label: str,
    args: list[str],
    *,
    env: dict[str, str] | None = None,
) -> tuple[subprocess.Popen[str], Path]:
    """Spawn `args`, stream merged stdout/stderr to a tempfile.

    Parameters
    ----------
    label
        Short prefix for the tempfile name (also useful in error messages).
    args
        Command and args, like `subprocess.Popen`.
    env
        Optional process environment.

    Returns
    -------
    proc
        The `Popen` handle. Output is being streamed in a daemon thread.
    log_path
        Tempfile path where stdout/stderr lines are written. Caller is
        responsible for unlinking.
    """
    log_fd, log_name = tempfile.mkstemp(prefix=f"{label}_", suffix=".log")
    os.close(log_fd)
    log_path = Path(log_name)

    proc = subprocess.Popen(
        args,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=1,
        text=True,
        start_new_session=True,
    )

    def _pump() -> None:
        assert proc.stdout is not None
        try:
            with log_path.open("w") as out:
                for line in proc.stdout:
                    out.write(line)
                    out.flush()
        except Exception:  # noqa: BLE001 — daemon thread, swallow
            pass

    t = threading.Thread(target=_pump, daemon=True, name=f"{label}-stdout-pump")
    t.start()

    return proc, log_path


def wait_for_log_pattern(
    log_path: Path, pattern: str, *, timeout: float = 600.0
) -> bool:
    """Tail the streamed log file until a line matching `pattern` appears.

    Used to gate fixture setup on a real subprocess milestone (e.g. Rapier
    snapshot fully restored, server physics active) instead of a wall-clock
    sleep. Polls the file every 0.5s and re-reads only the appended portion.
    """
    regex = re.compile(pattern)
    deadline = time.time() + timeout
    pos = 0
    while time.time() < deadline:
        try:
            with log_path.open("r", errors="replace") as f:
                f.seek(pos)
                chunk = f.read()
                pos = f.tell()
                if regex.search(chunk):
                    return True
        except FileNotFoundError:
            pass
        time.sleep(0.5)
    return False


def dump_log(label: str, log_path: Path, *, max_bytes: int = 50_000) -> None:
    """Print the captured output so pytest surfaces it in the test log."""
    print(f"\n========= {label} stdout/stderr =========")
    try:
        text = log_path.read_text(errors="replace")
    except OSError as exc:
        print(f"(could not read log: {exc})")
        return
    if not text:
        print("(empty — subprocess produced no output before termination)")
        return
    if len(text) > max_bytes:
        head = text[: max_bytes // 2]
        tail = text[-max_bytes // 2 :]
        print(head)
        print(f"\n... [{len(text) - max_bytes} bytes elided] ...\n")
        print(tail)
    else:
        print(text)
    print(f"========= end {label} =========\n")
