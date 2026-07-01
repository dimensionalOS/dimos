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

"""Subprocess helpers for long-running scene cooking tools."""

from __future__ import annotations

from collections import deque
from collections.abc import Callable, Sequence
import contextlib
import selectors
import shlex
import subprocess
import time

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_DEFAULT_HEARTBEAT_SECONDS = 30.0
_DEFAULT_TAIL_LINES = 30


def run_logged_command(
    args: Sequence[str],
    label: str,
    *,
    heartbeat_seconds: float = _DEFAULT_HEARTBEAT_SECONDS,
    tail_lines: int = _DEFAULT_TAIL_LINES,
    line_log_filter: Callable[[str], bool] | None = None,
) -> str:
    """Run a command while streaming output and emitting heartbeat logs.

    Blender and mesh optimizers can run for minutes on production scenes. Using
    ``subprocess.run(stdout=PIPE)`` hides all progress until the command exits,
    which makes failures and stalls indistinguishable to operators.
    """

    command = " ".join(shlex.quote(str(arg)) for arg in args)
    logger.info("scene cook command started", label=label, command=command)
    started = time.monotonic()
    last_heartbeat = started
    output_lines: list[str] = []
    tail: deque[str] = deque(maxlen=tail_lines)

    proc = subprocess.Popen(
        [str(arg) for arg in args],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    assert proc.stdout is not None
    stdout = proc.stdout

    selector = selectors.DefaultSelector()
    selector.register(stdout, selectors.EVENT_READ)
    stdout_open = True

    try:
        while True:
            if stdout_open:
                for _, _ in selector.select(timeout=1.0):
                    line = stdout.readline()
                    if line == "":
                        selector.unregister(stdout)
                        stdout_open = False
                        break
                    clean = line.rstrip()
                    output_lines.append(clean)
                    tail.append(clean)
                    if line_log_filter is None or line_log_filter(clean):
                        logger.info("scene cook command output", label=label, line=clean)
            else:
                time.sleep(0.1)

            returncode = proc.poll()
            now = time.monotonic()
            if now - last_heartbeat >= heartbeat_seconds:
                logger.info(
                    "scene cook command still running",
                    label=label,
                    elapsed_s=round(now - started, 1),
                    recent_output=list(tail),
                )
                last_heartbeat = now

            if returncode is not None:
                if stdout_open:
                    remaining = proc.stdout.read()
                    for line in remaining.splitlines():
                        output_lines.append(line)
                        tail.append(line)
                        if line_log_filter is None or line_log_filter(line):
                            logger.info("scene cook command output", label=label, line=line)
                    selector.unregister(proc.stdout)
                break
    except BaseException:
        if proc.poll() is None:
            logger.warning("scene cook command interrupted; terminating", label=label)
            proc.terminate()
            with contextlib.suppress(subprocess.TimeoutExpired):
                proc.wait(timeout=5.0)
            if proc.poll() is None:
                logger.warning("scene cook command did not terminate; killing", label=label)
                proc.kill()
                proc.wait()
        raise
    finally:
        selector.close()
        proc.stdout.close()

    elapsed_s = time.monotonic() - started
    output = "\n".join(output_lines)
    if returncode != 0:
        raise RuntimeError(
            f"{label} failed with exit code {returncode} after {elapsed_s:.1f}s\n"
            f"command: {command}\n"
            f"last output:\n{_tail(output, tail_lines)}"
        )

    logger.info(
        "scene cook command finished",
        label=label,
        elapsed_s=round(elapsed_s, 1),
        recent_output=list(tail),
    )
    return output


def _tail(output: str, tail_lines: int) -> str:
    return "\n".join(output.splitlines()[-tail_lines:])


def blender_output_line_is_interesting(line: str) -> bool:
    """Return true for Blender output worth streaming during normal cooks."""

    return (
        line.startswith("DIMOS_")
        or "Read blend:" in line
        or "Finished glTF" in line
        or line.startswith("Blender ")
        or line == "Blender quit"
        or "Traceback" in line
        or "ERROR" in line
        or line.startswith("Error:")
    )
