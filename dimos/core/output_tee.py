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

"""OutputTee — fd-level tee for stdout/stderr to log files.

Operates at the file-descriptor level so it captures output from native
extensions as well as Python ``print()`` / ``sys.stdout.write()``.
"""

from __future__ import annotations

import os
import re
import threading
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from pathlib import Path

_ANSI_RE = re.compile(rb"\x1b\[[0-9;]*m")


class OutputTee:
    """Fan out fd 1+2 to: stdout.log (with ANSI) and stdout.plain.log (stripped).

    Parameters
    ----------
    run_dir:
        Directory where ``stdout.log`` and ``stdout.plain.log`` are created.
    """

    _MAX_LOGGED_ERRORS = 5  # avoid flooding tee_errors.log on repeated failures

    def __init__(self, run_dir: Path) -> None:
        self._run_dir = run_dir
        self._reader_thread: threading.Thread | None = None
        self._stopped = False
        self._error_count = 0

        # Internal pipe: we dup2 stdout/stderr to write_fd; the reader
        # thread reads from read_fd and fans out.
        self._read_fd, self._write_fd = os.pipe()

        # Open log files
        run_dir.mkdir(parents=True, exist_ok=True)
        self._color_log = open(run_dir / "stdout.log", "ab")
        self._plain_log = open(run_dir / "stdout.plain.log", "ab")

        # Save original stdout fd so we can still write to the real terminal
        # (only relevant for the attached-mode case).
        self._orig_stdout_fd = os.dup(1)

    def start(self) -> None:
        """Redirect fd 1 and 2 to the internal pipe and start the reader thread."""
        os.dup2(self._write_fd, 1)
        os.dup2(self._write_fd, 2)

        self._reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True, name="output-tee"
        )
        self._reader_thread.start()

    def _reader_loop(self) -> None:
        """Read from the internal pipe and fan out to all destinations."""
        try:
            while True:
                data = os.read(self._read_fd, 65536)
                if not data:
                    break
                # Write to stdout.log (with ANSI colors)
                try:
                    self._color_log.write(data)
                    self._color_log.flush()
                except Exception as exc:
                    self._log_tee_error("stdout.log", exc)
                # Write to stdout.plain.log (ANSI stripped)
                try:
                    self._plain_log.write(_ANSI_RE.sub(b"", data))
                    self._plain_log.flush()
                except Exception as exc:
                    self._log_tee_error("stdout.plain.log", exc)
        except OSError:
            pass  # pipe closed

    def _log_tee_error(self, target: str, exc: Exception) -> None:
        """Record a write error to ``tee_errors.log`` in the run directory.

        The TUI runner polls for this file and shows a notification on
        first appearance.  Only the first few errors are logged to avoid
        flooding disk on sustained failures (e.g. disk full).
        """
        self._error_count += 1
        if self._error_count > self._MAX_LOGGED_ERRORS:
            return
        try:
            msg = f"OutputTee: failed to write to {target}: {exc}\n"
            err_path = self._run_dir / "tee_errors.log"
            with open(err_path, "a") as f:
                f.write(msg)
        except Exception:
            pass  # nothing more we can do

    def close(self) -> None:
        """Shut down the tee (flush, close files, join reader)."""
        self._stopped = True
        # Close the write end so the reader loop sees EOF
        try:
            os.close(self._write_fd)
        except OSError:
            pass
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=2.0)
        try:
            os.close(self._read_fd)
        except OSError:
            pass
        self._color_log.close()
        self._plain_log.close()
        try:
            os.close(self._orig_stdout_fd)
        except OSError:
            pass
