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

"""Run a subprocess with a timeout and process-group cleanup."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import time

CLEANUP_TIMEOUT_S = 5


def _signal_group(pid: int, sig: signal.Signals) -> None:
    deadline = time.monotonic() + CLEANUP_TIMEOUT_S
    while True:
        try:
            os.killpg(pid, sig)
            return
        except ProcessLookupError:
            return
        except PermissionError:
            # macOS returns EPERM for groups containing only unreaped zombies.
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.01)


def run_process(args: list[str], log_path: Path, timeout_s: float) -> int:
    """Log output, terminate the process group, and reap its leader.

    Descendants may outlive the leader, so always send SIGKILL to the group.
    """
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w") as log:
        process = subprocess.Popen(
            args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
        )
        try:
            return process.wait(timeout=timeout_s)
        finally:
            try:
                _signal_group(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=CLEANUP_TIMEOUT_S)
                except subprocess.TimeoutExpired:
                    pass
            finally:
                try:
                    _signal_group(process.pid, signal.SIGKILL)
                finally:
                    process.wait(timeout=CLEANUP_TIMEOUT_S)
