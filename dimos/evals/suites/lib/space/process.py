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

"""Bound subprocess duration and clean up its complete owned process group."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess


def _signal_group(pid: int, sig: signal.Signals) -> None:
    try:
        os.killpg(pid, sig)
    except ProcessLookupError:
        pass


def run_process(args: list[str], log_path: Path, timeout_s: float) -> int:
    """Log to disk and reap the leader and descendants, including on interruption.

    A leader may exit while a spawned worker ignores SIGTERM. Always kill the
    group after the bounded grace period, even when waiting for the leader
    already succeeded. No pipe buffers or Popen context-manager waits can block
    the cancellation path.
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
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    pass
            finally:
                try:
                    _signal_group(process.pid, signal.SIGKILL)
                finally:
                    process.wait(timeout=5)
