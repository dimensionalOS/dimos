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

"""Watchdog sidecar: terminate run descendants when main dies."""

from __future__ import annotations

import sys
import time

from dimos.core.coordination.process_lifecycle import (
    kill_run_processes,
    wait_for_pid_exit,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Give the main process's graceful-shutdown handler a head start after its
# PID disappears. Without this we'd race the pipe-based worker shutdown and
# kill workers that are about to exit on their own.
_GRACE_PERIOD_SECONDS = 0.5


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print(f"usage: {argv[0]} <main_pid> <run_id>", file=sys.stderr)
        return 2

    main_pid = int(argv[1])
    run_id = argv[2]

    wait_for_pid_exit(main_pid)

    time.sleep(_GRACE_PERIOD_SECONDS)

    kill_run_processes(run_id)

    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
