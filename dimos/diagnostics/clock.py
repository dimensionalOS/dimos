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

"""Identify a monotonic clock without assuming wall clocks are synchronized."""

from functools import lru_cache
from pathlib import Path
import platform
import socket
import subprocess
from uuid import uuid4


@lru_cache(maxsize=1)
def clock_domain() -> str:
    boot = Path("/proc/sys/kernel/random/boot_id")
    identity = uuid4().hex
    try:
        if boot.exists():
            identity = boot.read_text().strip()
        elif platform.system() == "Darwin":
            result = subprocess.run(
                ["/usr/sbin/sysctl", "-n", "kern.bootsessionuuid"],
                capture_output=True,
                text=True,
                timeout=1,
                check=False,
            )
            if result.returncode == 0 and result.stdout.strip():
                identity = result.stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        pass  # A process-local domain is conservative when the boot ID is unavailable.
    return f"{socket.gethostname()}/{identity}/monotonic"
