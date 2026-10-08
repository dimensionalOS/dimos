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

"""Boot Galaxea's vendor stack (drivers, cameras, controllers) when it is not already running."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The vendor's startup script, and the session profile the R1 Pro boots with.
VENDOR_STARTUP_SCRIPT = os.path.join(
    os.environ.get("HOME", ""),
    "galaxea-dimos/install/startup_config/share/startup_config/script/robot_startup.sh",
)
# ATCStandard minus the vendor head camera and Livox drivers, whose devices dimos opens itself.
VENDOR_PROFILE = "../sessions.d/DimOS/R1PROBody.d/"
# tmux sessions the profile creates; all present means the stack is already up.
VENDOR_SESSIONS: tuple[str, ...] = ("hdas", "mobiman")


def running_tmux_sessions() -> set[str]:
    result = subprocess.run(
        ["tmux", "list-sessions", "-F", "#{session_name}"],
        capture_output=True,
        text=True,
        check=False,
    )
    return set(result.stdout.split()) if result.returncode == 0 else set()


def boot_command(startup_script: str, profile: str, running: set[str]) -> list[str] | None:
    """The vendor boot command, or None when already up or not installed."""
    if set(VENDOR_SESSIONS) <= running:
        return None
    script = Path(startup_script).expanduser()
    if not script.exists():
        logger.warning("R1 vendor stack not running and %s does not exist", script)
        return None
    # The script resolves the profile relative to its own directory.
    if not (script.parent / profile).is_dir():
        logger.warning("R1 vendor stack not running and profile %s is not installed", profile)
        return None
    return ["bash", str(script), "boot", profile]


def boot_vendor_stack(startup_script: str, profile: str) -> None:
    """Boot the vendor stack with ``profile`` unless it is already running."""
    command = boot_command(startup_script, profile, running_tmux_sessions())
    if command is not None:
        logger.info("R1 vendor stack not running; booting %s", profile)
        subprocess.run(command, cwd=Path(command[1]).parent, check=True)
