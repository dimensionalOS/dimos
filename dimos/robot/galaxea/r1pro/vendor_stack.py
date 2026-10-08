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
import time

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The vendor's startup script, and the session profile the R1 Pro boots with.
VENDOR_STARTUP_SCRIPT = os.path.join(
    os.environ.get("HOME", ""),
    "galaxea-dimos/install/startup_config/share/startup_config/script/robot_startup.sh",
)
VENDOR_PROFILE = "../sessions.d/ATCStandard/R1PROBody.d/"
# tmux sessions the profile creates; all present means the stack is already up.
VENDOR_SESSIONS: tuple[str, ...] = ("hdas", "mobiman")
# The vendor livox_ros_driver2 window; it takes the Mid-360 from our own driver.
VENDOR_LIDAR_WINDOW = "hdas:livox"
# The vendor head camera node (process name, truncated by the kernel); it holds the head eyes our V4L2 cameras open.
VENDOR_HEAD_CAMERA_PROCESS = "signal_camera_n"


def running_tmux_sessions() -> set[str]:
    result = subprocess.run(
        ["tmux", "list-sessions", "-F", "#{session_name}"],
        capture_output=True,
        text=True,
        check=False,
    )
    return set(result.stdout.split()) if result.returncode == 0 else set()


class R1ProVendorStackConfig(ModuleConfig):
    enabled: bool = True
    startup_script: str = VENDOR_STARTUP_SCRIPT
    profile: str = VENDOR_PROFILE
    # Close the vendor lidar driver, for blueprints that run dimos's own Mid-360 driver.
    stop_vendor_lidar: bool = False


class R1ProVendorStack(Module):
    """Boots the vendor stack if its tmux sessions are missing, before any module starts."""

    config: R1ProVendorStackConfig

    @rpc
    def build(self) -> None:
        super().build()
        command = boot_command(self.config, running_tmux_sessions())
        if command is not None:
            logger.info("R1 vendor stack not running; booting %s", self.config.profile)
            subprocess.run(command, cwd=Path(command[1]).parent, check=True)
        if self.config.enabled and self.config.stop_vendor_lidar:
            # Before our driver's handshake, so the Livox streams to us, not the vendor.
            subprocess.run(["tmux", "kill-window", "-t", VENDOR_LIDAR_WINDOW], check=False)
        if self.config.enabled:
            # A freshly booted profile starts it a few seconds later.
            stop_vendor_head_cameras(appear_timeout_s=30.0 if command is not None else 0.0)


def _vendor_head_camera_running() -> bool:
    return (
        subprocess.run(["pgrep", "-x", VENDOR_HEAD_CAMERA_PROCESS], capture_output=True).returncode
        == 0
    )


def stop_vendor_head_cameras(appear_timeout_s: float) -> None:
    """SIGINT the vendor head camera node, so it shuts down cleanly, and wait for it to exit."""
    deadline = time.monotonic() + appear_timeout_s
    while not _vendor_head_camera_running() and time.monotonic() < deadline:
        time.sleep(0.5)
    if not _vendor_head_camera_running():
        return
    logger.info("Stopping the vendor head camera node")
    subprocess.run(["pkill", "-INT", "-x", VENDOR_HEAD_CAMERA_PROCESS], check=False)
    deadline = time.monotonic() + 10.0
    while _vendor_head_camera_running() and time.monotonic() < deadline:
        time.sleep(0.5)


def boot_command(config: R1ProVendorStackConfig, running: set[str]) -> list[str] | None:
    """The vendor boot command, or None when disabled, already up, or not installed."""
    if not config.enabled or set(VENDOR_SESSIONS) <= running:
        return None
    script = Path(config.startup_script).expanduser()
    if not script.exists():
        logger.warning("R1 vendor stack not running and %s does not exist", script)
        return None
    # The script resolves the profile relative to its own directory.
    return ["bash", str(script), "boot", config.profile]
