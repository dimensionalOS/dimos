# Copyright 2025 Dimensional Inc.
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

import subprocess
import sys
import threading
import traceback
import os
from functools import cache
from dataclasses import dataclass
from typing import Any, Callable, Optional, Protocol, runtime_checkable

import lcm

from dimos.protocol.service.spec import Service


@cache
def check_root() -> bool:
    """Return True if the current process is running as root (UID 0)."""
    try:
        return os.geteuid() == 0  # type: ignore[attr-defined]
    except AttributeError:
        # Platforms without geteuid (e.g. Windows) – assume non-root.
        return False


def check_multicast() -> list[str]:
    """Check if multicast configuration is needed and return required commands."""
    commands_needed = []

    sudo = "" if check_root() else "sudo "

    # Check if loopback interface has multicast enabled
    try:
        result = subprocess.run(["ip", "link", "show", "lo"], capture_output=True, text=True)
        if "MULTICAST" not in result.stdout:
            commands_needed.append(f"{sudo}ifconfig lo multicast")
    except Exception:
        commands_needed.append(f"{sudo}ifconfig lo multicast")

    # Check if multicast route exists
    try:
        result = subprocess.run(
            ["ip", "route", "show", "224.0.0.0/4"], capture_output=True, text=True
        )
        if not result.stdout.strip():
            commands_needed.append(f"{sudo}route add -net 224.0.0.0 netmask 240.0.0.0 dev lo")
    except Exception:
        commands_needed.append(f"{sudo}route add -net 224.0.0.0 netmask 240.0.0.0 dev lo")

    return commands_needed


def check_buffers() -> list[str]:
    """Check if buffer configuration is needed and return required commands."""
    commands_needed = []

    sudo = "" if check_root() else "sudo "

    # Check current buffer settings
    try:
        result = subprocess.run(["sysctl", "net.core.rmem_max"], capture_output=True, text=True)
        current_max = int(result.stdout.split("=")[1].strip())
        if current_max < 2097152:
            commands_needed.append(f"{sudo}sysctl -w net.core.rmem_max=2097152")
    except Exception:
        commands_needed.append(f"{sudo}sysctl -w net.core.rmem_max=2097152")

    try:
        result = subprocess.run(["sysctl", "net.core.rmem_default"], capture_output=True, text=True)
        current_default = int(result.stdout.split("=")[1].strip())
        if current_default < 2097152:
            commands_needed.append(f"{sudo}sysctl -w net.core.rmem_default=2097152")
    except Exception:
        commands_needed.append(f"{sudo}sysctl -w net.core.rmem_default=2097152")

    return commands_needed


def check_system() -> None:
    """Check if system configuration is needed and exit with required commands if not prepared."""
    commands_needed = []
    commands_needed.extend(check_multicast())
    commands_needed.extend(check_buffers())

    if commands_needed:
        print("System configuration required. Please run the following commands:")
        for cmd in commands_needed:
            print(f"  {cmd}")
        print("\nThen restart your application.")
        sys.exit(1)


def autoconf() -> None:
    """Auto-configure system by running checks and executing required commands if needed."""
    commands_needed = []
    commands_needed.extend(check_multicast())
    commands_needed.extend(check_buffers())

    if not commands_needed:
        return

    print("System configuration required. Executing commands...")
    for cmd in commands_needed:
        print(f"  Running: {cmd}")
        try:
            # Split command into parts for subprocess
            cmd_parts = cmd.split()
            result = subprocess.run(cmd_parts, capture_output=True, text=True, check=True)
            print("  ✓ Success")
        except subprocess.CalledProcessError as e:
            print(f"  ✗ Failed: {e}")
            print(f"    stdout: {e.stdout}")
            print(f"    stderr: {e.stderr}")
        except Exception as e:
            print(f"  ✗ Error: {e}")

    print("System configuration completed.")


@dataclass
class LCMConfig:
    ttl: int = 0
    url: str | None = None
    autoconf: bool = False


@runtime_checkable
class LCMMsg(Protocol):
    name: str

    @classmethod
    def lcm_decode(cls, data: bytes) -> "LCMMsg":
        """Decode bytes into an LCM message instance."""
        ...

    def lcm_encode(self) -> bytes:
        """Encode this message instance into bytes."""
        ...


@dataclass
class Topic:
    topic: str = ""
    lcm_type: Optional[type[LCMMsg]] = None

    def __str__(self) -> str:
        if self.lcm_type is None:
            return self.topic
        return f"{self.topic}#{self.lcm_type.name}"


class LCMService(Service[LCMConfig]):
    default_config = LCMConfig
    l: lcm.LCM
    _stop_event: threading.Event
    _thread: Optional[threading.Thread]

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.l = lcm.LCM(self.config.url) if self.config.url else lcm.LCM()
        self._stop_event = threading.Event()
        self._thread = None

    def start(self):
        if self.config.autoconf:
            autoconf()
        else:
            try:
                check_system()
            except Exception as e:
                print(f"Error checking system configuration: {e}")

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop)
        self._thread.daemon = True
        self._thread.start()

    def _loop(self) -> None:
        """LCM message handling loop."""
        while not self._stop_event.is_set():
            try:
                self.l.handle_timeout(50)
            except Exception as e:
                stack_trace = traceback.format_exc()
                print(f"Error in LCM handling: {e}\n{stack_trace}")
                if self._stop_event.is_set():
                    break

    def stop(self):
        """Stop the LCM loop."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join()
