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

import os
import subprocess
import sys
import threading
from dataclasses import dataclass
from functools import cache
from typing import Optional, Protocol, runtime_checkable
from weakref import WeakSet

import lcm

from dimos.protocol.service.spec import ConfigBase, Service
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.protocol.service.lcmservice")


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


def check_buffers() -> tuple[list[str], Optional[int]]:
    """Check if buffer configuration is needed and return required commands and current size.

    Returns:
        Tuple of (commands_needed, current_max_buffer_size)
    """
    commands_needed = []
    current_max = None

    sudo = "" if check_root() else "sudo "

    # Check current buffer settings
    try:
        result = subprocess.run(["sysctl", "net.core.rmem_max"], capture_output=True, text=True)
        current_max = int(result.stdout.split("=")[1].strip()) if result.returncode == 0 else None
        if not current_max or current_max < 2097152:
            commands_needed.append(f"{sudo}sysctl -w net.core.rmem_max=2097152")
    except Exception:
        commands_needed.append(f"{sudo}sysctl -w net.core.rmem_max=2097152")

    try:
        result = subprocess.run(["sysctl", "net.core.rmem_default"], capture_output=True, text=True)
        current_default = (
            int(result.stdout.split("=")[1].strip()) if result.returncode == 0 else None
        )
        if not current_default or current_default < 2097152:
            commands_needed.append(f"{sudo}sysctl -w net.core.rmem_default=2097152")
    except Exception:
        commands_needed.append(f"{sudo}sysctl -w net.core.rmem_default=2097152")

    return commands_needed, current_max


def check_system() -> None:
    """Check if system configuration is needed and exit only for critical issues.

    Multicast configuration is critical for LCM to work.
    Buffer sizes are performance optimizations - warn but don't fail in containers.
    """
    if os.environ.get("CI"):
        logger.debug("CI environment detected: Skipping system configuration checks.")
        return

    multicast_commands = check_multicast()
    buffer_commands, current_buffer_size = check_buffers()

    # Check multicast first - this is critical
    if multicast_commands:
        logger.error(
            "Critical: Multicast configuration required. Please run the following commands:"
        )
        for cmd in multicast_commands:
            logger.error(f"  {cmd}")
        logger.error("\nThen restart your application.")
        sys.exit(1)

    # Buffer configuration is just for performance
    elif buffer_commands:
        if current_buffer_size:
            logger.warning(
                f"UDP buffer size limited to {current_buffer_size} bytes ({current_buffer_size // 1024}KB). Large LCM packets may fail."
            )
        else:
            logger.warning("UDP buffer sizes are limited. Large LCM packets may fail.")
        logger.warning("For better performance, consider running:")
        for cmd in buffer_commands:
            logger.warning(f"  {cmd}")
        logger.warning("Note: This may not be possible in Docker containers.")


def autoconf() -> None:
    """Auto-configure system by running checks and executing required commands if needed."""
    if os.environ.get("CI"):
        logger.info("CI environment detected: Skipping automatic system configuration.")
        return

    commands_needed = []

    # Check multicast configuration
    commands_needed.extend(check_multicast())

    # Check buffer configuration
    buffer_commands, _ = check_buffers()
    commands_needed.extend(buffer_commands)

    if not commands_needed:
        return

    logger.info("System configuration required. Executing commands...")

    for cmd in commands_needed:
        logger.info(f"  Running: {cmd}")
        try:
            # Split command into parts for subprocess
            cmd_parts = cmd.split()
            subprocess.run(cmd_parts, capture_output=True, text=True, check=True)
            logger.info("  ✓ Success")
        except subprocess.CalledProcessError as e:
            # Check if this is a multicast/route command or a sysctl command
            if "route" in cmd or "multicast" in cmd:
                # Multicast/route failures should still fail
                logger.error(f"  ✗ Failed to configure multicast: {e}")
                logger.error(f"    stdout: {e.stdout}")
                logger.error(f"    stderr: {e.stderr}")
                raise
            elif "sysctl" in cmd:
                # Sysctl failures are just warnings (likely docker/container)
                logger.warning(
                    f"  ✗ Not able to auto-configure UDP buffer sizes (likely docker image): {e}"
                )
        except Exception as e:
            logger.error(f"  ✗ Error: {e}")
            if "route" in cmd or "multicast" in cmd:
                raise

    logger.info("System configuration completed.")


class LCMConfig(ConfigBase):
    ttl: int = 0
    url: Optional[str] = None
    autoconf: bool = True


@runtime_checkable
class LCMMsg(Protocol):
    msg_name: str

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
        return f"{self.topic}#{self.lcm_type.msg_name}"


class LCMShim:
    """Manages shared LCM instances and handle_timeout loops.

    Maintains a single LCM instance per URL and runs a shared event loop.
    Automatically cleans up when all referencing services are garbage collected.
    """

    _instances: dict[str, "LCMShim"] = {}
    _lock = threading.Lock()

    def __init__(self, url: Optional[str] = None):
        self.url = url or ""
        self.lcm = lcm.LCM(url) if url else lcm.LCM()
        self._services = WeakSet()
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._running = False

    @classmethod
    def get_instance(cls, url: Optional[str] = None) -> "LCMShim":
        """Get or create a shim instance for the given URL."""
        url_key = url or ""

        with cls._lock:
            if url_key not in cls._instances:
                logger.debug(f"Creating new LCMShim for URL: {url_key or 'default'}")
                cls._instances[url_key] = cls(url)
            else:
                logger.debug(
                    f"Reusing existing LCMShim for URL: {url_key or 'default'}, running={cls._instances[url_key]._running}"
                )
            return cls._instances[url_key]

    def register_service(self, service: "LCMService") -> None:
        """Register a service that uses this shim."""
        with self._lock:
            self._services.add(service)
            if not self._running:
                self._start_loop()

    def unregister_service(self, service: "LCMService") -> None:
        """Unregister a service. Stops loop if no services remain."""
        should_stop = False
        with self._lock:
            self._services.discard(service)
            if not self._services and self._running:
                should_stop = True

        # Call _stop_loop outside of lock context to avoid deadlock
        if should_stop:
            self._stop_loop()

    def _start_loop(self) -> None:
        """Start the shared handle_timeout loop."""
        if self._running:
            return

        self._running = True
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _stop_loop(self) -> None:
        """Stop the shared loop and cleanup."""
        if not self._running:
            return

        self._running = False
        self._stop_event.set()

        if self._thread and self._thread != threading.current_thread():
            self._thread.join(timeout=1.0)
            self._thread = None

        # Remove this shim instance from the global registry
        url_key = self.url or ""
        with LCMShim._lock:
            if url_key in LCMShim._instances and LCMShim._instances[url_key] is self:
                del LCMShim._instances[url_key]

    def _loop(self) -> None:
        """Shared LCM message handling loop."""
        while not self._stop_event.is_set():
            try:
                self.lcm.handle_timeout(50)
            except Exception as e:
                if not self._stop_event.is_set():
                    logger.error(f"Error in LCM handling: {e}", exc_info=True)

            # Check if we still have services
            with self._lock:
                if not self._services:
                    break

        # Cleanup after loop exits
        # Don't hold lock while modifying class variables to avoid deadlock
        self._running = False


class LCMService(Service[LCMConfig]):
    default_config = LCMConfig
    l: lcm.LCM
    _shim: LCMShim

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        # Always use the shim for shared LCM instance management
        self._shim = LCMShim.get_instance(self.config.url)
        self.l = self._shim.lcm
        self._shim.register_service(self)

    def start(self):
        if self.config.autoconf:
            autoconf()
        else:
            try:
                check_system()
            except Exception as e:
                print(f"Error checking system configuration: {e}")
        # Shim handles the loop

    def stop(self):
        """Stop the service."""
        # Unregister from shim - it will stop loop if no services remain
        self._shim.unregister_service(self)

    def __del__(self):
        """Ensure cleanup on garbage collection."""
        self._shim.unregister_service(self)
