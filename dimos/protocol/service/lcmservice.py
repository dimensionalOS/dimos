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

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from functools import cache
import os
import subprocess
import sys
import threading
import traceback
from typing import Protocol, runtime_checkable

import lcm

from dimos.protocol.service.spec import Service
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.protocol.service.lcmservice")

# Module-level shared LCM state
_shared_lcm_lock = threading.Lock()
_shared_lcm_wrapper: SharedLCMWrapper | None = None


class LCMWrapper:
    """Base wrapper for LCM instances."""

    def get_lcm(self) -> lcm.LCM:
        """Get the LCM instance."""
        raise NotImplementedError

    def start_handling(self) -> None:
        """Start the LCM message handling loop."""
        raise NotImplementedError

    def stop_handling(self) -> None:
        """Stop the LCM message handling loop."""
        raise NotImplementedError


class DedicatedLCMWrapper(LCMWrapper):
    """Wrapper for a dedicated LCM instance."""

    def __init__(self, url: str | None = None):
        self._lcm = lcm.LCM(url) if url else lcm.LCM()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def get_lcm(self) -> lcm.LCM:
        return self._lcm

    def start_handling(self) -> None:
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop)
        self._thread.daemon = True
        self._thread.start()

    def _loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._lcm.handle_timeout(10)
            except Exception as e:
                logger.error(f"Error in dedicated LCM handling: {e}")

    def stop_handling(self) -> None:
        self._stop_event.set()
        if self._thread and threading.current_thread() != self._thread:
            self._thread.join(timeout=1.0)
        del self._lcm


class SharedLCMWrapper(LCMWrapper):
    """Wrapper for a shared LCM instance with refcounting."""

    def __init__(self, url: str | None = None):
        self._url = url
        self._lcm: lcm.LCM | None = None
        self._refcount = 0
        self._stop_event: threading.Event | None = None
        self._thread: threading.Thread | None = None

    def get_lcm(self) -> lcm.LCM:
        if self._lcm is None:
            raise RuntimeError("LCM not initialized")
        return self._lcm

    def acquire(self) -> None:
        """Increment refcount and initialize if needed."""
        if self._refcount == 0:
            self._lcm = lcm.LCM(self._url) if self._url else lcm.LCM()
            logger.debug("Created shared LCM instance")
        self._refcount += 1
        logger.debug(f"Acquired shared LCM (refcount: {self._refcount})")

    def release(self) -> None:
        """Decrement refcount and cleanup if last reference."""
        self._refcount -= 1
        logger.debug(f"Released shared LCM (refcount: {self._refcount})")
        if self._refcount <= 0:
            self.stop_handling()
            if self._lcm:
                del self._lcm
                self._lcm = None

    def start_handling(self) -> None:
        if self._thread is None or not self._thread.is_alive():
            self._stop_event = threading.Event()
            self._thread = threading.Thread(target=self._loop)
            self._thread.daemon = True
            self._thread.start()
            logger.debug("Started shared LCM handling thread")

    def _loop(self) -> None:
        while self._stop_event and not self._stop_event.is_set():
            try:
                if self._lcm:
                    self._lcm.handle_timeout(10)
            except Exception as e:
                logger.error(f"Error in shared LCM handling: {e}")

    def stop_handling(self) -> None:
        if self._stop_event:
            self._stop_event.set()
        if self._thread and threading.current_thread() != self._thread:
            self._thread.join(timeout=2.0)
        self._thread = None
        self._stop_event = None


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


def check_buffers() -> tuple[list[str], int | None]:
    """Check if buffer configuration is needed and return required commands and current size.

    Uses currently configured buffer values if they're larger than the minimum required.

    Returns:
        Tuple of (commands_needed, current_max_buffer_size)
    """
    MINIMUM_RMEM_MAX = 67108864  # 64MB recommended for high-throughput LCM
    MINIMUM_RMEM_DEFAULT = 16777216  # 16MB recommended default
    commands_needed = []
    current_max = None

    sudo = "" if check_root() else "sudo "

    # Check current buffer settings and determine target size
    try:
        result = subprocess.run(["sysctl", "net.core.rmem_max"], capture_output=True, text=True)
        if result.returncode == 0:
            current_max = int(result.stdout.split("=")[1].strip())
            # Use the larger of current or minimum
            target_max = max(current_max, MINIMUM_RMEM_MAX) if current_max else MINIMUM_RMEM_MAX
            if current_max < MINIMUM_RMEM_MAX:
                commands_needed.append(f"{sudo}sysctl -w net.core.rmem_max={target_max}")
        else:
            commands_needed.append(f"{sudo}sysctl -w net.core.rmem_max={MINIMUM_RMEM_MAX}")
    except:
        commands_needed.append(f"{sudo}sysctl -w net.core.rmem_max={MINIMUM_RMEM_MAX}")

    try:
        result = subprocess.run(["sysctl", "net.core.rmem_default"], capture_output=True, text=True)
        if result.returncode == 0:
            current_default = int(result.stdout.split("=")[1].strip())
            # Use the larger of current or minimum
            target_default = (
                max(current_default, MINIMUM_RMEM_DEFAULT)
                if current_default
                else MINIMUM_RMEM_DEFAULT
            )
            if current_default < MINIMUM_RMEM_DEFAULT:
                commands_needed.append(f"{sudo}sysctl -w net.core.rmem_default={target_default}")
        else:
            commands_needed.append(f"{sudo}sysctl -w net.core.rmem_default={MINIMUM_RMEM_DEFAULT}")
    except:
        commands_needed.append(f"{sudo}sysctl -w net.core.rmem_default={MINIMUM_RMEM_DEFAULT}")

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


@dataclass
class LCMConfig:
    ttl: int = 0
    url: str | None = None
    autoconf: bool = True
    lcm: lcm.LCM | None = None
    use_shared_lcm: bool = True  # Share LCM instance across objects in same process


@runtime_checkable
class LCMMsg(Protocol):
    msg_name: str

    @classmethod
    def lcm_decode(cls, data: bytes) -> LCMMsg:
        """Decode bytes into an LCM message instance."""
        ...

    def lcm_encode(self) -> bytes:
        """Encode this message instance into bytes."""
        ...


@dataclass
class Topic:
    topic: str = ""
    lcm_type: type[LCMMsg] | None = None

    def __str__(self) -> str:
        if self.lcm_type is None:
            return self.topic
        return f"{self.topic}#{self.lcm_type.msg_name}"


class LCMService(Service[LCMConfig]):
    default_config = LCMConfig
    l: lcm.LCM | None
    _l_lock: threading.Lock
    _wrapper: LCMWrapper | None
    _stopped: bool

    def __init__(self, **kwargs) -> None:
        global _shared_lcm_wrapper
        super().__init__(**kwargs)

        # Support passing an existing LCM instance
        if self.config.lcm:
            self.l = self.config.lcm
            self._wrapper = None
        elif self.config.use_shared_lcm:
            # Use shared wrapper
            with _shared_lcm_lock:
                if _shared_lcm_wrapper is None:
                    _shared_lcm_wrapper = SharedLCMWrapper(self.config.url)
                self._wrapper = _shared_lcm_wrapper
                self._wrapper.acquire()
                self.l = self._wrapper.get_lcm()
        else:
            # Use dedicated wrapper
            self._wrapper = DedicatedLCMWrapper(self.config.url)
            self.l = self._wrapper.get_lcm()

        self._l_lock = threading.Lock()
        self._stopped = False

    def start(self) -> None:
        # Reinitialize LCM if needed (e.g., after unpickling)
        if self.l is None and self._wrapper:
            self.l = self._wrapper.get_lcm()

        if self.config.autoconf:
            autoconf()
        else:
            try:
                check_system()
            except Exception as e:
                print(f"Error checking system configuration: {e}")

        # Start the wrapper's handling loop
        if self._wrapper:
            self._wrapper.start_handling()

    def stop(self) -> None:
        """Stop the LCM loop."""
        if self._stopped:
            return
        self._stopped = True

        if self._wrapper:
            if isinstance(self._wrapper, SharedLCMWrapper):
                with _shared_lcm_lock:
                    self._wrapper.release()
            else:
                self._wrapper.stop_handling()
            self.l = None
        elif self.l and not self.config.lcm:
            # Clean up externally provided LCM
            with self._l_lock:
                del self.l
                self.l = None

        super().stop()
