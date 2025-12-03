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

logger = setup_logger()


@cache
def check_root() -> bool:
    """Return True if the current process is running as root (UID 0)."""
    try:
        return os.geteuid() == 0
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
    except:
        commands_needed.append(f"{sudo}sysctl -w net.core.rmem_max=2097152")

    try:
        result = subprocess.run(["sysctl", "net.core.rmem_default"], capture_output=True, text=True)
        current_default = (
            int(result.stdout.split("=")[1].strip()) if result.returncode == 0 else None
        )
        if not current_default or current_default < 2097152:
            commands_needed.append(f"{sudo}sysctl -w net.core.rmem_default=2097152")
    except:
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


@dataclass
class LCMConfig:
    ttl: int = 0
    url: str | None = None
    autoconf: bool = True
    lcm: lcm.LCM | None = None


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
    _stop_event: threading.Event
    _l_lock: threading.Lock
    _thread: threading.Thread | None
    _call_thread_pool: ThreadPoolExecutor | None = None
    _call_thread_pool_lock: threading.RLock = threading.RLock()

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)

        # we support passing an existing LCM instance
        if self.config.lcm:
            # TODO: If we pass LCM in, it's unsafe to use in this thread and the _loop thread.
            self.l = self.config.lcm
        else:
            self.l = lcm.LCM(self.config.url) if self.config.url else lcm.LCM()

        self._l_lock = threading.Lock()

        self._stop_event = threading.Event()
        self._thread = None

    def __getstate__(self):  # type: ignore[no-untyped-def]
        """Exclude unpicklable runtime attributes when serializing."""
        state = self.__dict__.copy()
        # Remove unpicklable attributes
        state.pop("l", None)
        state.pop("_stop_event", None)
        state.pop("_thread", None)
        state.pop("_l_lock", None)
        state.pop("_call_thread_pool", None)
        state.pop("_call_thread_pool_lock", None)
        return state

    def __setstate__(self, state) -> None:  # type: ignore[no-untyped-def]
        """Restore object from pickled state."""
        self.__dict__.update(state)
        # Reinitialize runtime attributes
        self.l = None
        self._stop_event = threading.Event()
        self._thread = None
        self._l_lock = threading.Lock()
        self._call_thread_pool = None
        self._call_thread_pool_lock = threading.RLock()

    def start(self) -> None:
        # Reinitialize LCM if it's None (e.g., after unpickling)
        if self.l is None:
            if self.config.lcm:
                self.l = self.config.lcm
            else:
                self.l = lcm.LCM(self.config.url) if self.config.url else lcm.LCM()

        if self.config.autoconf:
            autoconf()
        else:
            try:
                check_system()
            except Exception as e:
                print(f"Error checking system configuration: {e}")

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._lcm_loop)
        self._thread.daemon = True
        self._thread.start()

    def _lcm_loop(self) -> None:
        """LCM message handling loop."""
        while not self._stop_event.is_set():
            try:
                with self._l_lock:
                    if self.l is None:
                        break
                    self.l.handle_timeout(50)
            except Exception as e:
                stack_trace = traceback.format_exc()
                print(f"Error in LCM handling: {e}\n{stack_trace}")

    def stop(self) -> None:
        """Stop the LCM loop."""
        self._stop_event.set()
        if self._thread is not None:
            # Only join if we're not the LCM thread (avoid "cannot join current thread")
            if threading.current_thread() != self._thread:
                self._thread.join(timeout=1.0)
                if self._thread.is_alive():
                    logger.warning("LCM thread did not stop cleanly within timeout")

        # Clean up LCM instance if we created it
        if not self.config.lcm:
            with self._l_lock:
                if self.l is not None:
                    del self.l
                    self.l = None

        with self._call_thread_pool_lock:
            if self._call_thread_pool:
                # Check if we're being called from within the thread pool
                # If so, we can't wait for shutdown (would cause "cannot join current thread")
                current_thread = threading.current_thread()
                is_pool_thread = False

                # Check if current thread is one of the pool's threads
                # ThreadPoolExecutor threads have names like "ThreadPoolExecutor-N_M"
                if hasattr(self._call_thread_pool, "_threads"):
                    is_pool_thread = current_thread in self._call_thread_pool._threads
                elif "ThreadPoolExecutor" in current_thread.name:
                    # Fallback: check thread name pattern
                    is_pool_thread = True

                # Don't wait if we're in a pool thread to avoid deadlock
                self._call_thread_pool.shutdown(wait=not is_pool_thread)
                self._call_thread_pool = None

    def _get_call_thread_pool(self) -> ThreadPoolExecutor:
        with self._call_thread_pool_lock:
            if self._call_thread_pool is None:
                self._call_thread_pool = ThreadPoolExecutor(max_workers=4)
            return self._call_thread_pool
