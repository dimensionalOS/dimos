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

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from ipaddress import IPv4Address
import os
import platform
import threading
import traceback
from typing import Any
from urllib.parse import urlsplit
import zlib

import lcm as lcm_mod

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.protocol.service.spec import BaseConfig, Service
from dimos.protocol.service.system_configurator.base import configure_system
from dimos.protocol.service.system_configurator.lcm_config import lcm_configurators
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_DEFAULT_LCM_HOST = "239.255.76.67"
_DEFAULT_LCM_PORT = "7667"
# LCM_DEFAULT_URL is used by LCM (we didn't pick that env var name)
_DEFAULT_LCM_URL = os.getenv(
    "LCM_DEFAULT_URL", f"udpm://{_DEFAULT_LCM_HOST}:{_DEFAULT_LCM_PORT}?ttl=0"
)


def _multicast_bus_url(base_url: str, offset: int) -> str:
    """Derive an adjacent multicast group with its own port.

    The dedicated port is what guarantees socket-level separation: BSD/macOS
    demultiplexes multicast by bound port, so two buses sharing a port would
    leak traffic into each other's sockets. Non-udpm and non-multicast URLs
    (memq, unicast) are returned unchanged, collapsing the pool to one bus.
    """
    parsed = urlsplit(base_url)
    if parsed.scheme != "udpm" or parsed.hostname is None or parsed.port is None:
        return base_url
    try:
        address = IPv4Address(parsed.hostname)
    except ValueError:
        return base_url
    if not address.is_multicast:
        return base_url

    octets = list(address.packed)
    octets[-1] = ((octets[-1] - 1 + offset) % 254) + 1
    host = str(IPv4Address(bytes(octets)))
    port_span = 65535 - 1024 + 1
    port = ((parsed.port - 1024 + offset) % port_span) + 1024
    return parsed._replace(netloc=f"{host}:{port}").geturl()


_STREAM_BUS_COUNT = max(0, int(os.getenv("LCM_STREAM_BUSES", "16")))

_STREAM_BUS_URLS = tuple(
    dict.fromkeys(_multicast_bus_url(_DEFAULT_LCM_URL, i + 1) for i in range(_STREAM_BUS_COUNT))
)


def lcm_url_for_channel(channel: str) -> str:
    """Map a stream channel onto one bus of the stream pool.

    LCM filters channels in userspace, after the kernel has already copied
    every datagram on a bus into every socket joined to it. Hashing each
    channel onto its own (group, port) bus moves that filtering into the
    kernel: a socket only receives the channels that share its bus, so
    high-volume streams no longer fan out into every LCM handle in every
    process. The hash must be identical across processes for publishers and
    subscribers to meet, hence crc32 rather than the seeded builtin hash().

    Set LCM_STREAM_BUSES=0 to disable sharding (single shared bus), or to a
    different count to trade fewer sockets against more channel collisions.
    """
    if not _STREAM_BUS_URLS:
        return _DEFAULT_LCM_URL
    # Typed LCM channels are "/path#pkg.Msg"; shard on the path so transports
    # that hash the topic string and spies that see the wire name meet.
    path = channel.split("#", 1)[0]
    index = zlib.crc32(path.encode()) % len(_STREAM_BUS_URLS)
    return _STREAM_BUS_URLS[index]


def lcm_bus_urls() -> tuple[str, ...]:
    """Return each distinct DimOS LCM bus URL: the default bus plus the stream pool."""
    return tuple(dict.fromkeys((_DEFAULT_LCM_URL, *_STREAM_BUS_URLS)))


def autoconf(check_only: bool = False) -> None:
    checks = lcm_configurators()
    if not checks:
        logger.error(f"System configuration not supported on {platform.system()}")
        return
    configure_system(checks, check_only=check_only)


class LCMConfig(BaseConfig):
    ttl: int = 0
    url: str = _DEFAULT_LCM_URL
    lcm: lcm_mod.LCM | None = None


_LCM_LOOP_TIMEOUT = 50


# this class just sets up cpp LCM instance
# and runs its handle loop in a thread
# higher order stuff is done by pubsub/impl/lcmpubsub.py
class LCMService(Service):
    config: LCMConfig
    l: lcm_mod.LCM | None
    _stop_event: threading.Event
    _loop_running: threading.Event
    _l_lock: threading.Lock
    _start_lock: threading.Lock
    _thread: threading.Thread | None
    _call_thread_pool: ThreadPoolExecutor | None = None
    _call_thread_pool_lock: threading.RLock = threading.RLock()

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        # we support passing an existing LCM instance
        if self.config.lcm:
            self.l = self.config.lcm
        else:
            self.l = lcm_mod.LCM(self.config.url) if self.config.url else lcm_mod.LCM()

        self._l_lock = threading.Lock()
        self._start_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._loop_running = threading.Event()
        self._thread = None

    def __getstate__(self):  # type: ignore[no-untyped-def]
        """Exclude unpicklable runtime attributes when serializing."""
        state = self.__dict__.copy()
        # Remove unpicklable attributes
        state.pop("l", None)
        state.pop("_stop_event", None)
        state.pop("_loop_running", None)
        state.pop("_thread", None)
        state.pop("_l_lock", None)
        state.pop("_start_lock", None)
        state.pop("_call_thread_pool", None)
        state.pop("_call_thread_pool_lock", None)
        return state

    def __setstate__(self, state) -> None:  # type: ignore[no-untyped-def]
        """Restore object from pickled state."""
        self.__dict__.update(state)
        # Reinitialize runtime attributes
        self.l = None
        self._stop_event = threading.Event()
        self._loop_running = threading.Event()
        self._thread = None
        self._l_lock = threading.Lock()
        self._start_lock = threading.Lock()
        self._call_thread_pool = None
        self._call_thread_pool_lock = threading.RLock()

    def start(self) -> None:
        with self._start_lock:
            if self._thread is not None and self._thread.is_alive():
                return

            # Reinitialize LCM if it's None (e.g., after unpickling)
            if self.l is None:
                if self.config.lcm:
                    self.l = self.config.lcm
                else:
                    self.l = lcm_mod.LCM(self.config.url) if self.config.url else lcm_mod.LCM()

            self._stop_event.clear()
            self._loop_running.clear()
            self._thread = threading.Thread(target=self._lcm_loop)
            self._thread.daemon = True
            self._thread.start()
            if not self._loop_running.wait(timeout=5.0):
                raise RuntimeError("LCM handler thread failed to start within 5s")

    def _lcm_loop(self) -> None:
        """LCM message handling loop."""
        primed = False
        while not self._stop_event.is_set():
            try:
                with self._l_lock:
                    if self.l is None:
                        break
                    self.l.handle_timeout(_LCM_LOOP_TIMEOUT)
            except Exception as e:
                stack_trace = traceback.format_exc()
                print(f"Error in LCM handling: {e}\n{stack_trace}")
            if not primed:
                # Signal start() only after one full poll cycle, so callers
                # don't race the first handle_timeout dispatch.
                primed = True
                self._loop_running.set()

    def stop(self) -> None:
        with self._start_lock:
            self._stop_event.set()

            if self._thread is not None:
                # Only join if we're not the LCM thread (avoid "cannot join current thread")
                if threading.current_thread() != self._thread:
                    self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
                    if self._thread.is_alive():
                        logger.warning("LCM thread did not stop cleanly within timeout")

                    self._thread = None

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
