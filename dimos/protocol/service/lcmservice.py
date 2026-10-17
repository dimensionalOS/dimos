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
import os
import platform
import threading
import traceback
from typing import Any, ClassVar

import lcm as lcm_mod

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.global_config import TransportBackend
from dimos.protocol.service.call_pool import CallPool
from dimos.protocol.service.spec import Service, SessionConfig
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


def autoconf(check_only: bool = False) -> None:
    checks = lcm_configurators()
    if not checks:
        logger.error(f"System configuration not supported on {platform.system()}")
        return
    configure_system(checks, check_only=check_only)


class LCMConfig(SessionConfig):
    transport: ClassVar[TransportBackend] = "lcm"

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
    _startup_complete: threading.Event
    _l_lock: threading.Lock
    _pending_unsubs: list[lcm_mod.LCMSubscription]
    _thread: threading.Thread | None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        # we support passing an existing LCM instance
        if self.config.lcm:
            self.l = self.config.lcm
        else:
            self.l = lcm_mod.LCM(self.config.url) if self.config.url else lcm_mod.LCM()

        self._l_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._pending_unsubs = []
        self._startup_complete = threading.Event()
        self._thread = None
        self._call_pool = CallPool(max_workers=4)

    def __getstate__(self):  # type: ignore[no-untyped-def]
        """Exclude unpicklable runtime attributes when serializing."""
        state = self.__dict__.copy()
        # Remove unpicklable attributes
        state.pop("l", None)
        state.pop("_stop_event", None)
        state.pop("_startup_complete", None)
        state.pop("_thread", None)
        state.pop("_l_lock", None)
        state.pop("_pending_unsubs", None)
        return state

    def __setstate__(self, state) -> None:  # type: ignore[no-untyped-def]
        """Restore object from pickled state."""
        self.__dict__.update(state)
        # Reinitialize runtime attributes
        self.l = None
        self._stop_event = threading.Event()
        self._startup_complete = threading.Event()
        self._thread = None
        self._l_lock = threading.Lock()
        self._pending_unsubs = []

    def start(self) -> None:
        """Start dispatch, or raise if a previous shutdown is still in progress.

        Wait for the first poll to finish (or for its callback to stop dispatch).
        After callback shutdown or a join timeout, retry only once dispatch exits.
        """
        with self._call_pool.lock:
            if self._thread is not None and self._thread.is_alive() and self._stop_event.is_set():
                raise RuntimeError("LCM service is still stopping; cannot restart yet")
            self._call_pool.start()
            if self._thread is None or not self._thread.is_alive():
                # Reinitialize LCM if it's None (e.g., after unpickling)
                if self.l is None:
                    # Any pending unsubscribes reference the destroyed instance.
                    self._pending_unsubs.clear()
                    if self.config.lcm:
                        self.l = self.config.lcm
                    else:
                        self.l = lcm_mod.LCM(self.config.url) if self.config.url else lcm_mod.LCM()

                self._stop_event = threading.Event()
                # Waiters retain their own run's event across a later restart.
                self._startup_complete = threading.Event()
                self._thread = threading.Thread(target=self._lcm_loop)
                self._thread.daemon = True
                self._thread.start()
            thread = self._thread
            startup_complete = self._startup_complete
        # The first dispatch may call stop(), which also needs the lifecycle lock.
        if threading.current_thread() is not thread and not startup_complete.wait(timeout=5.0):
            raise RuntimeError("LCM handler thread failed to start within 5s")

    def _defer_unsubscribe(self, subscription: lcm_mod.LCMSubscription) -> None:
        """Queue an unsubscribe for the loop thread to apply between polls.

        lcm-python's unsubscribe is not safe against a concurrently
        dispatching handle_timeout (it frees the subscription's state under
        the handler → segfault), and excluding the loop with a lock would
        block the caller for up to a poll cycle — a deadlock hazard for
        callers on event-loop threads. Deferring to the dispatch thread
        makes unsubscribe non-blocking and race-free; the subscription may
        receive callbacks for at most one more poll.
        """
        if self.l is None:
            return
        self._pending_unsubs.append(subscription)

    def _lcm_loop(self) -> None:
        """LCM message handling loop."""
        primed = False
        try:
            while not self._stop_event.is_set():
                try:
                    with self._l_lock:
                        if self.l is None:
                            break
                        while self._pending_unsubs:
                            self.l.unsubscribe(self._pending_unsubs.pop())
                        self.l.handle_timeout(_LCM_LOOP_TIMEOUT)
                except Exception as e:
                    stack_trace = traceback.format_exc()
                    print(f"Error in LCM handling: {e}\n{stack_trace}")
                if not primed:
                    # Signal start() only after one full poll cycle, so callers
                    # don't race the first handle_timeout dispatch.
                    primed = True
                    self._startup_complete.set()
        finally:
            # A callback may request stop while handle_timeout holds _l_lock.
            # Only release the native instance after dispatch has unwound.
            self._release_lcm()
            self._startup_complete.set()

    def _release_lcm(self) -> None:
        if not self.config.lcm:
            with self._l_lock:
                # Destroying the instance frees its subscriptions; drop
                # pendings so a restart doesn't apply them to a new one.
                self._pending_unsubs.clear()
                self.l = None

    def stop(self) -> None:
        """Request shutdown, joining dispatch unless called from its callback.

        A join timeout leaves the native instance alive until dispatch exits.
        Callbacks and pool workers request pool shutdown without waiting for it.
        Every other caller waits for the handler pool to drain before stopping.
        Restart is rejected while a stop call or stopping dispatch is still active.
        """
        with self._call_pool.stop_scope():
            with self._call_pool.lock:
                self._stop_event.set()
                thread = self._thread
                if thread is None:
                    # Construction opens LCM even if start() was never called.
                    self._release_lcm()

            self._shutdown_thread_pool()
            if thread is not None and threading.current_thread() is not thread:
                # Callbacks can stop while another caller joins, without a lock cycle.
                thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
                if thread.is_alive():
                    logger.warning("LCM thread did not stop cleanly within timeout")
                else:
                    with self._call_pool.lock:
                        if self._thread is thread:
                            self._thread = None

    def _shutdown_thread_pool(self) -> None:
        # Dispatch cannot wait for handlers that may themselves stop dispatch.
        with self._call_pool.lock:
            is_dispatch_thread = threading.current_thread() is self._thread
        self._call_pool.shutdown(wait=not is_dispatch_thread)

    def _get_call_thread_pool(self) -> ThreadPoolExecutor:
        return self._call_pool.get()
