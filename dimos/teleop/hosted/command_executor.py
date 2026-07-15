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

"""Serialized command executor with nonce dedup + safety-epoch fencing.

Robot-agnostic: a hosted-teleop command module (Go2, arm, ...) HOLDS one of
these and calls ``submit()``. A single worker thread runs blocking driver
commands off the transport callback, deduped by operator nonce, bounded backlog,
with an urgent bypass (E-STOP) and a safety epoch that aborts queued/in-flight
work after an E-STOP / operator-lost event.

Dependencies are injected (not inherited): ``send_ack(nonce, ok)`` to publish a
command ack, and ``is_estopped()`` — True while the host's E-STOP latch is set.
Lifecycle: call ``start()`` from the host's start(), ``stop()`` from its stop().
"""

from __future__ import annotations

from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
import threading
import time
from typing import Any

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class SerializedCommandExecutor:
    """Single-worker command executor + nonce dedup + safety epoch."""

    _MAX_PENDING_CMDS: int = 4
    _NONCE_TTL_SEC: float = 10.0
    _NONCE_CACHE_MAX: int = 64

    def __init__(
        self, send_ack: Callable[[Any, bool], None], is_estopped: Callable[[], bool]
    ) -> None:
        self._send_ack = send_ack
        self._is_estopped = is_estopped
        self._executor: ThreadPoolExecutor | None = None
        self._pending = 0
        self._lock = threading.Lock()
        self._safety_epoch = 0
        self._nonce_results: dict[Any, tuple[bool | None, float]] = {}
        self._urgent_threads: set[threading.Thread] = set()

    def start(self) -> None:
        """Create the single worker; call from the host's start()."""
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="HostedCmd")

    def stop(self) -> None:
        """Shut down the worker (cancel pending); call from the host's stop()."""
        self.bump_safety_epoch()
        if self._executor is not None:
            self._executor.shutdown(wait=True, cancel_futures=True)
            self._executor = None
        for thread in self._take_urgent_threads():
            if thread is threading.current_thread():
                continue
            thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            if thread.is_alive():
                logger.warning("Urgent command thread did not stop cleanly: %s", thread.name)

    # ─── safety epoch (E-STOP / operator-lost fence) ──────────────────

    def bump_safety_epoch(self) -> int:
        """Invalidate any in-flight / queued non-urgent task. Returns new epoch."""
        with self._lock:
            self._safety_epoch += 1
            return self._safety_epoch

    def safety_ok(self, epoch: int) -> bool:
        """True while no E-STOP / operator-lost has fired since `epoch`."""
        with self._lock:
            return self._safety_epoch == epoch

    @property
    def safety_epoch(self) -> int:
        """Current safety epoch (snapshot; use safety_ok() for a fence check)."""
        with self._lock:
            return self._safety_epoch

    def clear_nonces(self) -> None:
        """Drop the dedup cache — after operator-lost so a reconnected operator's
        retried nonces aren't silently re-acked from the prior session."""
        with self._lock:
            self._nonce_results.clear()

    def _take_urgent_threads(self) -> list[threading.Thread]:
        with self._lock:
            threads = list(self._urgent_threads)
            self._urgent_threads.clear()
            return threads

    # ─── submission ───────────────────────────────────────────────────

    def submit(
        self, label: str, nonce: Any, task: Callable[[int], bool], *, urgent: bool = False
    ) -> None:
        """Run a blocking command off the loop and ack it. Non-urgent commands
        serialize on one worker (bounded backlog, busy-rejected past
        _MAX_PENDING_CMDS); urgent (Damp/E-STOP) bypasses the queue.

        The task receives the safety epoch captured at SUBMIT time. If E-STOP or
        operator-lost fires before it runs, it is refused; multi-step tasks pass
        the epoch to ``safety_ok`` between steps so in-flight work can't resume
        motion after the safety event.
        """

        # E-STOP latch: only urgent work (Damp itself) may run while latched.
        if self._is_estopped() and not urgent:
            logger.warning("%s rejected: E-STOP latched", label)
            self._send_ack(nonce, False)
            return

        submit_epoch = self._safety_epoch

        if nonce is not None and not urgent:
            now = time.monotonic()
            with self._lock:
                self._nonce_results = {
                    n: (r, t)
                    for n, (r, t) in self._nonce_results.items()
                    if now - t < self._NONCE_TTL_SEC
                }
                if nonce in self._nonce_results:
                    prior, _ = self._nonce_results[nonce]
                    logger.info(
                        "%s: duplicate nonce %r — %s",
                        label,
                        nonce,
                        "re-acking" if prior is not None else "in flight",
                    )
                    if prior is not None:
                        self._send_ack(nonce, prior)
                    return
                if len(self._nonce_results) >= self._NONCE_CACHE_MAX:
                    oldest = min(self._nonce_results, key=lambda n: self._nonce_results[n][1])
                    del self._nonce_results[oldest]
                self._nonce_results[nonce] = (None, now)

        def _unwind_nonce() -> None:
            if nonce is not None:
                with self._lock:
                    self._nonce_results.pop(nonce, None)

        def runner() -> None:
            ok = False
            try:
                # Refuse if a safety event fired between submit and run (a queued
                # command must not resume motion after E-STOP / operator-lost).
                if not urgent and not self.safety_ok(submit_epoch):
                    logger.warning("%s aborted: E-STOP / operator-lost before run", label)
                else:
                    ok = bool(task(submit_epoch))
            except Exception:
                logger.exception("%s failed", label)
            finally:
                if not urgent:
                    with self._lock:
                        self._pending -= 1
            if nonce is not None and not urgent:
                with self._lock:
                    self._nonce_results[nonce] = (ok, time.monotonic())
            self._send_ack(nonce, ok)

        if urgent:

            def urgent_runner() -> None:
                try:
                    runner()
                finally:
                    with self._lock:
                        self._urgent_threads.discard(thread)

            thread = threading.Thread(target=urgent_runner, daemon=True, name=f"HostedCmd-{label}")
            with self._lock:
                self._urgent_threads.add(thread)
            thread.start()
            return

        executor = self._executor
        if executor is None:  # not started / already stopped
            _unwind_nonce()
            self._send_ack(nonce, False)
            return
        with self._lock:
            busy = self._pending >= self._MAX_PENDING_CMDS
            if busy:
                self._nonce_results.pop(nonce, None)
            else:
                self._pending += 1
        if busy:
            logger.warning("%s rejected: command backlog full", label)
            self._send_ack(nonce, False)
            return
        try:
            executor.submit(runner)
        except RuntimeError:  # shutdown raced us
            with self._lock:
                self._pending -= 1
            _unwind_nonce()
            self._send_ack(nonce, False)
