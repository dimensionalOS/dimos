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

"""Provider contract for WebRTC DataChannel backends.

``Provider`` is the runtime interface; ``ProviderConfig`` is its picklable,
hashable description. Transports carry configs across process boundaries
(module workers receive their transports by pickle) and resolve them to a
per-process singleton provider — one PeerConnection per process, shared by
every transport with an equal config.
"""

from __future__ import annotations

import asyncio
from collections.abc import Callable
import importlib.util
import threading
from typing import Any, Protocol, runtime_checkable

from pydantic import BaseModel

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# find_spec instead of importing: aiortc takes ~150ms and core.transport pulls
# this module in everywhere. Providers import aiortc lazily on start().
WEBRTC_AVAILABLE = (
    importlib.util.find_spec("aiortc") is not None and importlib.util.find_spec("httpx") is not None
)


@runtime_checkable
class Provider(Protocol):
    """WebRTC DataChannel backend (Cloudflare Realtime broker, ...).

    Implementations own signaling, ICE/DTLS, and channel lifecycle, and expose
    bytes-level publish/subscribe on named topics. DataChannels may be
    unidirectional (as with Cloudflare) or bidirectional; the provider
    handles this transparently.
    """

    def start(self) -> None: ...

    def stop(self) -> None: ...

    def publish(self, topic: str, data: bytes) -> None: ...

    def subscribe(
        self, topic: str, callback: Callable[[bytes, str], None]
    ) -> Callable[[], None]: ...

    @property
    def is_connected(self) -> bool: ...


_providers: dict[ProviderConfig, Provider] = {}
_providers_lock = threading.Lock()


def shutdown_all_providers() -> None:
    """Stop every live provider in this process and clear the registry.

    Providers are process-scoped singletons that ``Transport.stop()`` leaves
    running, so nothing else disconnects them on teardown — without this the
    broker session's DELETE never fires and the worker hangs until force-killed
    (the broker then reaps it ~30s later). Idempotent; safe with no providers.
    """
    with _providers_lock:
        providers = list(_providers.values())
        _providers.clear()
    for provider in providers:
        try:
            provider.stop()
        except Exception:
            logger.exception("Error stopping provider %s", type(provider).__name__)


class ProviderConfig(BaseModel):
    """Picklable provider factory. Equal configs share one provider per process."""

    model_config = {"frozen": True, "extra": "forbid"}

    def _create(self) -> Provider:
        raise NotImplementedError

    def provider(self) -> Provider:
        with _providers_lock:
            if self not in _providers:
                _providers[self] = self._create()
            return _providers[self]


class AsyncProviderBase:
    """Daemon asyncio loop thread + connect lifecycle shared by providers.

    ``start()`` spawns the loop thread and runs ``_connect()`` on it; a failed
    connect tears the thread down again so a later ``start()`` can retry
    cleanly. ``stop()`` runs ``_disconnect()`` and joins the thread.

    Locks: ``_lifecycle_lock`` serializes start/stop and is never taken on the
    loop thread. ``self._lock`` guards shared data (``_started``, subclass
    channel/callback state) and must only be held for short non-blocking
    sections — never across an await or a ``_run_sync``.
    """

    def __init__(self) -> None:
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._stop_ev: asyncio.Event | None = None
        self._started = False
        self._lock = threading.RLock()
        self._lifecycle_lock = threading.Lock()

    async def _connect(self) -> None:
        raise NotImplementedError

    async def _disconnect(self) -> None:
        raise NotImplementedError

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._started

    def start(self) -> None:
        with self._lifecycle_lock:
            if self.is_connected:
                return
            ready = threading.Event()
            self._thread = threading.Thread(
                target=self._run_loop, args=(ready,), daemon=True, name=type(self).__name__
            )
            self._thread.start()
            if not ready.wait(timeout=5.0):
                self._teardown()  # same cleanup contract as a failed _connect()
                raise RuntimeError(f"{type(self).__name__} event loop failed to start")
            try:
                self._run_sync(self._connect())
            except BaseException:
                # _connect may have built resources (http client, peer
                # connection, server-side session) before failing — release
                # them so a retrying caller doesn't leak one set per attempt.
                try:
                    self._run_sync(self._disconnect())
                except Exception:
                    logger.exception("Cleanup after failed %s connect", type(self).__name__)
                self._teardown()
                raise
            with self._lock:
                self._started = True

    def stop(self) -> None:
        with self._lifecycle_lock:
            if not self.is_connected:
                return
            with self._lock:
                self._started = False
            try:
                self._run_sync(self._disconnect())
            except Exception:
                logger.exception("Error during %s disconnect", type(self).__name__)
            self._teardown()

    def _teardown(self) -> None:
        loop, stop_ev = self._loop, self._stop_ev
        if loop is not None and stop_ev is not None and loop.is_running():
            loop.call_soon_threadsafe(stop_ev.set)
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        self._thread = None
        self._loop = None
        self._stop_ev = None

    def _run_loop(self, ready: threading.Event) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._loop = loop
        self._stop_ev = asyncio.Event()
        ready.set()

        try:
            loop.run_until_complete(self._stop_ev.wait())
        finally:
            tasks = asyncio.all_tasks(loop)
            for task in tasks:
                task.cancel()
            loop.run_until_complete(asyncio.gather(*tasks, return_exceptions=True))
            loop.run_until_complete(loop.shutdown_default_executor())
            loop.close()

    def _run_sync(self, coro: Any, timeout: float = 30.0) -> Any:
        assert self._loop is not None
        return asyncio.run_coroutine_threadsafe(coro, self._loop).result(timeout=timeout)


async def wait_connected(pc: Any, timeout: float = 15.0) -> None:
    """Wait until an RTCPeerConnection reaches the ``connected`` state."""
    if pc.connectionState == "connected":
        return
    ev = asyncio.Event()

    @pc.on("connectionstatechange")  # type: ignore[untyped-decorator]
    def _on_state() -> None:
        if pc.connectionState in ("connected", "failed", "closed"):
            ev.set()

    await asyncio.wait_for(ev.wait(), timeout)
    if pc.connectionState != "connected":
        raise RuntimeError(f"PeerConnection failed: {pc.connectionState}")


async def wait_open(channel: Any, timeout: float = 15.0) -> None:
    """Wait until an RTCDataChannel is open."""
    if channel.readyState == "open":
        return
    ev = asyncio.Event()

    @channel.on("open")  # type: ignore[untyped-decorator]
    def _on_open() -> None:
        ev.set()

    await asyncio.wait_for(ev.wait(), timeout)
