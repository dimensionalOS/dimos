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

"""Shared fakes and helpers for the RelayBridgeModule unit-test files
(test_relay_bridge_module.py, test_relay_bridge_authoring.py): no network,
no Deno, no LCM. A fake relay client is injected under `connect_with_backoff`
and fake transports under the module's `In` streams, so lazy
subscribe/unsubscribe, the encode path, publishing, and reconnect are all
observable directly.
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator, Callable
import threading
import time
from typing import Any

import pytest

from dimos.web.relay_bridge import relay_bridge_module
from dimos.web.relay_bridge.protocol import DataFrame, Msg
from dimos.web.relay_bridge.relay_bridge_module import RelayBridgeModule


class FakeWriter:
    def __init__(self) -> None:
        self.offers: list[tuple[bytes, dict[str, Any] | None]] = []
        # ts kept apart so offer-list equality asserts ignore it (a replay
        # carries its source arrival time, a live frame None).
        self.tss: list[float | None] = []

    def offer(
        self, payload: bytes, meta: dict[str, Any] | None = None, ts: float | None = None
    ) -> None:
        self.offers.append((payload, meta))
        self.tss.append(ts)


class FakeClient:
    """Duck-typed RelayClient: everything the module touches, nothing else."""

    def __init__(self, hello_error: Exception | None = None) -> None:
        self.hello_args: tuple[Any, Any] | None = None
        self.hello_error = hello_error
        self.control_msgs: asyncio.Queue[Msg | DataFrame] = asyncio.Queue()
        self.closed = asyncio.Event()
        self.writers: dict[str, FakeWriter] = {}
        self.frames: list[tuple[str, bytes, str, dict[str, Any] | None]] = []
        # Robot-opened one-shot @control frames (publish acks/nacks).
        self.control_frames: list[Msg] = []
        self.close_count = 0

    async def hello(self, timeout: float = 5.0, *, robot: Any = None, manifest: Any = None) -> None:
        self.hello_args = (robot, manifest)
        if self.hello_error is not None:
            raise self.hello_error

    def latest_writer(self, ch: str, *, stale_after: float = 0.5) -> FakeWriter:
        writer = FakeWriter()
        self.writers[ch] = writer
        return writer

    def send_frame(
        self,
        ch: str,
        payload: bytes,
        *,
        delivery: str = "reliable",
        meta: dict[str, Any] | None = None,
        ts: float | None = None,
    ) -> int:
        self.frames.append((ch, bytes(payload), delivery, meta))
        return 1

    def send_control_frame(self, msg: Msg) -> int:
        self.control_frames.append(msg)
        return 1

    async def control_messages(self) -> AsyncIterator[Msg | DataFrame]:
        while True:
            get = asyncio.ensure_future(self.control_msgs.get())
            closed = asyncio.ensure_future(self.closed.wait())
            try:
                done, _ = await asyncio.wait({get, closed}, return_when=asyncio.FIRST_COMPLETED)
            finally:
                closed.cancel()
                if not get.done():
                    get.cancel()
            if get in done:
                yield get.result()
                continue
            return

    async def close(self) -> None:
        self.close_count += 1
        self.closed.set()


class FakeTransport:
    """In-stream transport stub: counts subscribers, publishes synchronously
    (the test thread plays the LCM callback thread)."""

    def __init__(self) -> None:
        self.subscribers: list[Callable[[Any], Any]] = []
        self.unsubscribed = 0
        self.unsubscribe_attempts = 0
        self.unsubscribe_error: Exception | None = None

    def subscribe(self, cb: Callable[[Any], Any], stream: Any = None) -> Callable[[], None]:
        self.subscribers.append(cb)

        def unsubscribe() -> None:
            self.unsubscribe_attempts += 1
            if self.unsubscribe_error is not None:
                raise self.unsubscribe_error
            self.subscribers.remove(cb)
            self.unsubscribed += 1

        return unsubscribe

    def publish(self, msg: Any) -> None:
        for cb in list(self.subscribers):
            cb(msg)

    def stop(self) -> None:  # called by In.stop() during module close
        self.subscribers.clear()


class FakeRelay:
    """RelayProcess stand-in for the respawn/teardown paths."""

    def __init__(self, running: bool) -> None:
        self.running = running
        self.stops = 0

    def is_running(self) -> bool:
        return self.running

    def poll(self) -> int | None:
        return None  # what a FAILED start reads: no process at all

    def stop(self) -> None:
        self.stops += 1
        self.running = False


def wait_until(cond: Callable[[], bool], timeout: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return True
        time.sleep(0.01)
    return cond()


def flush_loop(module: RelayBridgeModule) -> None:
    """Wait until all callbacks already queued on the module loop have run."""
    loop = module._loop
    assert loop is not None
    flushed = threading.Event()
    loop.call_soon_threadsafe(flushed.set)
    assert flushed.wait(timeout=5.0)


def make_bridge(
    monkeypatch: pytest.MonkeyPatch,
    *,
    wire: tuple[str, ...] = ("color_image", "odom"),
    available_channels: tuple[str, ...] | None = None,
    manifest: dict[str, Any] | None = None,
    hello_errors: tuple[Exception | None, ...] = (),
    relay: FakeRelay | None = None,
) -> tuple[RelayBridgeModule, list[FakeClient]]:
    clients: list[FakeClient] = []

    async def fake_connect(url: str, role: str, **kwargs: Any) -> FakeClient:
        error = hello_errors[len(clients)] if len(clients) < len(hello_errors) else None
        clients.append(FakeClient(hello_error=error))
        return clients[-1]

    monkeypatch.setattr(relay_bridge_module, "connect_with_backoff", fake_connect)
    module = RelayBridgeModule(
        relay_url="https://127.0.0.1:1",
        open_browser=False,
        robot_id="unit-bot",
        available_channels=available_channels,
        manifest=manifest,
    )
    module._relay = relay  # type: ignore[assignment]  # duck-typed RelayProcess stand-in
    for ch in wire:
        getattr(module, ch).transport = FakeTransport()
    module.start()
    return module, clients


def push(module: RelayBridgeModule, client: FakeClient, msg: Msg | DataFrame) -> None:
    """Deliver a relay push onto the module's own event loop (queue affinity)."""
    assert module._loop is not None
    module._loop.call_soon_threadsafe(client.control_msgs.put_nowait, msg)


def kill_session(module: RelayBridgeModule, client: FakeClient) -> None:
    assert module._loop is not None
    module._loop.call_soon_threadsafe(client.closed.set)


def image_transport(module: RelayBridgeModule) -> FakeTransport:
    transport = module.color_image.transport
    assert isinstance(transport, FakeTransport)
    return transport


def odom_transport(module: RelayBridgeModule) -> FakeTransport:
    transport = module.odom.transport
    assert isinstance(transport, FakeTransport)
    return transport


def costmap_transport(module: RelayBridgeModule) -> FakeTransport:
    transport = module.global_costmap.transport
    assert isinstance(transport, FakeTransport)
    return transport


def start_authored(
    monkeypatch: pytest.MonkeyPatch, blueprint: Any, wire: tuple[str, ...]
) -> tuple[RelayBridgeModule, list[FakeClient]]:
    """Start a cockpit()-compiled bridge atom (possibly a generated class)
    against the fake relay client, mirroring make_bridge."""
    (atom,) = blueprint.blueprints
    clients: list[FakeClient] = []

    async def fake_connect(url: str, role: str, **kwargs: Any) -> FakeClient:
        clients.append(FakeClient())
        return clients[-1]

    monkeypatch.setattr(relay_bridge_module, "connect_with_backoff", fake_connect)
    module = atom.module(
        relay_url="https://127.0.0.1:1", open_browser=False, robot_id="unit-bot", **atom.kwargs
    )
    for ch in wire:
        getattr(module, ch).transport = FakeTransport()
    module.start()
    return module, clients


def transport_of(module: RelayBridgeModule, ch: str) -> FakeTransport:
    transport = getattr(module, ch).transport
    assert isinstance(transport, FakeTransport)
    return transport
