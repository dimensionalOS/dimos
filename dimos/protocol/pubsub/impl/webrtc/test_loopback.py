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

"""Loopback provider tests — real aiortc SCTP over localhost, keyless."""

from __future__ import annotations

from collections.abc import Iterator
import threading

import pytest

from dimos.protocol.pubsub.impl.webrtc.providers.spec import WEBRTC_AVAILABLE

if not WEBRTC_AVAILABLE:
    pytest.skip("aiortc not installed", allow_module_level=True)

from dimos.protocol.pubsub.impl.webrtc.providers.loopback import LoopbackConfig, LoopbackProvider

TIMEOUT = 10.0


@pytest.fixture
def provider() -> Iterator[LoopbackProvider]:
    p = LoopbackProvider()
    p.start()
    yield p
    p.stop()


class Collector:
    """Thread-safe callback sink with event-based waiting (no sleeps)."""

    def __init__(self) -> None:
        self.messages: list[tuple[bytes, str]] = []
        self._event = threading.Event()

    def __call__(self, data: bytes, topic: str) -> None:
        self.messages.append((data, topic))
        self._event.set()

    def wait(self, count: int = 1) -> list[tuple[bytes, str]]:
        while len(self.messages) < count:
            assert self._event.wait(TIMEOUT), f"got {len(self.messages)}/{count} messages"
            self._event.clear()
        return self.messages


def test_roundtrip(provider: LoopbackProvider) -> None:
    got = Collector()
    provider.subscribe("test/topic", got)
    provider.publish("test/topic", b"hello")
    assert got.wait() == [(b"hello", "test/topic")]


def test_topics_are_isolated(provider: LoopbackProvider) -> None:
    got_a, got_b = Collector(), Collector()
    provider.subscribe("test/a", got_a)
    provider.subscribe("test/b", got_b)
    provider.publish("test/a", b"for-a")
    provider.publish("test/b", b"for-b")
    assert got_a.wait() == [(b"for-a", "test/a")]
    assert got_b.wait() == [(b"for-b", "test/b")]


def test_large_message(provider: LoopbackProvider) -> None:
    got = Collector()
    provider.subscribe("test/large", got)
    payload = bytes(range(256)) * 128  # 32 KiB through real SCTP fragmentation
    provider.publish("test/large", payload)
    assert got.wait() == [(payload, "test/large")]


def test_unsubscribe_stops_delivery(provider: LoopbackProvider) -> None:
    dropped, kept = Collector(), Collector()
    unsub = provider.subscribe("test/topic", dropped)
    provider.subscribe("test/topic", kept)
    unsub()
    provider.publish("test/topic", b"after-unsub")
    kept.wait()
    assert dropped.messages == []


def test_restart_after_stop() -> None:
    provider = LoopbackConfig().provider()
    provider.start()
    provider.stop()
    provider.start()
    try:
        got = Collector()
        provider.subscribe("test/restart", got)
        provider.publish("test/restart", b"alive")
        assert got.wait() == [(b"alive", "test/restart")]
    finally:
        provider.stop()


def test_config_singleton() -> None:
    assert LoopbackConfig().provider() is LoopbackConfig().provider()
    assert LoopbackConfig(ordered=False).provider() is not LoopbackConfig().provider()
