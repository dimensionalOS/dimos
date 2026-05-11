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

"""Integration tests for the WebRTC pubsub.

These tests talk to a live Cloudflare Realtime SFU and are skipped
unless ``CF_TELEOP_APP_ID`` and ``CF_TELEOP_APP_SECRET`` are set in the
environment.
"""

from __future__ import annotations

from collections.abc import Generator
import os
import threading
import time

import pytest

try:
    from dimos.protocol.pubsub.impl.webrtc_providers.cloudflare import (
        CloudflareProvider,
        _sanitize_topic as _sanitize_dc_name,
    )
    from dimos.protocol.pubsub.impl.webrtcpubsub import (
        WEBRTC_AVAILABLE,
        WebRTCPubSub,
    )
except ImportError:  # pragma: no cover - aiortc missing
    WEBRTC_AVAILABLE = False
    WebRTCPubSub = None  # type: ignore[assignment,misc]
    CloudflareProvider = None  # type: ignore[assignment,misc]
    _sanitize_dc_name = None  # type: ignore[assignment]

CF_CREDS_PRESENT = bool(os.environ.get("CF_TELEOP_APP_ID")) and bool(
    os.environ.get("CF_TELEOP_APP_SECRET")
)

skip_unless_cf = pytest.mark.skipif(
    not (WEBRTC_AVAILABLE and CF_CREDS_PRESENT),
    reason="Requires aiortc + CF_TELEOP_APP_ID/CF_TELEOP_APP_SECRET",
)


# ---------- unit tests (no network) -----------------------------------


def test_import() -> None:
    """Module should be importable even without aiortc installed."""
    from dimos.protocol.pubsub.impl import webrtcpubsub  # noqa: F401


@pytest.mark.skipif(not WEBRTC_AVAILABLE, reason="aiortc not installed")
def test_sanitize_dc_name() -> None:
    assert _sanitize_dc_name("simple") == "simple"
    assert _sanitize_dc_name("benchmark/webrtc") == "benchmark_webrtc"
    assert _sanitize_dc_name("a" * 100) == "a" * 64
    # Empty / fully-stripped names get a fallback so we never produce ""
    assert _sanitize_dc_name("///") == "___"


# ---------- live integration tests (require CF) -----------------------


@pytest.fixture
def pubsub() -> Generator[WebRTCPubSub, None, None]:
    provider = CloudflareProvider()
    ps = WebRTCPubSub(provider=provider)
    ps.start()
    try:
        yield ps
    finally:
        ps.stop()


@skip_unless_cf
@pytest.mark.timeout(60)
def test_basic_pub_sub(pubsub: WebRTCPubSub) -> None:
    """Send a single message and verify it is received."""
    received: list[tuple[bytes, str]] = []
    done = threading.Event()

    def cb(msg: bytes, topic: str) -> None:
        received.append((msg, topic))
        done.set()

    unsub = pubsub.subscribe("test_basic", cb)
    try:
        # Tiny pause for the subscribe-side DataChannel to settle.
        time.sleep(0.2)
        pubsub.publish("test_basic", b"hello world")
        assert done.wait(timeout=10.0), "Did not receive published message"
        assert received[0][0] == b"hello world"
        assert received[0][1] == "test_basic"
    finally:
        unsub()


@skip_unless_cf
@pytest.mark.timeout(60)
def test_latency(pubsub: WebRTCPubSub) -> None:
    """Measure single-message round-trip latency.

    We publish small messages back-to-back and record the delta between
    publish and callback. CF SFU + STUN over the public internet adds a
    floor of ~30-80 ms; we mostly care that this is in a sane ballpark
    (< 1s p50) and not infinite.
    """
    n = 30
    durations: list[float] = []
    received = threading.Event()
    pending_t = [0.0]

    def cb(_msg: bytes, _topic: str) -> None:
        durations.append(time.perf_counter() - pending_t[0])
        received.set()

    unsub = pubsub.subscribe("test_latency", cb)
    try:
        time.sleep(0.3)
        for i in range(n):
            received.clear()
            pending_t[0] = time.perf_counter()
            pubsub.publish("test_latency", f"ping-{i}".encode())
            assert received.wait(timeout=5.0), f"Timed out on ping {i}"

        assert len(durations) == n
        # very loose sanity bound; CF SFU is typically <250 ms
        med = sorted(durations)[len(durations) // 2]
        assert med < 1.0, f"Median latency too high: {med * 1000:.0f} ms"
        print(f"\n  WebRTC median RTT: {med * 1000:.1f} ms (n={n})")
    finally:
        unsub()


@skip_unless_cf
@pytest.mark.timeout(120)
@pytest.mark.parametrize("size", [64, 1024, 16384])
def test_throughput(pubsub: WebRTCPubSub, size: int) -> None:
    """Measure messages-per-second at a few payload sizes."""
    received_count = [0]
    target_seq = [0]
    all_received = threading.Event()
    lock = threading.Lock()

    def cb(_msg: bytes, _topic: str) -> None:
        with lock:
            received_count[0] += 1
            if target_seq[0] > 0 and received_count[0] >= target_seq[0]:
                all_received.set()

    topic = f"test_throughput_{size}"
    unsub = pubsub.subscribe(topic, cb)
    try:
        time.sleep(0.3)
        payload = bytes(size)
        deadline = time.perf_counter() + 0.5
        sent = 0
        while time.perf_counter() < deadline:
            pubsub.publish(topic, payload)
            sent += 1
            if sent >= 2000:
                break
        target_seq[0] = sent
        publish_end = time.perf_counter()
        with lock:
            if received_count[0] >= sent:
                all_received.set()
        all_received.wait(timeout=2.0)
        with lock:
            recv = received_count[0]
        elapsed = max(time.perf_counter() - publish_end + (publish_end - (deadline - 0.5)), 1e-6)
        rate = recv / elapsed if elapsed > 0 else 0.0
        print(
            f"\n  WebRTC throughput @ {size}B: sent={sent} recv={recv} "
            f"rate={rate:.0f} msgs/s elapsed={elapsed * 1000:.0f} ms"
        )
        # We don't enforce a strict floor (CI variability), just check we
        # actually moved bytes through the SFU.
        assert recv > 0, f"Received 0 messages of {size}B over WebRTC"
    finally:
        unsub()
