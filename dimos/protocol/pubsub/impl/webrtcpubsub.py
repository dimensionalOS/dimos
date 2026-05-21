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

"""WebRTC DataChannel pubsub transport.

Two layers:

* ``DataChannelProvider`` — abstract interface for managing WebRTC
  DataChannels. Implementations handle signaling, PeerConnection
  lifecycle, and DataChannel creation for a specific SFU backend
  (Cloudflare Realtime, LiveKit, etc.).

* ``WebRTCPubSub`` — thin pubsub facade that delegates to a provider.
  Exposes the standard ``publish``/``subscribe`` bytes-on-the-wire
  interface used by other DimOS transports.

Providers are in ``dimos/protocol/pubsub/impl/webrtc_providers/``.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class DataChannelProvider(ABC):
    """Abstract interface for WebRTC DataChannel backends.

    A provider manages the WebRTC PeerConnection(s) and exposes
    publish/subscribe semantics over named DataChannels. Implementations
    handle signaling, ICE, DTLS, and channel lifecycle for their specific
    SFU (Cloudflare Realtime, LiveKit, Janus, etc.).

    DataChannels may be unidirectional (CF) or bidirectional (LiveKit).
    The provider handles this transparently.
    """

    @abstractmethod
    def start(self) -> None:
        """Connect to the SFU and establish transport."""

    @abstractmethod
    def stop(self) -> None:
        """Disconnect and release resources."""

    @abstractmethod
    def publish(self, topic: str, data: bytes) -> None:
        """Send bytes on a named topic/channel."""

    @abstractmethod
    def subscribe(self, topic: str, callback: Callable[[bytes, str], None]) -> Callable[[], None]:
        """Subscribe to bytes on a named topic. Returns unsubscribe callable."""

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """Whether the provider is connected and ready."""


class WebRTCPubSub:
    """Bytes-on-the-wire pubsub over WebRTC DataChannels.

    Delegates to a :class:`DataChannelProvider` implementation.
    Same interface as ``LCMPubSubBase`` and ``BytesSharedMemory``.
    """

    def __init__(self, provider: DataChannelProvider) -> None:
        self._provider = provider
        self._started = False

    @property
    def provider(self) -> DataChannelProvider:
        return self._provider

    def start(self) -> None:
        if self._started:
            return
        self._provider.start()
        self._started = True

    def stop(self) -> None:
        if not self._started:
            return
        self._provider.stop()
        self._started = False

    def publish(self, topic: str, msg: bytes) -> None:
        if not self._started:
            self.start()
        self._provider.publish(topic, msg)

    def subscribe(self, topic: str, callback: Callable[[bytes, str], None]) -> Callable[[], None]:
        if not self._started:
            self.start()
        return self._provider.subscribe(topic, callback)


# Re-export provider availability flag
try:
    from dimos.protocol.pubsub.impl.webrtc_providers.cloudflare import (
        CLOUDFLARE_AVAILABLE,
        CloudflareProvider,
    )

    WEBRTC_AVAILABLE = CLOUDFLARE_AVAILABLE
except ImportError:
    WEBRTC_AVAILABLE = False
    CloudflareProvider = None  # type: ignore[assignment,misc]
    CLOUDFLARE_AVAILABLE = False

try:
    from dimos.protocol.pubsub.impl.webrtc_providers.broker import (
        BROKER_AVAILABLE,
        BrokerProvider,
    )
except ImportError:
    BROKER_AVAILABLE = False  # type: ignore[assignment]
    BrokerProvider = None  # type: ignore[assignment,misc]


__all__ = [
    "BROKER_AVAILABLE",
    "WEBRTC_AVAILABLE",
    "BrokerProvider",
    "CloudflareProvider",
    "DataChannelProvider",
    "WebRTCPubSub",
]
