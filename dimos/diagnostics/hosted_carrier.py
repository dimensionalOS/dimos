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

"""Dedicated diagnostic message type over providers with fixed directional channels."""

from __future__ import annotations

from collections.abc import Callable
import hashlib
from typing import Any

from dimos.core.stream import Stream, Transport
from dimos.msgs.std_msgs.String import String
from dimos.protocol.pubsub.impl.webrtc.providers.spec import Provider, ProviderConfig

_MAGIC = b"DIMOS_TYPED_TRACE_V1\0"


class HostedTraceCarrier(Transport[Any]):
    """Both sides choose their publish/receive channels explicitly.

    The broker's physical channel names remain unchanged. A dedicated type magic
    and run/link namespace prevent legacy command bytes or other traced links
    from being decoded as this carrier. Original typed bytes remain inside the
    versioned envelope. No operator or robot session is created implicitly.
    """

    def __init__(
        self, topic: str, config: ProviderConfig, publish_channel: str, receive_channel: str
    ) -> None:
        if not publish_channel or not receive_channel:
            raise ValueError("hosted tracing needs explicit directional channels")
        self.channel = topic
        self.config = config
        self.publish_channel = publish_channel
        self.receive_channel = receive_channel
        self.prefix = _MAGIC + hashlib.sha256(topic.encode()).digest()[:16]
        self._provider: Provider | None = None

    def cbor_encode(self) -> dict[str, Any]:
        return {
            "topic": self.channel,
            "config": self.config,
            "publish_channel": self.publish_channel,
            "receive_channel": self.receive_channel,
        }

    def start(self) -> None:
        if self._provider is None:
            provider = self.config.provider()
            provider.start()
            self._provider = provider

    def broadcast(self, selfstream: Stream[Any] | None, value: Any) -> None:
        self.start()
        wire = self.prefix + value.data.encode("utf-8")
        if len(wire) > 32 * 1024:
            raise ValueError("hosted trace carrier exceeds provider's 32 KiB message budget")
        assert self._provider is not None
        self._provider.publish(self.publish_channel, wire)

    def subscribe(
        self, callback: Callable[[Any], Any], selfstream: Stream[Any] | None = None
    ) -> Callable[[], None]:
        self.start()

        def receive(data: bytes, _channel: str) -> None:
            if len(data) <= 32 * 1024 and data.startswith(self.prefix):
                try:
                    text = data[len(self.prefix) :].decode("utf-8")
                except UnicodeDecodeError:
                    return
                callback(String(data=text))

        assert self._provider is not None
        return self._provider.subscribe(self.receive_channel, receive)

    def stop(self) -> None:
        # The provider is process-scoped and may serve unrelated streams.
        pass
