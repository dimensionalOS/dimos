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

"""Bytes-on-the-wire pubsub over WebRTC DataChannels.

Delegates connection and channel lifecycle to a
:class:`~dimos.protocol.pubsub.impl.webrtc.providers.spec.Provider`
(Cloudflare Realtime, broker, ...) and conforms to the standard DimOS
pubsub interface, so the grid tests in ``pubsub/test_spec.py`` and the
benchmark harness in ``pubsub/benchmark`` apply directly.
"""

from __future__ import annotations

from collections.abc import Callable
import threading
from typing import Any

from dimos.protocol.pubsub.impl.webrtc.providers.spec import Provider
from dimos.protocol.pubsub.spec import AllPubSub, accept_all
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class WebRTCPubSub(AllPubSub[str, bytes]):
    """AllPubSub[str, bytes] over a WebRTC DataChannel provider.

    WebRTC DataChannels are inherently "receive all" — messages arrive on a
    shared multiplexed channel and are demuxed by topic/fingerprint. This
    matches LCM multicast semantics, hence AllPubSub.
    """

    def __init__(self, provider: Provider) -> None:
        self._provider = provider
        self._started = False
        self._lock = threading.Lock()
        self._all_callbacks: list[Callable[[bytes, str], Any]] = []
        # Topics that already have the all-callback dispatcher attached.
        self._all_dispatch_topics: set[str] = set()

    @property
    def provider(self) -> Provider:
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

    def publish(self, topic: str, message: bytes) -> None:
        if not self._started:
            self.start()
        self._provider.publish(topic, message)

    def subscribe(self, topic: str, callback: Callable[[bytes, str], None]) -> Callable[[], None]:
        if not self._started:
            self.start()
        unsubscribe = self._provider.subscribe(topic, callback)
        self._ensure_all_dispatch(topic)
        return unsubscribe

    def _ensure_all_dispatch(self, topic: str) -> None:
        """Attach the subscribe_all dispatcher to a topic exactly once.

        A separate provider subscription per topic (instead of fanning out
        inside every per-subscription wrapper) keeps all-callback delivery at
        one call per message no matter how many subscriptions a topic has.
        """
        with self._lock:
            if topic in self._all_dispatch_topics:
                return
            self._all_dispatch_topics.add(topic)

        def _dispatch(data: bytes, t: str) -> None:
            for all_cb in list(self._all_callbacks):
                try:
                    all_cb(data, t)
                except Exception:
                    logger.exception("subscribe_all callback error")

        self._provider.subscribe(topic, _dispatch)

    def subscribe_all(
        self,
        callback: Callable[[bytes, str], Any],
        accept: Callable[[str], bool] = accept_all,
    ) -> Callable[[], None]:
        """Receive every message delivered to any subscribed topic."""

        def filtered(data: bytes, topic: str) -> None:
            if accept(topic):
                callback(data, topic)

        self._all_callbacks.append(filtered)

        def _unsub() -> None:
            try:
                self._all_callbacks.remove(filtered)
            except ValueError:
                pass

        return _unsub
