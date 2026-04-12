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

"""Zenoh PubSub implementation.

Provides raw bytes pub/sub over Zenoh (ZenohPubSubBase) and
encoder-composed variants (Zenoh, PickleZenoh).
"""

from __future__ import annotations

from collections.abc import Callable
import threading
from typing import TYPE_CHECKING, Any

from dimos.protocol.pubsub.encoders import LCMEncoderMixin, PickleEncoderMixin
from dimos.protocol.pubsub.impl.lcmpubsub import Topic
from dimos.protocol.pubsub.spec import AllPubSub
from dimos.protocol.service.zenohservice import ZenohService
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    import zenoh

logger = setup_logger()


class ZenohPubSubBase(ZenohService, AllPubSub[Topic, bytes]):
    """Raw bytes pub/sub over Zenoh.

    Publishers are cached per-topic to avoid re-declaring on every publish.
    Subscribers are tracked for cleanup on stop().
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._publishers: dict[str, zenoh.Publisher] = {}
        self._publisher_lock = threading.Lock()
        self._subscribers: list[zenoh.Subscriber] = []
        self._subscriber_lock = threading.Lock()

    def _get_publisher(self, key_expr: str) -> zenoh.Publisher:
        """Get or create a Publisher for the given key expression."""
        with self._publisher_lock:
            if key_expr not in self._publishers:
                self._publishers[key_expr] = self.session.declare_publisher(key_expr)
            return self._publishers[key_expr]

    def publish(self, topic: Topic, message: bytes) -> None:
        """Publish bytes to a Zenoh key expression.

        Transport-level errors (session closed, invalid key expression) are
        logged but not raised. Delivery guarantees are handled by Zenoh's
        reliability protocol (RELIABLE mode retransmits at each hop) — these
        do not surface as exceptions from put().
        """
        key_expr = topic.topic if isinstance(topic.topic, str) else topic.pattern
        try:
            publisher = self._get_publisher(key_expr)
            publisher.put(message)
        except Exception:
            logger.error(f"Error publishing to {key_expr}", exc_info=True)

    def subscribe(
        self, topic: Topic, callback: Callable[[bytes, Topic], None]
    ) -> Callable[[], None]:
        """Subscribe to a Zenoh key expression.

        Returns an unsubscribe callable.
        """
        key_expr = topic.topic if isinstance(topic.topic, str) else topic.pattern

        def on_sample(sample: zenoh.Sample) -> None:
            callback(sample.payload.to_bytes(), topic)

        sub = self.session.declare_subscriber(key_expr, on_sample)
        with self._subscriber_lock:
            self._subscribers.append(sub)

        undeclared = False

        def unsubscribe() -> None:
            nonlocal undeclared
            if undeclared:
                return
            undeclared = True
            sub.undeclare()
            with self._subscriber_lock:
                try:
                    self._subscribers.remove(sub)
                except ValueError:
                    pass

        return unsubscribe

    def subscribe_all(self, callback: Callable[[bytes, Topic], Any]) -> Callable[[], None]:
        """Subscribe to all dimos key expressions via wildcard."""
        return self.subscribe(Topic("dimos/**"), callback)

    def stop(self) -> None:
        """Clean up publishers and subscribers."""
        with self._subscriber_lock:
            for subscriber in self._subscribers:
                subscriber.undeclare()
            self._subscribers.clear()
        with self._publisher_lock:
            for publisher in self._publishers.values():
                publisher.undeclare()
            self._publishers.clear()
        super().stop()


class Zenoh(  # type: ignore[misc]
    LCMEncoderMixin,  # type: ignore[type-arg]
    ZenohPubSubBase,
):
    """Zenoh pub/sub with LCM encoding for typed DimosMsg."""

    ...


class PickleZenoh(  # type: ignore[misc]
    PickleEncoderMixin,  # type: ignore[type-arg]
    ZenohPubSubBase,
):
    """Zenoh pub/sub with pickle encoding for arbitrary Python objects."""

    ...


__all__ = [
    "PickleZenoh",
    "Topic",
    "Zenoh",
    "ZenohPubSubBase",
]
