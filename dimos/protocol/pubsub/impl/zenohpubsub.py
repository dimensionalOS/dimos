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

from collections.abc import Callable
from dataclasses import dataclass
import threading
from typing import TYPE_CHECKING, Any, TypeAlias

from dimos.protocol.pubsub.spec import PubSub
from dimos.protocol.service.zenohservice import ZenohService
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    import zenoh

logger = setup_logger()


@dataclass(frozen=True)
class Topic:
    """Represents a Zenoh topic (key expression)."""

    name: str

    def __str__(self) -> str:
        return self.name


MessageCallback: TypeAlias = Callable[[Any, Topic], None]


class ZenohPubSub(ZenohService, PubSub[Topic, Any]):
    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._publishers: dict[Topic, zenoh.Publisher] = {}
        self._publisher_lock = threading.Lock()
        self._subscribers: list[zenoh.Subscriber] = []
        self._subscriber_lock = threading.Lock()

    def _get_publisher(self, topic: Topic) -> zenoh.Publisher:
        """Get or create a Publisher for the given topic."""
        with self._publisher_lock:
            if topic not in self._publishers:
                self._publishers[topic] = self.session.declare_publisher(topic.name)
            return self._publishers[topic]

    def publish(self, topic: Topic, message: bytes | str) -> None:
        """Publish a message to a Zenoh topic."""
        publisher = self._get_publisher(topic)
        try:
            publisher.put(message)
        except Exception as e:
            logger.error(f"Error publishing to topic {topic}: {e}", exc_info=True)

    def subscribe(self, topic: Topic, callback: MessageCallback) -> Callable[[], None]:
        """Subscribe to a Zenoh topic with a callback.

        Each call declares its own Zenoh subscriber (Zenoh spawns a
        background thread per callback handler). Unsubscribe undeclares it.
        """

        def on_sample(sample: zenoh.Sample) -> None:
            callback(sample.payload.to_bytes(), topic)

        sub = self.session.declare_subscriber(topic.name, on_sample)
        with self._subscriber_lock:
            self._subscribers.append(sub)

        def unsubscribe() -> None:
            sub.undeclare()
            with self._subscriber_lock:
                try:
                    self._subscribers.remove(sub)
                except ValueError:
                    pass

        return unsubscribe

    def stop(self) -> None:
        """Stop the Zenoh pub/sub and clean up resources."""
        with self._subscriber_lock:
            for subscriber in self._subscribers:
                subscriber.undeclare()
            self._subscribers.clear()
        with self._publisher_lock:
            for publisher in self._publishers.values():
                publisher.undeclare()
            self._publishers.clear()
        super().stop()


__all__ = [
    "MessageCallback",
    "Topic",
    "ZenohPubSub",
]
