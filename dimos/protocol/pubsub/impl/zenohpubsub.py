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


def _topic_to_key_expr(topic: Topic) -> str:
    """Convert a Topic to a Zenoh key expression.

    Embeds the lcm_type in the key using '/' instead of '#' (which is
    forbidden in Zenoh key expressions). This mirrors how LCM channels
    carry type info in the channel name for subscribe_all decoding.

    Examples:
        Topic("dimos/cmd_vel", Twist) → "dimos/cmd_vel/geometry_msgs.Twist"
        Topic("dimos/data")           → "dimos/data"

    Known limitation: the type name becomes a path segment. If a topic
    name itself looks like a type name (e.g., "dimos/geometry_msgs.Twist"),
    _key_expr_to_topic may misparse it on the receiving end. In practice
    this doesn't happen because topic names come from stream names (e.g.,
    "cmd_vel", "lidar"), not from type names.
    """
    base = topic.topic if isinstance(topic.topic, str) else topic.pattern
    if topic.lcm_type is not None:
        return f"{base}/{topic.lcm_type.msg_name}"
    return base


def _key_expr_to_topic(key_expr: str, default_lcm_type: type | None = None) -> Topic:
    """Reconstruct a Topic from a Zenoh key expression.

    Parses the last '/' segment and attempts to resolve it as a DimosMsg
    type via resolve_msg_type(). If resolution succeeds, the segment is
    treated as the type suffix and the remainder as the base topic.

    Examples:
        "dimos/cmd_vel/geometry_msgs.Twist" → Topic("dimos/cmd_vel", Twist)
        "dimos/data"                        → Topic("dimos/data", default_lcm_type)
        "dimos/data/unknown.Foo"            → Topic("dimos/data/unknown.Foo", default_lcm_type)

    Known limitation: if a topic's base path ends with a segment that
    happens to match a registered DimosMsg type name, this function will
    incorrectly split it. See _topic_to_key_expr docstring for details.
    """
    from dimos.msgs.helpers import resolve_msg_type

    # Try to resolve the last segment as a message type
    parts = key_expr.rsplit("/", 1)
    if len(parts) == 2:
        base, maybe_type = parts
        lcm_type = resolve_msg_type(maybe_type)
        if lcm_type is not None:
            return Topic(topic=base, lcm_type=lcm_type)
    return Topic(topic=key_expr, lcm_type=default_lcm_type)


class ZenohPubSubBase(ZenohService, AllPubSub[Topic, bytes]):
    """Raw bytes pub/sub over Zenoh.

    Publishers are cached per-topic to avoid re-declaring on every publish.
    Subscribers are tracked for cleanup on stop().
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._publishers: dict[str, zenoh.Publisher] = {}
        self._publisher_lock = threading.Lock()
        self._subscribers: list[zenoh.Subscriber[Any]] = []
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
        key_expr = _topic_to_key_expr(topic)
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
        key_expr = _topic_to_key_expr(topic)

        def on_sample(sample: zenoh.Sample) -> None:
            try:
                data = sample.payload.to_bytes()
            except Exception:
                logger.error(f"Error reading payload from {key_expr}", exc_info=True)
                return
            # Reconstruct topic with type info from the key expression
            # (needed for subscribe_all where the subscription topic has no lcm_type)
            recv_topic = _key_expr_to_topic(str(sample.key_expr), topic.lcm_type)
            callback(data, recv_topic)

        sub = self.session.declare_subscriber(key_expr, on_sample)
        with self._subscriber_lock:
            self._subscribers.append(sub)

        undeclared = False

        def unsubscribe() -> None:
            nonlocal undeclared
            if undeclared:
                return
            undeclared = True
            with self._subscriber_lock:
                if sub not in self._subscribers:
                    return  # Already removed by stop() — stop() owns the undeclare
                self._subscribers.remove(sub)
            sub.undeclare()  # type: ignore[no-untyped-call]

        return unsubscribe

    def subscribe_all(self, callback: Callable[[bytes, Topic], Any]) -> Callable[[], None]:
        """Subscribe to all dimos key expressions via wildcard."""
        return self.subscribe(Topic("dimos/**"), callback)

    def stop(self) -> None:
        """Clean up publishers and subscribers."""
        with self._subscriber_lock:
            for subscriber in self._subscribers:
                subscriber.undeclare()  # type: ignore[no-untyped-call]
            self._subscribers.clear()
        with self._publisher_lock:
            for publisher in self._publishers.values():
                publisher.undeclare()  # type: ignore[no-untyped-call]
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
