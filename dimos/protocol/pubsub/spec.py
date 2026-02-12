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

from abc import ABC, abstractmethod
import asyncio
from collections.abc import AsyncIterator, Callable
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import Any, Generic, Protocol, TypeVar, runtime_checkable

MsgT = TypeVar("MsgT")
TopicT = TypeVar("TopicT")
MsgT_co = TypeVar("MsgT_co", covariant=True)
TopicT_co = TypeVar("TopicT_co", covariant=True)


class PubSubBaseMixin(Generic[TopicT, MsgT]):
    """Mixin class providing sugar methods for PubSub implementations.
    Depends on the basic publish and subscribe methods being implemented.
    """

    def subscribe(
        self, topic: TopicT, callback: Callable[[MsgT, TopicT], None]
    ) -> Callable[[], None]:
        """Subscribe to a topic. Implemented by subclasses."""
        raise NotImplementedError

    @dataclass(slots=True)
    class _Subscription:
        _bus: "PubSubBaseMixin[Any, Any]"
        _topic: Any
        _cb: Callable[[Any, Any], None]
        _unsubscribe_fn: Callable[[], None]

        def unsubscribe(self) -> None:
            self._unsubscribe_fn()

        def __enter__(self) -> "PubSubBaseMixin._Subscription":
            return self

        def __exit__(self, *exc: Any) -> None:
            self.unsubscribe()

    def sub(self, topic: TopicT, cb: Callable[[MsgT, TopicT], None]) -> "_Subscription":
        unsubscribe_fn = self.subscribe(topic, cb)
        return self._Subscription(self, topic, cb, unsubscribe_fn)

    async def aiter(self, topic: TopicT, *, max_pending: int | None = None) -> AsyncIterator[MsgT]:
        q: asyncio.Queue[MsgT] = asyncio.Queue(maxsize=max_pending or 0)

        def _cb(msg: MsgT, topic: TopicT) -> None:
            q.put_nowait(msg)

        unsubscribe_fn = self.subscribe(topic, _cb)
        try:
            while True:
                yield await q.get()
        finally:
            unsubscribe_fn()

    @asynccontextmanager
    async def queue(
        self, topic: TopicT, *, max_pending: int | None = None
    ) -> AsyncIterator[asyncio.Queue[MsgT]]:
        q: asyncio.Queue[MsgT] = asyncio.Queue(maxsize=max_pending or 0)

        def _queue_cb(msg: MsgT, topic: TopicT) -> None:
            q.put_nowait(msg)

        unsubscribe_fn = self.subscribe(topic, _queue_cb)
        try:
            yield q
        finally:
            unsubscribe_fn()


class PubSub(PubSubBaseMixin[TopicT, MsgT], ABC):
    """Abstract base class for pub/sub implementations with sugar methods."""

    @abstractmethod
    def publish(self, topic: TopicT, message: MsgT) -> None:
        """Publish a message to a topic."""
        ...

    @abstractmethod
    def subscribe(
        self, topic: TopicT, callback: Callable[[MsgT, TopicT], None]
    ) -> Callable[[], None]:
        """Subscribe to a topic with a callback. returns unsubscribe function"""
        ...


# AllPubSub and DiscoveryPubSub are complementary mixins:
#
# AllPubsub supports subscribing to all topics (Redis, LCM, MQTT)
# DiscoveryPubSub supports discovering new topics (ROS)
#
# These capabilities are orthogonal but they can implement one another.
# Implementations should subclass whichever matches their native capability.
# The other method will be synthesized automatically.
#
# - AllPubSub: Native support for subscribing to all topics at once.
#   Provides a default subscribe_new_topics() by tracking seen topics.
#
# - DiscoveryPubSub: Native support for discovering new topics as they appear.
#   Provides a default subscribe_all() by subscribing to each discovered topic.
class AllPubSub(PubSub[TopicT, MsgT], ABC):
    """Mixin for PubSub that supports subscribing to all topics.

    Subclass from this if you support native subscribe-all (e.g. MQTT #, Redis *).
    Provides a default subscribe_new_topics() implementation.
    """

    @abstractmethod
    def subscribe_all(self, callback: Callable[[MsgT, TopicT], Any]) -> Callable[[], None]:
        """Subscribe to all topics."""
        ...

    def subscribe_new_topics(self, callback: Callable[[TopicT], Any]) -> Callable[[], None]:
        """Discover new topics by tracking seen topics from subscribe_all."""
        import threading

        seen: set[TopicT] = set()
        lock = threading.Lock()

        def on_msg(msg: MsgT, topic: TopicT) -> None:
            with lock:
                if topic not in seen:
                    seen.add(topic)
                    callback(topic)

        return self.subscribe_all(on_msg)


# This is for ros for now
class DiscoveryPubSub(PubSub[TopicT, MsgT], ABC):
    """Mixin for PubSub that supports discovery of topics.

    Subclass from this if you support topic discovery (e.g. MQTT, Redis, NATS, RabbitMQ).
    """

    @abstractmethod
    def subscribe_new_topics(self, callback: Callable[[TopicT], Any]) -> Callable[[], None]:
        """Get notified when new topics are discovered."""
        ...

    def subscribe_all(self, callback: Callable[[MsgT, TopicT], Any]) -> Callable[[], None]:
        """Subscribe to all topics by subscribing to each discovered topic."""
        import threading

        subscriptions: list[Callable[[], None]] = []
        lock = threading.Lock()

        def on_new_topic(topic: TopicT) -> None:
            unsub = self.subscribe(topic, callback)
            with lock:
                subscriptions.append(unsub)

        discovery_unsub = self.subscribe_new_topics(on_new_topic)

        def unsubscribe_all() -> None:
            discovery_unsub()
            with lock:
                subs = subscriptions.copy()
            for unsub in subs:
                unsub()

        return unsubscribe_all


@runtime_checkable
class SubscribeAllCapable(Protocol[MsgT_co, TopicT_co]):
    """Protocol for pubsubs that support subscribe_all.

    Both AllPubSub (native) and DiscoveryPubSub (synthesized) satisfy this.
    """

    def subscribe_all(self, callback: Callable[[Any, Any], Any]) -> Callable[[], None]:
        """Subscribe to all messages on all topics."""
        ...
