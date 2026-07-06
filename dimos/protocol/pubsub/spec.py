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
import threading
from typing import Any, Generic, Protocol, TypeVar, runtime_checkable

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

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
class SubscribeLatestMixin(Generic[TopicT, MsgT], ABC):
    """Conflated subscribe-all, built on top of subscribe_all().

    Mixed into AllPubSub and DiscoveryPubSub so both expose subscribe_latest().
    """

    @abstractmethod
    def subscribe_all(self, callback: Callable[[MsgT, TopicT], Any]) -> Callable[[], None]: ...

    def _register_drain_stop(
        self, stop_drain: Callable[[], None], thread: threading.Thread
    ) -> bool:
        """Start ``thread`` and record ``stop_drain`` for transport-level shutdown.

        Default: just start the thread. Transports whose stop() must join live
        drains (e.g. zenoh) override this to register under their own lock and
        return False if already stopped (subscribe_latest then bails).
        """
        thread.start()
        return True

    def _unregister_drain_stop(self, stop_drain: Callable[[], None]) -> bool:
        """Forget a drain stopper. Returns False if already removed."""
        return True

    def subscribe_latest(self, callback: Callable[[MsgT, TopicT], Any]) -> Callable[[], None]:
        """Subscribe to all topics, conflated: newest message per topic wins.

        For consumers that only need the current value per topic and must not
        lag behind a fast producer (e.g. the rerun bridge). When the callback
        is slower than the message flow, intermediate messages on a topic are
        dropped and only the latest is delivered.

        Contract:
        - The callback runs on a dedicated drain thread, never on the
          transport's delivery thread.
        - Per topic, the newest pending message is always eventually delivered
          (no starvation); intermediate ones may be skipped.
        - The returned callable unsubscribes the underlying subscribe_all and
          stops+joins the drain thread.
        """
        latest: dict[str, tuple[MsgT, TopicT]] = {}
        lock = threading.Lock()
        wake = threading.Event()
        stop = threading.Event()

        def collect(msg: MsgT, topic: TopicT) -> None:
            # Fast path on the transport's delivery thread: keep only newest per topic.
            with lock:
                latest[str(topic)] = (msg, topic)
            wake.set()

        def drain() -> None:
            while not stop.is_set():
                wake.wait()
                wake.clear()
                with lock:
                    batch = list(latest.values())
                    latest.clear()
                for msg, topic in batch:
                    try:
                        callback(msg, topic)
                    except Exception:
                        logger.error("Error in subscribe_latest callback", exc_info=True)

        thread = threading.Thread(target=drain, name="subscribe-latest-drain", daemon=True)

        def stop_drain() -> None:
            stop.set()
            wake.set()  # unblock the drain so it observes the stop flag
            thread.join(timeout=2.0)

        # Register + start the drain atomically w.r.t. transport shutdown, then
        # subscribe. If the transport already stopped, bail without subscribing.
        if not self._register_drain_stop(stop_drain, thread):
            return lambda: None
        try:
            inner_unsub = self.subscribe_all(collect)
        except BaseException:
            # Don't leak the drain thread if the underlying subscribe fails.
            if self._unregister_drain_stop(stop_drain):
                stop_drain()
            raise

        def unsubscribe() -> None:
            if not self._unregister_drain_stop(stop_drain):
                return  # Already removed by stop() or a concurrent unsubscribe
            inner_unsub()
            stop_drain()

        return unsubscribe


class AllPubSub(SubscribeLatestMixin[TopicT, MsgT], PubSub[TopicT, MsgT], ABC):
    """Mixin for PubSub that supports subscribing to all topics.

    Subclass from this if you support native subscribe-all (e.g. MQTT #, Redis *).
    Provides a default subscribe_new_topics() implementation.
    """

    @abstractmethod
    def subscribe_all(self, callback: Callable[[MsgT, TopicT], Any]) -> Callable[[], None]:
        """Subscribe to all topics.

        Contract: delivers EVERY message on every topic this transport can
        observe — implementations must not conflate, sample, or drop beyond
        the transport's own delivery semantics. Consumers that only want the
        newest message per topic use subscribe_latest() instead.
        """
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
class DiscoveryPubSub(SubscribeLatestMixin[TopicT, MsgT], PubSub[TopicT, MsgT], ABC):
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
        """Subscribe to all messages on all topics (every message, no conflation)."""
        ...

    def subscribe_latest(self, callback: Callable[[Any, Any], Any]) -> Callable[[], None]:
        """Subscribe to all topics, conflated to the newest message per topic."""
        ...
