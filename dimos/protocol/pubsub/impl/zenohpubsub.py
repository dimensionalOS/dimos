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

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from functools import lru_cache
import threading
from typing import Any, Literal

import zenoh

from dimos.msgs.helpers import resolve_msg_type
from dimos.protocol.pubsub.encoders import LCMEncoderMixin, PickleEncoderMixin
from dimos.protocol.pubsub.impl.lcmpubsub import Topic as LCMTopic
from dimos.protocol.pubsub.spec import AllPubSub
from dimos.protocol.service.zenohservice import ZenohService
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_RELIABILITY = {
    "reliable": zenoh.Reliability.RELIABLE,
    "best_effort": zenoh.Reliability.BEST_EFFORT,
}
_CONGESTION_CONTROL = {
    "drop": zenoh.CongestionControl.DROP,
    "block": zenoh.CongestionControl.BLOCK,
}


@dataclass(frozen=True)
class ZenohQoS:
    """Publisher QoS for one topic. Unset fields use zenoh's defaults (reliable, drop)."""

    reliability: Literal["reliable", "best_effort"] | None = None
    congestion_control: Literal["drop", "block"] | None = None

    def publisher_kwargs(self) -> dict[str, Any]:
        """`declare_publisher` kwargs for this QoS."""
        kwargs: dict[str, Any] = {}
        if self.reliability is not None:
            kwargs["reliability"] = _RELIABILITY[self.reliability]
        if self.congestion_control is not None:
            kwargs["congestion_control"] = _CONGESTION_CONTROL[self.congestion_control]
        return kwargs


# The two delivery classes used across DimOS. Everything else keeps zenoh's
# publisher defaults (reliable, drop under congestion).
QOS_NEVER_DROP = ZenohQoS(reliability="reliable", congestion_control="block")
QOS_LATEST_WINS = ZenohQoS(reliability="best_effort", congestion_control="drop")


@dataclass
class Topic(LCMTopic):
    """LCM-compatible Topic plus Zenoh per-topic settings.

    `qos` applies to the publisher declared for this topic (subscribing ignores
    it). Plain lcmpubsub Topics can be published too; they get default QoS.
    """

    qos: ZenohQoS | None = None


def _topic_to_key_expr(topic: LCMTopic) -> str:
    """Convert a Topic to a Zenoh key expression.

    Embeds the lcm_type in the key using '/' instead of '#' (what LCM does).

    Examples:
        Topic("dimos/cmd_vel", Twist) -> "dimos/cmd_vel/geometry_msgs.Twist"
        Topic("dimos/data")           -> "dimos/data"
    """
    base = topic.topic if isinstance(topic.topic, str) else topic.pattern
    if topic.lcm_type is not None:
        return f"{base}/{topic.lcm_type.msg_name}"
    return base


@lru_cache(maxsize=1024)
def _key_expr_to_topic(key_expr: str, default_lcm_type: type | None = None) -> Topic:
    """Reconstruct a Topic from a Zenoh key expression.

    Parses the last '/' segment and attempts to resolve it as a DimosMsg
    type via resolve_msg_type(). If resolution succeeds, the segment is
    treated as the type suffix and the remainder as the base topic.

    Results are cached; callers must treat the returned Topic as immutable.

    Examples:
        "dimos/cmd_vel/geometry_msgs.Twist" -> Topic("dimos/cmd_vel", Twist)
        "dimos/data"                        -> Topic("dimos/data", default_lcm_type)
        "dimos/data/unknown.Foo"            -> Topic("dimos/data/unknown.Foo", default_lcm_type)
    """
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
        self._drain_stops: list[Callable[[], None]] = []
        self._subscriber_lock = threading.Lock()
        self._stopped = False

    def _get_publisher(self, key_expr: str, qos: ZenohQoS | None) -> zenoh.Publisher:
        """Get or declare the cached publisher for a key expression.

        The publisher is declared with the first publish's QoS; the cache is
        keyed by key expression, so a different QoS on a later publish to the
        same key is ignored.
        """
        with self._publisher_lock:
            if key_expr not in self._publishers:
                kwargs = qos.publisher_kwargs() if qos is not None else {}
                self._publishers[key_expr] = self.session.declare_publisher(key_expr, **kwargs)
            return self._publishers[key_expr]

    def publish(self, topic: Topic, message: bytes) -> None:
        """Publish bytes to a Zenoh key expression.

        Transport-level errors (session closed, invalid key expression) are
        logged but not raised. Delivery guarantees are handled by Zenoh's
        reliability protocol (RELIABLE mode retransmits at each hop).
        """
        key_expr = _topic_to_key_expr(topic)
        qos = topic.qos if isinstance(topic, Topic) else None
        try:
            publisher = self._get_publisher(key_expr, qos)
            publisher.put(message)
        except Exception:
            logger.error(f"Error publishing to {key_expr}", exc_info=True)

    def subscribe(
        self, topic: Topic, callback: Callable[[bytes, Topic], None]
    ) -> Callable[[], None]:
        """Subscribe to a Zenoh key expression."""
        key_expr = _topic_to_key_expr(topic)

        def on_sample(sample: zenoh.Sample) -> None:
            try:
                data = sample.payload.to_bytes()
            except Exception:
                logger.error(f"Error reading payload from {key_expr}", exc_info=True)
                return
            # Concrete subscriptions only ever receive their own key, so the
            # subscribed topic can be passed through without re-parsing.
            sample_key = str(sample.key_expr)
            if sample_key == key_expr:
                recv_topic = topic
            else:
                recv_topic = _key_expr_to_topic(sample_key, topic.lcm_type)
            callback(data, recv_topic)

        sub = self.session.declare_subscriber(key_expr, on_sample)
        with self._subscriber_lock:
            if self._stopped:
                sub.undeclare()
                return lambda: None
            self._subscribers.append(sub)

        def unsubscribe() -> None:
            with self._subscriber_lock:
                if sub not in self._subscribers:
                    return  # Already removed by stop() or a concurrent unsubscribe
                self._subscribers.remove(sub)
            sub.undeclare()

        return unsubscribe

    def subscribe_all(self, callback: Callable[[bytes, Topic], Any]) -> Callable[[], None]:
        """Subscribe to all dimos topics, delivering only the latest per topic.

        Unlike `subscribe`, this is best effort. If it's done otherwise, rerun lags behind.
        """
        latest: dict[str, tuple[bytes, Topic]] = {}
        lock = threading.Lock()
        wake = threading.Event()
        stop = threading.Event()

        def collect(msg: bytes, topic: Topic) -> None:
            # Fast path on the Zenoh delivery thread: keep only the newest per topic.
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
                        logger.error("Error in subscribe_all callback", exc_info=True)

        thread = threading.Thread(target=drain, name="zenoh-subscribe-all", daemon=True)

        def stop_drain() -> None:
            stop.set()
            wake.set()  # unblock the drain so it observes the stop flag
            thread.join(timeout=2.0)

        # Register the stop callback and launch the drain thread under the lock so
        # stop() can't run between them and miss the thread. If stop() already ran,
        # bail without starting anything.
        with self._subscriber_lock:
            if self._stopped:
                return lambda: None
            self._drain_stops.append(stop_drain)
            thread.start()

        inner_unsub = self.subscribe(Topic("dimos/**"), collect)

        def unsubscribe() -> None:
            with self._subscriber_lock:
                if stop_drain not in self._drain_stops:
                    return  # Already removed by stop() or a concurrent unsubscribe
                self._drain_stops.remove(stop_drain)
            inner_unsub()
            stop_drain()

        return unsubscribe

    def stop(self) -> None:
        with self._subscriber_lock:
            self._stopped = True
            drain_stops = list(self._drain_stops)
            self._drain_stops.clear()
        for stop_drain in drain_stops:
            stop_drain()
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
    LCMEncoderMixin,
    ZenohPubSubBase,
):
    """Zenoh pub/sub with LCM encoding for typed DimosMsg."""

    ...


class PickleZenoh(
    PickleEncoderMixin,  # type: ignore[type-arg]
    ZenohPubSubBase,
):
    """Zenoh pub/sub with pickle encoding for arbitrary Python objects."""

    ...
