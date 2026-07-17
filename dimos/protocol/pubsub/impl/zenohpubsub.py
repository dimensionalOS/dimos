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

from collections.abc import Callable
from dataclasses import dataclass
from functools import lru_cache
import threading
from typing import Any, ClassVar, Literal

import zenoh

from dimos.msgs.helpers import resolve_msg_type
from dimos.protocol.pubsub.encoders import LCM as LCM_CODEC, PICKLE, RAW, Codec
from dimos.protocol.pubsub.spec import AllPubSub, accept_all
from dimos.protocol.pubsub.topic import Topic
from dimos.protocol.service.zenohservice import ZenohService

_RELIABILITY = {
    "reliable": zenoh.Reliability.RELIABLE,
    "best_effort": zenoh.Reliability.BEST_EFFORT,
}
_CONGESTION = {
    "drop": zenoh.CongestionControl.DROP,
    "block": zenoh.CongestionControl.BLOCK,
}


@dataclass(frozen=True, slots=True)
class ZenohQoS:
    reliability: Literal["reliable", "best_effort"] | None = None
    congestion_control: Literal["drop", "block"] | None = None

    def publisher_kwargs(self) -> dict[str, Any]:
        values = (
            ("reliability", self.reliability, _RELIABILITY),
            ("congestion_control", self.congestion_control, _CONGESTION),
        )
        return {name: mapping[value] for name, value, mapping in values if value is not None}

    def to_wire(self) -> dict[str, str]:
        values = (
            ("reliability", self.reliability),
            ("congestion_control", self.congestion_control),
        )
        return {name: value for name, value in values if value is not None}


QOS_NEVER_DROP = ZenohQoS("reliable", "block")
QOS_LATEST_WINS = ZenohQoS("best_effort", "drop")


def _topic_to_key_expr(topic: Topic) -> str:
    return topic.key_expr


@lru_cache(maxsize=1024)
def _key_expr_to_topic(key_expr: str, default_lcm_type: type | None = None) -> Topic:
    if "/" in key_expr:
        name, type_name = key_expr.rsplit("/", 1)
        lcm_type = resolve_msg_type(type_name)
        if lcm_type is not None:
            return Topic(name, lcm_type)
    return Topic(key_expr, default_lcm_type)


class ZenohPubSubBase(ZenohService, AllPubSub[Topic, Any]):
    codec: ClassVar[Codec] = RAW

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._publishers: dict[str, zenoh.Publisher] = {}
        self._publisher_lock = threading.Lock()
        self._subscribers: list[zenoh.Subscriber[Any]] = []
        self._subscriber_lock = threading.Lock()

    def publish(self, topic: Topic, message: Any) -> None:
        key = topic.key_expr
        with self._publisher_lock:
            publisher = self._publishers.get(key)
            if publisher is None:
                qos = topic.qos
                kwargs = qos.publisher_kwargs() if qos is not None else {}
                publisher = self.session.declare_publisher(key, **kwargs)
                self._publishers[key] = publisher
        publisher.put(self.codec[0](message, topic))

    def subscribe(
        self,
        topic: Topic,
        callback: Callable[[Any, Topic], None],
        accept: Callable[[Topic], bool] = accept_all,
    ) -> Callable[[], None]:
        key = topic.key_expr

        def on_sample(sample: zenoh.Sample) -> None:
            sample_key = str(sample.key_expr)
            received_topic = (
                topic if sample_key == key else _key_expr_to_topic(sample_key, topic.lcm_type)
            )
            if accept(received_topic):
                callback(self.codec[1](sample.payload.to_bytes(), received_topic), received_topic)

        subscriber = self.session.declare_subscriber(key, on_sample)
        with self._subscriber_lock:
            self._subscribers.append(subscriber)

        def unsubscribe() -> None:
            with self._subscriber_lock:
                self._subscribers.remove(subscriber)
            subscriber.undeclare()

        return unsubscribe

    def subscribe_all(
        self,
        callback: Callable[[Any, Topic], Any],
        accept: Callable[[Topic], bool] = accept_all,
    ) -> Callable[[], None]:
        return self.subscribe(Topic("dimos/**"), callback, accept)

    def stop(self) -> None:
        with self._subscriber_lock:
            subscribers, self._subscribers = self._subscribers, []
        for subscriber in subscribers:
            subscriber.undeclare()
        with self._publisher_lock:
            publishers, self._publishers = self._publishers, {}
        for publisher in publishers.values():
            publisher.undeclare()
        super().stop()


class Zenoh(ZenohPubSubBase):
    codec = LCM_CODEC


class PickleZenoh(ZenohPubSubBase):
    codec = PICKLE
