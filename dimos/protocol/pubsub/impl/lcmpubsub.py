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

from collections.abc import Callable
import re
from typing import Any, ClassVar

from dimos.protocol.pubsub.encoders import LCM as LCM_CODEC, PICKLE, RAW, Codec, DecodingError
from dimos.protocol.pubsub.spec import AllPubSub, accept_all
from dimos.protocol.pubsub.topic import Topic
from dimos.protocol.service.lcmservice import LCMService


class LCMPubSubBase(LCMService, AllPubSub[Topic, Any]):
    codec: ClassVar[Codec] = RAW

    def publish(self, topic: Topic | str, message: Any) -> None:
        channel = str(topic)
        self.handle.publish(channel, self.codec[0](message, topic))

    def subscribe_all(
        self,
        callback: Callable[[Any, Topic], Any],
        accept: Callable[[Topic], bool] = accept_all,
    ) -> Callable[[], None]:
        def filtered(message: Any, topic: Topic) -> None:
            if accept(topic):
                callback(message, topic)

        return self.subscribe(Topic(re.compile(".*")), filtered)

    def subscribe(self, topic: Topic, callback: Callable[[Any, Topic], None]) -> Callable[[], None]:
        def deliver(payload: bytes, received_topic: Topic) -> None:
            try:
                message = self.codec[1](payload, received_topic)
            except DecodingError:
                return
            callback(message, received_topic)

        if topic.is_pattern:

            def handler(channel: str, payload: bytes) -> None:
                if channel != "LCM_SELF_TEST":
                    deliver(payload, Topic.from_channel_str(channel, topic.lcm_type))

            pattern = str(topic)
            if not pattern.endswith("*"):
                pattern += "(#.*)?"
            subscription = self.handle.subscribe(pattern, handler)
        else:
            subscription = self.handle.subscribe(
                str(topic), lambda _, payload: deliver(payload, topic)
            )

        subscription.set_queue_capacity(10000)

        def unsubscribe() -> None:
            self.handle.unsubscribe(subscription)

        return unsubscribe


class LCM(LCMPubSubBase):
    codec = LCM_CODEC


class PickleLCM(LCMPubSubBase):
    codec = PICKLE
