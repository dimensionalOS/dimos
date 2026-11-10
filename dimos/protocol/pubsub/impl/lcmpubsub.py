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
from dataclasses import dataclass, field
import re
import threading
from typing import Any

from dimos.protocol.pubsub.codec import MessageDescriptor, describe_message
from dimos.protocol.pubsub.encoders import CborEncoderMixin, MessageEncoderMixin
from dimos.protocol.pubsub.patterns import Glob
from dimos.protocol.pubsub.spec import AllPubSub
from dimos.protocol.service.lcmservice import LCMService
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class Topic:
    topic: str | re.Pattern[str] | Glob
    lcm_type: type[Any] | None = None
    message_type: type[Any] | None = field(default=None, kw_only=True)
    message: MessageDescriptor[Any] | None = field(init=False, default=None, repr=False)

    def __post_init__(self) -> None:
        if (
            self.lcm_type is not None
            and self.message_type is not None
            and self.lcm_type is not self.message_type
        ):
            raise TypeError("Conflicting declared message types")
        declared = self.message_type if self.message_type is not None else self.lcm_type
        self.message = describe_message(declared) if declared is not None else None
        self.message_type = declared
        self.lcm_type = declared

    @property
    def is_pattern(self) -> bool:
        return isinstance(self.topic, re.Pattern | Glob)

    @property
    def pattern(self) -> str:
        if isinstance(self.topic, re.Pattern):
            return self.topic.pattern
        if isinstance(self.topic, Glob):
            return self.topic.pattern
        return self.topic

    def __str__(self) -> str:
        if self.message is None or self.message.encoding == "cdr":
            return self.pattern
        assert self.message is not None
        return f"{self.pattern}#{self.message.name}"

    @staticmethod
    def from_channel_str(channel: str, default_lcm_type: type[Any] | None = None) -> Topic:
        """Create Topic from channel string.

        Channel format: /topic#module.ClassName
        Falls back to default_lcm_type if type cannot be parsed.
        """
        from dimos.protocol.pubsub.message_types import resolve_declared_message_type

        if default_lcm_type is not None and describe_message(default_lcm_type).encoding == "cdr":
            return Topic(channel, message_type=default_lcm_type)

        if "#" not in channel:
            return Topic(topic=channel, lcm_type=default_lcm_type)

        topic_str, type_name = channel.rsplit("#", 1)
        lcm_type = resolve_declared_message_type(type_name)
        return Topic(topic=topic_str, lcm_type=lcm_type or default_lcm_type)


class LCMPubSubBase(LCMService, AllPubSub[Topic, Any]):
    """LCM-based PubSub with native regex subscription support.

    LCM natively supports regex patterns in subscribe(), so we implement
    RegexSubscribable directly without needing discovery-based fallback.
    """

    _stop_event: threading.Event
    _thread: threading.Thread | None

    def publish(self, topic: Topic | str, message: bytes) -> None:
        """Publish a message to the specified channel."""
        if self.l is None:
            logger.error("Tried to publish after LCM was closed")
            return

        topic_str = str(topic) if isinstance(topic, Topic) else topic
        self.l.publish(topic_str, message)

    def subscribe_all(self, callback: Callable[[bytes, Topic], Any]) -> Callable[[], None]:
        return self.subscribe(Topic(re.compile(".*")), callback)

    def subscribe(
        self, topic: Topic, callback: Callable[[bytes, Topic], None]
    ) -> Callable[[], None]:
        def noop() -> None:
            pass

        if self.l is None:
            logger.error("Tried to subscribe after LCM was closed")
            return noop

        # The lcm-level detach is deferred to the loop thread (see
        # _defer_unsubscribe), but delivery must stop the moment unsubscribe()
        # returns — gate dispatch on a liveness flag.
        alive = True
        handler: Callable[[str, bytes], None]

        if topic.is_pattern:

            def pattern_handler(channel: str, msg: bytes) -> None:
                if not alive or channel == "LCM_SELF_TEST":
                    return
                callback(msg, Topic.from_channel_str(channel, topic.lcm_type))

            handler = pattern_handler
            channel_str = str(topic)
            if not channel_str.endswith("*"):
                channel_str = f"{channel_str}(#.*)?"
        else:

            def plain_handler(_: str, msg: bytes) -> None:
                if alive:
                    callback(msg, topic)

            handler = plain_handler
            channel_str = str(topic)

        # lcm_subscribe grows the channel's handler array, which the loop
        # thread iterates without a lock while dispatching: a use-after-free
        # that segfaulted CI workers. Exclude the loop for the native call.
        # This waits for the current dispatch to return, so never call
        # subscribe() while holding a lock that a callback may take.
        with self._l_lock:
            if self.l is None:
                return noop
            lcm_subscription = self.l.subscribe(channel_str, handler)
            # Set queue capacity to 10000 to handle high-volume bursts
            lcm_subscription.set_queue_capacity(10000)

        def unsubscribe() -> None:
            nonlocal alive
            if not alive:
                return
            alive = False
            self._defer_unsubscribe(lcm_subscription)

        return unsubscribe


# these ignoress might be unsolvable
# and should use composition not inheritance for encoding/decoding


class LCM(  # type: ignore[misc]
    MessageEncoderMixin,
    LCMPubSubBase,
): ...


class CborLCM(
    CborEncoderMixin,  # type: ignore[type-arg]
    LCMPubSubBase,
): ...
