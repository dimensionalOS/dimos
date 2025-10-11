# Copyright 2025 Dimensional Inc.
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

import pickle
import subprocess
import sys
import threading
import traceback
from dataclasses import dataclass
from typing import Any, Callable, Optional, Protocol, runtime_checkable

import lcm

from dimos.protocol.pubsub.spec import PickleEncoderMixin, PubSub, PubSubEncoderMixin
from dimos.protocol.service.lcmservice import LCMConfig, LCMService, autoconf, check_system
from dimos.utils.deprecation import deprecated
from dimos.utils.logging_config import setup_logger


logger = setup_logger(__name__)


@runtime_checkable
class LCMMsg(Protocol):
    msg_name: str

    @classmethod
    def lcm_decode(cls, data: bytes) -> "LCMMsg":
        """Decode bytes into an LCM message instance."""
        ...

    def lcm_encode(self) -> bytes:
        """Encode this message instance into bytes."""
        ...


@dataclass
class Topic:
    topic: str = ""
    lcm_type: Optional[type[LCMMsg]] = None

    def __str__(self) -> str:
        if self.lcm_type is None:
            return self.topic
        return f"{self.topic}#{self.lcm_type.msg_name}"


class LCMPubSubBase(LCMService, PubSub[Topic, Any]):
    default_config = LCMConfig
    _stop_event: threading.Event
    _thread: Optional[threading.Thread]
    _callbacks: dict[str, list[Callable[[Any], None]]]

    def __init__(self, **kwargs) -> None:
        LCMService.__init__(self, **kwargs)
        super().__init__(**kwargs)
        self._callbacks = {}

    def publish(self, topic: Topic, message: bytes):
        """Publish a message to the specified channel."""
        if self.l is None:
            logger.error("Tried to publish after LCM was closed")
            return
        self.l.publish(str(topic), message)

    def subscribe(
        self, topic: Topic, callback: Callable[[bytes, Topic], Any]
    ) -> Callable[[], None]:
        if self.l is None:
            logger.error("Tried to subscribe after LCM was closed")

            def noop():
                pass

            return noop

        lcm_subscription = self.l.subscribe(str(topic), lambda _, msg: callback(msg, topic))

        def unsubscribe():
            if self.l is None:
                return
            self.l.unsubscribe(lcm_subscription)

        return unsubscribe

    @deprecated("Listen for the lastest message directly")
    def wait_for_message(self, topic: Topic, timeout: float = 1.0) -> Any:
        """Wait for a single message on the specified topic.

        Args:
            topic: The topic to listen on
            timeout: Maximum time to wait for a message in seconds

        Returns:
            The received message or None if timeout occurred
        """

        if self.l is None:
            logger.error("Tried to wait for message after LCM was closed")
            return None

        received_message = None
        message_event = threading.Event()

        def message_handler(channel, data):
            nonlocal received_message
            try:
                # Decode the message if type is specified
                if hasattr(self, "decode") and topic.lcm_type is not None:
                    received_message = self.decode(data, topic)
                else:
                    received_message = data
                message_event.set()
            except Exception as e:
                print(f"Error decoding message: {e}")
                message_event.set()

        # Subscribe to the topic
        subscription = self.l.subscribe(str(topic), message_handler)

        try:
            # Wait for message or timeout
            message_event.wait(timeout)
            return received_message
        finally:
            # Clean up subscription
            self.l.unsubscribe(subscription)


class LCMEncoderMixin(PubSubEncoderMixin[Topic, Any]):
    def encode(self, msg: LCMMsg, _: Topic) -> bytes:
        return msg.lcm_encode()

    def decode(self, msg: bytes, topic: Topic) -> LCMMsg:
        if topic.lcm_type is None:
            raise ValueError(
                f"Cannot decode message for topic '{topic.topic}': no lcm_type specified"
            )
        return topic.lcm_type.lcm_decode(msg)


class LCM(
    LCMEncoderMixin,
    LCMPubSubBase,
): ...


class PickleLCM(
    PickleEncoderMixin,
    LCMPubSubBase,
): ...
