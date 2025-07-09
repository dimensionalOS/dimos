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
from dimos.protocol.service.spec import Service


@runtime_checkable
class LCMMsg(Protocol):
    name: str

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
        return f"{self.topic}#{self.lcm_type.name}"


class LCMPubSubBase(PubSub[Topic, Any], LCMService):
    default_config = LCMConfig
    lc: lcm.LCM
    _stop_event: threading.Event
    _thread: Optional[threading.Thread]
    _callbacks: dict[str, list[Callable[[Any], None]]]

    def __init__(self, **kwargs) -> None:
        LCMService.__init__(self, **kwargs)
        super().__init__(**kwargs)
        self._callbacks = {}

    def publish(self, topic: Topic, message: bytes):
        """Publish a message to the specified channel."""
        self.l.publish(str(topic), message)

    def subscribe(
        self, topic: Topic, callback: Callable[[bytes, Topic], Any]
    ) -> Callable[[], None]:
        lcm_subscription = self.l.subscribe(str(topic), lambda _, msg: callback(msg, topic))

        def unsubscribe():
            self.l.unsubscribe(lcm_subscription)

        return unsubscribe


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
