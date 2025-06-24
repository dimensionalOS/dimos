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

import os
import threading
from dataclasses import dataclass
from typing import Any, Callable, Optional, Protocol, runtime_checkable

import lcm

from dimos.protocol.pubsub.spec import PubSub, PubSubEncoderMixin
from dimos.protocol.service.spec import Service


@dataclass
class LCMConfig:
    ttl: int = 0
    url: str | None = None
    # auto configure routing
    auto_configure_multicast: bool = True
    auto_configure_buffers: bool = False


@runtime_checkable
class LCMMsg(Protocol):
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
    lcm_type: Optional[LCMMsg] = None

    def __str__(self) -> str:
        return f"{self.topic}#{self.lcm_type}"


class LCMbase(PubSub[Topic, Any], Service[LCMConfig]):
    default_config = LCMConfig
    lc: lcm.LCM
    _running: bool
    _callbacks: dict[str, list[Callable[[Any], None]]]

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.lc = lcm.LCM(self.config.url)
        self._running = False
        self._callbacks = {}

    def publish(self, topic: Topic, message: Any):
        """Publish a message to the specified channel."""
        self.lc.publish(str(topic), message.encode())

    def subscribe(self, topic: Topic, callback: Callable[[Any], None]):
        """Subscribe to the specified channel with a callback."""
        topic_str = str(topic)

        # Create a wrapper callback that matches LCM's expected signature
        def lcm_callback(channel: str, data: bytes) -> None:
            # Here you would typically decode the data back to the message type
            # For now, we'll pass the raw data - this might need refinement based on usage
            callback(data)

        # Store the original callback for unsubscription
        if topic_str not in self._callbacks:
            self._callbacks[topic_str] = []
        self._callbacks[topic_str].append(callback)

        self.lc.subscribe(topic_str, lcm_callback)

    def unsubscribe(self, topic: Topic, callback: Callable[[Any], None]):
        """Unsubscribe a callback from a topic."""
        topic_str = str(topic)

        # Remove from our tracking
        if topic_str in self._callbacks and callback in self._callbacks[topic_str]:
            self._callbacks[topic_str].remove(callback)
            if not self._callbacks[topic_str]:
                del self._callbacks[topic_str]

        # Note: LCM doesn't provide a direct way to unsubscribe specific callbacks
        # You might need to track and manage callbacks differently for full unsubscribe support

    def start(self):
        if self.config.auto_configure_multicast:
            os.system("sudo ifconfig lo multicast")
            os.system("sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo")

        if self.config.auto_configure_buffers:
            os.system("sudo sysctl -w net.core.rmem_max=2097152")
            os.system("sudo sysctl -w net.core.rmem_default=2097152")

        self._running = True
        self.thread = threading.Thread(target=self._loop)
        self.thread.daemon = True
        self.thread.start()

    def _loop(self) -> None:
        """LCM message handling loop."""
        while self._running:
            try:
                self.lc.handle()
            except Exception as e:
                print(f"Error in LCM handling: {e}")

    def stop(self):
        """Stop the LCM loop."""
        self._running = False
        self.thread.join()


class LCMEncoderMixin(PubSubEncoderMixin[Topic, Any]):
    def encode(msg: LCMMsg, _: Topic) -> bytes:
        return msg.lcm_encode()

    def decode(msg: bytes, topic: Topic) -> LCMMsg:
        if topic.lcm_type is None:
            raise ValueError(
                f"Cannot decode message for topic '{topic.topic}': no lcm_type specified"
            )
        return topic.lcm_type.lcm_decode(msg)


class LCM(LCMbase, PubSubEncoderMixin[Topic, Any]): ...
