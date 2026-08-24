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

"""ROS 2 pub/sub over rosless.

ROS 2 is DDS plus a naming convention, and rosless speaks both without needing
rclpy or a sourced workspace. Messages cross this boundary as `rosless.Message`,
whose fields are attributes, so a subscription never builds an rclpy object and
image or point cloud payloads arrive as `bytes` pointing at one copy of the DDS
buffer.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import os
import threading
import time
from typing import TYPE_CHECKING, cast

import rosless

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.protocol.pubsub.impl.rospubsub_conversion import (
    derive_ros_type_name,
    dimos_to_ros,
    ros_to_dimos,
)
from dimos.protocol.pubsub.spec import PubSub
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.protocol import DimosMsg

logger = setup_logger()

# Matches the rclpy implementation this replaced: reliable delivery with enough
# history to absorb a burst without stalling the writer.
DEFAULT_QUEUE_DEPTH = 5000

# How long a reader blocks in DDS before looking at its stop flag again.
_READ_POLL_SECONDS = 0.1

# How often `wait_for_subscriber` re-reads the publisher's matched reader count.
_MATCH_POLL_SECONDS = 0.02


@dataclass(frozen=True)
class ROSQoS:
    """The QoS policies of an rclpy `QoSProfile`, under the same names.

    `reliability` and `durability` decide whether a reader and a writer match at
    all, and DDS refuses the pairing silently, so a mismatch shows up as a topic
    that is simply never delivered. `history` and `depth` are local queueing and
    can never be the reason a topic is quiet.
    """

    reliability: rosless.Reliability = "reliable"
    durability: rosless.Durability = "volatile"
    history: rosless.History = "keep_last"
    depth: int = DEFAULT_QUEUE_DEPTH


@dataclass
class RawROSTopic:
    """Topic descriptor for raw ROS pubsub, carrying a ROS type name."""

    topic: str
    ros_type: str
    qos: ROSQoS | None = None


@dataclass
class ROSTopic:
    """Topic descriptor for DimosROS pubsub (uses dimos message types)."""

    topic: str
    msg_type: type[DimosMsg]
    qos: ROSQoS | None = None


def _default_domain() -> int:
    return int(os.environ.get("ROS_DOMAIN_ID", "0"))


class _TopicReader:
    """Pumps one rosless subscription into one callback on its own thread."""

    def __init__(
        self,
        subscription: rosless.Subscription,
        topic: RawROSTopic,
        callback: Callable[[rosless.Message, RawROSTopic], None],
    ) -> None:
        self.callback = callback
        self._subscription = subscription
        self._topic = topic
        self._stopped = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name=f"ros_reader_{topic.topic}", daemon=True
        )
        self._thread.start()

    def _run(self) -> None:
        with self._subscription:
            while not self._stopped.is_set():
                try:
                    message = self._subscription.read(timeout=_READ_POLL_SECONDS)
                except Exception:
                    logger.exception(f"ROS subscription on {self._topic.topic} failed")
                    return
                if message is None or self._stopped.is_set():
                    continue
                try:
                    # `subscribe` rejects uncatalogued types, so this is never raw CDR.
                    self.callback(cast("rosless.Message", message), self._topic)
                except Exception:
                    logger.exception(f"ROS callback on {self._topic.topic} raised")

    def stop(self) -> None:
        self._stopped.set()

    def join(self) -> None:
        self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)


class RawROS(PubSub[RawROSTopic, rosless.Message]):
    """ROS 2 pubsub over rosless, exchanging messages as `rosless.Message`."""

    def __init__(self, domain: int | None = None, qos: ROSQoS | None = None) -> None:
        """Initialize the ROS pubsub.

        Args:
            domain: DDS domain id (defaults to `ROS_DOMAIN_ID`, as rclpy does)
            qos: Optional QoS, applied to any topic that does not carry its own
        """
        self._domain = _default_domain() if domain is None else domain
        self._qos = qos if qos is not None else ROSQoS()
        self._node: rosless.Node | None = None
        self._publishers: dict[str, rosless.Publisher] = {}
        self._readers: list[_TopicReader] = []
        self._lock = threading.Lock()

    def start(self) -> None:
        """Join the DDS domain. Every topic shares this one membership."""
        with self._lock:
            if self._node is None:
                self._node = rosless.Node(domain=self._domain)

    def stop(self) -> None:
        """Leave the domain, closing every publisher and subscription."""
        with self._lock:
            readers = self._readers
            publishers = list(self._publishers.values())
            self._readers = []
            self._publishers = {}
            self._node = None

        for reader in readers:
            reader.stop()
        for reader in readers:
            reader.join()
        for publisher in publishers:
            publisher.close()

    def _qos_for(self, topic: RawROSTopic | ROSTopic) -> ROSQoS:
        return topic.qos if topic.qos is not None else self._qos

    def _get_or_create_publisher(self, topic: RawROSTopic) -> rosless.Publisher:
        with self._lock:
            existing = self._publishers.get(topic.topic)
            if existing is not None:
                return existing
            if self._node is None:
                raise RuntimeError("Pubsub must be started before publishing")
            qos = self._qos_for(topic)
            publisher = self._node.advertise(
                topic.topic,
                topic.ros_type,
                reliability=qos.reliability,
                durability=qos.durability,
                history=qos.history,
                depth=qos.depth,
            )
            self._publishers[topic.topic] = publisher
            return publisher

    def publish(self, topic: RawROSTopic, message: rosless.Message) -> None:
        """Publish a message to a ROS topic.

        Args:
            topic: RawROSTopic descriptor with topic name and ROS type name
            message: `rosless.Message` of that ROS type
        """
        if self._node is None:
            return
        self._get_or_create_publisher(topic).send(message)

    def wait_for_subscriber(self, topic: RawROSTopic, timeout: float) -> bool:
        """Block until some subscriber has matched this topic's publisher.

        DDS discards anything written before a reader has been discovered, so a
        process that publishes once has to wait for that match first.

        Returns:
            Whether a subscriber appeared before the timeout
        """
        publisher = self._get_or_create_publisher(topic)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if publisher.subscriber_count > 0:
                return True
            time.sleep(_MATCH_POLL_SECONDS)
        return False

    def subscribe(
        self, topic: RawROSTopic, callback: Callable[[rosless.Message, RawROSTopic], None]
    ) -> Callable[[], None]:
        """Subscribe to a ROS topic with a callback.

        Args:
            topic: RawROSTopic descriptor with topic name and ROS type name
            callback: Function called with (message, topic) when a message arrives

        Returns:
            Unsubscribe function
        """
        with self._lock:
            if self._node is None:
                raise RuntimeError("ROS pubsub not started")
            if rosless.lookup(topic.ros_type) is None:
                raise ValueError(f"{topic.ros_type} is not in the rosless type catalogue")
            qos = self._qos_for(topic)
            subscription = self._node.subscribe(
                topic.topic,
                topic.ros_type,
                reliability=qos.reliability,
                durability=qos.durability,
                history=qos.history,
                depth=qos.depth,
            )
            reader = _TopicReader(subscription, topic, callback)
            self._readers.append(reader)

        def unsubscribe() -> None:
            reader.stop()
            with self._lock:
                if reader in self._readers:
                    self._readers.remove(reader)

        return unsubscribe


class DimosROS(PubSub[ROSTopic, "DimosMsg"]):
    """ROS pubsub with automatic dimos.msgs conversion, composing RawROS."""

    def __init__(self, domain: int | None = None, qos: ROSQoS | None = None) -> None:
        """Initialize the DimosROS pubsub.

        Args:
            domain: DDS domain id (defaults to `ROS_DOMAIN_ID`, as rclpy does)
            qos: Optional QoS, applied to any topic that does not carry its own
        """
        self._raw = RawROS(domain, qos)

    def start(self) -> None:
        self._raw.start()

    def stop(self) -> None:
        self._raw.stop()

    def _to_raw_topic(self, topic: ROSTopic) -> RawROSTopic:
        return RawROSTopic(
            topic=topic.topic, ros_type=derive_ros_type_name(topic.msg_type), qos=topic.qos
        )

    def publish(self, topic: ROSTopic, message: DimosMsg) -> None:
        """Publish a dimos message to a ROS topic."""
        raw_topic = self._to_raw_topic(topic)
        self._raw.publish(raw_topic, dimos_to_ros(message, raw_topic.ros_type))

    def wait_for_subscriber(self, topic: ROSTopic, timeout: float) -> bool:
        """Block until some subscriber has matched this topic's publisher."""
        return self._raw.wait_for_subscriber(self._to_raw_topic(topic), timeout)

    def subscribe(
        self, topic: ROSTopic, callback: Callable[[DimosMsg, ROSTopic], None]
    ) -> Callable[[], None]:
        """Subscribe to a ROS topic, converting each message to `topic.msg_type`."""

        def wrapped_callback(ros_msg: rosless.Message, _raw_topic: RawROSTopic) -> None:
            callback(ros_to_dimos(ros_msg, topic.msg_type), topic)

        return self._raw.subscribe(self._to_raw_topic(topic), wrapped_callback)


ROS = DimosROS
